# grblhal-interlock

A grblHAL plugin providing two independent spindle safety interlocks. Primarily intended for ATC (automatic tool change) machines, where the spindle and the drawbar must never be active at the same time.

The plugin is a **read-only observer**: it watches digital ports and vetoes unsafe operations, but never claims a port itself. Any free or already-claimed port can be monitored.

Ideally, this plugin should **NEVER** trigger, so it will throw an e-stop if it does. You should have a solid ATC workflow with macros that controls the states.

---

## The two interlocks

When either interlock fires it does two things: it suppresses the unsafe operation, and it raises a latching **E-stop** (the same effect as pressing the physical e-stop button). The machine drops into an alarm state and the spindle, coolant, and motion are all killed; clearing it requires a reset and unlock. This is deliberate. A tripped interlock means the ATC workflow already commanded something unsafe, so the plugin fails hard rather than quietly continuing with the parser and hardware out of sync.

### Forward interlock: block spindle start

Monitors any number of digital input or output ports. If **any** monitored port is in its configured *active* state when the spindle is commanded on (`M3`/`M4`), the spindle is held off and an E-stop is raised.

*Typical use:* block spindle start while the drawbar is in tool-change/clamp mode.

The check happens **only at spindle start.** A port going active mid-cut does **not** stop a running spindle (this is V1 scope, no mid-cut feed-hold).

### Inverse interlock: block output writes while spindle runs

Guards specific **output** ports from being written while the spindle is running. If an `M62`/`M63`/`M64`/`M65` targets a guarded port while the spindle is on, the write is suppressed and an E-stop is raised.

*Typical use:* prevent a drawbar/clamp release output from firing while the spindle is spinning.

This guard is ISR-safe and covers both immediate (`M64`/`M65`) and motion-synced (`M62`/`M63`) output writes.

---

## Port numbers

Port values entered in settings are the **macro `Pxxx` aux-port numbers**, exactly the numbers you use in gcode:

- Output ports: the `Pn` in `M62 Pn` / `M63 Pn`
- Input ports: the `Pn` in `M66 Pn`

Input and output `P`-spaces are **independent** (output `P0` and input `P0` are different pins), which is why each forward slot has its own direction setting.

> **Limitation:** ports *claimed* by another plugin are compacted out of the macro `Pxxx` numbering, so a claimed port cannot be monitored by macro number. The ATC use cases (drawbar sensor input plus release output) are unclaimed macro ports, so this is not a problem in practice.

---

## Settings

Settings occupy **`$900`-`$915`** (the ATC-reserved range).

The layout depends on the configured slot counts: **N** forward slots and **M** inverse slots (both default 4). With the defaults:

| Setting | Slot | Description | Default |
|---|---|---|---|
| `$900`-`$903` | FWD 0-3 | Forward monitor **port** (`Pxxx`, `-1` = disabled) | `-1` |
| `$904`-`$907` | FWD 0-3 | Forward port **direction** (`0` = Input, `1` = Output) | Input |
| `$908`-`$911` | FWD 0-3 | Forward **active level** (`0` = active low, `1` = active high) | Active high |
| `$912`-`$915` | INV 0-3 | Inverse **guarded output** port (`Pxxx`, `-1` = disabled) | `-1` |

Port settings display as decimals (e.g. `3.0`); this is normal for grblHAL port settings. `-1` means the slot is disabled.

A slot with its port set to `-1` is ignored. Restore defaults (`$RST=$`) disables all slots and resets forward slots to Input / active-high.

### Example: ATC clamp/release safety

The drawbar is driven by two outputs : **Spindle Clamp** on output `P0`
and **Spindle Release** on output `P1`, both active high. We want a two-way interlock:

- **Forward**: don't let the spindle start while *either* clamp or release is asserted, **or**
  while the drawbar sensor reads out (`1`).
- **Inverse**: don't let *either* output fire while the spindle is running.

That needs three forward slots (two outputs plus the drawbar input) and two inverse slots (guarding
the outputs). The drawbar sensor is an **input**, so the inverse interlock doesn't apply to it; it's
just an extra spindle-start check, and the solenoid that drives the drawbar is already guarded by
the inverse slots above.

```
; --- Forward: block spindle start if clamp or release is active ---
$900=0      ; FWD0 monitors port 0 (Spindle Clamp)
$904=1      ; FWD0 direction = Output
$908=1      ; FWD0 active level = high
$901=1      ; FWD1 monitors port 1 (Spindle Release)
$905=1      ; FWD1 direction = Output
$909=1      ; FWD1 active level = high

; --- Forward: also block if the drawbar sensor reads out ---
$902=3      ; FWD2 monitors input port 3 (Drawbar sensor)
$906=0      ; FWD2 direction = Input
$910=1      ; FWD2 active level = high  (1 = drawbar out)

; --- Inverse: block clamp/release writes while spindle runs ---
$912=0      ; INV0 guards output port 0 (Spindle Clamp)
$913=1      ; INV1 guards output port 1 (Spindle Release)
```

Result: the spindle won't start while clamp or release is asserted or the drawbar sensor reads out,
and neither the clamp nor the release output (`M62`/`M63 P0`/`P1`) will fire while the spindle is
spinning. Any violation raises an E-stop.

---

## When an interlock fires

Both interlocks raise a latching **E-stop**: the machine enters an alarm state and the spindle, coolant, and steppers are killed. You will need to reset and unlock (and re-home if required) before continuing, exactly as with the physical e-stop button.

A descriptive warning is also pushed to the sender's console:

```
[MSG:Spindle start blocked by interlock]
[MSG:Output blocked: spindle running]
```

The output-blocked message is deferred out of the stepper ISR, so it appears shortly after the blocked write rather than exactly at the instant of the block.

---

## Compile-time configuration

Override in `my_machine.h` (or as build flags) before building:

| Macro | Default | Meaning |
|---|---|---|
| `SPINDLE_INTERLOCK_FWD_SLOTS` | `4` | Number of forward (spindle-start) monitor slots (**N**) |
| `SPINDLE_INTERLOCK_INV_SLOTS` | `4` | Number of inverse (guarded output) slots (**M**) |

Changing the slot counts shifts the setting IDs above `$900` (the table is `3N + M` entries).

---

## Integration

### 1. Add the plugin files

Place `spindle_interlock.c` and `spindle_interlock.h` in your driver source tree, or add this repository as a submodule:

```sh
git submodule add https://github.com/gcormier/grblhal-interlock plugins/interlock
```

### 2. Enable in `my_machine.h`

```c
#define SPINDLE_INTERLOCK_ENABLE 1
```

### 3a. Build with CMake

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    plugins/interlock/spindle_interlock.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    plugins/interlock
)
```

### 3b. Build with PlatformIO

```ini
build_flags =
    -D SPINDLE_INTERLOCK_ENABLE=1

lib_deps =
    https://github.com/gcormier/grblhal-interlock
```

---

## Requirements

- grblHAL core with ioports support (`IOPORTS_ENABLE`)
- A single-spindle configuration (the forward veto assumes one spindle)
- Tested on STM32F4xx (SLBEXT); should work on any grblHAL driver with ioports and output readback

---

## License

LGPL-3.0, same as grblHAL core.

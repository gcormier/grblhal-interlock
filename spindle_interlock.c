/*
  spindle_interlock.c — grblHAL general-purpose spindle interlock plugin

  Two independent safety interlocks, primarily for ATC drawbar protection:

  Forward interlock — prevent spindle start (M3/M4):
    Monitor any number of digital input or output ports. If any monitored port is
    in its configured "active" state when the spindle is commanded on, the spindle
    hardware is suppressed and a warning is emitted. The sender still receives `ok`
    (grblHAL has no hard-error path for M3/M4), but the spindle does not start.

  Inverse interlock — prevent output writes while spindle running (M62-M65):
    Guard specific output ports from being written while the spindle is on. If an
    M62/M63/M64/M65 targets a guarded port while the spindle runs, the write is
    suppressed and a warning is emitted.

  Settings $900-$915 (ATC-reserved range; the user-plugin range 450-459 is full —
  pneumaseal 450-454, thermistor 455-459). settings_register() does no ID range
  validation, so the empty ATC block is used.

  Port values entered in settings are macro Pxxx aux-port numbers — the same numbers
  used in M62/M63 (outputs) and M66 (inputs) gcode. Numbering is per-direction.

  License: LGPL-3.0 (same as grblHAL core)
*/

#include "driver.h"

#if SPINDLE_INTERLOCK_ENABLE

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/ioports.h"
#include "grbl/motion_control.h"
#include "grbl/nvs_buffer.h"
#include "grbl/report.h"
#include "grbl/settings.h"
#include "grbl/system.h"
#include "grbl/task.h"

#include "spindle_interlock.h"

// ── compile-time config (overridable in my_machine.h) ──────────────────────────

#ifndef SPINDLE_INTERLOCK_FWD_SLOTS
#define SPINDLE_INTERLOCK_FWD_SLOTS 4   // N — forward (spindle-start) monitor slots
#endif
#ifndef SPINDLE_INTERLOCK_INV_SLOTS
#define SPINDLE_INTERLOCK_INV_SLOTS 4   // M — inverse (guarded output) slots
#endif

#define IL_N SPINDLE_INTERLOCK_FWD_SLOTS
#define IL_M SPINDLE_INTERLOCK_INV_SLOTS

// Setting ID base — ATC-reserved range (900-999), empty in the core enum.
#define IL_SETTING_BASE     900
#define IL_FWD_PORT_BASE    (IL_SETTING_BASE)
#define IL_FWD_DIR_BASE     (IL_SETTING_BASE + IL_N)
#define IL_FWD_LEVEL_BASE   (IL_SETTING_BASE + 2 * IL_N)
#define IL_INV_PORT_BASE    (IL_SETTING_BASE + 3 * IL_N)
#define IL_N_SETTINGS       (3 * IL_N + IL_M)

// ── NVS block ──────────────────────────────────────────────────────────────────

typedef struct {
    uint8_t fwd_port[IL_N];     // IOPORT_UNASSIGNED (255) = disabled
    uint8_t fwd_dir[IL_N];      // 0 = Port_Input, 1 = Port_Output
    uint8_t fwd_level[IL_N];    // 0 = active-low, 1 = active-high
    uint8_t inv_port[IL_M];     // IOPORT_UNASSIGNED (255) = disabled (output P-number)
} interlock_settings_t;

static interlock_settings_t cfg;
static nvs_address_t nvs_addr = 0;

// Runtime-built settings table (variable slot counts) + generated name strings.
static setting_detail_t il_settings[IL_N_SETTINGS];
static char il_names[IL_N_SETTINGS][24];

// Max-value strings for the Format_Decimal port settings. These MUST be persisted in
// static storage: ioports_cfg() writes port_maxs into a stack-local io_port_cfg_t, so a
// setting_detail_t.max_value pointing at it would dangle once il_build_settings returns
// (read_float on the freed stack then fails → error:2 / Status_BadNumberFormat).
static char il_fwd_max[4];
static char il_inv_max[4];

// ── runtime state ──────────────────────────────────────────────────────────────

// spindle_active is the only spindle-state source safe to read from the stepper ISR
// (where M62/M63 synced outputs fire). Updated in on_spindle_programmed (task context).
static volatile bool spindle_active = false;
static volatile bool output_blocked = false;   // set in ISR-context wrapper, reported from realtime
static volatile uint8_t blocked_port = IOPORT_UNASSIGNED;   // guarded port that fired the inverse veto

// ── chained callbacks / wrapped pointers ───────────────────────────────────────

static on_report_options_ptr        prev_on_report_options      = NULL;
static on_spindle_select_ptr        prev_on_spindle_select      = NULL;
static on_spindle_programmed_ptr    prev_on_spindle_programmed  = NULL;
static on_execute_realtime_ptr      prev_on_execute_realtime    = NULL;
static spindle_set_state_ptr        prev_set_state              = NULL;
static digital_out_ptr              prev_digital_out            = NULL;

// ── forward interlock ──────────────────────────────────────────────────────────

// Read each enabled forward slot's live hardware state. Returns the number of slots found
// active and, if buf is non-NULL, appends a human-readable description of every active slot
// (e.g. "FWD0 IN P3 high, FWD2 OUT P5 low") so the operator can see exactly what tripped.
// Unreadable ports (no driver get_value, or read error) are skipped (fail-open) —
// a misconfigured guard must not permanently brick the spindle.
static uint_fast8_t describe_fwd_active (char *buf, size_t buflen)
{
    uint_fast8_t n_active = 0;
    size_t len = 0;

    for (uint_fast8_t i = 0; i < IL_N; i++) {

        uint8_t port = cfg.fwd_port[i];
        if (port == IOPORT_UNASSIGNED)
            continue;

        io_port_direction_t dir = cfg.fwd_dir[i] ? Port_Output : Port_Input;
        xbar_t *xbar = ioport_get_info(Port_Digital, dir, port);   // static ptr — use immediately
        if (xbar == NULL || xbar->get_value == NULL)
            continue;

        float v = xbar->get_value(xbar);
        if (v < 0.0f)                       // read error
            continue;

        bool high = v >= 0.5f;
        if (high == (cfg.fwd_level[i] != 0)) {  // active-high when level==1, active-low when level==0
            if (buf && len < buflen)
                len += snprintf(buf + len, buflen - len, "%sFWD%u %s P%u %s",
                                n_active ? ", " : "", (unsigned)i,
                                cfg.fwd_dir[i] ? "OUT" : "IN", (unsigned)port,
                                high ? "high" : "low");
            n_active++;
        }
    }

    return n_active;
}

// Trigger a latching E-stop when an interlock fires. ISR-safe: mc_reset() is ISR_CODE and
// system_set_exec_alarm() is an atomic flag write, so this is callable from both the task-
// context forward path and the stepper-ISR inverse path.
//
// mc_reset() sets EXEC_RESET, which is what makes the protocol realtime loop actually kill
// the spindle + coolant + steppers (spindle_all_off is gated on EXEC_RESET — the alarm flag
// alone does NOT stop the spindle). With no physical e-stop signal asserted mc_reset() would
// only produce a soft abort, so we set Alarm_EStop afterwards to upgrade it to a latching
// E-stop. Order matters: mc_reset() may set Alarm_AbortCycle internally when in CYCLE, so
// our EStop must be written last to win the (overwriting) atomic alarm store.
static void trigger_interlock_estop (void)
{
    mc_reset();
    system_set_exec_alarm(Alarm_EStop);
}

// Wrapper installed over spindle->set_state. Runs in task context (M3/M4 execution),
// so report_message is safe here. A forward guard active at spindle-on is treated as a
// hard fault: E-stop rather than silently leaving the parser thinking the spindle is
// running while the hardware stays off.
static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    char detail[120];
    if (state.on && describe_fwd_active(detail, sizeof(detail))) {
        char msg[160];
        snprintf(msg, sizeof(msg), "Spindle start blocked by interlock: %s", detail);
        report_message(msg, Message_Warning);
        trigger_interlock_estop();
        return;     // do not chain — spindle stays off
    }

    if (prev_set_state)
        prev_set_state(spindle, state, rpm);
}

// Hooked into grbl.on_spindle_select. The core calls this on a local copy of the
// spindle struct and memcpy's it into the committed HAL afterward, so replacing
// set_state here persists. Single-spindle assumption: one saved prev_set_state.
static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    if (spindle->set_state != onSpindleSetState) {
        prev_set_state = spindle->set_state;
        spindle->set_state = onSpindleSetState;
    }

    return prev_on_spindle_select == NULL || prev_on_spindle_select(spindle);
}

// ── inverse interlock ──────────────────────────────────────────────────────────

static bool is_guarded (uint8_t port)
{
    for (uint_fast8_t i = 0; i < IL_M; i++)
        if (cfg.inv_port[i] != IOPORT_UNASSIGNED && cfg.inv_port[i] == port)
            return true;

    return false;
}

// Wrapper installed over hal.port.digital_out. M62/M63 synced outputs call this from
// the stepper ISR, so it MUST be ISR-safe: no allocation, no blocking, no reporting.
// Suppress the write, queue an E-stop (system_set_exec_alarm is an atomic flag write —
// the only alarm primitive safe from an ISR), and flag the block; the descriptive message
// is reported later from on_execute_realtime (task context).
static void onDigitalOut (uint8_t port, bool on)
{
    if (spindle_active && is_guarded(port)) {
        blocked_port = port;
        output_blocked = true;
        trigger_interlock_estop();
        return;     // suppress write
    }

    if (prev_digital_out)
        prev_digital_out(port, on);
}

// ── event callbacks ────────────────────────────────────────────────────────────

static void on_spindle_programmed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if (prev_on_spindle_programmed)
        prev_on_spindle_programmed(spindle, state, rpm, mode);

    spindle_active = state.on;
}

static void on_execute_realtime (sys_state_t state)
{
    if (prev_on_execute_realtime)
        prev_on_execute_realtime(state);

    if (output_blocked) {
        output_blocked = false;
        char msg[80];
        snprintf(msg, sizeof(msg), "Output P%u blocked by interlock: spindle running",
                 (unsigned)blocked_port);
        report_message(msg, Message_Warning);
    }
}

static void on_report_options (bool newopt)
{
    if (prev_on_report_options)
        prev_on_report_options(newopt);

    if (!newopt)
        report_plugin("Interlock", "0.6");
}

// ── settings set/get ───────────────────────────────────────────────────────────
//
// Port settings use our OWN set/get (not the io_port_cfg_t helper that fans.c uses):
// that helper rejects ports where cap.claimable is false — i.e. ports already claimed
// by another plugin — but this read-only observer must be able to watch any port.
// We validate existence only (never claimable). -1.0 <-> IOPORT_UNASSIGNED (255).

static status_code_t set_fwd_port (setting_id_t id, float value)
{
    uint_fast8_t slot = id - IL_FWD_PORT_BASE;

    if (value < 0.0f) {
        cfg.fwd_port[slot] = IOPORT_UNASSIGNED;
        return Status_OK;
    }

    uint8_t port = (uint8_t)value;
    // A forward slot may be either direction; the direction setting is independent and
    // may not be set yet, so accept the port if it exists in either direction.
    if (ioport_get_info(Port_Digital, Port_Input, port) == NULL &&
        ioport_get_info(Port_Digital, Port_Output, port) == NULL)
        return Status_AuxiliaryPortUnavailable;

    cfg.fwd_port[slot] = port;
    return Status_OK;
}

static float get_fwd_port (setting_id_t id)
{
    uint8_t port = cfg.fwd_port[id - IL_FWD_PORT_BASE];
    return port == IOPORT_UNASSIGNED ? -1.0f : (float)port;
}

static status_code_t set_fwd_dir (setting_id_t id, uint_fast16_t value)
{
    cfg.fwd_dir[id - IL_FWD_DIR_BASE] = value ? 1 : 0;
    return Status_OK;
}

static uint32_t get_fwd_dir (setting_id_t id)
{
    return cfg.fwd_dir[id - IL_FWD_DIR_BASE];
}

static status_code_t set_fwd_level (setting_id_t id, uint_fast16_t value)
{
    cfg.fwd_level[id - IL_FWD_LEVEL_BASE] = value ? 1 : 0;
    return Status_OK;
}

static uint32_t get_fwd_level (setting_id_t id)
{
    return cfg.fwd_level[id - IL_FWD_LEVEL_BASE];
}

static status_code_t set_inv_port (setting_id_t id, float value)
{
    uint_fast8_t slot = id - IL_INV_PORT_BASE;

    if (value < 0.0f) {
        cfg.inv_port[slot] = IOPORT_UNASSIGNED;
        return Status_OK;
    }

    uint8_t port = (uint8_t)value;
    // Guarded ports are matched against M62/M63 targets, which are output P-numbers.
    if (ioport_get_info(Port_Digital, Port_Output, port) == NULL)
        return Status_AuxiliaryPortUnavailable;

    cfg.inv_port[slot] = port;
    return Status_OK;
}

static float get_inv_port (setting_id_t id)
{
    uint8_t port = cfg.inv_port[id - IL_INV_PORT_BASE];
    return port == IOPORT_UNASSIGNED ? -1.0f : (float)port;
}

// ── settings lifecycle ─────────────────────────────────────────────────────────

static void il_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&cfg, sizeof(interlock_settings_t), true);
}

static void il_settings_restore (void)
{
    for (uint_fast8_t i = 0; i < IL_N; i++) {
        cfg.fwd_port[i]  = IOPORT_UNASSIGNED;
        cfg.fwd_dir[i]   = 0;        // Port_Input
        cfg.fwd_level[i] = 1;        // active-high default
    }
    for (uint_fast8_t i = 0; i < IL_M; i++)
        cfg.inv_port[i] = IOPORT_UNASSIGNED;

    il_settings_save();
}

static void il_settings_load (void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&cfg, nvs_addr, sizeof(interlock_settings_t), true) != NVS_TransferResult_OK)
        il_settings_restore();
}

// Copy a port_maxs string into a static max_value buffer, substituting "-1" for an empty
// source so the setting's max_value is always a parseable number (see il_build_settings).
static void il_copy_max (char *dst, const char *src)
{
    strcpy(dst, (src && *src) ? src : "-1");
}

// Build the variable-length settings table at init. Names go in static buffers so the
// const char* pointers stay valid for the lifetime of the firmware.
static void il_build_settings (void)
{
    // Forward port settings can target either direction — advertise the larger range.
    io_port_cfg_t d_in, d_out;
    ioports_cfg(&d_in,  Port_Digital, Port_Input);
    ioports_cfg(&d_out, Port_Digital, Port_Output);
    // Copy port_maxs out of the stack-local cfg structs into static storage so the
    // setting_detail_t.max_value pointers stay valid for the firmware lifetime. When a
    // direction has no ports, ioports_cfg leaves port_maxs as "" — read_float() on an empty
    // max_value fails with error:2, so fall back to "-1" (only the disabled value accepted).
    // Unlike pneumaseal/thermistor this observer registers settings unconditionally, so it
    // must tolerate zero available ports.
    il_copy_max(il_fwd_max, d_in.n_ports >= d_out.n_ports ? d_in.port_maxs : d_out.port_maxs);
    il_copy_max(il_inv_max, d_out.port_maxs);
    const char *fwd_max = il_fwd_max;

    uint_fast8_t k = 0;

    for (uint_fast8_t i = 0; i < IL_N; i++, k++) {
        snprintf(il_names[k], sizeof(il_names[k]), "Interlock FWD%u port", (unsigned)i);
        il_settings[k] = (setting_detail_t){
            .id = IL_FWD_PORT_BASE + i, .group = Group_AuxPorts, .name = il_names[k],
            .datatype = Format_Decimal, .format = "-#0", .min_value = "-1", .max_value = fwd_max,
            .type = Setting_NonCoreFn, .value = set_fwd_port, .get_value = get_fwd_port
        };
    }

    for (uint_fast8_t i = 0; i < IL_N; i++, k++) {
        snprintf(il_names[k], sizeof(il_names[k]), "Interlock FWD%u dir", (unsigned)i);
        il_settings[k] = (setting_detail_t){
            .id = IL_FWD_DIR_BASE + i, .group = Group_AuxPorts, .name = il_names[k],
            .datatype = Format_RadioButtons, .format = "Input,Output",
            .type = Setting_NonCoreFn, .value = set_fwd_dir, .get_value = get_fwd_dir
        };
    }

    for (uint_fast8_t i = 0; i < IL_N; i++, k++) {
        snprintf(il_names[k], sizeof(il_names[k]), "Interlock FWD%u level", (unsigned)i);
        il_settings[k] = (setting_detail_t){
            .id = IL_FWD_LEVEL_BASE + i, .group = Group_AuxPorts, .name = il_names[k],
            .datatype = Format_Bool, .format = "Active low,Active high",
            .type = Setting_NonCoreFn, .value = set_fwd_level, .get_value = get_fwd_level
        };
    }

    for (uint_fast8_t i = 0; i < IL_M; i++, k++) {
        snprintf(il_names[k], sizeof(il_names[k]), "Interlock INV%u port", (unsigned)i);
        il_settings[k] = (setting_detail_t){
            .id = IL_INV_PORT_BASE + i, .group = Group_AuxPorts, .name = il_names[k],
            .datatype = Format_Decimal, .format = "-#0", .min_value = "-1", .max_value = il_inv_max,
            .type = Setting_NonCoreFn, .value = set_inv_port, .get_value = get_inv_port
        };
    }
}

// ── init ───────────────────────────────────────────────────────────────────────

void spindle_interlock_init (void)
{
    static setting_details_t setting_details = {
        .settings   = il_settings,
        .n_settings = IL_N_SETTINGS,
        .save       = il_settings_save,
        .load       = il_settings_load,
        .restore    = il_settings_restore,
    };

    if (!(nvs_addr = nvs_alloc(sizeof(interlock_settings_t)))) {
        task_run_on_startup(report_warning, "Interlock: NVS allocation failed");
        return;
    }

    il_build_settings();
    settings_register(&setting_details);

    // Forward interlock: wrap spindle set_state via on_spindle_select.
    prev_on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = onSpindleSelect;

    // Track spindle on/off for the ISR-safe inverse check.
    prev_on_spindle_programmed = grbl.on_spindle_programmed;
    grbl.on_spindle_programmed = on_spindle_programmed;

    // Deferred reporting for blocked outputs.
    prev_on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = on_execute_realtime;

    // Inverse interlock: wrap the digital output handler (intercepts M62-M65). Safe to
    // do here — plugins_init runs after the core has installed its ioports aggregator.
    prev_digital_out = hal.port.digital_out;
    hal.port.digital_out = onDigitalOut;

    prev_on_report_options = grbl.on_report_options;
    grbl.on_report_options = on_report_options;
}

#endif // SPINDLE_INTERLOCK_ENABLE

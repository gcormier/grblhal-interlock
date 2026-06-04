add_library(grblhal-interlock INTERFACE)
target_sources(grblhal-interlock INTERFACE ${CMAKE_CURRENT_LIST_DIR}/spindle_interlock.c)
target_include_directories(grblhal-interlock INTERFACE ${CMAKE_CURRENT_LIST_DIR})

zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
zephyr_library_sources(layout.c)
if(CONFIG_ZMK_HID_INDICATORS)
  zephyr_library_sources(capslock.c)
endif()
if(CONFIG_DAISY_FACTORY)
  zephyr_library_sources(factory_hid.c)
endif()
# RF test modes (protocol group 0xB) for CE/FCC certification: the group 0xB
# handlers plus the vendored Nordic radio_test driver they wrap.
if(CONFIG_DAISY_FACTORY_RF)
  zephyr_library_sources(factory_rf.c)
  zephyr_library_sources(rf/radio_test.c)
endif()
# Crash breadcrumb (fatal-error handler + __noinit record). Not factory-only:
# the record is also what makes RESET_ON_FATAL_ERROR reboots diagnosable.
zephyr_library_sources(crash_info.c)
if(CONFIG_ZMK_HID_TOUCHPAD)
  zephyr_library_sources(touchpad_power.c)
  zephyr_library_sources(behavior_touchpad_power.c)
endif()
# Touchpad firmware carried in the keyboard image: parse + CRC-check the
# vendor FWHD file at build time and emit the two flash sections as .rodata
# (~7 KB). A malformed file fails the build; a silently wrong blob bricks pads.
if(CONFIG_DAISY_TOUCHPAD_UPDATE)
  set(tp_fw_src ${CMAKE_CURRENT_SOURCE_DIR}/touchpad_firmware/${CONFIG_DAISY_TOUCHPAD_FW_FILE})
  set(tp_fw_gen ${CMAKE_CURRENT_BINARY_DIR}/tp_fw_blob.inc)
  add_custom_command(
    OUTPUT ${tp_fw_gen}
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/touchpad_firmware/gen_tp_fw.py
            ${tp_fw_src} ${tp_fw_gen}
    DEPENDS ${tp_fw_src} ${CMAKE_CURRENT_SOURCE_DIR}/touchpad_firmware/gen_tp_fw.py
    COMMENT "Generating touchpad firmware blob from ${CONFIG_DAISY_TOUCHPAD_FW_FILE}"
    VERBATIM)
  # Same shape as Zephyr's generate_inc_file_for_target(): the blob is an
  # include file, and a target dependency orders its generation before the
  # library that includes it compiles (the library is created in a parent
  # scope, so a generated *source* would not be accepted here).
  add_custom_target(daisy_tp_fw_blob DEPENDS ${tp_fw_gen})
  add_dependencies(${ZEPHYR_CURRENT_LIBRARY} daisy_tp_fw_blob)
  zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})
  zephyr_library_sources(touchpad_update.c)
endif()
#zephyr_library_sources(pmic.c)
#zephyr_library_sources(leds.c)
zephyr_library_sources(behavior_pairing.c)
zephyr_library_sources(charging_led.c)
# nPM1300 ship-mode entry, shared by the factory SHIP_MODE command and the
# low-battery cutoff. Always built: it compiles to stubs on targets with no
# PMIC, and two callers must not run separate state machines against it.
zephyr_library_sources(pmic_ship.c)
if(CONFIG_DAISY_BATTERY_CUTOFF)
  zephyr_library_sources(battery_cutoff.c)
endif()
if(CONFIG_DAISY_CHARGE_TIMEOUT_RECOVERY)
  zephyr_library_sources(charge_timeout_recovery.c)
endif()
zephyr_library_sources(pairing_leds.c)
zephyr_library_sources(protocol_switch.c)
zephyr_library_sources(pwm_init.c)
# DEBUG (sleep/wake investigation) — remove sleep_debug.c when done.
zephyr_library_sources(sleep_debug.c)

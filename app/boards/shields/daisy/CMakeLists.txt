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

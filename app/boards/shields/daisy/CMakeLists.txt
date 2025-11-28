if(CONFIG_BOARD_DAISY)
  zephyr_library_sources(layout.c)
endif()
zephyr_library_sources(button_switch.c)

if(CONFIG_ZMK_HID_INDICATORS)
  zephyr_library_sources(capslock.c)
endif()

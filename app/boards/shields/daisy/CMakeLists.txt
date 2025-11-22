if(CONFIG_BOARD_DAISY)
  zephyr_library_sources(layout.c)
endif()
zephyr_library_sources(button_switch.c)


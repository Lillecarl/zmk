# Copyright (c) 2020-2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_MOCK input_mock.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_GPIO_MATRIX input_gpio_matrix.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_GPIO_DIRECT input_gpio_direct.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_COMPOSITE input_composite.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_DEGHOST input_deghost.c)

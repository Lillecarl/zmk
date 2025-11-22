/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int layout_init(void) {
    const struct device *p0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    gpio_pin_configure(p0, 30, GPIO_INPUT);
    gpio_pin_configure(p0, 31, GPIO_INPUT);

    int iso = gpio_pin_get(p0, 30);
    int ansi = gpio_pin_get(p0, 31);

    if (iso && ansi) {
        LOG_ERR("JIS Layout");
    } else if (iso && !ansi) {
        LOG_ERR("ISO Layout");
    } else if (!iso && ansi) {
        LOG_ERR("ANSI Layout");
    }

    // TODO: Re-assign layout

    return 0;
}

SYS_INIT(layout_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

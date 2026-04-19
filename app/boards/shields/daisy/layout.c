/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, iso_strap_gpios) && \
    DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ansi_strap_gpios)

static const struct gpio_dt_spec iso_strap =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, iso_strap_gpios);
static const struct gpio_dt_spec ansi_strap =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ansi_strap_gpios);

static int layout_init(void) {
    if (!gpio_is_ready_dt(&iso_strap) || !gpio_is_ready_dt(&ansi_strap)) {
        LOG_ERR("Layout strap GPIOs not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&iso_strap, GPIO_INPUT);
    gpio_pin_configure_dt(&ansi_strap, GPIO_INPUT);

    int iso = gpio_pin_get_dt(&iso_strap);
    int ansi = gpio_pin_get_dt(&ansi_strap);

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

#else

static int layout_init(void) {
    LOG_WRN("Layout straps not configured");
    return 0;
}

#endif

SYS_INIT(layout_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

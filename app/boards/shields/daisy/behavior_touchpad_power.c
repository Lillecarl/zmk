/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_touchpad_power

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/behavior.h>

#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>

#include "touchpad_power.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Keymap-bindable front end for the user touchpad enable/disable: param1 is
 * 1 to enable, 0 to disable. Bound to the E+T / D+T combos in daisy.keymap,
 * which fire while Win is held -- Win can't be a combo member itself (a
 * combo's window starts at its first key, and Win is naturally held long
 * before the letters), so the GUI-modifier check lives here instead.
 * Compiled under the same conditions as touchpad_power.c's pad-control
 * section so the setter it calls always exists; without it the behavior
 * device is simply absent and pressing the combo logs an error. */
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) &&                                                    \
    DT_NODE_HAS_STATUS(DT_NODELABEL(touchpad), okay) && IS_ENABLED(CONFIG_PM_DEVICE)

static void tap_usage(uint16_t usage, int64_t timestamp) {
    raise_zmk_keycode_state_changed_from_encoded(ZMK_HID_USAGE(HID_USAGE_KEY, usage), true,
                                                 timestamp);
    raise_zmk_keycode_state_changed_from_encoded(ZMK_HID_USAGE(HID_USAGE_KEY, usage), false,
                                                 timestamp);
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const bool enable = binding->param1 != 0;
    const uint16_t letter = enable ? HID_USAGE_KEY_KEYBOARD_E : HID_USAGE_KEY_KEYBOARD_D;

    if (!(zmk_hid_get_explicit_mods() & (MOD_LGUI | MOD_RGUI))) {
        /* The chord landed during normal typing (Win not held): put the two
         * swallowed letters back. Press order is lost -- they always come
         * out E/D then T -- but require-prior-idle-ms makes this path rare. */
        tap_usage(letter, event.timestamp);
        tap_usage(HID_USAGE_KEY_KEYBOARD_T, event.timestamp);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    /* The host has already seen the bare Win press; tap F24 so the eventual
     * Win release isn't a clean tap that opens the Start menu (Windows) or
     * the Activities overview (GNOME). */
    tap_usage(HID_USAGE_KEY_KEYBOARD_F24, event.timestamp);

    LOG_INF("touchpad user-%s via combo", enable ? "enabled" : "disabled");
    daisy_touchpad_user_set_enabled(enable);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_touchpad_power_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_touchpad_power_driver_api);

#endif /* compat okay && touchpad okay && CONFIG_PM_DEVICE */

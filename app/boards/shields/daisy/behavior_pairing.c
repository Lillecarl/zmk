/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_pairing

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/ble.h>

#include "protocol_switch.h"

#if IS_ENABLED(CONFIG_DAISY_FACTORY)
#include "factory_state.h"
#else
/* No factory interface built -> factory mode can never be active. */
static inline bool daisy_factory_mode_active(void) { return false; }
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Keymap-bindable front end for the physical pairing button (the sole key on
 * the `buttons_input` direct-GPIO device), wrapped in the `pairing_ht`
 * hold-tap: tap = next BLE profile, hold 1 s = clear the active profile's bond.
 *
 * Unlike `&bt BT_NXT` / `&bt BT_CLR` it is inert while the protocol switch
 * sits in the WIRED position. Without that gate the button still switched the
 * active profile behind the user's back -- a settings write -- and, held,
 * silently wiped the bond of a keyboard that is deliberately cabled-only.
 *
 * The gate reads the switch position (protocol_switch.h) rather than
 * `zmk_ble_adv_enabled_get()`: the switch is what the user physically sees, so
 * the button's liveness tracks the thing they just flipped, and it stays live
 * when advertising is off for some unrelated reason (`aster --bt-adv-off`, a
 * host-initiated stop) -- pressing it is then a reasonable way to ask for
 * pairing back. Factory pairing tests are covered too: apply_policy() is
 * skipped in factory mode, so the button stays live there regardless of the
 * switch.
 *
 * Note this no longer matches pairing_leds.c exactly, which still blanks on
 * `zmk_ble_adv_enabled_get()`: with the switch wireless but advertising
 * suppressed the LEDs are dark while the button works.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* param1 values (see daisy.keymap). */
#define PAIRING_NEXT_PROFILE 0
#define PAIRING_CLEAR_BOND 1

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (!daisy_protocol_switch_is_wireless() && !daisy_factory_mode_active()) {
        LOG_INF("pairing button ignored: protocol switch is in the wired position");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    switch (binding->param1) {
    case PAIRING_CLEAR_BOND:
        LOG_INF("pairing button held: clearing the active profile's bond");
        zmk_ble_clear_bonds();
        break;
    case PAIRING_NEXT_PROFILE:
        LOG_INF("pairing button tapped: selecting the next profile");
        zmk_ble_prof_next();
        break;
    default:
        LOG_WRN("unknown pairing action %d", binding->param1);
        break;
    }
#else
    LOG_WRN("pairing button pressed but BLE is not built");
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pairing_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_pairing_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

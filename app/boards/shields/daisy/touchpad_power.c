/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Touchpad power control + automatic host-follow + user enable/disable.
 *
 * Owns daisy_touchpad_power_set() (see touchpad_power.h) and, on top of it,
 * mirrors the host's power state onto the pad the way a laptop's i2c-hid
 * driver would: when the selected endpoint truly can't reach the host (USB
 * disconnected, BLE profile disconnected, or no endpoint at all) the pad
 * drops to modern standby with the passthrough quiesced; when the host comes
 * back it returns to full operation. Suspended in factory mode so the factory
 * tool keeps exclusive control of the pad during a test run.
 *
 * Exception — USB suspend (host asleep but still connected): the pad stays
 * powered (Run/Idle) with its DR interrupt armed, so a touch still produces a
 * real input report. The passthrough sees that report is arriving while the
 * bus is suspended and issues a USB remote wakeup (zmk_usb_wakeup_request),
 * resuming the host — a laptop touchpad's wake-on-touch. Dropping to standby
 * here (as we do for a real disconnect) would disable the DR interrupt and
 * kill wake-on-touch. Idle mode already keeps power low when untouched.
 *
 * The user can also turn the pad off/on from the keyboard: holding Win and
 * chording D+T disables it, Win and E+T re-enables it (combos in
 * daisy.keymap -> behavior_touchpad_power.c). The override is ANDed with
 * host reachability and lives until reboot.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#endif

#include <hid_touchpad.h>

#include "touchpad_power.h"

#if IS_ENABLED(CONFIG_DAISY_FACTORY)
#include "factory_state.h"
#include "factory_mode_changed.h"
#else
/* No factory interface built -> factory mode can never be active. */
static inline bool daisy_factory_mode_active(void) { return false; }
#endif

#if IS_ENABLED(CONFIG_DAISY_TOUCHPAD_UPDATE)
#include "touchpad_update_state_changed.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daisy_tp_power, CONFIG_ZMK_LOG_LEVEL);

/* Everything below needs the pad present and the driver's PM hook; keep the
 * file buildable (as an empty translation unit) on targets without either
 * (dev kits). factory_hid.c's HAS_TOUCHPAD uses the same condition, so the
 * factory handler never references daisy_touchpad_power_set when it is
 * compiled out here. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(touchpad), okay) && IS_ENABLED(CONFIG_PM_DEVICE)

static const struct device *const touchpad = DEVICE_DT_GET(DT_NODELABEL(touchpad));

int daisy_touchpad_power_set(enum daisy_touchpad_power state) {
    if (!device_is_ready(touchpad)) {
        return -ENODEV;
    }

    int err;
    switch (state) {
    case DAISY_TOUCHPAD_POWER_ON:
        err = hid_touchpad_set_power(touchpad, I2C_HID_PWR_ON);
        if (err == 0) {
            /* Re-arm (and drain) the data-ready interrupt after an OFF. */
            err = pm_device_action_run(touchpad, PM_DEVICE_ACTION_RESUME);
            if (err == -EALREADY) {
                err = 0;
            }
        }
        break;
    case DAISY_TOUCHPAD_POWER_SLEEP:
        err = hid_touchpad_set_power(touchpad, I2C_HID_PWR_SLEEP);
        break;
    case DAISY_TOUCHPAD_POWER_OFF:
        /* Quiesce the driver first so a wake-on-touch report can't race the
         * standby entry. */
        err = pm_device_action_run(touchpad, PM_DEVICE_ACTION_SUSPEND);
        if (err == -EALREADY) {
            err = 0;
        }
        if (err == 0) {
            err = hid_touchpad_set_power(touchpad, I2C_HID_PWR_SLEEP);
        }
        break;
    default:
        return -EINVAL;
    }
    if (err) {
        LOG_ERR("touchpad power %d failed: %d", state, err);
    }
    return err;
}

/* --- automatic host-follow ---------------------------------------------- */

/* User override, flipped by the Win,E,T / Win,D,T key sequences below. */
static bool tp_user_enabled = true;

/* True when the pad should stay powered (Run/Idle) for the selected endpoint.
 *
 * For USB this deliberately includes the suspended (host-asleep) state: the
 * pad must remain wake-capable so a touch can trigger USB remote wakeup (see
 * the file header). We only require the HID interface to be configured, which
 * stays true across suspend/resume (conn_state holds at ZMK_USB_CONN_HID); it
 * drops to POWERED/NONE on deconfigure/disconnect, which correctly powers the
 * pad down. */
static bool host_reachable(void) {
    struct zmk_endpoint_instance selected = zmk_endpoint_get_selected();

    switch (selected.transport) {
    case ZMK_TRANSPORT_USB:
        return zmk_usb_is_hid_ready();
#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_TRANSPORT_BLE:
        return zmk_ble_active_profile_is_connected();
#endif
    default:
        return false;
    }
}

/* The pad boots in full operation (the driver sends SET_POWER on at init),
 * so that's the baseline state. Reset to it after a firmware update too: the
 * update ends with a reset into application mode, i.e. a fresh boot. */
static bool pad_on = true;

/* While the firmware updater holds the pad every driver call returns -EBUSY;
 * stand down until it announces it is done. */
static bool tp_update_running;

static void tp_follow_host_work(struct k_work *work) {
    if (daisy_factory_mode_active() || tp_update_running) {
        return;
    }

    const bool host_ok = host_reachable();
    const bool want_on = tp_user_enabled && host_ok;
    if (want_on == pad_on) {
        return;
    }
    if (daisy_touchpad_power_set(want_on ? DAISY_TOUCHPAD_POWER_ON
                                         : DAISY_TOUCHPAD_POWER_OFF) == 0) {
        pad_on = want_on;
        LOG_INF("touchpad %s (host %s, user %s)", want_on ? "on" : "standby",
                host_ok ? "reachable" : "away", tp_user_enabled ? "enabled" : "disabled");
    }
}

K_WORK_DEFINE(tp_follow_work, tp_follow_host_work);

void daisy_touchpad_user_set_enabled(bool enabled) {
    tp_user_enabled = enabled;
    k_work_submit(&tp_follow_work);
}

/* Events fire from several threads; the I2C + PM work happens on the system
 * workqueue where a short blocking write is fine (the factory handler and
 * key-inject already run there). */
static int tp_power_event_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_DAISY_TOUCHPAD_UPDATE)
    const struct daisy_touchpad_update_state_changed *upd =
        as_daisy_touchpad_update_state_changed(eh);
    if (upd) {
        tp_update_running = upd->running;
        if (!upd->running) {
            pad_on = true;
        }
    }
#endif
    k_work_submit(&tp_follow_work);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(daisy_tp_power, tp_power_event_listener);
/* USB suspend/resume flips zmk_usb_is_suspended() and re-raises this event
 * even though the conn state value is unchanged. */
ZMK_SUBSCRIPTION(daisy_tp_power, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(daisy_tp_power, zmk_endpoint_changed);
#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(daisy_tp_power, zmk_ble_active_profile_changed);
#endif
#if IS_ENABLED(CONFIG_DAISY_FACTORY)
/* tp_follow_host_work stands down while factory mode is active; nothing else
 * re-runs it on exit, so the pad could stay stale until the next endpoint
 * event. */
ZMK_SUBSCRIPTION(daisy_tp_power, daisy_factory_mode_changed);
#endif
#if IS_ENABLED(CONFIG_DAISY_TOUCHPAD_UPDATE)
ZMK_SUBSCRIPTION(daisy_tp_power, daisy_touchpad_update_state_changed);
#endif

/* pad_on's baseline is "on" and only events correct it, so on a hostless
 * boot (battery, no bond connected) the pad stayed in Run/Idle until the
 * first endpoint/BLE event. Run one follow pass after boot. Delayed past USB
 * enumeration so a USB-powered boot doesn't bounce the pad off then on. */
#define TP_BOOT_FOLLOW_DELAY_MS 2000

static void tp_boot_follow(struct k_work *work) { k_work_submit(&tp_follow_work); }
static K_WORK_DELAYABLE_DEFINE(tp_boot_follow_work, tp_boot_follow);

static int tp_power_init(void) {
    k_work_schedule(&tp_boot_follow_work, K_MSEC(TP_BOOT_FOLLOW_DELAY_MS));
    return 0;
}

SYS_INIT(tp_power_init, APPLICATION, 99);

#endif /* touchpad okay && CONFIG_PM_DEVICE */

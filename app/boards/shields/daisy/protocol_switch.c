#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

#include "protocol_switch.h"

#if IS_ENABLED(CONFIG_DAISY_FACTORY)
#include "factory_state.h"
#else
/* No factory interface built -> factory mode can never be active. */
static inline bool daisy_factory_mode_active(void) { return false; }
#endif

LOG_MODULE_REGISTER(daisy_protocol_switch, LOG_LEVEL_INF);

/*
 * Physical USB/BT mode switch (the `protocol_switch` gpio-keys node).
 *
 *   WIRED    -> USB only. BLE advertising is stopped and any live link is
 *               dropped. When the cable is unplugged there is nothing left to
 *               do, so the SoC enters System OFF to save power; flipping the
 *               switch to wireless wakes it (reset -> reboot into wireless).
 *
 *   WIRELESS -> BLE is enabled and made the preferred transport. ZMK's own
 *               transport fallback (endpoints.c) then does exactly what we
 *               want for free: output goes over USB while the active profile
 *               is NOT connected, and moves to BLE the instant it connects
 *               (USB stays enumerated but stops carrying reports). See
 *               get_selected_transport().
 *
 * Switch polarity (confirmed on daisy_kb_evt 2026-07-19 via
 * `aster --protocol-switch-state`, flipping both ways): the pin is
 * GPIO_ACTIVE_LOW and logical-asserted (gpio_pin_get_dt() == 1, gpio-keys
 * value == 1) is the WIRELESS position; de-asserted (0) is WIRED.
 *
 * The touchpad honours the same endpoint selection independently (its USB and
 * BLE passthroughs gate on zmk_endpoint_get_selected()), so pointer output
 * follows the keyboard and never mirrors to both buses at once.
 */

#define PROTOCOL_SWITCH_NODE DT_NODELABEL(protocol_switch)

#if DT_NODE_EXISTS(PROTOCOL_SWITCH_NODE)

static const struct gpio_dt_spec protocol_switch =
    GPIO_DT_SPEC_GET(PROTOCOL_SWITCH_NODE, gpios);

/* Grace period after we first see "wired + unplugged" before powering off, so a
 * switch bounce or a brief unplug/replug (and USB enumeration on a cabled boot)
 * doesn't trip a spurious power-down. */
#define POWER_OFF_DELAY K_SECONDS(5)

/*
 * Dedicated cooperative work queue. The power evaluation runs on USB unplug,
 * and the system workqueue has historically frozen for the whole
 * USB-unplugged window (a blocking USB work item — see factory_hid.c and
 * daisy.md). Keep this work off the sysworkq. The BLE adv/disconnect calls and
 * sys_poweroff() are safe on a coop thread.
 */
static K_THREAD_STACK_DEFINE(pq_stack, 1024);
static struct k_work_q pq;

enum mode { MODE_WIRED, MODE_WIRELESS };
static enum mode current_mode = MODE_WIRED; /* replaced by the real reading at init */
static bool mode_known;                     /* false while current_mode is still the default */

static bool read_switch_wireless(void) {
    int lvl = gpio_pin_get_dt(&protocol_switch);
    if (lvl < 0) {
        LOG_WRN("switch read failed (%d); assuming wired", lvl);
        return false;
    }
    return lvl != 0; /* logical-asserted == wireless (see header) */
}

bool daisy_protocol_switch_is_wireless(void) {
    if (!mode_known) {
        /* Asked before our SYS_INIT. Other APPLICATION-level inits (notably
         * pairing_leds.c) share our priority, so link order decides who runs
         * first, and answering with the MODE_WIRED default would be a guess.
         * Read the pin instead -- gpio-keys configured it as an input back at
         * POST_KERNEL, so this is valid from any APPLICATION init onwards. */
        current_mode = read_switch_wireless() ? MODE_WIRELESS : MODE_WIRED;
        mode_known = true;
    }
    return current_mode == MODE_WIRELESS;
}

static void enter_low_power(void) {
    LOG_INF("wired + unplugged: entering System OFF (flip to wireless to wake)");

    /* Arm wake on the switch reaching the wireless (active) position. On nRF a
     * level interrupt configures GPIO SENSE, which drives the System-OFF DETECT
     * wake; the SoC resets on wake and reboots — reading the switch as wireless
     * on the way back up. Bonds/settings survive (flash-backed). */
    int err = gpio_pin_interrupt_configure_dt(&protocol_switch, GPIO_INT_LEVEL_ACTIVE);
    if (err) {
        LOG_ERR("failed to arm switch wake (%d); powering off anyway", err);
    }

    /* TODO(HW-verify): also wake on USB VBUS replug. On nRF54L this is not a
     * GPIO sense — it needs the USB regulator's VBUS-detect wake, which has to
     * be validated on hardware. Until then a replug may need a manual reset. */

    sys_poweroff(); /* does not return */
}

static void apply_policy(void) {
    if (daisy_factory_mode_active()) {
        /* Factory mode does pairing and gpio testing -- don't react on switch */
        return;
    }

    if (current_mode == MODE_WIRELESS) {
        LOG_INF("mode WIRELESS: BLE enabled, USB only until BLE connects");
#if IS_ENABLED(CONFIG_ZMK_BLE)
        zmk_ble_adv_enabled_set(true);
#endif
        zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_BLE);
    } else {
        LOG_INF("mode WIRED: USB only, BLE off");
        zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_USB);
#if IS_ENABLED(CONFIG_ZMK_BLE)
        /* Stops advertising and drops every profile's link (bonds retained). */
        zmk_ble_adv_enabled_set(false);
#endif
    }

#if IS_ENABLED(CONFIG_ZMK_BLE)
    /* Nudge the BLE-state indicators to re-evaluate. Toggling advertising while
     * no profile is connected raises no ble_active_profile_changed on its own,
     * so pairing_leds.c would otherwise keep its stale blink (going to wired) or
     * miss the resumed blink (going to wireless). We reuse that event even
     * though the active profile itself is unchanged: every subscriber re-reads
     * state via the zmk_ble_* getters and ignores the payload, hence the NULL
     * profile pointer. */
    raise_zmk_ble_active_profile_changed((struct zmk_ble_active_profile_changed){
        .index = zmk_ble_active_profile_index(), .profile = NULL});
#endif
}

static void power_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    /* Re-check after the grace period — the cable may have come back or the
     * switch may have moved to wireless in the meantime. */
    if (current_mode == MODE_WIRED && !zmk_usb_is_powered()) {
        enter_low_power();
    }
}
static K_WORK_DELAYABLE_DEFINE(power_work, power_work_handler);

static void eval_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    apply_policy();

    if (current_mode == MODE_WIRED && !zmk_usb_is_powered()) {
        k_work_reschedule_for_queue(&pq, &power_work, POWER_OFF_DELAY);
    } else {
        k_work_cancel_delayable(&power_work);
    }
}
static K_WORK_DELAYABLE_DEFINE(eval_work, eval_work_handler);

static void schedule_eval(void) { k_work_reschedule_for_queue(&pq, &eval_work, K_NO_WAIT); }

/*
 * The boot evaluation waits for ZMK's BLE startup instead of running straight
 * out of SYS_INIT.
 *
 * ZMK brings BLE up from the "ble" settings handler's h_commit
 * (zmk_ble_complete_startup -> zmk_ble_ready -> update_advertising), which runs
 * at the end of settings_load() -- after our SYS_INIT. Firing apply_policy()
 * before that gets `Advertising failed to start (err -11)`: bt_le_adv_start()
 * bails with -EAGAIN before BT_DEV_READY, and the failure is swallowed.
 *
 * (While reconnect advertising was directed, landing after BLE startup also
 * gave a bonded profile's resolving-list entry a second chance to reach the
 * controller on a battery-only boot -- see
 * ../../../../aster/issues/reset-reconnect-failure.md. Stealth undirected
 * advertising no longer depends on the resolving list, so the -EAGAIN
 * swallowing above is the one remaining reason for this delay.)
 */
#define BOOT_EVAL_DELAY K_MSEC(500)

/* gpio-keys turns the switch into INPUT_KEY_1 key events (value 1 = asserted =
 * wireless). Listen on all input devices and filter by code — the board's
 * buttons node has no label to bind to directly, and this matches activity.c. */
static void switch_input_cb(struct input_event *evt, void *user_data) {
    ARG_UNUSED(user_data);
    if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_1) {
        return;
    }
    enum mode new_mode = evt->value ? MODE_WIRELESS : MODE_WIRED;
    if (mode_known && new_mode == current_mode) {
        return;
    }
    current_mode = new_mode;
    mode_known = true;
    LOG_INF("protocol switch -> %s", current_mode == MODE_WIRELESS ? "wireless" : "wired");
    schedule_eval();
}
INPUT_CALLBACK_DEFINE(NULL, switch_input_cb, NULL);

static int protocol_switch_usb_listener(const zmk_event_t *eh) {
    if (as_zmk_usb_conn_state_changed(eh)) {
        /* Power state changed — re-evaluate the wired-unplug power-off. */
        schedule_eval();
    }
    return 0;
}
ZMK_LISTENER(daisy_protocol_switch, protocol_switch_usb_listener);
ZMK_SUBSCRIPTION(daisy_protocol_switch, zmk_usb_conn_state_changed);

static int protocol_switch_init(void) {
    if (!gpio_is_ready_dt(&protocol_switch)) {
        LOG_ERR("protocol switch gpio not ready");
        return -ENODEV;
    }

    k_work_queue_start(&pq, pq_stack, K_THREAD_STACK_SIZEOF(pq_stack), K_PRIO_COOP(7), NULL);
    k_thread_name_set(&pq.thread, "daisy_protocol");

    current_mode = read_switch_wireless() ? MODE_WIRELESS : MODE_WIRED;
    mode_known = true;
    LOG_INF("boot protocol switch: %s", current_mode == MODE_WIRELESS ? "wireless" : "wired");

#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (current_mode == MODE_WIRED) {
        /* Clear permit_adv here rather than waiting for BOOT_EVAL_DELAY. Only
         * the *enable* path needs BT_DEV_READY (see BOOT_EVAL_DELAY); this one
         * just drops a flag that zmk_ble_complete_startup() reads later, from
         * settings_load() in main() -- i.e. after every SYS_INIT. So a wired
         * boot never advertises at all, instead of advertising for the first
         * half second.
         *
         * That half second was visible: every wake from System OFF is a reset
         * + reboot (a keypress wakes via the matrix' GPIO SENSE), and on a
         * wired+unplugged wake the keyboard advertised and blinked the pairing
         * LED until the deferred eval shut BLE down again. pairing_leds.c
         * inits after us (see its SYS_INIT priority) so it now comes up dark. */
        zmk_ble_adv_enabled_set(false);
    }
#endif

    /* Defer the first apply to the coop queue so it runs after BLE/endpoint
     * init has settled, and so a wired+unplugged boot powers off cleanly.
     * See BOOT_EVAL_DELAY for why the boot one waits rather than running now. */
    k_work_reschedule_for_queue(&pq, &eval_work, BOOT_EVAL_DELAY);
    return 0;
}
SYS_INIT(protocol_switch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_NODE_EXISTS(protocol_switch) */

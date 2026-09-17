#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
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
 * Physical USB/BT mode switch (the `protocol_switch` node's `gpios`).
 *
 *   WIRED    -> USB only. BLE advertising is stopped and any live link is
 *               dropped. When the cable is unplugged there is nothing left to
 *               do, so the SoC enters System OFF to save power; flipping the
 *               switch to wireless or plugging the cable back in wakes it
 *               (reset -> reboot, reading the new state on the way up).
 *
 *   WIRELESS -> BLE is enabled and made the preferred transport. ZMK's own
 *               transport fallback (endpoints.c) then does exactly what we
 *               want for free: output goes over USB while the active profile
 *               is NOT connected, and moves to BLE the instant it connects
 *               (USB stays enumerated but stops carrying reports). See
 *               get_selected_transport().
 *
 * The switch is read directly as a GPIO. The board files still declare it as
 * a gpio-keys child of the same label, but the shield overlays set that child
 * to status = "disabled"; we only borrow its `gpios` spec (as factory_hid.c
 * does). A bistable switch is state, not an event stream, so there is no
 * input-event bookkeeping: any edge schedules a settle delay, after which
 * eval_work reads the pin and applies whatever position it finds.
 *
 * Interrupt idiom (the same one input_gpio_matrix uses): the pin is armed
 * with a *level* trigger for the position the switch is NOT in, disarmed on
 * the first fire, and re-armed by eval_work after the read. On nRF a level
 * trigger is GPIO SENSE, so the very configuration that catches a flip while
 * running is also the System-OFF wake source in wired+unplugged: sitting in
 * wired == armed for "wireless", nothing to reconfigure before sys_poweroff().
 * Level rather than edge also means a flip that lands between the read and
 * the re-arm fires immediately instead of being lost, and no GPIOTE channel
 * is consumed.
 *
 * Electrically the switch is SPDT: 100 k to 3.3 V in one position, 100 k to
 * GND in the other, open during travel. Never add an internal pull in DT --
 * the nRF's ~13 k pull would overpower the 100 k and pin the level. The
 * settle delay covers the open interval.
 *
 * Switch polarity (confirmed on daisy_kb_evt 2026-07-19 via
 * `aster --protocol-switch-state`, flipping both ways): logical-asserted
 * (gpio_pin_get_dt() == 1) is the WIRELESS position; de-asserted (0) is
 * WIRED. The board files carry the physical polarity (ACTIVE_LOW on EVT,
 * ACTIVE_HIGH on DVT1).
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

/* Settle time after an edge before the pin is trusted. The SPDT contacts are
 * open mid-travel (pin floating behind 100 k), so one flip can produce several
 * edges over tens of ms; each one just pushes the read out again. */
#define SWITCH_SETTLE K_MSEC(50)

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
/* The position the policy was last applied for (set from the pin at init). */
static enum mode current_mode = MODE_WIRED;

static bool read_switch_wireless(void) {
    int lvl = gpio_pin_get_dt(&protocol_switch);
    if (lvl < 0) {
        LOG_WRN("switch read failed (%d); assuming wired", lvl);
        return false;
    }
    return lvl != 0; /* logical-asserted == wireless (see header) */
}

/* The live pin, not current_mode: callers (behavior_pairing.c, pairing_leds.c)
 * ask at arbitrary times, including from SYS_INITs at our own APPLICATION
 * priority that link order may run before protocol_switch_init(). The pin is
 * configured as an input at POST_KERNEL (protocol_switch_pin_init) so the
 * read is valid from any APPLICATION-level init onwards. */
bool daisy_protocol_switch_is_wireless(void) { return read_switch_wireless(); }

/* Arm a level trigger for the position the switch is NOT in (see header). */
static void arm_switch_interrupt(bool wireless) {
    int err = gpio_pin_interrupt_configure_dt(
        &protocol_switch, wireless ? GPIO_INT_LEVEL_INACTIVE : GPIO_INT_LEVEL_ACTIVE);
    if (err) {
        LOG_ERR("failed to arm switch interrupt (%d)", err);
    }
}

static void enter_low_power(void) {
    LOG_INF("wired + unplugged: entering System OFF (flip to wireless or plug in to wake)");

    /* Wake sources, both of which reset the SoC into a fresh boot:
     *  - the switch reaching the wireless position: its level trigger (GPIO
     *    SENSE) is already armed for exactly that; re-assert it so the wake
     *    never depends on eval_work ordering;
     *  - VBUS rising: the nRF54L wakes from System OFF on its own VBUS pin
     *    (VREGUSB stays started across usbd_disable(); datasheet §5.2,
     *    HW-verified 2026-09-17).
     * Bonds/settings survive (flash-backed).
     *
     * TODO(plan wired-sleep-wake-sources): the matrix rows and the pairing
     * button idle with SENSE armed too, so a keypress also wakes us -- into a
     * pointless 5 s boot. Disarm them (and cut the touchpad rail) here. */
    arm_switch_interrupt(false);

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

    bool wireless = read_switch_wireless();
    enum mode new_mode = wireless ? MODE_WIRELESS : MODE_WIRED;
    if (new_mode != current_mode) {
        current_mode = new_mode;
        LOG_INF("protocol switch -> %s", wireless ? "wireless" : "wired");
    }

    /* Re-arm for the opposite position before applying policy: a flip that
     * landed during the settle window then fires straight away (level
     * trigger) and simply reschedules us. */
    arm_switch_interrupt(wireless);

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

static void switch_isr(const struct device *port, struct gpio_callback *cb,
                       gpio_port_pins_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    /* Level trigger: it would keep firing for as long as the switch sits in
     * its new position. Disarm until eval_work has read the settled position
     * and re-armed for the opposite one. */
    gpio_pin_interrupt_configure_dt(&protocol_switch, GPIO_INT_DISABLE);
    k_work_reschedule_for_queue(&pq, &eval_work, SWITCH_SETTLE);
}
static struct gpio_callback switch_cb;

static int protocol_switch_usb_listener(const zmk_event_t *eh) {
    if (as_zmk_usb_conn_state_changed(eh)) {
        /* Power state changed — re-evaluate the wired-unplug power-off. */
        schedule_eval();
    }
    return 0;
}
ZMK_LISTENER(daisy_protocol_switch, protocol_switch_usb_listener);
ZMK_SUBSCRIPTION(daisy_protocol_switch, zmk_usb_conn_state_changed);

/* Configure the pin early, at POST_KERNEL. daisy_protocol_switch_is_wireless()
 * reads it directly, and other APPLICATION-level inits (pairing_leds.c) may
 * call it before protocol_switch_init() below. An unconfigured nRF input reads
 * 0 (input buffer disconnected), which on an active-low board would be
 * "wireless". Interrupt setup needs the work queue and waits for init. */
static int protocol_switch_pin_init(void) {
    if (!gpio_is_ready_dt(&protocol_switch)) {
        LOG_ERR("protocol switch gpio not ready");
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&protocol_switch, GPIO_INPUT);
    if (err) {
        LOG_ERR("failed to configure protocol switch pin (%d)", err);
    }
    return err;
}
SYS_INIT(protocol_switch_pin_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int protocol_switch_init(void) {
    k_work_queue_start(&pq, pq_stack, K_THREAD_STACK_SIZEOF(pq_stack), K_PRIO_COOP(7), NULL);
    k_thread_name_set(&pq.thread, "daisy_protocol");

    bool wireless = read_switch_wireless();
    current_mode = wireless ? MODE_WIRELESS : MODE_WIRED;
    LOG_INF("boot protocol switch: %s", wireless ? "wireless" : "wired");

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
         * LED until the deferred eval shut BLE down again. */
        zmk_ble_adv_enabled_set(false);
    }
#endif

    gpio_init_callback(&switch_cb, switch_isr, BIT(protocol_switch.pin));
    int err = gpio_add_callback_dt(&protocol_switch, &switch_cb);
    if (err) {
        LOG_ERR("failed to add switch callback (%d)", err);
        return err;
    }
    arm_switch_interrupt(wireless);

    /* Defer the first apply to the coop queue so it runs after BLE/endpoint
     * init has settled, and so a wired+unplugged boot powers off cleanly.
     * See BOOT_EVAL_DELAY for why the boot one waits rather than running now. */
    k_work_reschedule_for_queue(&pq, &eval_work, BOOT_EVAL_DELAY);
    return 0;
}
SYS_INIT(protocol_switch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_NODE_EXISTS(protocol_switch) */

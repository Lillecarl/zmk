#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

LOG_MODULE_REGISTER(daisy_pairing_leds, LOG_LEVEL_INF);

/*
 * BLE pairing indicator: one white LED per BLE host profile.
 *
 * The four white LEDs (pwm_led_w1..w4) form the `pairing_leds` pwm-leds node,
 * driven on pwm20 on both daisy_kb_evt and daisy_kb_dvt1. They are driven at
 * 50% duty (PAIRING_DUTY) via the LED API.
 *
 * For the currently-active profile:
 *   connected             -> solid on
 *   open (nothing paired)  -> fast blink (500 ms)
 *   paired, not connected  -> slow blink (1500 ms)
 * Every other profile's LED is off.
 *
 * This is the pairing-LED slice of the (unbuilt) pmic.c reference, with the
 * on/off led_on/led_off calls replaced by 50%-duty led_set_brightness. It
 * drives nothing but the pairing LEDs — no RGB, protocol switch, or endpoints.
 *
 * Partial-duty output on pwm20 depends on pwm_init.c having primed every
 * channel to 0 at boot (a single nRF PWM instance shares one sequence buffer).
 */

#define PAIRING_LEDS_NODE DT_NODELABEL(pairing_leds)
#define HAS_PAIRING_LEDS  DT_NODE_EXISTS(PAIRING_LEDS_NODE)

#if HAS_PAIRING_LEDS && IS_ENABLED(CONFIG_ZMK_BLE)

#define PAIRING_DUTY      50
#define NUM_PAIRING_LEDS  DT_CHILD_NUM(PAIRING_LEDS_NODE)
#define FAST_BLINK_MS     500
#define SLOW_BLINK_MS     1500

static const struct device *pairing_leds = DEVICE_DT_GET(PAIRING_LEDS_NODE);

/* The blink toggle must NOT run on the system workqueue: on daisy the sysworkq
 * freezes for the whole USB-unplugged window (a USB-stack work item blocks it
 * until replug — see factory_hid.c). A blink scheduled there stalls mid-cycle,
 * leaving the LED stuck solid-on or off until the cable is reconnected. Run it
 * on a dedicated cooperative queue so it keeps toggling regardless of VBUS.
 * pwm20 is on-SoC (no I2C/blocking), so the handler is safe on a coop thread. */
static K_THREAD_STACK_DEFINE(blink_q_stack, 512);
static struct k_work_q blink_q;

static void led_set(uint32_t idx, bool on)
{
    led_set_brightness(pairing_leds, idx, on ? PAIRING_DUTY : 0);
}

/* Only the active profile's LED ever blinks, so a single timer suffices. */
struct blink_context {
    struct k_work_delayable work;
    uint32_t led_idx;
    uint32_t half_period_ms;
    bool is_on;
};
static struct blink_context blink_ctx;

static void blink_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct blink_context *ctx = CONTAINER_OF(dwork, struct blink_context, work);

    /* Bail out if BLE has been turned off since the blink started (e.g. the
     * protocol switch moved to wired). Disconnecting an *unconnected* profile
     * raises no ble_active_profile_changed event, so refresh_pairing_leds()
     * never re-runs to stop us — this self-check is what actually halts the
     * blink and leaves the LED dark. */
    if (!zmk_ble_adv_enabled_get()) {
        led_set(ctx->led_idx, false);
        return;
    }

    ctx->is_on = !ctx->is_on;
    led_set(ctx->led_idx, ctx->is_on);
    k_work_reschedule_for_queue(&blink_q, &ctx->work, K_MSEC(ctx->half_period_ms));
}

static void start_blink(uint32_t led_idx, uint32_t period_ms)
{
    blink_ctx.led_idx = led_idx;
    blink_ctx.half_period_ms = period_ms / 2; /* on for half, off for half */
    blink_ctx.is_on = true;
    led_set(led_idx, true);
    k_work_reschedule_for_queue(&blink_q, &blink_ctx.work, K_MSEC(blink_ctx.half_period_ms));
}

static void stop_blink(void)
{
    k_work_cancel_delayable(&blink_ctx.work);
}

static void refresh_pairing_leds(void)
{
    if (!device_is_ready(pairing_leds)) {
        LOG_ERR("pairing_leds (%s) not ready", pairing_leds->name);
        return;
    }

    /* Start from a clean slate: cancel any blink, turn every LED off. */
    stop_blink();
    for (uint32_t i = 0; i < NUM_PAIRING_LEDS; i++) {
        led_set(i, false);
    }

    /* Bluetooth off (wired mode via the protocol switch drops links and clears
     * permit_adv): there is no pairing state to show, so leave every LED dark. */
    if (!zmk_ble_adv_enabled_get()) {
        return;
    }

    int active = zmk_ble_active_profile_index();

    for (uint32_t i = 0; i < NUM_PAIRING_LEDS && i < ZMK_BLE_PROFILE_COUNT; i++) {
        /* One LED per profile. The PWM channels are wired in reverse of the
         * physical LED order (channel 0 is the 4th LED), so profile 0 maps to
         * the last channel. */
        uint32_t led_index = (NUM_PAIRING_LEDS - 1) - i;

        if ((int)i != active) {
            /* Non-active profiles stay off. */
            continue;
        }

        if (zmk_ble_active_profile_is_connected()) {
            /* Paired and connected: solid on. */
            led_set(led_index, true);
        } else if (zmk_ble_active_profile_is_open()) {
            /* Nothing paired yet: fast blink. */
            start_blink(led_index, FAST_BLINK_MS);
        } else {
            /* Paired but not connected: slow blink. */
            start_blink(led_index, SLOW_BLINK_MS);
        }
    }
}

static int pairing_leds_listener(const zmk_event_t *eh)
{
    if (as_zmk_ble_active_profile_changed(eh)) {
        refresh_pairing_leds();
    }
    return 0;
}

ZMK_LISTENER(daisy_pairing_leds, pairing_leds_listener);
ZMK_SUBSCRIPTION(daisy_pairing_leds, zmk_ble_active_profile_changed);

static int pairing_leds_init(void)
{
    k_work_queue_start(&blink_q, blink_q_stack, K_THREAD_STACK_SIZEOF(blink_q_stack),
                       K_PRIO_COOP(7), NULL);
    k_thread_name_set(&blink_q.thread, "daisy_pairing_led");
    k_work_init_delayable(&blink_ctx.work, blink_handler);
    refresh_pairing_leds();
    return 0;
}
SYS_INIT(pairing_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* HAS_PAIRING_LEDS && CONFIG_ZMK_BLE */

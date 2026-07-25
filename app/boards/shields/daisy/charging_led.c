#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

#if IS_ENABLED(CONFIG_DAISY_FACTORY)
#include "factory_state.h"
#else
/* No factory interface built -> factory mode can never be active. */
static inline bool daisy_factory_mode_active(void) { return false; }
#endif

LOG_MODULE_REGISTER(daisy_charging_led, LOG_LEVEL_INF);

/*
 * Charge-status RGB indicator.
 *
 * This is a momentary plug-in acknowledgement, not a persistent charge gauge:
 * when a cable is attached the LED shows the charge status for one second and
 * then goes dark. It stays off the rest of the time (including while unplugged).
 *
 *   plug in + charging      -> amber (R + G) for 1 s, then off
 *   plug in + full/complete -> white (R + G + B) for 1 s, then off
 *
 * The two keyboard boards wire the status RGB differently, so the color
 * output is abstracted behind set_rgb():
 *
 *   daisy_kb_evt : R/G/B are plain gpio-leds (led_r/led_g/led_b, on P2.xx).
 *                  Driven as GPIO on/off.
 *   daisy_kb_dvt1: the same pins are also exposed as a pwm-leds node
 *                  (rgb_pwmleds -> pwm21, channels pwm_led_r/g/b). Driven
 *                  via the LED API at 25% duty.
 *
 * The PWM node is preferred whenever it exists (dvt1); otherwise the GPIO
 * shadows are used (evt). Boards using the daisy shield without either node
 * (e.g. dev kits) compile set_rgb() as a no-op.
 */

#define HAS_RGB_PWM DT_NODE_EXISTS(DT_NODELABEL(pwm_led_r))
#define HAS_RGB_GPIO                                                            \
    (DT_NODE_EXISTS(DT_NODELABEL(led_r)) &&                                     \
     DT_NODE_EXISTS(DT_NODELABEL(led_g)) &&                                     \
     DT_NODE_EXISTS(DT_NODELABEL(led_b)))

#if HAS_RGB_PWM
/* dvt1: partial-duty PWM on the shared pwm21 RGB channels. */
#define RGB_PWM_DUTY 25
static const struct device *rgb_dev =
    DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(pwm_led_r)));
/* Channel index = position of each color's child within the pwm-leds node. */
#define RGB_CH_R DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_r))
#define RGB_CH_G DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_g))
#define RGB_CH_B DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_b))

static bool rgb_ready(void)
{
    return device_is_ready(rgb_dev);
}

static void set_rgb(bool r, bool g, bool b)
{
    if (!rgb_ready()) {
        return;
    }
    led_set_brightness(rgb_dev, RGB_CH_R, r ? RGB_PWM_DUTY : 0);
    led_set_brightness(rgb_dev, RGB_CH_G, g ? RGB_PWM_DUTY : 0);
    led_set_brightness(rgb_dev, RGB_CH_B, b ? RGB_PWM_DUTY : 0);
}

static void rgb_init(void)
{
    /* Prime all three channels so a later per-channel duty change doesn't
     * disturb siblings sharing the pwm instance. */
    set_rgb(false, false, false);
}

#elif HAS_RGB_GPIO
/* evt: plain GPIO RGB (on/off only). */
static const struct gpio_dt_spec rgb_r =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_r), gpios);
static const struct gpio_dt_spec rgb_g =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_g), gpios);
static const struct gpio_dt_spec rgb_b =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_b), gpios);

static bool rgb_ready(void)
{
    return gpio_is_ready_dt(&rgb_r) && gpio_is_ready_dt(&rgb_g) &&
           gpio_is_ready_dt(&rgb_b);
}

static void set_rgb(bool r, bool g, bool b)
{
    if (!rgb_ready()) {
        return;
    }
    gpio_pin_set_dt(&rgb_r, r);
    gpio_pin_set_dt(&rgb_g, g);
    gpio_pin_set_dt(&rgb_b, b);
}

static void rgb_init(void)
{
    if (!rgb_ready()) {
        LOG_ERR("rgb gpio pins not ready");
        return;
    }
    gpio_pin_configure_dt(&rgb_r, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&rgb_g, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&rgb_b, GPIO_OUTPUT_INACTIVE);
}

#else
static void set_rgb(bool r, bool g, bool b)
{
    ARG_UNUSED(r);
    ARG_UNUSED(g);
    ARG_UNUSED(b);
}
static void rgb_init(void) {}
#endif

/*
 * "Charging" vs "fully charged" is a charge state, not a level, so read it
 * straight from the nPM1300 charger's BCHGCHARGESTATUS register rather than
 * from zmk_battery_state_of_charge() (which now reports a voltage-estimated
 * SoC percentage via the zmk,battery chosen node, a different thing).
 */
#define HAS_CHARGER DT_NODE_EXISTS(DT_NODELABEL(npm1300_charger))
#if HAS_CHARGER
static const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));

/* BCHGCHARGESTATUS bit fields (nPM1300 product spec). */
#define CHG_STAT_BATTERY_DETECTED BIT(0)
#define CHG_STAT_COMPLETED        BIT(1)
#define CHG_STAT_TRICKLE          BIT(2)
#define CHG_STAT_CONST_CURRENT    BIT(3)
#define CHG_STAT_CONST_VOLTAGE    BIT(4)

/* Returns true if the charger reports the battery as fully charged. */
static bool battery_full(void)
{
    if (!device_is_ready(charger)) {
        return false;
    }

    struct sensor_value status;
    int rc = sensor_sample_fetch(charger);
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &status);
    }
    if (rc != 0) {
        LOG_WRN("charger status read failed: %d", rc);
        /* Powered but status unknown -> treat as still charging (amber). */
        return false;
    }

    LOG_DBG("charge status=0x%02x", status.val1);
    return (status.val1 & CHG_STAT_COMPLETED) != 0;
}
#else
static bool battery_full(void)
{
    return false;
}
#endif /* HAS_CHARGER */

/* How long the charge color stays lit after a cable is plugged in. */
#define ON_DURATION K_SECONDS(1)
static void off_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(off_work, off_work_handler);

static void charge_led_off(void)
{
    if (daisy_factory_mode_active()) {
        /* Factory mode owns the LEDs -- leave them untouched. */
        return;
    }
    set_rgb(false, false, false);
}

/* Show the current charge status briefly, then arm the auto-off. */
static void show_charge_indication(void)
{
    if (daisy_factory_mode_active()) {
        /* Factory mode owns the LEDs -- leave them untouched. */
        return;
    }

    if (battery_full()) {
        set_rgb(true, true, true); /* white */
    } else {
        set_rgb(true, true, false); /* amber */
    }
    k_work_reschedule(&off_work, ON_DURATION);
}

static void off_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    charge_led_off();
}

static int daisy_charging_led_update_listener(const zmk_event_t *eh)
{
    struct zmk_usb_conn_state_changed *usb_changed = as_zmk_usb_conn_state_changed(eh);
    if (!usb_changed) {
        return 0;
    }

    LOG_INF("usb powered: %d", zmk_usb_is_powered());

    if (zmk_usb_is_powered()) {
        /* Cable just plugged in: flash the charge status, then auto-off. */
        show_charge_indication();
    } else {
        /* Unplugged before the timer fired: cancel it and go dark now. */
        k_work_cancel_delayable(&off_work);
        charge_led_off();
    }
    return 0;
}

ZMK_LISTENER(daisy_charging_led, daisy_charging_led_update_listener);
ZMK_SUBSCRIPTION(daisy_charging_led, zmk_usb_conn_state_changed);

static int daisy_charging_led_init(void)
{
    rgb_init();
    /* If a cable is already attached at boot, give the same brief flash. */
    if (zmk_usb_is_powered()) {
        show_charge_indication();
    }
    return 0;
}
SYS_INIT(daisy_charging_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/usb_conn_state_changed.h>

LOG_MODULE_REGISTER(daisy_leds, LOG_LEVEL_INF);

/* Standalone LED demo for daisy_kb_evt: drives the 4 white LEDs
 * (led_w1..led_w4) based on the pairing button and protocol switch.
 * Intentionally independent of BLE/USB/PMIC state. */

#define HAS_DEMO_LEDS                                                           \
    (DT_NODE_EXISTS(DT_NODELABEL(led_w1)) &&                                    \
     DT_NODE_EXISTS(DT_NODELABEL(led_w2)) &&                                    \
     DT_NODE_EXISTS(DT_NODELABEL(led_w3)) &&                                    \
     DT_NODE_EXISTS(DT_NODELABEL(led_w4)) &&                                    \
     DT_NODE_EXISTS(DT_NODELABEL(pairing_button)) &&                            \
     DT_NODE_EXISTS(DT_NODELABEL(protocol_switch)))

#if HAS_DEMO_LEDS

static const struct gpio_dt_spec white_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_w1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_w2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_w3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_w4), gpios),
};
#define NUM_LEDS ARRAY_SIZE(white_leds)

static const struct gpio_dt_spec pairing_button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pairing_button), gpios);
static const struct gpio_dt_spec protocol_switch =
    GPIO_DT_SPEC_GET(DT_NODELABEL(protocol_switch), gpios);

#define HAS_STATUS_RGB DT_NODE_EXISTS(DT_NODELABEL(rgb1_red_pwm_led))
#if HAS_STATUS_RGB
/* Parent pwm-leds node owns channels 0=R, 1=G, 2=B. */
static const struct device *rgb_leds =
    DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(rgb1_red_pwm_led)));

static int daisy_leds_update_listener(const zmk_event_t *eh);
ZMK_LISTENER(daisy_leds, daisy_leds_update_listener);
ZMK_SUBSCRIPTION(daisy_leds, zmk_usb_conn_state_changed);

/* Same physical pins are also declared as gpio-leds (led_r/g/b). The
 * gpio-leds driver configures them as GPIO outputs at POST_KERNEL, which
 * fights pwm21's PSEL. Release them to input (high-Z) so the PWM can drive. */
#define HAS_RGB_GPIO_SHADOWS                                                    \
    (DT_NODE_EXISTS(DT_NODELABEL(led_r)) &&                                     \
     DT_NODE_EXISTS(DT_NODELABEL(led_g)) &&                                     \
     DT_NODE_EXISTS(DT_NODELABEL(led_b)))
#if HAS_RGB_GPIO_SHADOWS
static const struct gpio_dt_spec rgb_shadow_pins[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_r), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_g), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_b), gpios),
};
#endif
#endif

static struct gpio_callback button_cb_data;
static struct gpio_callback switch_cb_data;

static uint32_t press_count;
static int64_t last_button_ms;
static int64_t last_switch_ms;
#define DEBOUNCE_MS 30

static void all_off(uint32_t idx)
{
    for (size_t i = 0; i < NUM_LEDS; i++) {
        gpio_pin_set_dt(&white_leds[i], 0);
    }
}

static void show_led(uint32_t idx)
{
    for (size_t i = 0; i < NUM_LEDS; i++) {
        gpio_pin_set_dt(&white_leds[i], i == idx);
    }
}

static void refresh_leds(void)
{
    int sw = gpio_pin_get_dt(&protocol_switch);
    uint32_t idx = (NUM_LEDS - 1) - (press_count % NUM_LEDS);
    if (sw) {
        all_off(idx);
    } else  {
        show_led(idx);
    }
}

/* GPIO callbacks run in ISR context on nRF. Doing the LED refresh and
 * logging here can delay the BLE LLL prepare slot past its budget (seen as
 * lll_adv.c `EVENT_OVERHEAD_START_US` assertions). Defer to a k_work. */
static void leds_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int sw = gpio_pin_get_dt(&protocol_switch);
    LOG_INF("state: presses=%u switch=%d", press_count, sw);
    refresh_leds();
}
static K_WORK_DEFINE(leds_work, leds_work_handler);

static void button_pressed(const struct device *port, struct gpio_callback *cb,
                           gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int64_t now = k_uptime_get();
    if (now - last_button_ms < DEBOUNCE_MS) {
        return;
    }
    last_button_ms = now;

    press_count++;
    k_work_submit(&leds_work);
}

static void switch_changed(const struct device *port, struct gpio_callback *cb,
                           gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int64_t now = k_uptime_get();
    if (now - last_switch_ms < DEBOUNCE_MS) {
        return;
    }
    last_switch_ms = now;

    k_work_submit(&leds_work);
}

static int daisy_leds_init(void)
{
    for (size_t i = 0; i < NUM_LEDS; i++) {
        if (!gpio_is_ready_dt(&white_leds[i])) {
            LOG_ERR("led_w%zu not ready", i + 1);
            return 0;
        }
        gpio_pin_configure_dt(&white_leds[i], GPIO_OUTPUT_INACTIVE);
    }

    if (!gpio_is_ready_dt(&pairing_button) || !gpio_is_ready_dt(&protocol_switch)) {
        LOG_ERR("button/switch not ready");
        return 0;
    }

    gpio_pin_configure_dt(&pairing_button, GPIO_INPUT);
    gpio_pin_configure_dt(&protocol_switch, GPIO_INPUT);

    /* Pairing button: fire on the press edge (active-low -> falling). */
    gpio_pin_interrupt_configure_dt(&pairing_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(pairing_button.pin));
    gpio_add_callback(pairing_button.port, &button_cb_data);

    /* Bistable switch: react to either direction. */
    gpio_pin_interrupt_configure_dt(&protocol_switch, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&switch_cb_data, switch_changed, BIT(protocol_switch.pin));
    gpio_add_callback(protocol_switch.port, &switch_cb_data);

#if HAS_STATUS_RGB
// #if HAS_RGB_GPIO_SHADOWS
//     for (size_t i = 0; i < ARRAY_SIZE(rgb_shadow_pins); i++) {
//         if (gpio_is_ready_dt(&rgb_shadow_pins[i])) {
//             gpio_pin_configure_dt(&rgb_shadow_pins[i], GPIO_INPUT);
//         }
//     }
// #endif
    if (device_is_ready(rgb_leds)) {
        /* Partial-duty PWM output on pwm21 isn't currently working on this
         * board (nrfx_pwm_simple_playback path), but led_on() hits a nrfx
         * fast path that drives the pin directly via PSEL at 100% duty. */
        int r0 = led_off(rgb_leds, 0);
        int r1 = led_off(rgb_leds, 1);
        int r2 = led_off(rgb_leds, 2);
        if (zmk_usb_is_powered()) {
            led_on(rgb_leds, 0);
            led_on(rgb_leds, 1);
            led_off(rgb_leds, 2);
        } else {
            led_off(rgb_leds, 0);
            led_off(rgb_leds, 1);
            led_off(rgb_leds, 2);
        }
        LOG_INF("rgb %s led_on ret=%d,%d,%d", rgb_leds->name, r0, r1, r2);
    } else {
        LOG_ERR("rgb_leds (%s) not ready", rgb_leds->name);
    }
#endif

    refresh_leds();
    return 0;
}
SYS_INIT(daisy_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int daisy_leds_update_listener(const zmk_event_t *eh)
{
#if HAS_STATUS_RGB
    struct zmk_usb_conn_state_changed *usb_changed = as_zmk_usb_conn_state_changed(eh);
    if (usb_changed) {
        LOG_INF("zoid: zmk_usb_is_powered: %d\n", zmk_usb_is_powered());
        if (device_is_ready(rgb_leds)) {
            /* Partial-duty PWM output on pwm21 isn't currently working on this
             * board (nrfx_pwm_simple_playback path), but led_on() hits a nrfx
             * fast path that drives the pin directly via PSEL at 100% duty. */
            if (zmk_usb_is_powered()) {
                led_on(rgb_leds, 0);
                led_on(rgb_leds, 1);
                led_off(rgb_leds, 2);
            } else {
                led_off(rgb_leds, 0);
                led_off(rgb_leds, 1);
                led_off(rgb_leds, 2);
            }
        } else {
            LOG_ERR("rgb_leds (%s) not ready", rgb_leds->name);
        }
    }
#endif
    return 0;
}

#endif /* HAS_DEMO_LEDS */

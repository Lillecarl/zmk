#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/hid_indicators.h>

#if DT_NODE_EXISTS(DT_NODELABEL(caps_led))

#define CAPS_LED DT_NODELABEL(caps_led)
static const struct gpio_dt_spec caps_led = GPIO_DT_SPEC_GET(CAPS_LED, gpios);

static int led_keylock_listener_cb(const zmk_event_t *eh) {
  zmk_hid_indicators_t flags = zmk_hid_indicators_get_current_profile();

  if (!gpio_is_ready_dt(&caps_led)) {
    return 0;
  }


  gpio_pin_set_dt(&caps_led, flags & HID_USAGE_LED_CAPS_LOCK);

  return 0;
}

ZMK_LISTENER(led_indicators_listener, led_keylock_listener_cb);
ZMK_SUBSCRIPTION(led_indicators_listener, zmk_hid_indicators_changed);

#endif /* DT_NODE_EXISTS(DT_NODELABEL(caps_led)) */

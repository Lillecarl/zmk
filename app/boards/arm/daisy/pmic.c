#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/drivers/led.h>
#include <zephyr/dt-bindings/regulator/npm1300.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/printk.h>
#include <getopt.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/wpm_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>

static const struct device *pmic_leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_leds));
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led), gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led), gpios);

static int daisy_leds_update_listener(const zmk_event_t *eh);
ZMK_LISTENER(daisy_leds, daisy_leds_update_listener);

ZMK_SUBSCRIPTION(daisy_leds, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(daisy_leds, zmk_battery_state_changed);
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(daisy_leds, zmk_usb_conn_state_changed);
#endif
#if defined(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(daisy_leds, zmk_ble_active_profile_changed);
#endif

static int daisy_leds_update_listener(const zmk_event_t *eh)
{
	struct zmk_endpoint_changed *ep_changed = as_zmk_endpoint_changed(eh);
	struct zmk_battery_state_changed *bat_changed = as_zmk_battery_state_changed(eh);
	struct zmk_usb_conn_state_changed *usb_changed = as_zmk_usb_conn_state_changed(eh);
	struct zmk_ble_active_profile_changed *ble_changed = as_zmk_ble_active_profile_changed(eh);

	if (usb_changed) {
		printk("zoid: usb_conn_state_changed\n");
		printk("zoid: zmk_usb_is_powered: %d\n", zmk_usb_is_powered());
		printk("zoid: zmk_usb_get_status: %d\n", zmk_usb_get_status());
		printk("zoid: zmk_usb_get_conn_state: %d\n", zmk_usb_get_conn_state());
		if (zmk_usb_get_conn_state() == ZMK_USB_CONN_HID) {
			// White - Compter connected
			gpio_pin_set_dt(&red_led, 1);
			gpio_pin_set_dt(&green_led, 1);
			gpio_pin_set_dt(&blue_led, 1);
		} else if (zmk_usb_get_conn_state() == ZMK_USB_CONN_POWERED) {
			// Yellow - No Computer, but USB connected (e.g. charger)
			gpio_pin_set_dt(&red_led, 1);
			gpio_pin_set_dt(&green_led, 1);
			gpio_pin_set_dt(&blue_led, 0);
		} else if (zmk_usb_get_conn_state() == ZMK_USB_CONN_NONE) {
			// Red - Battery connected, no USB
			gpio_pin_set_dt(&red_led, 1);
			gpio_pin_set_dt(&green_led, 1);
			gpio_pin_set_dt(&blue_led, 0);
		}
	}

	if (ble_changed) {
		printk("zoid: ble_active_profile_changed\n");
		printk("zoid: BLE Active Profile Index: %d\n", zmk_ble_active_profile_index());
		printk("zoid: BLE Active Profile Conn:  %d\n", zmk_ble_active_profile_is_connected());
		printk("zoid: BLE Active Profile Open:  %d\n", zmk_ble_active_profile_is_open());
		for (int i = 0; i < 3; i++) {
			if (i == zmk_ble_active_profile_index()) {
				if (zmk_ble_active_profile_is_connected()) {
					led_on(pmic_leds, i);
				} else {
					// TODO: Not blinking
					// led_blink(pmic_leds, i, 100, 1000);
					led_off(pmic_leds, i);
				}
			} else {
				led_off(pmic_leds, i);
			}
		}
	}
	if (bat_changed) {
		printk("zoid: battery_state_changed\n");
		uint8_t soc = bat_changed->state_of_charge;
		//uint8_t soc = zmk_battery_state_of_charge();
		printk("zoid: Battery SOC: %d\n", soc);
	}

	if (ep_changed) {
		printk("zoid: endpoint_changed\n");
		if (ep_changed->endpoint.transport == ZMK_TRANSPORT_USB) {
			printk("zoid: endpoint.transport: USB\n");
			// All LEDs off
			led_off(pmic_leds, 0U);
			led_off(pmic_leds, 1U);
			led_off(pmic_leds, 2U);
		} else if (ep_changed->endpoint.transport == ZMK_TRANSPORT_BLE) {
			printk("zoid: endpoint.transport: BLE\n");
			printk("zoid: endpoint.ble.profile_index: %d\n", ep_changed->endpoint.ble.profile_index);
			// zmk_ble_active_profile_changed doesn't get called
			// So we have to manually enable the LED again
			led_on(pmic_leds, ep_changed->endpoint.ble.profile_index);
		}
	}

	return 0;
}

static void input_cb(struct input_event *evt)
{
	/* Ignore unrelated events */
	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	switch (evt->code) {
	case INPUT_BTN_SELECT:
		printk("daisy: Pressed pairing button - %d\n", evt->value);
		// Short press selects the next profile
		// TODO: This breaks the USB shell - why? Debug with UART
		// zmk_ble_prof_next();
		// TODO: Long press should repair the current profile
		break;
	case INPUT_BTN_MODE:
		printk("daisy: Changed mode switch - %d. Current transport: %d\n", evt->value, zmk_endpoints_selected().transport);
		if (evt->value)
			zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
		else
			zmk_endpoints_select_transport(ZMK_TRANSPORT_BLE);
		break;
	default:
		printk("Unrecognized input code %u value %d\n",
			evt->code, evt->value);
		return;
	}
}
INPUT_CALLBACK_DEFINE(NULL, input_cb);

int led_init(void)
{
	if (!device_is_ready(pmic_leds)) {
		printk("Error: led device is not ready\n");
		return 0;
	}

	/* Turn all BLE LEDs off on boot */
	led_off(pmic_leds, 0U);
	led_off(pmic_leds, 1U);
	led_off(pmic_leds, 2U);

	if (!gpio_is_ready_dt(&red_led)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&green_led)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&blue_led)) {
		return 0;
	}
	gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_ACTIVE);

	/* Turn all the power LEDs off on boot */
	gpio_pin_set_dt(&red_led, 0);
	gpio_pin_set_dt(&green_led, 0);
	gpio_pin_set_dt(&blue_led, 0);

	return 0;
}
SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

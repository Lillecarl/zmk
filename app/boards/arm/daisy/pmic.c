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
#include <zmk/behavior.h>
#include <drivers/behavior.h>

#define DT_DRV_COMPAT zmk_behavior_led_trigger
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static const struct device *pmic_leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_leds));
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led), gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led), gpios);
static const struct gpio_dt_spec protocol_switch = GPIO_DT_SPEC_GET(DT_NODELABEL(protocol_switch), gpios);

/* State of the protocol switch (BLE or USB). 1 if set to USB */
static int protocol_switch_usb;

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

struct led_blink_context {
    struct k_work_delayable blink_work;
    uint32_t led_idx;
    bool is_on;
    uint32_t interval_ms;
};
static struct led_blink_context led_ctx;

static void led_blink_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct led_blink_context *ctx = CONTAINER_OF(dwork, struct led_blink_context, blink_work);

    ctx->is_on = !ctx->is_on;

    if (ctx->is_on) {
        led_on(pmic_leds, ctx->led_idx);
    } else {
        led_off(pmic_leds, ctx->led_idx);
    }

    /* Blink again after timeout */
    k_work_reschedule(&ctx->blink_work, K_MSEC(ctx->interval_ms));
}

void start_blinking_led(uint32_t led_idx, uint32_t period_ms)
{
    if (!device_is_ready(pmic_leds)) {
        printk("LED device not ready\n");
        return;
    }

    led_ctx.led_idx = led_idx;
    led_ctx.interval_ms = period_ms / 2; // On for half, off for half
    led_ctx.is_on = false;

    k_work_init_delayable(&led_ctx.blink_work, led_blink_handler);
    k_work_reschedule(&led_ctx.blink_work, K_NO_WAIT);
}

void stop_blinking_led(void)
{
    /* Cancel any pending work */
    k_work_cancel_delayable(&led_ctx.blink_work);

    /* Ensure LED is off */
    led_off(pmic_leds, led_ctx.led_idx);
}

void update_power_led(void)
{
	// | Power Level   | Plugged in | Color |
	// | <  10%        | No         | Red   |
	// | >= 10%        | No         | Off   |
	// | <  90%        | Yes        | Amber |
	// | >= 90%        | Yes        | White |
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
		gpio_pin_set_dt(&green_led, 0);
		gpio_pin_set_dt(&blue_led, 0);
	} else {
		printk("daisy: ERROR! Unexpected state *\n");
	}
}

void update_ble_leds(void)
{
	printk("daisy: update_ble_leds: %s\n", protocol_switch_usb ? "USB" : "BLE");
	/* Stop all blinking and turn all LEDs off. Start clean */
	stop_blinking_led();
	for (int i = 0; i < 3; i++) {
		led_off(pmic_leds, i);
	}

	/* Check the switch status not the currently selected transport.
	 * Because bluetooth transport is selected only if we have an active
	 * profile. If it's not set to BLE, we don't have to show any pairing
	 * LEDs. */
	protocol_switch_usb = gpio_pin_get_dt(&protocol_switch);
	if (protocol_switch_usb)
		return;

	for (int i = 0; i < 3; i++) {
		if (i != zmk_ble_active_profile_index()) {
			/* Turn LEDs of not active profiles off */
			led_off(pmic_leds, i);
		} else if (zmk_ble_active_profile_is_connected()) {
			/* Paired and connected, solid on */
			led_on(pmic_leds, i);
		} else if (zmk_ble_active_profile_is_open()) {
			/* Fast blink if nothing paired */
			start_blinking_led(i, 700);
		} else {
			/* Slow blink if paired but not connected */
			start_blinking_led(i, 1400);
		}
	}
}

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
		update_power_led();
	}

	if (ble_changed) {
		printk("zoid: ble_active_profile_changed\n");
		printk("zoid: BLE Active Profile Index: %d\n", zmk_ble_active_profile_index());
		printk("zoid: BLE Active Profile Conn:  %d\n", zmk_ble_active_profile_is_connected());
		printk("zoid: BLE Active Profile Open:  %d\n", zmk_ble_active_profile_is_open());
		update_ble_leds();
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
			update_ble_leds();
		}
	}

	return 0;
}

int led_init(void)
{
	if (!device_is_ready(pmic_leds)) {
		printk("Error: led device is not ready\n");
		return 0;
	}

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

	protocol_switch_usb = gpio_pin_get_dt(&protocol_switch);
	printk("daisy: Protocol switch on boot: %s\n",
			protocol_switch_usb ? "USB" : "BLE");
	if (protocol_switch_usb)
		zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
	else
		zmk_endpoints_select_transport(ZMK_TRANSPORT_BLE);

	/* Initialize the status LEDs */
	update_power_led();
	update_ble_leds();

	return 0;
}
SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int on_led_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event)
{
	protocol_switch_usb = binding->param1;
	printk("daisy: Changed mode switch behavior %d. Current transport: %d\n", protocol_switch_usb, zmk_endpoints_selected().transport);

	if (protocol_switch_usb)
		zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
	else
		zmk_endpoints_select_transport(ZMK_TRANSPORT_BLE);

	/* If we booted in BLE mode without any device connected, the
	 * transport is USB. So switching the switch to USB does not
	 * trigger a transport change. So we need to make sure the LEDs
	 * are correct here. */
	update_ble_leds();

	return ZMK_BEHAVIOR_OPAQUE;
}

/* Define the behavior driver API */
static const struct behavior_driver_api behavior_led_driver_api = {
    .binding_pressed = on_led_binding_pressed,
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

/* Register the behavior as "zmk,behavior-led-trigger" */
BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_led_driver_api);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

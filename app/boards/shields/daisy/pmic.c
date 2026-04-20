#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/mfd/npm13xx.h>
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
#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>
#include <zmk/behavior.h>
#include <drivers/behavior.h>

#define DT_DRV_COMPAT zmk_behavior_led_trigger
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Boards using the daisy shield expose their 3-channel pairing indicator
 * via a &pairing_leds node (npm1300 LED controller on flower/daisy, PWM
 * whites on daisy_kb_evt). led_on/off work on both via the LED API. The
 * RGB status indicator is a pwm-leds node labeled rgb1_red_pwm_led. */
#define HAS_PAIRING_LEDS DT_NODE_EXISTS(DT_NODELABEL(pairing_leds))
#define HAS_STATUS_RGB   DT_NODE_EXISTS(DT_NODELABEL(rgb1_red_pwm_led))

#if HAS_PAIRING_LEDS
static const struct device *pairing_leds = DEVICE_DT_GET(DT_NODELABEL(pairing_leds));
#endif
#if HAS_STATUS_RGB
/* Use the parent pwm-leds node explicitly — boards with more than one
 * pwm-leds instance (e.g. daisy_kb_evt has a separate white-LED block)
 * would otherwise get a non-deterministic pick from DT_COMPAT_GET_ANY. */
static const struct device *rgb_leds =
	DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(rgb1_red_pwm_led)));
#endif
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

#if HAS_PAIRING_LEDS
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
        led_on(pairing_leds, ctx->led_idx);
    } else {
        led_off(pairing_leds, ctx->led_idx);
    }

    /* Blink again after timeout */
    k_work_reschedule(&ctx->blink_work, K_MSEC(ctx->interval_ms));
}

void start_blinking_led(uint32_t led_idx, uint32_t period_ms)
{
    if (!device_is_ready(pairing_leds)) {
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
    led_off(pairing_leds, led_ctx.led_idx);
}
#endif /* HAS_PAIRING_LEDS */

#define BRT        25
#define SET_OFF    set_rgb_color(  0,   0,   0)
#define SET_RED    set_rgb_color(BRT,   0,   0)
#define SET_GREEN  set_rgb_color(  0, BRT,   0)
#define SET_BLUE   set_rgb_color(  0,   0, BRT)
#define SET_AMBER  set_rgb_color(BRT, BRT,   0)
#define SET_DARK_AMBER set_rgb_color(BRT*3/2, BRT,   0)
#define SET_PURPLE set_rgb_color(BRT,   0, BRT)
#define SET_CYAN   set_rgb_color(  0, BRT, BRT)
#define SET_WHITE  set_rgb_color(BRT, BRT, BRT)

#if HAS_STATUS_RGB
void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!device_is_ready(rgb_leds)) {
        return;
    }

    led_set_brightness(rgb_leds, 0, r);
    led_set_brightness(rgb_leds, 1, g);
    led_set_brightness(rgb_leds, 2, b);
}
#endif /* HAS_STATUS_RGB */

void check_protocol_switch()
{
	protocol_switch_usb = gpio_pin_get_raw(protocol_switch.port, protocol_switch.pin);
	printk("daisy: update_ble_leds: %s (%d)\n", protocol_switch_usb ? "USB" : "BLE", protocol_switch_usb);

	zmk_ble_adv_enabled_set(!protocol_switch_usb);
}

#if HAS_STATUS_RGB
void update_power_led(void)
{
	uint8_t soc = zmk_battery_state_of_charge();
	enum zmk_usb_conn_state conn_state = zmk_usb_get_conn_state();
	printk("daisy: update_power_led - SOC: %d Connected: %d\n",
			soc, conn_state != ZMK_USB_CONN_NONE);
	// | Power Level   | Plugged in | Color  | Comment             |
	// |     0%        | No         | Purple | Impossible/critical |
	// | <  10%        | No         | Red    | Critically low      |
	// | >= 10%        | No         | Off    | Okay                |
	// |     0%        | Yes        | Green  | No battery          |
	// | <  10%        | Yes        | Blue   | Charging (low)      |
	// | <  90%        | Yes        | Amber  | Charging            |
	// | >= 90%        | Yes        | White  | Fully charged       |
	if (conn_state == ZMK_USB_CONN_NONE) {
		if (soc == 0) {
			// Purple - For debugging only. Happens during startup
			SET_PURPLE;
		} else if (soc < 10) {
			// Red
			SET_RED;
		} else {
			// Off
			SET_OFF;
		}
	} else {
		if (soc == 0) {
			// Green - For debugging only
			// SET_GREEN;
		} else if (soc < 10) {
			// Blue - For debugging only
			// SET_BLUE;
			SET_DARK_AMBER;
		} else if (soc < 90) {
			// Amber
			SET_AMBER;
		} else {
			// White
			SET_WHITE;
		}
	}
}

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static int led_battery_listener_cb(const zmk_event_t *eh) {
    // uint8_t battery_level = as_zmk_battery_state_changed(eh)->state_of_charge;
    update_power_led();
    return 0;
}

ZMK_LISTENER(led_battery_listener, led_battery_listener_cb);
ZMK_SUBSCRIPTION(led_battery_listener, zmk_battery_state_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
#endif /* HAS_STATUS_RGB */

void update_ble_leds(void)
{
	check_protocol_switch();
#if HAS_PAIRING_LEDS
	/* Stop all blinking and turn all LEDs off. Start clean */
	stop_blinking_led();
	for (int i = 0; i < 3; i++) {
		led_off(pairing_leds, i);
	}

	/* Check the switch status not the currently selected transport.
	 * Because bluetooth transport is selected only if we have an active
	 * profile. If it's not set to BLE, we don't have to show any pairing
	 * LEDs. */
	if (protocol_switch_usb)
		return;

	for (int i = 0; i < 3; i++) {
		/* Order of GPIOs and label is reversed */
		int led_index = 2 - i;
		if (i != zmk_ble_active_profile_index()) {
			/* Turn LEDs of not active profiles off */
			led_off(pairing_leds, led_index);
		} else if (zmk_ble_active_profile_is_connected()) {
			/* Paired and connected, solid on */
			led_on(pairing_leds, led_index);
		} else if (zmk_ble_active_profile_is_open()) {
			/* Fast blink if nothing paired */
			start_blinking_led(led_index, 500);
		} else {
			/* Slow blink if paired but not connected */
			start_blinking_led(led_index, 1500);
		}
	}
#endif /* HAS_PAIRING_LEDS */
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
		printk("zoid: zmk_usb_get_conn_state: %d\n", zmk_usb_get_conn_state());
#if HAS_STATUS_RGB
		update_power_led();
#endif
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
#if HAS_PAIRING_LEDS
			// All LEDs off
			led_off(pairing_leds, 0U);
			led_off(pairing_leds, 1U);
			led_off(pairing_leds, 2U);
#endif
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
#if HAS_PAIRING_LEDS
	if (!device_is_ready(pairing_leds)) {
		printk("Error: PMIC led device is not ready\n");
		return 0;
	}
#endif

#if HAS_STATUS_RGB
	if (!device_is_ready(rgb_leds)) {
		printk("Error: PWM LED device %s is not ready", rgb_leds->name);
		return 0;
	}
#endif

	protocol_switch_usb = gpio_pin_get_dt(&protocol_switch);
	printk("daisy: Protocol switch on boot: %s (%d)\n",
			protocol_switch_usb ? "USB" : "BLE",
			protocol_switch_usb);
	protocol_switch_usb = gpio_pin_get_raw(protocol_switch.port, protocol_switch.pin);
	printk("daisy: Protocol switch on boot: RAW %s (%d)\n",
			protocol_switch_usb ? "USB" : "BLE",
			protocol_switch_usb);
	if (protocol_switch_usb)
		zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_USB);
	else
		zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_BLE);

	/* Initialize the status LEDs */
#if HAS_STATUS_RGB
	update_power_led();
#endif
	update_ble_leds();

	return 0;
}
SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int on_led_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event)
{
	protocol_switch_usb = binding->param1;
	printk("daisy: Changed mode switch behavior %d. Current transport: %d\n", protocol_switch_usb, zmk_endpoint_get_selected().transport);
	protocol_switch_usb = gpio_pin_get_raw(protocol_switch.port, protocol_switch.pin);
	printk("daisy: gpio RAW: %s (%d)\n", protocol_switch_usb ? "USB" : "BLE", protocol_switch_usb);

	if (protocol_switch_usb)
		zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_USB);
	else
		zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_BLE);

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

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/drivers/led.h>
#include <zephyr/dt-bindings/regulator/npm1300.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/sys/printk.h>
#include <getopt.h>

int led_init(void)
{
	if (!device_is_ready(leds)) {
		printk("Error: led device is not ready\n");
		return 0;
	}

	/* Turn all LEDs off on boot */
	led_off(leds, 0U);
	led_off(leds, 1U);
	led_off(leds, 2U);

	return 0;
}
SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

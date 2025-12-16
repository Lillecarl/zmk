#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zmk/backlight.h>
#include <zmk/rgb_underglow.h>

int startup(void)
{
	const struct device *p0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

	/* Disable bootloader circuitry */
	gpio_pin_configure(p0, 5, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(p0, 5, 0);

	/* Check lid-closed pin (Previously called SLEEP#) */
	gpio_pin_configure(p0, 0, GPIO_ACTIVE_LOW | GPIO_PULL_UP);
	int lid_closed = gpio_pin_get(p0, 0);
	printk("Framework 16 - Lid Closed: %d\n", lid_closed);
	/* TODO: To avoid waking the system, keyboard scan should be disabled
	 * if lid is closed
	 **/

	/* Turn backlight brightness to 100% */
	zmk_backlight_on();
	zmk_backlight_set_brt(100);

	zmk_rgb_underglow_on();

	return 0;
}
SYS_INIT(startup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

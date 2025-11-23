int startup(void)
{
	const struct device *p0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

	/* Disable bootloader circuitry */
	gpio_pin_configure(&p0, 5, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(&p0, 5, 0);

	/* Check lid-closed pin (Previously called SLEEP#) */
	gpio_pin_configure(&p0, 0, GPIO_ACTIVE_LOW | GPIO_PULL_UP);
	int lid_closed = gpio_pin_get(&p0, 0);
	printk("Framework 16 - Lid Closed: %d", lid_closed);
	/* TODO: To avoid waking the system, keyboard scan should be disabled
	 * if lid is closed
	 **/

	return 0;
}
SYS_INIT(startup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(daisy_pwm_init, LOG_LEVEL_INF);

/* The pairing-indicator LEDs (pwm_led_w1..w4) are four channels of a single
 * nRF PWM instance, which is one waveform generator sharing a single 4-slot
 * sequence buffer. pwm_nrfx only runs the peripheral when a channel needs
 * partial duty; at 0%/100% it takes a static-GPIO fast path and stops the
 * peripheral. While stopped the pins look independent, but the moment one
 * channel is dimmed the shared sequence starts and loads every slot at once.
 * Channels never touched since boot still hold the driver's init slot value,
 * which decodes to full-on here -- so dimming one LED blasts the others to
 * ~100%.
 *
 * led_pwm never initializes its channels, so prime every channel to 0 once at
 * startup. After that each slot holds a defined "off" value the running
 * sequence honors, and dimming a single channel no longer disturbs the rest. */

#define PAIRING_LEDS_NODE DT_NODELABEL(pwm_led_w1)
#define HAS_PAIRING_LEDS  DT_NODE_EXISTS(PAIRING_LEDS_NODE)

#if HAS_PAIRING_LEDS

#define PWMLEDS_NODE  DT_PARENT(PAIRING_LEDS_NODE)
#define NUM_PWM_LEDS  DT_CHILD_NUM(PWMLEDS_NODE)

static int daisy_pwm_init(void)
{
    const struct device *leds = DEVICE_DT_GET(PWMLEDS_NODE);

    if (!device_is_ready(leds)) {
        LOG_ERR("pwm-leds (%s) not ready", leds->name);
        return 0;
    }

    for (uint32_t ch = 0; ch < NUM_PWM_LEDS; ch++) {
        int ret = led_set_brightness(leds, ch, 0);
        if (ret < 0) {
            LOG_ERR("prime ch%u failed: %d", ch, ret);
        }
    }

    LOG_INF("primed %u %s channels to 0", NUM_PWM_LEDS, leds->name);
    return 0;
}
SYS_INIT(daisy_pwm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* HAS_PAIRING_LEDS */

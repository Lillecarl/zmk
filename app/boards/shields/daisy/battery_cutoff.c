/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Low-battery cutoff.
 *
 * The pack (PACK-GPT-RD.SPC-010123-FT612178PA-1200-FA322 §3.5) specifies a
 * 3.0 V discharge cut-off, and its protection IC (CM1006-DAA) trips
 * over-discharge at 2.92/3.00/3.08 V. Without this module nothing stops the
 * load before that: BUCK1/BUCK2 are both configured for 3.3 V, so once the
 * cell sags past ~3.4 V the bucks run into dropout and the rails simply
 * follow the cell down -- the nRF54LM20A runs to 1.7 V, so the board keeps
 * working right into the pack's own protection trip. Repeatedly bottoming a
 * cell out there is what the pack vendor reported.
 *
 * The nPM1300 cannot do this in hardware on this board. It has no battery
 * UVLO (PS §6.2.2), and its POF comparator -- which *does* have a 3.2 V
 * setting (POFCONFIG.POFVSYSTHRESHSEL = 6) -- only emits a warning on a PMIC
 * GPIO, and GPIO0..GPIO4 are unconnected on daisy_kb_dvt1/dvt2. Worse, an
 * unhandled POF resets the PMIC and reverts the threshold to its 2.8 V
 * default (§7.2 note), i.e. below the pack's own trip, so enabling it blind
 * would give a reset loop rather than a cutoff. Hence: software.
 *
 * Policy, all thresholds Kconfig-tunable:
 *
 *   <= SHED_MV    (3.40 V)  turn the backlight off -- the largest load, and
 *                           shedding it buys real runtime near the floor
 *   <= WARN_MV    (3.35 V)  blink the status LED red
 *   <= CUTOFF_MV  (3.20 V)  for CONSECUTIVE polls -> ship mode
 *
 * The consecutive-sample requirement is load-bearing, not belt-and-braces:
 * VBAT sags hard on BLE TX bursts and with the backlight on, so a single
 * sample below 3.2 V can happen with plenty of charge left. At the default
 * 5 s poll x 3 samples the cell has to hold below the threshold for 15 s
 * before the board turns itself off. (It was 60 s x 5 = 5 minutes; a bench
 * test stepping the supply 3.2 -> 3.0 -> 2.8 V never gave that a chance and
 * a real pack near the floor is discharging into its protection trip the
 * whole time. 15 s still outlasts any TX burst or key-press sag.)
 *
 * VBUS present at any point resets all of it -- a charging pack is not
 * over-discharging, and ship mode is refused while plugged in anyway.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/usb_conn_state_changed.h>

#include "pmic_ship.h"

LOG_MODULE_REGISTER(daisy_battery_cutoff, LOG_LEVEL_INF);

#define HAS_CHARGER DT_NODE_EXISTS(DT_NODELABEL(npm1300_charger))

#if HAS_CHARGER

static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));

/* VBUSINSTATUS bit 0 = VBUS present. */
#define CHG_VBUS_PRESENT BIT(0)

#define CUTOFF_MV      CONFIG_DAISY_BATTERY_CUTOFF_MV
#define WARN_MV        CONFIG_DAISY_BATTERY_WARN_MV
#define SHED_MV        CONFIG_DAISY_BATTERY_BACKLIGHT_SHED_MV
#define HYSTERESIS_MV  CONFIG_DAISY_BATTERY_CUTOFF_HYSTERESIS_MV
#define POLL_SECONDS   CONFIG_DAISY_BATTERY_CUTOFF_POLL_SEC
#define CUTOFF_SAMPLES CONFIG_DAISY_BATTERY_CUTOFF_CONSECUTIVE

BUILD_ASSERT(CUTOFF_MV < WARN_MV, "cutoff must be below the warn threshold");
BUILD_ASSERT(WARN_MV <= SHED_MV, "warn must not be above the backlight-shed threshold");
/* Below the pack's 3.08 V max over-discharge detect the pack protection wins
 * and the point of this module is gone. */
BUILD_ASSERT(CUTOFF_MV > 3080, "cutoff must stay above the pack protection trip (3.08 V max)");

/* ---- status LED (red channel only; charging_led.c owns green/blue) ------- */

#define HAS_RGB_PWM DT_NODE_EXISTS(DT_NODELABEL(pwm_led_r))
#define HAS_RGB_GPIO DT_NODE_EXISTS(DT_NODELABEL(led_r))

#if HAS_RGB_PWM
/* dvt1: partial-duty PWM on the shared pwm21 RGB channels, same duty as
 * charging_led.c so the two indicators look like one system. */
#define RGB_PWM_DUTY 25
static const struct device *const rgb_dev = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(pwm_led_r)));
#define RGB_CH_R DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_r))

static void set_red(bool on) {
    if (device_is_ready(rgb_dev)) {
        led_set_brightness(rgb_dev, RGB_CH_R, on ? RGB_PWM_DUTY : 0);
    }
}
#elif HAS_RGB_GPIO
/* evt: plain GPIO RGB. charging_led.c configures the pin as an output at
 * init; this only sets its level. */
static const struct gpio_dt_spec rgb_r = GPIO_DT_SPEC_GET(DT_NODELABEL(led_r), gpios);

static void set_red(bool on) {
    if (gpio_is_ready_dt(&rgb_r)) {
        gpio_pin_set_dt(&rgb_r, on);
    }
}
#else
static void set_red(bool on) { ARG_UNUSED(on); }
#endif

/* A short flash on a long period: visible without being a meaningful load of
 * its own on a pack that is nearly empty. */
#define WARN_FLASH_ON_MS 120
#define WARN_FLASH_PERIOD_MS 4000

static void warn_blink_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(warn_blink, warn_blink_handler);
static bool warn_lit;

static void warn_blink_handler(struct k_work *work) {
    ARG_UNUSED(work);
    warn_lit = !warn_lit;
    set_red(warn_lit);
    k_work_reschedule(&warn_blink,
                      K_MSEC(warn_lit ? WARN_FLASH_ON_MS
                                      : WARN_FLASH_PERIOD_MS - WARN_FLASH_ON_MS));
}

static bool warning;

static void warn_set(bool on) {
    if (on == warning) {
        return;
    }
    warning = on;
    if (on) {
        warn_lit = false;
        k_work_reschedule(&warn_blink, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&warn_blink);
        warn_lit = false;
        set_red(false);
    }
}

/* ---- backlight ---------------------------------------------------------- */

#define HAS_BACKLIGHT DT_NODE_EXISTS(DT_NODELABEL(backlight_pwms))
#if HAS_BACKLIGHT
static const struct device *const backlight = DEVICE_DT_GET(DT_NODELABEL(backlight_pwms));
#endif

static bool backlight_shed;

/* One-way on purpose: nothing in production firmware drives the backlight
 * automatically yet (it is off at boot; only the factory BACKLIGHT_SET_PWM
 * command turns it on), so there is no previous brightness to restore and
 * guessing one would switch a dark backlight on. Whoever gains ownership of
 * backlight state later should restore it when this module clears the flag. */
static void backlight_shed_set(bool shed) {
    if (shed == backlight_shed) {
        return;
    }
    backlight_shed = shed;
    if (!shed) {
        return;
    }
#if HAS_BACKLIGHT
    if (device_is_ready(backlight)) {
        LOG_INF("shedding backlight");
        led_set_brightness(backlight, 0, 0);
    }
#endif
}

/* ---- poll --------------------------------------------------------------- */

static int cutoff_samples;

static void reset_state(void) {
    cutoff_samples = 0;
    warn_set(false);
    backlight_shed_set(false);
}

static void cutoff_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(cutoff_work, cutoff_work_handler);

/* Runs on the system workqueue. The known sysworkq wedge (daisy.md, "USB
 * unplug wedges the sysworkq") stalls it while USB is unplugged, which is
 * exactly when this module matters -- but only for as long as the USB stack
 * holds it, and a poll that runs late merely delays the cutoff rather than
 * missing it. Ship-mode entry itself, which does have to keep polling through
 * that window, has its own queue (pmic_ship.c). */
static void cutoff_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (daisy_pmic_ship_pending()) {
        /* Already on the way down; stop reading the charger. */
        return;
    }

    struct sensor_value vbat, vbus;
    int rc = device_is_ready(charger) ? sensor_sample_fetch(charger) : -ENODEV;
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus);
    }
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &vbat);
    }
    if (rc != 0) {
        /* Never cut off on a read we don't have. A persistently unreadable
         * charger is its own bug; the pack protection is still the backstop. */
        LOG_WRN("charger read failed: %d; holding", rc);
        cutoff_samples = 0;
        goto resched;
    }

    if (vbus.val1 & CHG_VBUS_PRESENT) {
        reset_state();
        goto resched;
    }

    int32_t vbat_mv = vbat.val1 * 1000 + vbat.val2 / 1000;

    /* Hysteresis on the way back up only: a pack recovers above these
     * thresholds after the load drops, and re-shedding the backlight every
     * time VBAT wobbles across 3.40 V would be worse than not shedding it. */
    if (vbat_mv > SHED_MV + HYSTERESIS_MV) {
        backlight_shed_set(false);
    } else if (vbat_mv <= SHED_MV) {
        backlight_shed_set(true);
    }

    if (vbat_mv > WARN_MV + HYSTERESIS_MV) {
        warn_set(false);
    } else if (vbat_mv <= WARN_MV) {
        warn_set(true);
    }

    if (vbat_mv > CUTOFF_MV) {
        if (cutoff_samples != 0) {
            LOG_INF("VBAT recovered to %d mV; cutoff countdown reset", vbat_mv);
            cutoff_samples = 0;
        }
        goto resched;
    }

    cutoff_samples++;
    LOG_WRN("VBAT %d mV <= cutoff %d mV (%d/%d consecutive)", vbat_mv, CUTOFF_MV, cutoff_samples,
            CUTOFF_SAMPLES);
    if (cutoff_samples < CUTOFF_SAMPLES) {
        goto resched;
    }

    LOG_ERR("VBAT held at/below %d mV for %d s; entering ship mode to stop discharging the pack",
            CUTOFF_MV, CUTOFF_SAMPLES * POLL_SECONDS);
    /* Solid red on the way out, so a user watching sees the reason. It costs
     * nothing: the rail is about to be cut. */
    warn_set(false);
    set_red(true);

    rc = daisy_pmic_ship_request(DAISY_SHIP_REASON_LOW_BATTERY);
    if (rc != 0 && rc != -EALREADY) {
        LOG_ERR("ship request failed: %d; will retry next poll", rc);
        /* Back off one sample so the next poll retries immediately rather
         * than restarting the whole countdown. */
        cutoff_samples = CUTOFF_SAMPLES - 1;
        set_red(false);
        warn_set(true);
        goto resched;
    }
    return;

resched:
    k_work_reschedule(&cutoff_work, K_SECONDS(POLL_SECONDS));
}

/* A plug-in must clear the warning immediately rather than up to a poll
 * period later: charging_led.c flashes its own color on the same LED at that
 * moment, and a stale low-battery blink underneath it reads as a fault. */
static int cutoff_usb_listener(const zmk_event_t *eh) {
    if (!as_zmk_usb_conn_state_changed(eh)) {
        return 0;
    }
    if (!daisy_pmic_ship_pending()) {
        reset_state();
        k_work_reschedule(&cutoff_work, K_SECONDS(POLL_SECONDS));
    }
    return 0;
}

ZMK_LISTENER(daisy_battery_cutoff, cutoff_usb_listener);
ZMK_SUBSCRIPTION(daisy_battery_cutoff, zmk_usb_conn_state_changed);

static int cutoff_init(void) {
    /* One poll period of grace: the charger's first VBAT reading after boot
     * is taken under the inrush of everything else initializing. */
    k_work_reschedule(&cutoff_work, K_SECONDS(POLL_SECONDS));
    LOG_INF("low-battery cutoff armed: shed %d mV, warn %d mV, cutoff %d mV x%d @ %d s", SHED_MV,
            WARN_MV, CUTOFF_MV, CUTOFF_SAMPLES, POLL_SECONDS);
    return 0;
}
SYS_INIT(cutoff_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* HAS_CHARGER */

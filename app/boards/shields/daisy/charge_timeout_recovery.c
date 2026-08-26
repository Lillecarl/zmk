#include <zephyr/device.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

LOG_MODULE_REGISTER(daisy_chg_recovery, LOG_LEVEL_INF);

/*
 * nPM1300 charge safety-timer recovery.
 *
 * The charger's safety timer (tOUTCHARGE, ~7 h) is shorter than a full 0.2C
 * charge of the 1200 mAh pack (~7.3 h; longer through a high-resistance
 * path), so every deep charge latches CHARGETIMEOUT in BCHGERRREASON, the
 * charger hard-disables mid-taper at ~93-95% SOC, and only a VBUS replug
 * would resume it. Analysis + lab captures:
 * aster/issues/charge-stops-early-safety-timer.md.
 *
 * Recovery is reactive and uses only the datasheet-documented error path
 * (PS 4490_483 v1.1 §6.2.7): when CHARGE_TIMEOUT is latched, strobe the
 * error-clear tasks and re-enable charging - the software equivalent of the
 * replug. Charging then restarts a fresh cycle with a fresh hardware timer
 * and, being near-full, terminates normally well inside it.
 *
 * The hardware timer exists to stop a degenerate cell that never reaches
 * the termination current, so extending it needs our own bound: recovery is
 * refused once a VBUS session has accumulated CHARGE_BUDGET_S of active
 * charging (reset on unplug or on normal COMPLETED termination). NTC/JEITA
 * and die-temperature protection are hardware-autonomous and unaffected by
 * any of this; the VBAT/temperature guards below are additional backstops.
 */

#define HAS_PMIC_CHARGER                                                        \
    (DT_NODE_EXISTS(DT_NODELABEL(npm1300)) &&                                   \
     DT_NODE_EXISTS(DT_NODELABEL(npm1300_charger)))

#if HAS_PMIC_CHARGER

static const struct device *const pmic_mfd = DEVICE_DT_GET(DT_NODELABEL(npm1300));
static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));

/* CHARGER task/status registers (nPM1300 product spec §6.2.14). The sensor
 * driver has no API for the safety-timer/error tasks, so they are strobed
 * through the MFD, same as factory_hid.c's clear-error command. */
#define NPM13XX_CHGR_BASE 0x03U
#define CHGR_OFFSET_TASKRELEASEERROR 0x00U
#define CHGR_OFFSET_TASKCLEARCHGERR 0x01U
#define CHGR_OFFSET_TASKCLEARSAFETYTIMER 0x02U
#define CHGR_OFFSET_BCHGENABLESET 0x04U

/* BCHGCHARGESTATUS bits. */
#define CHG_STAT_COMPLETED BIT(1)
#define CHG_STAT_TRICKLE BIT(2)
#define CHG_STAT_CONST_CURRENT BIT(3)
#define CHG_STAT_CONST_VOLTAGE BIT(4)
#define CHG_STAT_ACTIVE (CHG_STAT_TRICKLE | CHG_STAT_CONST_CURRENT | CHG_STAT_CONST_VOLTAGE)

/* BCHGERRREASON bits. */
#define CHG_ERR_CHARGE_TIMEOUT BIT(5)

/* VBUSINSTATUS bits. */
#define CHG_VBUS_PRESENT BIT(0)

#define POLL_SECONDS 60
/* Cumulative active-charging budget per VBUS session. A clean 0.2C full
 * charge needs ~7.3 h; each recovery grants at most one more ~7 h hardware
 * timer period, so 12 h allows the one or two recoveries a real charge needs
 * and still latches a cell that refuses to terminate. */
#define CHARGE_BUDGET_S (12 * 3600)

/* Recovery sanity backstops (the pack spec's charge window is 4.2 V, 0-43 C
 * per the DT thermistor thresholds). */
#define VBAT_MAX_MV 4250
#define NTC_MIN_C 0
#define NTC_MAX_C 43

/* Per-VBUS-session state; only touched from the poll work + the listener. */
static uint32_t charging_seconds;
static uint32_t recoveries;
static bool budget_warned;
static uint8_t last_error;

static void reset_session(void) {
    charging_seconds = 0;
    recoveries = 0;
    budget_warned = false;
    last_error = 0;
}

static int strobe_recovery(void) {
    /* Order per §6.2.7: clear the expired safety timer, wipe the latched
     * error registers, release the charger from the error state, then
     * re-enable charging (matches the driver's own enable path; idempotent). */
    static const uint8_t tasks[] = {
        CHGR_OFFSET_TASKCLEARSAFETYTIMER,
        CHGR_OFFSET_TASKCLEARCHGERR,
        CHGR_OFFSET_TASKRELEASEERROR,
        CHGR_OFFSET_BCHGENABLESET,
    };

    for (size_t i = 0; i < ARRAY_SIZE(tasks); i++) {
        int rc = mfd_npm13xx_reg_write(pmic_mfd, NPM13XX_CHGR_BASE, tasks[i], 1U);
        if (rc != 0) {
            LOG_WRN("recovery write 0x%02x failed: %d", tasks[i], rc);
            return rc;
        }
    }
    return 0;
}

/* Returns false (skip recovery this poll) unless VBAT and NTC readings are
 * available and inside the charge window. */
static bool recovery_guards_pass(void) {
    struct sensor_value val;

    if (sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &val) != 0) {
        LOG_WRN("VBAT read failed; skipping recovery");
        return false;
    }
    int32_t vbat_mv = val.val1 * 1000 + val.val2 / 1000;
    if (vbat_mv >= VBAT_MAX_MV) {
        LOG_WRN("VBAT %d mV out of range; skipping recovery", vbat_mv);
        return false;
    }

    if (sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &val) != 0) {
        LOG_WRN("NTC read failed; skipping recovery");
        return false;
    }
    if (val.val1 < NTC_MIN_C || val.val1 > NTC_MAX_C ||
        (val.val1 == NTC_MIN_C && val.val2 < 0)) {
        LOG_WRN("NTC %d C out of range; skipping recovery", val.val1);
        return false;
    }

    return true;
}

static void recovery_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(recovery_work, recovery_work_handler);

/* Runs on the system workqueue on purpose: the known sysworkq wedge happened
 * only while USB was *unplugged* (see "USB unplug wedges the sysworkq" in
 * daisy.md), which is exactly when this work is cancelled. */
static void recovery_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!zmk_usb_is_powered()) {
        return;
    }

    struct sensor_value status, error, vbus;
    int rc = device_is_ready(charger) ? sensor_sample_fetch(charger) : -ENODEV;
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &status);
    }
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &error);
    }
    if (rc == 0) {
        rc = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus);
    }
    if (rc != 0) {
        LOG_WRN("charger read failed: %d", rc);
        goto resched;
    }

    /* The conn-state event also fires on suspend edges; VBUSINSTATUS is the
     * authoritative "cable is really powering us" signal. */
    if (!(vbus.val1 & CHG_VBUS_PRESENT)) {
        goto resched;
    }

    if (status.val1 & CHG_STAT_ACTIVE) {
        charging_seconds += POLL_SECONDS;
    } else if (status.val1 & CHG_STAT_COMPLETED) {
        if (recoveries > 0) {
            LOG_INF("charge completed after %u recover%s, %u min charging",
                    recoveries, recoveries == 1 ? "y" : "ies", charging_seconds / 60);
        }
        /* Normal termination: maintenance recharge cycles each get a fresh
         * hardware timer, so they get a fresh budget too. */
        reset_session();
    }

    uint8_t err = (uint8_t)error.val1;
    if (err != last_error) {
        if (err & ~CHG_ERR_CHARGE_TIMEOUT) {
            /* Real fault (NTC/VBAT sensor, trickle timeout, ...): never
             * auto-cleared here, only surfaced. */
            LOG_WRN("charger error latched: 0x%02x (status 0x%02x)", err, status.val1);
        }
        last_error = err;
    }

    if (err & CHG_ERR_CHARGE_TIMEOUT) {
        if (charging_seconds >= CHARGE_BUDGET_S) {
            if (!budget_warned) {
                LOG_WRN("charge timeout latched but %u min budget spent; leaving charger off",
                        charging_seconds / 60);
                budget_warned = true;
            }
        } else if (recovery_guards_pass() && strobe_recovery() == 0) {
            recoveries++;
            last_error = 0;
            LOG_INF("charge safety timer expired after %u min charging; cleared and re-enabled (recovery %u)",
                    charging_seconds / 60, recoveries);
        }
    }

resched:
    k_work_reschedule(&recovery_work, K_SECONDS(POLL_SECONDS));
}

static bool session_active;

static void set_session_active(bool active) {
    if (active == session_active) {
        /* Suspend/resume edges re-raise the conn-state event while powered;
         * don't reset the session budget for those. */
        return;
    }
    session_active = active;
    reset_session();
    if (active) {
        k_work_reschedule(&recovery_work, K_SECONDS(POLL_SECONDS));
    } else {
        /* A replug clears the hardware error state on its own. */
        k_work_cancel_delayable(&recovery_work);
    }
}

static int chg_recovery_listener(const zmk_event_t *eh) {
    if (!as_zmk_usb_conn_state_changed(eh)) {
        return 0;
    }
    set_session_active(zmk_usb_is_powered());
    return 0;
}

ZMK_LISTENER(daisy_chg_recovery, chg_recovery_listener);
ZMK_SUBSCRIPTION(daisy_chg_recovery, zmk_usb_conn_state_changed);

static int chg_recovery_init(void) {
    /* A reboot while plugged in raises no conn-state event (and may leave a
     * CHARGE_TIMEOUT already latched from before the reboot). */
    if (zmk_usb_is_powered()) {
        set_session_active(true);
    }
    return 0;
}
SYS_INIT(chg_recovery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* HAS_PMIC_CHARGER */

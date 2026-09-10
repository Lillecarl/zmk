/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * nPM1300 ship-mode entry (battery cutoff).
 *
 * Ship mode (PS 4490_483 v1.1 §7.4) opens the VBAT->VSYS path and drops the
 * PMIC to IQSHIP (370 nA typ). It is the only real cutoff the nPM1300 offers:
 * the part has no battery UVLO of its own (§6.2.2: "An undervoltage lockout
 * circuit is not included on the device and must be set up on the battery
 * pack"), and POF is a warning delivered on a PMIC GPIO -- GPIO0..GPIO4 are
 * unconnected on daisy_kb_dvt1/dvt2, so nothing can receive it.
 *
 * Entry is not a single register write:
 *
 *   - The PMIC ignores the task while VBUS is present and does not latch the
 *     request, so it must be issued after USB is unplugged. The nRF keeps
 *     running from BUCK2 (battery) across the unplug, so this polls.
 *   - §7.4: "When VBUS is not present, the device enters Ship mode
 *     immediately. The host software must wait until EVENTSVBUSIN0SET ... to
 *     ensure VBUS is disconnected and discharged before writing to the
 *     register."
 *   - The strobe can still be refused while residual VBUS discharges, so it
 *     is retried until the PMIC takes it or the overall timeout hits.
 *
 * Every caller shares this one state machine (see pmic_ship.h).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

#include "pmic_ship.h"

LOG_MODULE_REGISTER(daisy_pmic_ship, LOG_LEVEL_INF);

#define HAS_PMIC                                                                \
    (DT_NODE_EXISTS(DT_NODELABEL(npm1300_regulators)) &&                        \
     DT_NODE_EXISTS(DT_NODELABEL(npm1300)))
#define HAS_CHARGER DT_NODE_EXISTS(DT_NODELABEL(npm1300_charger))

#if HAS_PMIC

/* regulator_parent_ship_mode() is the supported way in (per Nordic); it
 * strobes SHIP.TASKENTERSHIPMODE. The MFD itself is needed for the VBUS event
 * registers the entry sequence gates on. */
static const struct device *const pmic_regulators =
    DEVICE_DT_GET(DT_NODELABEL(npm1300_regulators));
static const struct device *const pmic_mfd = DEVICE_DT_GET(DT_NODELABEL(npm1300));
#if HAS_CHARGER
static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));
#endif

/* MAIN.EVENTSVBUSIN0SET / ...CLR (base 0x00, offsets 0x16/0x17): bit1 latches
 * on "VBUS removed", the "disconnected and discharged" indicator §7.4 wants
 * before the ship task. */
#define NPM13XX_MAIN_BASE 0x00U
#define MAIN_OFFSET_EVENTSVBUSIN0SET 0x16U
#define MAIN_OFFSET_EVENTSVBUSIN0CLR 0x17U
#define EVENT_VBUS_REMOVED 0x02U
#define EVENTSVBUSIN_ALL 0x3FU

/* VBUSINSTATUS bit 0 = VBUS present. */
#define CHG_VBUS_PRESENT 0x01U

/* Give the caller time to finish (ack a factory report, flush a log line)
 * before power drops. */
#define SHIP_ENTER_DELAY_MS 250
#define SHIP_POLL_INTERVAL_MS 200
/* Stop polling eventually so a request that can never be satisfied (user never
 * unplugs) doesn't spin forever. Generous: an operator may take a while to get
 * to the cable. */
#define SHIP_POLL_TIMEOUT_MS (5 * 60 * 1000)
#define SHIP_POLL_MAX_ATTEMPTS (SHIP_POLL_TIMEOUT_MS / SHIP_POLL_INTERVAL_MS)
/* Fallback for the removed-event gate below. */
#define SHIP_VBUS_SETTLE_MS 1000
#define SHIP_VBUS_SETTLE_POLLS (SHIP_VBUS_SETTLE_MS / SHIP_POLL_INTERVAL_MS)

static void ship_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ship_work, ship_work_handler);

/* Dedicated queue: on USB unplug a USB-stack work item blocks the system
 * workqueue for as long as VBUS is absent (observed: a ship poll due 200 ms
 * after unplug ran 2 minutes later, at replug, together with the delayed USB
 * teardown logs -- see daisy.md "USB unplug wedges the sysworkq"). Ship entry
 * must keep polling exactly during that window, so it cannot share the system
 * workqueue. Cooperative priority so USB churn can't starve it. */
static K_THREAD_STACK_DEFINE(ship_q_stack, 1024);
static struct k_work_q ship_q;

/* Poll counters are touched only from the ship_q thread. `ship_armed` is the
 * handoff, and is claimed atomically: the factory command and the low-battery
 * cutoff run on different threads, and two of them getting past the check
 * would put two pollers on one PMIC. */
static int ship_poll_attempts;
static int ship_vbus_clear_polls;
static enum daisy_ship_reason ship_reason;
static atomic_t ship_armed;

static const char *reason_str(enum daisy_ship_reason reason) {
    switch (reason) {
    case DAISY_SHIP_REASON_FACTORY:
        return "factory command";
    case DAISY_SHIP_REASON_LOW_BATTERY:
        return "low battery";
    default:
        return "unknown";
    }
}

/* Returns true if VBUS is currently present (USB plugged in). Without the
 * charger we cannot tell, so report "present" and keep waiting rather than
 * cutting power while possibly still plugged in. */
static bool ship_vbus_present(void) {
#if HAS_CHARGER
    if (!device_is_ready(charger)) {
        return true;
    }
    struct sensor_value vbus;
    int err = sensor_sample_fetch(charger);
    if (err == 0) {
        err = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus);
    }
    if (err != 0) {
        LOG_WRN("ship poll %d: VBUS status read failed: %d", ship_poll_attempts, err);
        return true;
    }
    LOG_DBG("ship poll %d: VBUSINSTATUS=0x%02x", ship_poll_attempts, (uint8_t)vbus.val1);
    return (vbus.val1 & CHG_VBUS_PRESENT) != 0;
#else
    return true;
#endif
}

static void ship_abandon(const char *why) {
    LOG_WRN("ship mode (%s) abandoned: %s", reason_str(ship_reason), why);
    atomic_clear(&ship_armed);
}

static void ship_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!device_is_ready(pmic_regulators)) {
        ship_abandon("regulators not ready");
        return;
    }

    if (ship_vbus_present()) {
        ship_vbus_clear_polls = 0;
        if (++ship_poll_attempts >= SHIP_POLL_MAX_ATTEMPTS) {
            ship_abandon("VBUS still present at timeout");
            return;
        }
        k_work_schedule_for_queue(&ship_q, &ship_work, K_MSEC(SHIP_POLL_INTERVAL_MS));
        return;
    }
    ship_vbus_clear_polls++;

    /* The latched removed-event is the precise "disconnected and discharged"
     * signal §7.4 asks for, and it latches within a poll or two of a real
     * unplug -- which is the factory path. It is NOT sufficient on its own:
     * a unit that has run on battery since boot may never have seen VBUS, so
     * the event never latches and waiting on it alone would silently abandon
     * every low-battery cutoff. Fall back to a settle window -- VBUS cannot
     * still be discharging after SHIP_VBUS_SETTLE_MS of reading as absent. */
    uint8_t events = 0;
    int err = mfd_npm13xx_reg_read(pmic_mfd, NPM13XX_MAIN_BASE, MAIN_OFFSET_EVENTSVBUSIN0SET,
                                   &events);
    if (err != 0) {
        LOG_ERR("ship events read failed: %d", err);
        goto rearm;
    }
    if (!(events & EVENT_VBUS_REMOVED) && ship_vbus_clear_polls < SHIP_VBUS_SETTLE_POLLS) {
        LOG_DBG("ship poll %d: VBUS clear but no removed event yet (0x%02x)", ship_poll_attempts,
                events);
        goto rearm;
    }

    err = mfd_npm13xx_reg_write(pmic_mfd, NPM13XX_MAIN_BASE, MAIN_OFFSET_EVENTSVBUSIN0CLR,
                                EVENTSVBUSIN_ALL);
    if (err != 0) {
        LOG_ERR("ship events clear failed: %d", err);
        goto rearm;
    }

    /* Ship mode, not hibernate: hibernate always arms the wake-up timer and
     * would reboot the board instead of leaving it off until SHPHLD/VBUS.
     * (On this board SHPHLD reaches only nRF P3.02 through Q4 and a test pad,
     * and the nRF is unpowered in ship mode -- so recovery is a USB plug-in,
     * which is what you were going to do with a flat pack anyway.)
     *
     * If we're still running next poll the PMIC refused (residual VBUS);
     * strobe again until it accepts or the overall timeout hits. */
    LOG_INF("ship strobe %d (%s): TASKENTERSHIPMODE (events were 0x%02x)", ship_poll_attempts,
            reason_str(ship_reason), events);
    err = regulator_parent_ship_mode(pmic_regulators);
    if (err != 0) {
        LOG_ERR("ship mode entry failed: %d", err);
        ship_abandon("ship task write failed");
        return;
    }

rearm:
    if (++ship_poll_attempts < SHIP_POLL_MAX_ATTEMPTS) {
        k_work_schedule_for_queue(&ship_q, &ship_work, K_MSEC(SHIP_POLL_INTERVAL_MS));
    } else {
        ship_abandon("kept running past timeout");
    }
}

int daisy_pmic_ship_request(enum daisy_ship_reason reason) {
    if (!device_is_ready(pmic_regulators) || !device_is_ready(pmic_mfd)) {
        return -ENODEV;
    }
    if (!atomic_cas(&ship_armed, 0, 1)) {
        /* Both reasons end in the same place; don't restart the poll loop or
         * two callers could interleave strobes. */
        LOG_INF("ship mode (%s) already armed; ignoring %s", reason_str(ship_reason),
                reason_str(reason));
        return -EALREADY;
    }

    ship_reason = reason;
    ship_poll_attempts = 0;
    ship_vbus_clear_polls = 0;
    LOG_INF("ship mode armed: %s", reason_str(reason));
    k_work_schedule_for_queue(&ship_q, &ship_work, K_MSEC(SHIP_ENTER_DELAY_MS));
    return 0;
}

bool daisy_pmic_ship_pending(void) { return atomic_get(&ship_armed) != 0; }

static int ship_q_init(void) {
    k_work_queue_start(&ship_q, ship_q_stack, K_THREAD_STACK_SIZEOF(ship_q_stack), K_PRIO_COOP(7),
                       NULL);
    k_thread_name_set(k_work_queue_thread_get(&ship_q), "daisy_ship");
    return 0;
}
/* PRE_KERNEL is too early for a work queue and APPLICATION would race the
 * factory HID init that used to start this queue itself; POST_KERNEL at the
 * default app priority puts it before both. */
SYS_INIT(ship_q_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

#else /* !HAS_PMIC */

int daisy_pmic_ship_request(enum daisy_ship_reason reason) {
    ARG_UNUSED(reason);
    return -ENOTSUP;
}

bool daisy_pmic_ship_pending(void) { return false; }

#endif /* HAS_PMIC */

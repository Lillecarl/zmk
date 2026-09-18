/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Touchpad firmware convergence.
 *
 * The keyboard image carries one PixArt PJP360 firmware release
 * (touchpad_firmware/, selected by CONFIG_DAISY_TOUCHPAD_FW_FILE and turned
 * into daisy_touchpad_fw_image at build time). A few seconds after boot we
 * read the pad's version and flash it when it differs from the embedded one
 * in either direction, so pad firmware is a pure function of keyboard
 * firmware. A pad running from ROM (its flash image invalid, e.g. a power
 * loss mid-update) is always recovered regardless of the power gate, since
 * it is useless as-is.
 *
 * Runs on its own cooperative work queue, never the sysworkq: the update
 * holds the pad for seconds. State is announced over the event bus
 * (daisy_touchpad_update_state_changed) so touchpad_power.c can suspend its
 * host-follow and LEDs can show progress without another shared header.
 *
 * See aster plans/touchpad-firmware-self-update.md.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zmk/event_manager.h>
#include <zmk/usb.h>
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
#include <zmk/battery.h>
#endif

#include <pixart_tp_update.h>

#include "touchpad_update.h"
#include "touchpad_update_state_changed.h"

#if IS_ENABLED(CONFIG_DAISY_FACTORY)
#include "factory_state.h"
#else
static inline bool daisy_factory_mode_active(void) { return false; }
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daisy_tp_update, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(daisy_touchpad_update_state_changed);

/* daisy_touchpad_fw_image: generated from the vendor file by
 * touchpad_firmware/gen_tp_fw.py at build time (see CMakeLists.txt). */
#include "tp_fw_blob.inc"

#if DT_NODE_HAS_STATUS(DT_NODELABEL(touchpad), okay)

static const struct device *const touchpad = DEVICE_DT_GET(DT_NODELABEL(touchpad));

/* Past touchpad_power.c's 2 s boot follow-pass and USB enumeration, and past
 * the first battery sample, so the power gate sees real numbers. */
#define BOOT_CHECK_DELAY_MS 3000
/* Automatic attempts per power cycle. A pad that fails twice in a row is
 * left in ROM mode for the next boot rather than looped on. */
#define MAX_AUTO_ATTEMPTS 2

static K_THREAD_STACK_DEFINE(update_q_stack, 3072);
static struct k_work_q update_q;

static struct daisy_touchpad_update_status status = {
    .state = DAISY_TP_UPDATE_UNKNOWN,
};
static struct k_mutex status_lock;

static void set_state(enum daisy_touchpad_update_state state, int err) {
    k_mutex_lock(&status_lock, K_FOREVER);
    status.state = state;
    status.last_err = err;
    if (state != DAISY_TP_UPDATE_RUNNING) {
        status.progress_pct = 0;
    }
    k_mutex_unlock(&status_lock);

    struct daisy_touchpad_update_state_changed ev = {
        .state = state,
        .running = state == DAISY_TP_UPDATE_RUNNING,
    };
    raise_daisy_touchpad_update_state_changed(ev);
}

static void record_info(const struct pixart_tp_info *info) {
    k_mutex_lock(&status_lock, K_FOREVER);
    status.info_valid = true;
    status.pad_version = info->version;
    status.pad_part_id = info->part_id;
    status.pad_boot_status = info->boot_status;
    k_mutex_unlock(&status_lock);
}

static void progress_cb(enum pixart_tp_update_stage stage, unsigned int done, unsigned int total,
                        void *user) {
    if (stage == PIXART_TP_STAGE_PROGRAM && total) {
        k_mutex_lock(&status_lock, K_FOREVER);
        status.progress_pct = (uint8_t)(done * 100 / total);
        k_mutex_unlock(&status_lock);
    }
}

static bool power_ok(void) {
    if (zmk_usb_is_powered()) {
        return true;
    }
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    return zmk_battery_state_of_charge() >= CONFIG_DAISY_TOUCHPAD_UPDATE_MIN_SOC;
#else
    return false;
#endif
}

struct update_req {
    struct k_work_delayable work;
    bool automatic; /* boot check: honour factory/power gates and attempt cap */
    bool force;     /* flash even when versions match */
};

static void update_work(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct update_req *req = CONTAINER_OF(dw, struct update_req, work);
    const struct pixart_tp_fw_image *img = &daisy_touchpad_fw_image;
    struct pixart_tp_info info;

    if (!device_is_ready(touchpad)) {
        LOG_WRN("touchpad not ready, skipping firmware check");
        set_state(DAISY_TP_UPDATE_FAILED, -ENODEV);
        return;
    }
    if (req->automatic && daisy_factory_mode_active()) {
        LOG_INF("factory mode active, touchpad firmware check skipped");
        set_state(DAISY_TP_UPDATE_SKIPPED_FACTORY, 0);
        return;
    }

    int err = pixart_tp_read_info(touchpad, &info);
    if (err) {
        LOG_ERR("cannot read touchpad info: %d", err);
        set_state(DAISY_TP_UPDATE_FAILED, err);
        return;
    }
    record_info(&info);
    LOG_INF("touchpad part 0x%04x boot 0x%02x fw 0x%04x, embedded 0x%04x", info.part_id,
            info.boot_status, info.version, img->version);

    if (info.part_id != img->part_id) {
        LOG_ERR("touchpad part 0x%04x, embedded image is for 0x%04x", info.part_id, img->part_id);
        set_state(DAISY_TP_UPDATE_UNSUPPORTED, -ENODEV);
        return;
    }
    if (info.boot_status == PIXART_TP_BOOT_STATUS_FLASHLESS) {
        LOG_WRN("flashless touchpad, never updated");
        set_state(DAISY_TP_UPDATE_UNSUPPORTED, -ENOTSUP);
        return;
    }

    const bool in_rom = info.boot_status == PIXART_TP_BOOT_STATUS_ROM;
    if (!req->force && !in_rom && info.version == img->version) {
        set_state(DAISY_TP_UPDATE_IN_SYNC, 0);
        return;
    }
    if (req->automatic) {
        if (!in_rom && !power_ok()) {
            LOG_INF("touchpad fw 0x%04x != 0x%04x but on battery below %d%%, deferred",
                    info.version, img->version, CONFIG_DAISY_TOUCHPAD_UPDATE_MIN_SOC);
            set_state(DAISY_TP_UPDATE_SKIPPED_POWER, 0);
            return;
        }
        if (status.attempts >= MAX_AUTO_ATTEMPTS) {
            LOG_ERR("touchpad update failed %u times this power cycle, giving up", status.attempts);
            return;
        }
    }

    LOG_INF("updating touchpad firmware 0x%04x -> 0x%04x%s", info.version, img->version,
            in_rom ? " (pad in ROM mode)" : "");
    set_state(DAISY_TP_UPDATE_RUNNING, 0);
    err = pixart_tp_update(touchpad, img, progress_cb, NULL);
    k_mutex_lock(&status_lock, K_FOREVER);
    status.attempts++;
    k_mutex_unlock(&status_lock);

    if (pixart_tp_read_info(touchpad, &info) == 0) {
        record_info(&info);
    }
    if (err) {
        LOG_ERR("touchpad update failed: %d", err);
        set_state(DAISY_TP_UPDATE_FAILED, err);
    } else {
        LOG_INF("touchpad now at fw 0x%04x", info.version);
        set_state(DAISY_TP_UPDATE_DONE, 0);
    }
}

static struct update_req boot_req = {.automatic = true};
static struct update_req manual_req = {.automatic = false};

void daisy_touchpad_update_get_status(struct daisy_touchpad_update_status *out) {
    k_mutex_lock(&status_lock, K_FOREVER);
    *out = status;
    out->embedded_version = daisy_touchpad_fw_image.version;
    k_mutex_unlock(&status_lock);
}

int daisy_touchpad_update_request(bool force) {
    if (status.state == DAISY_TP_UPDATE_RUNNING) {
        return -EBUSY;
    }
    manual_req.force = force;
    /* Cancel a not-yet-fired boot check so the two don't run back to back. */
    k_work_cancel_delayable(&boot_req.work);
    k_work_reschedule_for_queue(&update_q, &manual_req.work, K_NO_WAIT);
    return 0;
}

static int touchpad_update_init(void) {
    k_mutex_init(&status_lock);
    k_work_init_delayable(&boot_req.work, update_work);
    k_work_init_delayable(&manual_req.work, update_work);
    k_work_queue_start(&update_q, update_q_stack, K_THREAD_STACK_SIZEOF(update_q_stack),
                       K_PRIO_COOP(7), NULL);
    k_thread_name_set(&update_q.thread, "tp_update");
    k_work_schedule_for_queue(&update_q, &boot_req.work, K_MSEC(BOOT_CHECK_DELAY_MS));
    return 0;
}

SYS_INIT(touchpad_update_init, APPLICATION, 99);

#else /* no touchpad node */

void daisy_touchpad_update_get_status(struct daisy_touchpad_update_status *out) {
    *out = (struct daisy_touchpad_update_status){
        .state = DAISY_TP_UPDATE_UNSUPPORTED,
        .last_err = -ENODEV,
        .embedded_version = daisy_touchpad_fw_image.version,
    };
}

int daisy_touchpad_update_request(bool force) { return -ENODEV; }

#endif

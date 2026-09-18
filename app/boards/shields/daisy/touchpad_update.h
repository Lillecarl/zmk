/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Boot-time convergence of the touchpad's firmware to the release embedded
 * in the keyboard image (touchpad_update.c). This header is the C-side
 * status/trigger API for the factory protocol (TOUCHPAD_FW_STATUS /
 * TOUCHPAD_FW_UPDATE); everything else learns about the updater through
 * the daisy_touchpad_update_state_changed event.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Wire values of TOUCHPAD_FW_STATUS byte 0; append only. */
enum daisy_touchpad_update_state {
    DAISY_TP_UPDATE_UNKNOWN = 0,         /* boot check has not run yet */
    DAISY_TP_UPDATE_IN_SYNC = 1,         /* pad already runs the embedded version */
    DAISY_TP_UPDATE_SKIPPED_FACTORY = 2, /* factory mode owns the pad */
    DAISY_TP_UPDATE_SKIPPED_POWER = 3,   /* on battery below the SOC gate */
    DAISY_TP_UPDATE_RUNNING = 4,
    DAISY_TP_UPDATE_DONE = 5,        /* flashed and verified this power cycle */
    DAISY_TP_UPDATE_FAILED = 6,      /* see last_err; pad may be in ROM mode */
    DAISY_TP_UPDATE_UNSUPPORTED = 7, /* flashless pad or wrong part ID */
};

struct daisy_touchpad_update_status {
    enum daisy_touchpad_update_state state;
    int last_err;         /* errno of the last failure, 0 otherwise */
    uint8_t attempts;     /* flash attempts this power cycle */
    uint8_t progress_pct; /* PROGRAM stage progress while running */
    uint32_t last_duration_ms; /* wall time of the last flash attempt, 0 if none */
    uint16_t embedded_version;
    /* Last successful read of the pad; valid when info_valid. */
    bool info_valid;
    uint16_t pad_version;
    uint16_t pad_part_id;
    uint8_t pad_boot_status;
};

void daisy_touchpad_update_get_status(struct daisy_touchpad_update_status *out);

/**
 * Queue an update now, bypassing factory-mode and power gates. With @p force
 * the pad is flashed even when its version already matches.
 * @return 0, or -EBUSY while an update is running.
 */
int daisy_touchpad_update_request(bool force);

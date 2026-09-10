/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>

/* nPM1300 ship-mode entry (battery cutoff), shared by every caller that needs
 * to cut the board's power: the factory SHIP_MODE command and the low-battery
 * cutoff in battery_cutoff.c. Both must go through here rather than strobing
 * SHIP.TASKENTERSHIPMODE themselves -- entry is a poll loop with real
 * preconditions (see pmic_ship.c), and two of them racing on the same PMIC
 * would interleave their strobes. */

enum daisy_ship_reason {
    /* Factory SHIP_MODE command: the unit is being put in its box. */
    DAISY_SHIP_REASON_FACTORY,
    /* VBAT hit the cutoff threshold; stop discharging the pack. */
    DAISY_SHIP_REASON_LOW_BATTERY,
};

/* Arm ship-mode entry. Returns immediately: the entry itself is deferred (it
 * powers the board off, so callers need to finish what they were doing first)
 * and then polls until the PMIC will accept it.
 *
 * 0 on success, -ENOTSUP on a target with no PMIC (dev kits), -ENODEV if the
 * PMIC is present but not ready, -EALREADY if entry is already armed (the
 * first reason wins; both end in the same place). */
int daisy_pmic_ship_request(enum daisy_ship_reason reason);

/* True once a request has been armed and not yet abandoned. */
bool daisy_pmic_ship_pending(void);

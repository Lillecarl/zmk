/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>

enum zmk_activity_state { ZMK_ACTIVITY_ACTIVE, ZMK_ACTIVITY_IDLE, ZMK_ACTIVITY_SLEEP };

enum zmk_activity_state zmk_activity_get_state(void);

/* Weakly defined (returning false) in activity.c; another module can provide a
 * strong definition to keep the keyboard from entering deep sleep, e.g. the
 * daisy factory HID interface while a factory test run drives the device. */
bool zmk_sleep_inhibited(void);

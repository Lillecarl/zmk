/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Raised by touchpad_update.c when the boot-time touchpad firmware
 * convergence changes state. While `running` the pad is claimed for seconds
 * and every other driver call fails with -EBUSY; modules with their own pad
 * policy (touchpad_power.c host-follow, LEDs) stand down on running=true and
 * re-converge on running=false, without calling into the updater directly.
 * Only defined when CONFIG_DAISY_TOUCHPAD_UPDATE=y.
 */

#pragma once

#include <stdbool.h>
#include <zmk/event_manager.h>

#include "touchpad_update.h"

struct daisy_touchpad_update_state_changed {
    enum daisy_touchpad_update_state state;
    bool running;
};

ZMK_EVENT_DECLARE(daisy_touchpad_update_state_changed);

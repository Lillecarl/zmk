/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Raised by factory_hid.c whenever factory mode is entered or left
 * (DAISY_FACTORY_CMD_FACTORY_MODE_SET). Modules that suspend their own
 * policy while the factory tool has control (touchpad power, LEDs) subscribe
 * to this to re-converge on exit, rather than being called into directly.
 * Only defined when CONFIG_DAISY_FACTORY=y (factory_hid.c owns the impl).
 */

#pragma once

#include <stdbool.h>
#include <zmk/event_manager.h>

struct daisy_factory_mode_changed {
    bool active;
};

ZMK_EVENT_DECLARE(daisy_factory_mode_changed);

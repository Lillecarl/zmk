/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Runtime factory-mode state, shared across the daisy shield's board glue.
 *
 * This is the C-side query, distinct from factory.h (which is the on-the-wire
 * protocol contract). Defined in factory_hid.c and compiled only when
 * CONFIG_DAISY_FACTORY=y; callers in always-compiled files should fall back to
 * a constant false when the factory interface isn't built (see the LED
 * modules).
 */

#ifndef DAISY_FACTORY_STATE_H
#define DAISY_FACTORY_STATE_H

#include <stdbool.h>

/* True while the keyboard is in factory mode (DAISY_FACTORY_CMD_FACTORY_MODE_SET).
 * Board indicators (charging/pairing LEDs) must relinquish the LEDs while this
 * is set so the factory tool has exclusive control of them. */
bool daisy_factory_mode_active(void);

#endif /* DAISY_FACTORY_STATE_H */

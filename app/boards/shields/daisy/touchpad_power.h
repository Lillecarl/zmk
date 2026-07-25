/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Touchpad power control for the daisy shield.
 *
 * One entry point shared by the two writers of the pad's power state: the
 * factory interface (DAISY_FACTORY_CMD_TOUCHPAD_POWER_SET in factory_hid.c)
 * and the automatic host-follow logic in touchpad_power.c (USB suspend/BLE
 * link state -> standby). State values mirror enum
 * daisy_factory_touchpad_power in factory.h -- same numbers on purpose.
 */

#ifndef DAISY_TOUCHPAD_POWER_H
#define DAISY_TOUCHPAD_POWER_H

#include <stdbool.h>

enum daisy_touchpad_power {
    DAISY_TOUCHPAD_POWER_OFF = 0,   /* standby + passthrough quiesced */
    DAISY_TOUCHPAD_POWER_ON = 1,    /* run: full operation */
    DAISY_TOUCHPAD_POWER_SLEEP = 2, /* modern standby: touch-detect only */
};

/* Apply a touchpad power state. All states use the HID-I2C SET_POWER command
 * (the PCT1036's register-flow suspend is a no-op in HID-I2C mode); OFF also
 * suspends the passthrough driver so no reports flow. Idempotent. Returns 0
 * or a negative errno (-ENODEV: driver not ready, -EINVAL: bad state, else
 * the I2C/PM error). */
int daisy_touchpad_power_set(enum daisy_touchpad_power state);

/* User-level enable/disable, as bound to the Win-held E+T / D+T chords
 * (combo behavior in behavior_touchpad_power.c). ANDed with the automatic
 * host-follow: disabling turns the pad off right away, enabling hands it
 * back to host-follow control. Not persisted across reboots. */
void daisy_touchpad_user_set_enabled(bool enabled);

#endif /* DAISY_TOUCHPAD_POWER_H */

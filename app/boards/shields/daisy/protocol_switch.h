/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Physical USB/BT mode switch state, shared across the daisy shield's board
 * glue. Defined in protocol_switch.c, which owns the debounced reading (boot
 * GPIO level + gpio-keys input events) and the policy it drives.
 *
 * This is the switch position only -- deliberately not the same question as
 * "is BLE advertising permitted", which the factory interface and
 * `aster --bt-adv-off` can also answer (see zmk_ble_adv_enabled_get()).
 */

#ifndef DAISY_PROTOCOL_SWITCH_H
#define DAISY_PROTOCOL_SWITCH_H

#include <stdbool.h>

#include <zephyr/devicetree.h>

#if DT_NODE_EXISTS(DT_NODELABEL(protocol_switch))

/* True while the switch sits in the WIRELESS position, false for WIRED. */
bool daisy_protocol_switch_is_wireless(void);

#else

/* No switch on this build -> nothing is holding the keyboard in wired mode. */
static inline bool daisy_protocol_switch_is_wireless(void) { return true; }

#endif

#endif /* DAISY_PROTOCOL_SWITCH_H */

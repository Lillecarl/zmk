/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/keys.h>
#include <zmk/hid.h>

enum zmk_usb_conn_state {
    ZMK_USB_CONN_NONE,
    ZMK_USB_CONN_POWERED,
    ZMK_USB_CONN_HID,
};

enum zmk_usb_conn_state zmk_usb_get_conn_state(void);

static inline bool zmk_usb_is_powered(void) {
    return zmk_usb_get_conn_state() != ZMK_USB_CONN_NONE;
}
bool zmk_usb_is_hid_ready(void);

/* Request USB remote wakeup: ask a suspended host to resume the bus so an input
 * report (keypress, pointer motion) can wake a sleeping computer. A no-op
 * returning 0 when the bus isn't suspended; when it is, forwards to
 * usbd_wakeup_request(), which only signals if the host armed remote wakeup and
 * the controller supports it. Returns 0 on success or a negative errno. */
int zmk_usb_wakeup_request(void);

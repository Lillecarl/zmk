/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Daisy factory-test wire protocol.
 *
 * This header is the SINGLE SOURCE OF TRUTH for the on-the-wire contract
 * between the Daisy keyboard firmware and the `aster` factory tool. It is
 * deliberately written with only fixed-width integers and plain #defines so it
 * can be transcribed (or codegen'd) into the Rust host side without change.
 *
 * Transport: a vendor-defined USB HID interface (usage page 0xFF60, usage
 * 0x61 -- the QMK "raw HID" convention, so the OS HID class driver claims it
 * with no custom driver on Windows/Linux/macOS). All reports are a fixed
 * DAISY_FACTORY_REPORT_SIZE bytes with no report ID. The host writes an Output
 * report (a request frame); the device answers with an Input report (a
 * response frame) carrying the same `seq`.
 *
 * Versioning: the host calls CORE_INFO first and checks `proto_version`. New
 * commands may be added within a protocol version; the frame layout and the
 * meaning of existing commands never change within a version. Breaking changes
 * bump DAISY_FACTORY_PROTO_VERSION.
 */

#ifndef DAISY_FACTORY_H
#define DAISY_FACTORY_H

#include <stdint.h>

/* Bumped only on a breaking change to the frame layout or existing command
 * semantics. Adding a new command does NOT bump this. */
#define DAISY_FACTORY_PROTO_VERSION 1

/* Fixed HID report size, in bytes. Matches QMK raw HID. */
#define DAISY_FACTORY_REPORT_SIZE 32

/* Frame header is 5 bytes; the rest is payload. */
#define DAISY_FACTORY_HEADER_SIZE 5
#define DAISY_FACTORY_PAYLOAD_SIZE (DAISY_FACTORY_REPORT_SIZE - DAISY_FACTORY_HEADER_SIZE)

/* Byte offsets within a frame (request and response share this layout). */
#define DAISY_FACTORY_OFF_CMD 0    /* opcode (daisy_factory_cmd) */
#define DAISY_FACTORY_OFF_SEQ 1     /* transaction id, echoed in the response */
#define DAISY_FACTORY_OFF_FLAGS 2   /* bitmask, see DAISY_FACTORY_FLAG_* */
#define DAISY_FACTORY_OFF_STATUS 3  /* daisy_factory_status (responses only) */
#define DAISY_FACTORY_OFF_LEN 4     /* payload bytes present in THIS frame */
#define DAISY_FACTORY_OFF_PAYLOAD 5

/* Frame flags. */
#define DAISY_FACTORY_FLAG_RESPONSE (1u << 0) /* set by device on replies */
#define DAISY_FACTORY_FLAG_MORE (1u << 1)     /* more frames follow (chunking) */
#define DAISY_FACTORY_FLAG_ERROR (1u << 2)    /* status holds an error code */

/* Opcodes, encoded as (group << 4) | action so they read like the aster CLI
 * namespaces and stay sparse for future additions. */
enum daisy_factory_cmd {
    /* group 0x0: core / transport */
    DAISY_FACTORY_CMD_INFO = 0x00, /* -> struct daisy_factory_info */
    DAISY_FACTORY_CMD_PING = 0x01, /* payload echoed back verbatim */
    /* Read the SoC's unique hardware device id (nRF FICR->INFO.DEVICEID, via
     * the Zephyr hwinfo driver). No payload in; out: the raw id bytes (8 on
     * nRF54), big-endian as hwinfo returns them (DEVICEID[1] first). Permanent
     * and unique per chip; independent of any writable serial number. */
    DAISY_FACTORY_CMD_DEVICE_ID = 0x02,

    /* group 0x1: battery (read-only) */
    DAISY_FACTORY_CMD_BATTERY_TEMP = 0x11,    /* -> i16 hundredths of degC, LE */
    DAISY_FACTORY_CMD_BATTERY_VOLTAGE = 0x12, /* -> u16 millivolts, LE */
    DAISY_FACTORY_CMD_BATTERY_CURRENT = 0x13, /* -> i16 milliamps (signed: + charge), LE */

    /* group 0x2: charging */
    DAISY_FACTORY_CMD_CHARGING_SET = 0x20, /* payload: [enable u8 (0=stop, 1=start)] */

    /* group 0x3: LEDs */
    DAISY_FACTORY_CMD_LED_SET_PWM = 0x30,      /* payload: [index u8][percent u8 0-100] */
    DAISY_FACTORY_CMD_BACKLIGHT_SET_PWM = 0x31, /* payload: [percent u8 0-100] */
    /* payload: [color u8 (daisy_factory_rgb)][percent u8 0-100]. On boards
     * where the status RGB is GPIO-only (EVT), any nonzero percent is "on". */
    DAISY_FACTORY_CMD_RGB_SET_PWM = 0x32,
    /* payload: [on u8 (0=off, 1=on)]. Drives the caps lock LED GPIO directly;
     * note the firmware's HID-indicator listener will overwrite this state on
     * the next caps-lock change from the host. */
    DAISY_FACTORY_CMD_CAPS_LED_SET = 0x33,

    /* group 0x5: power */
    DAISY_FACTORY_CMD_SHIP_MODE = 0x50, /* enter nPM1300 ship mode shortly after acking */
    /* payload: [enable u8 (0=exit, 1=enter)]. While enabled the keyboard never
     * enters deep sleep, so it stays reachable for the whole test run even
     * with USB unplugged. RAM-only: a reboot or power loss exits the mode. */
    DAISY_FACTORY_CMD_FACTORY_MODE_SET = 0x51,
    /* Reboot into the MCUboot serial-recovery bootloader (enumerates as a
     * CDC-ACM "Recovery" device for mcumgr). Acks first, then reboots shortly
     * after so the response reaches the host. No payload. */
    DAISY_FACTORY_CMD_BOOTLOADER_JUMP = 0x52,

    /* group 0x4: GPIO / straps (read-only) */
    DAISY_FACTORY_CMD_GPIO_GET = 0x40, /* in: [gpio_id u8]; out: [level u8 0/1] */

    /* group 0x6: bluetooth */
    /* Clear all BLE bonds: every pairing profile is unpaired (persisted
     * immediately) and the keyboard switches back to profile 0. No payload. */
    DAISY_FACTORY_CMD_BT_CLEAR_BONDS = 0x60,
    /* Unpair the ACTIVE profile only (persisted immediately); the profile
     * starts advertising as open again. No payload. */
    DAISY_FACTORY_CMD_BT_UNPAIR = 0x61,
    /* Switch the active BLE profile. payload: [index u8 0..profile_count-1].
     * BAD_ARG if the index is out of range. */
    DAISY_FACTORY_CMD_BT_PROF_SELECT = 0x62,
    DAISY_FACTORY_CMD_BT_PROF_NEXT = 0x63, /* cycle to the next profile (wraps) */
    DAISY_FACTORY_CMD_BT_PROF_PREV = 0x64, /* cycle to the previous profile (wraps) */
    /* -> struct daisy_factory_bt_status. Per-profile masks are bit N ==
     * profile N. "Bonded" is the inverse of ZMK's "open" (no bond stored). */
    DAISY_FACTORY_CMD_BT_STATUS = 0x65,
    /* Enable/disable BLE advertising. payload: [enable u8 (0=off, 1=on)]. */
    DAISY_FACTORY_CMD_BT_ADV_SET = 0x66,
    /* Read the BLE identity address (the address seen in scan reports). No
     * payload in; -> struct daisy_factory_bt_addr. All profiles share this one
     * identity, so it is not per-profile. */
    DAISY_FACTORY_CMD_BT_ADDR = 0x67,
    /* Relay a passkey to a pending passkey-entry pairing (MITM). payload:
     * [passkey u32 little-endian, 0..999999]. Lets the host complete an
     * authenticated pair by pushing the host-displayed passkey over USB instead
     * of a human typing it on the keyboard (the passkey never goes on air).
     * UNSUPPORTED if passkey entry isn't built in; HW if no pairing is awaiting
     * a passkey; BAD_ARG if out of range. */
    DAISY_FACTORY_CMD_BT_PASSKEY = 0x68,

    /* group 0x7: HID endpoints (where input reports are routed) */
    /* -> [preferred u8 (daisy_factory_transport)][selected u8][ble_profile u8].
     * `selected` is what is actually in use right now and may be NONE (or
     * differ from `preferred`) when the preferred transport isn't connected.
     * `ble_profile` is only meaningful when selected == BLE. */
    DAISY_FACTORY_CMD_ENDPOINT_GET = 0x70,
    /* Set the preferred transport (persisted). payload:
     * [transport u8 (daisy_factory_transport)]. Note: routing only -- the USB
     * interfaces (including this factory interface) stay up either way. */
    DAISY_FACTORY_CMD_ENDPOINT_SET = 0x71,

    /* group 0x8: input injection */
    /* Tap a key: press, send an input report over the selected endpoint,
     * release, send again. payload: [usage u8, HID keyboard/keypad page].
     * Meant for end-to-end report-routing tests with an invisible key
     * (e.g. F24 = 0x73); the host really receives a keystroke. */
    DAISY_FACTORY_CMD_KEY_INJECT = 0x80,
};

/* HID transport selector for the ENDPOINT_* commands. Values match ZMK's
 * enum zmk_transport (settings-stable there, so stable here too). */
enum daisy_factory_transport {
    DAISY_FACTORY_TRANSPORT_NONE = 0, /* input reports routed nowhere */
    DAISY_FACTORY_TRANSPORT_USB = 1,
    DAISY_FACTORY_TRANSPORT_BLE = 2,
};

/* BT_STATUS response payload. */
struct daisy_factory_bt_status {
    uint8_t profile_count; /* number of BLE profiles (bond slots) */
    uint8_t active_index;  /* currently active profile */
    uint8_t flags;         /* DAISY_FACTORY_BT_FLAG_* */
    uint8_t connected_mask; /* bit N: profile N has an active connection */
    uint8_t bonded_mask;    /* bit N: profile N has a stored bond */
} __attribute__((packed));

/* BT_ADDR response payload: the keyboard's BLE addresses.
 * - `identity_*` is the static identity address (FICR-derived, permanent, from
 *   bt_id_get) -- what appears in scan reports when privacy is OFF.
 * - `current_*` is the address actually being advertised right now: equal to
 *   the identity when privacy is OFF, or the current rotating RPA when privacy
 *   is ON (from bt_le_oob_get_local).
 * - `privacy` is 1 when CONFIG_BT_PRIVACY is enabled (current is an RPA that
 *   rotates every BT_RPA_TIMEOUT), else 0.
 * Each address is little-endian (same byte order as bt_addr_le_t / HCI --
 * val[5] is the most-significant byte, printed first in the colon form).
 * `*_type` is the Bluetooth address type (0 = public, 1 = random). */
struct daisy_factory_bt_addr {
    uint8_t identity_type;
    uint8_t identity_val[6];
    uint8_t current_type;
    uint8_t current_val[6];
    uint8_t privacy;
} __attribute__((packed));

#define DAISY_FACTORY_BT_FLAG_ACTIVE_CONNECTED (1u << 0)
#define DAISY_FACTORY_BT_FLAG_ACTIVE_OPEN (1u << 1) /* active profile has no bond */
#define DAISY_FACTORY_BT_FLAG_ADV_ENABLED (1u << 2)

/* Logical GPIO identifiers for DAISY_FACTORY_CMD_GPIO_GET. Each maps in the
 * firmware to a board gpio_dt_spec; the level returned is the *logical* level
 * (gpio_pin_get_dt), i.e. it already accounts for GPIO_ACTIVE_LOW. The host
 * turns the level into a human string (e.g. Pressed/Released, Wired/Wireless). */
enum daisy_factory_gpio {
    DAISY_FACTORY_GPIO_PAIRING_BUTTON = 0,
    DAISY_FACTORY_GPIO_PROTOCOL_SWITCH = 1,
    DAISY_FACTORY_GPIO_ISO_STRAP = 2,
    DAISY_FACTORY_GPIO_ANSI_STRAP = 3,
};

/* Response status codes. 0 == success. */
enum daisy_factory_status {
    DAISY_FACTORY_OK = 0,
    DAISY_FACTORY_ERR_UNKNOWN_CMD = 1,
    DAISY_FACTORY_ERR_BAD_LENGTH = 2,
    DAISY_FACTORY_ERR_UNSUPPORTED = 3,
    DAISY_FACTORY_ERR_HW = 4,
    DAISY_FACTORY_ERR_LOCKED = 5,  /* reserved: destructive op needs unlock */
    DAISY_FACTORY_ERR_BAD_ARG = 6, /* argument out of range (bad index/value) */
};

/* Status RGB color selector for DAISY_FACTORY_CMD_RGB_SET_PWM. */
enum daisy_factory_rgb {
    DAISY_FACTORY_RGB_RED = 0,
    DAISY_FACTORY_RGB_GREEN = 1,
    DAISY_FACTORY_RGB_BLUE = 2,
};

/* Device class, for tools that drive both keyboard and dongle. */
enum daisy_factory_device_type {
    DAISY_FACTORY_DEVICE_KEYBOARD = 0,
    DAISY_FACTORY_DEVICE_DONGLE = 1,
};

/* CORE_INFO response payload. */
struct daisy_factory_info {
    uint8_t proto_version; /* == DAISY_FACTORY_PROTO_VERSION */
    uint8_t device_type;   /* daisy_factory_device_type */
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    uint8_t _reserved[3];     /* keep capabilities 32-bit aligned */
    uint32_t capabilities; /* bitmap of supported command groups, LE */
} __attribute__((packed));

#endif /* DAISY_FACTORY_H */

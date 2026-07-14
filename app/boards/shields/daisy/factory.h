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
};

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

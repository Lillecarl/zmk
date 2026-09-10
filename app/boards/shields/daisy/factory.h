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
    /* Read (and clear) the crash breadcrumb from the previous boot: when the
     * firmware dies on a fatal error (hard fault, oops, failed assert) it
     * records where in __noinit RAM and warm-reboots; that record survives
     * the reboot. payload in: [clear u8 (0=peek, 1=clear after read)];
     * out: struct daisy_factory_crash_info. `valid` is 0 (and the rest
     * zeroed) when the previous shutdown was clean. Resolve `pc`/`lr`
     * against the matching zmk.elf with addr2line. */
    DAISY_FACTORY_CMD_CRASH_INFO = 0x03,

    /* group 0x1: battery (read-only) */
    DAISY_FACTORY_CMD_BATTERY_TEMP = 0x11,    /* -> i16 hundredths of degC, LE */
    DAISY_FACTORY_CMD_BATTERY_VOLTAGE = 0x12, /* -> u16 millivolts, LE */
    DAISY_FACTORY_CMD_BATTERY_CURRENT = 0x13, /* -> i16 milliamps (signed: + charge), LE */

    /* group 0x2: charging */
    DAISY_FACTORY_CMD_CHARGING_SET = 0x20, /* payload: [enable u8 (0=stop, 1=start)] */
    /* Read the nPM1300 charger state: the raw BCHGCHARGESTATUS, BCHGERRREASON
     * and VBUSINSTATUS registers. No payload in; out: struct
     * daisy_factory_charging_status (bit meanings: DAISY_FACTORY_CHG_*).
     * A nonzero `error` is latched: the charger has stopped on that error and
     * stays stopped until CHARGING_CLEAR_ERROR (or a CHARGING_SET restart)
     * releases it. UNSUPPORTED on targets without the PMIC. */
    DAISY_FACTORY_CMD_CHARGING_STATUS = 0x21,
    /* Recover from a latched charger error: strobes TASKCLEARCHGERR (clears
     * BCHGERRREASON/BCHGERRSENSOR) then TASKRELEASEERROR (releases the charger
     * from its error state so charging resumes). Action-only, no payload;
     * harmless when no error is latched. UNSUPPORTED without the PMIC. */
    DAISY_FACTORY_CMD_CHARGING_CLEAR_ERROR = 0x22,

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
    /* Warm-reboot back into the application (a plain reset, not a bootloader
     * jump). Acks first, then reboots shortly after so the response reaches the
     * host. No payload. */
    DAISY_FACTORY_CMD_REBOOT = 0x53,
    /* Return the keyboard to a factory-default state, then reboot. Clears every
     * BLE bond (and selects profile 0), reverts all ZMK Studio keymap/layout
     * modifications to the built-in defaults, exits factory mode, and finally
     * warm-reboots. Persisted changes are written before the reboot. Acks
     * first, then reboots shortly after so the response reaches the host (same
     * pattern as REBOOT). No payload. */
    DAISY_FACTORY_CMD_FACTORY_RESET = 0x54,
    /* Enter standby (ZMK deep sleep / SoC System OFF): suspend every device's
     * PM hook, then power the SoC off. Acks first, then enters standby
     * shortly after so the response reaches the host; the keyboard drops off
     * USB/BLE. There is deliberately no exit command -- in System OFF the
     * factory interface is gone; a configured wake source (key press) wakes
     * it, which is a full reboot. UNSUPPORTED without CONFIG_ZMK_SLEEP. No
     * payload. */
    DAISY_FACTORY_CMD_STANDBY_ENTER = 0x55,

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
    /* Read one BLE profile (bond slot). payload in: [index u8]; out: struct
     * daisy_factory_bt_profile. BAD_ARG if index >= profile_count (from
     * BT_STATUS). Reports the peer address stored for that slot -- i.e. which
     * host the slot is paired with -- plus per-slot bonded/connected/active
     * flags. The peer address is all-zero (BT_ADDR_LE_ANY) for an open/unbonded
     * slot. Iterate 0..profile_count on the host to enumerate every slot; ZMK
     * stores nothing else per slot (the name field is unused). */
    DAISY_FACTORY_CMD_BT_PROFILE_GET = 0x69,
    /* BLE link-layer diagnostics: what the radio actually did, not just what
     * the host permitted. No payload in; -> struct daisy_factory_bt_diag.
     * Distinguishes advertising-directed / advertising-open /
     * not-advertising (which BT_STATUS's adv-enabled flag cannot), and
     * reports the controller's CONNECT_IND acceptance counters plus the
     * nRF54L address-resolver (AAR) health counters. Reads are cheap and
     * side-effect free; counters reset on reboot. */
    DAISY_FACTORY_CMD_BT_DIAG = 0x6A,

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

    /* group 0x9: touchpad */
    /* Set the touchpad power state. payload:
     * [state u8 (daisy_factory_touchpad_power)]. ON is normal operation.
     * SLEEP is the pad's "modern standby" (HID-I2C SET_POWER sleep, ~7 uA):
     * it stops streaming but still detects a touch; only ON resumes
     * reporting. OFF is SLEEP plus the passthrough driver suspended (its
     * data-ready interrupt can't fire), so no reports flow even if the pad
     * wakes itself on a touch. The PCT1036's deeper register-flow suspend is
     * a no-op in HID-I2C mode, so modern standby is the lowest reachable
     * state. Idempotent, RAM-only: a keyboard reboot returns the pad to ON
     * (the driver sends SET_POWER on at init). UNSUPPORTED on targets
     * without a touchpad; HW on an I2C failure. */
    DAISY_FACTORY_CMD_TOUCHPAD_POWER_SET = 0x90,
    /* Read a raw 8-bit touchpad register (PCT1036 vendor register space,
     * outside the HID protocol). payload in: [reg u8]; out: [value u8].
     * Debug/bring-up aid. NOTE: in HID-I2C mode the pad ACKs these but the
     * vendor register space does not appear to be wired up (reads return 0,
     * writes change nothing). UNSUPPORTED without a touchpad; HW on I2C
     * NAK. */
    DAISY_FACTORY_CMD_TOUCHPAD_REG_READ = 0x91,
    /* Write a raw 8-bit touchpad register. payload: [reg u8, value u8].
     * Debug/bring-up aid; same HID-I2C-mode caveat as TOUCHPAD_REG_READ. */
    DAISY_FACTORY_CMD_TOUCHPAD_REG_WRITE = 0x92,

    /* --- synthetic touchpad input (group 0x9, cont.) ---------------------
     *
     * The three INJECT commands hand a fabricated input report to the
     * passthrough backends exactly where the I2C read path hands them a real
     * one: same callbacks, same zmk_endpoint_get_selected() routing, same BLE
     * pacing and urgency logic. So a host can exercise the whole outbound
     * chain -- USB HID interface, BLE HOGP to a phone/PC, or BLE to the
     * Framework dongle and out its USB -- with no finger on the pad, and with
     * frame contents and timing it controls exactly.
     *
     * Nothing is written to the pad over I2C: injection works even with the
     * pad asleep (TOUCHPAD_POWER_SET 0x90) or absent, and a real finger
     * moving at the same time simply interleaves its frames with the
     * synthetic ones.
     *
     * Pick the endpoint FIRST (ENDPOINT_SET 0x71, and BT_PROF_SELECT 0x62 for
     * BLE): a frame injected while another transport is selected is dropped
     * by the backends, exactly as a real one would be.
     *
     * All three are UNSUPPORTED on a target built without a touchpad. */

    /* Stage raw report bytes and optionally send them. payload:
     * [report_id u8][offset u8][flags u8][bytes...].
     *
     * `bytes` are copied into a staging buffer at `offset`; with
     * DAISY_FACTORY_TP_INJECT_SEND set in `flags` the buffer's first
     * (offset + len(bytes)) bytes are then dispatched as one input report of
     * `report_id`. The staging buffer exists because a full PTP frame (id 4,
     * 29 bytes) does not fit in one DAISY_FACTORY_PAYLOAD_SIZE frame: send
     * offset 0 with 24 bytes, then offset 24 with the last 5 and SEND.
     * A mouse-mode report (id 1, 8 bytes) fits in a single SEND frame.
     *
     * A frame at a non-zero offset must carry the same `report_id` as the
     * sequence it continues, or it is BAD_ARG: that stops a half-staged
     * buffer being completed by an unrelated report. A SEND releases the
     * binding, so the next sequence is free to use a different id.
     * offset + len must be <= DAISY_FACTORY_TP_INJECT_MAX_REPORT (BAD_ARG).
     * Report IDs the descriptor doesn't declare are accepted here and
     * dropped downstream; this command deliberately doesn't second-guess the
     * descriptor.
     *
     * The buffer is NOT cleared between reports, which is a feature: stage
     * the constant part of a frame once, then send a series that only
     * rewrites the moving bytes. */
    DAISY_FACTORY_CMD_TOUCHPAD_INJECT_RAW = 0x93,

    /* Build and send one PTP (id 4) frame from a compact description.
     * payload: [contact_count u8][buttons u8][n_fingers u8]
     *          then n_fingers x [status u8][x u16 LE][y u16 LE].
     *
     * One frame on the wire per report, where INJECT_RAW needs two, so this
     * is the one to stream from the host. `status` is the descriptor's own
     * finger status byte: bit0 confidence, bit1 tip switch, bits 4-7 contact
     * id. Fingers are placed in report order starting at contact slot 0;
     * unlisted slots are zero (confidence 0, tip 0, position 0), which is how
     * a real pad reports a lifted contact.
     *
     * `contact_count` is sent as given rather than derived from n_fingers, so
     * a deliberately inconsistent frame can be injected to see what the host
     * does with it. Scan time is filled in by the firmware from its own
     * uptime in 100 us units, matching the descriptor's unit exponent -4 --
     * a host-supplied one would be a lie the moment the frame is paced.
     *
     * n_fingers > DAISY_FACTORY_TP_INJECT_PTP_MAX_FINGERS is BAD_ARG; that
     * limit is what fits in one frame, and the descriptor's fifth contact
     * slot is only reachable through INJECT_RAW. */
    DAISY_FACTORY_CMD_TOUCHPAD_INJECT_PTP = 0x94,

    /* Generate a straight-line one-finger drag on the keyboard and send it as
     * a series of PTP frames. payload:
     * [x0 u16 LE][y0 u16 LE][x1 u16 LE][y1 u16 LE]
     * [frames u16 LE][interval_us u32 LE][flags u8]
     * -> [duration_ms u32 LE]
     *
     * `frames` positions are linearly interpolated from (x0,y0) to (x1,y1)
     * inclusive, emitted on an ABSOLUTE tick grid `interval_us` apart, then
     * (unless NO_LIFT) one final frame with contact count 0 and the tip
     * released. The grid is anchored once at the start, so the cadence does
     * not accumulate per-frame scheduling drift.
     *
     * That fixed cadence is the point: driving the same frames from the host
     * puts a USB round-trip and the host scheduler between them, which is
     * indistinguishable in an arrival histogram from the firmware pacing
     * defects these tests exist to measure. interval_us is microseconds, not
     * milliseconds, because the intervals that matter (a 7.5 ms connection
     * interval, a 7.10 ms pad frame period) are not whole milliseconds.
     *
     * `interval_us` is rounded UP to a whole kernel tick (32 us on this SoC,
     * CONFIG_SYS_CLOCK_TICKS_PER_SEC=31250), so 7500 becomes 7520. The
     * returned `duration_ms` is measured on that real grid, not on the
     * requested interval, so waiting it out always outlasts the run.
     *
     * Returns immediately, before the first frame goes out; `duration_ms` is
     * how long the run will take, for the host to wait out. A sweep started
     * while one is running REPLACES it (no error) -- the new anchor is now.
     * `frames` == 0 cancels a running sweep and sends nothing, not even a
     * lift-off frame, which leaves the host holding a pressed contact on
     * purpose: that is how you test what it does with a stuck finger.
     *
     * frames > DAISY_FACTORY_TP_INJECT_SWEEP_MAX_FRAMES, or interval_us
     * outside [DAISY_FACTORY_TP_INJECT_SWEEP_MIN_US,
     * DAISY_FACTORY_TP_INJECT_SWEEP_MAX_US], is BAD_ARG. Coordinates are not
     * range-checked against the descriptor's logical maximum: injecting an
     * out-of-range position is a legitimate test. */
    DAISY_FACTORY_CMD_TOUCHPAD_INJECT_SWEEP = 0x95,

    /* group 0xA: PERT (packet-error-rate test via Bluetooth Direct Test Mode)
     *
     * Raw-PHY per-channel PER measurement between the keyboard and a test
     * peer (the Framework dongle). DTM owns the radio exclusively: the host
     * must send PERT_QUIESCE first and poll BT_STATUS until nothing is
     * connected or advertising before starting a test. DTM leaves the BLE
     * stack's scheduler out of the loop, so after a test run the only
     * supported recovery is a REBOOT (0x53); don't expect the pre-test
     * connection to resume. All PERT commands are UNSUPPORTED unless the
     * firmware was built with CONFIG_BT_CTLR_DTM_HCI (see the PERT
     * capability bit). */

    /* Quiesce the BLE link layer for DTM: disconnect every profile and stop
     * advertising. The disconnects are asynchronous -- poll BT_STATUS until
     * connected_mask == 0 and ADV_ENABLED is clear before starting a test.
     * No payload. */
    DAISY_FACTORY_CMD_PERT_QUIESCE = 0xA0,
    /* Start transmitting DTM test packets, continuously until PERT_END.
     * payload: [chan u8][phy u8][len u8][pattern u8][tx_power i8]:
     *   chan: DTM channel 0-39 (freq = 2402 + 2*chan MHz); BAD_ARG if > 39.
     *     Note this is NOT group 0xB's channel numbering, which is MHz above
     *     2400 (0..80). Channel 20 here is 2442 MHz; in group 0xB it's 2420.
     *   phy: daisy_factory_pert_phy; BAD_ARG otherwise.
     *   len: test-data length in bytes, 0-255.
     *   pattern: daisy_factory_pert_pattern; BAD_ARG if out of range or
     *     PRBS15 (unimplemented in the Zephyr controller's table).
     *   tx_power: optional; dBm, floored to the next supported step
     *     (nRF54LM20A: +8..-46). Defaults to 0 dBm -- the product's
     *     operating power -- when the payload is 4 bytes.
     * HW if the controller refuses (e.g. a test is already running or the
     * radio isn't quiesced). */
    DAISY_FACTORY_CMD_PERT_TX_START = 0xA1,
    /* Start receiving DTM test packets, counting until PERT_END.
     * payload: [chan u8][phy u8] (same validation as PERT_TX_START). */
    DAISY_FACTORY_CMD_PERT_RX_START = 0xA2,
    /* Stop the running DTM test. -> [num_rx u16 LE]: packets received since
     * PERT_RX_START (0 after a TX test). HW if no test was running. */
    DAISY_FACTORY_CMD_PERT_END = 0xA3,

    /* group 0xB: RF test modes (CE/FCC certification)
     *
     * Nordic's radio_test driven straight off the nrfx radio HAL: unmodulated
     * (CW) TX, modulated TX, duty-cycled modulated TX, RX with a packet
     * count, and TX/RX channel sweeps -- the modes Lite-On's BT-test SOP asks
     * for, at any channel and TX power. This is the same code the standalone
     * radio_test image runs, compiled into the product firmware so the lab
     * needs no image swap.
     *
     * Distinct from group 0xA in every way that matters: PERT rides the BLE
     * controller's Direct Test Mode and speaks BLE packets, while this owns
     * the radio outright and the BLE stack is gone.
     *
     * !! CHANNEL NUMBERING !! Group 0xB channels are MHz above 2400, range
     * 0..80 -- 2402/2440/2480 MHz are 2/40/80. Group 0xA channels are DTM
     * indices 0..39 (freq = 2402 + 2*chan). The same number means a different
     * frequency in the two groups; do not copy one into the other.
     *
     * Lifecycle: RF_ENTER once (it tears the BLE stack down and takes the
     * radio), then any number of RF_START/RF_STOP/RF_STATUS. Entering is
     * ONE-WAY -- there is no command that gives the radio back. Recovery is
     * REBOOT (0x53). USB is untouched throughout, so this interface stays up
     * for the whole test session.
     *
     * All group 0xB commands are UNSUPPORTED unless the firmware was built
     * with CONFIG_DAISY_FACTORY_RF (see the RF capability bit). */

    /* Take the radio for RF testing: stop advertising, disconnect every
     * profile, shut the BLE stack down (bt_disable), start the HFXO, and
     * initialize the radio-test driver. No payload. Idempotent -- a second
     * call on an already-entered device just succeeds.
     *
     * LOCKED unless factory mode is active (FACTORY_MODE_SET 0x51). That gate
     * is deliberate: this ships in production firmware and kills BLE until
     * the next reboot, so it must not be one stray HID report away on a
     * user's keyboard.
     *
     * HW if the BLE teardown or the radio/clock init fails. */
    DAISY_FACTORY_CMD_RF_ENTER = 0xB0,
    /* Configure and start an RF test mode. Any running test is cancelled
     * first, so RF_START can be called back to back without RF_STOP.
     * payload: [mode u8][rate u8][pattern u8][tx_power i8][chan_start u8]
     *          [chan_end u8][dwell_ms u16 LE][duty u8][packets u32 LE]
     *          [payload_len u8, optional]
     *   mode:       daisy_factory_rf_mode; BAD_ARG otherwise.
     *   rate:       daisy_factory_rf_rate; BAD_ARG otherwise.
     *   pattern:    daisy_factory_rf_pattern; BAD_ARG otherwise.
     *   tx_power:   dBm (nRF54LM20A: +8 max, down to -46). Values off the
     *               hardware's step list fall back to 0 dBm.
     *   chan_start: 0..80, MHz above 2400. The channel for the non-sweep
     *               modes, the first channel for the sweeps. BAD_ARG if > 80.
     *   chan_end:   0..80, last channel of a sweep; ignored otherwise.
     *               BAD_ARG if > 80 or < chan_start when a sweep is selected.
     *   dwell_ms:   0..99 ms on each channel of a sweep; ignored otherwise.
     *               BAD_ARG if > 99 (the radio-test timer's range).
     *   duty:       1..90 percent, MODULATED_TX_DUTY_CYCLE only; BAD_ARG if
     *               out of range for that mode, ignored for the others.
     *   packets:    number of packets for MODULATED_TX and RX; 0 = run until
     *               RF_STOP. When nonzero the test ends by itself and
     *               RF_STATUS's `running` clears -- that is how a host waits
     *               for a counted run to finish.
     *   payload_len: OPTIONAL 14th byte. Test-data bytes per transmitted
     *               packet, 1..255, for the MODULATED_TX modes; ignored by
     *               the others. Omitted, or 0, selects 255 -- what every
     *               firmware before this field sent, so a 13-byte RF_START
     *               means exactly what it always did.
     *
     *               0 rather than "absent" is the unset value because the
     *               dongle's config protocol carries a fixed-size buffer with
     *               no length, so it cannot tell a byte that was left out from
     *               one that was left zero. The two devices answer the same
     *               host code, so they agree on the sentinel.
     *
     *               It sets the time on air, and so the runtime of a counted
     *               run: a BLE 1M packet is (8 + payload_len) bytes on air,
     *               2104 us at 255 and 168 us at 13. Certification wants the
     *               long packet; a link test wants a short one, and Lite-On's
     *               PERT guide measures its <=1% PER against a 13-byte packet,
     *               so PER is only comparable between runs of equal length.
     *
     *               Both ends of a link need the SAME value. It is not only
     *               the transmitted length: PCNF1.MAXLEN is set from it, and
     *               that sizes the window the radio spends on each reception
     *               whatever the on-air header says. A receiver left at 255
     *               counts about (8 + payload_len) / 263 of a short
     *               transmitter's packets -- measured 8.1% at 13 bytes -- and
     *               one set shorter than the transmitter drops everything.
     *
     *               Whether the firmware honours it is visible in RF_STATUS:
     *               a response carrying `payload_len` (24 bytes, not 23) has
     *               it. There is no capability bit, because a host driving a
     *               link test has to ask the same question of the dongle,
     *               whose config protocol has no capability word.
     * HW if RF_ENTER hasn't run. */
    DAISY_FACTORY_CMD_RF_START = 0xB1,
    /* Stop the running test and return the RX packet count.
     * -> [rx_packets u32 LE]: packets received with a valid CRC since the
     * last RX-mode RF_START (0 after a TX-mode test). Safe to call when no
     * test is running. HW if RF_ENTER hasn't run. */
    DAISY_FACTORY_CMD_RF_STOP = 0xB2,
    /* Read the RF test state. No payload in; out: struct
     * daisy_factory_rf_status. Serves as both the "what is configured"
     * readback and the completion poll for counted runs. Valid before
     * RF_ENTER (it reports entered = 0). */
    DAISY_FACTORY_CMD_RF_STATUS = 0xB3,
};

/* Test mode for RF_START. Values mirror `enum radio_test_mode` in
 * rf/radio_test.h, but are pinned here because this is the wire contract --
 * a reordering upstream must not silently change the protocol. */
enum daisy_factory_rf_mode {
    DAISY_FACTORY_RF_MODE_UNMODULATED_TX = 0, /* CW carrier */
    DAISY_FACTORY_RF_MODE_MODULATED_TX = 1,
    DAISY_FACTORY_RF_MODE_RX = 2,
    DAISY_FACTORY_RF_MODE_TX_SWEEP = 3,
    DAISY_FACTORY_RF_MODE_RX_SWEEP = 4,
    DAISY_FACTORY_RF_MODE_MODULATED_TX_DUTY_CYCLE = 5,
};

/* Radio data rate for RF_START. Only the BLE PHYs daisy's cert scope covers
 * are offered; the radio also does IEEE 802.15.4 and the proprietary nRF
 * rates, which are out of scope (daisy is BLE-only). */
enum daisy_factory_rf_rate {
    DAISY_FACTORY_RF_RATE_BLE_1M = 0,
    DAISY_FACTORY_RF_RATE_BLE_2M = 1,
};

/* Transmit pattern for the modulated RF_START modes. Values mirror
 * `enum transmit_pattern` in rf/radio_test.h; pinned here for the same reason
 * as the modes. The SOP's modulated-TX step uses 11110000. */
enum daisy_factory_rf_pattern {
    DAISY_FACTORY_RF_PAT_RANDOM = 0,
    DAISY_FACTORY_RF_PAT_11110000 = 1,
    DAISY_FACTORY_RF_PAT_11001100 = 2,
};

/* Channel bounds for RF_START, in MHz above 2400. The radio-test driver
 * accepts the full 0..80 range; the SOP's low/mid/high are 2/40/80. */
#define DAISY_FACTORY_RF_CHANNEL_MAX 80
/* Sweep dwell time bound, in ms (the radio-test timer's usable range). */
#define DAISY_FACTORY_RF_DWELL_MS_MAX 99
/* Duty-cycle bounds, in percent, for MODULATED_TX_DUTY_CYCLE. */
#define DAISY_FACTORY_RF_DUTY_MIN 1
#define DAISY_FACTORY_RF_DUTY_MAX 90

/* RF_STATUS response payload.
 * - `entered` is 1 once RF_ENTER has taken the radio (BLE is gone).
 * - `running` is 1 while a test is on air. It clears by itself when a counted
 *   MODULATED_TX or RX run completes, which is how a host waits for one to
 *   finish -- the shell image printed "The modulated TX has finished"; over a
 *   request/response protocol this flag is the equivalent.
 * - the config fields echo the last accepted RF_START (zeroed before the
 *   first one), so this doubles as the `parameters_print` readback.
 * - `rx_packets` is the valid-CRC count since the last RX-mode RF_START, live
 *   while RX is running; it is NOT cleared by RF_STOP (same semantics as the
 *   shell image's print_rx: the next RX start resets it).
 * - `tx_packets` is the count put on air since the current run started, live
 *   while it runs. It is the only host-visible evidence that a *continuous*
 *   transmit is really keying the radio -- `running` is just the firmware's own
 *   flag. Nonzero only for the modulated TX modes; 0 for an unmodulated
 *   carrier (which sends no packets), for RX and for the sweeps.
 *
 * - `payload_len` echoes the transmitted test-data length of the last accepted
 *   RF_START (see the command). Its presence is also how a host detects that
 *   this firmware has the field at all: 24 bytes back, not 23.
 *
 * The first 19 bytes through `rx_packets` are byte-identical to the dongle's
 * `rf_status_t` (hid-remapper-private, firmware/src/types.h), `tx_packets` sits
 * at the same offset 19 there and `payload_len` at 23, so one host decoder
 * serves both. Append any future field at the end, on both sides -- and never
 * inside the embedded config, which would move everything after it. */
struct daisy_factory_rf_status {
    uint8_t entered;
    uint8_t running;
    uint8_t mode;    /* daisy_factory_rf_mode */
    uint8_t rate;    /* daisy_factory_rf_rate */
    uint8_t pattern; /* daisy_factory_rf_pattern */
    int8_t tx_power; /* dBm */
    uint8_t chan_start;
    uint8_t chan_end;
    uint16_t dwell_ms;   /* LE */
    uint8_t duty;        /* percent */
    uint32_t packets;    /* LE, 0 = continuous */
    uint32_t rx_packets; /* LE */
    uint32_t tx_packets; /* LE */
    uint8_t payload_len; /* test-data bytes per transmitted packet */
} __attribute__((packed));

/* PHY selector for the PERT_*_START commands. Values match the HCI LE
 * (enhanced) transmitter/receiver-test PHY encoding. Coded PHY is not
 * offered: the daisy link never uses it and rx would need S=2/S=8 split. */
enum daisy_factory_pert_phy {
    DAISY_FACTORY_PERT_PHY_1M = 1,
    DAISY_FACTORY_PERT_PHY_2M = 2,
};

/* DTM packet payload patterns for PERT_TX_START. Values match the HCI
 * transmitter-test pkt_payload encoding (BT Core Vol 4 Part E 7.8.29).
 * PRBS15 (3) is rejected: the Zephyr split controller stubs its table. */
enum daisy_factory_pert_pattern {
    DAISY_FACTORY_PERT_PAT_PRBS9 = 0,
    DAISY_FACTORY_PERT_PAT_11110000 = 1,
    DAISY_FACTORY_PERT_PAT_10101010 = 2,
    DAISY_FACTORY_PERT_PAT_PRBS15 = 3, /* unsupported, documented for completeness */
    DAISY_FACTORY_PERT_PAT_11111111 = 4,
    DAISY_FACTORY_PERT_PAT_00000000 = 5,
    DAISY_FACTORY_PERT_PAT_00001111 = 6,
    DAISY_FACTORY_PERT_PAT_01010101 = 7,
};

/* Touchpad power states for DAISY_FACTORY_CMD_TOUCHPAD_POWER_SET, ordered so
 * that 0/1 read as a plain off/on toggle. */
enum daisy_factory_touchpad_power {
    DAISY_FACTORY_TOUCHPAD_OFF = 0,   /* standby + passthrough quiesced */
    DAISY_FACTORY_TOUCHPAD_ON = 1,    /* run: full operation */
    DAISY_FACTORY_TOUCHPAD_SLEEP = 2, /* modern standby: touch-detect only */
};

/* --- synthetic touchpad input (TOUCHPAD_INJECT_*) ------------------------ */

/* Report IDs the pad's descriptor declares for input. Injecting anything else
 * is accepted by INJECT_RAW and dropped by the passthrough backends. */
#define DAISY_FACTORY_TP_REPORT_ID_MOUSE 1 /* 8 data bytes, RELATIVE motion */
#define DAISY_FACTORY_TP_REPORT_ID_PTP 4   /* 29 data bytes, ABSOLUTE position */

/* Largest report INJECT_RAW will stage, i.e. the size of its staging buffer.
 * Comfortably above the 29-byte PTP frame, which is the biggest input report
 * the descriptor declares. */
#define DAISY_FACTORY_TP_INJECT_MAX_REPORT 64

/* Flags byte of DAISY_FACTORY_CMD_TOUCHPAD_INJECT_RAW. */
/* Dispatch the staged buffer as an input report once this frame's bytes are
 * copied in. Without it the frame only stages bytes. */
#define DAISY_FACTORY_TP_INJECT_SEND (1u << 0)

/* PTP frame layout, mirroring the report descriptor (see the `touchpad` node's
 * report-descriptor in daisy-touchpad.dtsi): 5 contact records of
 * [status u8][x u16 LE][y u16 LE], then contact count, buttons, scan time
 * u16 LE. Used by INJECT_PTP and the sweep generator, and the numbers a host
 * needs to hand-assemble a frame for INJECT_RAW. */
#define DAISY_FACTORY_TP_PTP_REPORT_LEN 29
#define DAISY_FACTORY_TP_PTP_MAX_CONTACTS 5
#define DAISY_FACTORY_TP_PTP_STRIDE 5     /* bytes per contact record */
#define DAISY_FACTORY_TP_PTP_COUNT_OFF 25 /* contact count */
#define DAISY_FACTORY_TP_PTP_BTN_OFF 26   /* buttons, bits 0-2 */
#define DAISY_FACTORY_TP_PTP_SCAN_OFF 27  /* scan time u16 LE, 100 us units */

/* Bits of a contact record's status byte. */
#define DAISY_FACTORY_TP_PTP_CONFIDENCE (1u << 0)
#define DAISY_FACTORY_TP_PTP_TIP (1u << 1)
#define DAISY_FACTORY_TP_PTP_CONTACT_ID_SHIFT 4

/* Fingers INJECT_PTP can describe in one frame: what fits in
 * DAISY_FACTORY_PAYLOAD_SIZE after its 3-byte header, at 5 bytes each. The
 * descriptor's fifth contact slot is reachable only through INJECT_RAW. */
#define DAISY_FACTORY_TP_INJECT_PTP_MAX_FINGERS 4

/* Bounds on DAISY_FACTORY_CMD_TOUCHPAD_INJECT_SWEEP. The frame cap keeps a
 * typo from starting a run that outlives the test; the interval floor keeps
 * the generator from starving its own work queue. */
#define DAISY_FACTORY_TP_INJECT_SWEEP_MAX_FRAMES 4096
#define DAISY_FACTORY_TP_INJECT_SWEEP_MIN_US 500
#define DAISY_FACTORY_TP_INJECT_SWEEP_MAX_US 1000000

/* Flags byte of DAISY_FACTORY_CMD_TOUCHPAD_INJECT_SWEEP. */
/* Leave the contact pressed at the end: no final count-0 frame. The host is
 * left holding a finger that never lifts, on purpose. */
#define DAISY_FACTORY_TP_SWEEP_NO_LIFT (1u << 0)
/* Hold button 1 down for every frame of the sweep (a drag rather than a
 * move). The lift-off frame releases it. */
#define DAISY_FACTORY_TP_SWEEP_BUTTON (1u << 1)

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

/* BT_DIAG advertising states (ZMK's enum advertising_type). */
enum daisy_factory_bt_adv_status {
    DAISY_FACTORY_BT_ADV_NONE = 0,    /* not advertising */
    DAISY_FACTORY_BT_ADV_STEALTH = 1, /* connectable but non-discoverable, for the
                                       * bonded host (was ADV_DIRECT_IND "directed"
                                       * on firmware before 2026-08-20) */
    DAISY_FACTORY_BT_ADV_OPEN = 2,    /* connectable undirected, discoverable */
};

/* BT_DIAG response payload. All counters are u16 little-endian, increasing
 * since boot and saturating at 0xFFFF (u16 so the whole struct fits the
 * single-frame 27-byte payload; saturation loses nothing a diagnosis needs).
 * The ci_* counters instrument the controller's CONNECT_IND acceptance path
 * while advertising (lll_adv.c); the ar_* counters instrument the radio's
 * hardware address resolver (AAR), which on nRF54LM20A frequently never
 * completes -- ar_end_timeout climbing while ar_resolved stays 0 is that bug
 * in action, and ci_rx_unresolved growing in step with ci_seen means
 * initiators' RPAs are reaching the acceptance check unresolved (see aster
 * issues/reset-reconnect-failure.md). */
struct daisy_factory_bt_diag {
    uint8_t adv_status; /* enum daisy_factory_bt_adv_status */
    uint8_t adv_phase;  /* stealth advertising cadence: 0 = n/a, 1 = fast
                         * burst window (20 ms interval), 2 = slow fallback
                         * (100-150 ms). Older directed-adv firmware also used
                         * the high nibble (bursts left in the chain); now
                         * always 0 there. */
    uint16_t ci_seen;           /* CONNECT_INDs received while advertising */
    uint16_t ci_accepted;       /* ... accepted (connection proceeds) */
    uint16_t ci_rl_not_allowed; /* rejected: initiator disallowed by resolving list */
    uint16_t ci_adva_bad;       /* rejected: AdvA in CONNECT_IND != ours */
    uint16_t ci_tgta_bad;       /* rejected: InitA failed the TargetA check */
    uint16_t ci_rx_unresolved;  /* InitA was an RPA the radio did not resolve */
    uint16_t ar_configured;     /* AAR armed for an advertising event */
    uint16_t ar_no_bitcount;    /* AAR skipped: RX bit counter never matched */
    uint16_t ar_end_timeout;    /* AAR never completed (nRF54L hardware bug) */
    uint16_t ar_notresolved;    /* AAR completed: no IRK matched */
    uint16_t ar_resolved;       /* AAR completed: resolved to a bonded peer */
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

/* BT_PROFILE_GET response payload: the detail for one bond slot.
 * - `addr_*` is the bonded peer's address (which host this slot is paired
 *   with), little-endian / HCI order (val[5] is the MSB, printed first in the
 *   colon form); addr_val is all-zero when the slot is open (unbonded).
 * - `addr_type` is the peer's Bluetooth address type (0 = public, 1 = random).
 * - `flags` carries the per-slot state (DAISY_FACTORY_BT_PROF_FLAG_*). */
struct daisy_factory_bt_profile {
    uint8_t index;       /* echoes the requested slot index */
    uint8_t flags;       /* DAISY_FACTORY_BT_PROF_FLAG_* */
    uint8_t addr_type;   /* peer address type (0 public, 1 random) */
    uint8_t addr_val[6]; /* peer address, LE; all-zero if the slot is open */
} __attribute__((packed));

#define DAISY_FACTORY_BT_PROF_FLAG_BONDED (1u << 0)    /* slot holds a stored bond */
#define DAISY_FACTORY_BT_PROF_FLAG_CONNECTED (1u << 1) /* slot has an active connection */
#define DAISY_FACTORY_BT_PROF_FLAG_ACTIVE (1u << 2)    /* slot is the active profile */

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

/* CHARGING_STATUS response payload: raw nPM1300 charger registers, one byte
 * each, decoded with the DAISY_FACTORY_CHG_* bit masks below.
 * - `status` = BCHGCHARGESTATUS (0x03:0x34), the live charging state.
 * - `error`  = BCHGERRREASON (0x03:0x36), the latched stop reason; 0 = none.
 * - `vbus`   = VBUSINSTATUS (0x02:0x07), the USB input supply state. */
struct daisy_factory_charging_status {
    uint8_t status;
    uint8_t error;
    uint8_t vbus;
} __attribute__((packed));

/* BCHGCHARGESTATUS bits. */
#define DAISY_FACTORY_CHG_STATUS_BATT_DETECTED (1u << 0)
#define DAISY_FACTORY_CHG_STATUS_COMPLETED (1u << 1)     /* charged to VTERM */
#define DAISY_FACTORY_CHG_STATUS_TRICKLE (1u << 2)
#define DAISY_FACTORY_CHG_STATUS_CONST_CURRENT (1u << 3)
#define DAISY_FACTORY_CHG_STATUS_CONST_VOLTAGE (1u << 4)
#define DAISY_FACTORY_CHG_STATUS_RECHARGE (1u << 5)
#define DAISY_FACTORY_CHG_STATUS_DIETEMP_PAUSED (1u << 6) /* die too hot */
#define DAISY_FACTORY_CHG_STATUS_SUPPLEMENT (1u << 7)     /* battery assisting VBUS */

/* BCHGERRREASON bits (latched until CHARGING_CLEAR_ERROR). */
#define DAISY_FACTORY_CHG_ERR_NTC_SENSOR (1u << 0)
#define DAISY_FACTORY_CHG_ERR_VBAT_SENSOR (1u << 1)
#define DAISY_FACTORY_CHG_ERR_VBAT_LOW (1u << 2)
#define DAISY_FACTORY_CHG_ERR_VTRICKLE (1u << 3)
#define DAISY_FACTORY_CHG_ERR_MEAS_TIMEOUT (1u << 4)
#define DAISY_FACTORY_CHG_ERR_CHARGE_TIMEOUT (1u << 5)
#define DAISY_FACTORY_CHG_ERR_TRICKLE_TIMEOUT (1u << 6)

/* VBUSINSTATUS bits. */
#define DAISY_FACTORY_CHG_VBUS_PRESENT (1u << 0)
#define DAISY_FACTORY_CHG_VBUS_CURR_LIMIT (1u << 1)
#define DAISY_FACTORY_CHG_VBUS_OVERVOLT_PROT (1u << 2)
#define DAISY_FACTORY_CHG_VBUS_UNDERVOLT (1u << 3)
#define DAISY_FACTORY_CHG_VBUS_SUSPENDED (1u << 4)
#define DAISY_FACTORY_CHG_VBUS_OUT_ACTIVE (1u << 5)

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

/* Capability bits for daisy_factory_info.capabilities. Bit N corresponds to
 * command group N where a whole group is build-time optional; groups always
 * compiled in don't set a bit (the bitmap started life all-zero and existing
 * hosts ignore it). */
#define DAISY_FACTORY_CAP_PERT (1u << 0xA) /* group 0xA built (CONFIG_BT_CTLR_DTM_HCI) */
#define DAISY_FACTORY_CAP_RF (1u << 0xB)   /* group 0xB built (CONFIG_DAISY_FACTORY_RF) */

/* CRASH_INFO response payload. `file` and `thread` are NUL-padded but may
 * fill their fields without a terminator. `line`/`file` are only set when
 * the fatal error was a failed assert; `pc`/`lr` only when an exception
 * frame was available. */
struct daisy_factory_crash_info {
    uint8_t valid;    /* 1 = a crash record follows, 0 = clean boot */
    uint8_t reason;   /* k_fatal_error_reason */
    uint16_t line;    /* assert line number, LE */
    uint32_t pc;      /* faulting program counter, LE */
    uint32_t lr;      /* link register, LE */
    char file[9];     /* assert file name, truncated (fills the 27-byte payload) */
    char thread[6];   /* thread that died, truncated */
} __attribute__((packed));

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

/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Daisy factory-test HID interface.
 *
 * Exposes a vendor-defined HID interface (usage page 0xFF60, usage 0x61 -- the
 * QMK "raw HID" convention) so the `aster` factory tool can talk to the
 * keyboard with no custom OS driver. See factory.h for the wire protocol.
 *
 * Compiled only when CONFIG_DAISY_FACTORY=y, which the `daisy-factory` snippet
 * sets alongside adding the `factory_hid` DT node. Production firmware omits
 * the snippet and ships none of this.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/usb_buf.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>

#include <zephyr/app_version.h>

#include <zmk/activity.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#include "factory_state.h"
#include "factory_mode_changed.h"
#include "crash_info.h"
#include "pmic_ship.h"

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/ble.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#endif

#if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
#include <zephyr/retention/bootmode.h>
#include <zephyr/sys/reboot.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
#include <zephyr/sys/poweroff.h>
#include <zmk/pm.h>
#endif

#include "factory.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daisy_factory, CONFIG_ZMK_LOG_LEVEL);

#define FACTORY_HID_NODE DT_NODELABEL(factory_hid)

BUILD_ASSERT(DT_NODE_EXISTS(FACTORY_HID_NODE),
             "Node label 'factory_hid' missing; the daisy-factory snippet must add it");

static const struct device *const hid_dev = DEVICE_DT_GET(FACTORY_HID_NODE);

/* The nPM1300 charger only exists on the real daisy board; the devkit has no
 * PMIC, so guard the reference to keep the factory HID buildable there. */
#define HAS_CHARGER DT_NODE_EXISTS(DT_NODELABEL(npm1300_charger))
#if HAS_CHARGER
static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));
#endif

/* The nPM1300 MFD, used here for the charger error-recovery tasks the sensor
 * driver exposes no API for. Ship mode (the SHIP_MODE command below) lives in
 * pmic_ship.c, which owns the entry sequence for every caller. */
#define HAS_PMIC                                                                                    \
    (DT_NODE_EXISTS(DT_NODELABEL(npm1300_regulators)) && DT_NODE_EXISTS(DT_NODELABEL(npm1300)))
#if HAS_PMIC
static const struct device *const pmic_mfd = DEVICE_DT_GET(DT_NODELABEL(npm1300));
#endif

/* The 4 white pairing LEDs (pwm20 channels 0-3), driven via the Zephyr LED API
 * by index. Absent on targets without the pwmleds node (e.g. the devkit). */
#define HAS_PAIRING_LEDS DT_NODE_EXISTS(DT_NODELABEL(pairing_leds))
#if HAS_PAIRING_LEDS
static const struct device *const pairing_leds = DEVICE_DT_GET(DT_NODELABEL(pairing_leds));
#define PAIRING_LED_COUNT DT_CHILD_NUM(DT_NODELABEL(pairing_leds))
#endif

/* Single-channel keyboard backlight (pwm21). NOTE: partial duty on pwm21 has
 * been unreliable on this board (only 0%/100% reliably take effect via the
 * nrfx fast path); the command still works, the dimming may not. */
#define HAS_BACKLIGHT DT_NODE_EXISTS(DT_NODELABEL(backlight_pwms))
#if HAS_BACKLIGHT
static const struct device *const backlight = DEVICE_DT_GET(DT_NODELABEL(backlight_pwms));
#endif

/* Status RGB. DVT1 drives it via PWM (rgb_pwmleds, real dimming); EVT has no
 * PWM RGB node, so it falls back to the led_r/g/b GPIOs (on/off only). The two
 * are mutually exclusive: prefer PWM when the node exists. */
#define HAS_RGB_PWM DT_NODE_EXISTS(DT_NODELABEL(rgb_pwmleds))
#define HAS_RGB_GPIO                                                                                \
    (!HAS_RGB_PWM && DT_NODE_EXISTS(DT_NODELABEL(led_r)) &&                                         \
     DT_NODE_EXISTS(DT_NODELABEL(led_g)) && DT_NODE_EXISTS(DT_NODELABEL(led_b)))
#define HAS_RGB (HAS_RGB_PWM || HAS_RGB_GPIO)

#if HAS_RGB_PWM
static const struct device *const rgb_leds = DEVICE_DT_GET(DT_NODELABEL(rgb_pwmleds));
/* color -> child index within rgb_pwmleds (DT child order is G,B,R, so derive
 * each index from its node rather than hardcoding). */
static const uint8_t rgb_pwm_index[] = {
    [DAISY_FACTORY_RGB_RED] = DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_r)),
    [DAISY_FACTORY_RGB_GREEN] = DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_g)),
    [DAISY_FACTORY_RGB_BLUE] = DT_NODE_CHILD_IDX(DT_NODELABEL(pwm_led_b)),
};
#elif HAS_RGB_GPIO
static const struct gpio_dt_spec rgb_gpio[] = {
    [DAISY_FACTORY_RGB_RED] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_r), gpios),
    [DAISY_FACTORY_RGB_GREEN] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_g), gpios),
    [DAISY_FACTORY_RGB_BLUE] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_b), gpios),
};
#endif

/* Touchpad (PCT1036 behind the hid-touchpad passthrough driver). Power-state
 * control needs the driver's PM hook (CONFIG_PM_DEVICE) to gate its data-ready
 * interrupt; without it the command reports UNSUPPORTED. The node exists but is
 * disabled on targets without a pad (dev kits), hence the status check. */
#define HAS_TOUCHPAD                                                                                \
    (DT_NODE_HAS_STATUS(DT_NODELABEL(touchpad), okay) && IS_ENABLED(CONFIG_PM_DEVICE))
#if HAS_TOUCHPAD
#include <hid_touchpad.h>
#include "touchpad_power.h"
static const struct device *const touchpad = DEVICE_DT_GET(DT_NODELABEL(touchpad));
#endif

/* PERT (group 0xA): raw-PHY packet-error-rate testing via the controller's
 * Direct Test Mode. Requires CONFIG_BT_CTLR_DTM_HCI=y (daisy.conf) on top of
 * BLE. The DTM commands go through the Bluetooth host as HCI commands (the
 * flow BT_CTLR_DTM_HCI exists for, and the same pattern the dongle firmware
 * uses) -- NOT via the controller-private ll_test_* API: calling that
 * directly hard-faults the firmware even with every link down, advertising
 * off, or the whole stack bt_disable()d (verified on EVT hardware
 * 2026-07-25). Over HCI the command executes in the host's TX-thread
 * context with proper command flow control. */
#define HAS_PERT (IS_ENABLED(CONFIG_BT_CTLR_DTM_HCI) && IS_ENABLED(CONFIG_ZMK_BLE))
#if HAS_PERT
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <nrfx_clock.h>
#include <hal/nrf_clock.h>
#define PERT_MAX_CHANNEL 39
#endif

/* RF test modes (group 0xB): Nordic's radio_test driven off the nrfx radio
 * HAL, for CE/FCC certification. Unlike PERT this owns the radio outright --
 * entering shuts the whole Bluetooth stack down -- so the handlers live in
 * factory_rf.c with the driver. */
#define HAS_RF IS_ENABLED(CONFIG_DAISY_FACTORY_RF)
#if HAS_RF
#include "factory_rf.h"
#endif

/* Caps lock LED (plain GPIO, gpio-leds child on the board). Normally driven by
 * the HID-indicator listener in capslock.c; the factory command pokes the pin
 * directly, so a caps-lock change from the host will overwrite it. */
#define HAS_CAPS_LED DT_NODE_EXISTS(DT_NODELABEL(caps_led))
#if HAS_CAPS_LED
static const struct gpio_dt_spec caps_led = GPIO_DT_SPEC_GET(DT_NODELABEL(caps_led), gpios);
#endif

/* Generic GPIO query targets. Each daisy_factory_gpio id maps to a board
 * gpio_dt_spec here; absent nodes simply aren't offered (the handler returns
 * UNSUPPORTED). The pairing button and protocol/mode switch are gpio-keys
 * nodes on the board; the layout straps live on the shield's zephyr,user node
 * (same source layout.c reads at boot). */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define HAS_PAIRING_BUTTON DT_NODE_EXISTS(DT_NODELABEL(pairing_button))
#if HAS_PAIRING_BUTTON
static const struct gpio_dt_spec pairing_button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pairing_button), gpios);
#endif

#define HAS_PROTOCOL_SWITCH DT_NODE_EXISTS(DT_NODELABEL(protocol_switch))
#if HAS_PROTOCOL_SWITCH
static const struct gpio_dt_spec protocol_switch =
    GPIO_DT_SPEC_GET(DT_NODELABEL(protocol_switch), gpios);
#endif

#define HAS_LAYOUT_STRAPS                                                                           \
    (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, iso_strap_gpios) &&                                         \
     DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ansi_strap_gpios))
#if HAS_LAYOUT_STRAPS
static const struct gpio_dt_spec iso_strap = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, iso_strap_gpios);
static const struct gpio_dt_spec ansi_strap = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ansi_strap_gpios);
#endif

static bool hid_ready;

/* Factory mode: while set, the keyboard never enters deep sleep (strong
 * override of the weak zmk_sleep_inhibited in activity.c), so a factory test
 * run can't lose the device mid-sequence. RAM-only; a reboot exits the mode. */
static bool factory_mode;

bool zmk_sleep_inhibited(void) { return factory_mode; }

/* Shared with the board LED modules so they stop driving the LEDs while a
 * factory test run has (exclusive) control of them. See factory_state.h. */
bool daisy_factory_mode_active(void) { return factory_mode; }

ZMK_EVENT_IMPL(daisy_factory_mode_changed);

/*
 * QMK-compatible raw HID report descriptor: a single vendor application
 * collection with a 32-byte Input report (device->host) and a 32-byte Output
 * report (host->device), no report ID.
 */
static const uint8_t factory_report_desc[] = {
    0x06, 0x60, 0xFF,             /* Usage Page (Vendor-Defined 0xFF60) */
    0x09, 0x61,                   /* Usage (0x61) */
    0xA1, 0x01,                   /* Collection (Application) */
    0x09, 0x62,                   /*   Usage (0x62) -- data in */
    0x15, 0x00,                   /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,             /*   Logical Maximum (255) */
    0x75, 0x08,                   /*   Report Size (8) */
    0x95, DAISY_FACTORY_REPORT_SIZE, /*   Report Count (32) */
    0x81, 0x02,                   /*   Input (Data, Variable, Absolute) */
    0x09, 0x63,                   /*   Usage (0x63) -- data out */
    0x91, 0x02,                   /*   Output (Data, Variable, Absolute) */
    0xC0,                         /* End Collection */
};

/* Host requests are processed off the USB control-transfer path: doing a
 * (potentially blocking) I2C sensor read inside set_report_cb would stall the
 * control transfer and can break enumeration. set_report_cb copies the frame
 * into req_buf and submits process_work; the work handler builds the response
 * and submits it as an Input report. The factory tool is strictly synchronous
 * (one request outstanding at a time), so a single slot is sufficient. */
static uint8_t req_buf[DAISY_FACTORY_REPORT_SIZE];
static struct k_work process_work;

/* Handlers run on their own queue, not the system workqueue. Two reasons:
 * a handler can take a long time (RF_ENTER tears the whole Bluetooth stack
 * down) and must not stall unrelated system work; and the system workqueue is
 * cooperative (CONFIG_SYSTEM_WORKQUEUE_PRIORITY=-1), so a handler that spins
 * there starves every preemptible thread -- including the deferred log thread,
 * which turns a stuck command into a device that says nothing about why it is
 * stuck. Preemptible priority keeps logging alive in that case.
 *
 * Stack is sized for the deepest handler: RF_ENTER's bt_disable() plus
 * radio_test_init(), which the system workqueue's 2 KB would not have covered
 * with much room to spare. */
static K_THREAD_STACK_DEFINE(process_q_stack, 4096);
static struct k_work_q process_q;
#define PROCESS_Q_PRIORITY K_PRIO_PREEMPT(5)

/* Input reports bounce through a UDC-aligned static buffer: the DWC2 DMA path
 * rejects buffers less aligned than USB_BUF_ALIGN. */
UDC_STATIC_BUF_DEFINE(resp_buf, DAISY_FACTORY_REPORT_SIZE);

static void iface_ready_cb(const struct device *dev, const bool ready) {
    LOG_INF("Factory HID interface %s", ready ? "ready" : "not ready");
    hid_ready = ready;
}

/* GET_REPORT is mandatory for every HID device (the USBD HID class refuses to
 * register without it, which would take down the whole USB device). The
 * factory protocol delivers all data via Input reports on the interrupt
 * endpoint, so GET_REPORT carries no protocol meaning; answer INPUT probes with
 * a zeroed report (avoids a control-endpoint stall) and reject everything else. */
static int get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, uint8_t *const buf) {
    if (type != HID_REPORT_TYPE_INPUT) {
        return -ENOTSUP;
    }
    memset(buf, 0, len);
    return len;
}

static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, const uint8_t *const buf) {
    if (type != HID_REPORT_TYPE_OUTPUT) {
        return -ENOTSUP;
    }
    if (len > sizeof(req_buf)) {
        return -EINVAL;
    }

    /* Copy and defer; never block the control transfer. */
    memset(req_buf, 0, sizeof(req_buf));
    memcpy(req_buf, buf, len);
    k_work_submit_to_queue(&process_q, &process_work);
    return 0;
}

static const struct hid_device_ops ops = {
    .iface_ready = iface_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

/* --- command handlers: fill the response payload, return a status code ----- */

static uint8_t handle_info(uint8_t *payload, uint8_t *out_len) {
    struct daisy_factory_info info = {
        .proto_version = DAISY_FACTORY_PROTO_VERSION,
        .device_type = DAISY_FACTORY_DEVICE_KEYBOARD,
        .fw_major = APP_VERSION_MAJOR,
        .fw_minor = APP_VERSION_MINOR,
        .fw_patch = APP_PATCHLEVEL,
        .capabilities = (HAS_PERT ? DAISY_FACTORY_CAP_PERT : 0) |
                        (HAS_RF ? DAISY_FACTORY_CAP_RF : 0),
    };
    memcpy(payload, &info, sizeof(info));
    *out_len = sizeof(info);
    return DAISY_FACTORY_OK;
}

/* Read the SoC's unique hardware device id via the Zephyr hwinfo driver. On
 * nRF this returns FICR->INFO.DEVICEID (8 bytes, big-endian, DEVICEID[1]
 * first). Read-only; the bytes are burned in at manufacture and unique per
 * chip. */
static uint8_t handle_device_id(uint8_t *payload, uint8_t *out_len) {
    ssize_t n = hwinfo_get_device_id(payload, DAISY_FACTORY_PAYLOAD_SIZE);
    if (n <= 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    *out_len = (uint8_t)n;
    return DAISY_FACTORY_OK;
}

/* Read (and optionally clear) the crash breadcrumb from the previous boot.
 * req = [clear]; -> struct daisy_factory_crash_info (valid=0 on clean boot). */
static uint8_t handle_crash_info(const uint8_t *req, uint8_t req_len, uint8_t *payload,
                                 uint8_t *out_len) {
    struct daisy_crash_info info;
    struct daisy_factory_crash_info out = {0};

    if (daisy_crash_info_get(&info)) {
        out.valid = 1;
        out.reason = (uint8_t)info.reason;
        out.line = (uint16_t)info.line;
        out.pc = info.pc;
        out.lr = info.lr;
        memcpy(out.file, info.file, MIN(sizeof(out.file), sizeof(info.file)));
        memcpy(out.thread, info.thread, MIN(sizeof(out.thread), sizeof(info.thread)));
        if (req_len >= 1 && req[0] != 0) {
            daisy_crash_info_clear();
        }
    }

    memcpy(payload, &out, sizeof(out));
    *out_len = sizeof(out);
    return DAISY_FACTORY_OK;
}

#if HAS_CHARGER
/* Sample the fuel gauge and read one channel. Returns 0 on success. */
static int read_charger_channel(enum sensor_channel chan, struct sensor_value *val) {
    if (!device_is_ready(charger)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(charger) < 0) {
        return -EIO;
    }
    return sensor_channel_get(charger, chan, val);
}
#endif

static uint8_t handle_battery_voltage(uint8_t *payload, uint8_t *out_len) {
#if !HAS_CHARGER
    /* No PMIC on this target (e.g. the devkit). */
    return DAISY_FACTORY_ERR_HW;
#else
    struct sensor_value val;
    if (read_charger_channel(SENSOR_CHAN_GAUGE_VOLTAGE, &val) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }

    /* sensor_value is volts (val1) + micro-volts (val2); report millivolts. */
    uint16_t mv = (uint16_t)(val.val1 * 1000 + val.val2 / 1000);
    sys_put_le16(mv, payload);
    *out_len = sizeof(mv);
    return DAISY_FACTORY_OK;
#endif
}

static uint8_t handle_battery_temp(uint8_t *payload, uint8_t *out_len) {
#if !HAS_CHARGER
    return DAISY_FACTORY_ERR_HW;
#else
    struct sensor_value val;
    if (read_charger_channel(SENSOR_CHAN_GAUGE_TEMP, &val) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }

    /* sensor_value is degC (val1) + micro-degC (val2); report signed
     * hundredths of a degree (0.01 degC resolution). */
    int16_t centi = (int16_t)(val.val1 * 100 + val.val2 / 10000);
    sys_put_le16((uint16_t)centi, payload);
    *out_len = sizeof(centi);
    return DAISY_FACTORY_OK;
#endif
}

static uint8_t handle_battery_current(uint8_t *payload, uint8_t *out_len) {
#if !HAS_CHARGER
    return DAISY_FACTORY_ERR_HW;
#else
    struct sensor_value val;
    if (read_charger_channel(SENSOR_CHAN_GAUGE_AVG_CURRENT, &val) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }

    /* sensor_value is amps (val1) + micro-amps (val2); report signed
     * milliamps (positive = charging into the battery). */
    int16_t ma = (int16_t)(val.val1 * 1000 + val.val2 / 1000);
    sys_put_le16((uint16_t)ma, payload);
    *out_len = sizeof(ma);
    return DAISY_FACTORY_OK;
#endif
}

/* Start or stop battery charging. req = [enable]. Action-only. Charging uses
 * the current configured in devicetree (current-microamp); this only toggles
 * the charge enable bit (and clears latched errors when starting). */
static uint8_t handle_charging_set(const uint8_t *req, uint8_t req_len) {
#if !HAS_CHARGER
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (!device_is_ready(charger)) {
        return DAISY_FACTORY_ERR_HW;
    }
    struct sensor_value val = {.val1 = req[0] ? 1 : 0, .val2 = 0};
    if (sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
                        SENSOR_ATTR_CONFIGURATION, &val) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Read the charger state: BCHGCHARGESTATUS, BCHGERRREASON and VBUSINSTATUS,
 * raw. The sensor driver refreshes all three on every sample fetch. */
static uint8_t handle_charging_status(uint8_t *payload, uint8_t *out_len) {
#if !HAS_CHARGER
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    struct sensor_value status, error, vbus;
    if (read_charger_channel(SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &status) < 0 ||
        sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &error) < 0 ||
        sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    struct daisy_factory_charging_status out = {
        .status = (uint8_t)status.val1,
        .error = (uint8_t)error.val1,
        .vbus = (uint8_t)vbus.val1,
    };
    memcpy(payload, &out, sizeof(out));
    *out_len = sizeof(out);
    return DAISY_FACTORY_OK;
#endif
}

/* Recover from a latched charger error. Action-only. The sensor driver has no
 * API for the error tasks (its only error clear is bundled into a charge
 * re-enable), so strobe them through the MFD like the ship-mode sequence:
 * TASKCLEARCHGERR wipes the latched BCHGERRREASON/BCHGERRSENSOR, then
 * TASKRELEASEERROR releases the charger's error state so charging resumes. */
#define NPM13XX_CHGR_BASE 0x03U
#define CHGR_OFFSET_TASKRELEASEERROR 0x00U
#define CHGR_OFFSET_TASKCLEARCHGERR 0x01U
static uint8_t handle_charging_clear_error(void) {
#if !HAS_PMIC
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (!device_is_ready(pmic_mfd)) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (mfd_npm13xx_reg_write(pmic_mfd, NPM13XX_CHGR_BASE, CHGR_OFFSET_TASKCLEARCHGERR, 1U) != 0 ||
        mfd_npm13xx_reg_write(pmic_mfd, NPM13XX_CHGR_BASE, CHGR_OFFSET_TASKRELEASEERROR, 1U) != 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Enter ship mode (battery cutoff). Action-only. Powers the board off, so the
 * request is armed and we ack now: pmic_ship.c defers the entry and then polls
 * until the PMIC accepts it (it refuses while VBUS is present, and keeps the
 * board alive on VBUS regardless), which gives this response time to reach the
 * host first. */
static uint8_t handle_ship_mode(void) {
    int err = daisy_pmic_ship_request(DAISY_SHIP_REASON_FACTORY);
    switch (err) {
    case 0:
    case -EALREADY:
        return DAISY_FACTORY_OK;
    case -ENOTSUP:
        return DAISY_FACTORY_ERR_UNSUPPORTED;
    default:
        return DAISY_FACTORY_ERR_HW;
    }
}

#if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
/* Rebooting tears down USB, so it can't run inline with the response. The
 * handler stamps the boot mode and acks; this fires shortly after, giving the
 * Input report time to reach the host before the reboot. */
static void bootloader_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    sys_reboot(SYS_REBOOT_WARM);
}
static K_WORK_DELAYABLE_DEFINE(bootloader_work, bootloader_work_handler);
#define BOOTLOADER_JUMP_DELAY_MS 250
#endif

/* Reboot into the MCUboot serial-recovery bootloader. Action-only. The boot
 * mode is stamped into the retention area (gpregret1) here so a failure is
 * still reported to the host; only the reboot itself is deferred. */
static uint8_t handle_bootloader_jump(void) {
#if !IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (bootmode_set(BOOT_MODE_TYPE_BOOTLOADER) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    k_work_schedule(&bootloader_work, K_MSEC(BOOTLOADER_JUMP_DELAY_MS));
    return DAISY_FACTORY_OK;
#endif
}

/* Rebooting tears down USB, so it can't run inline with the response. This
 * fires shortly after the ack, giving the Input report time to reach the host.
 * Unlike the bootloader jump, no boot mode is stamped: it is a plain warm
 * reboot straight back into the application. */
static void reboot_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    sys_reboot(SYS_REBOOT_WARM);
}
static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_work_handler);
#define REBOOT_DELAY_MS 250

/* Warm-reboot back into the application. Action-only; the reboot is deferred so
 * the ack reaches the host first. */
static uint8_t handle_reboot(void) {
    k_work_schedule(&reboot_work, K_MSEC(REBOOT_DELAY_MS));
    return DAISY_FACTORY_OK;
}

/* Return the keyboard to a factory-default state, then reboot. Action-only.
 * Clears every BLE bond (and selects profile 0), reverts all ZMK Studio
 * keymap/layout modifications to the firmware defaults, exits factory mode, and
 * warm-reboots. Each step persists its own state, so the deferred reboot (same
 * ack-first pattern as handle_reboot) only reboots after the writes are done.
 * Runs on the system workqueue, the context the ZMK BLE/keymap APIs expect. */
static uint8_t handle_factory_reset(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    /* Unpair every profile (persisted immediately). This already selects
     * profile 0; do it explicitly too so the "back to slot 1" intent is local
     * to this handler and survives any change to clear_all_bonds. */
    zmk_ble_clear_all_bonds();
    zmk_ble_prof_select(0);
    /* prof_select debounces the active-profile save by
     * CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE (60 s); the reboot below fires long
     * before that, so force the save now or the previously-selected profile is
     * restored on the next boot (observed: reset ended up on slot 2). */
    zmk_ble_save_profile_immediate();
#endif

    /* Revert ZMK Studio keymap/layout changes to the firmware defaults -- the
     * same reset the Studio "restore stock settings" flow performs. Returns
     * -ENOTSUP when keymap settings storage isn't built in (nothing was
     * persisted, so there is nothing to clear); that is not a reset failure. */
    int err = zmk_keymap_reset_settings();
    if (err < 0 && err != -ENOTSUP) {
        LOG_ERR("factory reset: keymap settings reset failed: %d", err);
        return DAISY_FACTORY_ERR_HW;
    }

    /* Leave factory mode so normal sleep behavior resumes. The reboot below
     * clears it too (RAM-only), but make the intent explicit. */
    factory_mode = false;

    LOG_INF("factory reset: state cleared, rebooting");
    k_work_schedule(&reboot_work, K_MSEC(REBOOT_DELAY_MS));
    return DAISY_FACTORY_OK;
}

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
/* Standby entry runs deferred (same ack-first pattern as handle_reboot) so
 * the ack reaches the host before the SoC powers off. Mirrors the sleep path
 * in activity.c: suspend every device's PM hook, then System OFF. Wake is a
 * configured wake source (key press) and comes back as a full reboot. */
static void standby_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    LOG_INF("factory standby: entering System OFF (deep sleep)");
    if (zmk_pm_suspend_devices() < 0) {
        LOG_ERR("factory standby: failed to suspend all devices, staying awake");
        zmk_pm_resume_devices();
        return;
    }
    sys_poweroff();
}
static K_WORK_DELAYABLE_DEFINE(standby_work, standby_work_handler);
#endif

/* Enter standby (ZMK deep sleep / System OFF). Action-only; deferred so the
 * ack reaches the host first. */
static uint8_t handle_standby_enter(void) {
#if !IS_ENABLED(CONFIG_ZMK_SLEEP)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    k_work_schedule(&standby_work, K_MSEC(REBOOT_DELAY_MS));
    return DAISY_FACTORY_OK;
#endif
}

/* Set a pairing LED to a PWM duty cycle. req = [index, percent]. Action-only:
 * no response payload. */
static uint8_t handle_led_set_pwm(const uint8_t *req, uint8_t req_len) {
#if !HAS_PAIRING_LEDS
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 2) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t index = req[0];
    uint8_t percent = req[1];
    if (index >= PAIRING_LED_COUNT || percent > 100) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (!device_is_ready(pairing_leds)) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (led_set_brightness(pairing_leds, index, percent) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Set the backlight to a PWM duty cycle. req = [percent]. Action-only. */
static uint8_t handle_backlight_set_pwm(const uint8_t *req, uint8_t req_len) {
#if !HAS_BACKLIGHT
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t percent = req[0];
    if (percent > 100) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (!device_is_ready(backlight)) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (led_set_brightness(backlight, 0, percent) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Set a status-RGB color channel. req = [color, percent]. On PWM boards (DVT1)
 * percent is a real duty cycle; on GPIO boards (EVT) any nonzero percent is on. */
static uint8_t handle_rgb_set_pwm(const uint8_t *req, uint8_t req_len) {
#if !HAS_RGB
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 2) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t color = req[0];
    uint8_t percent = req[1];
    if (color > DAISY_FACTORY_RGB_BLUE || percent > 100) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
#if HAS_RGB_PWM
    if (!device_is_ready(rgb_leds)) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (led_set_brightness(rgb_leds, rgb_pwm_index[color], percent) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
#else /* HAS_RGB_GPIO */
    const struct gpio_dt_spec *spec = &rgb_gpio[color];
    if (!gpio_is_ready_dt(spec)) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (gpio_pin_set_dt(spec, percent ? 1 : 0) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
#endif
    return DAISY_FACTORY_OK;
#endif
}

/* Enter or exit factory mode (sleep inhibit). req = [enable]. Action-only. */
static uint8_t handle_factory_mode_set(const uint8_t *req, uint8_t req_len) {
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    factory_mode = req[0] != 0;
    LOG_INF("factory mode %s", factory_mode ? "entered" : "exited");
    /* Let the modules that stood down for the factory tool (touchpad power
     * follow, ...) re-converge on exit. */
    raise_daisy_factory_mode_changed((struct daisy_factory_mode_changed){.active = factory_mode});
    return DAISY_FACTORY_OK;
}

/* Turn the caps lock LED on or off. req = [on]. Action-only. */
static uint8_t handle_caps_led_set(const uint8_t *req, uint8_t req_len) {
#if !HAS_CAPS_LED
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (!gpio_is_ready_dt(&caps_led)) {
        return DAISY_FACTORY_ERR_HW;
    }
    /* Idempotent: the pin is normally configured by the gpio-leds driver, but
     * reconfigure as output here so the command works regardless of who set it
     * up (mirrors handle_gpio_get). */
    if (gpio_pin_configure_dt(&caps_led, GPIO_OUTPUT) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (gpio_pin_set_dt(&caps_led, req[0] ? 1 : 0) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Clear all BLE bonds: unpair every profile (persisted immediately) and switch
 * back to profile 0. Action-only. Runs on the system workqueue, same context
 * the &bt_clr behavior uses, so calling the ZMK BLE API directly is safe. */
static uint8_t handle_bt_clear_bonds(void) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    zmk_ble_clear_all_bonds();
    return DAISY_FACTORY_OK;
#endif
}

/* Unpair the active profile only. Action-only. Same workqueue-context note as
 * handle_bt_clear_bonds. */
static uint8_t handle_bt_unpair(void) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    zmk_ble_clear_bonds();
    return DAISY_FACTORY_OK;
#endif
}

/* Switch the active BLE profile. req = [index]. Action-only. */
static uint8_t handle_bt_prof_select(const uint8_t *req, uint8_t req_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    int err = zmk_ble_prof_select(req[0]);
    if (err == -ERANGE) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    return err ? DAISY_FACTORY_ERR_HW : DAISY_FACTORY_OK;
#endif
}

/* Cycle the active BLE profile. Action-only. */
static uint8_t handle_bt_prof_cycle(bool next) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    ARG_UNUSED(next);
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    int err = next ? zmk_ble_prof_next() : zmk_ble_prof_prev();
    return err ? DAISY_FACTORY_ERR_HW : DAISY_FACTORY_OK;
#endif
}

/* Report BLE state: active profile, per-profile connected/bonded masks, and
 * whether advertising is enabled. -> struct daisy_factory_bt_status. */
static uint8_t handle_bt_status(uint8_t *payload, uint8_t *out_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    struct daisy_factory_bt_status st = {
        .profile_count = ZMK_BLE_PROFILE_COUNT,
        .active_index = (uint8_t)zmk_ble_active_profile_index(),
        .flags = (zmk_ble_active_profile_is_connected() ? DAISY_FACTORY_BT_FLAG_ACTIVE_CONNECTED
                                                        : 0) |
                 (zmk_ble_active_profile_is_open() ? DAISY_FACTORY_BT_FLAG_ACTIVE_OPEN : 0) |
                 (zmk_ble_adv_enabled_get() ? DAISY_FACTORY_BT_FLAG_ADV_ENABLED : 0),
    };
    for (uint8_t i = 0; i < MIN(ZMK_BLE_PROFILE_COUNT, 8); i++) {
        if (zmk_ble_profile_is_connected(i)) {
            st.connected_mask |= BIT(i);
        }
        if (!zmk_ble_profile_is_open(i)) {
            st.bonded_mask |= BIT(i);
        }
    }
    memcpy(payload, &st, sizeof(st));
    *out_len = sizeof(st);
    return DAISY_FACTORY_OK;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_BLE)
/* Link-layer diagnostic counters, defined in this tree's zephyr fork
 * (subsys/bluetooth/controller: lll_adv.c and hal/nrf5/radio/radio.c).
 * Defined __weak here so this file still links against a zephyr without
 * those patches -- the weak zeros lose to the strong definitions when the
 * instrumented controller is present. `advertising_status` is ZMK's own
 * (app/src/ble.c), which is always there. */
__weak uint32_t zmk_diag_ci_seen;
__weak uint32_t zmk_diag_ci_accepted;
__weak uint32_t zmk_diag_ci_rl_not_allowed;
__weak uint32_t zmk_diag_ci_adva_bad;
__weak uint32_t zmk_diag_ci_tgta_bad;
__weak uint32_t zmk_diag_ci_rx_unresolved;
__weak uint32_t zmk_diag_ar_cfg;
__weak uint32_t zmk_diag_ar_no_bc;
__weak uint32_t zmk_diag_ar_end_timeout;
__weak uint32_t zmk_diag_ar_notresolved;
__weak uint32_t zmk_diag_ar_resolved;
extern enum advertising_type advertising_status;
#endif

/* Report BLE link-layer diagnostics: real advertising state (stealth vs open
 * vs off) and the controller's CONNECT_IND / address-resolver counters.
 * -> struct daisy_factory_bt_diag. */
static uint8_t handle_bt_diag(uint8_t *payload, uint8_t *out_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
/* Counters saturate at u16 on the wire so the struct fits the single-frame
 * payload; a diagnosis only needs "is this climbing", never the exact count. */
#define BT_DIAG_SAT(counter) sys_cpu_to_le16((uint16_t)MIN((counter), UINT16_MAX))
    struct daisy_factory_bt_diag diag = {
        .adv_status = (uint8_t)advertising_status,
        .adv_phase = zmk_ble_adv_phase_diag(),
        .ci_seen = BT_DIAG_SAT(zmk_diag_ci_seen),
        .ci_accepted = BT_DIAG_SAT(zmk_diag_ci_accepted),
        .ci_rl_not_allowed = BT_DIAG_SAT(zmk_diag_ci_rl_not_allowed),
        .ci_adva_bad = BT_DIAG_SAT(zmk_diag_ci_adva_bad),
        .ci_tgta_bad = BT_DIAG_SAT(zmk_diag_ci_tgta_bad),
        .ci_rx_unresolved = BT_DIAG_SAT(zmk_diag_ci_rx_unresolved),
        .ar_configured = BT_DIAG_SAT(zmk_diag_ar_cfg),
        .ar_no_bitcount = BT_DIAG_SAT(zmk_diag_ar_no_bc),
        .ar_end_timeout = BT_DIAG_SAT(zmk_diag_ar_end_timeout),
        .ar_notresolved = BT_DIAG_SAT(zmk_diag_ar_notresolved),
        .ar_resolved = BT_DIAG_SAT(zmk_diag_ar_resolved),
    };
#undef BT_DIAG_SAT
    BUILD_ASSERT(sizeof(diag) <= DAISY_FACTORY_PAYLOAD_SIZE,
                 "bt_diag response must fit a single frame");

    memcpy(payload, &diag, sizeof(diag));
    *out_len = sizeof(diag);
    return DAISY_FACTORY_OK;
#endif
}

/* Enable/disable BLE advertising. req = [enable]. Action-only. */
static uint8_t handle_bt_adv_set(const uint8_t *req, uint8_t req_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    zmk_ble_adv_enabled_set(req[0] != 0);
    return DAISY_FACTORY_OK;
#endif
}

/* Report the keyboard's BLE addresses: the static identity (scan-report address
 * when privacy is off) and the address currently being advertised (a rotating
 * RPA when privacy is on). -> struct daisy_factory_bt_addr. All ZMK profiles
 * share the one identity. */
static uint8_t handle_bt_addr(uint8_t *payload, uint8_t *out_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    bt_addr_le_t ids[CONFIG_BT_ID_MAX];
    size_t count = ARRAY_SIZE(ids);

    bt_id_get(ids, &count);
    if (count == 0) {
        /* Stack not ready / no identity yet. */
        return DAISY_FACTORY_ERR_HW;
    }

    struct daisy_factory_bt_addr out = {
        .identity_type = ids[BT_ID_DEFAULT].type,
        .privacy = IS_ENABLED(CONFIG_BT_PRIVACY) ? 1 : 0,
    };
    memcpy(out.identity_val, ids[BT_ID_DEFAULT].a.val, sizeof(out.identity_val));

    /* Current advertised address. With privacy off this is the identity; with
     * privacy on bt_le_oob_get_local yields the current RPA. NOTE: with privacy
     * on this call may regenerate the RPA -- validate on hardware that it
     * matches the address actually on air before relying on it for pairing. */
    struct bt_le_oob oob;
    if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob) == 0) {
        out.current_type = oob.addr.type;
        memcpy(out.current_val, oob.addr.a.val, sizeof(out.current_val));
    } else {
        /* Fall back to the identity if the current address can't be read. */
        out.current_type = out.identity_type;
        memcpy(out.current_val, out.identity_val, sizeof(out.current_val));
    }

    memcpy(payload, &out, sizeof(out));
    *out_len = sizeof(out);
    return DAISY_FACTORY_OK;
#endif
}

/* Report the detail for one BLE profile (bond slot). req = [index]. Reports the
 * bonded peer address for that slot (which host it is paired with) plus the
 * per-slot bonded/connected/active flags. -> struct daisy_factory_bt_profile. */
static uint8_t handle_bt_profile_get(const uint8_t *req, uint8_t req_len, uint8_t *payload,
                                     uint8_t *out_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t index = req[0];
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }

    bt_addr_le_t *peer = zmk_ble_profile_address(index);
    struct daisy_factory_bt_profile out = {
        .index = index,
        .addr_type = peer->type,
    };
    memcpy(out.addr_val, peer->a.val, sizeof(out.addr_val));
    if (!zmk_ble_profile_is_open(index)) {
        out.flags |= DAISY_FACTORY_BT_PROF_FLAG_BONDED;
    }
    if (zmk_ble_profile_is_connected(index)) {
        out.flags |= DAISY_FACTORY_BT_PROF_FLAG_CONNECTED;
    }
    if (index == (uint8_t)zmk_ble_active_profile_index()) {
        out.flags |= DAISY_FACTORY_BT_PROF_FLAG_ACTIVE;
    }

    memcpy(payload, &out, sizeof(out));
    *out_len = sizeof(out);
    return DAISY_FACTORY_OK;
#endif
}

/* Relay a host-displayed passkey to a pending passkey-entry pairing.
 * req = [passkey u32 little-endian]. Action-only. */
static uint8_t handle_bt_passkey(const uint8_t *req, uint8_t req_len) {
#if !IS_ENABLED(CONFIG_ZMK_BLE)
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 4) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint32_t passkey = sys_get_le32(req);
    if (passkey > 999999) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    int err = zmk_ble_passkey_entry(passkey);
    if (err == -ENOTSUP) {
        return DAISY_FACTORY_ERR_UNSUPPORTED;
    } else if (err == -ENOTCONN) {
        /* Nothing is awaiting a passkey right now. */
        return DAISY_FACTORY_ERR_HW;
    } else if (err) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Report HID endpoint routing. -> [preferred, selected, ble_profile]. */
static uint8_t handle_endpoint_get(uint8_t *payload, uint8_t *out_len) {
    struct zmk_endpoint_instance selected = zmk_endpoint_get_selected();
    payload[0] = (uint8_t)zmk_endpoint_get_preferred_transport();
    payload[1] = (uint8_t)selected.transport;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    payload[2] = selected.transport == ZMK_TRANSPORT_BLE ? selected.ble.profile_index : 0;
#else
    payload[2] = 0;
#endif
    *out_len = 3;
    return DAISY_FACTORY_OK;
}

/* Set the preferred transport (persisted). req = [transport]. Action-only.
 * Routing only: USB stays enumerated (and this interface reachable) either way. */
static uint8_t handle_endpoint_set(const uint8_t *req, uint8_t req_len) {
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (req[0] > DAISY_FACTORY_TRANSPORT_BLE) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (zmk_endpoint_set_preferred_transport((enum zmk_transport)req[0]) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
}

/* Tap a key on the HID keyboard/keypad page so the host receives a real
 * keystroke over whichever endpoint is selected. req = [usage]. Action-only.
 * The short sleep between press and release keeps the two reports distinct;
 * blocking the system workqueue that long is fine in a factory context. */
static uint8_t handle_key_inject(const uint8_t *req, uint8_t req_len) {
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (zmk_hid_keyboard_press(req[0]) < 0) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    int err = zmk_endpoint_send_report(HID_USAGE_KEY);
    k_msleep(20);
    if (zmk_hid_keyboard_release(req[0]) < 0 ||
        zmk_endpoint_send_report(HID_USAGE_KEY) < 0 || err < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
}

/* Set the touchpad power state. req = [state (daisy_factory_touchpad_power)].
 * Action-only, idempotent. Delegates to daisy_touchpad_power_set() in
 * touchpad_power.c (shared with the automatic host-follow logic; the state
 * values match by construction). While factory mode is active the host-follow
 * logic stands down, so a state set here sticks for the whole test run. */
static uint8_t handle_touchpad_power_set(const uint8_t *req, uint8_t req_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (req[0] > DAISY_FACTORY_TOUCHPAD_SLEEP) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (daisy_touchpad_power_set((enum daisy_touchpad_power)req[0]) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Read a raw 8-bit touchpad register. req = [reg]; payload out = [value].
 * Debug/bring-up aid for the PCT1036 vendor register space. */
static uint8_t handle_touchpad_reg_read(const uint8_t *req, uint8_t req_len, uint8_t *payload,
                                        uint8_t *out_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t val;
    if (hid_touchpad_reg_read(touchpad, req[0], &val) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    payload[0] = val;
    *out_len = 1;
    return DAISY_FACTORY_OK;
#endif
}

/* Write a raw 8-bit touchpad register. req = [reg, value]. Action-only. */
static uint8_t handle_touchpad_reg_write(const uint8_t *req, uint8_t req_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 2) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    if (hid_touchpad_reg_write(touchpad, req[0], req[1]) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* --- synthetic touchpad input (0x93-0x95) --------------------------------
 *
 * All three commands funnel into hid_touchpad_inject_input(), which hands the
 * frame to the passthrough backends at the same point the driver's I2C read
 * path does. So the frame takes the real route out: endpoint selection, BLE
 * pacing/urgency, USB TX semaphore -- see factory.h's group 0x9 notes.
 *
 * Handlers run on process_q, which may block (the USB backend waits on its TX
 * semaphore), so building a frame here is fine. The sweep is the exception: it
 * has to keep a cadence for up to seconds, so it gets its own queue.
 */
#if HAS_TOUCHPAD

/* Assemble a PTP (id 4) frame's fingers/count/buttons in place and stamp it
 * with the current uptime, then inject it. `frame` must be
 * DAISY_FACTORY_TP_PTP_REPORT_LEN bytes with the contact records and trailer
 * already filled except for scan time. */
static void tp_inject_ptp_frame(uint8_t *frame) {
    /* Descriptor unit exponent -4: scan time counts 100 us ticks. It wraps
     * every 6.55 s, which is what a real pad's 16-bit counter does too.
     *
     * Off the tick counter, not k_uptime_get(): that returns MILLISECONDS, so
     * stamping from it quantized the field to 1 ms and made a 7.5 ms cadence
     * read back as an alternating 7/8 -- indistinguishable, in the device
     * timeline a host or a test analyses, from the pad actually pacing
     * unevenly. Ticks are 32 us here (31250/s), comfortably finer than the
     * 100 us unit. */
    uint16_t scan = (uint16_t)((k_ticks_to_us_floor64(k_uptime_ticks()) / 100) & 0xFFFF);
    sys_put_le16(scan, &frame[DAISY_FACTORY_TP_PTP_SCAN_OFF]);
    hid_touchpad_inject_input(touchpad, DAISY_FACTORY_TP_REPORT_ID_PTP, frame,
                              DAISY_FACTORY_TP_PTP_REPORT_LEN);
}

/* Staging buffer for INJECT_RAW. Only ever touched from process_q (one
 * request outstanding at a time), so it needs no lock. `staged_id` remembers
 * which report the partial buffer belongs to, so a SEND can't complete a
 * sequence some other report started. */
static uint8_t tp_stage_buf[DAISY_FACTORY_TP_INJECT_MAX_REPORT];
static uint8_t tp_stage_id;
static bool tp_stage_valid;

/* Sweep generator. Its own queue and thread: it must hold a cadence for the
 * whole run, and the inject call it makes can block on the USB backend's TX
 * semaphore -- on process_q that would either be blocked by, or block, an
 * unrelated command, and on the system workqueue it would contend with the
 * ZMK HID submit paths that already stall it for up to 100 ms. */
static K_THREAD_STACK_DEFINE(tp_sweep_q_stack, 1024);
static struct k_work_q tp_sweep_q;
#define TP_SWEEP_Q_PRIORITY K_PRIO_PREEMPT(4)

static void tp_sweep_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tp_sweep_work, tp_sweep_work_handler);

/* Written by the handler (process_q) before the work is scheduled and read
 * only by the sweep thread afterwards; a restart cancels the work first, so
 * the two never touch this concurrently. */
static struct {
    uint16_t x0, y0, x1, y1;
    uint16_t frames;
    uint8_t flags;
    uint16_t next;       /* index of the frame to emit, 0..frames (frames = lift-off) */
    k_ticks_t anchor;    /* uptime ticks of frame 0 */
    k_ticks_t step;      /* ticks between frames */
} tp_sweep;

/* Linear interpolation across `frames` positions, endpoints included. The
 * division is over frames-1 so the last motion frame lands exactly on
 * (x1,y1); a single-frame sweep is just (x0,y0). Rounded, so a short sweep
 * doesn't bias every step towards the start. */
static uint16_t tp_sweep_lerp(uint16_t a, uint16_t b, uint16_t i, uint16_t frames) {
    if (frames <= 1) {
        return a;
    }
    int32_t span = (int32_t)b - (int32_t)a;
    int32_t steps = (int32_t)frames - 1;
    int32_t num = span * (int32_t)i;
    /* Round half away from zero without floating point. */
    int32_t delta = (num >= 0) ? (num + steps / 2) / steps : (num - steps / 2) / steps;
    return (uint16_t)((int32_t)a + delta);
}

static void tp_sweep_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    uint8_t frame[DAISY_FACTORY_TP_PTP_REPORT_LEN] = {0};
    uint16_t i = tp_sweep.next;

    if (i < tp_sweep.frames) {
        /* One contact in slot 0: confident, tip down, contact id 0. */
        frame[0] = DAISY_FACTORY_TP_PTP_CONFIDENCE | DAISY_FACTORY_TP_PTP_TIP;
        sys_put_le16(tp_sweep_lerp(tp_sweep.x0, tp_sweep.x1, i, tp_sweep.frames), &frame[1]);
        sys_put_le16(tp_sweep_lerp(tp_sweep.y0, tp_sweep.y1, i, tp_sweep.frames), &frame[3]);
        frame[DAISY_FACTORY_TP_PTP_COUNT_OFF] = 1;
        frame[DAISY_FACTORY_TP_PTP_BTN_OFF] =
            (tp_sweep.flags & DAISY_FACTORY_TP_SWEEP_BUTTON) ? 1 : 0;
    } else {
        /* Lift-off: confidence/tip clear, buttons released and the contact
         * count down to 0, which is what a real pad sends on release. The
         * zeroed frame is all of that already; spell the count out anyway,
         * since it is the byte a host actually keys the release off. */
        frame[DAISY_FACTORY_TP_PTP_COUNT_OFF] = 0;
    }
    tp_inject_ptp_frame(frame);

    tp_sweep.next = i + 1;
    bool more = (tp_sweep.flags & DAISY_FACTORY_TP_SWEEP_NO_LIFT)
                    ? (tp_sweep.next < tp_sweep.frames)
                    : (tp_sweep.next <= tp_sweep.frames);
    if (more) {
        /* Absolute deadline off the one anchor: per-frame scheduling latency
         * (the inject can block on the USB TX semaphore) then costs a late
         * frame, not a permanently shifted grid. A frame whose deadline has
         * already passed goes out immediately, so the run still ends on time
         * with a compressed gap rather than stretching. */
        k_work_reschedule_for_queue(
            &tp_sweep_q, &tp_sweep_work,
            K_TIMEOUT_ABS_TICKS(tp_sweep.anchor + (k_ticks_t)tp_sweep.next * tp_sweep.step));
    }
}

#endif /* HAS_TOUCHPAD */

/* Stage raw report bytes, and dispatch the buffer if SEND is set.
 * req = [report_id, offset, flags, bytes...]. */
static uint8_t handle_touchpad_inject_raw(const uint8_t *req, uint8_t req_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 3) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t report_id = req[0];
    uint8_t offset = req[1];
    uint8_t flags = req[2];
    uint8_t n = req_len - 3;

    if ((size_t)offset + n > sizeof(tp_stage_buf)) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    /* A sequence must stay on one report id: continuing (or completing) a
     * partial buffer with a different id would silently send a mixture. */
    if (tp_stage_valid && offset != 0 && report_id != tp_stage_id) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    memcpy(&tp_stage_buf[offset], &req[3], n);
    tp_stage_id = report_id;
    tp_stage_valid = true;

    if (flags & DAISY_FACTORY_TP_INJECT_SEND) {
        hid_touchpad_inject_input(touchpad, report_id, tp_stage_buf, offset + n);
        /* Buffer contents survive on purpose (factory.h): the next send can
         * rewrite only the bytes that moved. Only the id binding is released. */
        tp_stage_valid = false;
    }
    return DAISY_FACTORY_OK;
#endif
}

/* Build one PTP frame from a compact finger list and inject it.
 * req = [contact_count, buttons, n_fingers, n x (status, x u16, y u16)]. */
static uint8_t handle_touchpad_inject_ptp(const uint8_t *req, uint8_t req_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 3) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t n_fingers = req[2];
    if (n_fingers > DAISY_FACTORY_TP_INJECT_PTP_MAX_FINGERS) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (req_len < 3 + n_fingers * DAISY_FACTORY_TP_PTP_STRIDE) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }

    uint8_t frame[DAISY_FACTORY_TP_PTP_REPORT_LEN] = {0};
    /* Contact records are laid out identically on the wire and in the report,
     * so the finger list copies straight across; unlisted slots stay zero,
     * which is how a real pad reports a lifted contact. */
    memcpy(frame, &req[3], n_fingers * DAISY_FACTORY_TP_PTP_STRIDE);
    frame[DAISY_FACTORY_TP_PTP_COUNT_OFF] = req[0];
    frame[DAISY_FACTORY_TP_PTP_BTN_OFF] = req[1];
    tp_inject_ptp_frame(frame);
    return DAISY_FACTORY_OK;
#endif
}


/* Start (or replace) a sweep. req = [x0 u16, y0 u16, x1 u16, y1 u16,
 * frames u16, interval_us u32, flags u8]; payload out = [duration_ms u32].
 * Returns before the first frame goes out. */
static uint8_t handle_touchpad_inject_sweep(const uint8_t *req, uint8_t req_len, uint8_t *payload,
                                            uint8_t *out_len) {
#if !HAS_TOUCHPAD
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 15) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint16_t frames = sys_get_le16(&req[8]);
    uint32_t interval_us = sys_get_le32(&req[10]);

    /* A run already in flight is cancelled either way: frames == 0 is the
     * documented cancel, and a new sweep replaces the old one. Cancel first
     * so the work handler can't fire between here and the state update. */
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&tp_sweep_work, &sync);
    if (frames == 0) {
        *out_len = 4;
        sys_put_le32(0, payload);
        return DAISY_FACTORY_OK;
    }

    if (frames > DAISY_FACTORY_TP_INJECT_SWEEP_MAX_FRAMES ||
        interval_us < DAISY_FACTORY_TP_INJECT_SWEEP_MIN_US ||
        interval_us > DAISY_FACTORY_TP_INJECT_SWEEP_MAX_US) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }

    tp_sweep.x0 = sys_get_le16(&req[0]);
    tp_sweep.y0 = sys_get_le16(&req[2]);
    tp_sweep.x1 = sys_get_le16(&req[4]);
    tp_sweep.y1 = sys_get_le16(&req[6]);
    tp_sweep.frames = frames;
    tp_sweep.flags = req[14];
    tp_sweep.next = 0;
    tp_sweep.step = k_us_to_ticks_ceil64(interval_us);
    tp_sweep.anchor = k_uptime_ticks();

    uint16_t total = frames + ((tp_sweep.flags & DAISY_FACTORY_TP_SWEEP_NO_LIFT) ? 0 : 1);
    /* Duration to the last frame's deadline: (total - 1) gaps, measured on the
     * grid actually used. The step was rounded UP to whole ticks (32 us here),
     * so reporting the requested interval instead would under-state the run --
     * 7500 us becomes 7520, i.e. 8 ms short over 400 frames, and a host that
     * waits out the returned value would stop capturing mid-sweep. */
    uint64_t step_us = k_ticks_to_us_ceil64(tp_sweep.step);
    sys_put_le32((uint32_t)(((uint64_t)(total - 1) * step_us + 999) / 1000), payload);
    *out_len = 4;

    k_work_reschedule_for_queue(&tp_sweep_q, &tp_sweep_work, K_NO_WAIT);
    return DAISY_FACTORY_OK;
#endif
}

/* Read a named GPIO's logical level. req = [gpio_id]; payload out = [level].
 * The level is gpio_pin_get_dt (active-low aware): 1 = asserted. */
static uint8_t handle_gpio_get(const uint8_t *req, uint8_t req_len, uint8_t *payload,
                               uint8_t *out_len) {
    if (req_len < 1) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }

    const struct gpio_dt_spec *spec;
    switch (req[0]) {
#if HAS_PAIRING_BUTTON
    case DAISY_FACTORY_GPIO_PAIRING_BUTTON:
        spec = &pairing_button;
        break;
#endif
#if HAS_PROTOCOL_SWITCH
    case DAISY_FACTORY_GPIO_PROTOCOL_SWITCH:
        spec = &protocol_switch;
        break;
#endif
#if HAS_LAYOUT_STRAPS
    case DAISY_FACTORY_GPIO_ISO_STRAP:
        spec = &iso_strap;
        break;
    case DAISY_FACTORY_GPIO_ANSI_STRAP:
        spec = &ansi_strap;
        break;
#endif
    default:
        return DAISY_FACTORY_ERR_UNSUPPORTED;
    }

    if (!gpio_is_ready_dt(spec)) {
        return DAISY_FACTORY_ERR_HW;
    }
    /* Idempotent: configure as input each read so the query works whether or
     * not another driver (e.g. gpio-keys) already owns the pin. */
    if (gpio_pin_configure_dt(spec, GPIO_INPUT) < 0) {
        return DAISY_FACTORY_ERR_HW;
    }
    int level = gpio_pin_get_dt(spec);
    if (level < 0) {
        return DAISY_FACTORY_ERR_HW;
    }

    payload[0] = level ? 1 : 0;
    *out_len = 1;
    return DAISY_FACTORY_OK;
}

#if HAS_PERT
/* Validate the [chan, phy] prefix every PERT start command carries.
 * ll_test_tx/rx index tables by chan and phy with NO bounds check of their
 * own, so out-of-range values must be rejected here. */
static uint8_t pert_check_chan_phy(uint8_t chan, uint8_t phy) {
    if (chan > PERT_MAX_CHANNEL) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    if (phy != DAISY_FACTORY_PERT_PHY_1M && phy != DAISY_FACTORY_PERT_PHY_2M) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    return DAISY_FACTORY_OK;
}

/* Pre-warm the HFXO before a DTM start. The controller's DTM init requests
 * the HF clock through the generic onoff manager (lll_hfclock_on_wait) and
 * hard-asserts when the ready notification doesn't arrive within a few ms;
 * on nRF54LM20A that notification never comes when the XO has to be started
 * from cold in that path (found via the crash breadcrumb: UDF at
 * lll_test.c:508, the LL_ASSERT after lll_hfclock_on_wait). Normal BLE never
 * hits this because the LL uses the z_nrf_clock_bt_ctlr_hf_* fast path
 * between events. Request the XO through that same fast path and wait until
 * it reports high accuracy: the DTM init's generic request then takes the
 * driver's synchronous already-started branch (generic_hfclk_start in
 * clock_control_nrf.c checks "BT user holds it AND it runs at high
 * accuracy") and needs no notification at all. The controller's TEST_END
 * releases the BT-user flag again; re-warming before every start keeps a
 * channel sweep safe, and the post-test reboot cleans up whatever remains. */
static struct k_sem pert_hfxo_sem;
static struct onoff_client pert_hfxo_cli;
static bool pert_hfxo_granted;

static void pert_hfxo_ready(struct onoff_manager *mgr, struct onoff_client *cli, uint32_t state,
                            int res) {
    ARG_UNUSED(mgr);
    ARG_UNUSED(cli);
    ARG_UNUSED(state);
    ARG_UNUSED(res);
    k_sem_give(&pert_hfxo_sem);
}

static uint8_t pert_hfxo_prewarm(void) {
    nrf_clock_hfclk_t type = NRF_CLOCK_HFCLK_LOW_ACCURACY;

    if (pert_hfxo_granted) {
        return DAISY_FACTORY_OK;
    }

    /* Step 1: physically start the XO via the BT fast path, so the generic
     * driver's start op below takes its synchronous already-started branch
     * (no dependence on the start-notification IRQ). */
    z_nrf_clock_bt_ctlr_hf_request();
    for (int i = 0; i < 100; i++) {
        (void)nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &type);
        if (type == NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
            break;
        }
        k_msleep(1);
    }
    if (type != NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
        LOG_ERR("pert: HFXO did not reach high accuracy");
        return DAISY_FACTORY_ERR_HW;
    }

#if defined(NRF54LM20A_XXAA)
    /* MLTPAN-39: on nRF54LM20A the radio's PLL must be started explicitly or
     * TX produces no usable RF (the state machine runs, nothing decodable on
     * air). Same workaround the radio_test sample applies after its clock
     * init; nothing in the split controller or the clock driver does it. */
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

    /* Step 2: take (and hold) a generic onoff grant on the HF service, so
     * the controller's own request inside the DTM start finds the service
     * ON with a nonzero count and is notified synchronously. If the manager
     * sits in ONOFF_STATE_ERROR (a previously failed transition poisons it:
     * every request returns an error until a reset), reset it and retry --
     * this is the failure the breadcrumb kept pointing at (UDF on the
     * LL_ASSERT after lll_hfclock_on_wait, lll_test.c:508). Held until the
     * post-test reboot. */
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    k_sem_init(&pert_hfxo_sem, 0, 1);
    for (int attempt = 0; attempt < 2; attempt++) {
        sys_notify_init_callback(&pert_hfxo_cli.notify, pert_hfxo_ready);

        /* Requested with interrupts locked and HF_USER_BT freshly set, so
         * generic_hfclk_start() (clock_control_nrf.c:374) is guaranteed to
         * observe the crystal as already started and notify synchronously.
         *
         * Without the lock the link layer can clear HF_USER_BT in between --
         * it releases the BT user at the end of every radio event -- and the
         * generic start then falls through to waiting on an XOSTARTED event,
         * which the hardware does not produce for an XO that is already
         * running. The request is accepted and simply never completes, which
         * is the "grant not signalled" failure. Same trap as the RF path's
         * HFXO wait; see rf_clock_init() in factory_rf.c. */
        unsigned int key = irq_lock();
        z_nrf_clock_bt_ctlr_hf_request();
        int err = onoff_request(mgr, &pert_hfxo_cli);
        irq_unlock(key);

        if (err >= 0) {
            if (k_sem_take(&pert_hfxo_sem, K_MSEC(100)) == 0) {
                pert_hfxo_granted = true;
                LOG_INF("pert: HFXO running, generic grant held");
                return DAISY_FACTORY_OK;
            }
            (void)nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &type);
            LOG_ERR("pert: HF clock grant not signalled (high_acc=%d onoff_err=%d)",
                    type == NRF_CLOCK_HFCLK_HIGH_ACCURACY, onoff_has_error(mgr));
            /* Drop the client that never completed before trying again;
             * leaving it registered would corrupt the manager's client list. */
            (void)onoff_cancel(mgr, &pert_hfxo_cli);
            continue;
        }
        LOG_WRN("pert: HF onoff request failed (%d), resetting the service", err);
        sys_notify_init_callback(&pert_hfxo_cli.notify, pert_hfxo_ready);
        err = onoff_reset(mgr, &pert_hfxo_cli);
        if (err < 0) {
            LOG_ERR("pert: HF onoff reset failed: %d", err);
            return DAISY_FACTORY_ERR_HW;
        }
        (void)k_sem_take(&pert_hfxo_sem, K_MSEC(100));
    }
    LOG_ERR("pert: HF clock service unusable");
    return DAISY_FACTORY_ERR_HW;
}

/* Run one HCI command against the local controller through the Bluetooth
 * host. Blocks until the command completes (fine on the sysworkq in a
 * factory context; the dongle firmware does the same from its main thread).
 * Returns a factory status code; on success *rsp (when requested) holds the
 * command-complete parameters and must be net_buf_unref'd by the caller. */
static uint8_t pert_hci_cmd(uint16_t opcode, const void *params, size_t params_len,
                            struct net_buf **rsp) {
    struct net_buf *buf = bt_hci_cmd_alloc(K_FOREVER);
    if (buf == NULL) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (params_len > 0) {
        net_buf_add_mem(buf, params, params_len);
    }
    int err = bt_hci_cmd_send_sync(opcode, buf, rsp);
    if (err) {
        LOG_ERR("pert: HCI cmd 0x%04x failed: %d", opcode, err);
        return DAISY_FACTORY_ERR_HW;
    }
    return DAISY_FACTORY_OK;
}
#endif

/* Quiesce the BLE link layer for DTM: disconnect every profile and stop
 * advertising (the host stack itself stays up -- the DTM commands travel
 * through it as HCI commands). Action-only. The disconnects complete
 * asynchronously; the host polls BT_STATUS until connected_mask == 0 and
 * ADV_ENABLED clears. Recovery after the test run is a REBOOT. */
static uint8_t handle_pert_quiesce(void) {
#if !HAS_PERT
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    zmk_ble_adv_enabled_set(false);
    return DAISY_FACTORY_OK;
#endif
}

/* Start DTM transmit. req = [chan, phy, len, pattern, tx_power?]; transmits
 * continuously until PERT_END. Action-only.
 *
 * Uses LE_TX_TEST_V4 rather than the enhanced command: the enhanced handler
 * hardcodes POWER_MAX_SET (+8 dBm on nRF54LM20A), a TXPOWER value nothing
 * else on this board exercises; V4 carries an explicit tx_power. Default is
 * 0 dBm, matching the product's operating power. */
static uint8_t handle_pert_tx_start(const uint8_t *req, uint8_t req_len) {
#if !HAS_PERT
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 4) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t status = pert_check_chan_phy(req[0], req[1]);
    if (status != DAISY_FACTORY_OK) {
        return status;
    }
    /* PRBS15's lookup table is stubbed in the split controller; values past
     * 01010101 are undefined in the HCI encoding. */
    if (req[3] == DAISY_FACTORY_PERT_PAT_PRBS15 || req[3] > DAISY_FACTORY_PERT_PAT_01010101) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    int8_t tx_power = (req_len >= 5) ? (int8_t)req[4] : 0;
    status = pert_hfxo_prewarm();
    if (status != DAISY_FACTORY_OK) {
        return status;
    }
    /* V4 params: fixed struct + ant_ids[switch_pattern_len] (empty here) +
     * trailing tx_power byte. */
    struct {
        struct bt_hci_cp_le_tx_test_v4 cp;
        struct bt_hci_cp_le_tx_test_v4_tx_power power;
    } __packed cp = {
        .cp = {
            .tx_ch = req[0],
            .test_data_len = req[2],
            .pkt_payload = req[3],
            .phy = req[1],
            .cte_len = BT_HCI_LE_TEST_CTE_DISABLED,
            .cte_type = BT_HCI_LE_TEST_CTE_TYPE_ANY,
            .switch_pattern_len = BT_HCI_LE_TEST_SWITCH_PATTERN_LEN_ANY,
        },
        .power = {.tx_power = tx_power},
    };
    uint8_t status2 = pert_hci_cmd(BT_HCI_OP_LE_TX_TEST_V4, &cp, sizeof(cp), NULL);
    if (status2 != DAISY_FACTORY_OK) {
        return status2;
    }
    LOG_INF("pert: TX started (chan %u, phy %u, len %u, pattern %u, %d dBm)", req[0], req[1],
            req[2], req[3], tx_power);
    return DAISY_FACTORY_OK;
#endif
}

/* Start DTM receive. req = [chan, phy]; counts packets until PERT_END.
 * Action-only. */
static uint8_t handle_pert_rx_start(const uint8_t *req, uint8_t req_len) {
#if !HAS_PERT
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (req_len < 2) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }
    uint8_t status = pert_check_chan_phy(req[0], req[1]);
    if (status != DAISY_FACTORY_OK) {
        return status;
    }
    status = pert_hfxo_prewarm();
    if (status != DAISY_FACTORY_OK) {
        return status;
    }
    struct bt_hci_cp_le_enh_rx_test cp = {
        .rx_ch = req[0],
        .phy = req[1],
        .mod_index = BT_HCI_LE_MOD_INDEX_STANDARD,
    };
    uint8_t status2 = pert_hci_cmd(BT_HCI_OP_LE_ENH_RX_TEST, &cp, sizeof(cp), NULL);
    if (status2 != DAISY_FACTORY_OK) {
        return status2;
    }
    LOG_INF("pert: RX started (chan %u, phy %u)", req[0], req[1]);
    return DAISY_FACTORY_OK;
#endif
}

/* End the running DTM test. -> [num_rx u16 LE], the packets received since
 * PERT_RX_START (0 after a TX test). */
static uint8_t handle_pert_end(uint8_t *payload, uint8_t *out_len) {
#if !HAS_PERT
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    struct net_buf *rsp = NULL;
    uint8_t status = pert_hci_cmd(BT_HCI_OP_LE_TEST_END, NULL, 0, &rsp);
    if (status != DAISY_FACTORY_OK) {
        return status;
    }
    struct bt_hci_rp_le_test_end *rp = (struct bt_hci_rp_le_test_end *)rsp->data;
    uint16_t num_rx = sys_le16_to_cpu(rp->rx_pkt_count);
    net_buf_unref(rsp);
    LOG_INF("pert: test ended, num_rx %u", num_rx);
    sys_put_le16(num_rx, payload);
    *out_len = sizeof(num_rx);
    return DAISY_FACTORY_OK;
#endif
}

/* RF test modes (group 0xB). The handlers themselves live in factory_rf.c --
 * they pull in the radio_test driver, which is a lot of machinery to have
 * inline here. These wrappers exist so the dispatch table reads the same as
 * every other group and so a build without CONFIG_DAISY_FACTORY_RF still
 * answers the opcodes (with UNSUPPORTED) rather than UNKNOWN_CMD. */
static uint8_t handle_rf_enter(void) {
#if !HAS_RF
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    return daisy_factory_rf_enter();
#endif
}

static uint8_t handle_rf_start(const uint8_t *req, uint8_t req_len) {
#if !HAS_RF
    ARG_UNUSED(req);
    ARG_UNUSED(req_len);
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    return daisy_factory_rf_start(req, req_len);
#endif
}

static uint8_t handle_rf_stop(uint8_t *payload, uint8_t *out_len) {
#if !HAS_RF
    ARG_UNUSED(payload);
    ARG_UNUSED(out_len);
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    return daisy_factory_rf_stop(payload, out_len);
#endif
}

static uint8_t handle_rf_status(uint8_t *payload, uint8_t *out_len) {
#if !HAS_RF
    ARG_UNUSED(payload);
    ARG_UNUSED(out_len);
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    return daisy_factory_rf_status(payload, out_len);
#endif
}

static void process_work_handler(struct k_work *work) {
    if (!hid_ready) {
        return;
    }

    const uint8_t cmd = req_buf[DAISY_FACTORY_OFF_CMD];
    const uint8_t seq = req_buf[DAISY_FACTORY_OFF_SEQ];

    uint8_t payload[DAISY_FACTORY_PAYLOAD_SIZE] = {0};
    uint8_t payload_len = 0;
    uint8_t status;

    switch (cmd) {
    case DAISY_FACTORY_CMD_INFO:
        status = handle_info(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_PING:
        payload_len = MIN(req_buf[DAISY_FACTORY_OFF_LEN], DAISY_FACTORY_PAYLOAD_SIZE);
        memcpy(payload, &req_buf[DAISY_FACTORY_OFF_PAYLOAD], payload_len);
        status = DAISY_FACTORY_OK;
        break;
    case DAISY_FACTORY_CMD_DEVICE_ID:
        status = handle_device_id(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_CRASH_INFO:
        status = handle_crash_info(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                   req_buf[DAISY_FACTORY_OFF_LEN], payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BATTERY_TEMP:
        status = handle_battery_temp(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BATTERY_VOLTAGE:
        status = handle_battery_voltage(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BATTERY_CURRENT:
        status = handle_battery_current(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_CHARGING_SET:
        status = handle_charging_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                     req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_CHARGING_STATUS:
        status = handle_charging_status(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_CHARGING_CLEAR_ERROR:
        status = handle_charging_clear_error();
        break;
    case DAISY_FACTORY_CMD_SHIP_MODE:
        status = handle_ship_mode();
        break;
    case DAISY_FACTORY_CMD_FACTORY_MODE_SET:
        status = handle_factory_mode_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                         req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_LED_SET_PWM:
        status = handle_led_set_pwm(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                    req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_BACKLIGHT_SET_PWM:
        status = handle_backlight_set_pwm(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                          req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_RGB_SET_PWM:
        status = handle_rgb_set_pwm(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                    req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_CAPS_LED_SET:
        status = handle_caps_led_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                     req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_BOOTLOADER_JUMP:
        status = handle_bootloader_jump();
        break;
    case DAISY_FACTORY_CMD_REBOOT:
        status = handle_reboot();
        break;
    case DAISY_FACTORY_CMD_FACTORY_RESET:
        status = handle_factory_reset();
        break;
    case DAISY_FACTORY_CMD_STANDBY_ENTER:
        status = handle_standby_enter();
        break;
    case DAISY_FACTORY_CMD_BT_CLEAR_BONDS:
        status = handle_bt_clear_bonds();
        break;
    case DAISY_FACTORY_CMD_BT_UNPAIR:
        status = handle_bt_unpair();
        break;
    case DAISY_FACTORY_CMD_BT_PROF_SELECT:
        status = handle_bt_prof_select(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                       req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_BT_PROF_NEXT:
        status = handle_bt_prof_cycle(true);
        break;
    case DAISY_FACTORY_CMD_BT_PROF_PREV:
        status = handle_bt_prof_cycle(false);
        break;
    case DAISY_FACTORY_CMD_BT_STATUS:
        status = handle_bt_status(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BT_DIAG:
        status = handle_bt_diag(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BT_ADV_SET:
        status = handle_bt_adv_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                   req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_BT_ADDR:
        status = handle_bt_addr(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_BT_PASSKEY:
        status = handle_bt_passkey(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                   req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_BT_PROFILE_GET:
        status = handle_bt_profile_get(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                       req_buf[DAISY_FACTORY_OFF_LEN], payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_ENDPOINT_GET:
        status = handle_endpoint_get(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_ENDPOINT_SET:
        status = handle_endpoint_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                     req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_KEY_INJECT:
        status = handle_key_inject(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                   req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_POWER_SET:
        status = handle_touchpad_power_set(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                           req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_REG_READ:
        status = handle_touchpad_reg_read(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                          req_buf[DAISY_FACTORY_OFF_LEN], payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_REG_WRITE:
        status = handle_touchpad_reg_write(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                           req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_INJECT_RAW:
        status = handle_touchpad_inject_raw(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                            req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_INJECT_PTP:
        status = handle_touchpad_inject_ptp(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                            req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_TOUCHPAD_INJECT_SWEEP:
        status = handle_touchpad_inject_sweep(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                              req_buf[DAISY_FACTORY_OFF_LEN], payload,
                                              &payload_len);
        break;
    case DAISY_FACTORY_CMD_GPIO_GET:
        status = handle_gpio_get(&req_buf[DAISY_FACTORY_OFF_PAYLOAD], req_buf[DAISY_FACTORY_OFF_LEN],
                                 payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_PERT_QUIESCE:
        status = handle_pert_quiesce();
        break;
    case DAISY_FACTORY_CMD_PERT_TX_START:
        status = handle_pert_tx_start(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                      req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_PERT_RX_START:
        status = handle_pert_rx_start(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                      req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_PERT_END:
        status = handle_pert_end(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_RF_ENTER:
        status = handle_rf_enter();
        break;
    case DAISY_FACTORY_CMD_RF_START:
        status = handle_rf_start(&req_buf[DAISY_FACTORY_OFF_PAYLOAD],
                                 req_buf[DAISY_FACTORY_OFF_LEN]);
        break;
    case DAISY_FACTORY_CMD_RF_STOP:
        status = handle_rf_stop(payload, &payload_len);
        break;
    case DAISY_FACTORY_CMD_RF_STATUS:
        status = handle_rf_status(payload, &payload_len);
        break;
    default:
        status = DAISY_FACTORY_ERR_UNKNOWN_CMD;
        break;
    }

    /* Build the response frame in the DMA-safe buffer. */
    memset(resp_buf, 0, DAISY_FACTORY_REPORT_SIZE);
    resp_buf[DAISY_FACTORY_OFF_CMD] = cmd;
    resp_buf[DAISY_FACTORY_OFF_SEQ] = seq;
    resp_buf[DAISY_FACTORY_OFF_FLAGS] =
        DAISY_FACTORY_FLAG_RESPONSE | (status ? DAISY_FACTORY_FLAG_ERROR : 0);
    resp_buf[DAISY_FACTORY_OFF_STATUS] = status;
    resp_buf[DAISY_FACTORY_OFF_LEN] = payload_len;
    memcpy(&resp_buf[DAISY_FACTORY_OFF_PAYLOAD], payload, payload_len);

    int err = hid_device_submit_report(hid_dev, DAISY_FACTORY_REPORT_SIZE, resp_buf);
    if (err) {
        LOG_ERR("factory response submit failed (cmd 0x%02x): %d", cmd, err);
    }
}

static int daisy_factory_hid_init(void) {
    if (!device_is_ready(hid_dev)) {
        LOG_ERR("Factory HID device not ready");
        return -ENODEV;
    }

    k_work_init(&process_work, process_work_handler);

    /* Started before hid_device_register() so the queue exists by the time the
     * host can send the first request. See process_q for why it isn't the
     * system workqueue. */
    k_work_queue_start(&process_q, process_q_stack, K_THREAD_STACK_SIZEOF(process_q_stack),
                       PROCESS_Q_PRIORITY, NULL);
    k_thread_name_set(k_work_queue_thread_get(&process_q), "daisy_factory");

#if HAS_TOUCHPAD
    /* Synthetic-touchpad sweep generator; see tp_sweep_q for why it needs a
     * thread of its own. Idle until a TOUCHPAD_INJECT_SWEEP arrives. */
    k_work_queue_start(&tp_sweep_q, tp_sweep_q_stack, K_THREAD_STACK_SIZEOF(tp_sweep_q_stack),
                       TP_SWEEP_Q_PRIORITY, NULL);
    k_thread_name_set(k_work_queue_thread_get(&tp_sweep_q), "daisy_tp_sweep");
#endif

    int err = hid_device_register(hid_dev, factory_report_desc, sizeof(factory_report_desc), &ops);
    if (err) {
        LOG_ERR("Factory HID register failed: %d", err);
        return err;
    }

#if HAS_PAIRING_LEDS
    /* Prime every pairing-LED channel to 0. On a shared nRF PWM instance, a
     * channel never set since boot floats to ~100% once a sibling channel runs
     * a partial duty cycle; explicitly driving each to 0 avoids that glitch. */
    if (device_is_ready(pairing_leds)) {
        for (uint8_t i = 0; i < PAIRING_LED_COUNT; i++) {
            led_set_brightness(pairing_leds, i, 0);
        }
    }
#endif
#if HAS_BACKLIGHT
    if (device_is_ready(backlight)) {
        led_set_brightness(backlight, 0, 0);
    }
#endif
#if HAS_RGB_PWM
    /* Same priming rationale as the pairing LEDs (shared pwm20 instance). */
    if (device_is_ready(rgb_leds)) {
        for (uint8_t c = DAISY_FACTORY_RGB_RED; c <= DAISY_FACTORY_RGB_BLUE; c++) {
            led_set_brightness(rgb_leds, rgb_pwm_index[c], 0);
        }
    }
#elif HAS_RGB_GPIO
    for (uint8_t c = DAISY_FACTORY_RGB_RED; c <= DAISY_FACTORY_RGB_BLUE; c++) {
        if (gpio_is_ready_dt(&rgb_gpio[c])) {
            gpio_pin_configure_dt(&rgb_gpio[c], GPIO_OUTPUT_INACTIVE);
        }
    }
#endif

    LOG_INF("Daisy factory HID interface initialized");
    return 0;
}

/* Must run before zmk_usb_init (APPLICATION priority 96) so the class is
 * registered before usbd_register_all_classes(). Mirrors the touchpad
 * passthrough at priority 95. */
SYS_INIT(daisy_factory_hid_init, APPLICATION, 95);

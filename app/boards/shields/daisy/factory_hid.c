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

#include <zephyr/app_version.h>

#include <zmk/activity.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/ble.h>
#endif

#if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
#include <zephyr/retention/bootmode.h>
#include <zephyr/sys/reboot.h>
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

/* The nPM1300 MFD itself, used to enter ship mode (battery cutoff). */
#define HAS_PMIC DT_NODE_EXISTS(DT_NODELABEL(npm1300))
#if HAS_PMIC
static const struct device *const pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300));
/* Ship mode powers the board off, so it can't run inline with the response.
 * The handler acks immediately and schedules this to fire shortly after, giving
 * the USB Input report time to reach the host before power drops.
 *
 * The nPM1300 refuses to enter hibernate while VBUS is present and does not
 * latch the request, so the task must be issued only once USB is unplugged. The
 * nRF stays powered from BUCK2 (battery) after unplug, so this work polls the
 * charger's VBUS status and re-arms itself until VBUS drops, then cuts power. */
static void ship_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ship_work, ship_work_handler);
#define SHIP_ENTER_DELAY_MS 250
#define SHIP_POLL_INTERVAL_MS 200
/* Stop polling after this long unplugged-or-not so we don't spin forever if the
 * user never unplugs (e.g. command sent by mistake). */
#define SHIP_POLL_TIMEOUT_MS 30000
#define SHIP_POLL_MAX_ATTEMPTS (SHIP_POLL_TIMEOUT_MS / SHIP_POLL_INTERVAL_MS)
static int ship_poll_attempts;
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
    k_work_submit(&process_work);
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
        .capabilities = 0,
    };
    memcpy(payload, &info, sizeof(info));
    *out_len = sizeof(info);
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

#if HAS_PMIC
/* Returns true if VBUS is currently present (USB plugged in). On targets without
 * the charger we can't tell, so assume it's already safe to enter hibernate. */
static bool ship_vbus_present(void) {
#if HAS_CHARGER
    if (!device_is_ready(charger)) {
        return false;
    }
    struct sensor_value vbus;
    int err = sensor_sample_fetch(charger);
    if (err == 0) {
        err = sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus);
    }
    if (err != 0) {
        /* Can't read status; treat as present so we keep waiting rather than
         * cutting power while still plugged in. */
        LOG_WRN("ship poll %d: VBUS status read failed: %d", ship_poll_attempts, err);
        return true;
    }
    LOG_INF("ship poll %d: VBUSINSTATUS=0x%02x", ship_poll_attempts, (uint8_t)vbus.val1);
    /* val1 is the raw VBUS status register; bit 0 = VBUS present. */
    return (vbus.val1 & 0x01) != 0;
#else
    return false;
#endif
}

static void ship_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (!device_is_ready(pmic)) {
        return;
    }
    /* The PMIC ignores hibernate while VBUS is up. Wait for unplug, re-arming
     * until then (or until we give up so we don't poll forever). */
    if (ship_vbus_present()) {
        if (++ship_poll_attempts >= SHIP_POLL_MAX_ATTEMPTS) {
            LOG_WRN("ship mode aborted: VBUS still present after %d ms", SHIP_POLL_TIMEOUT_MS);
            return;
        }
        k_work_schedule(&ship_work, K_MSEC(SHIP_POLL_INTERVAL_MS));
        return;
    }
    /* Ship mode, not hibernate: hibernate always arms the wake-up timer and
     * would reboot the board instead of leaving it off until SHPHLD/VBUS. */
    LOG_INF("VBUS gone, strobing TASKENTERSHIPMODE");
    int err = mfd_npm13xx_ship_mode(pmic);
    if (err) {
        LOG_ERR("ship mode entry failed: %d", err);
        return;
    }
    /* If we're still alive to log this, the PMIC ignored the task. */
    k_msleep(50);
    LOG_ERR("still running 50 ms after TASKENTERSHIPMODE strobe");
}
#endif

/* Enter ship mode (battery cutoff). Action-only. Powers the board off, so we
 * ack now and arm the actual entry on a delay; the work then polls VBUS and only
 * cuts power once USB is unplugged (the PMIC ignores hibernate while VBUS is up,
 * and keeps the board alive on VBUS regardless). */
static uint8_t handle_ship_mode(void) {
#if !HAS_PMIC
    return DAISY_FACTORY_ERR_UNSUPPORTED;
#else
    if (!device_is_ready(pmic)) {
        return DAISY_FACTORY_ERR_HW;
    }
    ship_poll_attempts = 0;
    k_work_schedule(&ship_work, K_MSEC(SHIP_ENTER_DELAY_MS));
    return DAISY_FACTORY_OK;
#endif
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
    case DAISY_FACTORY_CMD_BT_CLEAR_BONDS:
        status = handle_bt_clear_bonds();
        break;
    case DAISY_FACTORY_CMD_GPIO_GET:
        status = handle_gpio_get(&req_buf[DAISY_FACTORY_OFF_PAYLOAD], req_buf[DAISY_FACTORY_OFF_LEN],
                                 payload, &payload_len);
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

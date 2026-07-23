/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/usb_buf.h>

#include <zmk/usb.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#include <zmk/pointing/resolution_multipliers.h>
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

#include <zmk/event_manager.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device *hid_dev;
static bool hid_ready;

#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
static uint8_t hid_protocol = 1; /* Report Protocol */

void zmk_usb_hid_set_protocol(uint8_t protocol) { hid_protocol = protocol; }
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

static void iface_ready_cb(const struct device *dev, const bool ready) {
    LOG_INF("HID interface %s", ready ? "ready" : "not ready");
    hid_ready = ready;
}

static uint8_t *get_keyboard_report(size_t *len) {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    if (hid_protocol != 1) {
        zmk_hid_boot_report_t *boot_report = zmk_hid_get_boot_report();
        *len = sizeof(*boot_report);
        return (uint8_t *)boot_report;
    }
#endif
    struct zmk_hid_keyboard_report *report = zmk_hid_get_keyboard_report();
    *len = sizeof(*report);
    return (uint8_t *)report;
}

static int get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, uint8_t *const buf) {
    switch (type) {
    case HID_REPORT_TYPE_FEATURE:
        switch (id) {
#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
        case ZMK_HID_REPORT_ID_MOUSE: {
            struct zmk_endpoint_instance endpoint = {
                .transport = ZMK_TRANSPORT_USB,
            };
            struct zmk_pointing_resolution_multipliers mult =
                zmk_pointing_resolution_multipliers_get_profile(endpoint);
            struct zmk_hid_mouse_resolution_feature_report_body body = {
                .wheel_res = mult.wheel,
                .hwheel_res = mult.hor_wheel,
            };
            if (len < sizeof(body)) {
                return -EINVAL;
            }
            memcpy(buf, &body, sizeof(body));
            return sizeof(body);
        }
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
        default:
            return -ENOTSUP;
        }
    case HID_REPORT_TYPE_INPUT:
        switch (id) {
        case ZMK_HID_REPORT_ID_KEYBOARD: {
            size_t size;
            uint8_t *report = get_keyboard_report(&size);
            /* Skip the report ID byte */
            size_t body_size = size - 1;
            if (len < body_size) {
                return -EINVAL;
            }
            memcpy(buf, report + 1, body_size);
            return body_size;
        }
        case ZMK_HID_REPORT_ID_CONSUMER: {
            struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();
            size_t body_size = sizeof(report->body);
            if (len < body_size) {
                return -EINVAL;
            }
            memcpy(buf, &report->body, body_size);
            return body_size;
        }
        default:
            LOG_ERR("Invalid report ID %d requested", id);
            return -EINVAL;
        }
    default:
        LOG_ERR("Unsupported report type %d requested", type);
        return -ENOTSUP;
    }
}

static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, const uint8_t *const buf) {
    switch (type) {
    case HID_REPORT_TYPE_FEATURE:
        switch (id) {
#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
        case ZMK_HID_REPORT_ID_MOUSE: {
            if (len != sizeof(struct zmk_hid_mouse_resolution_feature_report_body)) {
                return -EINVAL;
            }
            struct zmk_hid_mouse_resolution_feature_report_body *body =
                (struct zmk_hid_mouse_resolution_feature_report_body *)buf;
            struct zmk_endpoint_instance endpoint = {
                .transport = ZMK_TRANSPORT_USB,
            };
            zmk_pointing_resolution_multipliers_process_report(body, endpoint);
            return 0;
        }
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
        default:
            return -ENOTSUP;
        }
    case HID_REPORT_TYPE_OUTPUT:
        switch (id) {
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
        case ZMK_HID_REPORT_ID_LEDS: {
            /* The USB-next HID class passes the full control payload, including
             * the leading report-ID byte, so the buffer is a whole
             * zmk_hid_led_report (report_id + body), not just the body. */
            if (len != sizeof(struct zmk_hid_led_report)) {
                LOG_ERR("LED set report is malformed: length=%d", len);
                return -EINVAL;
            }
            struct zmk_hid_led_report *report = (struct zmk_hid_led_report *)buf;
            struct zmk_endpoint_instance endpoint = {
                .transport = ZMK_TRANSPORT_USB,
            };
            zmk_hid_indicators_process_report(&report->body, endpoint);
            return 0;
        }
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
        default:
            LOG_ERR("Invalid report ID %d requested", id);
            return -EINVAL;
        }
    default:
        LOG_ERR("Unsupported report type %d requested", type);
        return -ENOTSUP;
    }
}

static uint32_t idle_duration;

static void set_idle_cb(const struct device *dev, const uint8_t id, const uint32_t duration) {
    idle_duration = duration;
}

static uint32_t get_idle_cb(const struct device *dev, const uint8_t id) {
    return idle_duration;
}

static void set_protocol_cb(const struct device *dev, const uint8_t proto) {
    LOG_INF("Protocol changed to %s", proto == 0U ? "Boot" : "Report");
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    hid_protocol = proto;
#endif
}

static struct hid_device_ops ops = {
    .iface_ready = iface_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .set_idle = set_idle_cb,
    .get_idle = get_idle_cb,
    .set_protocol = set_protocol_cb,
};

/* Bounce reports through a UDC-aligned static buffer. ZMK report structs are
 * __packed (1-byte alignment), but the DWC2 DMA path rejects buffers less
 * aligned than USB_BUF_ALIGN. Matches the in-report-size of the keyboard-hid
 * node (64 bytes); submissions are synchronous because ops has no
 * input_report_done callback, so the buffer is free again on return. */
UDC_STATIC_BUF_DEFINE(hid_tx_buf, 64);
static K_MUTEX_DEFINE(hid_tx_mutex);

static int zmk_usb_hid_send_report(const uint8_t *report, size_t len) {
    if (!hid_ready) {
        return -ENODEV;
    }

    if (zmk_usb_is_suspended()) {
        /* Host is asleep: the IN endpoint isn't polled, so submitting would
         * only tie up the TX buffer (its done callback can't fire until
         * resume) and then time out. Ask the host to wake instead and drop
         * this report; the current HID state is re-sent once the bus resumes
         * (and the key release, if any, follows as its own report). */
        zmk_usb_wakeup_request();
        return 0;
    }

    if (len > sizeof(hid_tx_buf)) {
        LOG_ERR("HID report too large: %zu > %zu", len, sizeof(hid_tx_buf));
        return -EINVAL;
    }

    k_mutex_lock(&hid_tx_mutex, K_FOREVER);
    memcpy(hid_tx_buf, report, len);
    int ret = hid_device_submit_report(hid_dev, len, hid_tx_buf);
    k_mutex_unlock(&hid_tx_mutex);
    return ret;
}

int zmk_usb_hid_send_keyboard_report(void) {
    size_t len;
    uint8_t *report = get_keyboard_report(&len);
    return zmk_usb_hid_send_report(report, len);
}

int zmk_usb_hid_send_consumer_report(void) {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    if (hid_protocol == 0) {
        return -ENOTSUP;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

    struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();
    return zmk_usb_hid_send_report((uint8_t *)report, sizeof(*report));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)
int zmk_usb_hid_send_mouse_report() {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    if (hid_protocol == 0) {
        return -ENOTSUP;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

    struct zmk_hid_mouse_report *report = zmk_hid_get_mouse_report();
    return zmk_usb_hid_send_report((uint8_t *)report, sizeof(*report));
}
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

#if !DT_HAS_CHOSEN(zmk_keyboard_hid)
#error "CONFIG_ZMK_USB requires chosen zmk,keyboard-hid to point at a zephyr,hid-device node"
#endif

static int zmk_usb_hid_init(void) {
    int err;

    hid_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_keyboard_hid));
    if (!device_is_ready(hid_dev)) {
        LOG_ERR("HID device is not ready");
        return -ENODEV;
    }

    err = hid_device_register(hid_dev, zmk_hid_report_desc, sizeof(zmk_hid_report_desc), &ops);
    if (err) {
        LOG_ERR("Failed to register HID device (%d)", err);
        return err;
    }

    return 0;
}

SYS_INIT(zmk_usb_hid_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);

/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/usb/usbd.h>

#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
#include <zmk/usb_hid.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* USB descriptor strings */
USBD_DESC_LANG_DEFINE(zmk_lang);
USBD_DESC_MANUFACTURER_DEFINE(zmk_mfr, CONFIG_USB_DEVICE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(zmk_product, CONFIG_USB_DEVICE_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(zmk_sn);
USBD_DESC_CONFIG_DEFINE(zmk_fs_cfg_desc, "FS Configuration");

/* Full speed configuration (max power = 125 * 2mA = 250mA) */
USBD_CONFIGURATION_DEFINE(zmk_fs_config, USB_SCD_REMOTE_WAKEUP, 125, &zmk_fs_cfg_desc);

#if USBD_SUPPORTS_HIGH_SPEED
USBD_DESC_CONFIG_DEFINE(zmk_hs_cfg_desc, "HS Configuration");
USBD_CONFIGURATION_DEFINE(zmk_hs_config, USB_SCD_REMOTE_WAKEUP, 125, &zmk_hs_cfg_desc);
#endif

/* USB device context */
USBD_DEVICE_DEFINE(zmk_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   CONFIG_USB_DEVICE_VID, CONFIG_USB_DEVICE_PID);

static enum zmk_usb_conn_state conn_state = ZMK_USB_CONN_NONE;
static bool is_configured;

static void raise_usb_status_changed_event(struct k_work *_work) {
    raise_zmk_usb_conn_state_changed(
        (struct zmk_usb_conn_state_changed){.conn_state = zmk_usb_get_conn_state()});
}

K_WORK_DEFINE(usb_status_notifier_work, raise_usb_status_changed_event);

enum zmk_usb_conn_state zmk_usb_get_conn_state(void) { return conn_state; }

bool zmk_usb_is_hid_ready(void) { return conn_state == ZMK_USB_CONN_HID && is_configured; }

struct usbd_context *zmk_usb_get_usbd(void) { return &zmk_usbd; }

static void usbd_msg_cb(struct usbd_context *const usbd_ctx, const struct usbd_msg *const msg) {
    LOG_DBG("USBD message: %s", usbd_msg_type_string(msg->type));

    switch (msg->type) {
    case USBD_MSG_CONFIGURATION:
        LOG_INF("USB configuration %d", msg->status);
        if (msg->status > 0) {
            conn_state = ZMK_USB_CONN_HID;
            is_configured = true;
        } else {
            conn_state = ZMK_USB_CONN_POWERED;
            is_configured = false;
        }
        break;
    case USBD_MSG_SUSPEND:
    case USBD_MSG_RESUME:
        conn_state = is_configured ? ZMK_USB_CONN_HID : ZMK_USB_CONN_POWERED;
        break;
    case USBD_MSG_RESET:
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
        zmk_usb_hid_set_protocol(1);
#endif
        conn_state = ZMK_USB_CONN_POWERED;
        is_configured = false;
        break;
    case USBD_MSG_VBUS_REMOVED:
        conn_state = ZMK_USB_CONN_NONE;
        is_configured = false;
        break;
    case USBD_MSG_VBUS_READY:
        if (usbd_can_detect_vbus(usbd_ctx)) {
            if (usbd_enable(usbd_ctx)) {
                LOG_ERR("Failed to enable USB on VBUS ready");
            }
        }
        conn_state = ZMK_USB_CONN_POWERED;
        break;
    default:
        break;
    }

    k_work_submit(&usb_status_notifier_work);
}

static int zmk_usb_init(void) {
    int err;

    err = usbd_add_descriptor(&zmk_usbd, &zmk_lang);
    if (err) {
        LOG_ERR("Failed to add language descriptor (%d)", err);
        return err;
    }

    err = usbd_add_descriptor(&zmk_usbd, &zmk_mfr);
    if (err) {
        LOG_ERR("Failed to add manufacturer descriptor (%d)", err);
        return err;
    }

    err = usbd_add_descriptor(&zmk_usbd, &zmk_product);
    if (err) {
        LOG_ERR("Failed to add product descriptor (%d)", err);
        return err;
    }

    err = usbd_add_descriptor(&zmk_usbd, &zmk_sn);
    if (err) {
        LOG_ERR("Failed to add serial number descriptor (%d)", err);
        return err;
    }

#if USBD_SUPPORTS_HIGH_SPEED
    if (usbd_caps_speed(&zmk_usbd) == USBD_SPEED_HS) {
        err = usbd_add_configuration(&zmk_usbd, USBD_SPEED_HS, &zmk_hs_config);
        if (err) {
            LOG_ERR("Failed to add HS configuration (%d)", err);
            return err;
        }

        err = usbd_register_all_classes(&zmk_usbd, USBD_SPEED_HS, 1, NULL);
        if (err) {
            LOG_ERR("Failed to register HS USB classes (%d)", err);
            return err;
        }

        if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
            usbd_device_set_code_triple(&zmk_usbd, USBD_SPEED_HS,
                                        USB_BCC_MISCELLANEOUS, 0x02, 0x01);
        } else {
            usbd_device_set_code_triple(&zmk_usbd, USBD_SPEED_HS, 0, 0, 0);
        }
    }
#endif

    err = usbd_add_configuration(&zmk_usbd, USBD_SPEED_FS, &zmk_fs_config);
    if (err) {
        LOG_ERR("Failed to add FS configuration (%d)", err);
        return err;
    }

    err = usbd_register_all_classes(&zmk_usbd, USBD_SPEED_FS, 1, NULL);
    if (err) {
        LOG_ERR("Failed to register USB classes (%d)", err);
        return err;
    }

    if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
        /* CDC ACM uses IAD, which requires the Miscellaneous class code */
        usbd_device_set_code_triple(&zmk_usbd, USBD_SPEED_FS,
                                    USB_BCC_MISCELLANEOUS, 0x02, 0x01);
    } else {
        usbd_device_set_code_triple(&zmk_usbd, USBD_SPEED_FS, 0, 0, 0);
    }

    err = usbd_msg_register_cb(&zmk_usbd, usbd_msg_cb);
    if (err) {
        LOG_ERR("Failed to register message callback (%d)", err);
        return err;
    }

    err = usbd_init(&zmk_usbd);
    if (err) {
        LOG_ERR("Failed to init USB device support (%d)", err);
        return err;
    }

    if (!usbd_can_detect_vbus(&zmk_usbd)) {
        err = usbd_enable(&zmk_usbd);
        if (err) {
            LOG_ERR("Failed to enable USB (%d)", err);
            return err;
        }
    }

    return 0;
}

SYS_INIT(zmk_usb_init, APPLICATION, CONFIG_ZMK_USB_INIT_PRIORITY);

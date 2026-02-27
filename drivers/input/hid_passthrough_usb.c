/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "hid_touchpad.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_usb, CONFIG_HID_PASSTHROUGH_USB_LOG_LEVEL);

#define HID_GET_REPORT_TYPE_MASK 0xFF00
#define HID_GET_REPORT_ID_MASK   0x00FF

#define HID_REPORT_TYPE_INPUT   0x0100
#define HID_REPORT_TYPE_OUTPUT  0x0200
#define HID_REPORT_TYPE_FEATURE 0x0300

static const struct device *hid_dev;
static const struct device *tp_dev;

static K_SEM_DEFINE(hid_sem, 1, 1);

static void in_ready_cb(const struct device *dev) {
    k_sem_give(&hid_sem);
}

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup,
                          int32_t *len, uint8_t **data) {
    uint16_t report_type = setup->wValue & HID_GET_REPORT_TYPE_MASK;
    uint8_t report_id = setup->wValue & HID_GET_REPORT_ID_MASK;
    LOG_DBG("get_report_cb: type=0x%04x id=0x%02x", report_type, report_id);
    uint8_t i2c_type;

    switch (report_type) {
    case HID_REPORT_TYPE_FEATURE:
        i2c_type = I2C_HID_REPORT_TYPE_FEATURE;
        break;
    case HID_REPORT_TYPE_INPUT:
        i2c_type = I2C_HID_REPORT_TYPE_INPUT;
        break;
    default:
        return -ENOTSUP;
    }

    /* Use a static buffer for the response. Max feature report is 256 bytes + overhead.
     * Zephyr USB HID expects the report ID as the first byte of the response. */
    static uint8_t report_buf[265];
    uint16_t out_len = 0;

    report_buf[0] = report_id;
    int err = hid_touchpad_get_report(tp_dev, i2c_type, report_id,
                                      report_buf + 1, sizeof(report_buf) - 1, &out_len);
    if (err) {
        LOG_ERR("get_report proxy failed for type=0x%02x id=%d: %d", i2c_type, report_id, err);
        return err;
    }

    *data = report_buf;
    *len = out_len + 1;
    return 0;
}

static int set_report_cb(const struct device *dev, struct usb_setup_packet *setup,
                          int32_t *len, uint8_t **data) {
    uint16_t report_type = setup->wValue & HID_GET_REPORT_TYPE_MASK;
    uint8_t report_id = setup->wValue & HID_GET_REPORT_ID_MASK;
    LOG_DBG("set_report_cb: type=0x%04x id=0x%02x len=%d", report_type, report_id, *len);
    uint8_t i2c_type;

    switch (report_type) {
    case HID_REPORT_TYPE_FEATURE:
        i2c_type = I2C_HID_REPORT_TYPE_FEATURE;
        break;
    case HID_REPORT_TYPE_OUTPUT:
        i2c_type = I2C_HID_REPORT_TYPE_OUTPUT;
        break;
    default:
        return -ENOTSUP;
    }

    /* Zephyr USB HID includes the report ID as the first byte of *data.
     * Skip it — hid_touchpad_set_report adds the report ID itself. */
    int err = hid_touchpad_set_report(tp_dev, i2c_type, report_id,
                                      *data + 1, *len - 1);
    if (err) {
        LOG_ERR("set_report proxy failed for type=0x%02x id=%d: %d", i2c_type, report_id, err);
        return err;
    }

    return 0;
}

static void tp_input_cb(const struct device *dev, uint8_t report_id,
                         const uint8_t *data, uint16_t len) {
    if (!hid_dev) {
        return;
    }

    /* Prepend report ID to the data for USB HID */
    uint8_t buf[64];
    if (len + 1 > sizeof(buf)) {
        LOG_WRN("input report too large: %d", len);
        return;
    }
    buf[0] = report_id;
    memcpy(&buf[1], data, len);

    k_sem_take(&hid_sem, K_MSEC(30));
    int err = hid_int_ep_write(hid_dev, buf, len + 1, NULL);
    if (err) {
        k_sem_give(&hid_sem);
        LOG_DBG("USB write failed: %d", err);
    }
}

static const struct hid_ops ops = {
    .int_in_ready = in_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

static int hid_passthrough_usb_init(void) {
    tp_dev = DEVICE_DT_GET(DT_NODELABEL(touchpad));
    if (!device_is_ready(tp_dev)) {
        LOG_ERR("Touchpad device not ready");
        return -ENODEV;
    }

    hid_dev = device_get_binding("HID_1");
    if (hid_dev == NULL) {
        LOG_ERR("Unable to locate HID_1 device");
        return -EINVAL;
    }

    usb_hid_register_device(hid_dev, tp_report_desc, tp_report_desc_size, &ops);
    usb_hid_init(hid_dev);

    hid_touchpad_register_input_cb(tp_dev, tp_input_cb);

    LOG_INF("USB HID passthrough initialized on HID_1");
    return 0;
}

SYS_INIT(hid_passthrough_usb_init, APPLICATION, 96);

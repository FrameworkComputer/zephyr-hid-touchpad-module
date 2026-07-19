/*
 * Copyright (c) 2026 The ZMK Contributors
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/usb_buf.h>

#include "hid_touchpad.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_usb, CONFIG_HID_PASSTHROUGH_USB_LOG_LEVEL);

#define TP_HID_NODE DT_NODELABEL(tp_hid)

BUILD_ASSERT(DT_NODE_EXISTS(TP_HID_NODE),
             "Node label 'tp_hid' missing; add a zephyr,hid-device node to the shield/board DT");

static const struct device *const hid_dev = DEVICE_DT_GET(TP_HID_NODE);
static const struct device *tp_dev;
static bool hid_ready;

static void iface_ready_cb(const struct device *dev, const bool ready) {
    LOG_INF("TP HID interface %s", ready ? "ready" : "not ready");
    hid_ready = ready;
}

static int get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, uint8_t *const buf) {
    uint8_t i2c_type;

    LOG_DBG("get_report: type=%u id=0x%02x len=%u", type, id, len);

    if (tp_dev == NULL || !device_is_ready(tp_dev)) {
        return -ENOTSUP;
    }

    switch (type) {
    case HID_REPORT_TYPE_FEATURE:
        i2c_type = I2C_HID_REPORT_TYPE_FEATURE;
        break;
    case HID_REPORT_TYPE_INPUT:
        i2c_type = I2C_HID_REPORT_TYPE_INPUT;
        break;
    default:
        return -ENOTSUP;
    }

    /* When report IDs are in use, USB HID (and Linux hid-core in particular)
     * expects the returned data to begin with the report ID byte. Place it
     * at buf[0] and ask the touchpad for its data starting at buf[1]. */
    if (len < 1) {
        return -EINVAL;
    }
    buf[0] = id;

    uint16_t out_len = 0;
    int err = hid_touchpad_get_report(tp_dev, i2c_type, id, buf + 1, len - 1, &out_len);
    if (err) {
        LOG_ERR("get_report proxy failed type=0x%02x id=%u: %d", i2c_type, id, err);
        return err;
    }
    return (int)out_len + 1;
}

static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, const uint8_t *const buf) {
    uint8_t i2c_type;

    LOG_DBG("set_report: type=%u id=0x%02x len=%u", type, id, len);

    if (tp_dev == NULL || !device_is_ready(tp_dev)) {
        return -ENOTSUP;
    }

    switch (type) {
    case HID_REPORT_TYPE_FEATURE:
        i2c_type = I2C_HID_REPORT_TYPE_FEATURE;
        break;
    case HID_REPORT_TYPE_OUTPUT:
        i2c_type = I2C_HID_REPORT_TYPE_OUTPUT;
        break;
    default:
        return -ENOTSUP;
    }

    /* Linux (and per the HID spec) sends SET_REPORT for numbered reports
     * with the report ID as the first data byte, so buf[0] == id. Skip it:
     * hid_touchpad_set_report frames its own I2C HID payload including id. */
    if (len < 1) {
        return -EINVAL;
    }
    int err = hid_touchpad_set_report(tp_dev, i2c_type, id, buf + 1, len - 1);
    if (err) {
        LOG_ERR("set_report proxy failed type=0x%02x id=%u: %d", i2c_type, id, err);
    }
    return err;
}

/* TX bounce buffer for async submits: with ops.input_report_done set,
 * hid_device_submit_report() returns right after enqueue and the USB stack
 * owns the buffer until the done callback — so it cannot live on the stack.
 * UDC-aligned because the DWC2 DMA path rejects under-aligned buffers.
 * Without the callback the submit blocks on a K_FOREVER semaphore until the
 * IN transfer completes; when USB drops mid-transfer that wedges the calling
 * (touchpad) context until replug. tp_tx_free tracks buffer ownership with a
 * bounded wait instead: a stalled transfer costs at most TP_TX_TIMEOUT per
 * frame and drops it (PTP recovers on the next frame). */
UDC_STATIC_BUF_DEFINE(tp_tx_buf, 64);
static K_SEM_DEFINE(tp_tx_free, 1, 1);
#define TP_TX_TIMEOUT K_MSEC(20)

static void input_report_done_cb(const struct device *dev, const uint8_t *const report) {
    ARG_UNUSED(dev);
    ARG_UNUSED(report);
    k_sem_give(&tp_tx_free);
}

static const struct hid_device_ops ops = {
    .iface_ready = iface_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .input_report_done = input_report_done_cb,
};

static void tp_input_cb(const struct device *dev, uint8_t report_id,
                        const uint8_t *data, uint16_t len) {
    if (!hid_ready) {
        return;
    }

    /* hid_device_submit_report() expects the report ID as the first byte. */
    if ((size_t)len + 1 > sizeof(tp_tx_buf)) {
        LOG_WRN("input report too large: %u", len);
        return;
    }
    if (k_sem_take(&tp_tx_free, TP_TX_TIMEOUT) != 0) {
        LOG_WRN("TX buffer still in flight, dropping frame");
        return;
    }
    tp_tx_buf[0] = report_id;
    memcpy(&tp_tx_buf[1], data, len);

    int err = hid_device_submit_report(hid_dev, len + 1, tp_tx_buf);
    if (err) {
        /* Not enqueued; the done callback will never fire for it. */
        k_sem_give(&tp_tx_free);
        LOG_WRN("submit_report failed: %d", err);
    }
}

static int hid_passthrough_usb_init(void) {
    if (!device_is_ready(hid_dev)) {
        LOG_ERR("TP HID device not ready");
        return -ENODEV;
    }

    /* Register the HID class unconditionally so the new USBD stack can
     * initialize its interface, even if the physical touchpad isn't
     * present or failed to init. Without this, usbd_register_all_classes()
     * refuses to bring up ANY interface (including the keyboard HID). */
    int err = hid_device_register(hid_dev, tp_report_desc, tp_report_desc_size, &ops);
    if (err) {
        LOG_ERR("hid_device_register failed: %d", err);
        return err;
    }

    tp_dev = DEVICE_DT_GET(DT_NODELABEL(touchpad));
    if (!device_is_ready(tp_dev)) {
        LOG_WRN("Touchpad device not ready; passthrough will stay idle");
        return 0;
    }

    hid_touchpad_register_input_cb(tp_dev, tp_input_cb);

    LOG_INF("USB HID touchpad passthrough (next stack) initialized");
    return 0;
}

/* Must run before zmk_usb_init (APPLICATION/ZMK_USB_INIT_PRIORITY=96),
 * since hid_device_register must happen before usbd_register_all_classes. */
SYS_INIT(hid_passthrough_usb_init, APPLICATION, 95);

/*
 * Copyright (c) 2025 The ZMK Contributors
 * Copyright (c) 2025-2026 Framework Computer Inc
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

#include <zmk/endpoints.h>
#include <zmk/usb.h>

#include "hid_touchpad.h"
#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
#include "apple_mt.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_usb, CONFIG_HID_PASSTHROUGH_USB_LOG_LEVEL);

#define TP_HID_NODE DT_NODELABEL(tp_hid)
#define TP_PTP_INPUT_REPORT_ID DT_PROP(DT_NODELABEL(touchpad), ptp_input_report_id)

BUILD_ASSERT(DT_NODE_EXISTS(TP_HID_NODE),
             "Node label 'tp_hid' missing; add a zephyr,hid-device node to the shield/board DT");

static const struct device *const hid_dev = DEVICE_DT_GET(TP_HID_NODE);
static const struct device *tp_dev;
static bool hid_ready;

#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
static struct apple_mt apple_mt;
/* Frames are stamped from the first one, as VoodooInput does. */
static int64_t apple_mt_epoch;

/* No macOS driver sets the PTP Input Mode, and the pad powers up as a mouse,
 * so switch it on every enumeration. I2C, so not from the USB callback. */
static void ptp_mode_work_fn(struct k_work *work) {
    const uint8_t ptp_mode = 3;
    int err = hid_touchpad_set_report(tp_dev, I2C_HID_REPORT_TYPE_FEATURE,
                                      CONFIG_HID_PASSTHROUGH_USB_APPLE_MT_INPUT_MODE_REPORT_ID,
                                      &ptp_mode, 1);
    if (err) {
        LOG_ERR("apple-mt: setting PTP input mode failed: %d", err);
    } else {
        LOG_INF("apple-mt: pad in PTP mode");
    }
}
static K_WORK_DEFINE(ptp_mode_work, ptp_mode_work_fn);
#endif

static void iface_ready_cb(const struct device *dev, const bool ready) {
    LOG_INF("TP HID interface %s", ready ? "ready" : "not ready");
    hid_ready = ready;
#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
    if (ready && tp_dev != NULL && device_is_ready(tp_dev)) {
        k_work_submit(&ptp_mode_work);
    }
#endif
}

static int get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, uint8_t *const buf) {
    uint8_t i2c_type;

    LOG_DBG("get_report: type=%u id=0x%02x len=%u", type, id, len);

#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
    /* The host talks to the Apple trackpad we present, never to the pad. */
    int n = apple_mt_get_report(&apple_mt, id, buf, len);
    LOG_INF("apple-mt: GET type=%u id=0x%02x len=%u -> %d", type, id, len, n);
    return n < 0 ? -ENOTSUP : n;
#endif

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

#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
    LOG_INF("apple-mt: SET type=%u id=0x%02x len=%u cmd=0x%02x", type, id, len,
            len > 1 ? buf[1] : 0);
    apple_mt_set_report(&apple_mt, id, buf, len);
    return 0;
#endif

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

/* Send one input report, report ID first, through the bounce buffer. */
static void submit_report(const uint8_t *report, size_t len) {
    if (len > sizeof(tp_tx_buf)) {
        LOG_WRN("input report too large: %u", (unsigned)len);
        return;
    }
    if (k_sem_take(&tp_tx_free, TP_TX_TIMEOUT) != 0) {
        LOG_WRN("TX buffer still in flight, dropping frame");
        return;
    }
    memcpy(tp_tx_buf, report, len);

    int err = hid_device_submit_report(hid_dev, len, tp_tx_buf);
    if (err) {
        /* Not enqueued; the done callback will never fire for it. */
        k_sem_give(&tp_tx_free);
        LOG_WRN("submit_report failed: %d", err);
    }
}

#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
static void apple_mt_input(uint8_t report_id, const uint8_t *data, uint16_t len) {
    uint8_t frame[APPLE_MT_MAX_FRAME];
    bool lifted;

    if (report_id != TP_PTP_INPUT_REPORT_ID) {
        return; /* the pad's mouse report: macOS gets pointer motion from us */
    }
    if (apple_mt_epoch == 0) {
        apple_mt_epoch = k_uptime_get();
    }
    uint32_t now = (uint32_t)(k_uptime_get() - apple_mt_epoch);
    size_t n = apple_mt_frame(&apple_mt, data, len, now, frame, &lifted);
    if (n == 0) {
        return;
    }
    submit_report(frame, n);
    if (lifted) {
        for (int step = 0; step < 3; step++) {
            now += step == 1 ? 0 : 10;
            submit_report(frame, apple_mt_lift_frame(&apple_mt, step, now, frame));
        }
    }
}
#endif

static void tp_input_cb(const struct device *dev, uint8_t report_id,
                        const uint8_t *data, uint16_t len) {
    if (!hid_ready) {
        return;
    }

    /* Follow the same endpoint selection as the keyboard (zmk_endpoints):
     * only stream the pad over USB when USB is the selected transport, so it
     * doesn't mirror to both USB and BLE at once. */
    if (zmk_endpoint_get_selected().transport != ZMK_TRANSPORT_USB) {
        return;
    }

    if (zmk_usb_is_suspended()) {
        /* Host asleep: request remote wakeup and drop the frame (PTP recovers
         * once the bus resumes). Submitting while suspended would only stall
         * on the un-polled endpoint and time out. */
        zmk_usb_wakeup_request();
        return;
    }

#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
    apple_mt_input(report_id, data, len);
#else
    /* hid_device_submit_report() expects the report ID as the first byte. */
    uint8_t report[64];

    if ((size_t)len + 1 > sizeof(report)) {
        LOG_WRN("input report too large: %u", len);
        return;
    }
    report[0] = report_id;
    memcpy(&report[1], data, len);
    submit_report(report, len + 1);
#endif
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
#ifdef CONFIG_HID_PASSTHROUGH_USB_APPLE_MT
    apple_mt_init(&apple_mt, CONFIG_HID_PASSTHROUGH_USB_APPLE_MT_X_MAX,
                  CONFIG_HID_PASSTHROUGH_USB_APPLE_MT_Y_MAX,
                  CONFIG_HID_PASSTHROUGH_USB_APPLE_MT_WIDTH,
                  CONFIG_HID_PASSTHROUGH_USB_APPLE_MT_HEIGHT);
    int err = hid_device_register(hid_dev, apple_mt_report_desc, apple_mt_report_desc_size, &ops);
#else
    int err = hid_device_register(hid_dev, tp_report_desc, tp_report_desc_size, &ops);
#endif
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

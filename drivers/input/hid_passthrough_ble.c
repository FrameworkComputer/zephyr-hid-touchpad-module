/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>

#include "hid_touchpad.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_ble, CONFIG_HID_PASSTHROUGH_BLE_LOG_LEVEL);

#define TP_NODE DT_NODELABEL(touchpad)

#define TP_MOUSE_INPUT_REPORT_ID  DT_PROP(TP_NODE, mouse_input_report_id)
#define TP_PTP_INPUT_REPORT_ID    DT_PROP(TP_NODE, ptp_input_report_id)
#define TP_MOUSE_INPUT_REPORT_SIZE DT_PROP(TP_NODE, mouse_input_report_size)
#define TP_PTP_INPUT_REPORT_SIZE  DT_PROP(TP_NODE, ptp_input_report_size)

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version;
    uint8_t code;
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;
    uint8_t type;
} __packed;

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_info tp_info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE | HIDS_REMOTE_WAKE,
};

/* Report reference descriptors for input reports */
static struct hids_report mouse_input_ref = { .id = TP_MOUSE_INPUT_REPORT_ID, .type = HIDS_INPUT };
static struct hids_report ptp_input_ref = { .id = TP_PTP_INPUT_REPORT_ID, .type = HIDS_INPUT };

/* Generate feature report reference descriptors from devicetree */
#define FEATURE_REF_DECL(node, prop, idx) \
    static struct hids_report feature_ref_##idx = { \
        .id = DT_PROP_BY_IDX(node, prop, idx), .type = HIDS_FEATURE };
DT_FOREACH_PROP_ELEM(TP_NODE, feature_report_ids, FEATURE_REF_DECL)

static uint8_t ctrl_point;
static const struct device *tp_dev;

/* Cached last input reports for BLE read */
static uint8_t cached_mouse_report[TP_MOUSE_INPUT_REPORT_SIZE];
static uint8_t cached_ptp_report[TP_PTP_INPUT_REPORT_SIZE];

/* ---- GATT read/write callbacks ---- */

static ssize_t read_hids_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_info));
}

static ssize_t read_hids_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_report));
}

static ssize_t read_hids_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, tp_report_desc,
                             tp_report_desc_size);
}

static ssize_t read_mouse_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, cached_mouse_report,
                             sizeof(cached_mouse_report));
}

static ssize_t read_ptp_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, cached_ptp_report,
                             sizeof(cached_ptp_report));
}

static ssize_t read_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    struct hids_report *ref = (struct hids_report *)attr->user_data;
    if (!ref || !tp_dev) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    static uint8_t feature_buf[264];
    uint16_t out_len = 0;

    int err = hid_touchpad_get_report(tp_dev, I2C_HID_REPORT_TYPE_FEATURE, ref->id,
                                      feature_buf, sizeof(feature_buf), &out_len);
    if (err) {
        LOG_ERR("BLE get feature report %d failed: %d", ref->id, err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, feature_buf, out_len);
}

static ssize_t write_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags) {
    struct hids_report *ref = (struct hids_report *)attr->user_data;
    if (!ref || !tp_dev || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    int err = hid_touchpad_set_report(tp_dev, I2C_HID_REPORT_TYPE_FEATURE, ref->id, buf, len);
    if (err) {
        LOG_ERR("BLE set feature report %d failed: %d", ref->id, err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

static void tp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("TP CCC changed: %d", value);
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t *value = attr->user_data;
    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    memcpy(value + offset, buf, len);
    return len;
}

/* Per HoGP 1.0 Table 4.3, Feature Report characteristics advertise only
 * Read + Write. Advertising Write-Without-Response makes BlueZ classify the
 * characteristic as an Output Report, which silently suppresses feature
 * GET_REPORT requests (no ATT read is ever issued to us). */
#define FEATURE_GATT_ATTRS(node, prop, idx) \
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, \
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, \
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, \
                           read_feature_report, write_feature_report, &feature_ref_##idx), \
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, \
                       read_hids_report_ref, NULL, &feature_ref_##idx),

/* Attribute indices for notify targets (stable: precede variable-length feature section) */
#define TP_MOUSE_INPUT_ATTR_IDX  6
#define TP_PTP_INPUT_ATTR_IDX   10

BT_GATT_SERVICE_DEFINE(
    tp_hog_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    /* HID Info */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_hids_info, NULL, &tp_info),

    /* Report Map */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_report_map, NULL, NULL),

    /* Input Report: Mouse */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_mouse_input_report, NULL, NULL),
    BT_GATT_CCC(tp_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &mouse_input_ref),

    /* Input Report: PTP multitouch */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_ptp_input_report, NULL, NULL),
    BT_GATT_CCC(tp_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &ptp_input_ref),

    /* Feature reports (generated from devicetree) */
    DT_FOREACH_PROP_ELEM(TP_NODE, feature_report_ids, FEATURE_GATT_ATTRS)

    /* HID Control Point */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

/* Work queue for BLE notifications */
K_THREAD_STACK_DEFINE(tp_hog_q_stack, 768);
static struct k_work_q tp_hog_work_q;

/* Message queue entry: report_id + data */
struct tp_report_msg {
    uint8_t report_id;
    uint8_t data[MAX(TP_MOUSE_INPUT_REPORT_SIZE, TP_PTP_INPUT_REPORT_SIZE)];
    uint16_t len;
};

K_MSGQ_DEFINE(tp_report_msgq, sizeof(struct tp_report_msg), 8, 4);

struct conn_collect_ctx {
    struct bt_conn *conns[CONFIG_BT_MAX_CONN];
    int count;
};

static void collect_conn_cb(struct bt_conn *conn, void *user_data) {
    struct conn_collect_ctx *ctx = user_data;
    if (ctx->count < CONFIG_BT_MAX_CONN) {
        ctx->conns[ctx->count] = bt_conn_ref(conn);
        ctx->count++;
    }
}

static void send_tp_report_callback(struct k_work *work) {
    struct tp_report_msg msg;

    while (k_msgq_get(&tp_report_msgq, &msg, K_NO_WAIT) == 0) {
        struct conn_collect_ctx ctx = { .count = 0 };
        bt_conn_foreach(BT_CONN_TYPE_LE, collect_conn_cb, &ctx);

        const struct bt_gatt_attr *attr;
        if (msg.report_id == TP_MOUSE_INPUT_REPORT_ID) {
            attr = &tp_hog_svc.attrs[TP_MOUSE_INPUT_ATTR_IDX];
            memcpy(cached_mouse_report, msg.data,
                   MIN(msg.len, sizeof(cached_mouse_report)));
        } else if (msg.report_id == TP_PTP_INPUT_REPORT_ID) {
            attr = &tp_hog_svc.attrs[TP_PTP_INPUT_ATTR_IDX];
            memcpy(cached_ptp_report, msg.data,
                   MIN(msg.len, sizeof(cached_ptp_report)));
        } else {
            for (int i = 0; i < ctx.count; i++) {
                bt_conn_unref(ctx.conns[i]);
            }
            continue;
        }

        for (int i = 0; i < ctx.count; i++) {
            struct bt_gatt_notify_params notify_params = {
                .attr = attr,
                .data = msg.data,
                .len = msg.len,
            };

            int err = bt_gatt_notify_cb(ctx.conns[i], &notify_params);
            if (err == -EPERM) {
                bt_conn_set_security(ctx.conns[i], BT_SECURITY_L2);
            } else if (err) {
                LOG_DBG("Error notifying %d", err);
            }
            bt_conn_unref(ctx.conns[i]);
        }
    }
}

K_WORK_DEFINE(tp_hog_work, send_tp_report_callback);

static void tp_ble_input_cb(const struct device *dev, uint8_t report_id,
                             const uint8_t *data, uint16_t len) {
    /* Only forward input reports (mouse and PTP) */
    if (report_id != TP_MOUSE_INPUT_REPORT_ID && report_id != TP_PTP_INPUT_REPORT_ID) {
        return;
    }

    struct tp_report_msg msg;
    msg.report_id = report_id;
    msg.len = MIN(len, sizeof(msg.data));
    memcpy(msg.data, data, msg.len);

    int err = k_msgq_put(&tp_report_msgq, &msg, K_NO_WAIT);
    if (err) {
        LOG_WRN("TP report queue full, dropping report ID %d", report_id);
        return;
    }

    k_work_submit_to_queue(&tp_hog_work_q, &tp_hog_work);
}

static int hid_passthrough_ble_init(void) {
    tp_dev = DEVICE_DT_GET(TP_NODE);
    if (!device_is_ready(tp_dev)) {
        LOG_ERR("Touchpad device not ready");
        return -ENODEV;
    }

    static const struct k_work_queue_config queue_config = {
        .name = "TP HOG Send Work"
    };
    k_work_queue_start(&tp_hog_work_q, tp_hog_q_stack,
                       K_THREAD_STACK_SIZEOF(tp_hog_q_stack), 5, &queue_config);

    hid_touchpad_register_input_cb(tp_dev, tp_ble_input_cb);

    LOG_INF("BLE HID passthrough initialized");
    return 0;
}

SYS_INIT(hid_passthrough_ble_init, APPLICATION, 96);

/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "hid_touchpad.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_ble, CONFIG_INPUT_LOG_LEVEL);

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

/* Report reference descriptors for all known reports */
static struct hids_report mouse_input_ref = { .id = 1, .type = HIDS_INPUT };
static struct hids_report ptp_input_ref = { .id = 4, .type = HIDS_INPUT };

static struct hids_report feature_2_ref = { .id = 2, .type = HIDS_FEATURE };
static struct hids_report feature_3_ref = { .id = 3, .type = HIDS_FEATURE };
static struct hids_report feature_5_ref = { .id = 5, .type = HIDS_FEATURE };
static struct hids_report feature_6_ref = { .id = 6, .type = HIDS_FEATURE };
static struct hids_report feature_7_ref = { .id = 7, .type = HIDS_FEATURE };
static struct hids_report feature_10_ref = { .id = 10, .type = HIDS_FEATURE };
static struct hids_report feature_11_ref = { .id = 11, .type = HIDS_FEATURE };
static struct hids_report feature_65_ref = { .id = 65, .type = HIDS_FEATURE };
static struct hids_report feature_66_ref = { .id = 66, .type = HIDS_FEATURE };
static struct hids_report feature_67_ref = { .id = 67, .type = HIDS_FEATURE };

static uint8_t ctrl_point;
static const struct device *tp_dev;

/* Cached last input reports for BLE read */
static uint8_t cached_mouse_report[8];
static uint8_t cached_ptp_report[29];

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

/*
 * Static GATT service definition for touchpad HID passthrough.
 *
 * Attribute index reference:
 *  0: Primary Service (HIDS)
 *  1: HID Info Characteristic declaration
 *  2: HID Info value
 *  3: Report Map Characteristic declaration
 *  4: Report Map value
 *
 *  5: Mouse Input Report (ID=1) Characteristic declaration
 *  6: Mouse Input Report value                         ← notify target for mouse
 *  7: Mouse Input Report CCC
 *  8: Mouse Input Report Reference
 *
 *  9: PTP Input Report (ID=4) Characteristic declaration
 * 10: PTP Input Report value                            ← notify target for PTP
 * 11: PTP Input Report CCC
 * 12: PTP Input Report Reference
 *
 * 13-14: Feature ID 2 (Characteristic decl + value)
 * 15: Feature ID 2 Report Reference
 * 16-17: Feature ID 3
 * 18: Feature ID 3 Report Reference
 * 19-20: Feature ID 5
 * 21: Feature ID 5 Report Reference
 * 22-23: Feature ID 6
 * 24: Feature ID 6 Report Reference
 * 25-26: Feature ID 7
 * 27: Feature ID 7 Report Reference
 * 28-29: Feature ID 10
 * 30: Feature ID 10 Report Reference
 * 31-32: Feature ID 11
 * 33: Feature ID 11 Report Reference
 * 34-35: Feature ID 65
 * 36: Feature ID 65 Report Reference
 * 37-38: Feature ID 66
 * 39: Feature ID 66 Report Reference
 * 40-41: Feature ID 67
 * 42: Feature ID 67 Report Reference
 *
 * 43: HID Control Point Characteristic declaration
 * 44: HID Control Point value
 */

/* Attribute indices for notify targets */
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

    /* Input Report ID 1 (Mouse) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_mouse_input_report, NULL, NULL),
    BT_GATT_CCC(tp_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &mouse_input_ref),

    /* Input Report ID 4 (PTP multitouch) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_ptp_input_report, NULL, NULL),
    BT_GATT_CCC(tp_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &ptp_input_ref),

    /* Feature Report ID 2 (Contact Max) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_2_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_2_ref),

    /* Feature Report ID 3 (Input Mode) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_3_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_3_ref),

    /* Feature Report ID 5 (Surface/Button Switch) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_5_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_5_ref),

    /* Feature Report ID 6 (Button Type) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_6_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_6_ref),

    /* Feature Report ID 7 (Vendor) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_7_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_7_ref),

    /* Feature Report ID 10 (Vendor, 256 bytes) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_10_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_10_ref),

    /* Feature Report ID 11 (Vendor) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_11_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_11_ref),

    /* Feature Report ID 65 (Vendor FW update, 256 bytes) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_65_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_65_ref),

    /* Feature Report ID 66 (Vendor FW update) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_66_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_66_ref),

    /* Feature Report ID 67 (Vendor FW update) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_feature_report, write_feature_report, &feature_67_ref),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,
                       read_hids_report_ref, NULL, &feature_67_ref),

    /* HID Control Point */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

/* Work queue for BLE notifications */
K_THREAD_STACK_DEFINE(tp_hog_q_stack, 768);
static struct k_work_q tp_hog_work_q;

/* Message queue entry: report_id + data */
struct tp_report_msg {
    uint8_t report_id;
    uint8_t data[29]; /* Max is PTP at 29 bytes */
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
        if (msg.report_id == 1) {
            attr = &tp_hog_svc.attrs[TP_MOUSE_INPUT_ATTR_IDX];
            memcpy(cached_mouse_report, msg.data,
                   MIN(msg.len, sizeof(cached_mouse_report)));
        } else if (msg.report_id == 4) {
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
    if (report_id != 1 && report_id != 4) {
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
    tp_dev = DEVICE_DT_GET(DT_NODELABEL(touchpad));
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

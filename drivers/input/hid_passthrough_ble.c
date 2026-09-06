/*
 * Copyright (c) 2025 The ZMK Contributors
 * Copyright (c) 2025-2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

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

/* HIDS HID Information. version is bcdHID, the HID *specification* version
 * we claim (1.11, like the keyboard HOG), NOT the pad's firmware version;
 * hosts may parse it as such. The pad FW version is host-readable via the
 * vendor feature reports instead. */
static struct hids_info tp_info = {
    .version = 0x0111,
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
    if (!ref || tp_dev == NULL || !device_is_ready(tp_dev)) {
        LOG_WRN("BLE read_feature_report: ref=%p tp_dev=%p", ref, tp_dev);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    LOG_DBG("BLE read feature id=%u len=%u offset=%u", ref->id, len, offset);

    /* One-entry response cache. A report longer than MTU-1 arrives as an
     * ATT read followed by read-blob requests at increasing offsets; each
     * used to re-run the I2C GET, so a 256-byte report at a 23-byte MTU
     * cost ~12 round trips and every chunk could come from a different
     * snapshot. Fetch on offset 0 and serve the blobs from the copy, as long
     * as they ask for the same id within the cached length; anything else
     * (a blob for another id, a stale offset) refetches. Single-central
     * device and all GATT callbacks run on the BT RX thread, so no locking
     * is needed. */
    static uint8_t feature_buf[264];
    static uint16_t feature_buf_len; /* 0 = nothing cached */
    static uint8_t feature_buf_id;

    bool cached = offset > 0 && feature_buf_len > 0 && feature_buf_id == ref->id &&
                  offset <= feature_buf_len; /* == : the empty terminating blob */
    if (!cached) {
        uint16_t out_len = 0;
        feature_buf_len = 0;
        int err = hid_touchpad_get_report(tp_dev, I2C_HID_REPORT_TYPE_FEATURE, ref->id,
                                          feature_buf, sizeof(feature_buf), &out_len);
        if (err) {
            LOG_ERR("BLE get feature report %d failed: %d", ref->id, err);
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
        feature_buf_len = out_len;
        feature_buf_id = ref->id;

        LOG_DBG("BLE feature id=%u returned %u bytes (first: 0x%02x)",
                ref->id, out_len, out_len > 0 ? feature_buf[0] : 0);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, feature_buf, feature_buf_len);
}

static ssize_t write_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags) {
    struct hids_report *ref = (struct hids_report *)attr->user_data;
    if (!ref || tp_dev == NULL || !device_is_ready(tp_dev)) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    if (offset != 0) {
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

/* Mouse-mode (id 1) reports are RELATIVE — coalescing would lose deltas —
 * so they pass through this queue unpaced. Only hosts that never switch
 * the pad to PTP mode ever see them. */
K_MSGQ_DEFINE(tp_report_msgq, sizeof(struct tp_report_msg), 8, 4);

/* PTP (id 4) frames carry ABSOLUTE positions, so only the newest pending
 * frame matters; older unsent motion is subsumed by it. Pure-motion frames
 * are paced to one notification per connection interval: BLE delivers
 * whatever is queued as a burst within one connection event, and
 * burst-mates (arrival deltas of ~50 us) trip libinput's touchpad jump
 * detection, which then DISCARDS their motion (~half the cursor travel,
 * measured 0.53x). One frame per connection event keeps host arrival
 * timestamps regular, and the scan-time field still carries true device
 * timing for hosts that use it. Frames with tip/confidence/contact-id/
 * button changes skip the pacing so tap/click latency is unaffected.
 *
 * The pace follows the interval the central actually granted (refreshed
 * from the active profile's connection, see tp_pace_refresh), minus a
 * small margin so scheduling jitter can't land two sends in one event. A
 * fixed pace that doesn't divide the interval quantizes onto the event
 * grid as a beat: 10 ms sends on the 7.5 ms both our centrals grant went
 * on air 15/7.5/7.5 ms apart with every third frame carrying double
 * travel. At 7.5 ms the pace (7.0 ms) sits below the pad period (~7.6 ms
 * in Run), so nothing is coalesced and every pad frame gets its own event;
 * at >= 10 ms grants pacing re-engages and coalesces regularly. The
 * Kconfig value is only the fallback when no connection info is available
 * (0 turns pacing off entirely). Arithmetic is in ticks: 7.5 ms is not
 * expressible in ms and a 0.5 ms/frame error would be the beat again. */
#define TP_PACE_FALLBACK_US (CONFIG_HID_PASSTHROUGH_BLE_PACE_MS * 1000)
#define TP_PACE_MARGIN_US   500
#define TP_PACE_MIN_US      2000
#define TP_PACE_ENABLED     (CONFIG_HID_PASSTHROUGH_BLE_PACE_MS > 0)

static struct tp_report_msg tp_pending;
static bool tp_pending_valid;
static bool tp_pending_urgent;
static struct k_spinlock tp_pending_lock;
static int64_t tp_last_send_ticks;
/* Current pace in ticks (0 = every frame goes out immediately); read and
 * written under tp_pending_lock. */
static k_ticks_t tp_pace_ticks;
/* PTP payload layout (report-descriptor order): TP_PTP_MAX_CONTACTS finger
 * records of (status byte + x u16 + y u16), then contact count (1), buttons
 * (1), scan time (2). Everything derives from ptp-input-report-size so a
 * pad with a different per-contact record still lands on the right bytes;
 * the contact count is fixed at 5 like the descriptor's. */
#define TP_PTP_MAX_CONTACTS 5
#define TP_PTP_TRAILER_LEN  4 /* count + buttons + scan time */
#define TP_PTP_FINGERS_LEN  (TP_PTP_INPUT_REPORT_SIZE - TP_PTP_TRAILER_LEN)
#define TP_PTP_STRIDE       (TP_PTP_FINGERS_LEN / TP_PTP_MAX_CONTACTS)
#define TP_PTP_COUNT_OFF    TP_PTP_FINGERS_LEN
#define TP_PTP_BTN_OFF      (TP_PTP_FINGERS_LEN + 1)
BUILD_ASSERT(TP_PTP_FINGERS_LEN > 0 && TP_PTP_FINGERS_LEN % TP_PTP_MAX_CONTACTS == 0,
             "ptp-input-report-size doesn't fit 5 equal contact records + 4 trailer bytes");
#define TP_PTP_STATE_LEN    (TP_PTP_MAX_CONTACTS + 2) /* status bytes + count + buttons */

/* Finger status bytes (confidence/tip/contact-id), contact count and
 * buttons of the last frame actually sent; a change makes a frame urgent.
 * Read and written only under tp_pending_lock: the input callback (TP
 * workq, higher coop priority) can preempt the sender mid-update, and a
 * torn read here would mark a tip/button transition non-urgent, letting a
 * later overwrite of the pending slot swallow it. */
static uint8_t tp_sent_state[TP_PTP_STATE_LEN];

/* Extract everything except positions and scan time. */
static void tp_frame_state(const uint8_t *data, uint16_t len, uint8_t out[TP_PTP_STATE_LEN]) {
    memset(out, 0, TP_PTP_STATE_LEN);
    for (int i = 0; i < TP_PTP_MAX_CONTACTS; i++) {
        if (i * TP_PTP_STRIDE < len) {
            out[i] = data[i * TP_PTP_STRIDE];
        }
    }
    if (len > TP_PTP_COUNT_OFF) {
        out[TP_PTP_MAX_CONTACTS] = data[TP_PTP_COUNT_OFF];
    }
    if (len > TP_PTP_BTN_OFF) {
        out[TP_PTP_MAX_CONTACTS + 1] = data[TP_PTP_BTN_OFF];
    }
}

/* Derive the pace from a connection's granted interval; NULL selects the
 * Kconfig fallback. Runs from the BT RX thread (conn callbacks), the system
 * workqueue (ZMK events) and init. */
static void tp_pace_refresh(struct bt_conn *conn) {
    struct bt_conn_info info;
    uint32_t interval_us = 0;
    uint32_t pace_us = TP_PACE_FALLBACK_US;

    if (conn != NULL && bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE &&
        info.le.interval_us > 0) {
        interval_us = info.le.interval_us;
        pace_us = MAX(interval_us - TP_PACE_MARGIN_US, TP_PACE_MIN_US);
    }

    k_ticks_t ticks = TP_PACE_ENABLED ? k_us_to_ticks_ceil64(pace_us) : 0;

    k_spinlock_key_t key = k_spin_lock(&tp_pending_lock);
    bool changed = ticks != tp_pace_ticks;
    tp_pace_ticks = ticks;
    k_spin_unlock(&tp_pending_lock, key);

    if (changed) {
        LOG_INF("PTP pace %u us (conn interval %u us%s)", TP_PACE_ENABLED ? pace_us : 0,
                interval_us, interval_us ? "" : ", fallback");
    }
}

/* The conn callbacks fire for every connection; only the active profile's
 * one sets the pace (same peer-address match ZMK uses to find it). */
static bool tp_conn_is_active_profile(struct bt_conn *conn) {
    return bt_addr_le_cmp(bt_conn_get_dst(conn), zmk_ble_active_profile_addr()) == 0;
}

static void tp_conn_connected(struct bt_conn *conn, uint8_t err) {
    if (err == 0 && tp_conn_is_active_profile(conn)) {
        tp_pace_refresh(conn);
    }
}

static void tp_conn_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (tp_conn_is_active_profile(conn)) {
        tp_pace_refresh(NULL);
    }
}

static void tp_conn_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                     uint16_t timeout) {
    if (tp_conn_is_active_profile(conn)) {
        tp_pace_refresh(conn);
    }
}

BT_CONN_CB_DEFINE(tp_conn_callbacks) = {
    .connected = tp_conn_connected,
    .disconnected = tp_conn_disconnected,
    .le_param_updated = tp_conn_le_param_updated,
};

/* A profile switch changes which connection's interval matters without any
 * connection-parameter event firing. */
static int tp_profile_changed_listener(const zmk_event_t *eh) {
    struct bt_conn *conn = NULL;
    if (zmk_ble_active_profile_is_connected()) {
        conn = zmk_ble_active_profile_conn();
    }
    tp_pace_refresh(conn);
    if (conn != NULL) {
        bt_conn_unref(conn);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(hid_passthrough_ble, tp_profile_changed_listener);
ZMK_SUBSCRIPTION(hid_passthrough_ble, zmk_ble_active_profile_changed);

static void tp_notify_conns(const struct tp_report_msg *msg) {
    const struct bt_gatt_attr *attr;
    if (msg->report_id == TP_MOUSE_INPUT_REPORT_ID) {
        attr = &tp_hog_svc.attrs[TP_MOUSE_INPUT_ATTR_IDX];
        memcpy(cached_mouse_report, msg->data,
               MIN(msg->len, sizeof(cached_mouse_report)));
    } else if (msg->report_id == TP_PTP_INPUT_REPORT_ID) {
        attr = &tp_hog_svc.attrs[TP_PTP_INPUT_ATTR_IDX];
        memcpy(cached_ptp_report, msg->data,
               MIN(msg->len, sizeof(cached_ptp_report)));
    } else {
        return;
    }

    /* Notify only the central for the currently selected BLE profile, mirroring
     * ZMK's keyboard HOG (zmk_ble_active_profile_conn). Fanning out to every
     * connected LE central would mirror the pad to all bonded hosts at once. */
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn == NULL) {
        return;
    }

    struct bt_gatt_notify_params notify_params = {
        .attr = attr,
        .data = msg->data,
        .len = msg->len,
    };

    /* A failed notify is a dropped frame (e.g. -ENOMEM = ATT TX buffer
     * exhaustion, -ENOTCONN/-EINVAL = no CCC). Warn on the first failure and
     * on every success->failure edge only: while connected+selected but the
     * CCC not yet enabled (fresh bond mid-enumeration) this fails for every
     * frame, and a WRN per frame at ~130 Hz drowns the log. */
    static bool notify_failing;
    int err = bt_gatt_notify_cb(conn, &notify_params);
    if (err == -EPERM) {
        bt_conn_set_security(conn, BT_SECURITY_L2);
    } else if (err) {
        if (!notify_failing) {
            LOG_WRN("Error notifying %d (further failures at DBG until one succeeds)", err);
            notify_failing = true;
        } else {
            LOG_DBG("Error notifying %d", err);
        }
    } else if (notify_failing) {
        notify_failing = false;
        LOG_INF("Notifications flowing again");
    }

    bt_conn_unref(conn);
}

static void send_tp_report_callback(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tp_send_work, send_tp_report_callback);

static void send_tp_report_callback(struct k_work *work) {
    struct tp_report_msg msg;

    /* Unpaced mouse-mode reports first */
    while (k_msgq_get(&tp_report_msgq, &msg, K_NO_WAIT) == 0) {
        tp_notify_conns(&msg);
    }

    k_spinlock_key_t key = k_spin_lock(&tp_pending_lock);
    if (!tp_pending_valid) {
        k_spin_unlock(&tp_pending_lock, key);
        return;
    }
    k_ticks_t due = tp_last_send_ticks + tp_pace_ticks - k_uptime_ticks();
    if (tp_pace_ticks > 0 && !tp_pending_urgent && due > 0) {
        k_spin_unlock(&tp_pending_lock, key);
        /* schedule (not reschedule): never push out an earlier deadline */
        k_work_schedule_for_queue(&tp_hog_work_q, &tp_send_work, K_TICKS(due));
        return;
    }
    msg = tp_pending;
    tp_pending_valid = false;
    tp_pending_urgent = false;
    tp_last_send_ticks = k_uptime_ticks();
    /* ~10 loads; cheap enough to keep under the lock (see tp_sent_state) */
    tp_frame_state(msg.data, msg.len, tp_sent_state);
    k_spin_unlock(&tp_pending_lock, key);

    tp_notify_conns(&msg);
}

static void tp_ble_input_cb(const struct device *dev, uint8_t report_id,
                             const uint8_t *data, uint16_t len) {
    /* Follow the same endpoint selection as the keyboard (zmk_endpoints):
     * only stream the pad over BLE when BLE is the selected transport, so it
     * doesn't mirror to both USB and BLE at once. */
    if (zmk_endpoint_get_selected().transport != ZMK_TRANSPORT_BLE) {
        return;
    }

    if (report_id == TP_MOUSE_INPUT_REPORT_ID) {
        struct tp_report_msg msg;
        msg.report_id = report_id;
        msg.len = MIN(len, sizeof(msg.data));
        memcpy(msg.data, data, msg.len);

        if (k_msgq_put(&tp_report_msgq, &msg, K_NO_WAIT) != 0) {
            LOG_WRN("TP report queue full, dropping report ID %d", report_id);
            return;
        }
        k_work_reschedule_for_queue(&tp_hog_work_q, &tp_send_work, K_NO_WAIT);
        return;
    }
    if (report_id != TP_PTP_INPUT_REPORT_ID) {
        return;
    }

    /* Urgency is judged against the last frame actually SENT, so once a
     * state change is pending, every following frame stays urgent until
     * one goes out — overwriting the pending frame can't swallow it. */
    uint8_t state[sizeof(tp_sent_state)];
    tp_frame_state(data, len, state);

    k_spinlock_key_t key = k_spin_lock(&tp_pending_lock);
    bool urgent = memcmp(state, tp_sent_state, sizeof(state)) != 0;
    tp_pending.report_id = report_id;
    tp_pending.len = MIN(len, sizeof(tp_pending.data));
    memcpy(tp_pending.data, data, tp_pending.len);
    tp_pending_valid = true;
    tp_pending_urgent = tp_pending_urgent || urgent;
    k_ticks_t due = tp_last_send_ticks + tp_pace_ticks - k_uptime_ticks();
    bool now = tp_pace_ticks <= 0 || tp_pending_urgent || due <= 0;
    k_spin_unlock(&tp_pending_lock, key);

    k_work_reschedule_for_queue(&tp_hog_work_q, &tp_send_work,
                                now ? K_NO_WAIT : K_TICKS(due));
}

/* TP_*_INPUT_ATTR_IDX index into the service array BT_GATT_SERVICE_DEFINE
 * builds, and any edit above the input characteristics silently shifts
 * them: notifications would then go out on the wrong characteristic with
 * no error. A BUILD_ASSERT can't see into that array, so check at init.
 * The UUID alone can't tell mouse from PTP; the read callback can. */
static bool tp_input_attr_ok(size_t idx, bt_gatt_attr_read_func_t read) {
    if (idx >= tp_hog_svc.attr_count) {
        return false;
    }
    const struct bt_gatt_attr *attr = &tp_hog_svc.attrs[idx];
    return bt_uuid_cmp(attr->uuid, BT_UUID_HIDS_REPORT) == 0 && attr->read == read;
}

static int hid_passthrough_ble_init(void) {
    /* The GATT service is registered statically whatever happens here, so a
     * pad that failed init still shows up to the host. Leave tp_dev NULL in
     * that case: the feature callbacks then answer with an ATT error instead
     * of clocking I2C transactions against an unconfigured device
     * (command_reg/data_reg = 0). Returning an error from SYS_INIT would buy
     * nothing, so stay idle like the USB backend does. */
    const struct device *dev = DEVICE_DT_GET(TP_NODE);
    if (!device_is_ready(dev)) {
        LOG_ERR("Touchpad device not ready, BLE passthrough idle");
        return 0;
    }
    tp_dev = dev;

    if (!tp_input_attr_ok(TP_MOUSE_INPUT_ATTR_IDX, read_mouse_input_report) ||
        !tp_input_attr_ok(TP_PTP_INPUT_ATTR_IDX, read_ptp_input_report)) {
        /* Leave the input callback unregistered: the pad keeps working over
         * USB rather than notifying garbage on some other characteristic. */
        LOG_ERR("Input report attribute indices don't match the service layout, "
                "BLE passthrough disabled");
        return 0;
    }

    static const struct k_work_queue_config queue_config = {
        .name = "TP HOG Send Work"
    };
    k_work_queue_start(&tp_hog_work_q, tp_hog_q_stack,
                       K_THREAD_STACK_SIZEOF(tp_hog_q_stack), 5, &queue_config);

    tp_pace_refresh(NULL);
    hid_touchpad_register_input_cb(tp_dev, tp_ble_input_cb);

    LOG_INF("BLE HID passthrough initialized");
    return 0;
}

SYS_INIT(hid_passthrough_ble_init, APPLICATION, 96);

/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <zephyr/sys/util_macro.h>

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE_HOG_PTP)
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/devicetree.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)

/**
 * Resample the pad's mouse reports by time before they go to a BLE host
 * (see tp_mouse_take in hid_passthrough_ble.c), for firmware that makes
 * them from the pad's PTP frames itself. Off by default: the pad's own
 * mouse mode goes as the pad sends it.
 */
void hid_passthrough_ble_resample_mouse(bool on);

#else

static inline void hid_passthrough_ble_resample_mouse(bool on) {}

#endif

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE_HOG_PTP)

/*
 * The precision touchpad in the keyboard's HID service, for hosts that use
 * only a device's first HID service (Android). The keyboard's service
 * (ZMK's hog.c) appends hid_passthrough_hog_report_map to its report map
 * and puts HID_PASSTHROUGH_HOG_ATTRS among its characteristics; this module
 * serves them and decides per host which service's PTP input gets the
 * frames.
 */

#define HID_PASSTHROUGH_HOG_NODE DT_NODELABEL(touchpad)

/** Bytes the keyboard's report map gets appended: hog-report-descriptor. */
#define HID_PASSTHROUGH_HOG_REPORT_MAP_SIZE                                                        \
    DT_PROP_LEN(HID_PASSTHROUGH_HOG_NODE, hog_report_descriptor)
extern const uint8_t hid_passthrough_hog_report_map[HID_PASSTHROUGH_HOG_REPORT_MAP_SIZE];

/** A report of the touchpad in the keyboard's service. */
struct hid_passthrough_hog_report {
    /* The Report Reference: report ID in the keyboard's map, type. */
    uint8_t ref[2];
    /* The pad's own ID for it. */
    uint8_t pad_id;
};

ssize_t hid_passthrough_hog_read_input(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset);
ssize_t hid_passthrough_hog_read_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
ssize_t hid_passthrough_hog_read_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len, uint16_t offset);
ssize_t hid_passthrough_hog_write_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset,
                                          uint8_t flags);
ssize_t hid_passthrough_hog_ccc_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      uint16_t value);

extern struct hid_passthrough_hog_report hid_passthrough_hog_input;
#define HID_PASSTHROUGH_HOG_FEATURE_DECL(node, prop, idx)                                         \
    extern struct hid_passthrough_hog_report hid_passthrough_hog_feature_##idx;
DT_FOREACH_PROP_ELEM(HID_PASSTHROUGH_HOG_NODE, hog_feature_report_ids,
                     HID_PASSTHROUGH_HOG_FEATURE_DECL)

/* Feature reports: Read + Write only, as in the pad's own service (HoGP 1.0
 * Table 4.3; Write Without Response makes BlueZ take it for an output). */
#define HID_PASSTHROUGH_HOG_FEATURE_ATTRS(node, prop, idx)                                        \
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,           \
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,                \
                           hid_passthrough_hog_read_feature, hid_passthrough_hog_write_feature,   \
                           &hid_passthrough_hog_feature_##idx),                                   \
        BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,                    \
                           hid_passthrough_hog_read_ref, NULL, &hid_passthrough_hog_feature_##idx),

/** The touchpad's characteristics in the keyboard's service, each followed by a comma. */
#define HID_PASSTHROUGH_HOG_ATTRS                                                                  \
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,          \
                           BT_GATT_PERM_READ_ENCRYPT, hid_passthrough_hog_read_input, NULL,       \
                           &hid_passthrough_hog_input),                                           \
        BT_GATT_CCC_WITH_WRITE_CB(NULL, hid_passthrough_hog_ccc_write,                            \
                                  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),        \
        BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT,                    \
                           hid_passthrough_hog_read_ref, NULL, &hid_passthrough_hog_input),       \
        DT_FOREACH_PROP_ELEM(HID_PASSTHROUGH_HOG_NODE, hog_feature_report_ids,                    \
                             HID_PASSTHROUGH_HOG_FEATURE_ATTRS)

#endif

#ifdef __cplusplus
}
#endif

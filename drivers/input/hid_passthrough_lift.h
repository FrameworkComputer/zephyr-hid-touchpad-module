/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The input reports a host keeps a button or contact down from, and the
 * report that lets go of it. A host holds the last report's state until it
 * gets another, so a backend whose reports stop going to a host (endpoint or
 * profile switch) sends it a lift first, or the host keeps dragging. */

#define TP_NODE DT_NODELABEL(touchpad)

#define TP_MOUSE_INPUT_REPORT_ID  DT_PROP(TP_NODE, mouse_input_report_id)
#define TP_PTP_INPUT_REPORT_ID    DT_PROP(TP_NODE, ptp_input_report_id)
#define TP_MOUSE_INPUT_REPORT_SIZE DT_PROP(TP_NODE, mouse_input_report_size)
#define TP_PTP_INPUT_REPORT_SIZE  DT_PROP(TP_NODE, ptp_input_report_size)

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
#define TP_PTP_SCAN_OFF     (TP_PTP_FINGERS_LEN + 2)
BUILD_ASSERT(TP_PTP_FINGERS_LEN > 0 && TP_PTP_FINGERS_LEN % TP_PTP_MAX_CONTACTS == 0,
             "ptp-input-report-size doesn't fit 5 equal contact records + 4 trailer bytes");

/* Finger status byte: confidence, tip, padding, contact id. */
#define TP_PTP_TIP BIT(1)

/* Scan time counts 100 us (PTP requires that unit); a lift goes one of the
 * pad's 7.1 ms frames after the report it follows. */
#define TP_PTP_LIFT_SCAN_STEP 71

/* Whether a host that got this input report (`data` without the report ID)
 * holds a button or a contact down. */
static inline bool tp_report_down(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (report_id == TP_MOUSE_INPUT_REPORT_ID) {
        return len > 0 && data[0] != 0;
    }
    if (report_id != TP_PTP_INPUT_REPORT_ID) {
        return false;
    }
    if (len > TP_PTP_BTN_OFF && data[TP_PTP_BTN_OFF] != 0) {
        return true;
    }
    for (int i = 0; i < TP_PTP_MAX_CONTACTS && i * TP_PTP_STRIDE < len; i++) {
        if (data[i * TP_PTP_STRIDE] & TP_PTP_TIP) {
            return true;
        }
    }
    return false;
}

/* Turn the last input report a host got into the one that lets go of it, in
 * place: a mouse report with no buttons and no motion, or a PTP frame with
 * no buttons and every contact's tip up where it was, which is how the pad
 * itself reports a lift. */
static inline void tp_report_lift(uint8_t report_id, uint8_t *data, uint16_t len) {
    if (report_id == TP_MOUSE_INPUT_REPORT_ID) {
        memset(data, 0, len);
        return;
    }
    if (report_id != TP_PTP_INPUT_REPORT_ID) {
        return;
    }
    for (int i = 0; i < TP_PTP_MAX_CONTACTS && i * TP_PTP_STRIDE < len; i++) {
        data[i * TP_PTP_STRIDE] &= ~TP_PTP_TIP;
    }
    if (len > TP_PTP_BTN_OFF) {
        data[TP_PTP_BTN_OFF] = 0;
    }
    if (len >= TP_PTP_SCAN_OFF + 2) {
        sys_put_le16(sys_get_le16(&data[TP_PTP_SCAN_OFF]) + TP_PTP_LIFT_SCAN_STEP,
                     &data[TP_PTP_SCAN_OFF]);
    }
}

#ifdef __cplusplus
}
#endif

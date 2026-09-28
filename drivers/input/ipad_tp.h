/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Present a PTP touchpad as an accessory trackpad per Apple's Accessory
 * Design Guidelines, chapter 15 (iPadOS 14.5+): one Digitizer Touch Pad
 * collection, five Finger collections, and no Input Mode -- the host never
 * switches it, the pad reports contacts from the start. Unlike apple_mt.c it
 * needs no Apple VID, which the chapter forbids.
 *
 * Layout follows the chapter's example descriptor (15.2.1): report 3 carries
 * Relative Scan Time, two buttons and five fingers of tip, confidence,
 * a 6-bit transducer index and 12-bit X/Y; feature report 3 carries Surface
 * Switch and Report Rate.
 *
 * Plain C with no Zephyr dependency, so it can be tested on a host.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IPAD_TP_REPORT_ID 0x03
#define IPAD_TP_FINGERS   5
/* ID, 16-bit scan time, button byte, five 4-byte fingers. */
#define IPAD_TP_REPORT_BYTES (1 + 2 + 1 + 4 * IPAD_TP_FINGERS)
#define IPAD_TP_DESC_MAX     400

struct ipad_tp {
    uint8_t desc[IPAD_TP_DESC_MAX];
    size_t desc_len;

    bool surface_on;
    uint16_t report_rate;

    /* PTP contact ID holding each transducer index, or -1. */
    int8_t index_owner[IPAD_TP_FINGERS];
    /* Indexes whose contact lifted in the last report: zeroed in the next. */
    bool lifted[IPAD_TP_FINGERS];
};

/*
 * x_max/y_max: PTP logical maxima (< 4096, the fields are 12 bits).
 * width/height: surface size in 0.1 mm.
 */
void ipad_tp_init(struct ipad_tp *tp, uint16_t x_max, uint16_t y_max, uint16_t width_01mm,
                  uint16_t height_01mm, uint16_t report_rate_hz);

/* GET_REPORT: answer, report ID first, and its length; -1 if unknown. */
int ipad_tp_get_report(struct ipad_tp *tp, uint8_t id, uint8_t *buf, size_t len);

/* SET_REPORT with the report ID in buf[0]. */
void ipad_tp_set_report(struct ipad_tp *tp, uint8_t id, const uint8_t *buf, size_t len);

/*
 * Re-encode one PTP input report (payload after its ID; the layout
 * apple_mt_frame() documents) into `out` (IPAD_TP_REPORT_BYTES). Returns
 * the length, or 0 if the report is too short.
 */
size_t ipad_tp_frame(struct ipad_tp *tp, const uint8_t *ptp, size_t len, uint8_t *out);

#ifdef __cplusplus
}
#endif

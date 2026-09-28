/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Present a PTP touchpad to macOS as an Apple multitouch trackpad.
 *
 * macOS never turns PTP digitizer reports into pointer motion; only its own
 * multitouch stack (AppleMultitouchTrackpadHIDEventDriver) does gestures.
 * That driver binds an Apple top-case trackpad by VID/PID, asks it a fixed
 * set of feature reports, and then reads Magic Trackpad 2 style frames on
 * input report 0x02. The descriptor, the feature answers and the frame
 * layout are VoodooInput's (VoodooInputSimulatorDevice), which feeds the
 * same driver on Hackintoshes; the finger encoding matches Linux
 * hid-magicmouse's MT2 decoder.
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

#define APPLE_MT_INPUT_REPORT_ID 0x02
#define APPLE_MT_HEADER_BYTES    12
#define APPLE_MT_FINGER_BYTES    9
#define APPLE_MT_MAX_FINGERS     5
#define APPLE_MT_MAX_FRAME       (APPLE_MT_HEADER_BYTES + APPLE_MT_FINGER_BYTES * APPLE_MT_MAX_FINGERS)

/* Contact IDs are 4 bits in both PTP and the MT2 frame. */
#define APPLE_MT_CONTACT_IDS 16

extern const uint8_t apple_mt_report_desc[];
extern const size_t apple_mt_report_desc_size;

struct apple_mt {
    /* PTP logical maxima of X and Y. */
    uint16_t x_max;
    uint16_t y_max;
    /* Surface size in 0.01 mm, as the D9 feature report states it. */
    uint16_t width;
    uint16_t height;

    /* Answer to the next GET_REPORT of ID 0x01, chosen by the last SET. */
    uint8_t pending[20];
    uint8_t pending_len;

    bool touching[APPLE_MT_CONTACT_IDS];

    /* The last frame sent, which the lift sequence repeats. */
    uint8_t last[APPLE_MT_MAX_FRAME];
    uint8_t last_len;
};

void apple_mt_init(struct apple_mt *mt, uint16_t x_max, uint16_t y_max, uint16_t width_01mm,
                   uint16_t height_01mm);

/*
 * GET_REPORT for `id`: writes the answer, report ID first, into `buf` and
 * returns its length (truncated to `len`), or -1 for a report we don't know.
 */
int apple_mt_get_report(struct apple_mt *mt, uint8_t id, uint8_t *buf, size_t len);

/* SET_REPORT with the report ID in buf[0]. */
void apple_mt_set_report(struct apple_mt *mt, uint8_t id, const uint8_t *buf, size_t len);

/*
 * Encode one PTP input report (payload after the report ID) as an MT2 frame,
 * report ID first, into `out` (APPLE_MT_MAX_FRAME bytes). Returns the frame
 * length, or 0 when the report is not one we can read.
 *
 * The PTP layout is the Windows Precision Touchpad one this pad declares:
 * 5-byte finger slots (confidence bit 0, tip bit 1, contact ID in bits 4-7,
 * then 16-bit X and Y), followed by contact count, a button byte and a
 * 16-bit scan time.
 *
 * `*lifted` is set when no finger is down and the button is up; the caller
 * then sends apple_mt_lift_frame() for steps 0, 1 and 2.
 */
size_t apple_mt_frame(struct apple_mt *mt, const uint8_t *ptp, size_t len, uint32_t now_ms,
                      uint8_t *out, bool *lifted);

/*
 * The frames VoodooInput sends after the last finger lifts; the multitouch
 * stack ends the gesture on them. `step` is 0, 1 or 2.
 */
size_t apple_mt_lift_frame(struct apple_mt *mt, int step, uint32_t now_ms, uint8_t *out);

#ifdef __cplusplus
}
#endif

/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#include "ipad_tp.h"

#include <string.h>

#define PTP_FINGER_BYTES 5
#define PTP_TAIL_BYTES   4 /* contact count, button byte, 16-bit scan time */

static size_t emit(uint8_t *d, size_t n, const uint8_t *items, size_t len) {
    memcpy(d + n, items, len);
    return n + len;
}

/* Item for item the chapter's example (15.2.1), which is byte-compared in
 * tests/host/ipad_tp_test.c: the first finger inherits Report Size 1 and
 * Logical Maximum 1 from the button items and declares the units, the
 * others restate the first two. */
static size_t finger_collection(uint8_t *d, size_t n, uint16_t x_max, uint16_t y_max,
                                uint16_t width, uint16_t height, bool first) {
    const uint8_t first_head[] = {
        0x05, 0x0D, /* Usage Page (Digitizer) */
        0x09, 0x22, /* Usage (Finger) */
        0xA1, 0x00, /* Collection (Physical) */
        0x09, 0x42, /* Usage (Tip Switch) */
        0x09, 0x47, /* Usage (Confidence) */
        0x95, 0x02, /* Report Count (2) */
        0x81, 0x02, /* Input (Data,Var,Abs) */
        0x25, 0x05, /* Logical Maximum (5) */
        0x09, 0x38, /* Usage (Transducer Index) */
        0x75, 0x06, /* Report Size (6) */
        0x95, 0x01, /* Report Count (1) */
        0x81, 0x02, /* Input (Data,Var,Abs) */
        0x05, 0x01, /* Usage Page (Generic Desktop) */
        0x46, width & 0xff, width >> 8, /* Physical Maximum */
        0x35, 0x00,                     /* Physical Minimum (0) */
        0x26, x_max & 0xff, x_max >> 8, /* Logical Maximum */
        0x75, 0x0C,                     /* Report Size (12) */
        0x55, 0x0E,                     /* Unit Exponent (-2) */
        0x65, 0x11,                     /* Unit (cm): with the exponent, 0.1 mm */
    };
    const uint8_t other_head[] = {
        0x05, 0x0D, /* Usage Page (Digitizer) */
        0x09, 0x22, /* Usage (Finger) */
        0xA1, 0x00, /* Collection (Physical) */
        0x25, 0x01, /* Logical Maximum (1) */
        0x09, 0x42, /* Usage (Tip Switch) */
        0x09, 0x47, /* Usage (Confidence) */
        0x75, 0x01, /* Report Size (1) */
        0x95, 0x02, /* Report Count (2) */
        0x81, 0x02, /* Input (Data,Var,Abs) */
        0x09, 0x38, /* Usage (Transducer Index) */
        0x25, 0x05, /* Logical Maximum (5) */
        0x75, 0x06, /* Report Size (6) */
        0x95, 0x01, /* Report Count (1) */
        0x81, 0x02, /* Input (Data,Var,Abs) */
        0x05, 0x01, /* Usage Page (Generic Desktop) */
        0x46, width & 0xff, width >> 8, /* Physical Maximum */
        0x26, x_max & 0xff, x_max >> 8, /* Logical Maximum */
        0x75, 0x0C,                     /* Report Size (12) */
    };
    const uint8_t xy[] = {
        0x09, 0x30,                       /* Usage (X) */
        0x81, 0x02,                       /* Input (Data,Var,Abs) */
        0x46, height & 0xff, height >> 8, /* Physical Maximum */
        0x26, y_max & 0xff, y_max >> 8,   /* Logical Maximum */
        0x09, 0x31,                       /* Usage (Y) */
        0x81, 0x02,                       /* Input (Data,Var,Abs) */
        0xC0,                             /* End Collection */
    };

    n = first ? emit(d, n, first_head, sizeof(first_head))
              : emit(d, n, other_head, sizeof(other_head));
    return emit(d, n, xy, sizeof(xy));
}

void ipad_tp_init(struct ipad_tp *tp, uint16_t x_max, uint16_t y_max, uint16_t width_01mm,
                  uint16_t height_01mm, uint16_t report_rate_hz) {
    static const uint8_t head[] = {
        0x05, 0x0D,                   /* Usage Page (Digitizer) */
        0x09, 0x05,                   /* Usage (Touch Pad) */
        0xA1, 0x01,                   /* Collection (Application) */
        0x85, IPAD_TP_REPORT_ID,      /* Report ID (3) */
        0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535) */
        0x09, 0x56,                   /* Usage (Relative Scan Time) */
        0x75, 0x10,                   /* Report Size (16) */
        0x95, 0x01,                   /* Report Count (1) */
        0x81, 0x02,                   /* Input (Data,Var,Abs) */
        0x25, 0x01,                   /* Logical Maximum (1) */
        0x75, 0x01,                   /* Report Size (1) */
        0x09, 0x57,                   /* Usage (Surface Switch) */
        0xB1, 0x02,                   /* Feature (Data,Var,Abs) */
        0x26, 0xFF, 0x7F,             /* Logical Maximum (32767) */
        0x75, 0x0F,                   /* Report Size (15) */
        0x09, 0xA1,                   /* Usage (Report Rate) */
        0xB1, 0x02,                   /* Feature (Data,Var,Abs) */
        0x05, 0x09,                   /* Usage Page (Button) */
        0x25, 0x01,                   /* Logical Maximum (1) */
        0x19, 0x01,                   /* Usage Minimum (Button 1) */
        0x29, 0x02,                   /* Usage Maximum (Button 2) */
        0x95, 0x02,                   /* Report Count (2) */
        0x75, 0x01,                   /* Report Size (1) */
        0x81, 0x02,                   /* Input (Data,Var,Abs) */
        0x95, 0x06,                   /* Report Count (6) */
        0x81, 0x01,                   /* Input (Const) */
    };

    memset(tp, 0, sizeof(*tp));
    size_t n = emit(tp->desc, 0, head, sizeof(head));
    for (int i = 0; i < IPAD_TP_FINGERS; i++) {
        n = finger_collection(tp->desc, n, x_max, y_max, width_01mm, height_01mm, i == 0);
    }
    tp->desc[n++] = 0xC0; /* End Collection (Application) */
    tp->desc_len = n;

    tp->surface_on = true;
    tp->report_rate = report_rate_hz & 0x7fff;
    memset(tp->index_owner, -1, sizeof(tp->index_owner));
}

int ipad_tp_get_report(struct ipad_tp *tp, uint8_t id, uint8_t *buf, size_t len) {
    if (id != IPAD_TP_REPORT_ID || len < 1) {
        return -1;
    }
    uint16_t v = (uint16_t)(tp->surface_on | (tp->report_rate << 1));
    uint8_t b[3] = {IPAD_TP_REPORT_ID, v & 0xff, v >> 8};
    size_t n = len < sizeof(b) ? len : sizeof(b);
    memcpy(buf, b, n);
    return (int)n;
}

void ipad_tp_set_report(struct ipad_tp *tp, uint8_t id, const uint8_t *buf, size_t len) {
    if (id == IPAD_TP_REPORT_ID && len >= 2) {
        tp->surface_on = buf[1] & 0x01;
    }
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int index_of(const struct ipad_tp *tp, int contact) {
    for (int i = 0; i < IPAD_TP_FINGERS; i++) {
        if (tp->index_owner[i] == contact) {
            return i;
        }
    }
    return -1;
}

static void put_finger(uint8_t *f, bool tip, bool confident, int index, uint16_t x, uint16_t y) {
    f[0] = (uint8_t)((tip ? 0x01 : 0) | (confident ? 0x02 : 0) | (index << 2));
    f[1] = x & 0xff;
    f[2] = (uint8_t)(((x >> 8) & 0x0f) | ((y & 0x0f) << 4));
    f[3] = (uint8_t)(y >> 4);
}

size_t ipad_tp_frame(struct ipad_tp *tp, const uint8_t *ptp, size_t len, uint8_t *out) {
    if (len < PTP_FINGER_BYTES + PTP_TAIL_BYTES) {
        return 0;
    }
    size_t slots = (len - PTP_TAIL_BYTES) / PTP_FINGER_BYTES;
    const uint8_t *tail = ptp + slots * PTP_FINGER_BYTES;
    size_t contacts = tail[0] < slots ? tail[0] : slots;

    memset(out, 0, IPAD_TP_REPORT_BYTES);
    out[0] = IPAD_TP_REPORT_ID;
    out[1] = tail[2];
    out[2] = tail[3];
    out[3] = tail[1] & 0x01;

    /* A contact reported lifted last time is gone now; its fields stay 0. */
    for (int i = 0; i < IPAD_TP_FINGERS; i++) {
        if (tp->lifted[i]) {
            tp->lifted[i] = false;
            tp->index_owner[i] = -1;
        }
    }
    if (!tp->surface_on) {
        return IPAD_TP_REPORT_BYTES;
    }

    for (size_t s = 0; s < contacts; s++) {
        const uint8_t *p = ptp + s * PTP_FINGER_BYTES;
        bool confident = p[0] & 0x01;
        bool tip = p[0] & 0x02;
        int contact = p[0] >> 4;
        int i = index_of(tp, contact);

        if (i < 0) {
            if (!tip) {
                continue; /* lift of a contact we never reported */
            }
            i = index_of(tp, -1);
            if (i < 0) {
                continue; /* more than five contacts */
            }
            tp->index_owner[i] = (int8_t)contact;
        }
        uint16_t x = le16(p + 1) & 0x0fff;
        uint16_t y = le16(p + 3) & 0x0fff;
        /* A lifted contact goes out once with tip 0, confidence 1 and its
         * last coordinates, then as zeros (example 15.2.2, steps 4 and 5). */
        put_finger(out + 4 + 4 * i, tip, tip ? confident : true, i, x, y);
        if (!tip) {
            tp->lifted[i] = true;
        }
    }
    return IPAD_TP_REPORT_BYTES;
}

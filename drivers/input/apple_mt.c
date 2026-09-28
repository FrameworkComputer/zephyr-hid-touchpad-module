/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#include "apple_mt.h"

#include <string.h>

/* VoodooInputSimulatorDevice's descriptor: a Mouse collection first, which
 * makes the interface's primary usage Mouse (what the driver matches), then
 * the empty Touch Pad and vendor collections Apple's trackpads declare. */
const uint8_t apple_mt_report_desc[] = {
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29,
    0x03, 0x15, 0x00, 0x25, 0x01, 0x85, 0x02, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x05, 0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7f, 0x75,
    0x08, 0x95, 0x02, 0x81, 0x06, 0x95, 0x04, 0x75, 0x08, 0x81, 0x01, 0xc0, 0xc0, 0x05, 0x0d,
    0x09, 0x05, 0xa1, 0x01, 0x06, 0x00, 0xff, 0x09, 0x0c, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75,
    0x08, 0x95, 0x10, 0x85, 0x3f, 0x81, 0x22, 0xc0, 0x06, 0x00, 0xff, 0x09, 0x0c, 0xa1, 0x01,
    0x06, 0x00, 0xff, 0x09, 0x0c, 0x15, 0x00, 0x26, 0xff, 0x00, 0x85, 0x44, 0x75, 0x08, 0x96,
    0x6b, 0x05, 0x81, 0x00, 0xc0,
};
const size_t apple_mt_report_desc_size = sizeof(apple_mt_report_desc);

/* The coordinate span VoodooInput scales every pad to; the D9 surface size
 * tells the driver what it means physically. */
#define MT2_MAX_X 8134
#define MT2_MAX_Y 5206

/* Finger state, bits 5-7 of byte 3 (hid-magicmouse: down == 0x80). */
#define STATE_INACTIVE 0x0
#define STATE_START    0x3
#define STATE_ACTIVE   0x4
#define STATE_STOP     0x7

#define FINGER_UNDEFINED 0
#define FINGER_THUMB     1
#define FINGER_INDEX     2
#define FINGER_LITTLE    5
#define FINGER_PALM      6

#define PTP_FINGER_BYTES 5
#define PTP_TAIL_BYTES   4 /* contact count, button byte, 16-bit scan time */

static const uint8_t sensor_rows_cols[] = {0xD3, 0x01, 0x16, 0x1E, 0x03, 0x95, 0x00,
                                           0x14, 0x1E, 0x62, 0x05, 0x00, 0x00};
static const uint8_t sensor_region[] = {0xD0, 0x02, 0x01, 0x00, 0x14, 0x01, 0x00, 0x1E,
                                        0x00, 0x02, 0x14, 0x02, 0x01, 0x0E, 0x02, 0x00};
static const uint8_t sensor_region_param[] = {0xA1, 0x00, 0x00, 0x05, 0x00, 0xFC, 0x01};

void apple_mt_init(struct apple_mt *mt, uint16_t x_max, uint16_t y_max, uint16_t width_01mm,
                   uint16_t height_01mm) {
    memset(mt, 0, sizeof(*mt));
    mt->x_max = x_max ? x_max : 1;
    mt->y_max = y_max ? y_max : 1;
    mt->width = width_01mm;
    mt->height = height_01mm;
}

static size_t surface(const struct apple_mt *mt, uint8_t *b) {
    const uint8_t d[] = {0xD9, mt->width & 0xff, mt->width >> 8, 0x00, 0x00,
                         mt->height & 0xff, mt->height >> 8, 0x00, 0x00,
                         0x44, 0xE3, 0x52, 0xFF, 0xBD, 0x1E, 0xE4, 0x26};
    memcpy(b, d, sizeof(d));
    return sizeof(d);
}

static size_t put(uint8_t *b, const uint8_t *src, size_t n) {
    memcpy(b, src, n);
    return n;
}

int apple_mt_get_report(struct apple_mt *mt, uint8_t id, uint8_t *buf, size_t len) {
    uint8_t b[80];
    size_t n = 0;

    switch (id) {
    case 0x00:
        b[n++] = 0x00;
        b[n++] = 0x01;
        break;
    case 0x01:
        if (mt->pending_len == 0) {
            return -1;
        }
        n = put(b, mt->pending, mt->pending_len);
        break;
    case 0x02:
        b[n++] = 0x02;
        b[n++] = 0x01;
        break;
    case 0xD1: /* family ID */
        b[n++] = 0xD1;
        b[n++] = 0x81;
        break;
    case 0xD3:
        n = put(b, sensor_rows_cols, sizeof(sensor_rows_cols));
        break;
    case 0xD0:
        n = put(b, sensor_region, sizeof(sensor_region));
        break;
    case 0xA1:
        n = put(b, sensor_region_param, sizeof(sensor_region_param));
        break;
    case 0xD9:
        n = surface(mt, b);
        break;
    case 0x7F:
        memset(b, 0, 5);
        b[0] = 0x7F;
        n = 5;
        break;
    case 0xC8:
        b[n++] = 0xC8;
        b[n++] = 0x08;
        break;
    case 0xDB: {
        /* All of the above in one report, each block after a length pair. */
        static const uint8_t head[] = {0xDB, 0x01, 0x02, 0x00, 0xD1, 0x81, 0x0D, 0x00};
        n = put(b, head, sizeof(head));
        n += put(b + n, sensor_rows_cols, sizeof(sensor_rows_cols));
        b[n++] = 0x10;
        b[n++] = 0x00;
        n += put(b + n, sensor_region, sizeof(sensor_region));
        b[n++] = 0x07;
        b[n++] = 0x00;
        n += put(b + n, sensor_region_param, sizeof(sensor_region_param));
        b[n++] = 0x11;
        b[n++] = 0x00;
        n += surface(mt, b + n);
        memset(b + n, 0, 5);
        b[n] = 0x7F;
        n += 5;
        break;
    }
    default:
        return -1;
    }

    if (n > len) {
        n = len;
    }
    memcpy(buf, b, n);
    return (int)n;
}

void apple_mt_set_report(struct apple_mt *mt, uint8_t id, const uint8_t *buf, size_t len) {
    if (id != 0x01 || len < 2) {
        return;
    }
    /* The driver writes a command byte to report 0x01 and reads the answer
     * back from the same report. */
    uint8_t cmd = buf[1];
    uint8_t *p = mt->pending;
    uint8_t v;

    switch (cmd) {
    case 0xDB: v = 0x49; break;
    case 0xD1: v = 0x01; break;
    case 0xD3: v = 0x0C; break;
    case 0xD0: v = 0x0F; break;
    case 0xA1: v = 0x06; break;
    case 0x7F: v = 0x04; break;
    case 0xC8: v = 0x01; break;
    case 0xD9:
        /* VoodooInput answers this one with the D9 report itself. */
        mt->pending_len = (uint8_t)surface(mt, p);
        return;
    default:
        mt->pending_len = 0;
        return;
    }
    p[0] = 0x01;
    p[1] = cmd;
    p[2] = 0x00;
    p[3] = v;
    p[4] = 0x00;
    mt->pending_len = 5;
}

static void timestamp(uint8_t *hdr, uint32_t ms) {
    hdr[9] = (uint8_t)((ms << 3) | 0x4);
    hdr[10] = (uint8_t)((ms >> 5) & 0xff);
    hdr[11] = (uint8_t)((ms >> 13) & 0xff);
}

static void put_finger(uint8_t *f, int16_t x, int16_t y, uint8_t finger, uint8_t state,
                       uint8_t major, uint8_t minor, uint8_t size, uint8_t pressure, uint8_t id,
                       uint8_t angle) {
    uint16_t ux = (uint16_t)x & 0x1fff;
    uint16_t uy = (uint16_t)y & 0x1fff;

    f[0] = ux & 0xff;
    f[1] = (uint8_t)((ux >> 8) | ((uy & 0x07) << 5));
    f[2] = (uint8_t)(uy >> 3);
    f[3] = (uint8_t)((uy >> 11) | ((finger & 0x7) << 2) | ((state & 0x7) << 5));
    f[4] = major;
    f[5] = minor;
    f[6] = size;
    f[7] = pressure;
    f[8] = (uint8_t)((id & 0xf) | ((angle & 0x7) << 5));
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

size_t apple_mt_frame(struct apple_mt *mt, const uint8_t *ptp, size_t len, uint32_t now_ms,
                      uint8_t *out, bool *lifted) {
    if (len < PTP_FINGER_BYTES + PTP_TAIL_BYTES) {
        return 0;
    }
    size_t slots = (len - PTP_TAIL_BYTES) / PTP_FINGER_BYTES;
    const uint8_t *tail = ptp + slots * PTP_FINGER_BYTES;
    size_t contacts = tail[0];
    bool button = tail[1] & 0x01;

    if (contacts > slots) {
        contacts = slots;
    }
    if (contacts > APPLE_MT_MAX_FINGERS) {
        contacts = APPLE_MT_MAX_FINGERS;
    }

    memset(out, 0, APPLE_MT_HEADER_BYTES);
    out[0] = APPLE_MT_INPUT_REPORT_ID;
    out[1] = button;
    out[8] = 0x31;
    timestamp(out, now_ms);

    bool active = button;
    size_t n = 0;

    for (size_t i = 0; i < contacts; i++) {
        const uint8_t *s = ptp + i * PTP_FINGER_BYTES;
        bool confident = s[0] & 0x01;
        bool tip = s[0] & 0x02;
        uint8_t id = s[0] >> 4;
        uint16_t px = le16(s + 1);
        uint16_t py = le16(s + 3);

        if (!tip && !mt->touching[id]) {
            continue; /* a slot the pad left empty */
        }

        int32_t sx = (int32_t)px * MT2_MAX_X / mt->x_max;
        int32_t sy = (int32_t)py * MT2_MAX_Y / mt->y_max;
        int16_t x = (int16_t)(sx - MT2_MAX_X / 2);
        int16_t y = (int16_t)(-(sy - MT2_MAX_Y / 2));

        uint8_t finger = !confident ? FINGER_PALM
                         : n < 4    ? (uint8_t)(FINGER_INDEX + n)
                                    : FINGER_THUMB;
        uint8_t state = mt->touching[id] ? STATE_ACTIVE : STATE_START;
        uint8_t pressure = button ? 120 : 5;
        uint8_t size = 10, major = 20, minor = 20;

        if (!tip) {
            state = STATE_STOP;
            pressure = size = major = minor = 0;
        }
        mt->touching[id] = tip;
        active |= tip;

        put_finger(out + APPLE_MT_HEADER_BYTES + n * APPLE_MT_FINGER_BYTES, x, y, finger, state,
                   major, minor, size, pressure, (uint8_t)(id + 1), 4);
        n++;
    }

    out[7] = active ? 0x3 : 0x2;
    *lifted = !active;
    if (!active) {
        memset(mt->touching, 0, sizeof(mt->touching));
    }

    size_t total = APPLE_MT_HEADER_BYTES + n * APPLE_MT_FINGER_BYTES;
    memcpy(mt->last, out, total);
    mt->last_len = (uint8_t)total;
    return total;
}

size_t apple_mt_lift_frame(struct apple_mt *mt, int step, uint32_t now_ms, uint8_t *out) {
    uint8_t *f0 = mt->last + APPLE_MT_HEADER_BYTES;
    bool has_finger = mt->last_len > APPLE_MT_HEADER_BYTES;

    switch (step) {
    case 0:
        if (has_finger) {
            f0[4] = f0[5] = f0[6] = f0[7] = 0; /* major, minor, size, pressure */
        }
        timestamp(mt->last, now_ms);
        break;
    case 1:
        if (has_finger) {
            /* finger type undefined, state inactive; keep Y's top bits */
            f0[3] &= 0x03;
        }
        break;
    default:
        timestamp(mt->last, now_ms);
        mt->last_len = APPLE_MT_HEADER_BYTES;
        break;
    }
    memcpy(out, mt->last, mt->last_len);
    return mt->last_len;
}

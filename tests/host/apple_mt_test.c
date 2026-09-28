/*
 * Host test for drivers/input/apple_mt.c: encode PTP frames and decode them
 * with Linux hid-magicmouse's MT2 formulas (magicmouse_emit_touch).
 *
 *   cc -I../../drivers/input -o apple_mt_test apple_mt_test.c ../../drivers/input/apple_mt.c
 *   ./apple_mt_test
 */

#include "apple_mt.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

struct decoded {
    int id, x, y, state, finger, pressure;
    int down;
    /* All three state bits; Linux masks only the top two (0xC0). */
    int state3;
};

/* drivers/hid/hid-magicmouse.c, the MAGICTRACKPAD2 branch. */
static struct decoded decode(const uint8_t *t) {
    struct decoded d;
    d.id = t[8] & 0xf;
    d.x = (int)((unsigned)t[1] << 27 | (unsigned)t[0] << 19) >> 19;
    d.y = -((int)((unsigned)t[3] << 30 | (unsigned)t[2] << 22 | (unsigned)t[1] << 14) >> 19);
    d.pressure = t[7];
    d.state = t[3] & 0xC0;
    d.down = d.state == 0x80;
    d.finger = (t[3] >> 2) & 0x7;
    d.state3 = t[3] >> 5;
    return d;
}

static void slot(uint8_t *s, int tip, int conf, int id, int x, int y) {
    s[0] = (uint8_t)((conf ? 1 : 0) | (tip ? 2 : 0) | (id << 4));
    s[1] = x & 0xff;
    s[2] = x >> 8;
    s[3] = y & 0xff;
    s[4] = y >> 8;
}

int main(void) {
    struct apple_mt mt;
    uint8_t ptp[29], out[APPLE_MT_MAX_FRAME];
    bool lifted;

    apple_mt_init(&mt, 796, 998, 6640, 8320);

    /* One finger lands in the top-left corner. */
    memset(ptp, 0, sizeof(ptp));
    slot(ptp, 1, 1, 3, 0, 0);
    ptp[25] = 1;
    size_t n = apple_mt_frame(&mt, ptp, sizeof(ptp), 100, out, &lifted);
    assert(n == 12 + 9 && out[0] == 0x02 && out[8] == 0x31 && out[7] == 0x3 && !lifted);
    struct decoded d = decode(out + 12);
    printf("land   id=%d x=%d y=%d state=0x%02x finger=%d\n", d.id, d.x, d.y, d.state, d.finger);
    assert(d.id == 4 && d.x == -4067 && d.y == -2603 && d.state3 == 0x3 && !d.down &&
           d.finger == 2);

    /* It moves to the bottom-right corner, a second finger lands mid-pad. */
    memset(ptp, 0, sizeof(ptp));
    slot(ptp, 1, 1, 3, 796, 998);
    slot(ptp + 5, 1, 1, 5, 398, 499);
    ptp[25] = 2;
    n = apple_mt_frame(&mt, ptp, sizeof(ptp), 110, out, &lifted);
    assert(n == 12 + 18);
    d = decode(out + 12);
    printf("move   id=%d x=%d y=%d down=%d\n", d.id, d.x, d.y, d.down);
    assert(d.down && d.x == 4067 && d.y == 2603);
    d = decode(out + 21);
    printf("second id=%d x=%d y=%d state=0x%02x finger=%d\n", d.id, d.x, d.y, d.state, d.finger);
    assert(d.id == 6 && d.state3 == 0x3 && d.finger == 3 && d.x >= -2 && d.x <= 2);

    /* Both lift: stop state, zero pressure, and the frame says idle. */
    memset(ptp, 0, sizeof(ptp));
    slot(ptp, 0, 1, 3, 796, 998);
    slot(ptp + 5, 0, 1, 5, 398, 499);
    ptp[25] = 2;
    n = apple_mt_frame(&mt, ptp, sizeof(ptp), 120, out, &lifted);
    d = decode(out + 12);
    printf("lift   state=0x%02x pressure=%d touch_active=0x%x lifted=%d\n", d.state, d.pressure,
           out[7], lifted);
    assert(n == 12 + 18 && d.state3 == 0x7 && d.pressure == 0 && out[7] == 0x2 && lifted);

    /* The three trailing frames, the last one header-only. */
    assert(apple_mt_lift_frame(&mt, 0, 130, out) == 12 + 18);
    assert(apple_mt_lift_frame(&mt, 1, 130, out) == 12 + 18);
    assert(((out[12 + 3] >> 2) & 0x3f) == 0);
    assert(apple_mt_lift_frame(&mt, 2, 140, out) == 12);

    /* An empty report (no contacts) sends a header-only idle frame. */
    memset(ptp, 0, sizeof(ptp));
    n = apple_mt_frame(&mt, ptp, sizeof(ptp), 150, out, &lifted);
    assert(n == 12 && lifted);

    /* Palm: confidence 0 is finger type 6. */
    slot(ptp, 1, 0, 1, 100, 100);
    ptp[25] = 1;
    apple_mt_frame(&mt, ptp, sizeof(ptp), 160, out, &lifted);
    assert(decode(out + 12).finger == 6);

    /* Handshake: SET 0x01 selects, GET 0x01 answers; D9 carries the size. */
    uint8_t cmd[] = {0x01, 0xD1};
    uint8_t buf[80];
    apple_mt_set_report(&mt, 0x01, cmd, sizeof(cmd));
    int r = apple_mt_get_report(&mt, 0x01, buf, sizeof(buf));
    assert(r == 5 && buf[0] == 0x01 && buf[1] == 0xD1 && buf[3] == 0x01);
    r = apple_mt_get_report(&mt, 0xD9, buf, sizeof(buf));
    assert(r == 17 && buf[1] == (6640 & 0xff) && buf[2] == (6640 >> 8) && buf[5] == (8320 & 0xff));
    r = apple_mt_get_report(&mt, 0xDB, buf, sizeof(buf));
    printf("DB report %d bytes\n", r);
    assert(r == 8 + 13 + 2 + 16 + 2 + 7 + 2 + 17 + 5);
    assert(apple_mt_get_report(&mt, 0x55, buf, sizeof(buf)) == -1);
    /* Byte-identical to VoodooInputSimulatorDevice's report_descriptor. */
    assert(apple_mt_report_desc_size == 110);

    puts("apple_mt: all checks passed");
    return 0;
}

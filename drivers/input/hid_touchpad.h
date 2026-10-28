/*
 * Copyright (c) 2025 The ZMK Contributors
 * Copyright (c) 2025-2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C HID protocol constants */
#define I2C_HID_SET_REPORT      0x03
#define I2C_HID_GET_REPORT      0x02
#define I2C_HID_SET_POWER       0x08
#define I2C_HID_REPORT_TYPE_INPUT   0x10
#define I2C_HID_REPORT_TYPE_OUTPUT  0x20
#define I2C_HID_REPORT_TYPE_FEATURE 0x30

/* SET_POWER power states (HID over I2C spec 1.00; 2 and 3 are reserved) */
#define I2C_HID_PWR_ON    0x00
#define I2C_HID_PWR_SLEEP 0x01

/* Report descriptor, provided by devicetree (`report-descriptor`) */
extern const uint8_t tp_report_desc[];
extern const size_t tp_report_desc_size;

struct hid_touchpad_config {
    struct i2c_dt_spec i2c_bus;
    const struct gpio_dt_spec dr;
    uint8_t hid_desc_register;
};

typedef void (*hid_touchpad_input_cb_t)(const struct device *dev, uint8_t report_id,
                                        const uint8_t *data, uint16_t len);

/* Returns true when it consumed the report, so no input callback sees it. */
typedef bool (*hid_touchpad_input_filter_t)(const struct device *dev, uint8_t report_id,
                                            const uint8_t *data, uint16_t len);

/* USB + BLE passthroughs, plus one for the application (daisy's activity hook). */
#define HID_TOUCHPAD_MAX_CBS 3

/* One-byte feature reports held across pad power transitions
 * (hid_touchpad_retain_feature); the host-owned PTP reports need three. */
#define HID_TOUCHPAD_MAX_RETAINED 4

struct hid_touchpad_retained {
    uint8_t id;
    uint8_t value;
    /* The pad was written `value` since it was last powered on or reset. */
    bool on_pad;
};

struct hid_touchpad_data {
    const struct device *dev;
    uint8_t command_reg;
    uint8_t data_reg;
    uint16_t max_input_len;
    struct gpio_callback gpio_cb;
    struct k_work work;
    hid_touchpad_input_cb_t input_cbs[HID_TOUCHPAD_MAX_CBS];
    uint8_t num_cbs;
    hid_touchpad_input_filter_t input_filter;
    uint8_t report_buf[64];
    /* Serialises every I2C transaction the driver issues (input reads,
     * GET/SET_REPORT, SET_POWER, raw registers). */
    struct k_mutex lock;
    /* Thread holding an exclusive claim (hid_touchpad_claim), or NULL. */
    k_tid_t claim_owner;
    /* Not `lock`: a value must be recorded even while the pad is claimed,
     * so the claim owner's restore picks it up. Held across the retained
     * writes, so a restore can't put back a value replaced meanwhile. Taken
     * before `lock`, never while holding it. */
    struct k_mutex retained_lock;
    struct hid_touchpad_retained retained[HID_TOUCHPAD_MAX_RETAINED];
    uint8_t num_retained;
    /* The last SET_POWER was on, so the pad takes the retained writes. */
    bool powered;
    /* hid_touchpad_hold_retained(): record retained values, write none. */
    bool retained_held;
    /* Writes the retained values again after the pad reset itself. */
    struct k_work reset_work;
};

/**
 * Register a callback to receive raw input reports from the touchpad.
 * The callback is invoked from the driver's dedicated work queue; it may
 * block briefly (bounded), but every ms spent here delays the next report.
 */
void hid_touchpad_register_input_cb(const struct device *dev, hid_touchpad_input_cb_t cb);

/**
 * Let `filter` see every input report before the input callbacks do, e.g.
 * for firmware that turns the pad's PTP frames into reports of its own and
 * sends those with hid_touchpad_inject_input() (which `filter` sees too).
 * One filter; NULL removes it. Runs where the callbacks would: keep it short.
 */
void hid_touchpad_set_input_filter(const struct device *dev, hid_touchpad_input_filter_t filter);

/**
 * The driver's own work queue, which reads the pad and runs the input
 * filter and callbacks. Work submitted here is serialised with them, and
 * isn't held up by the system workqueue's HID submits.
 */
struct k_work_q *hid_touchpad_work_queue(void);

/**
 * Feed a synthetic input report to every registered callback, exactly as if
 * the pad had produced it: same callbacks, same order, same endpoint
 * selection and BLE pacing downstream. Nothing is sent to the pad over I2C,
 * so it works whether or not a physical pad answers.
 *
 * For end-to-end tests of the passthrough path (see the Daisy factory
 * protocol's TOUCHPAD_INJECT_* commands). `report_id` must be one the
 * report descriptor declares, or the backends drop the frame.
 *
 * Callable from any thread context. The callbacks are the real ones, so the
 * USB backend can block on its TX semaphore for up to its own timeout --
 * don't call this from an ISR or a latency-critical callback.
 */
void hid_touchpad_inject_input(const struct device *dev, uint8_t report_id,
                               const uint8_t *data, uint16_t len);

/**
 * Get a feature/input report from the touchpad via I2C HID protocol.
 * @param type Report type (I2C_HID_REPORT_TYPE_FEATURE, etc.)
 * @param id   Report ID
 * @param buf  Buffer to receive report data (excluding report ID prefix)
 * @param buf_len Size of buf
 * @param out_len Actual data length returned (excluding length prefix and report ID)
 */
int hid_touchpad_get_report(const struct device *dev, uint8_t type, uint8_t id,
                            uint8_t *buf, uint16_t buf_len, uint16_t *out_len);

/**
 * Data size (excluding report ID) of a feature report as declared in
 * devicetree (feature-report-sizes), or -ENOENT for an unknown ID.
 */
int hid_touchpad_feature_size(uint8_t id);

/**
 * Set a feature/output report on the touchpad via I2C HID protocol.
 * @param type Report type (I2C_HID_REPORT_TYPE_FEATURE, etc.)
 * @param id   Report ID
 * @param data Report data (excluding report ID)
 * @param len  Length of data
 */
int hid_touchpad_set_report(const struct device *dev, uint8_t type, uint8_t id,
                            const uint8_t *data, uint16_t len);

/**
 * Hold a one-byte feature report at `value`, so it survives whatever the pad
 * does to its state across power transitions. Replaces the value held for
 * `id` if there is one.
 *
 * Writes the report only when the pad doesn't already hold `value` and is
 * powered on: a pad in SET_POWER sleep is left asleep, and gets every held
 * value written on the next SET_POWER on, after a PixArt firmware update,
 * when the pad signals a reset of its own and on
 * hid_touchpad_restore_retained(). The value is recorded even when the
 * write fails (e.g. -EBUSY while the pad is claimed), and the write is
 * retried on the next call or restore.
 *
 * @return 0 if the pad holds `value` or will get it on power-on, the write's
 *         error, or -ENOMEM when HID_TOUCHPAD_MAX_RETAINED reports are
 *         already held.
 */
int hid_touchpad_retain_feature(const struct device *dev, uint8_t id, uint8_t value);

/**
 * Write every report held by hid_touchpad_retain_feature() to a pad that was
 * just reset, so is powered on and has lost them.
 * hid_touchpad_set_power(I2C_HID_PWR_ON), pixart_tp_update() and the pad's
 * own reset signal already do this; call it after anything else that resets
 * the pad. Writes nothing
 * while the retained values are held back.
 * @return 0, or the first failed write's error (the rest are still tried).
 */
int hid_touchpad_restore_retained(const struct device *dev);

/**
 * Stop writing retained feature reports to the pad (`hold` true), e.g. while
 * a test tool sets the pad's modes itself: hid_touchpad_retain_feature()
 * only records, and power-on and restore write nothing. Releasing writes
 * every held value, since the pad's own may have been changed meanwhile.
 * @return 0, or on release the first failed write's error.
 */
int hid_touchpad_hold_retained(const struct device *dev, bool hold);

/**
 * Issue an I2C HID SET_POWER command (I2C_HID_PWR_ON / I2C_HID_PWR_SLEEP).
 * The device sends no response; per spec it must transition within 1 s.
 * After a successful I2C_HID_PWR_ON the retained feature reports are
 * restored (hid_touchpad_restore_retained); after I2C_HID_PWR_SLEEP they are
 * only recorded until then.
 */
int hid_touchpad_set_power(const struct device *dev, uint8_t state);

/**
 * Read/write a raw (non-HID) 8-bit device register. For vendor-specific
 * control flows the HID protocol doesn't cover, e.g. the PCT1036
 * suspend/resume register sequence.
 */
int hid_touchpad_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
int hid_touchpad_reg_write(const struct device *dev, uint8_t reg, uint8_t val);

/**
 * Take exclusive ownership of the pad for a multi-transaction sequence
 * (e.g. a firmware update, where a 256-byte burst write must not be
 * interleaved with anything else).
 *
 * While claimed:
 *  - every driver call from a thread other than the owner fails with -EBUSY
 *    immediately -- a host blocked on a USB control transfer must never
 *    stall behind the owner's work;
 *  - the DR interrupt is disabled and the input work item is cancelled, so
 *    the driver's own input-report reads stop too. Input resumes on release;
 *    a report that went ready meanwhile is drained then (DR is
 *    edge-triggered and would otherwise never fire again).
 *
 * @param timeout How long to wait for an in-flight transaction to finish.
 * @return 0, -EALREADY if already claimed, or the k_mutex_lock error
 *         (-EBUSY for K_NO_WAIT, -EAGAIN on timeout).
 *
 * Must not be called from the driver's own work queue (it waits for that
 * queue's work item to finish).
 */
int hid_touchpad_claim(const struct device *dev, k_timeout_t timeout);

/**
 * Give the pad back. Re-arms the DR interrupt and drains a pending report.
 * @return 0, or -EPERM when the caller does not hold the claim.
 */
int hid_touchpad_release(const struct device *dev);

/**
 * Re-read the I2C HID descriptor and refresh command_reg / data_reg /
 * max_input_len from it. Needed after anything that reboots the pad's HID
 * stack (a vendor reset into bootloader or application mode): the
 * registers are only guaranteed for the descriptor they came from. Retries
 * for up to ~1 s while the pad comes back. Subject to the claim like every
 * other call.
 */
int hid_touchpad_reinit_descriptor(const struct device *dev);

#ifdef __cplusplus
}
#endif

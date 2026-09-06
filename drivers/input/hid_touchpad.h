/*
 * Copyright (c) 2025 The ZMK Contributors
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

/* Hardcoded report descriptor (686 bytes, from PCT1036 touchpad) */
extern const uint8_t tp_report_desc[];
extern const size_t tp_report_desc_size;

struct hid_touchpad_config {
    struct i2c_dt_spec i2c_bus;
    const struct gpio_dt_spec dr;
    uint8_t hid_desc_register;
};

typedef void (*hid_touchpad_input_cb_t)(const struct device *dev, uint8_t report_id,
                                        const uint8_t *data, uint16_t len);

#define HID_TOUCHPAD_MAX_CBS 2

struct hid_touchpad_data {
    const struct device *dev;
    uint8_t command_reg;
    uint8_t data_reg;
    uint16_t max_input_len;
    uint16_t input_reg;
    struct gpio_callback gpio_cb;
    struct k_work work;
    hid_touchpad_input_cb_t input_cbs[HID_TOUCHPAD_MAX_CBS];
    uint8_t num_cbs;
    uint8_t report_buf[64];
};

/**
 * Register a callback to receive raw input reports from the touchpad.
 * The callback is invoked from the driver's dedicated work queue; it may
 * block briefly (bounded), but every ms spent here delays the next report.
 */
void hid_touchpad_register_input_cb(const struct device *dev, hid_touchpad_input_cb_t cb);

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
 * Issue an I2C HID SET_POWER command (I2C_HID_PWR_ON / I2C_HID_PWR_SLEEP).
 * The device sends no response; per spec it must transition within 1 s.
 */
int hid_touchpad_set_power(const struct device *dev, uint8_t state);

/**
 * Read/write a raw (non-HID) 8-bit device register. For vendor-specific
 * control flows the HID protocol doesn't cover, e.g. the PCT1036
 * suspend/resume register sequence.
 */
int hid_touchpad_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
int hid_touchpad_reg_write(const struct device *dev, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

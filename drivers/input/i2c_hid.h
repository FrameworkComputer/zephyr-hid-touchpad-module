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
#define I2C_HID_REPORT_TYPE_INPUT   0x10
#define I2C_HID_REPORT_TYPE_OUTPUT  0x20
#define I2C_HID_REPORT_TYPE_FEATURE 0x30

/* HID report descriptor from devicetree */
extern const uint8_t i2c_hid_report_desc[];
extern const size_t i2c_hid_report_desc_size;

struct i2c_hid_config {
    struct i2c_dt_spec i2c_bus;
    const struct gpio_dt_spec dr;
    uint8_t hid_desc_register;
};

typedef void (*i2c_hid_input_cb_t)(const struct device *dev, uint8_t report_id,
                                    const uint8_t *data, uint16_t len);

#define I2C_HID_MAX_CBS 2

struct i2c_hid_data {
    const struct device *dev;
    uint8_t command_reg;
    uint8_t data_reg;
    uint16_t max_input_len;
    uint16_t input_reg;
    struct gpio_callback gpio_cb;
    struct k_work work;
    i2c_hid_input_cb_t input_cbs[I2C_HID_MAX_CBS];
    uint8_t num_cbs;
    uint8_t report_buf[64];
};

/**
 * Register a callback to receive raw input reports from the I2C HID device.
 * The callback is invoked from the system work queue.
 */
void i2c_hid_register_input_cb(const struct device *dev, i2c_hid_input_cb_t cb);

/**
 * Get a feature/input report from the I2C HID device.
 * @param type Report type (I2C_HID_REPORT_TYPE_FEATURE, etc.)
 * @param id   Report ID
 * @param buf  Buffer to receive report data (excluding report ID prefix)
 * @param buf_len Size of buf
 * @param out_len Actual data length returned (excluding length prefix and report ID)
 */
int i2c_hid_get_report(const struct device *dev, uint8_t type, uint8_t id,
                        uint8_t *buf, uint16_t buf_len, uint16_t *out_len);

/**
 * Set a feature/output report on the I2C HID device.
 * @param type Report type (I2C_HID_REPORT_TYPE_FEATURE, etc.)
 * @param id   Report ID
 * @param data Report data (excluding report ID)
 * @param len  Length of data
 */
int i2c_hid_set_report(const struct device *dev, uint8_t type, uint8_t id,
                        const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

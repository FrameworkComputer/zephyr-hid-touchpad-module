/*
 * Copyright (c) 2022 The ZMK Contributors
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

#define SET_REPORT          0b11
#define FEATURE_REPORT      0b110000

struct hid_touchpad_config {
    struct i2c_dt_spec i2c_bus;
    const struct gpio_dt_spec dr;
    uint8_t mouse_report_id;
    uint8_t inputmode_report_id;
    uint8_t ptp_report_id;
    uint8_t enable_ptp_mode;
    uint8_t relative_x_off;
    uint8_t relative_x_len;
    uint8_t relative_y_off;
    uint8_t relative_y_len;
    uint8_t button_off;
    uint8_t button_bit;
};

struct hid_touchpad_data {
    uint8_t command_reg;
    uint8_t data_reg;
    uint8_t btn_cache;
    bool in_int;
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
};

#ifdef __cplusplus
}
#endif

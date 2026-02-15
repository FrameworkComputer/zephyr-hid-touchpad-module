/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_hid_touchpad

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include "hid_touchpad.h"

#define LOG_LEVEL CONFIG_INPUT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_touchpad);

#define PTP_REPORT_SIZE 32
#define PTP_MAX_FINGERS 5
#define PTP_CONTACT_SIZE 5

// #define USE_FIFO 1

static int enable_ptp(const struct device *dev, uint8_t enable) {

    struct hid_touchpad_data *const data = (struct hid_touchpad_data *const)dev->data;
    struct hid_touchpad_config *config = (struct hid_touchpad_config *)dev->config;

    uint8_t mode = 0x00;
    if (enable)
        mode = 0x03;

    uint8_t cmd_data[10] = {
        data->command_reg, 0x00,
        FEATURE_REPORT | config->inputmode_report_id, SET_REPORT,
        data->data_reg, 0x00,
        0x04, 0x00, config->inputmode_report_id, mode
    };

    int err = i2c_write_dt(&config->i2c_bus, &cmd_data[0], sizeof(cmd_data));

    LOG_DBG("set mousemode to %b", enable);

    return err;
}

static void mouse_mode_report_data(const struct device *dev) {
    const struct hid_touchpad_config *config = dev->config;

    uint8_t buf[8] = {0};
    int err = i2c_read_dt(&config->i2c_bus, &buf[0], sizeof(buf));
    if (err != 0) {
        LOG_ERR("failed to read mousemode report: %d", err);
        return;
    }
    if (err == -EIO || err == -EBUSY) {
        LOG_ERR("I2C error, attempting recovery");
        i2c_recover_bus(config->i2c_bus.bus);

        err = i2c_read_dt(&config->i2c_bus, &buf[0], sizeof(buf));
        if (err != 0) {
            LOG_ERR("failed to read mousemode (again after bus recovery) report: %d", err);
            return;
        }
    }

    uint16_t report_len = sys_get_le16(&buf[0]);
    LOG_DBG("Report Len: %04X ReportId: %d", report_len, buf[2]);
    LOG_HEXDUMP_DBG(buf, sizeof(buf), "Raw Touchpad Data");
    if (buf[2] != config->mouse_report_id) {
        LOG_ERR("Unexpected Report ID: %d", buf[2]);
        return;
    }

    bool btn_val = IS_BIT_SET(buf[config->button_off], BIT(config->button_bit));

    int dx = 0;
    int dy = 0;
    if (config->relative_x_len == 1) {
      dx = (int8_t)buf[config->relative_x_off];
    } else {
      dx = (int16_t)sys_get_le16(&buf[config->relative_x_off]);
    }
    if (config->relative_x_len == 1) {
      dy = (int8_t)buf[config->relative_y_off];
    } else {
      dy = (int16_t)sys_get_le16(&buf[config->relative_y_off]);
    }
    input_report_key(dev, INPUT_BTN_0, btn_val, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
}

static void ptp_mode_report_data(const struct device *dev) {
    const struct hid_touchpad_config *config = dev->config;

    uint8_t buf[PTP_REPORT_SIZE] = {0};
    int err = i2c_read_dt(&config->i2c_bus, &buf[0], sizeof(buf));
    if (err != 0) {
        LOG_ERR("failed to read PTP report: %d", err);
        return;
    }
    if (err == -EIO || err == -EBUSY) {
        LOG_ERR("I2C error, attempting recovery");
        i2c_recover_bus(config->i2c_bus.bus);

        err = i2c_read_dt(&config->i2c_bus, &buf[0], sizeof(buf));
        if (err != 0) {
            LOG_ERR("failed to read PTP (again after bus recovery) report: %d", err);
            return;
        }
    }

    uint16_t report_len = sys_get_le16(&buf[0]);
    LOG_DBG("Report Len: %04X ReportId: %d", report_len, buf[2]);
    LOG_HEXDUMP_DBG(buf, sizeof(buf), "Raw Touchpad PTP Report");
    if (buf[2] != config->ptp_report_id) {
        LOG_ERR("Unexpected Report ID: %d", buf[2]);
        return;
    }

    uint8_t contact_count = buf[28];
    bool button_pressed = IS_BIT_SET(buf[29], BIT(0));
    // Units of 100us
    int16_t scan_time = sys_get_le16(&buf[30]);

    int contacts_active = 0;
    bool last_touch_status = false;
    for (int contact = 0; contact < contact_count; contact++) {
        size_t offset = 3 + (contact * PTP_CONTACT_SIZE);

        if (offset + PTP_CONTACT_SIZE > sizeof(buf)) {
            LOG_ERR("Report data out of bounds for contact %d", contact);
            break;
        }
        uint8_t status_byte = buf[offset];
        uint8_t finger_idx = status_byte >> 4;
        bool confidence = IS_BIT_SET(status_byte, BIT(0));
        bool tip_switch = IS_BIT_SET(status_byte, BIT(1));
	last_touch_status = confidence && tip_switch;
        if (confidence && tip_switch) {
            contacts_active += 1;
        }
        uint16_t x_pos = sys_get_le16(&buf[offset + 1]);
        uint16_t y_pos = sys_get_le16(&buf[offset + 3]);

#if !USE_FIFO
        input_report_abs(dev, INPUT_ABS_MT_SLOT, finger_idx, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_X, x_pos, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_Y, y_pos, false, K_FOREVER);
        input_report_key(dev, INPUT_BTN_TOUCH, confidence && tip_switch, false, K_FOREVER);
#endif
    }

    printk("Button: %d, Contacts: (%d/%d), Scan Time: %4d\n",
        button_pressed, contact_count, contacts_active, scan_time);

#if USE_FIFO
    k_fifo_put(&ptp_touchpad_fifo, &buf);
#else
    // Sync finger events - have the HID driver send them to the host as a single report
    if (contact_count > 0)
        input_report_key(dev, INPUT_BTN_TOUCH, last_touch_status, true, K_FOREVER);
#endif
}

static void hid_touchpad_report_data(const struct device *dev) {
    const struct hid_touchpad_config *config = dev->config;
    if (config->enable_ptp_mode)
        ptp_mode_report_data(dev);
    else
        mouse_mode_report_data(dev);
}

static int set_int(const struct device *dev, const bool en) {
    const struct hid_touchpad_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static void hid_touchpad_work_cb(struct k_work *work) {
    struct hid_touchpad_data *data = CONTAINER_OF(work, struct hid_touchpad_data, work);
    hid_touchpad_report_data(data->dev);
}

static void hid_touchpad_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct hid_touchpad_data *data = CONTAINER_OF(cb, struct hid_touchpad_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    data->in_int = true;
    k_work_submit(&data->work);
}

static int hid_touchpad_init(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_WRN("i2c bus not ready!");
        return -EINVAL;
    }

    /* Check if the i2c bus requires recovery. Can happen if something went
     * wrong with power sequencing */
    int err = i2c_recover_bus(config->i2c_bus.bus);
    if (err) {
        /* Usually not fatal, but good to log */
        LOG_WRN("I2C bus recovery failed or not supported: %d", err);
    }

    // TODO: Get version
    // uint16_t ic_version = 0;
    // int err = read_register(dev, REG_VERSION, &ic_version);
    // if (err != 0) {
    //     LOG_WRN("could not get IC version!");
    //     return err;
    // }


    uint8_t hid_desc[26] = {0};
    err = i2c_burst_read_dt(&config->i2c_bus, 0x20, &hid_desc[0], sizeof(hid_desc));
    if (err) {
      LOG_ERR("Failed to read hid descriptor with err: %d", err);
      return -ENODEV;
    } else {
      LOG_INF("descLen       %02X%02X", hid_desc[1], hid_desc[0]);
      LOG_INF("bcdVer        %02X%02X", hid_desc[3], hid_desc[2]);
      LOG_INF("reportDescLen %02X%02X", hid_desc[5], hid_desc[4]);
      LOG_INF("reportDescReg %02X%02X", hid_desc[7], hid_desc[6]);
      LOG_INF("wInputReg     %02X%02X", hid_desc[9], hid_desc[8]);
      LOG_INF("wCommandReg   %02X%02X", hid_desc[17], hid_desc[16]);
      LOG_INF("wDataReg      %02X%02X", hid_desc[19], hid_desc[18]);
      LOG_INF("PID           %02X%02X", hid_desc[23], hid_desc[22]);
      LOG_INF("VID           %02X%02X", hid_desc[21], hid_desc[20]);
      LOG_INF("PID           %02X%02X", hid_desc[23], hid_desc[22]);
      LOG_INF("Version       %02X%02X", hid_desc[25], hid_desc[24]);
      data->command_reg = hid_desc[16];
      data->data_reg = hid_desc[18];
    }

    // Switch touchpad to the configure mode
    enable_ptp(dev, config->enable_ptp_mode);

    data->dev = dev;
    data->in_int = false;

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, hid_touchpad_gpio_cb, BIT(config->dr.pin));
    int ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    set_int(dev, true);

    k_work_init(&data->work, hid_touchpad_work_cb);

    LOG_INF("device initialized at 0x%x", config->i2c_bus.addr);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int hid_touchpad_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define HID_TOUCHPAD_INIT(n)                                                                          \
    static struct hid_touchpad_data hid_touchpad_data_##n;                                                 \
    static const struct hid_touchpad_config hid_touchpad_config_##n = {                                    \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                      \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                 \
        .mouse_report_id = DT_INST_PROP(n, mouse_report_id),                                     \
        .inputmode_report_id = DT_INST_PROP(n, inputmode_report_id),                             \
        .ptp_report_id = DT_INST_PROP(n, ptp_report_id),                                         \
        .enable_ptp_mode = DT_INST_ENUM_IDX(n, reporting_mode),                                  \
        .relative_x_off = DT_INST_PROP(n, relative_x_off),                                       \
        .relative_x_len = DT_INST_PROP(n, relative_x_len),                                       \
        .relative_y_off = DT_INST_PROP(n, relative_y_off),                                       \
        .relative_y_len = DT_INST_PROP(n, relative_y_len),                                       \
        .button_off = DT_INST_PROP(n, button_off),                                               \
        .button_bit = DT_INST_PROP(n, button_bit),                                               \
    };                                                                                           \
    PM_DEVICE_DT_INST_DEFINE(n, hid_touchpad_pm_action);                                              \
    DEVICE_DT_INST_DEFINE(n, hid_touchpad_init, PM_DEVICE_DT_INST_GET(n), &hid_touchpad_data_##n,          \
                          &hid_touchpad_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,          \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(HID_TOUCHPAD_INIT)

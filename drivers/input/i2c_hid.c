/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_i2c_hid

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include "i2c_hid.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_hid, CONFIG_I2C_HID_LOG_LEVEL);

/* HID report descriptor from devicetree */
const uint8_t i2c_hid_report_desc[] = DT_INST_PROP(0, report_descriptor);
const size_t i2c_hid_report_desc_size = DT_INST_PROP_LEN(0, report_descriptor);

int i2c_hid_get_report(const struct device *dev, uint8_t type, uint8_t id,
                        uint8_t *buf, uint16_t buf_len, uint16_t *out_len) {
    struct i2c_hid_data *data = dev->data;
    const struct i2c_hid_config *config = dev->config;

    uint8_t cmd[6];
    if (id <= 0x0F) {
        cmd[0] = data->command_reg;
        cmd[1] = 0x00;
        cmd[2] = type | id;
        cmd[3] = I2C_HID_GET_REPORT;
        cmd[4] = data->data_reg;
        cmd[5] = 0x00;
    } else {
        /* For report IDs > 15, use the extended format */
        uint8_t cmd_ext[7] = {
            data->command_reg, 0x00,
            type | 0x0F, I2C_HID_GET_REPORT,
            id,
            data->data_reg, 0x00,
        };
        int err = i2c_write_read_dt(&config->i2c_bus, cmd_ext, sizeof(cmd_ext),
                                     buf, buf_len);
        if (err) {
            LOG_ERR("get_report (ext) failed: %d", err);
            return err;
        }
        LOG_HEXDUMP_DBG(cmd_ext, sizeof(cmd_ext), "get_report ext cmd");
        LOG_HEXDUMP_DBG(buf, MIN(16, buf_len), "get_report ext raw resp");
        uint16_t len = sys_get_le16(buf);
        if (len < 3 || len > buf_len) {
            LOG_ERR("get_report (ext) invalid length: %d", len);
            return -EINVAL;
        }
        /* buf[0..1] = length, buf[2] = report ID, buf[3..] = data */
        *out_len = len - 3;
        /* Shift data to start of buffer: data starts at buf[3] */
        memmove(buf, &buf[3], *out_len);
        return 0;
    }

    int err = i2c_write_read_dt(&config->i2c_bus, cmd, sizeof(cmd), buf, buf_len);
    if (err) {
        LOG_ERR("get_report failed: %d", err);
        return err;
    }

    uint16_t len = sys_get_le16(buf);
    if (len < 3 || len > buf_len) {
        LOG_ERR("get_report invalid length: %d", len);
        return -EINVAL;
    }
    /* buf[0..1] = length, buf[2] = report ID, buf[3..] = data */
    *out_len = len - 3;
    memmove(buf, &buf[3], *out_len);
    return 0;
}

int i2c_hid_set_report(const struct device *dev, uint8_t type, uint8_t id,
                        const uint8_t *report_data, uint16_t len) {
    struct i2c_hid_data *data = dev->data;
    const struct i2c_hid_config *config = dev->config;

    /* Wire format: [data_reg, 0x00, len_lo, len_hi, report_id, data...] */

    if (id <= 0x0F) {
        /* cmd: [cmd_reg, 0x00, (type|id), SET_REPORT] + [data_reg, 0x00, len_lo, len_hi, id, data...] */
        uint8_t msg[4 + 4 + 1 + 256]; /* max feature report is 256 bytes */
        if (9 + len > sizeof(msg)) {
            return -ENOMEM;
        }
        msg[0] = data->command_reg;
        msg[1] = 0x00;
        msg[2] = type | id;
        msg[3] = I2C_HID_SET_REPORT;
        msg[4] = data->data_reg;
        msg[5] = 0x00;
        /* Length includes the 2 length bytes + report_id + data */
        uint16_t payload_len = 2 + 1 + len;
        sys_put_le16(payload_len, &msg[6]);
        msg[8] = id;
        if (len > 0) {
            memcpy(&msg[9], report_data, len);
        }
        int err = i2c_write_dt(&config->i2c_bus, msg, 9 + len);
        if (err) {
            LOG_ERR("set_report failed: %d", err);
        }
        return err;
    } else {
        /* Extended format for report IDs > 15 */
        uint8_t msg[5 + 4 + 1 + 256];
        if (10 + len > sizeof(msg)) {
            return -ENOMEM;
        }
        msg[0] = data->command_reg;
        msg[1] = 0x00;
        msg[2] = type | 0x0F;
        msg[3] = I2C_HID_SET_REPORT;
        msg[4] = id;
        msg[5] = data->data_reg;
        msg[6] = 0x00;
        uint16_t payload_len = 2 + 1 + len;
        sys_put_le16(payload_len, &msg[7]);
        msg[9] = id;
        if (len > 0) {
            memcpy(&msg[10], report_data, len);
        }
        LOG_HEXDUMP_DBG(msg, 10 + len, "set_report ext I2C write");
        int err = i2c_write_dt(&config->i2c_bus, msg, 10 + len);
        if (err) {
            LOG_ERR("set_report (ext) failed: %d", err);
        }
        return err;
    }
}

void i2c_hid_register_input_cb(const struct device *dev, i2c_hid_input_cb_t cb) {
    struct i2c_hid_data *data = dev->data;
    if (data->num_cbs < I2C_HID_MAX_CBS) {
        data->input_cbs[data->num_cbs++] = cb;
    } else {
        LOG_ERR("Too many input callbacks registered");
    }
}

static void i2c_hid_report_data(const struct device *dev) {
    struct i2c_hid_data *data = dev->data;
    const struct i2c_hid_config *config = dev->config;

    int err = i2c_read_dt(&config->i2c_bus, data->report_buf, data->max_input_len);
    if (err) {
        LOG_ERR("failed to read input report: %d", err);
        return;
    }

    uint16_t report_len = sys_get_le16(data->report_buf);
    if (report_len == 0 || report_len == 0xFFFF) {
        /* Reset signal or no data */
        return;
    }
    if (report_len < 3) {
        LOG_WRN("report too short: %d", report_len);
        return;
    }
    if (report_len > data->max_input_len) {
        LOG_WRN("report length %d exceeds max %d", report_len, data->max_input_len);
        report_len = data->max_input_len;
    }

    uint8_t report_id = data->report_buf[2];
    /* Data starts at byte 3, length is report_len - 2 (minus the 2-byte length prefix) - 1 (report ID) */
    uint16_t data_len = report_len - 3;

    LOG_DBG("Report ID: %d, Len: %d", report_id, data_len);
    LOG_HEXDUMP_DBG(&data->report_buf[2], report_len - 2, "Raw report");

    for (int i = 0; i < data->num_cbs; i++) {
        if (data->input_cbs[i]) {
            data->input_cbs[i](dev, report_id, &data->report_buf[3], data_len);
        }
    }
}

static int set_int(const struct device *dev, const bool en) {
    const struct i2c_hid_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static void i2c_hid_work_cb(struct k_work *work) {
    struct i2c_hid_data *data = CONTAINER_OF(work, struct i2c_hid_data, work);
    i2c_hid_report_data(data->dev);
}

static void i2c_hid_gpio_cb(const struct device *port, struct gpio_callback *cb,
                              uint32_t pins) {
    struct i2c_hid_data *data = CONTAINER_OF(cb, struct i2c_hid_data, gpio_cb);
    k_work_submit(&data->work);
}

static int i2c_hid_init(const struct device *dev) {
    struct i2c_hid_data *data = dev->data;
    const struct i2c_hid_config *config = dev->config;

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_WRN("i2c bus not ready!");
        return -EINVAL;
    }

    int err = i2c_recover_bus(config->i2c_bus.bus);
    if (err) {
        LOG_WRN("I2C bus recovery failed or not supported: %d", err);
    }

    /* Read 30-byte I2C HID descriptor from configured register */
    uint8_t hid_desc[30] = {0};
    err = i2c_burst_read_dt(&config->i2c_bus, config->hid_desc_register, hid_desc, sizeof(hid_desc));
    if (err) {
        LOG_ERR("Failed to read HID descriptor: %d", err);
        return -ENODEV;
    }

    LOG_INF("descLen       %02X%02X", hid_desc[1], hid_desc[0]);
    LOG_INF("bcdVer        %02X%02X", hid_desc[3], hid_desc[2]);
    LOG_INF("reportDescLen %02X%02X", hid_desc[5], hid_desc[4]);
    LOG_INF("reportDescReg %02X%02X", hid_desc[7], hid_desc[6]);
    LOG_INF("wInputReg     %02X%02X", hid_desc[9], hid_desc[8]);
    LOG_INF("wCommandReg   %02X%02X", hid_desc[17], hid_desc[16]);
    LOG_INF("wDataReg      %02X%02X", hid_desc[19], hid_desc[18]);
    LOG_INF("VID           %02X%02X", hid_desc[21], hid_desc[20]);
    LOG_INF("PID           %02X%02X", hid_desc[23], hid_desc[22]);
    LOG_INF("Version       %02X%02X", hid_desc[25], hid_desc[24]);

    data->command_reg = hid_desc[16];
    data->data_reg = hid_desc[18];
    data->input_reg = sys_get_le16(&hid_desc[8]);
    data->max_input_len = sys_get_le16(&hid_desc[10]);

    /* Clamp max_input_len to our buffer size */
    if (data->max_input_len > sizeof(data->report_buf)) {
        LOG_WRN("max_input_len %d exceeds buffer, clamping to %d",
                data->max_input_len, (int)sizeof(data->report_buf));
        data->max_input_len = sizeof(data->report_buf);
    }

    LOG_INF("command_reg=0x%02x data_reg=0x%02x input_reg=0x%04x max_input_len=%d",
            data->command_reg, data->data_reg, data->input_reg, data->max_input_len);

    data->dev = dev;

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, i2c_hid_gpio_cb, BIT(config->dr.pin));
    int ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    set_int(dev, true);

    k_work_init(&data->work, i2c_hid_work_cb);

    LOG_INF("I2C HID device passthrough initialized at 0x%x", config->i2c_bus.addr);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int i2c_hid_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif /* IS_ENABLED(CONFIG_PM_DEVICE) */

#define I2C_HID_INIT(n)                                                                           \
    static struct i2c_hid_data i2c_hid_data_##n;                                                  \
    static const struct i2c_hid_config i2c_hid_config_##n = {                                     \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                       \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                  \
        .hid_desc_register = DT_INST_PROP(n, hid_descriptor_register),                             \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, i2c_hid_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, i2c_hid_init, PM_DEVICE_DT_INST_GET(n),                              \
                          &i2c_hid_data_##n, &i2c_hid_config_##n,                                  \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(I2C_HID_INIT)

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
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include "hid_touchpad.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_touchpad, CONFIG_HID_TOUCHPAD_LOG_LEVEL);

/* HID report descriptor from devicetree */
const uint8_t tp_report_desc[] = DT_INST_PROP(0, report_descriptor);
const size_t tp_report_desc_size = DT_INST_PROP_LEN(0, report_descriptor);

/* Dedicated work queue for reading input reports. The system workqueue is
 * NOT safe here: the ZMK HID submit paths run on it and can block it for up
 * to their TX-semaphore timeout (100 ms in usb_hid.c), which starves the
 * I2C reads — measured as ~117 ms holes in an otherwise 8 ms report stream
 * whenever a key is pressed. Shared by all instances (in practice one). */
static K_THREAD_STACK_DEFINE(tp_workq_stack, CONFIG_HID_TOUCHPAD_WORKQUEUE_STACK_SIZE);
static struct k_work_q tp_workq;

/* Upper bound on reports drained per work invocation, so a babbling device
 * can't monopolize the queue; if DR is still asserted afterwards the work
 * is resubmitted instead. */
#define TP_DRAIN_BURST 8

int hid_touchpad_get_report(const struct device *dev, uint8_t type, uint8_t id,
                            uint8_t *buf, uint16_t buf_len, uint16_t *out_len) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    /* I2C HID frames responses as [len_lo, len_hi, report_id, data...].
     * The caller's buf is sized for the unframed payload only (e.g. USB
     * passes wLength from the host), so do the I2C transfer into a local
     * buffer and copy only the data portion out. Max feature report in our
     * descriptor is 256 bytes; add 3 bytes of framing plus slack. */
    uint8_t i2c_buf[264];

    uint8_t cmd[7];
    size_t cmd_len;
    if (id <= 0x0F) {
        cmd[0] = data->command_reg;
        cmd[1] = 0x00;
        cmd[2] = type | id;
        cmd[3] = I2C_HID_GET_REPORT;
        cmd[4] = data->data_reg;
        cmd[5] = 0x00;
        cmd_len = 6;
    } else {
        cmd[0] = data->command_reg;
        cmd[1] = 0x00;
        cmd[2] = type | 0x0F;
        cmd[3] = I2C_HID_GET_REPORT;
        cmd[4] = id;
        cmd[5] = data->data_reg;
        cmd[6] = 0x00;
        cmd_len = 7;
    }

    int err = i2c_write_read_dt(&config->i2c_bus, cmd, cmd_len,
                                 i2c_buf, sizeof(i2c_buf));
    if (err) {
        LOG_ERR("get_report id=%u failed: %d", id, err);
        return err;
    }

    uint16_t frame_len = sys_get_le16(i2c_buf);
    if (frame_len < 3 || frame_len > sizeof(i2c_buf)) {
        LOG_ERR("get_report id=%u invalid frame length: %u", id, frame_len);
        return -EINVAL;
    }

    /* i2c_buf[0..1] = length, i2c_buf[2] = report ID, i2c_buf[3..] = data */
    uint16_t data_len = frame_len - 3;
    uint16_t copy_len = MIN(data_len, buf_len);
    memcpy(buf, &i2c_buf[3], copy_len);
    *out_len = copy_len;
    return 0;
}

int hid_touchpad_set_report(const struct device *dev, uint8_t type, uint8_t id,
                            const uint8_t *report_data, uint16_t len) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

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

int hid_touchpad_set_power(const struct device *dev, uint8_t state) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    /* Command low byte carries the power state in bits [1:0], high byte the
     * SET_POWER opcode in bits [3:0]. No response follows. */
    uint8_t msg[4] = {data->command_reg, 0x00, state, I2C_HID_SET_POWER};
    int err = i2c_write_dt(&config->i2c_bus, msg, sizeof(msg));
    if (err) {
        LOG_ERR("set_power %u failed: %d", state, err);
    }
    return err;
}

int hid_touchpad_reg_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct hid_touchpad_config *config = dev->config;
    return i2c_reg_read_byte_dt(&config->i2c_bus, reg, val);
}

int hid_touchpad_reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct hid_touchpad_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->i2c_bus, reg, val);
}

void hid_touchpad_register_input_cb(const struct device *dev, hid_touchpad_input_cb_t cb) {
    struct hid_touchpad_data *data = dev->data;
    if (data->num_cbs < HID_TOUCHPAD_MAX_CBS) {
        data->input_cbs[data->num_cbs++] = cb;
    } else {
        LOG_ERR("Too many input callbacks registered");
    }
}

static int hid_touchpad_report_data(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    int err = i2c_read_dt(&config->i2c_bus, data->report_buf, data->max_input_len);
    if (err) {
        LOG_ERR("failed to read input report: %d", err);
        return err;
    }

    uint16_t report_len = sys_get_le16(data->report_buf);
    if (report_len == 0 || report_len == 0xFFFF) {
        /* Reset signal or no data */
        return 0;
    }
    if (report_len < 3) {
        LOG_WRN("report too short: %d", report_len);
        return 0;
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

    /* DEBUG (TP-over-BLE choppiness): 1 Hz production-rate counter, to split
     * "TP frames read too slowly" from "frames pile up on the way out".
     * Remove when the investigation is done. */
    {
        static uint32_t frame_count;
        static int64_t last_report_ms;
        int64_t now = k_uptime_get();

        frame_count++;
        if (now - last_report_ms >= 1000) {
            LOG_INF("tp input: %u frames in %lld ms", frame_count, now - last_report_ms);
            frame_count = 0;
            last_report_ms = now;
        }
    }

    for (int i = 0; i < data->num_cbs; i++) {
        if (data->input_cbs[i]) {
            data->input_cbs[i](dev, report_id, &data->report_buf[3], data_len);
        }
    }
    return 0;
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
    const struct hid_touchpad_config *config = data->dev->config;

    /* Drain until DR de-asserts. The DR interrupt is edge-triggered
     * (EDGE_TO_ACTIVE): if a report becomes ready while we're busy (or while
     * the queue was stalled), the line just STAYS asserted and no new edge
     * ever fires — reading one report per edge then wedges the pad forever.
     * Re-checking the level after each read closes that window: when we see
     * DR low and exit, the next report produces a fresh edge. */
    for (int i = 0; i < TP_DRAIN_BURST; i++) {
        if (hid_touchpad_report_data(data->dev) != 0) {
            /* I2C failure: don't spin on a broken bus; wait for the next
             * edge (or give the device time to recover). */
            return;
        }
        if (gpio_pin_get_dt(&config->dr) <= 0) {
            return;
        }
    }
    /* Still asserted after a full burst — yield the queue and continue. */
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static void hid_touchpad_gpio_cb(const struct device *port, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct hid_touchpad_data *data = CONTAINER_OF(cb, struct hid_touchpad_data, gpio_cb);
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static int hid_touchpad_init(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_WRN("i2c bus not ready!");
        return -EINVAL;
    }

    int err = i2c_recover_bus(config->i2c_bus.bus);
    if (err) {
        LOG_WRN("I2C bus recovery failed or not supported: %d", err);
    }

    /* Read 30-byte I2C HID descriptor from configured register. Retry with
     * delay because an external enable rail may still be ramping or the
     * panel firmware may still be booting during POST_KERNEL — external
     * startup-delay can't be relied on across init-priority boundaries. */
    uint8_t hid_desc[30] = {0};
    err = -EIO;
    for (int attempt = 0; attempt < 10 && err != 0; attempt++) {
        if (attempt > 0) {
            k_msleep(100);
        }
        err = i2c_burst_read_dt(&config->i2c_bus, config->hid_desc_register,
                                hid_desc, sizeof(hid_desc));
    }
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

    /* Do NOT disable PTP — let the OS control input mode via feature reports */

    /* The pad keeps its power state across host reboots (it sits on its own
     * rail), so if it was left in SET_POWER sleep it would stay silent
     * forever. Send SET_POWER on unconditionally — a no-op when already
     * running. */
    uint8_t pwr_cmd[4] = {data->command_reg, 0x00, I2C_HID_PWR_ON, I2C_HID_SET_POWER};
    err = i2c_write_dt(&config->i2c_bus, pwr_cmd, sizeof(pwr_cmd));
    if (err) {
        LOG_WRN("SET_POWER on at init failed: %d", err);
    }

    data->dev = dev;

    static bool workq_started;
    if (!workq_started) {
        k_work_queue_start(&tp_workq, tp_workq_stack,
                           K_THREAD_STACK_SIZEOF(tp_workq_stack),
                           K_PRIO_COOP(CONFIG_HID_TOUCHPAD_WORKQUEUE_PRIORITY),
                           NULL);
        k_thread_name_set(&tp_workq.thread, "hid_touchpad");
        workq_started = true;
    }

    k_work_init(&data->work, hid_touchpad_work_cb);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, hid_touchpad_gpio_cb, BIT(config->dr.pin));
    int ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    set_int(dev, true);

    /* If a report is already pending, DR is asserted right now and the
     * edge-triggered interrupt will never fire for it — drain it once. */
    if (gpio_pin_get_dt(&config->dr) > 0) {
        k_work_submit_to_queue(&tp_workq, &data->work);
    }

    LOG_INF("HID touchpad passthrough initialized at 0x%x", config->i2c_bus.addr);

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

#endif /* IS_ENABLED(CONFIG_PM_DEVICE) */

#define HID_TOUCHPAD_INIT(n)                                                                      \
    static struct hid_touchpad_data hid_touchpad_data_##n;                                        \
    static const struct hid_touchpad_config hid_touchpad_config_##n = {                           \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                      \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                 \
        .hid_desc_register = DT_INST_PROP(n, hid_descriptor_register),                            \
    };                                                                                            \
    PM_DEVICE_DT_INST_DEFINE(n, hid_touchpad_pm_action);                                          \
    DEVICE_DT_INST_DEFINE(n, hid_touchpad_init, PM_DEVICE_DT_INST_GET(n),                         \
                          &hid_touchpad_data_##n, &hid_touchpad_config_##n,                       \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HID_TOUCHPAD_INIT)

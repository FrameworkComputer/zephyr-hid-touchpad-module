/*
 * Copyright (c) 2025 The ZMK Contributors
 * Copyright (c) 2025-2026 Framework Computer Inc
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

/* The descriptor and feature-size table below are taken from instance 0
 * while the device glue is instance-templated. That is deliberate: the BLE
 * service and the USB tp_hid node are single-instance too, and no board has
 * a second pad. Make the assumption explicit instead of half-supporting it. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "hid_touchpad supports exactly one enabled zmk,hid-touchpad instance");

/* HID report descriptor from devicetree */
const uint8_t tp_report_desc[] = DT_INST_PROP(0, report_descriptor);
const size_t tp_report_desc_size = DT_INST_PROP_LEN(0, report_descriptor);

/* Feature report id -> data size, from devicetree. Bounds the I2C read a
 * GET_REPORT clocks: the pad clock-stretches per byte, so an unbounded
 * 264-byte read cost ~14 ms (400 kHz) for a 1-byte report. */
static const uint8_t tp_feature_ids[] = DT_INST_PROP(0, feature_report_ids);
static const uint16_t tp_feature_sizes[] = DT_INST_PROP(0, feature_report_sizes);
BUILD_ASSERT(ARRAY_SIZE(tp_feature_ids) == ARRAY_SIZE(tp_feature_sizes),
             "feature-report-sizes must be parallel to feature-report-ids");

int hid_touchpad_feature_size(uint8_t id) {
    for (size_t i = 0; i < ARRAY_SIZE(tp_feature_ids); i++) {
        if (tp_feature_ids[i] == id) {
            return tp_feature_sizes[i];
        }
    }
    return -ENOENT;
}

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

/* Every I2C transaction goes through tp_lock()/tp_unlock(). The mutex keeps
 * transactions from interleaving (the USB control thread, the BLE work
 * queue, the factory protocol and our own input reads all talk to the same
 * pad); the claim owner check on top makes hid_touchpad_claim() exclusive
 * without the owner having to hold the mutex for seconds, which would leave
 * the other callers blocked instead of failing fast. The owner is checked
 * again after the lock is taken so a caller that raced past the first check
 * while a claim was being set up still backs off. */
static int tp_lock(struct hid_touchpad_data *data) {
    k_tid_t owner = data->claim_owner;
    if (owner != NULL && owner != k_current_get()) {
        return -EBUSY;
    }
    int err = k_mutex_lock(&data->lock, K_FOREVER);
    if (err) {
        return err;
    }
    owner = data->claim_owner;
    if (owner != NULL && owner != k_current_get()) {
        k_mutex_unlock(&data->lock);
        return -EBUSY;
    }
    return 0;
}

static void tp_unlock(struct hid_touchpad_data *data) { k_mutex_unlock(&data->lock); }

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

    /* Clock only what can be used: 2 length + 1 report ID bytes of framing
     * plus the smaller of the declared report size and the caller's buffer
     * (a host GET with a small wLength shouldn't occupy the bus for 256
     * bytes either). Unknown IDs and input-type GETs keep the full-buffer
     * read: rare, correct, just slow. The response's length field still
     * tells us the true frame size, so a device reporting a shorter frame
     * than we read is handled exactly as before. */
    size_t read_len = sizeof(i2c_buf);
    if (type == I2C_HID_REPORT_TYPE_FEATURE) {
        int size = hid_touchpad_feature_size(id);
        if (size >= 0) {
            read_len = MIN((size_t)MIN(size, buf_len) + 3, sizeof(i2c_buf));
        }
    }

    int err = tp_lock(data);
    if (err) {
        return err;
    }
    err = i2c_write_read_dt(&config->i2c_bus, cmd, cmd_len, i2c_buf, read_len);
    tp_unlock(data);
    if (err) {
        LOG_ERR("get_report id=%u failed: %d", id, err);
        return err;
    }

    uint16_t frame_len = sys_get_le16(i2c_buf);
    if (frame_len < 3 || frame_len > sizeof(i2c_buf)) {
        LOG_ERR("get_report id=%u invalid frame length: %u", id, frame_len);
        return -EINVAL;
    }

    /* i2c_buf[0..1] = length, i2c_buf[2] = report ID, i2c_buf[3..] = data.
     * Never hand out more than was actually read. */
    uint16_t data_len = MIN(frame_len, read_len) - 3;
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
        int err = tp_lock(data);
        if (err) {
            return err;
        }
        err = i2c_write_dt(&config->i2c_bus, msg, 9 + len);
        tp_unlock(data);
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
        int err = tp_lock(data);
        if (err) {
            return err;
        }
        err = i2c_write_dt(&config->i2c_bus, msg, 10 + len);
        tp_unlock(data);
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
    int err = tp_lock(data);
    if (err) {
        return err;
    }
    err = i2c_write_dt(&config->i2c_bus, msg, sizeof(msg));
    tp_unlock(data);
    if (err) {
        LOG_ERR("set_power %u failed: %d", state, err);
    }
    return err;
}

int hid_touchpad_reg_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;
    int err = tp_lock(data);
    if (err) {
        return err;
    }
    err = i2c_reg_read_byte_dt(&config->i2c_bus, reg, val);
    tp_unlock(data);
    return err;
}

int hid_touchpad_reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;
    int err = tp_lock(data);
    if (err) {
        return err;
    }
    err = i2c_reg_write_byte_dt(&config->i2c_bus, reg, val);
    tp_unlock(data);
    return err;
}

void hid_touchpad_register_input_cb(const struct device *dev, hid_touchpad_input_cb_t cb) {
    struct hid_touchpad_data *data = dev->data;
    if (data->num_cbs < HID_TOUCHPAD_MAX_CBS) {
        data->input_cbs[data->num_cbs++] = cb;
    } else {
        LOG_ERR("Too many input callbacks registered");
    }
}

/* Fan one input report out to the registered passthrough backends. Shared by
 * the I2C read path and hid_touchpad_inject_input() so a synthetic frame is
 * indistinguishable from a real one downstream. */
static void tp_dispatch_input(const struct device *dev, uint8_t report_id, const uint8_t *data,
                              uint16_t len) {
    const struct hid_touchpad_data *tp = dev->data;

    for (int i = 0; i < tp->num_cbs; i++) {
        if (tp->input_cbs[i]) {
            tp->input_cbs[i](dev, report_id, data, len);
        }
    }
}

void hid_touchpad_inject_input(const struct device *dev, uint8_t report_id, const uint8_t *data,
                               uint16_t len) {
    LOG_DBG("inject report id=%u len=%u", report_id, len);
    tp_dispatch_input(dev, report_id, data, len);
}

#ifdef CONFIG_HID_TOUCHPAD_INPUT_STATS
/* I2C read-duration stats, reported by the 1 Hz counter in
 * hid_touchpad_report_data. */
static uint32_t tp_dbg_read_us_sum, tp_dbg_read_us_n, tp_dbg_read_us_max;
#endif

static int hid_touchpad_report_data(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

#ifdef CONFIG_HID_TOUCHPAD_INPUT_STATS
    /* Time the I2C read itself, to split "each read is slow (pad
     * clock-stretching/wedged)" from "reads are fast but infrequent (work not
     * scheduled)". Reported by the 1 Hz counter below. */
    uint32_t c0 = k_cycle_get_32();
#endif
    int err = tp_lock(data);
    if (err) {
        /* Claimed: the owner disabled DR and will drain on release. */
        return err;
    }
    err = i2c_read_dt(&config->i2c_bus, data->report_buf, data->max_input_len);
    tp_unlock(data);
#ifdef CONFIG_HID_TOUCHPAD_INPUT_STATS
    uint32_t read_us = k_cyc_to_us_floor32(k_cycle_get_32() - c0);
    tp_dbg_read_us_sum += read_us;
    tp_dbg_read_us_n++;
    if (read_us > tp_dbg_read_us_max) {
        tp_dbg_read_us_max = read_us;
    }
#endif
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

#ifdef CONFIG_HID_TOUCHPAD_INPUT_STATS
    /* 1 Hz production-rate counter, to split "TP frames read too slowly" from
     * "frames pile up on the way out". */
    {
        static uint32_t frame_count;
        static int64_t last_report_ms;
        int64_t now = k_uptime_get();

        frame_count++;
        if (now - last_report_ms >= 1000) {
            uint32_t avg = tp_dbg_read_us_n ? tp_dbg_read_us_sum / tp_dbg_read_us_n : 0;
            LOG_INF("tp input: %u frames in %lld ms, i2c read avg %u us max %u us",
                    frame_count, now - last_report_ms, avg, tp_dbg_read_us_max);
            frame_count = 0;
            last_report_ms = now;
            tp_dbg_read_us_sum = tp_dbg_read_us_n = tp_dbg_read_us_max = 0;
        }
    }
#endif

    tp_dispatch_input(dev, report_id, &data->report_buf[3], data_len);
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

/* Enable the DR interrupt and, if a report is already pending, drain it:
 * DR is asserted right now and the edge-triggered interrupt will never fire
 * for it. Shared by init, PM resume and hid_touchpad_release(). */
static int tp_arm_dr(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    int ret = set_int(dev, true);
    if (ret == 0 && gpio_pin_get_dt(&config->dr) > 0) {
        k_work_submit_to_queue(&tp_workq, &data->work);
    }
    return ret;
}

int hid_touchpad_claim(const struct device *dev, k_timeout_t timeout) {
    struct hid_touchpad_data *data = dev->data;

    /* Take the mutex only to publish ourselves as owner atomically with
     * respect to an in-flight transaction; from then on tp_lock() turns every
     * other thread away, so we don't need to keep holding it. Not holding it
     * also makes the k_work_cancel_sync() below deadlock-free: a work item
     * that passed the owner check before we set it is blocked on the mutex,
     * gets it, does its one read, sees the owner on its next pass and exits. */
    int err = k_mutex_lock(&data->lock, timeout);
    if (err) {
        return err;
    }
    if (data->claim_owner != NULL) {
        k_mutex_unlock(&data->lock);
        return -EALREADY;
    }
    data->claim_owner = k_current_get();
    k_mutex_unlock(&data->lock);

    set_int(dev, false);
    struct k_work_sync sync;
    k_work_cancel_sync(&data->work, &sync);
    LOG_DBG("claimed by %p", data->claim_owner);
    return 0;
}

int hid_touchpad_release(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;

    if (data->claim_owner != k_current_get()) {
        return -EPERM;
    }
    data->claim_owner = NULL;
    LOG_DBG("released");
    return tp_arm_dr(dev);
}

/* Read the 30-byte I2C HID descriptor from the configured register and
 * refresh the register addresses we derive from it. Retries with delay
 * because an external enable rail may still be ramping or the pad firmware
 * may still be booting (at POST_KERNEL init, or right after a vendor reset)
 * — external startup-delay can't be relied on across init-priority
 * boundaries. Caller holds the lock (or is in init, before anyone else can
 * call). */
static int tp_read_descriptor(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    uint8_t hid_desc[30] = {0};
    int err = -EIO;
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
    data->max_input_len = sys_get_le16(&hid_desc[10]);

    /* A frame is at least the 2 length bytes + report ID. Anything smaller
     * means we read garbage instead of a descriptor (wrong register, pad not
     * in HID mode); the read path's report_len < 3 guard would keep us safe
     * later, but don't present a plausible-looking driver on top of it. */
    if (data->max_input_len < 3) {
        LOG_ERR("implausible wMaxInputLength %d, descriptor invalid", data->max_input_len);
        return -ENODEV;
    }
    /* Clamp max_input_len to our buffer size */
    if (data->max_input_len > sizeof(data->report_buf)) {
        LOG_WRN("max_input_len %d exceeds buffer, clamping to %d",
                data->max_input_len, (int)sizeof(data->report_buf));
        data->max_input_len = sizeof(data->report_buf);
    }

    /* The input register is not needed: per I2C-HID, input reports are read
     * with a plain read (no register address). Log it for reference only. */
    LOG_INF("command_reg=0x%02x data_reg=0x%02x input_reg=0x%04x max_input_len=%d",
            data->command_reg, data->data_reg, sys_get_le16(&hid_desc[8]), data->max_input_len);
    return 0;
}

int hid_touchpad_reinit_descriptor(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    int err = tp_lock(data);
    if (err) {
        return err;
    }
    err = tp_read_descriptor(dev);
    tp_unlock(data);
    return err;
}

static int hid_touchpad_init(const struct device *dev) {
    struct hid_touchpad_data *data = dev->data;
    const struct hid_touchpad_config *config = dev->config;

    /* Before anything that could fail: a not-ready device must still have a
     * usable mutex in case a caller ignores device_is_ready(). */
    k_mutex_init(&data->lock);
    data->claim_owner = NULL;

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_WRN("i2c bus not ready!");
        return -EINVAL;
    }

    int err = i2c_recover_bus(config->i2c_bus.bus);
    if (err) {
        LOG_WRN("I2C bus recovery failed or not supported: %d", err);
    }

    err = tp_read_descriptor(dev);
    if (err) {
        return err;
    }

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

    tp_arm_dr(dev);

    LOG_INF("HID touchpad passthrough initialized at 0x%x", config->i2c_bus.addr);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int hid_touchpad_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        /* A report that became ready while suspended holds DR asserted, and
         * the edge-triggered interrupt never fires for it — tp_arm_dr drains
         * it once, same as init does. */
        return tp_arm_dr(dev);
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

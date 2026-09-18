/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * PixArt PJP360 firmware update, ported from fwupd plugins/pixart-tp
 * (fu-pixart-tp-device.c, fu-pixart-tp-pjp360-device.c). The register map
 * and key values below are a direct transcription of fu-pixart-tp.rs, which
 * is the only documentation of this protocol we have; keep the names close
 * to it so the two can be diffed.
 *
 * Everything goes over three vendor HID feature reports the pad exposes:
 *   0x41 BURST   256 B  bulk write into the pad's SRAM page buffer
 *   0x42 SINGLE    3 B  system-bank register access  [addr, bank, val]
 *   0x43 USER      3 B  user-bank register access    [addr, bank, val]
 * A register read is a SET with bank|0x10 followed by a GET whose reply
 * carries the value in the last byte. fwupd sleeps 10 ms between the two
 * because a USB HID round trip needs it; over I2C-HID a GET is one
 * write-read transaction and the pad clock-stretches as needed.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "hid_touchpad.h"
#include "pixart_tp_update.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pixart_tp_update, CONFIG_HID_TOUCHPAD_LOG_LEVEL);

/* --- register map (fu-pixart-tp.rs) --------------------------------------- */

#define REPORT_ID_BURST 0x41
#define REPORT_ID_SINGLE 0x42
#define REPORT_ID_USER 0x43
#define OP_READ 0x10

#define SYS_BANK0 0x00
#define SYS_BANK1 0x01
#define SYS_BANK4 0x04
#define SYS_BANK6 0x06
#define USER_BANK0 0x00

/* system bank 0 */
#define REG_SYS0_PART_ID 0x78 /* u16 LE */
/* system bank 1 */
#define REG_SYS1_CLOCKS_POWER_UP 0x0d
#define REG_SYS1_RESET_KEY1 0x2c
#define REG_SYS1_RESET_KEY2 0x2d
#define CLOCKS_POWER_UP_CPU BIT(1)
#define RESET_KEY1_SUSPEND 0xaa
#define RESET_KEY2_REGULAR 0xbb
#define RESET_KEY2_BOOTLOADER 0xcc
/* system bank 4 (flash engine) */
#define REG_SYS4_FLASH_STATUS 0x1c
#define REG_SYS4_FLASH_INST_CMD 0x2c
#define REG_SYS4_FLASH_BUF_ADDR 0x2e /* u16 LE */
#define REG_SYS4_FLASH_CCR 0x40      /* u32 LE */
#define REG_SYS4_FLASH_DATA_CNT 0x44 /* u16 LE */
#define REG_SYS4_FLASH_ADDR 0x48     /* u32 LE */
#define REG_SYS4_FLASH_EXECUTE 0x56
#define FLASH_STATUS_BUSY 0x01
#define FLASH_STATUS_WEL 0x02
#define FLASH_INST_NONE 0x00
#define FLASH_INST_RD2_REG_BANK BIT(0)
#define FLASH_INST_PROGRAM BIT(2)
#define FLASH_INST_INTERNAL_SRAM BIT(7)
#define FLASH_CCR_WRITE_ENABLE 0x00000106
#define FLASH_CCR_READ_STATUS 0x01000105
#define FLASH_CCR_ERASE_SECTOR 0x00002520
#define FLASH_CCR_PROGRAM_PAGE 0x01002502
#define FLASH_EXEC_START 0x01
#define FLASH_EXEC_SUCCESS 0x00
/* system bank 6 (sram buffer) */
#define REG_SYS6_SRAM_SELECT 0x09
#define REG_SYS6_SRAM_TRIGGER 0x0a
#define REG_SYS6_SRAM_ADDR 0x10 /* u16 LE */
#define SRAM_TRIGGER_NCS_ENABLE 0x00
#define SRAM_TRIGGER_NCS_DISABLE 0x01
/* user bank 0 */
#define REG_USER0_BOOT_STATUS 0x00
#define REG_USER0_INTERNAL_VERSION 0x02 /* u16 LE: bits 0-11 minor, 12-15 major */
#define REG_USER0_CRC_CTRL 0x82
#define REG_USER0_CRC_RESULT 0x84            /* u32 LE */
#define REG_USER0_HID_VERSION_MAJOR_ONE 0x7e /* ROM code major 1 (IC ECO) */
#define REG_USER0_HID_VERSION_OTHER 0x80
#define CRC_CTRL_FW_BANK0 0x02
#define CRC_CTRL_PARAM_BANK0 0x04
#define CRC_CTRL_BUSY 0x01

/* Quirk [PIXARTTP\PARTID_0360]: PixartTpSramSelect = 0x00. The fwupd base
 * class defaults to 0x0F; that is NOT what runs against this part. */
#define PJP360_SRAM_SELECT 0x00

#define SECTOR_SIZE 4096
#define PAGE_SIZE 256

/* --- register access ------------------------------------------------------ */

static int reg_write(const struct device *dev, uint8_t report, uint8_t bank, uint8_t addr,
                     uint8_t val) {
    uint8_t buf[3] = {addr, bank, val};
    int err = hid_touchpad_set_report(dev, I2C_HID_REPORT_TYPE_FEATURE, report, buf, sizeof(buf));
    if (err) {
        LOG_ERR("reg write %02x bank%u 0x%02x=0x%02x: %d", report, bank, addr, val, err);
    }
    return err;
}

static int reg_read(const struct device *dev, uint8_t report, uint8_t bank, uint8_t addr,
                    uint8_t *val) {
    uint8_t cmd[3] = {addr, bank | OP_READ, 0x00};
    int err = hid_touchpad_set_report(dev, I2C_HID_REPORT_TYPE_FEATURE, report, cmd, sizeof(cmd));
    if (err) {
        LOG_ERR("reg read cmd %02x bank%u 0x%02x: %d", report, bank, addr, err);
        return err;
    }
    uint8_t resp[3] = {0};
    uint16_t out_len = 0;
    err = hid_touchpad_get_report(dev, I2C_HID_REPORT_TYPE_FEATURE, report, resp, sizeof(resp),
                                  &out_len);
    if (err) {
        LOG_ERR("reg read resp %02x bank%u 0x%02x: %d", report, bank, addr, err);
        return err;
    }
    if (out_len < 3) {
        LOG_ERR("reg read %02x bank%u 0x%02x: short reply %u", report, bank, addr, out_len);
        return -EIO;
    }
    *val = resp[2];
    return 0;
}

static int sys_write(const struct device *dev, uint8_t bank, uint8_t addr, uint8_t val) {
    return reg_write(dev, REPORT_ID_SINGLE, bank, addr, val);
}
static int sys_read(const struct device *dev, uint8_t bank, uint8_t addr, uint8_t *val) {
    return reg_read(dev, REPORT_ID_SINGLE, bank, addr, val);
}
static int user_write(const struct device *dev, uint8_t bank, uint8_t addr, uint8_t val) {
    return reg_write(dev, REPORT_ID_USER, bank, addr, val);
}
static int user_read(const struct device *dev, uint8_t bank, uint8_t addr, uint8_t *val) {
    return reg_read(dev, REPORT_ID_USER, bank, addr, val);
}

/* Multi-byte registers are little-endian, one byte per consecutive address. */
static int sys_write_n(const struct device *dev, uint8_t bank, uint8_t addr, uint32_t val, int n) {
    for (int i = 0; i < n; i++) {
        int err = sys_write(dev, bank, addr + i, (uint8_t)(val >> (8 * i)));
        if (err) {
            return err;
        }
    }
    return 0;
}

static int read_u16(const struct device *dev, uint8_t report, uint8_t bank, uint8_t addr,
                    uint16_t *val) {
    uint8_t lo, hi;
    int err = reg_read(dev, report, bank, addr, &lo);
    if (!err) {
        err = reg_read(dev, report, bank, addr + 1, &hi);
    }
    if (!err) {
        *val = (uint16_t)lo | ((uint16_t)hi << 8);
    }
    return err;
}

static int user_read_u32(const struct device *dev, uint8_t addr, uint32_t *val) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t b;
        int err = user_read(dev, USER_BANK0, addr + i, &b);
        if (err) {
            return err;
        }
        v |= (uint32_t)b << (8 * i);
    }
    *val = v;
    return 0;
}

/* --- device control -------------------------------------------------------- */

enum reset_mode { RESET_APPLICATION, RESET_BOOTLOADER };

static int pad_reset(const struct device *dev, enum reset_mode mode) {
    int err = sys_write(dev, SYS_BANK1, REG_SYS1_RESET_KEY1, RESET_KEY1_SUSPEND);
    if (err) {
        return err;
    }
    k_msleep(30);
    err = sys_write(dev, SYS_BANK1, REG_SYS1_RESET_KEY2,
                    mode == RESET_APPLICATION ? RESET_KEY2_REGULAR : RESET_KEY2_BOOTLOADER);
    if (err) {
        return err;
    }
    k_msleep(mode == RESET_APPLICATION ? 500 : 10);

    /* The pad's HID stack just rebooted. Refresh command/data registers from
     * its descriptor; in bootloader mode this may legitimately fail (fwupd
     * never re-reads and works), so only insist on it for application mode. */
    err = hid_touchpad_reinit_descriptor(dev);
    if (err) {
        if (mode == RESET_APPLICATION) {
            LOG_ERR("pad did not come back as an I2C-HID device: %d", err);
            return err;
        }
        LOG_WRN("no HID descriptor in bootloader mode (%d), keeping previous registers", err);
    }
    return 0;
}

static int read_info_locked(const struct device *dev, struct pixart_tp_info *info) {
    int err = read_u16(dev, REPORT_ID_SINGLE, SYS_BANK0, REG_SYS0_PART_ID, &info->part_id);
    if (err) {
        return err;
    }
    err = user_read(dev, USER_BANK0, REG_USER0_BOOT_STATUS, &info->boot_status);
    if (err) {
        return err;
    }
    uint16_t internal;
    err = read_u16(dev, REPORT_ID_USER, USER_BANK0, REG_USER0_INTERNAL_VERSION, &internal);
    if (err) {
        return err;
    }
    info->rom_major = (internal >> 12) & 0x0f;
    /* IC ECO swapped the HID version register; fwupd selects by ROM major. */
    uint8_t ver_addr =
        info->rom_major == 1 ? REG_USER0_HID_VERSION_MAJOR_ONE : REG_USER0_HID_VERSION_OTHER;
    return read_u16(dev, REPORT_ID_USER, USER_BANK0, ver_addr, &info->version);
}

int pixart_tp_read_info(const struct device *dev, struct pixart_tp_info *info) {
    int err = hid_touchpad_claim(dev, K_MSEC(100));
    if (err) {
        return err;
    }
    err = read_info_locked(dev, info);
    hid_touchpad_release(dev);
    return err;
}

/* --- flash engine ---------------------------------------------------------- */

static int flash_execute(const struct device *dev, uint8_t inst, uint32_t ccr, uint16_t cnt) {
    int err = sys_write(dev, SYS_BANK4, REG_SYS4_FLASH_INST_CMD, inst);
    if (!err) {
        err = sys_write_n(dev, SYS_BANK4, REG_SYS4_FLASH_CCR, ccr, 4);
    }
    if (!err) {
        err = sys_write_n(dev, SYS_BANK4, REG_SYS4_FLASH_DATA_CNT, cnt, 2);
    }
    if (!err) {
        err = sys_write(dev, SYS_BANK4, REG_SYS4_FLASH_EXECUTE, FLASH_EXEC_START);
    }
    if (err) {
        return err;
    }
    for (int i = 0; i < 10; i++) {
        uint8_t st;
        err = sys_read(dev, SYS_BANK4, REG_SYS4_FLASH_EXECUTE, &st);
        if (err) {
            return err;
        }
        if (st == FLASH_EXEC_SUCCESS) {
            return 0;
        }
        k_msleep(1);
    }
    LOG_ERR("flash execute inst=0x%02x ccr=0x%08x timed out", inst, ccr);
    return -ETIMEDOUT;
}

static int flash_read_status(const struct device *dev, uint8_t *status) {
    int err = flash_execute(dev, FLASH_INST_RD2_REG_BANK, FLASH_CCR_READ_STATUS, 1);
    if (err) {
        return err;
    }
    k_msleep(1);
    return sys_read(dev, SYS_BANK4, REG_SYS4_FLASH_STATUS, status);
}

static int flash_write_enable(const struct device *dev) {
    int err = flash_execute(dev, FLASH_INST_NONE, FLASH_CCR_WRITE_ENABLE, 0);
    if (err) {
        return err;
    }
    for (int i = 0; i < 10; i++) {
        uint8_t st;
        err = flash_read_status(dev, &st);
        if (err) {
            return err;
        }
        if (st & FLASH_STATUS_WEL) {
            return 0;
        }
    }
    LOG_ERR("flash write enable: WEL never set");
    return -ETIMEDOUT;
}

static int flash_wait_busy(const struct device *dev) {
    for (int i = 0; i < 1000; i++) {
        uint8_t st;
        int err = flash_read_status(dev, &st);
        if (err) {
            return err;
        }
        if (!(st & FLASH_STATUS_BUSY)) {
            return 0;
        }
    }
    LOG_ERR("flash still busy after 1000 polls");
    return -ETIMEDOUT;
}

static int flash_erase_sector(const struct device *dev, unsigned int sector) {
    if (sector > UINT8_MAX) {
        return -EINVAL;
    }
    int err = flash_wait_busy(dev);
    if (!err) {
        err = flash_write_enable(dev);
    }
    if (!err) {
        err = sys_write_n(dev, SYS_BANK4, REG_SYS4_FLASH_ADDR, sector * SECTOR_SIZE, 4);
    }
    if (!err) {
        LOG_DBG("erase sector %u", sector);
        err = flash_execute(dev, FLASH_INST_NONE, FLASH_CCR_ERASE_SECTOR, 0);
    }
    return err;
}

/* Load one 256-byte page into the pad's SRAM buffer via the burst report. */
static int write_sram_page(const struct device *dev, const uint8_t *page) {
    int err = sys_write_n(dev, SYS_BANK6, REG_SYS6_SRAM_ADDR, 0x0000, 2);
    if (!err) {
        err = sys_write(dev, SYS_BANK6, REG_SYS6_SRAM_SELECT, PJP360_SRAM_SELECT);
    }
    if (!err) {
        err = sys_write(dev, SYS_BANK6, REG_SYS6_SRAM_TRIGGER, SRAM_TRIGGER_NCS_ENABLE);
    }
    if (!err) {
        err = hid_touchpad_set_report(dev, I2C_HID_REPORT_TYPE_FEATURE, REPORT_ID_BURST, page,
                                      PAGE_SIZE);
        if (err) {
            LOG_ERR("burst write failed: %d", err);
        }
    }
    if (!err) {
        err = sys_write(dev, SYS_BANK6, REG_SYS6_SRAM_TRIGGER, SRAM_TRIGGER_NCS_DISABLE);
    }
    return err;
}

/* Program the SRAM buffer into flash at sector/page. */
static int flash_program_page(const struct device *dev, unsigned int sector, unsigned int page) {
    uint32_t addr = sector * SECTOR_SIZE + page * PAGE_SIZE;
    int err = flash_wait_busy(dev);
    if (!err) {
        err = flash_write_enable(dev);
    }
    if (!err) {
        err = sys_write_n(dev, SYS_BANK4, REG_SYS4_FLASH_BUF_ADDR, 0x0000, 2);
    }
    if (!err) {
        err = sys_write_n(dev, SYS_BANK4, REG_SYS4_FLASH_ADDR, addr, 4);
    }
    if (!err) {
        err = flash_execute(dev, FLASH_INST_PROGRAM | FLASH_INST_INTERNAL_SRAM,
                            FLASH_CCR_PROGRAM_PAGE, PAGE_SIZE);
    }
    return err;
}

static int write_page(const struct device *dev, unsigned int sector, unsigned int page,
                      const uint8_t *data, size_t len) {
    uint8_t buf[PAGE_SIZE];
    memset(buf, 0xff, sizeof(buf));
    memcpy(buf, data, MIN(len, sizeof(buf)));
    int err = write_sram_page(dev, buf);
    if (!err) {
        err = flash_program_page(dev, sector, page);
    }
    return err;
}

struct progress {
    pixart_tp_progress_cb_t cb;
    void *user;
    unsigned int done, total;
};

static void report(struct progress *p, enum pixart_tp_update_stage stage) {
    if (p->cb) {
        p->cb(stage, p->done, p->total, p->user);
    }
}

/* One 4 KiB sector's worth of a section: pages 1..N-1 first, page 0 last.
 * Page 0 carries the validity marker, so a power loss mid-sector leaves an
 * image the pad rejects rather than one it half-runs. Keep this order. */
static int write_sector(const struct device *dev, unsigned int sector, const uint8_t *data,
                        size_t len, struct progress *p) {
    unsigned int pages = DIV_ROUND_UP(len, PAGE_SIZE);
    for (unsigned int i = 1; i < pages; i++) {
        size_t off = i * PAGE_SIZE;
        int err = write_page(dev, sector, i, data + off, len - off);
        if (err) {
            return err;
        }
        p->done++;
        report(p, PIXART_TP_STAGE_PROGRAM);
    }
    int err = write_page(dev, sector, 0, data, len);
    if (err) {
        return err;
    }
    p->done++;
    report(p, PIXART_TP_STAGE_PROGRAM);
    return 0;
}

static int write_section(const struct device *dev, const struct pixart_tp_fw_section *sec,
                         struct progress *p) {
    if (sec->len == 0) {
        return 0;
    }
    if (sec->flash_start % SECTOR_SIZE) {
        LOG_ERR("section flash start 0x%08x not sector aligned", sec->flash_start);
        return -EINVAL;
    }
    unsigned int first = sec->flash_start / SECTOR_SIZE;
    unsigned int sectors = DIV_ROUND_UP(sec->len, SECTOR_SIZE);

    int err = sys_write(dev, SYS_BANK1, REG_SYS1_CLOCKS_POWER_UP, CLOCKS_POWER_UP_CPU);
    if (err) {
        return err;
    }
    for (unsigned int i = 0; i < sectors; i++) {
        err = flash_erase_sector(dev, first + i);
        if (err) {
            LOG_ERR("erase sector %u failed: %d", first + i, err);
            return err;
        }
    }
    for (unsigned int i = 0; i < sectors; i++) {
        size_t off = i * SECTOR_SIZE;
        err = write_sector(dev, first + i, sec->data + off, MIN(sec->len - off, SECTOR_SIZE), p);
        if (err) {
            LOG_ERR("write sector %u failed: %d", first + i, err);
            return err;
        }
    }
    return 0;
}

static const struct pixart_tp_fw_section *find_section(const struct pixart_tp_fw_image *img,
                                                       uint8_t type) {
    for (size_t i = 0; i < img->num_sections; i++) {
        if (img->sections[i].type == type) {
            return &img->sections[i];
        }
    }
    return NULL;
}

/* Erase the FW section's first sector so the pad boots to ROM: the "no valid
 * image" state, which is safe and which the next update recovers from. */
static int firmware_clear(const struct device *dev, const struct pixart_tp_fw_image *img) {
    const struct pixart_tp_fw_section *fw = find_section(img, PIXART_TP_UPDATE_TYPE_FW);
    if (!fw) {
        return -EINVAL;
    }
    return flash_erase_sector(dev, fw->flash_start / SECTOR_SIZE);
}

static int pad_crc(const struct device *dev, uint8_t ctrl, uint32_t *crc) {
    int err = user_write(dev, USER_BANK0, REG_USER0_CRC_CTRL, ctrl);
    if (err) {
        return err;
    }
    for (int i = 0; i < 1000; i++) {
        uint8_t st;
        err = user_read(dev, USER_BANK0, REG_USER0_CRC_CTRL, &st);
        if (err) {
            return err;
        }
        if (!(st & CRC_CTRL_BUSY)) {
            return user_read_u32(dev, REG_USER0_CRC_RESULT, crc);
        }
        k_msleep(10);
    }
    LOG_ERR("CRC engine (ctrl 0x%02x) never finished", ctrl);
    return -ETIMEDOUT;
}

static int verify_section(const struct device *dev, const struct pixart_tp_fw_image *img,
                          uint8_t type, uint8_t ctrl, const char *what) {
    const struct pixart_tp_fw_section *sec = find_section(img, type);
    if (!sec) {
        return -EINVAL;
    }
    uint32_t crc;
    int err = pad_crc(dev, ctrl, &crc);
    if (err) {
        return err;
    }
    if (crc != sec->crc) {
        LOG_ERR("%s CRC mismatch: pad 0x%08x file 0x%08x", what, crc, sec->crc);
        return -EBADMSG;
    }
    LOG_INF("%s CRC 0x%08x ok", what, crc);
    return 0;
}

static int update_locked(const struct device *dev, const struct pixart_tp_fw_image *img,
                         struct progress *p) {
    struct pixart_tp_info info;
    int err;

    /* A pad the shield's power policy put into I2C-HID sleep may not answer
     * vendor reports; waking it costs one 4-byte write. */
    (void)hid_touchpad_set_power(dev, I2C_HID_PWR_ON);

    err = read_info_locked(dev, &info);
    if (err) {
        LOG_ERR("cannot read pad info: %d", err);
        return err;
    }
    if (info.part_id != img->part_id) {
        LOG_ERR("pad part 0x%04x, image is for 0x%04x", info.part_id, img->part_id);
        return -ENODEV;
    }
    if (info.boot_status == PIXART_TP_BOOT_STATUS_FLASHLESS) {
        LOG_ERR("flashless pad, cannot update");
        return -ENOTSUP;
    }
    if (!find_section(img, PIXART_TP_UPDATE_TYPE_FW) ||
        !find_section(img, PIXART_TP_UPDATE_TYPE_PARAM)) {
        LOG_ERR("image lacks FW or PARAM section");
        return -EINVAL;
    }
    LOG_INF("pad part 0x%04x boot 0x%02x version 0x%04x -> image 0x%04x", info.part_id,
            info.boot_status, info.version, img->version);

    p->total = 0;
    for (size_t i = 0; i < img->num_sections; i++) {
        const struct pixart_tp_fw_section *s = &img->sections[i];
        unsigned int sectors = DIV_ROUND_UP(s->len, SECTOR_SIZE);
        for (unsigned int k = 0; k < sectors; k++) {
            size_t off = k * SECTOR_SIZE;
            p->total += DIV_ROUND_UP(MIN(s->len - off, (size_t)SECTOR_SIZE), PAGE_SIZE);
        }
    }

    /* detach */
    report(p, PIXART_TP_STAGE_DETACH);
    err = pad_reset(dev, RESET_BOOTLOADER);
    if (err) {
        return err;
    }

    /* invalidate: from here on a power loss leaves a pad in ROM mode */
    report(p, PIXART_TP_STAGE_ERASE);
    err = firmware_clear(dev, img);
    if (err) {
        LOG_ERR("firmware clear failed: %d", err);
        return err;
    }

    /* program, in file order (fwupd writes sections as the file lists them) */
    for (size_t i = 0; i < img->num_sections; i++) {
        err = write_section(dev, &img->sections[i], p);
        if (err) {
            return err;
        }
    }

    /* verify with the pad's CRC engine, from a fresh bootloader reset */
    report(p, PIXART_TP_STAGE_VERIFY);
    err = pad_reset(dev, RESET_BOOTLOADER);
    if (err) {
        return err;
    }
    err = verify_section(dev, img, PIXART_TP_UPDATE_TYPE_FW, CRC_CTRL_FW_BANK0, "firmware");
    if (!err) {
        err = verify_section(dev, img, PIXART_TP_UPDATE_TYPE_PARAM, CRC_CTRL_PARAM_BANK0,
                             "parameter");
    }
    if (err) {
        if (err == -EBADMSG) {
            /* Make sure the pad boots to ROM rather than running a bad image. */
            (void)firmware_clear(dev, img);
        }
        return err;
    }

    /* attach */
    report(p, PIXART_TP_STAGE_ATTACH);
    err = pad_reset(dev, RESET_APPLICATION);
    if (err) {
        return err;
    }
    err = read_info_locked(dev, &info);
    if (err) {
        LOG_ERR("pad unreadable after update: %d", err);
        return err;
    }
    LOG_INF("pad now boot 0x%02x version 0x%04x", info.boot_status, info.version);
    if (info.boot_status == PIXART_TP_BOOT_STATUS_ROM || info.version != img->version) {
        return -EIO;
    }
    return 0;
}

int pixart_tp_update(const struct device *dev, const struct pixart_tp_fw_image *img,
                     pixart_tp_progress_cb_t cb, void *user) {
    struct progress p = {.cb = cb, .user = user};

    int err = hid_touchpad_claim(dev, K_MSEC(500));
    if (err) {
        LOG_ERR("cannot claim pad: %d", err);
        return err;
    }
    int64_t t0 = k_uptime_get();
    err = update_locked(dev, img, &p);
    LOG_INF("update %s in %lld ms", err ? "FAILED" : "done", k_uptime_get() - t0);
    hid_touchpad_release(dev);
    return err;
}

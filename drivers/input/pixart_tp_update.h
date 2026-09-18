/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * PixArt touchpad (PJP360 / PCT1036) firmware update over I2C-HID vendor
 * feature reports. Port of fwupd's plugins/pixart-tp device code to run on
 * the keyboard MCU that hosts the pad, so the pad's firmware can be carried
 * in the keyboard image and converged at boot.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Section update types, as in the FWHD file's section headers. */
#define PIXART_TP_UPDATE_TYPE_FW 1
#define PIXART_TP_UPDATE_TYPE_PARAM 3

/* User bank 0 register 0x00 boot status values. */
#define PIXART_TP_BOOT_STATUS_ROM 0x8c       /* running from ROM: flash image invalid */
#define PIXART_TP_BOOT_STATUS_FLASHLESS 0x9c /* no flash at all: never update */

#define PIXART_TP_PART_ID_PJP360 0x0360

struct pixart_tp_fw_section {
    uint8_t type;         /* PIXART_TP_UPDATE_TYPE_* */
    uint32_t flash_start; /* target flash address, 4 KiB aligned */
    uint32_t crc;         /* section CRC32 from the file header; the pad's own
                           * CRC engine must reproduce it after programming */
    const uint8_t *data;
    size_t len;
};

struct pixart_tp_fw_image {
    uint16_t version;                            /* FWHD file_ver, e.g. 0x1204 */
    uint16_t part_id;                            /* FWHD ic_part_id, e.g. 0x0360 */
    const struct pixart_tp_fw_section *sections; /* in file order */
    size_t num_sections;
};

struct pixart_tp_info {
    uint16_t part_id;
    uint8_t boot_status;
    uint8_t rom_major; /* ROM code major; selects the version register */
    uint16_t version;  /* HID firmware version, e.g. 0x1201 */
};

enum pixart_tp_update_stage {
    PIXART_TP_STAGE_DETACH,
    PIXART_TP_STAGE_ERASE,
    PIXART_TP_STAGE_PROGRAM,
    PIXART_TP_STAGE_VERIFY,
    PIXART_TP_STAGE_ATTACH,
};

/**
 * Progress callback. @p done / @p total count 256-byte pages in the PROGRAM
 * stage and are 0/0 otherwise.
 */
typedef void (*pixart_tp_progress_cb_t)(enum pixart_tp_update_stage stage, unsigned int done,
                                        unsigned int total, void *user);

/**
 * Read part ID, boot status and firmware version. Claims the pad for the
 * duration (a register read is a SET followed by a GET and must not be
 * split), so it fails with -EBUSY/-EALREADY while someone else holds it.
 */
int pixart_tp_read_info(const struct device *dev, struct pixart_tp_info *info);

/**
 * Flash @p img onto the pad: reset to bootloader, invalidate the current
 * image, erase + program every section, verify both CRCs with the pad's own
 * engine, reset to application, re-read the HID descriptor.
 *
 * Claims the pad for the whole sequence (seconds); every other driver
 * caller gets -EBUSY meanwhile. Sleeps, so call from a thread that may
 * block -- never the system work queue.
 *
 * Refuses (-ENODEV) when the pad's part ID differs from the image's.
 * On any failure after the invalidate step the pad is left in ROM mode,
 * which is exactly the state a retry recovers from.
 *
 * @return 0 on success, -EIO on protocol errors, -EBADMSG on CRC mismatch,
 *         or the driver's error.
 */
int pixart_tp_update(const struct device *dev, const struct pixart_tp_fw_image *img,
                     pixart_tp_progress_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif

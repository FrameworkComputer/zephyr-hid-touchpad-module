/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/util_macro.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Host numbering for the per-host feature report copies: USB, then one per
 * BLE profile. */
#define TP_HOST_USB 0
#define TP_HOST_BLE(profile) (1 + (profile))

struct bt_conn;

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_HOST_STATE)

/**
 * Host a BLE connection belongs to: its bonded profile, or the selected
 * profile while it is still pairing (ZMK only pairs on the selected one).
 */
int tp_hosts_from_conn(struct bt_conn *conn);

/**
 * A host's feature SET. For a host-state report, store it in the host's copy
 * and write it to the pad only if the host is the selected one.
 * @return -ENOENT if `id` is not a host-state report: forward it to the pad.
 */
int tp_hosts_set_feature(int host, uint8_t id, const uint8_t *data, uint16_t len);

/**
 * A host's feature GET. For a host-state report, answer from the host's copy.
 * @return -ENOENT if `id` is not a host-state report: ask the pad.
 */
int tp_hosts_get_feature(int host, uint8_t id, uint8_t *buf, uint16_t buf_len,
                         uint16_t *out_len);

/**
 * Hand the pad to a test tool that sets its modes itself (`hold` true):
 * every host's feature GETs and SETs go straight to the pad, and no host's
 * copy is written to it. Releasing writes the selected host's copy again.
 */
void tp_hosts_hold(bool hold);

/**
 * The USB host let go of the pad (bus reset, deconfigured, unplugged): its
 * copy goes back to the defaults, and whatever enumerates us next writes its
 * own. For the USB backend's own callback, which sees every reset.
 */
void tp_hosts_usb_reset(void);

#else

static inline int tp_hosts_from_conn(struct bt_conn *conn) { return -ENOENT; }

static inline int tp_hosts_set_feature(int host, uint8_t id, const uint8_t *data,
                                       uint16_t len) {
    return -ENOENT;
}

static inline int tp_hosts_get_feature(int host, uint8_t id, uint8_t *buf, uint16_t buf_len,
                                       uint16_t *out_len) {
    return -ENOENT;
}

static inline void tp_hosts_hold(bool hold) {}

static inline void tp_hosts_usb_reset(void) {}

#endif

#ifdef __cplusplus
}
#endif

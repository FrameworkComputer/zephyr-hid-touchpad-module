/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <zephyr/sys/util_macro.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)

/**
 * Resample the pad's mouse reports by time before they go to a BLE host
 * (see tp_mouse_take in hid_passthrough_ble.c), for firmware that makes
 * them from the pad's PTP frames itself. Off by default: the pad's own
 * mouse mode goes as the pad sends it.
 */
void hid_passthrough_ble_resample_mouse(bool on);

/**
 * Whether the selected BLE profile's host reads the pad's HID Service: it
 * enabled notifications for the pad's mouse or PTP input report. Android's
 * HID host only uses the first HID Service on a device (the keyboard's), so
 * it never does, and nothing notified on the pad's service reaches it.
 * False while the selected profile has no connection, and on a reconnect
 * until the bond's subscriptions are restored (once the link is encrypted).
 */
bool hid_passthrough_ble_host_reads_pad(void);

#else

static inline void hid_passthrough_ble_resample_mouse(bool on) {}

static inline bool hid_passthrough_ble_host_reads_pad(void) { return false; }

#endif

#ifdef __cplusplus
}
#endif

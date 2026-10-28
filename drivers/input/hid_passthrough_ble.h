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

#else

static inline void hid_passthrough_ble_resample_mouse(bool on) {}

#endif

#ifdef __cplusplus
}
#endif

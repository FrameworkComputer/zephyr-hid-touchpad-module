# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_HID_TOUCHPAD  hid_touchpad.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_USB  hid_passthrough_usb.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_BLE  hid_passthrough_ble.c)
zephyr_library_sources_ifndef(CONFIG_ZMK_HID_TOUCHPAD ${ZEPHYR_BASE}/misc/empty_file.c)

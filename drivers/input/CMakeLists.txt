# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_I2C_HID  i2c_hid.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_USB  hid_passthrough_usb.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_BLE  hid_passthrough_ble.c)
zephyr_library_sources_ifndef(CONFIG_ZMK_I2C_HID ${ZEPHYR_BASE}/misc/empty_file.c)

# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library()

# The passthrough backends route the pad to whichever endpoint the keyboard
# has selected (zmk_endpoint_get_selected), so they need ZMK's core headers in
# app/include, which the app keeps PRIVATE to its own target.
zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ZMK_HID_TOUCHPAD  hid_touchpad.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_USB_LEGACY  hid_passthrough_usb.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_USB_NEXT    hid_passthrough_usb_next.c)
zephyr_library_sources_ifdef(CONFIG_HID_PASSTHROUGH_BLE  hid_passthrough_ble.c)
zephyr_library_sources_ifndef(CONFIG_ZMK_HID_TOUCHPAD ${ZEPHYR_BASE}/misc/empty_file.c)

# I2C HID Passthrough Module

Zephyr module that forwards raw HID reports from an I2C HID device to USB
and/or BLE (HOGP) host interfaces. The report descriptor is provided via
devicetree — the driver does not parse it from the device.

## Features

- I2C HID host driver (reads HID descriptor, handles get/set report)
- USB HID passthrough (registers as USB HID device with the I2C device's report descriptor)
- BLE HOGP passthrough (exposes HID service with configurable input + feature reports)
- Report descriptor and report IDs configured via devicetree
- Per-module logging with runtime filtering

## Limitations

- BLE passthrough currently supports exactly 2 input reports (configured via
  `mouse-input-report-id` and `ptp-input-report-id` properties)
- Report descriptor must be provided as a blob in devicetree (not read from device)

## Configuration

### Kconfig

| Symbol | Description |
|--------|-------------|
| `CONFIG_ZMK_I2C_HID` | Enable the I2C HID core driver |
| `CONFIG_HID_PASSTHROUGH_USB` | Enable USB HID forwarding |
| `CONFIG_HID_PASSTHROUGH_BLE` | Enable BLE HOGP forwarding |

### Devicetree

The devicetree node label must be `i2c_hid`. Example with a Framework touchpad
(works with any I2C HID device):

```dts
&xiao_i2c {
    status = "okay";

    i2c_hid: i2c_hid@2c {
        compatible = "zmk,i2c-hid";
        status = "okay";
        reg = <0x2c>;
        dr-gpios = <&xiao_d 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        hid-descriptor-register = <0x20>;
        report-descriptor = [ /* ... full HID report descriptor blob ... */ ];
        mouse-input-report-id = <2>;
        ptp-input-report-id = <4>;
        mouse-input-report-size = <7>;
        ptp-input-report-size = <28>;
        feature-report-ids = <6 7 8>;
    };
};
```

### west.yml

```yaml
manifest:
  remotes:
    - name: FrameworkComputer
      url-base: https://github.com/FrameworkComputer
  projects:
    - name: zephyr-hid-touchpad-module
      revision: main
      remote: FrameworkComputer
```

### Kconfig example

```
CONFIG_I2C=y
CONFIG_INPUT=y
CONFIG_ZMK_I2C_HID=y
```

## Support

This module was developed for use with Framework touchpad modules but works
with any I2C HID device. It is not an officially supported Framework product —
provided as reference for tinkerers reusing I2C HID devices without their
original host controller.

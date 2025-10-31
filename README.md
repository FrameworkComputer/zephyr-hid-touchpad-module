# HID Touchpad Driver

Zephyr/ZMK driver for HID touchpads. Should work with most modern touchpads
that support PTP/mouse mode.

The driver does dynamically not parse the report descriptor from the touchpad, so the reports and report structure has to be configured through device tree.

## Example
ZMK example configuration for Seedstudio XIAO compatible boards, like XIAO RP2040
or Adafruit QtPy with a Framework 13 or Framework 16 touchpad.

Pinout

| Touchpad | Xiao |
|----------|------|
| I2C SDA  | D4   |
| I2C SCL  | D5   |
| I2C INT  | D6   |

```
# west.yml
manifest:
  remotes:
    - name: FrameworkComputer
      url-base: https://github.com/FrameworkComputer
  projects:
    - name: zephyr-hid-touchpad-module
      revision: main
      remote: FrameworkComputer
```

```
# foo.conf
CONFIG_I2C=y
CONFIG_INPUT=y
CONFIG_ZMK_HID_TOUCHPAD=y
CONFIG_ZMK_MOUSE=y
CONFIG_ZMK_POINTING=y
```

```
# foo.dtsi
&xiao_i2c {
    status = "okay";

    touchpad: touchpad@2c {
        compatible = "zmk,hid-touchpad";
        status = "okay";
        reg = <0x2c>;
        dr-gpios = <&xiao_d 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        // Instead of parsing the HID report descriptor in firmware
        // We configure the report IDs and report structure here
        // PCT3854 - Used on Framework 13
        mouse-report-id = <2>;
        inputmode-report-id = <6>;
        relative-x-off = <4>;
        relative-x-len = <2>;
        relative-y-off = <6>;
        relative-y-len = <2>;
        button-off = <3>;
        button-bit = <0>;
    };
};

/ {
    touchpad_input: touchpad_input {
        compatible = "zmk,input-listener";
        device = <&touchpad>;
    };
};
```

## Support
Note, this is not an officially supported Framework product, just provided as
reference for tinkerers, reusing Framework touchpad modules without the mainboard.

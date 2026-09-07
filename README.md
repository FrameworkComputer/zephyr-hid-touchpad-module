# HID Touchpad Driver

Zephyr driver for I2C HID touchpads (Windows Precision Touchpad / PTP and
legacy mouse mode), plus two passthrough backends for
[ZMK](https://zmk.dev) that forward the pad's raw HID reports to the host
as a second HID interface over USB and as a second HOGP service over BLE.
The host's own touchpad stack (hid-multitouch + libinput on Linux, the
PTP driver on Windows) then talks to the pad as if it were built in,
including feature-report access for vendor tooling.

The driver does **not** parse the pad's report descriptor at runtime. The
descriptor blob and the report layout facts the passthrough needs are
configured in devicetree.

## Devicetree

Binding: `zmk,hid-touchpad` (`dts/bindings/input/zmk,hid-touchpad.yml`).

| property                   | meaning                                                         |
|----------------------------|-----------------------------------------------------------------|
| `reg`                      | I2C address                                                     |
| `dr-gpios`                 | data-ready (interrupt) line, active low                         |
| `hid-descriptor-register`  | I2C register of the 30-byte I2C-HID descriptor (usually `0x20`) |
| `report-descriptor`        | the full HID report descriptor blob (`uint8-array`)             |
| `mouse-input-report-id/-size` | report ID and data size of the relative mouse report         |
| `ptp-input-report-id/-size`   | report ID and data size of the PTP multitouch report         |
| `feature-report-ids`       | every feature report the pad exposes                            |
| `feature-report-sizes`     | data size of each, parallel to the IDs (bounds the I2C GET read)|

Sizes exclude the report ID byte. The worked example for a PixArt PCT1036
pad is `app/boards/shields/daisy/daisy-touchpad.dtsi` in the Daisy
keyboard tree; the blob is ~700 bytes, so it is not repeated here.

```dts
&i2c1 {
    status = "okay";
    /* the PCT1036 supports 400 kHz; halves GET_REPORT latency */
    clock-frequency = <I2C_BITRATE_FAST>;

    touchpad: touchpad@2c {
        compatible = "zmk,hid-touchpad";
        status = "okay";
        reg = <0x2c>;
        dr-gpios = <&gpio0 15 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        hid-descriptor-register = <0x20>;
        mouse-input-report-id = <1>;
        mouse-input-report-size = <8>;
        ptp-input-report-id = <4>;
        ptp-input-report-size = <29>;
        feature-report-ids   = <2 3 5 6 7 10 11 65 66 67>;
        feature-report-sizes = <1 1 1 1 1 256 1 256 3 3>;
        report-descriptor = [ 05 01 09 02 A1 01 85 01 /* ... */ C0 ];
    };
};
```

The USB backend additionally needs a `zephyr,hid-device` node labelled
`tp_hid` for the second USB HID interface (see `daisy.overlay`).

## Kconfig

```
CONFIG_I2C=y
CONFIG_INPUT=y
CONFIG_ZMK_HID_TOUCHPAD=y        # the driver (default y when the DT node exists)
CONFIG_HID_PASSTHROUGH_USB=y     # USB backend, needs ZMK_USB (legacy or USB-next stack)
CONFIG_HID_PASSTHROUGH_BLE=y     # BLE HOGP backend, needs ZMK_BLE
```

Both passthrough backends depend on ZMK: they follow the keyboard's
endpoint selection (the pad streams only over the transport the keyboard is
using) and the BLE one notifies the central of ZMK's active profile. The
plain driver and its `hid_touchpad_register_input_cb()` callback API have
no ZMK dependency and can feed any other consumer.

Other options: `HID_PASSTHROUGH_BLE_PACE_MS` (fallback pace for PTP
notifications; normally derived from the granted connection interval),
`HID_PASSTHROUGH_BLE_FEATURE_PAD_BYTE` (default y: BLE feature GET responses
carry one trailing dummy byte, so BlueZ <= 5.87, whose uhid bridge drops the
last byte of every numbered GET_REPORT reply, still delivers the full report;
fixed hosts discard the extra byte), `HID_PASSTHROUGH_BLE_WORKQUEUE_STACK_SIZE`,
`HID_TOUCHPAD_WORKQUEUE_*`, and `HID_TOUCHPAD_INPUT_STATS` (1 Hz frame-rate /
I2C timing diagnostic, off by default).

## Using it in a west workspace

```yaml
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

## Known limitations

- Feature report **writes over BLE** must fit in one ATT write (MTU − 3
  bytes); prepared/long writes are rejected with an invalid-offset error.
  This only affects the 256-byte vendor blobs, and no known tool writes
  them over BLE (register access goes through 3-byte reports). Reads of any
  size work, including ATT long reads.
- One touchpad instance per build (asserted at compile time).

## Support
Note, this is not an officially supported Framework product, just provided as
reference for tinkerers, reusing Framework touchpad modules without the mainboard.

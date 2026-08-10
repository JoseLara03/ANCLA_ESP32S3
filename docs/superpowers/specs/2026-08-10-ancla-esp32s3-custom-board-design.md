# ANCLA_ESP32S3 — custom Zephyr board design

Date: 2026-08-10
Status: approved

## Purpose

`ANCLA_ESP32S3` is a Zephyr application targeting a custom UWB anchor PCB built
around an ESP32-S3-WROOM-1-N8R2 module. Zephyr has no board definition for this
hardware, so this spec defines a new out-of-tree board,
`innovaforce/ancla_esp32s3`, so the app can be built with
`west build -b ancla_esp32s3/esp32s3/procpu`.

This is a hardware bring-up task only: get the SoC, buses, and fixed pin
assignments correctly described in devicetree/Kconfig. It does **not** include
writing a DW3220 (Qorvo UWB transceiver) Zephyr driver — Zephyr has no in-tree
Qorvo DW3000-family driver, and writing one is a separate, larger task.

## Hardware summary

| Peripheral | Signal | GPIO |
|---|---|---|
| MAX17048 (fuel gauge, I2C) | SDA | 40 |
| | SCL | 39 |
| | ALERT | 41 |
| RGB LED (WS2812-style, addressable, single-wire) | DIN | 38 |
| DW3220 (UWB transceiver, SPI) | MISO | 13 |
| | MOSI | 11 |
| | CS | 10 |
| | SCLK | 12 |
| | WAKE_UP | 14 |
| | RESET | 21 (via NPN inverter: LOW = reset asserted, HIGH = running) |
| | IRQ | 42 |
| USB-C | D-/D+ | 19/20 (fixed silicon pins, native USB-Serial-JTAG) |
| UART0 (link to external device / RPi4, not console) | TXD0/RXD0 | 43/44 (default pins) |

## Decisions

### 1. Out-of-tree board, not an SDK checkout edit

Board files live inside this repo at `boards/innovaforce/ancla_esp32s3/`, added
to `BOARD_ROOT` from this project's `CMakeLists.txt`. This keeps the board
definition version-controlled with the firmware and immune to `west update`
overwriting/desyncing it, at the cost of one extra `list(APPEND BOARD_ROOT ...)`
line in the top-level `CMakeLists.txt`.

### 2. SoC base: `esp32s3_wroom_n8r2.dtsi`

This is Zephyr's exact dtsi for an ESP32-S3-WROOM-1 module with 8MB flash + 2MB
Quad SPI PSRAM — matches the N8R2 part with no size overrides needed. Kconfig
selects `SOC_ESP32S3_WROOM_N8R2`.

Quad PSRAM (R2) shares the flash SPI bus (GPIO26-32) and does not consume
GPIO33-37, so those remain free for other use on this module (confirmed against
the in-tree dtsi, which only touches `flash0`/`psram0` reg/size — no extra
pinmux). This matters below because the LED's I2S-based driver needs several
"unused" signal pins and GPIO33-37/47 are where they land.

### 3. SPI2 for the DW3220 — stock pinout, no custom pinctrl

GPIO 10/11/12/13 (CS/MOSI/SCLK/MISO) are exactly Zephyr's built-in
`spim2_default` pinmux macros for this SoC. The board just enables `&spi2` with
that existing pinctrl group — no new pin macros needed for the bus itself.

### 4. DW3220 control lines — `zephyr,user` GPIOs, no device node

Since there's no DW3000-family binding/driver in this Zephyr tree, RESET, IRQ,
and WAKE_UP are exposed as plain named GPIOs under a `zephyr,user` devicetree
node (the standard Zephyr idiom for board-defined signals with no driver yet):

```
zephyr,user {
    dw3000-reset-gpios  = <&gpio0 21 GPIO_ACTIVE_HIGH>;
    dw3000-irq-gpios    = <&gpio1 10 GPIO_ACTIVE_HIGH>;
    dw3000-wakeup-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
};
```

(GPIO bank math: `gpio0` covers absolute pins 0-31 directly; `gpio1` covers
32-53 at index `pin - 32`, so IRQ on GPIO42 is `&gpio1 10`.)

App code reads these via `GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), dw3000_reset_gpios)`
etc. `GPIO_ACTIVE_HIGH` here describes the physical pin Zephyr's GPIO API will
treat as electrically active — HIGH releases/runs the DW3220 per the NPN
inverter, matching the hardware note in the request. Whether a future driver
calls that state "asserted" or "released" is a driver-level naming decision,
out of scope here.

No SPI child devicetree node is created for the DW3220 itself (no binding
exists to attach); that's part of the future driver work.

### 5. MAX17048 — real devicetree node (in-tree driver exists)

Zephyr ships a `maxim,max17048` fuel-gauge driver and binding, so this one gets
wired up for real, not just as a placeholder:

```
&i2c0 {
    pinctrl-0 = <&i2c0_default>;   /* custom: SDA=GPIO40, SCL=GPIO39 */
    pinctrl-names = "default";
    clock-frequency = <I2C_BITRATE_STANDARD>;

    max17048: max17048@36 {
        compatible = "maxim,max17048";
        reg = <0x36>;
        alert-gpios = <&gpio1 9 GPIO_ACTIVE_LOW>;  /* GPIO41, open-drain active-low */
    };
};
```

I2C0 is chosen (arbitrarily, only one I2C device exists on this board) with a
custom `i2c0_default` pinctrl group for GPIO39/40 (the chip's pinmux header has
per-pin macros for every GPIO, confirmed: `I2C0_SCL_GPIO39`, `I2C0_SDA_GPIO40`).

### 6. RGB LED — WS2812-style over I2S0 bit-banging

This Zephyr version has no RMT driver for ESP32, and GPIO-bit-banged WS2812
timing is fragile on Xtensa (cache misses). The proven in-tree approach for
this exact SoC family (used by an existing vendor board) is
`worldsemi,ws2812-i2s`: the I2S0 peripheral's serial-data output line is
repurposed to shift out the WS2812 bit pattern.

The I2S0 peripheral requires MCLK/WS/BCK/I_WS/I_BCK/I_SD signals to be pinned
even though only the data-out line is physically wired to the LED. Those
"phantom" signals are routed to GPIO33-37 and 47 — confirmed free (see
decision 2) and not used by anything else on this board:

```
i2s0_default: i2s0_default {
    group1 {
        pinmux = <I2S0_MCLK_GPIO33>,
                 <I2S0_O_WS_GPIO34>,
                 <I2S0_O_BCK_GPIO35>,
                 <I2S0_I_WS_GPIO36>,
                 <I2S0_I_BCK_GPIO37>,
                 <I2S0_O_SD_GPIO38>;      /* actual LED data pin */
        output-enable;
    };
    group2 {
        pinmux = <I2S0_I_SD_GPIO47>;      /* unused input, spare pin */
        input-enable;
    };
};
```

```
&i2s0 {
    pinctrl-0 = <&i2s0_default>;
    pinctrl-names = "default";
    status = "okay";
    dmas = <&dma 3>;
    dma-names = "tx";

    rgb_led: ws2812@0 {
        compatible = "worldsemi,ws2812-i2s";
        reg = <0>;
        chain-length = <1>;
        color-mapping = <LED_COLOR_ID_GREEN LED_COLOR_ID_RED LED_COLOR_ID_BLUE>;
        reset-delay = <500>;
    };
};

&dma {
    status = "okay";
};
```

Aliased as `rgb0` for app convenience.

### 7. UART0 — enabled, not console

`&uart0` gets the stock default pinctrl (GPIO43 TX / GPIO44 RX) and
`status = "okay"`, but is **not** wired to `zephyr,console` /
`zephyr,shell-uart` — so nothing prints there and it stays free for the app to
talk to the RPi4/external device.

### 8. Console — native USB-Serial-JTAG

```
chosen {
    zephyr,console = &usb_serial;
    zephyr,shell-uart = &usb_serial;
    ...
};

&usb_serial {
    status = "okay";
};
```

No USB device-stack/CDC-ACM Kconfig needed — this peripheral presents a plain
virtual COM port out of the box and is what esptool uses for flashing on
boards without a USB-UART bridge chip, matching this hardware exactly (only
D+/D- routed to the USB-C connector).

### 9. Everything else — copied from the closest in-tree template

Partition layout (`espressif/partitions_0x0_amp.dtsi`, 8MB flash — same as
`esp32s3_devkitc`, no size suffix needed), MCUboot as default sysbuild
bootloader with no signature, `wdt0`/`trng0`/`wifi`/`esp32_bt_hci` enabled,
`timer0`-`timer3` explicitly disabled, and `HEAP_MEM_POOL_ADD_SIZE_BOARD`
(4096 procpu / 256 appcpu) are all copied verbatim from
`boards/espressif/esp32s3_devkitc/`, since these are SoC-family conventions
unrelated to this board's specific peripherals. SPI3, I2S1, I2C1, TWAI, and the
touch peripheral are omitted entirely — nothing on this board uses them.

The `appcpu` variant (dts/yaml/defconfig) is created for parity with every
other ESP32-S3 board in this Zephyr tree (the SoC's AMP architecture expects
both cluster variants to exist), but is otherwise inert — the app only ever
builds `ancla_esp32s3/esp32s3/procpu`.

## Out of scope

- A Zephyr driver/binding for the DW3220 (Qorvo UWB transceiver). This spec
  only reserves its SPI bus and control-line GPIOs.
- Any application logic (ranging, positioning, etc.) — this is board bring-up
  only.
- Using the ESP32-S3's `appcpu` core for anything (AMP dual-image build).

## File list

```
ANCLA_ESP32S3/
  CMakeLists.txt                                  (add BOARD_ROOT)
  boards/innovaforce/ancla_esp32s3/
    board.yml
    Kconfig
    Kconfig.ancla_esp32s3
    Kconfig.sysbuild
    board.cmake
    ancla_esp32s3-pinctrl.dtsi
    ancla_esp32s3_procpu.dts
    ancla_esp32s3_procpu.yaml
    ancla_esp32s3_procpu_defconfig
    ancla_esp32s3_appcpu.dts
    ancla_esp32s3_appcpu.yaml
    ancla_esp32s3_appcpu_defconfig
```

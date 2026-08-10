# DW3220 Zephyr port — vendored decadriver module

Date: 2026-08-10

## What happened

`platform/` and `dwt_uwb_driver/` had been dropped into this repo as a verbatim
copy of the **ESP-IDF/FreeRTOS** platform layer from the sibling PlatformIO
project. That code cannot build under Zephyr:

- `freertos/FreeRTOS.h`, `freertos/task.h` — do not exist in Zephyr
  (`vTaskDelay`, `pdMS_TO_TICKS`, `portENABLE_INTERRUPTS` unavailable).
- `driver/spi_master.h` — does not exist anywhere in `hal_espressif`; the
  `components/driver/spi/include` entry in its CMake include list is stale and
  points at a non-existent directory. `port.h` itself would not compile.
- `driver/gpio.h` — header is present, but `esp_driver_gpio/src/gpio.c` is only
  compiled under `CONFIG_ESPRESSIF_RMT` / `CONFIG_PM` / `CONFIG_POWEROFF`, so
  `gpio_config`/`gpio_set_level` are undefined at link in a normal build. Using
  them would also mean two owners of the same pads, since `&gpio0`/`&gpio1` are
  `status = "okay"` and driven by Zephyr's own GPIO driver.

It also carried functional bugs: `DW_RESET_PIN` hardcoded to GPIO9 (this board
uses GPIO21), reset polarity documented one way in `port.h` and implemented the
other way in `port.c`, `peripherals_init()` leaving the part held in reset, and
`wakeup_device_with_io()` toggling the CS pin with `gpio_set_level()` while the
SPI peripheral owned that pin (so the toggle did nothing).

That drop has been removed and replaced with the upstream Zephyr module.

## Upstream

[br101/zephyr-dw3000-decadriver](https://github.com/br101/zephyr-dw3000-decadriver),
vendored in-tree at `modules/dw3000-decadriver/`, registered from the top-level
`CMakeLists.txt` via `ZEPHYR_EXTRA_MODULES`.

- Vendored revision: `6208d99f933872bf024a653b0c9e8bef92349162` (2026-06-02)
- Contains Qorvo `dwt_uwb_driver` 08.02.02 plus Zephyr GPIO/SPI/DTS bindings.

In-tree rather than a `west.yml` entry, for the same reason the board definition
is in-tree (see the board design spec): version-controlled with the firmware and
immune to `west update`.

Excluded when vendoring: `.git/` and the three PDFs under `doc/` and
`boards/shields/qorvo_dws3000/doc/` (~9 MB of datasheets, not build inputs).

## Local deltas vs upstream

Keep this list current — a future upstream pull will silently drop these.

### 1. `platform/dw3000_hw.c` — reset is driven push-pull

Upstream releases RESET by reconfiguring the pin to `GPIO_INPUT`, relying on the
DW3000's internal pull-up on the `RSTn` pad. Correct for a direct connection;
wrong here. On this board the pin drives a **transistor base**, not `RSTn`, so
`GPIO_INPUT` leaves the base floating. Changed to drive the pin inactive
instead, in both `dw3000_hw_init()` and `dw3000_hw_reset()`.

Driving it is safe precisely because it is not connected to the pad — Qorvo's
"never actively drive `RSTn` high" rule does not apply to this net.

### 2. `platform/dw3000_spi.c` — dropped the deprecated CS delay argument

`SPI_CS_CONTROL_INIT(DW_INST, 0)` → `SPI_CS_CONTROL_INIT(DW_INST)`. The explicit
delay parameter is deprecated as of Zephyr 4.x (it emits a `__WARN`); the
replacement is the `spi-cs-setup-delay-ns`/`spi-cs-hold-delay-ns` DT properties,
neither of which this board needs.

### 3. `CMakeLists.txt` — `zephyr_library_named(dw3000_decadriver)`

A bare `zephyr_library()` derives the library name from the module's path
relative to `ZEPHYR_BASE`. With this module out-of-tree under `Documents/`, that
produced a ~210-character object directory and tripped
`CMAKE_OBJECT_PATH_MAX` (250) on Windows — CMake warned that
`dw3000_device.c.obj` "cannot be safely placed under this directory". Naming the
library explicitly keeps the paths short.

## Board devicetree changes

The DW3220 control lines moved off the `zephyr,user` node and onto a real
`decawave,dw3000` device node under `&spi2`, which is what the module's
`DT_INST(0, decawave_dw3000)` lookups expect. `max17048-alert-gpios` stays on
`zephyr,user` (still no binding property for it).

**CS is now a GPIO, not the SPI2 hardware CSEL.** `SPIM2_CSEL_GPIO10` was removed
from the `spim2_default` pinctrl group and replaced with
`cs-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>` on `&spi2`. Two reasons:

- `dw3000_spi_wakeup()` wakes the DW3220 from DEEPSLEEP by holding CS low for
  500 µs. Hardware CS cannot do that.
- `struct spi_cs_control` is a **union** in current Zephyr, discriminated by
  `cs_is_gpio`. `dw3000_spi.c` unconditionally dereferences `cs.gpio`, which
  would reinterpret `setup_ns`/`hold_ns` as a `gpio_dt_spec` under native CS.

`spi-max-frequency` is set to 8 MHz, matching the fast rate the ESP-IDF port used
on the bench, not the DW3000's 38 MHz ceiling.

## RESET polarity — resolved on hardware (inverting)

`reset-gpios` is `GPIO_ACTIVE_HIGH`: **GPIO high = reset asserted, GPIO low =
running**. The NPN inverter behaves as a plain inverter — GPIO high turns the
transistor on, pulling `RSTn` low.

This was ambiguous in this repo's own sources and was settled by testing.
`GPIO_ACTIVE_LOW` was tried first, following the board design spec's pin table
and the `decawave,dw3000` binding's stock active-low convention; `dwt_probe()`
failed, because the part was being held in reset. Flipping the flag fixed it.

So the **board design spec's pin table is wrong** ("21 (via NPN inverter: LOW =
reset asserted, HIGH = running)"), and the old ESP-IDF `port.h` comment was
right. The old `port.c` code matched the spec, not its own header — meaning that
port also drove reset backwards.

Two consequences to keep in mind:

- This board's `reset-gpios` polarity is inverted relative to what the
  `decawave,dw3000` binding documents. That is fine — the flag exists to
  describe exactly this — but it will look wrong to anyone comparing against
  upstream's example overlays.
- Because reset is asserted by driving the pin *high*, and released by driving it
  *low*, local delta #1 (push-pull instead of `GPIO_INPUT`) is what makes the
  released state deterministic. Do not revert it.

## Call-order footgun: `dwt_initialise()` must be the first register access

`dw->priv` — the driver's local-data pointer — is assigned **only** inside
`ull_initialise()` (`dwt_uwb_driver/dw3000/dw3000_device.c:1025`). `dwt_probe()`
does not set it. Every register access funnels through `dwt_xfer3xxx()`, which
unconditionally dereferences `LOCAL_DATA(dw)->spicrc` (line 518 on reads, 496 on
writes).

So **any** register access between `dwt_probe()` and `dwt_initialise()` is a NULL
dereference. On ESP32-S3 it shows up as:

```
<err> os:  ** FATAL EXCEPTION
<err> os:  ** CPU 0 EXCCAUSE 28 (load prohibited)
<err> os:  **  PC 0x420055a5 VADDR 0x24
```

`VADDR 0x24` is the offset of `spicrc` within `dwt_local_data_t`. Resolve the PC
with the SDK's `addr2line` to confirm:

```sh
$ZEPHYR_SDK/gnu/xtensa-espressif_esp32s3_zephyr-elf/bin/\
xtensa-espressif_esp32s3_zephyr-elf-addr2line -e build/zephyr/zephyr.elf -fpiC 0x420055a5
```

This traps `dwt_readdevid()`, `dwt_checkidlerc()` and `dwt_configure()` alike.
**Note the upstream README's usage example hits it** — it shows `dwt_probe()`
followed directly by `dwt_readdevid()`. Do not copy that snippet.

The correct order is in `src/main.c`:

1. `dw3000_hw_init()` / `dw3000_hw_reset()`
2. `dwt_probe()` — reads DEV_ID over *raw* SPI (`dw->SPI->readfromspi`, bypassing
   the register layer) and matches the driver list. Success already proves bus,
   CS and reset polarity, so there is no reason to re-read DEV_ID to check that.
3. `dwt_initialise(DWT_DW_INIT)` — assigns `dw->priv`, reads OTP.
4. Everything else.

Qorvo's stock examples poll `dwt_checkidlerc()` *before* `dwt_initialise()`, which
is not possible here for the reason above. The 2 ms delay in `dw3000_hw_reset()`
covers the INIT_RC → IDLE_RC transition instead, and IDLE_RC is asserted right
after init.

## Configuration

```
CONFIG_GPIO=y
CONFIG_SPI=y
CONFIG_DW3000=y
CONFIG_DW3000_CHIP_DW3000=y   # DW3220 is a DW3000-family part, not DW3720/QM33
CONFIG_LOG=y
```

## Verification status

- `west build -b ancla_esp32s3/esp32s3/procpu` — **clean, zero warnings.**
  All 9 module sources compile (`dw3000_hw.c`, `dw3000_spi.c`, `deca_port.c`,
  `dw3000_device.c`, `deca_interface.c`, `deca_rsl.c`, `deca_compat.c`,
  `dw3000_spi_trace.c`, `qmath.c`) and link into `zephyr.elf`.
- Generated devicetree confirmed: `cs-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>`,
  `reset-gpios = <&gpio0 21 GPIO_ACTIVE_LOW>`, `irq-gpios = <&gpio1 10>` (GPIO42),
  `wakeup-gpios = <&gpio0 14>`, and `spim2_default/group1` down to two pinmux
  entries (MISO + SCLK only).
**On hardware: the full bring-up path passes (2026-08-10).**

```
*** Booting Zephyr OS build v4.4.0-3085-gbcf31c20f01f ***
<inf> dw3000: RESET on gpio@60004000 pin 21
<inf> dw3000: WAKEUP on gpio@60004000 pin 14
<inf> dw3000: DW3000 SPI (max 8MHz)
<inf> dw3000: WAKEUP PIN
<inf> main: DW3220 device ID: 0xdeca0312
<inf> dw3000: IRQ on gpio@60004800 pin 10
<inf> main: DW3220 ready, IRQ armed
```

Confirmed by that run:

- RESET (GPIO21, `GPIO_ACTIVE_HIGH`), WAKEUP (GPIO14) and IRQ (`gpio1` pin 10 =
  GPIO42) all resolve to the intended pads — `gpio@60004000` is `gpio0`,
  `gpio@60004800` is `gpio1`.
- SPI + GPIO CS work: `dwt_probe()`, `dwt_initialise()`, IDLE_RC check and
  `dwt_readdevid()` all succeed.
- `0xDECA0312` is `DWT_DW3000_PDOA_DEV_ID` — the PDoA-capable variant, as expected
  for a DW3220. Worth remembering when configuring the PHY, since this system runs
  with STS/PDoA off.
- Reset polarity settled: `GPIO_ACTIVE_HIGH` (see the RESET section above).

Not yet exercised: TX/RX, ranging, and the IRQ actually firing (it is armed, but
nothing has generated an interrupt yet).

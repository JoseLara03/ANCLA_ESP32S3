# ANCLA_ESP32S3 — project guide

Zephyr firmware for a custom UWB **anchor** PCB: ESP32-S3-WROOM-1-N8R2 +
Qorvo DW3220 (DW3000 family), plus a MAX17048 fuel gauge and a WS2812 RGB LED.

**Start here:** `docs/handoff-2026-08-10-dw3000-port.md` — current state, build
and flash commands, and the two traps that have already cost a debug cycle each.

## Build & flash

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

`$env:ZEPHYR_BASE` is **required** — this project lives outside the
`zephyrproject` west workspace. Zephyr 4.4.x, SDK at `~/zephyr-sdk-1.0.1`.

## Layout

- `boards/innovaforce/ancla_esp32s3/` — out-of-tree board definition, added to
  `BOARD_ROOT` from the top-level `CMakeLists.txt`. In-tree deliberately, so it
  is version-controlled with the firmware and immune to `west update`.
- `modules/dw3000-decadriver/` — **vendored third-party** Zephyr module
  (br101/zephyr-dw3000-decadriver @ `6208d99`, containing Qorvo dwt_uwb_driver
  08.02.02), registered via `ZEPHYR_EXTRA_MODULES`. Carries three deliberate
  local deltas; a naive upstream re-pull will silently drop them.
- `src/main.c` — DW3220 bring-up smoke test (reset → probe → initialise →
  IDLE_RC → DEV_ID → arm IRQ).
- `docs/dw3000-zephyr-port.md` — the port reference: local deltas, the DW3000
  call-order footgun, resolved RESET polarity, verification status.
- `docs/superpowers/{specs,plans}/` — board design spec and implementation plan.

## Hard-won facts (do not re-derive)

- **`dwt_initialise()` must be the first register access after `dwt_probe()`.**
  It is what assigns `dw->priv`; anything else first — including
  `dwt_configure()` — faults with `EXCCAUSE 28 / VADDR 0x24`. The upstream
  README's own example gets this wrong.
- **RESET (GPIO21) is `GPIO_ACTIVE_HIGH`**: the NPN inverter means GPIO high =
  reset asserted. This is inverted relative to the `decawave,dw3000` binding's
  documented convention, and the board design spec's pin table originally had it
  backwards.
- **CS (GPIO10) is a GPIO, not the SPI2 hardware CSEL** — the driver holds CS low
  by hand to wake the DW3220 from DEEPSLEEP, which hardware CS cannot do.
- Pin map: SPI2 MISO 13 / MOSI 11 / SCLK 12 / CS 10; DW3220 RESET 21, WAKEUP 14,
  IRQ 42. `gpio0` = pads 0-31, `gpio1` = pads 32-53 at index `pin - 32`.

## System context

Ranging is Two-Way Ranging (TWR); distance is computed on the tag, not the
anchor. The PHY contract every node must match — **channel 5, PLEN_1024, PAC32,
code 9, 850 kbps, SFD_IEEE_4Z, STS/PDoA off** — and the 802.15.4z wire format
live in the sibling ESP-IDF anchor project at
`../../../PlatformIO/Projects/ESP32S3UWB` (see its `CLAUDE.md`). That project is
the working reference implementation for ranging; none of it has been ported
here yet.

## Repo

Local git only, on `master`, **no remote configured** — nothing is pushed
anywhere.

# Handoff — DW3220 Zephyr port, 2026-08-10

## Where things stand

The DW3220 is alive and talking over SPI under Zephyr on the `ancla_esp32s3`
board. Verified on hardware:

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

`dwt_probe()` → `dwt_initialise()` → IDLE_RC → `dwt_readdevid()` all pass, and
the IRQ is armed. `0xDECA0312` is `DWT_DW3000_PDOA_DEV_ID`, the PDoA-capable
variant — correct for a DW3220.

**Read `docs/dw3000-zephyr-port.md` before touching any of this.** It is the
reference for the port: what the ESP-IDF drop got wrong, the three local deltas
against the vendored upstream module, and two hardware facts that cost a debug
cycle each.

## Build and flash

```powershell
cd C:\Users\JoseAntonioLaraPerez\Documents\zephyr_projects\esp32\ANCLA_ESP32S3
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"

west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

`$env:ZEPHYR_BASE` is required — this project sits outside the `zephyrproject`
west workspace, so `west` cannot find Zephyr without it. Set it once per
terminal. Add `-p always` to `west build` for a clean rebuild.

Console and flashing share the one USB-C port (native USB-Serial-JTAG,
GPIO19/20); `zephyr,console` and `zephyr,shell-uart` are both on `&usb_serial`.

## Two traps that will bite again

### 1. `dwt_initialise()` must be the first register access after `dwt_probe()`

`dw->priv` is assigned *only* inside `ull_initialise()`, and every register
access dereferences it. Calling `dwt_readdevid()`, `dwt_checkidlerc()` or
**`dwt_configure()`** before `dwt_initialise()` faults:

```
<err> os:  ** CPU 0 EXCCAUSE 28 (load prohibited)
<err> os:  **  PC 0x420055a5 VADDR 0x24
```

`VADDR 0x24` is the offset of `spicrc`. **The upstream README's usage example
hits this** — do not copy it. Full detail plus the `addr2line` recipe is in
`docs/dw3000-zephyr-port.md`.

This matters immediately for the next task, since PHY configuration means
calling `dwt_configure()`.

### 2. RESET is `GPIO_ACTIVE_HIGH` — inverted vs. the upstream binding

The NPN inverter means GPIO high = reset asserted. This is the opposite of the
`decawave,dw3000` binding's documented active-low convention, so it looks wrong
next to upstream's example overlays. It is correct; it was determined on
hardware. The board design spec's pin table originally had it backwards and has
been corrected in place.

If reset polarity ever regresses, the symptom is `dwt_probe()` returning an
error, not a crash.

## Suggested next step: TX/RX loopback

The IRQ line is wired and armed but **has never actually fired** — nothing has
generated an interrupt yet. That is the one integrated path still unproven, and
proving it is the natural next task:

1. Configure the PHY. It must match every node in the system: **channel 5,
   PLEN_1024, PAC32, code 9, 850 kbps, SFD_IEEE_4Z, STS/PDoA off.** The
   authoritative copy of this contract is the sibling anchor project's
   `CLAUDE.md` (`../../../PlatformIO/Projects/ESP32S3UWB`).
2. Transmit a frame, receive it on a second node (or the sibling ESP-IDF anchor,
   which already ranges), and confirm `dwt_isr()` runs off the GPIO42 callback
   rather than by polling.
3. Watch the SPI rate: `dw3000_spi_init()` starts at the 2 MHz slow config and
   only moves to the 8 MHz `spi-max-frequency` when the driver calls
   `setfastrate`. Confirm that switch actually happens before drawing any
   conclusions about turnaround timing.

Beyond that, the larger goal is porting the ranging work from the sibling
ESP-IDF project. Its `CLAUDE.md` carries the roadmap, the 802.15.4z wire-format
contract, and the known tension between the discovery (`0xE2`/`0xE4`) and legacy
sweep (`0xE0`/`0xE1`) frame conventions. None of that has been touched here.

## Repo notes

- Local git repo on `master`, **no remote configured**. Nothing is pushed
  anywhere; back it up if that matters.
- `modules/dw3000-decadriver/` is vendored third-party code with three
  deliberate local deltas. A naive upstream re-pull will silently drop them —
  re-read the deltas section of `docs/dw3000-zephyr-port.md` first.
- The removed ESP-IDF `platform/` and `dwt_uwb_driver/` drops were never
  committed. They still exist in the sibling PlatformIO project if you need
  them for reference.
- `build*/` is gitignored.

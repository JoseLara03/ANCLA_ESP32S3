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

## Console

Native USB-JTAG (not UART0), prompt `uwb:~$ `.

```
anchor show                    active config as JSON
anchor id <0..3>               ranging id (default 0)
anchor mode <slave|gateway>    boot mode (default slave)
anchor pos <x> <y> <z>         coordinates in metres; sets pos_valid
anchor ant <tx> <rx>           antenna delays (default 16385/16385)
anchor reset                   restore defaults
kernel reboot cold             apply — every setter persists immediately,
                               but changes take effect only on reboot
```

## Layout

- `boards/innovaforce/ancla_esp32s3/` — out-of-tree board definition, added to
  `BOARD_ROOT` from the top-level `CMakeLists.txt`. In-tree deliberately, so it
  is version-controlled with the firmware and immune to `west update`.
- `modules/dw3000-decadriver/` — **vendored third-party** Zephyr module
  (br101/zephyr-dw3000-decadriver @ `6208d99`, containing Qorvo dwt_uwb_driver
  08.02.02), registered via `ZEPHYR_EXTRA_MODULES`. Carries three deliberate
  local deltas; a naive upstream re-pull will silently drop them.
- `src/main.c` — boot: load config from NVS, bring the DW3220 up, dispatch on
  mode to `uwb_slave_run()` / `uwb_gateway_run()`.
- `src/uwb_config.{c,h}` — per-anchor config (mode, id, antenna delays,
  position). Pure C, host-tested in `tests/uwb_config/`.
- `src/uwb_store.{c,h}` — the above persisted in NVS on `storage_partition`,
  via Zephyr settings, one key per field under `anchor/`.
- `src/uwb_radio.{c,h}` — DW3220 bring-up shared by both modes.
- `src/uwb_frame_802_15_4z.{c,h}` — 802.15.4z frame codec, **copied byte-for-byte**
  from `tag_testting/src/`. Keep it identical: the tag is the on-air peer, and
  divergence is a wire-format bug waiting to happen. Host-tested in
  `tests/uwb_frame/` with the tag's own suite.
- `src/uwb_dwtime.{c,h}` — the Qorvo `shared_functions` helpers the vendored
  module does not carry, plus `UUS_TO_DWT_TIME` and `FCS_LEN`.
- `src/disc_schedule.{c,h}` — per-anchor discovery response stagger. Host-tested
  in `tests/disc_schedule/`.
- `src/anchor_respond.{c,h}` — the WAVE/0xE1 and DISCOVERY/0xE4 responders.
- `src/uwb_phy.h` — the fixed PHY contract. Not runtime-configurable.
- `src/anchor_shell.c` — the `anchor` console command tree.
- `src/uwb_slave.c` — SLAVE mode: interrupt-driven SS-TWR responder plus beacon
  observe. `src/uwb_gateway.c` is still a stub until spec D.
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
- **`dwt_configciadiag()` must be called after `dwt_configure()`**, or every
  diagnostic register reads zero — which breaks the CIR power/quality the
  DISCOVERY response carries.
- **This board reports DEV_ID `0xDECA0312`, not `0xDECA0302`.** That is the
  PDoA-capable variant id in the driver's table (`DWT_DW3000_PDOA_DEV_ID`);
  `0xDECA0302` is the base DW3000. Both probe and configure fine — do not treat
  `0312` as a wrong part.
- **The shell thread is already running before `main()`'s first statement** —
  the `uwb:~$` prompt precedes the boot banner in the log. `uwb_config_get()`'s
  lazy initialiser is unsynchronised, so it is safe only because a human cannot
  type a command inside the ~26 ms before `main()` claims the config. Anything
  that adds an earlier automatic caller (a `SYS_INIT` hook, a startup command,
  a second thread) breaks that assumption and must initialise explicitly.
- **`dwt_getframelength()` and `cb_data->datalength` include the 2-byte FCS.**
  Confirmed on hardware in the sibling project (`ESP32S3UWB@9e1087b`: flen=13 for
  an 11-byte payload). Always pass `flen - FCS_LEN` to the frame module. The
  `is_*` validators tolerate the extra bytes because they test `len >=`, but
  `uwb_frame_parse_beacon()` derives its slot count from the length — an
  unsubtracted FCS turns a 12-slot beacon into 13, with the FCS read as the
  thirteenth slot. It fails silently and the output looks plausible.
- **Never poll `DWT_INT_CIADONE_BIT_MASK` after an RX event.** `dwt_isr()` clears
  `SYS_STATUS_ALL_RX_GOOD` — which includes CIADONE — *before* it calls
  `cbRxOk` (`dw3000_device.c:4764` then `:4791`). Waiting on it hangs until the
  next frame. Test `cb_data->status` instead: it is the pre-clear snapshot
  (`:4602`). The nRF5 `anchor_read_cir()` polls it and cannot be transcribed.
- **Keep `TXFRS` out of the enabled interrupt mask.** `dwt_setinterrupt(DWT_INT_RX,
  0, DWT_ENABLE_INT)` is deliberate: `anchor_respond.c`'s `tx_delayed()` polls for
  transmit completion, and enabling the TX interrupt would let the ISR clear
  TXFRS first and hang that poll — the same failure as CIADONE, on the TX side.
- **br101 runs `dwt_isr()` from the system workqueue, not the GPIO handler**
  (`platform/dw3000_hw.c:71-82`). DW3000 callbacks are therefore in thread
  context and may do SPI. That is what makes reading CIA diagnostics inside
  `cbRxOk` legal at all.
- **`UUS_TO_DWT_TIME` is 65536 here.** The nRF5 source says 63898. Both peers on
  this network — the tag and `ESP32S3UWB` — use 65536, and it scales the
  delayed-TX turnaround the two ends must agree on.
- **Anchor short address is `0x0001 + anchor_id`.** The MAC contract reserves
  `0x0000` for the gateway, so the console's 0-based id cannot go on the wire
  directly. The tag reports anchors as 1..4 while `anchor show` says 0..3; both
  are printed by `anchor show` and the boot banner.
- **The nRF5 anchor's 2000 uus delayed-TX turnaround does not survive the port
  to this Zephyr stack — it fails 100% of the time, not occasionally.**
  Measured on the bench with a cycle-counter margin diagnostic in
  `tx_delayed()` (`src/anchor_respond.c`): the round trip from RX
  (`dwt_readrxdata`/`dwt_readrxtimestamp`/`dwt_readdiagnostics`) through the
  delayed-TX setup writes consumes **~1700 uus** before `dwt_starttx()` is even
  called — this port's vendored driver wraps every register access through an
  `ioctl`-style dispatch plus a bit-banged CS pin (`CS (GPIO10) is a GPIO`
  fact above), which costs far more per call than the bare-metal SPI path the
  nRF5/ESP32S3UWB budget was sized for. `DISC_BASE_UUS`
  (`src/disc_schedule.h`) and `POLL_RX_TO_RESP_TX_DLY_UUS`
  (`src/anchor_respond.c`) are both **6000** as a result. Tried dropping to
  2400 after downgrading the per-frame `LOG_INF` calls in this path to
  `LOG_DBG` (compiled out at this module's `LOG_LEVEL_INF`) — still failed
  100% of the time. **Logging was never the dominant cost** on this stack;
  don't re-litigate that guess.
- **`read_cir()`'s `dwt_readdiagnostics()` call is ~944 uus of the ~1700 uus
  RX-side overhead above — by far the single largest piece.** Traced to
  `uwb_radio.c:76`'s `dwt_configciadiag(DW_CIA_DIAG_LOG_ALL)`: in the
  non-double-buffer path (`dw3000_device.c` `ull_readdiagnostics()`,
  `default:` case), `DW_CIA_DIAG_LOG_ALL` triggers **two separate ~108-byte
  SPI burst reads** (216 bytes total), and critically, `ipatovPower` /
  `ipatovAccumCount` — the only two fields `read_cir()` uses — are populated
  **only** when `LOG_ALL` is set; any cheaper `cia_diagnostic` level leaves
  them zero. `dwt_readdiagnostics_acc()` does the identical two-burst read
  internally regardless of accumulator index, so there is no lighter public
  API for just these two fields — the vendored driver's public surface
  (`deca_device_api.h`) exposes no direct/targeted register read. Getting
  non-zero CIR power/quality through this driver's public API costs ~944 uus,
  full stop, unless the vendored module itself is changed (out of scope; see
  `modules/dw3000-decadriver/` above).
  **Open decision, not yet made:** `cir_power`/`cir_quality` are
  informational only — the tag ranks anchors by them, it does not accept or
  reject a range on them (`anchor_respond.h`'s own doc comment). Skipping the
  CIR read entirely in the DISCOVERY hot path (always report zero) would
  recover the ~944 uus, at the cost of the tag's anchor-selection signal when
  several anchors are in range. Not implemented pending a decision — see
  `read_cir()` in `src/uwb_slave.c` if/when this gets revisited.
- **An unbounded wait for TXFRS can hang the whole console.**
  `tx_delayed()`'s post-`dwt_starttx()` completion poll used to spin forever
  on `DWT_INT_TXFRS_BIT_MASK`. `dwt_starttx()`'s own internal HPDWARN deadline
  check (`dw3000_device.c` `ull_starttx()`) can race a borderline-late
  delayed TX and still report `DWT_SUCCESS` for a transmission that never
  actually completes — hit this for real on the bench while testing a too-
  tight `DISC_BASE_UUS`. TXFRS then never sets, and the spin freezes the main
  thread (priority 0) permanently, killing the shell along with it. Fixed:
  `uwb_wait_for_sysstatus_lo()` (`src/uwb_dwtime.{c,h}`) now takes a
  `timeout_ms` and returns `bool`; `tx_delayed()` treats a timeout like any
  other TX failure. **The timeout bound must exceed the worst-case scheduled
  delay, not just a frame's airtime** — `dwt_starttx()` returns almost
  immediately after arming a delayed TX, the transmission itself happens only
  once the full scheduled delay elapses. An earlier attempt at this bound
  (10 ms) was shorter than `disc_resp_delay_uus(2)` and `(3)` (13 ms / 16.5 ms
  at `DISC_BASE_UUS=6000`), so it force-cancelled every DISCOVERY response for
  anchor ids ≥ 2 before they'd had a chance to fire. `TX_COMPLETE_TIMEOUT_MS`
  (`src/anchor_respond.c`) is now 25 — comfortably past the id-3 worst case
  plus airtime margin. Revisit this constant if the delay budgets above are
  ever tuned further.
- **Ranging confirmed on the bench via an external DWM3001CDK sniffer**, not
  the tag (its serial output is deliberately silent to keep BLE comms
  overhead low). With a real gateway + a second fw-cre SLAVE device + the
  tag, the sniffer showed this anchor (short address `0x0002`) answering both
  paths correctly: legacy WAVE/VEWA with the configured position encoded
  (`x=1.0,y=0.0` as IEEE-754 bytes `00 00 80 3F` / `00 00 00 00`), and
  DISCOVERY/RANGE-RESPONSE (`0xE2`→`0xE4`) with `src=0x0002`. This only
  confirms the RF exchange is correct, not that the tag actually resolves a
  distance from it (no visibility into that without the tag's BLE output).
  Also observed on the same air, unrelated to this anchor: the other fw-cre
  SLAVE answering DISCOVERY with `src=0x0000` — a real protocol violation
  (collides with the gateway's reserved address) on that rig, not this one;
  and a `0xE6`/`0xE7`/`0xE8` JOIN/GRANT handshake between the gateway and
  another device — spec-D TDMA provisioning traffic this project doesn't
  implement.
- **A temporary cycle-counter profiling block is still in `uwb_slave.c`**
  (`k_cycle_get_32()` checkpoints around `read_cir()`/`dwt_readrxdata()`/
  `uwb_get_rx_timestamp_u64()`, logged as `"prof us: cir=... readdata=...
  readts=..."`). It produced the `read_cir()` finding above. Remove once the
  CIR-read decision above is made and no further RX-side profiling is
  needed.

## System context

Ranging is Two-Way Ranging (TWR); distance is computed on the tag, not the
anchor. The PHY contract every node must match — **channel 5, PLEN_1024, PAC32,
code 9, 850 kbps, SFD_IEEE_4Z, STS/PDoA off** — and the 802.15.4z wire format
live in the sibling ESP-IDF anchor project at
`../../../PlatformIO/Projects/ESP32S3UWB` (see its `CLAUDE.md`). That project is
the working reference implementation for ranging; the SLAVE responder here
(`src/anchor_respond.c`, `src/uwb_slave.c`) has now ported the WAVE and
DISCOVERY response paths from it, but the gateway/beacon side is still to come.
Both response paths are bench-confirmed correct over the air (sniffer capture,
see the "Ranging confirmed on the bench" hard-won fact above) after fixing the
delayed-TX turnaround budget and an unbounded-wait hang; the tag's actual
distance computation is not independently observable from this project.

## Host tests

Plain gcc, no Zephyr, following the sibling projects' pattern:

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe          # PASSED, exits 0

gcc -Wall -Wextra -o tests/uwb_frame/test_uwb_frame.exe tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
./tests/uwb_frame/test_uwb_frame.exe            # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/disc_schedule/test_disc_schedule.exe tests/disc_schedule/test_disc_schedule.c src/disc_schedule.c
./tests/disc_schedule/test_disc_schedule.exe    # PASSED, exits 0
```

## Repo

Local git only, on `master`, **no remote configured** — nothing is pushed
anywhere.

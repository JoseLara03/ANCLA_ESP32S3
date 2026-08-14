# ANCLA_ESP32S3 — project guide

Zephyr firmware for a custom UWB **anchor** PCB: ESP32-S3-WROOM-1-N8R2 +
Qorvo DW3220 (DW3000 family), plus a MAX17048 fuel gauge and a WS2812 RGB LED.

**Start here:** `docs/handoff-2026-08-10-dw3000-port.md` — current state, build
and flash commands, and the two traps that have already cost a debug cycle each.

## Status: the chain works end to end (2026-08-13)

Bench-verified with 3 anchors + 1 gateway + 1 tag in a ~1 m layout: a tag joins,
discovers three anchors, ranges them, solves a 2D fix, reports it as `0xEA`, and
the gateway republishes it to the customer platform over MQTT. Position fixes
are visible on the platform.

**The numbers it produces are not yet trustworthy.** Two things below must land
before any position is worth reading.

## URGENT next work

### 1. Antenna delay calibration

Every unit still runs the factory seed `ant_tx = ant_rx = 16385`
(`UWB_ANT_DELAY_DEFAULT`), which has never been trimmed on this hardware.

Measured effect: solving over a **1 m** anchor geometry produced residuals of
**1.9–2.0 m** in one session and **0.48–0.73 m** in another. A residual larger
than the array itself means the three ranges disagree wildly; the reported
`(x, y)` wandered ~0.7 m between consecutive fixes with nothing moving.

Until this is done, **no coordinate this system emits means anything**, and no
amount of solver or geometry work will change that.

**The calibration procedure itself is now fully implemented, built, and
code-reviewed** — branch `cal/antenna-delay`, four completed tasks: a pure-C
solver host-tested against the tag's own vectors (`src/cal_math.{c,h}`,
`src/cal_solve.{c,h}`), a separate calibration firmware image that is an
ordinary WAVE responder until told otherwise (`src/cal_run.{c,h}`, `cal.conf`),
an SS-TWR initiator so an anchor can poll a peer from the console
(`src/cal_initiator.{c,h}`), and the `cal ref`/`cal peer` shell commands
(`src/cal_shell.c`) that solve, apply hot, and persist. The full runnable
procedure — DWM3001CDK prerequisites, physical setup, per-anchor `cal ref`,
the `cal peer` cross-check and its acceptance threshold, and troubleshooting —
is written up in `docs/antenna-delay-calibration.md`.

**What has not happened yet: end-to-end hardware verification, on any board.**
The code was written and builds cleanly, but no board has actually been
flashed and run through this procedure. The outstanding hardware gates, in the
order a human would hit them running `docs/antenna-delay-calibration.md`:

- The cal image is a working WAVE responder on real hardware (one board,
  sniffer-confirmed against `0xE0`/`0xE1`).
- Two cal-image anchors range each other with a stable (if inaccurate) mean
  across repeated `cal peer` runs, and responder duty survives a batch.
- `cal ref` against the prepared DWM3001CDK reference node converges to
  `|error_mm| < 15` on a second run, survives `kernel reboot cold`, and is
  then repeated for all three anchors.
- The `cal peer` cross-check acceptance test: `|error_mm| < 30` on every
  anchor pair, on pairs never used to calibrate anything.
- The actual point of the branch: with all three anchors calibrated and
  reflashed to the **production** image, a tag's `0xEA` `residual` collapses
  from the 0.48–2.0 m recorded above to under ~0.1 m, with `(x, y)` stable
  between consecutive fixes.

None of these have been run. `docs/antenna-delay-calibration.md` is the
document to execute them from; until they are done, the residuals recorded at
the top of this section are still the current, unresolved measurement.

- Console hook already exists: `anchor ant <tx> <rx>`, persisted per board.
- **Only the sum `ant_tx + ant_rx` is observable — trim TX and pin RX.**
  `RTD_resp` as a *number* is `D + ant_delay_tx` and independent of
  `RX_ANT_DLY`, but the delayed TX time is derived *from* `poll_rx_ts`
  (`anchor_respond.c:138`), which the hardware has already shifted by
  `RX_ANT_DLY`. Raising `ant_delay_rx` by a tick therefore makes the anchor
  physically transmit a tick earlier, moving the initiator's result by exactly
  as much as raising `ant_delay_tx` does by growing the reported turnaround.
  Both are half a tick of range per unit. So putting the whole correction into
  TX is *equivalent*, not merely conventional — and `ant_delay_rx` is **not** a
  free parameter: change it after calibration and you have decalibrated the
  board. An earlier version of this bullet said RX "drops out of the SS-TWR
  result", which is the right conclusion by the wrong route. Full derivation in
  `docs/superpowers/specs/2026-08-13-antenna-delay-calibration-design.md` §3.1.
- With 3 anchors solving in 2D there is one degree of redundancy, so `residual`
  in the `0xEA` frame is a genuine (if weak) quality signal — it is the metric
  to watch collapse as the delays are trimmed.

### 2. Anchor auto-positioning

Anchor coordinates are hand-entered with `anchor pos` and are **silently wrong**
when unset: `position_valid` gates only the GATEWAY's beaconing, so a SLAVE with
no position happily answers ranging polls reporting **(0, 0)**. Three anchors all
claiming the origin yields a confidently meaningless fix with no error anywhere.
This has already cost a bench session — after an `anchor id` swap the
coordinates did *not* follow the ids, leaving two live anchors on the same
baseline and the apex coordinate stranded on a third board.

The anchors payload published retained to `uwb/anchor/setup/<zone>` is also
**still the stub** (`ANC-LOBBY-001..004` at the corners of a 2 m square, only
`-001` carrying a real lat/long — see `pos_json.c`). The platform therefore draws
the stub geometry, not the deployment. Auto-positioning should feed this payload
too, so the map and the solver agree by construction.

Approach to evaluate: **DS-TWR between anchors only.** Anchors are static and can
afford the extra exchange for precision, then the inter-anchor geometry is
solved. This is Phase 5 of the sibling project's roadmap and is deliberately
distinct from the tag-facing SS-TWR path.

## Build & flash

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

`$env:ZEPHYR_BASE` is **required** — this project lives outside the
`zephyrproject` west workspace. Zephyr 4.4.x, SDK at `~/zephyr-sdk-1.0.1`.

There is also a separate **calibration image**, selected with an
`EXTRA_CONF_FILE` overlay rather than a runtime mode:

```powershell
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
west flash
```

`cal.conf` sets `CONFIG_ANCLA_CAL_MODE=y` and turns off WiFi/MQTT/networking
entirely. The resulting image is an ordinary WAVE responder that can be told,
from the console, to become a temporary SS-TWR initiator for one batch of
antenna-delay calibration exchanges — see `docs/antenna-delay-calibration.md`
for the full procedure. Never flash this image to a deployed anchor: a board
transmitting unsolicited polls would collide with tag ranging traffic. The
production build (no `EXTRA_CONF_FILE`) compiles none of the `cal_*.c` files.

## Console

Native USB-JTAG (not UART0), prompt `uwb:~$ `.

```
anchor show                    active config as JSON
anchor id <0..3>               ranging id (default 0)
anchor mode <slave|gateway>    boot mode (default slave). GATEWAY refuses to
                               beacon unless `anchor pos` has been set.
anchor pos <x> <y> <z>         coordinates in metres; sets pos_valid
anchor ant <tx> <rx>           antenna delays (default 16385/16385)
anchor reset                   restore defaults
kernel reboot cold             apply — every setter persists immediately,
                               but changes take effect only on reboot
net show                       network config and live state as JSON
net ssid <ssid>                WiFi SSID
net pass <psk>                 WiFi passphrase (8..63) — NOT the MQTT password
net broker <host> [port]       MQTT broker; port defaults to 1883
net user <username>            MQTT username
net mqttpass <password>        MQTT password — NOT the WiFi passphrase
net reset                      restore network defaults
```

The calibration image (`CONFIG_ANCLA_CAL_MODE`, see "Build & flash") adds a
`cal` tree that exists **only** in that build, not in production:

```
cal ref <mm>                   calibrate ant_delay_tx against the reference
                                node at a known distance; applies hot and
                                persists to NVS
cal peer <id> <mm>              range anchor <id> at a known distance;
                                reports only, never persists — the cross-check
```

See `docs/antenna-delay-calibration.md` for the full procedure and acceptance
thresholds.

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
  module does not carry, plus `UUS_TO_DWT_TIME`, `FCS_LEN`, the `UUS_TO_HI32`
  macro, the TX-timestamp helper (`uwb_get_tx_timestamp_u64()`), and the
  bounded `uwb_wait_for_sysstatus_lo()` TXFRS wait.
- `src/disc_schedule.{c,h}` — per-anchor discovery response stagger. Host-tested
  in `tests/disc_schedule/`.
- `src/uwb_mac.h` — the single source of truth for the superframe constants
  (`T_SUPERFRAME_UUS`, `BEACON_OCCUPANCY_UUS`, `BEACON_GUARD_UUS`) that the
  gateway schedules the beacon from and the slaves predict it against; no
  header, no code file.
- `src/gw_core.{c,h}` — CFP seat table, leases and the tag address pool. Pure
  C, ported unchanged from the nRF5 gateway, host-tested in `tests/gw_core/`.
- `src/beacon_guard.{c,h}` — predicts the next beacon and refuses any delayed
  TX that would land on it. Pure C, host-tested in `tests/beacon_guard/`.
- `src/anchor_respond.{c,h}` — the WAVE/0xE1 and DISCOVERY/0xE4 responders.
- `src/uwb_phy.h` — the fixed PHY contract. Not runtime-configurable.
- `src/anchor_shell.c` — the `anchor` console command tree.
- `src/uwb_slave.c` — SLAVE mode: interrupt-driven SS-TWR responder, beacon
  observe, and beacon-collision TX suppression.
- `src/uwb_gateway.c` — GATEWAY mode: TDMA beacon plus the CAP seat protocol.
  MAC-only for *ranging*; it does not answer ranging polls. It does decode
  `0xEA` POS frames and hand them to `pos_sink`.
- `src/pos_sink.{c,h}` — consumes decoded tag position fixes. Logs one JSON line
  per fix (the only place `residual`/`batt` stay visible) and hands the fix to
  `net_uplink` through a bounded queue.
- `src/pos_json.{c,h}` — MQTT payload formatting. Pure C, host-tested in
  `tests/pos_json/`. The position payload is a **fixed contract** with the
  downstream consumer: `{"Tid":<decimal>,"x":...,"y":...,"z":0}` — `Tid` is
  `fix->src_addr` as a **plain decimal number** (not hex, not a string; e.g.
  `0x1234` → `4660`), `z` is the integer `0`, there is no `zoneName` (the
  consumer gets the zone from the anchors topic), and the diagnostic fields
  are deliberately absent. The anchors stub publishes zone `"852541"` and
  four named anchors (`ANC-LOBBY-001..004`) at the corners of a 2 m × 2 m
  square; only `ANC-LOBBY-001` is the axis/reference anchor and carries a
  real (building-level) lat/long — the rest are local-only placeholders.
- `src/net_config.{c,h}` — WiFi and MQTT settings. Pure C, host-tested in
  `tests/net_config/`. Explicitly initialised from `main()`, **not** lazily like
  `uwb_config_get()`, because `net_uplink` is a second thread.
- `src/net_store.{c,h}` — the above persisted under the `net/` settings subtree.
- `src/net_shell.c` — the `net` console command tree.
- `src/net_uplink.{c,h}` — the WiFi + MQTT uplink thread and the bounded fix
  queue. GATEWAY mode only.
- `src/cal_math.{c,h}` — the pure-C antenna-delay solver, copied verbatim from
  the tag (`tag_testting/src/`), same rule as `uwb_frame_802_15_4z.c`: keep it
  byte-identical so this stays a copy the tag and the anchor share rather than
  a fork that drifts.
- `src/cal_solve.{c,h}` — the one piece of arithmetic this project adds on top
  of `cal_math.c`: converts a solved combined antenna delay back into a
  TX-only value, holding `ant_delay_rx` fixed. Host-tested in
  `tests/cal_solve/`.
- `src/cal_initiator.{c,h}` — one SS-TWR exchange with this board as the
  initiator, the role the tag normally plays. Calibration image only.
- `src/cal_run.{c,h}` — CAL mode's main loop: an ordinary WAVE responder that
  can be told, from the console, to become a temporary SS-TWR initiator for
  one batch of exchanges.
- `src/cal_shell.c` — the `cal` console command tree (`cal ref`, `cal peer`).
  Calibration image only.
- `Kconfig` — the project's own `Kconfig`, sourcing `Kconfig.zephyr` last;
  defines `CONFIG_ANCLA_CAL_MODE`.
- `cal.conf` — the `EXTRA_CONF_FILE` overlay that builds the calibration image
  instead of production (see "Build & flash").
- `docs/antenna-delay-calibration.md` — the operator procedure for the above:
  DWM3001CDK prerequisites, physical setup, `cal ref`, the `cal peer`
  cross-check and its acceptance threshold, troubleshooting.
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
  unsubtracted FCS turns an 11-slot beacon into 12, with the FCS read as the
  twelfth slot. It fails silently and the output looks plausible.
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
- **The SPI bus defaults to 2 MHz and nothing switches it — you must call
  `dw3000_spi_speed_fast()` yourself.** `dw3000_spi_init()` selects
  `spi_cfgs[0]` (2 MHz); `spi_cfgs[1]` carries the DTS rate and is never
  selected unless something calls the switch. `.setfastrate` is wired into the
  driver vtable (`platform/deca_port.c:37`) but its only call sites are in a
  `static init()` wrapper that `dwt_initialise()` does not dispatch to — it
  goes to `ull_initialise` (`dw3000_device.c:9371`). The module's boot log
  prints `spi_cfgs[1].frequency`, so it reports the DTS rate whether or not
  that rate is in use: **that log line is not evidence.** `uwb_radio.c` makes
  the call after `dwt_checkidlerc()`, which is the earliest legal point — the
  part requires ≤7 MHz until it leaves INIT_RC (`deca_device_api.h:2381`,
  `:2601`).
- **The RX-side overhead was the 2 MHz bus, not the driver.** An earlier
  entry here blamed the vendored driver's ioctl-style dispatch and bit-banged
  CS for ~1700 uus between RX and `dwt_starttx()`, and concluded a 2000 uus
  turnaround was unreachable on this stack. It was reachable; the bus was
  running at a quarter of the configured rate. `read_cir()`'s
  `DW_CIA_DIAG_LOG_ALL` read is two ~108-byte bursts = 1728 bits, which is
  864 us of pure clock time at 2 MHz against ~944 us measured — the transfer
  was essentially all of it. The corollary also stands: logging was never the
  dominant cost, so do not re-litigate that either.
- **Maximum usable SPI rate here is 26.67 MHz (80/3).** The ESP32-S3 only
  produces 80 MHz/N, so the ladder is 20 / 26.67 / 40 with nothing between,
  and 40 overruns the DW3000's ~38 MHz ceiling. Request `26670000` in the DTS,
  never `26000000`: the HAL picks the highest rate **not exceeding** the
  request, so asking for 26 silently yields 20. GPIO 11/12/13 are the SPI2
  IO_MUX pins (FSPID/FSPICLK/FSPIQ), so there is no GPIO-matrix penalty at
  this rate. The module README's `spi-max-frequency = <32000000>` is the
  nRF52840 SPIM3 limit and does not transfer.
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
  at the then-current `DISC_BASE_UUS=6000`), so it force-cancelled every
  DISCOVERY response for anchor ids ≥ 2 before they'd had a chance to fire.
  At HEAD, `DISC_BASE_UUS` is 2000 (`src/disc_schedule.h`, dropped after the
  SPI bus moved to fast rate), so the id-3 worst case is
  `disc_resp_delay_uus(3) = 2000 + 3*3500 = 12500` uus (~12.5 ms), and
  `TX_COMPLETE_TIMEOUT_MS` (`src/anchor_respond.c`) is 18 — comfortably past
  that plus airtime margin. The gateway has its own separate constant of the
  same name in `src/uwb_gateway.c` (currently 11) — the two are unrelated and
  must not be confused. See the next bullet for why 11 and not 5. Revisit
  either if the relevant delay budgets are tuned further.
- **The gateway's `TX_COMPLETE_TIMEOUT_MS` is shared by two delayed-TX call
  sites with different worst cases — size it against the larger one.**
  `tx_beacon()`'s delayed beacon and `send_grant()`'s GRANT both wait on the
  same constant after `dwt_starttx()`. An earlier value (5 ms) was derived
  only from `send_grant()`'s budget (`RX_TO_TX_DLY_UUS` ≈ 2.05 ms + airtime)
  and treated `BEACON_ARM_MARGIN_UUS` as an upper ceiling this timeout had to
  stay under. That is backwards for the beacon: the main loop only breaks out
  of RX-servicing and calls `tx_beacon()` once `to_beacon <=
  BEACON_ARM_MARGIN_UUS`, so the delayed beacon can be armed with nearly the
  full ~5.13 ms margin still to run before it fires —
  `BEACON_ARM_MARGIN_UUS` is the *lower bound* this constant must clear, not
  an upper one it must stay under. With 5 ms, every delayed beacon timed out
  on hardware (`"beacon started but TXFRS never completed"` on every beacon
  after the first, immediate one) while the CAP JOIN/GRANT path kept working
  underneath it, since JOIN/GRANT traffic doesn't depend on the beacon
  actually firing. Now 11 ms
  (`ceil(BEACON_ARM_MARGIN_UUS * 1.0256/1000) + 5`), confirmed on hardware: no
  more TXFRS timeouts, and a slave observes the beacon's `counter` field
  incrementing every superframe on the sniffer.
- **`dwt_readsystimestamphi32()` wraps every ~17.2 s.** hi32 counts 256 DTU
  ≈ 4.006 ns per tick, so 2³² ticks is 17.2 seconds. Every comparison between
  two hi32 values must be signed-difference arithmetic — `(int32_t)(a - b)` —
  which is correct for any interval under ~8.6 s. A plain unsigned compare is
  wrong across the wrap and the failure is rare, timing-dependent and looks
  like a radio fault. `beacon_guard.c` does this correctly and
  `tests/beacon_guard/` covers both directions across the boundary.
- **POS (`0xEA`) is not gated on lease state.** The gateway publishes a fix from
  any tag, including one whose seat has expired. Telemetry should not depend on
  MAC bookkeeping, and a silently dropped fix is undebuggable from the broker.
- **The gateway is MAC-only and holds two reserved values at once.** It uses
  short address `0x0000` per the contract and consumes no `anchor_id`, so the
  four ranging slaves take ids 0..3 (`0x0001`–`0x0004`). The deployment is
  therefore **five boards**, not four. Three ranging anchors would satisfy the
  tag's solver (`pos_solver.c:10` accepts n ≥ 3) but only as an exact solve
  with no averaging; the fourth makes it overdetermined.
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
  another device — spec-D TDMA provisioning traffic this project didn't
  implement at the time (see the next bullet: it does now).
- **This project's own GATEWAY mode (beacon + CAP seats) confirmed on the
  bench**, after fixing the `TX_COMPLETE_TIMEOUT_MS` regression above.
  Sniffer capture with this gateway and a tag showed periodic `0xE5` beacon
  frames (src `0x0000`) with a monotonically incrementing `counter`, and two
  tags completing JOIN→GRANT (`0xE6`→`0xE7`): `GRANT addr=0x0100 slot=0
  tier=2 lease=50` then, after the first seat's 10 s lease (`GW_LEASE_SF`=50
  superframes) expired with no observed KEEPALIVE, `GRANT addr=0x0101 slot=0
  tier=1 lease=50` reusing the freed slot. Both tags then ranged against the
  SLAVE anchors using their newly granted short addresses as source, with no
  gateway involvement in that traffic — confirming the MAC-only contract
  holds under real JOIN/GRANT load, not just in isolation. KEEPALIVE and
  RELEASE are implemented (`src/gw_core.c`) but not yet independently
  observed on the bench; lease expiry substitutes for RELEASE above.
- **`CONFIG_NET_TC_THREAD_PREEMPTIVE=y` is load-bearing, not tuning.** The
  `NET_TC_THREAD_TYPE` choice has no explicit default, so it resolves to its
  first entry, `NET_TC_THREAD_COOPERATIVE`, at base priority 0 — i.e.
  `K_PRIO_COOP(0)`, the *same* priority `main()` is promoted to in GATEWAY mode.
  A cooperative thread cannot be preempted, so an RX burst runs to completion
  and can overrun `BEACON_ARM_MARGIN_UUS` (5 ms). Removing this line does not
  fail to build and does not fail immediately — it makes the beacon
  intermittently late under network load.
- **The GATEWAY loop runs at `K_PRIO_COOP(0)`, so every busy-wait on that path
  must be bounded.** No lower-priority thread — including the shell — can run
  while it spins. An unbounded TXFRS wait already froze this board once at
  priority 0; cooperatively it is unrecoverable by construction.
- **WiFi is not on its own core and cannot be.** `WIFI_ESP32` is
  `depends on !SMP`, and the AMP alternative cannot host the network stack
  because the blobs bind to procpu. Isolation is by thread priority: the
  gateway loop cooperative, the WiFi blob tasks preemptible at ≤7
  (`ESP32_WIFI_MAX_THREAD_PRIORITY`), and `net_uplink` at 10.
- **`WIFI_ESP32` selects `MBEDTLS` and `PSA_CRYPTO` regardless of MQTT.** They
  are needed for WPA2 supplicant crypto, so mbedTLS is linked even with a
  plain-TCP broker. Adding MQTT TLS later therefore costs the TLS heap arena, a
  CA certificate and SNTP — not the library itself.
- **There are two passwords and they are not interchangeable.** `net pass` is
  the WiFi PSK (NVS `net/psk`, JSON `"psk"`); `net mqttpass` is the MQTT
  password (NVS `net/mqttpass`, JSON `"mqttpass"`). Command, key and JSON field
  agree in each case, deliberately.
- **Driving the external PA requires `dwt_setfinegraintxseq(0)` *before*
  `dwt_setlnapamode()`.** Fine grain TX sequencing is ON by default and the
  Qorvo API forbids it while an external PA is enabled (note on
  `dwt_setlnapamode()` in `deca_device_api.h`); left on, EXTTXE does not hold
  the QM14070 asserted across the frame. `uwb_radio.c` does both, in that order.
  The board has a PA but **no LNA** — the `DWT_LNA_ENABLE` bit is therefore
  cosmetic here.
- **MQTT topics are zone-scoped and composed from `POS_JSON_ZONE_NAME`**, not
  written as literals: `uwb/anchor/setup/<zone>` (retained, QoS 1, published
  once per connect) and `uwb/response/position/<zone>` (QoS 0, one per fix).
  The zone id lives only in `pos_json.h`, so a topic can never disagree with the
  payload published on it.
- **A board that transmits exactly one frame per boot and then goes silent is a
  wedged DW3220, not a firmware bug.** Diagnosed the long way once: it happened
  identically in SLAVE *and* GATEWAY mode — two entirely different loops — with
  a **live shell**, so the main thread was not spinning. In GATEWAY mode every
  beacon carried `seq = 0` and `frame_counter = 0`, proving the loop never
  reached its second iteration: `uwb_gateway_run()`'s inner loop only exits when
  `dwt_readsystimestamphi32()` says the next beacon is due, so a frozen chip
  clock traps it forever. Prime physical suspect is the supply sagging under the
  PA's draw at the first TX, since everything works right up to that instant.
  Swapping the board fixed it.
- **To tell a firmware fault from a board fault, swap `anchor id` between a
  working unit and a suspect one.** Two console commands, no rebuild. If the
  symptom follows the *id*, it is timing or configuration; if it follows the
  *board*, it is hardware. This is what exonerated `DISC_BASE_UUS = 2000` after
  two captures had implicated it.
- **The tag needs three anchors and fails silently with two.**
  `UWB_NET_MIN_ANCHORS = 3` (tag side). Below it the tag oscillates
  DISCOVER→RANGING→DISCOVER forever and never emits `0xEA` — visible in a
  sniffer capture as alternating `0xE2`/`0xE4` and `0xE0`/`0xE1` bursts with no
  POS frame. A gateway does **not** count: it is MAC-only for ranging and never
  answers a poll.
- **Reading sniffer captures: `frame_seq_nb` is consumed at *build* time**, in
  `anchor_respond_discovery()` before both the beacon-guard check and
  `tx_delayed()`. So a gap in an anchor's sequence numbers means "built but
  never reached the sniffer" (suppressed, TX-failed, or lost in the air), while
  1:1 continuity against the tag's DISCOVERY broadcasts proves the responder
  answered every single one. That distinction is what separates an RF problem
  from a firmware problem without touching the boards. Note the sniffer's
  `RX[n]` length **excludes** the FCS, unlike `dwt_getframelength()`.
- **The ANCLA boards transmit well below free-space expectation, and this is
  unresolved.** Measured with the sniffer 0.5 m from the gateway: beacons at
  −84 dBm where ~−57 dBm is predicted, while the tag at ~3 m arrived at
  −74.5 dBm against ~−72.5 dBm predicted — i.e. the tag matches physics and the
  ANCLA boards are ~25 dB down. That ceiling is why early range tests died at
  2–3 m. The fine-grain TX fix above was applied but a controlled before/after
  (same positions, firmware toggled) was **never run**, so its contribution is
  unknown. Re-measure before trusting any range budget.
- **The DWM3001CDK reference node used for antenna-delay calibration must run
  `POLL_RX_TO_RESP_TX_DLY_UUS = 2000`, not the Qorvo `ss_twr_responder`
  example's stock 450.** At PLEN_1024 the preamble alone takes ~1.05 ms, so a
  450 uus turnaround would require the response to leave before the poll's
  preamble finished decoding — physically impossible. ANCLA's own responder
  already turns around at 2000 uus; matching it on the reference node is what
  lets `cal_initiator.c` use a single RX window for both peer types. See
  `docs/antenna-delay-calibration.md` §1.1.

## System context

Ranging is Two-Way Ranging (TWR); distance is computed on the tag, not the
anchor. The PHY contract every node must match — **channel 5, PLEN_1024, PAC32,
code 9, 850 kbps, SFD_IEEE_4Z, STS/PDoA off** — and the 802.15.4z wire format
live in the sibling ESP-IDF anchor project at
`../../../PlatformIO/Projects/ESP32S3UWB` (see its `CLAUDE.md`). That project is
the working reference implementation for ranging; the SLAVE responder here
(`src/anchor_respond.c`, `src/uwb_slave.c`) has now ported the WAVE and
DISCOVERY response paths from it. The gateway/beacon side is now implemented
(`src/uwb_gateway.c`, `src/gw_core.{c,h}`): a MAC-only gateway emits the
200 ms TDMA beacon and runs the CAP seat protocol (JOIN→GRANT, KEEPALIVE,
RELEASE), and slaves suppress ranging responses that would collide with that
beacon (`src/beacon_guard.{c,h}`). Beacon periodicity and JOIN→GRANT are
bench-confirmed (see "This project's own GATEWAY mode" above); KEEPALIVE and
RELEASE are implemented but not yet independently observed on the bench.
Both response paths are bench-confirmed correct over the air (sniffer capture,
see the "Ranging confirmed on the bench" hard-won fact above) after fixing the
delayed-TX turnaround budget and an unbounded-wait hang.

The tag's distance computation **is** now observable, end to end: with three
anchors answering, sniffer captures show the tag's `0xEA` POS frames carrying
`n_anchors = 3`, the responders' turnaround measured at exactly
`POLL_RX_TO_RESP_TX_DLY_UUS + ant_delay_tx` (2000 UUS × 65536 + ~16 000 DTU,
read straight off the wire), the gateway logging each fix through `pos_sink`,
and the platform receiving them over MQTT. What is **not** yet trustworthy is
the accuracy of those coordinates — see "URGENT next work" at the top.

## Host tests

Plain gcc, no Zephyr, following the sibling projects' pattern:

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe          # PASSED, exits 0

gcc -Wall -Wextra -o tests/uwb_frame/test_uwb_frame.exe tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
./tests/uwb_frame/test_uwb_frame.exe            # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/disc_schedule/test_disc_schedule.exe tests/disc_schedule/test_disc_schedule.c src/disc_schedule.c
./tests/disc_schedule/test_disc_schedule.exe    # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe tests/gw_core/test_gw_core.c src/gw_core.c
./tests/gw_core/test_gw_core.exe                # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/beacon_guard/test_beacon_guard.exe tests/beacon_guard/test_beacon_guard.c src/beacon_guard.c
./tests/beacon_guard/test_beacon_guard.exe      # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c
./tests/pos_json/test_pos_json.exe              # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/net_config/test_net_config.exe tests/net_config/test_net_config.c src/net_config.c
./tests/net_config/test_net_config.exe          # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/cal_solve/test_cal_solve.exe tests/cal_solve/test_cal_solve.c src/cal_solve.c src/cal_math.c
./tests/cal_solve/test_cal_solve.exe            # PASSED, exits 0
```

## Repo

Local git only, on `master`, **no remote configured** — nothing is pushed
anywhere.

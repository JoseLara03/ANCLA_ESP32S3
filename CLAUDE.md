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
before any position is worth reading. Both are now written, built and
code-reviewed; **neither has been run on hardware**, and until they have, the
residuals recorded below are still the current measurement.

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
(`src/ss_initiator.{c,h}`, then named `cal_initiator`), and the
`cal ref`/`cal peer` shell commands
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

Anchor coordinates used to be hand-entered with `anchor pos` and **silently
wrong** when unset — a SLAVE with no position answered ranging polls reporting
**(0, 0)**, and three anchors all claiming the origin yielded a confidently
meaningless fix with no error anywhere. That cost a bench session: after an
`anchor id` swap the coordinates did *not* follow the ids, leaving two live
anchors on the same baseline and the apex coordinate stranded on a third board.

**The survey is now fully implemented, built, and code-reviewed** — branch
`feat/anchor-auto-positioning`, fifteen tasks, plus a later branch
(`feat/apos-2d-survey`) that added a 2D solve mode alongside the original 3D
one. A gateway enumerates its anchors by EUI-64, commands every ordered pair
to range, solves the inter-anchor geometry with a gauge-constrained
Levenberg-Marquardt fit — in 3D (a 4-anchor gauge: origin/xaxis/plane/up) or
in 2D (a 3-anchor gauge: origin/xaxis/plane, no `up`) — (`src/apos_geom.c`,
host-tested), and pushes the solved coordinates back into each anchor's NVS
(`src/apos_gw.c`, `src/apos_node.c`, `src/apos_shell.c`). `apos gauge
origin=<id> xaxis=<id> plane=<id> [up=<id>]` selects the mode: omitting
`up=` (or passing `up=-1`) runs a 2D survey. Both motivating defects above
are closed: `anchor_respond_wave_poll()` now **refuses** to answer
unpositioned rather than reporting `(0, 0)`, and `pos_json_anchors()`
publishes the surveyed geometry on `uwb/anchor/setup/<zone>` instead of the
stub, falling back to the stub only when no survey has ever been applied. The
full runnable procedure is `docs/anchor-auto-positioning.md`.

**The approach is SS-TWR between anchors, not DS-TWR.** DS-TWR was evaluated
and deliberately deferred: the SS-TWR responder path is already written,
bench-confirmed and about to be antenna-delay-calibrated, the mandatory
`dwt_readclockoffset()` correction already handles the clock offset DS-TWR
would cancel by construction, and measuring **both directions of every pair and
averaging them** removes the antenna-delay asymmetry a single direction bakes
into the geometry. DS-TWR would have cost a new initiator, a new production
responder path and four new frame types for that. See
`docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md` §4 — do
not silently re-litigate this.

**What has not happened yet: any hardware run at all.** Not one survey has been
executed on a board. The outstanding hardware gates, in the order a human would
hit them running `docs/anchor-auto-positioning.md`:

- Anchors are antenna-delay calibrated first (§1 above). A survey run before
  calibration produces confidently wrong geometry that nothing will flag.
- `apos enum` on a gateway lists exactly four peers with distinct EUI-64s and
  distinct short addresses `0x0001`–`0x0004`.
- `apos run` completes with `missing_pairs:0`, all nodes placed, none
  ambiguous, and the beacon stays on time throughout — no
  `"beacon started but TXFRS never completed"` during the ranging phase, the
  solve, or the persist. `APOS_GW_SOLVE_BUDGET_UUS` (150000) and the
  survey-persist gate that reuses it have **never been timed on hardware**.
- The solved node-to-node distances match a **tape measure**. On a four-anchor
  array this is the only real check on the GEOMETRY — see the hard-won fact
  below on why `rms_mm` cannot be one, and read `max_reciprocal_mm` /
  `max_sd_mm` for the ranging.
- `apos apply` reports `ok:4, failed:0, skipped:0, persisted:1`; every anchor
  reports its new coordinates immediately and they survive `kernel reboot cold`.
- `apos ref` set and the retained anchors payload carrying the surveyed
  COORDINATES. The anchor names stay `ANC-LOBBY-00N` on purpose — the customer
  platform may key its records on `name`, so a survey changes coordinates only
  (`src/pos_json.c`).
- The point of the branch: a tag ranging four surveyed anchors, with `0xEA`
  `residual` under ~0.1 m and `(x, y)` stable between fixes.

None of these have been run.

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
production build (no `EXTRA_CONF_FILE`) compiles none of the `cal_*.c` files —
but it *does* compile `ss_initiator.c`, which the anchor survey needs; see the
hard-won fact below for where the safety property moved to.

There is also a **debug image**, same mechanism, for the ranging path:

```powershell
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_dbg -- "-DEXTRA_CONF_FILE=debug.conf"
west flash -d build_dbg
```

Quote the `-D`: PowerShell splits an unquoted `-DEXTRA_CONF_FILE=debug.conf` at
the dot and the build fails looking for a file named `debug`. `debug.conf` sets
`CONFIG_ANCLA_RANGING_DEBUG=y`, which compiles in a per-DISCOVERY and per-WAVE
verdict line, a per-second SLAVE RX heartbeat, and the beacon-guard state behind
every suppression. It changes no frame, no timing constant and no gate, so a
debug board is an ordinary peer on air and mixes with production boards — but it
is not a deployment image (per-frame console traffic plus one extra SPI read per
second on the SLAVE loop). See `docs/discovery-silent-anchor-debug.md`.

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

The anchor survey adds an `apos` tree. It is in the **production** image, and
every subcommand except `show` refuses unless `anchor mode` is `gateway`. No
`apos` command transmits: each sets state and returns, and the gateway loop does
the radio work and logs the outcome as JSON.

```
apos enum                      broadcast SURVEY_BEGIN and list the anchors that
                               answer, by EUI-64 and short address
apos gauge origin=<id> xaxis=<id> plane=<id> [up=<id>]
                               pin the coordinate frame; named arguments, any
                               order, ids are 0..3 (the same space `anchor id`
                               uses). Omit up= (or pass up=-1) for a 2D
                               (3-anchor) survey; a 4th id there selects the
                               existing full 3D solve.
apos run                       re-enumerate, range every ordered pair, solve,
                               and REPORT ONLY — persists NOTHING
apos apply [force]             push the result to every anchor, persist it and
                               close the survey. `force` overrides the
                               acceptance thresholds, not the unverified-mesh
                               warning
apos ref <lat> <lon>           the origin anchor's real-world position, for the
                               platform map; persists immediately, refuses
                               while a survey is running
apos zoff <metres>             shift z so z=0 is the floor rather than the
                               plane through the gauge anchors; applied on the
                               next `apos run`
apos show                      phase, enumerated anchors and the stored survey
                               as JSON — the one subcommand a SLAVE accepts
```

See `docs/anchor-auto-positioning.md` for the full procedure, and read its §0
before trusting an `"accepted":1`.

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
  are deliberately absent. `pos_json_anchors()` takes a
  `const struct apos_survey *` and publishes the **surveyed** geometry when one
  has been applied — one entry per surveyed anchor, `node[0]` (the gauge origin)
  carrying the `apos ref` lat/long and the rest local-only in metres relative to
  it. With no survey it falls back to the original four-anchor stub
  (`ANC-LOBBY-001..004` at the corners of a 2 m × 2 m square) so an unsurveyed
  gateway still publishes a schema-valid document. The stub's coordinates are
  placeholders; its schema is the contract.
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
- `src/ss_initiator.{c,h}` — one SS-TWR exchange with this board as the
  initiator, the role the tag normally plays. Was `cal_initiator.{c,h}` and
  calibration-image-only; it is now in **both** images, because the anchor
  survey needs an anchor to poll its peers. Renamed accordingly — nothing about
  it is calibration-specific.
- `src/apos_frame.{c,h}` — the survey wire format: one message type (`0xEB`)
  with a subtype byte carrying seven messages (SURVEY_BEGIN, ENUM_RSP,
  RANGE_CMD, RANGE_RSP, SETPOS, SETPOS_ACK, SURVEY_END). Deliberately *not*
  added to `uwb_frame_802_15_4z.c`, which must stay byte-identical to the tag's
  copy. Pure C, host-tested in `tests/apos_frame/`.
- `src/apos_geom.{c,h}` — the sparse geometry solver, 2D or 3D: a closed-form
  seed then a gauge-constrained Levenberg-Marquardt refine over a flat edge
  list, with residual, planarity (3D) / gauge-collinearity (2D) and
  reflection-ambiguity diagnostics. `enum apos_geom_dim` (`APOS_GEOM_3D`,
  the zero value for backward compatibility, or `APOS_GEOM_2D`) selects a
  4-node (origin/xaxis/plane/up) or 3-node (origin/xaxis/plane) gauge; z stays
  pinned at 0 for every node in 2D mode. Pure C, host-tested in
  `tests/apos_geom/`. `APOS_MAX_NODES` is 8, not `UWB_MAX_ANCHORS`.
- `src/apos_table.{c,h}` — the gateway's working set for one survey: peers keyed
  by **EUI-64** (an `anchor id` swap must not strand a coordinate again),
  directed measurements, and their symmetrisation into inverse-variance-weighted
  undirected edges. Pure C, host-tested in `tests/apos_table/`.
- `src/apos_store.{c,h}` — the last applied survey plus the site's geographic
  reference, persisted under the `apos/` settings subtree. Header is pure C so
  `pos_json.c` can consume `struct apos_survey` in a host test.
- `src/apos_gw.{c,h}` — GATEWAY orchestration: enumerate, range every ordered
  pair, solve, push, persist. A **step machine**, not a thread — `apos_gw_step()`
  emits at most one frame and returns, so a survey unfolds over many superframes
  and never delays the beacon. Also holds the acceptance thresholds and the
  unwired MQTT survey trigger.
- `src/apos_node.{c,h}` — the anchor side: answer enumeration in an
  EUI-64-hashed stagger slot, run exactly the ranging batch the gateway
  commands, store exactly the coordinates it is given. Holds the survey window
  that is the whole safety boundary for this board ever initiating anything.
- `src/apos_shell.c` — the `apos` console command tree. Gateway-only in
  practice, registered unconditionally so a slave gets a clear refusal rather
  than a missing command.
- `src/cal_run.{c,h}` — CAL mode's main loop: an ordinary WAVE responder that
  can be told, from the console, to become a temporary SS-TWR initiator for
  one batch of exchanges.
- `src/cal_shell.c` — the `cal` console command tree (`cal ref`, `cal peer`).
  Calibration image only.
- `Kconfig` — the project's own `Kconfig`, sourcing `Kconfig.zephyr` last;
  defines `CONFIG_ANCLA_CAL_MODE`.
- `cal.conf` — the `EXTRA_CONF_FILE` overlay that builds the calibration image
  instead of production (see "Build & flash").
- `src/uwb_debug.h` — `ANCLA_LOG_LEVEL`, the one symbol the ranging modules
  register their log level through, so `CONFIG_ANCLA_RANGING_DEBUG` can reach
  their `LOG_DBG` lines. Resolves to `LOG_LEVEL_INF` in production.
- `debug.conf` — the `EXTRA_CONF_FILE` overlay that builds the debug image (see
  "Build & flash").
- `docs/discovery-silent-anchor-debug.md` — the capture procedure and decision
  table for the open "a surveyed anchor stops answering DISCOVERY after a power
  cycle" fault: what each debug line means, and which candidate cause each log
  pattern confirms or kills.
- `docs/antenna-delay-calibration.md` — the operator procedure for the above:
  DWM3001CDK prerequisites, physical setup, `cal ref`, the `cal peer`
  cross-check and its acceptance threshold, troubleshooting.
- `docs/anchor-auto-positioning.md` — the operator procedure for the survey:
  prerequisites, the gauge, the `apos` walkthrough, how to read the report,
  troubleshooting, and — §0, read it first — why the acceptance check cannot
  validate a four-anchor array.
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
  lets `ss_initiator.c` use a single RX window for both peer types. See
  `docs/antenna-delay-calibration.md` §1.1.
- **On a four-anchor array solved in 3D mode, the survey's `rms_m` is
  identically zero and proves nothing** — but this is a 3D-mode-specific
  fact, not a hardware ceiling; see the 2D bullet just below for the
  exception this branch added. The 3D gauge pins 6 degrees of freedom, so a
  fit over N placed nodes has `3N-6` free parameters. At N = 4 a full mesh has
  exactly 6 edges against exactly 6 free parameters — an isostatic system with
  no spare equation — and LM re-embeds *any* set of distances exactly: `rms_m`
  and `worst_edge_m` come back at zero however bad the ranging was. From N = 5
  to 7 there is enough redundancy for a nonzero `rms_m` but not always enough
  for `worst_i`/`worst_j` to name the pair actually at fault, because
  least-squares masking can spread the disagreement onto a merely-correlated
  edge. Both were reproduced in `tests/apos_geom/test_apos_geom.c`. **The
  deployment is four ranging slaves, i.e. exactly the degenerate case when
  solved in 3D**, so `"accepted":1` there means only "nothing contradicted the
  ranges". The code says so rather than hiding it (`apos_gw_result_unverified()`,
  the `spare_edges`/`rms_meaningful` fields, a `LOG_WRN` pair at solve and
  again at apply, and a `shell_warn` under the operator's own `apos apply`);
  the check that actually validates the geometry is a **tape measure**. Do not
  "fix" this by loosening a threshold — the thresholds are not the problem,
  the edge count is. **There is no fifth-anchor option to suggest to an
  operator** for growing the 3D case: `UWB_MAX_ANCHORS` is 4, `anchor id` is
  bounded 0..3, and `apos_node.c` refuses a `RANGE_CMD` naming a peer at or
  beyond `UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS`, so `APOS_MAX_NODES` (8) is
  structural headroom, not a way to add a 5th ranging anchor. Growing past
  four ranging anchors is engineering work — `UWB_MAX_ANCHORS`,
  `disc_schedule`'s stagger, `anchor_respond.c`'s `TX_COMPLETE_TIMEOUT_MS`
  re-derived from that stagger, and the tag's `UWB_FRAME_MAX_ANCHORS` behind a
  frozen wire format. What the operator CAN read on the array they have is
  `max_reciprocal_mm` (largest `|d(A→B) − d(B→A)|`, computed by
  `apos_table_quality()` before `symmetrise()` averages it away) and
  `max_sd_mm`, both in the `apos_solve` JSON and in `apos show`. Those make the
  RANGING observable; they do **not** make the geometry over-determined — a
  rigid 4-node framework stays isostatic either way.
- **A bare 3-anchor 2D survey is exactly as degenerate as the 4-anchor 3D
  case above, for the same reason — but 2D mode run over the actual
  4-anchor deployment is NOT degenerate, and this is the one case on this
  hardware where `rms_mm` is a real (if weak) quality signal.** `2N-3` free
  parameters against exactly 3 edges at `N = 3` is isostatic, identical in
  kind to the 4-anchor 3D floor above. But the deployment described
  throughout this file is **four** ranging anchors, not three: a 2D gauge
  only *names* three of them (origin/xaxis/plane); the 4th rides along as an
  ordinary placed node, exactly as `apos gauge` already supports. At N = 4 in
  2D that is `2*4-3 = 5` free parameters against the full mesh's 6 edges — one
  spare equation, confirmed empirically in `tests/apos_geom/test_apos_geom.c`
  and by hand (corrupting one edge by 300 mm in a 4-node 2D mesh yields
  `rms_mm ≈ 36.4`, not zero). So on THIS hardware, running the survey in 2D
  mode over all four anchors is the one configuration where `rms_mm` actually
  means something; running it in 3D mode, or in 2D with only three anchors
  total, is not. See
  `docs/superpowers/specs/2026-08-18-apos-2d-survey-design.md`.
- **`ss_initiator.c` is compiled into the PRODUCTION image.** It was
  `cal_initiator.c` and calibration-only, and the old safety property — "a
  deployed anchor can never initiate a poll" — came from the build set. It no
  longer does: the anchor survey needs an anchor to poll its peers. The property
  now comes from `apos_node.c`'s two gates (a gateway-opened survey window, plus
  a session-matched `RANGE_CMD` from `0x0000`), plus the `APOS_MAX_EXCHANGES`
  count cap and the `APOS_RANGE_BATCH_DEADLINE_MS` wall-clock cap. Remove or
  weaken either gate and the collision hazard the cal image was kept separate to
  avoid is back, in production. What must stay out of production is the `cal`
  shell tree and `cal_run.c`'s unsolicited-poll loop, and it does.
- **Any flash write or erase stalls ALL execution on this part, whatever the
  thread priority.** The WROOM-1-N8R2 executes XIP from the same flash the
  settings subsystem writes, and a write or erase disables the instruction
  cache for its duration. `K_PRIO_COOP(0)` does not protect the gateway loop
  from that and nothing else can either. A settings append is sub-millisecond,
  but an NVS sector rotation erases 4 kB at ~25–45 ms — 5–9× the 5 ms
  `BEACON_ARM_MARGIN_UUS`. The beacon then arms late, its delayed TX times out
  on TXFRS, `beacon_tx_ts` is left unadvanced, and every slave's `beacon_guard`
  starts suppressing against a schedule that has shifted underneath it: a
  network-wide timing fault, not a dropped frame. So NVS writes on the gateway
  loop are budget-gated — `step_apply()`'s survey persist reuses
  `APOS_GW_SOLVE_BUDGET_UUS` (150000 uus, sized so only the first step after a
  beacon can satisfy it), and `apos ref` refuses outright while a survey is
  running. Neither is a hard bound and no in-loop gate can be one; the real fix,
  if this ever hurts, is to stop writing flash from this thread at all.
- **`APOS_MAX_NODES` (8) is not `UWB_MAX_ANCHORS` (4), deliberately — and
  surveying eight does not make eight rangeable.** The solver and table handle
  eight nodes so the survey does not inherit the tag-facing ranging MAC's limit,
  but an anchor refuses a `RANGE_CMD` naming a peer outside
  `UWB_ANCHOR_ADDR_BASE .. +UWB_MAX_ANCHORS` (`apos_node.c`), because anything
  above that would silently alias onto another anchor's wire id once truncated
  to a byte. Growing the deployment past four ranging anchors needs
  `disc_schedule.h`'s stagger and `anchor_respond.c`'s `TX_COMPLETE_TIMEOUT_MS`
  re-derived (18 ms is sized for `disc_resp_delay_uus(3)` = 12500 uus; an id 5
  would need ~19.5 ms and would silently lose every DISCOVERY response), plus
  `UWB_MAX_ANCHORS` and the tag's `UWB_FRAME_MAX_ANCHORS`.
- **An unpositioned anchor is now silent to tags, not `(0, 0)` — but it still
  answers DISCOVERY.** `anchor_respond_wave_poll()` refuses unless
  `position_valid` or a survey window is open; `anchor_respond_discovery()` is
  deliberately *not* gated. So an unsurveyed board enters the tag's anchor list
  and then never answers a poll, which on a sniffer looks exactly like an RF
  fault: `0xE2`/`0xE4` DISCOVERY traffic with no matching `0xE0`/`0xE1`. Look for
  `WAVE poll refused — no surveyed position` in the monitor before suspecting
  the radio. The survey-window exception is what lets the gate exist at all: in
  a cold deployment every anchor is unpositioned yet must answer its peers.
- **A per-module log level is a compile-time cap that no `.conf` and no shell
  command can lift.** Every module on the ranging path registers explicitly
  (`LOG_MODULE_REGISTER(anchor_respond, LOG_LEVEL_INF)`), and `LOG_DBG` under
  such a module is not filtered at runtime — it is not in the binary.
  `CONFIG_LOG_DEFAULT_LEVEL` only supplies a level to modules that register
  WITHOUT one, so raising it does nothing for these and merely promotes the whole
  WiFi/net/mbedTLS stack to DBG. `CONFIG_LOG_RUNTIME_FILTERING` is already `y`
  here, so `log enable dbg <module>` exists in production and is useless for the
  same reason: it can only narrow what a build compiled in. Recovering those
  lines takes a rebuild, which is what `ANCLA_LOG_LEVEL` (`src/uwb_debug.h`) and
  `debug.conf` exist for. Corollary, learned the hard way: Zephyr's `LOG_DBG`
  still **expands its arguments** where the level is compiled out
  (`Z_LOG_TO_PRINTK`), so any helper a debug-only log line calls must exist in
  the production build too — `static inline` keeps it from warning as unused.
- **`apos_gw_step()` may emit at most one frame per call.** It runs on the
  `K_PRIO_COOP(0)` loop that arms the beacon, so a step that transmits twice, or
  blocks, delays the beacon for the whole network. Same class of hazard as the
  unbounded TXFRS wait that once froze the console. Survey deadlines are all
  absolute wall-clock (`k_uptime_get()` deltas observed across steps), so a
  skipped step costs latency and never correctness — which is what makes
  refusing to step cheap enough to do liberally.

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

gcc -Wall -Wextra -Isrc -o tests/uwb_frame/test_uwb_frame.exe tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
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

gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
./tests/apos_geom/test_apos_geom.exe            # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/apos_table/test_apos_table.exe tests/apos_table/test_apos_table.c src/apos_table.c src/apos_geom.c -lm
./tests/apos_table/test_apos_table.exe          # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/apos_frame/test_apos_frame.exe tests/apos_frame/test_apos_frame.c src/apos_frame.c
./tests/apos_frame/test_apos_frame.exe          # PASSED, exits 0
```

`-lm` is required by the two suites that link `apos_geom.c` — the solver calls
`sqrtf`/`fabsf`. The others do not need it.

## Repo

Local git only, on `master`, **no remote configured** — nothing is pushed
anywhere.

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

**Partially run.** On 2026-08-25 one anchor was taken through `cal ref` at a
2 m reference distance and reached **-8 mm**, inside the `|error_mm| < 15` gate.
Everything else in the list above is still open on every board, including the
repeat run, `kernel reboot cold` survival, and the whole `cal peer` cross-check.
Treat the remaining two anchors as uncalibrated until a console reading says
otherwise — `anchor show` reports the live values.

A separate observation that is NOT calibration evidence, recorded so it is not
mistaken for it: the first successful `apos run` (2026-08-26, §2 below) reported
`max_reciprocal_mm: 5` across the whole mesh. That is the largest
|d(A->B) - d(B->A)|, and **it says nothing about antenna delay at all** — not
even about its asymmetry. Writing out the observable with per-board delay
errors `e`:

```
tof_measured = TOF_true - (e_tx_I + e_rx_I + e_tx_R + e_rx_R) / 2
```

which is **symmetric in initiator and responder**: swapping the roles gives
the identical expression, so SS-TWR range is EXACTLY reciprocal in antenna
delay and there is no antenna-delay asymmetry for reciprocity to bound or for
averaging to cancel. An earlier version of this paragraph said reciprocity
"bounds the antenna-delay asymmetry" and that averaging both directions
cancels it; the conclusion below is right, the reason was not — the same shape
as the `RX_ANT_DLY` bullet further down, which also reached a correct
conclusion by the wrong route.

What DOES flip sign between the two directions, and therefore what
`max_reciprocal_mm` actually measures, is the **residual clock-offset error**.
`dwt_readclockoffset()` is measured by whichever board is the initiator, so
its estimation error enters with opposite sign each way, and averaging the two
directions cancels it. The sensitivity is large: `d(range)/d(offset) =
rtd_resp/2 x 4.69 mm`, which at this project's 2000 uus turnaround is
**~307 mm per ppm** (consistent with the "5 ppm is ~1.5 m" figure in
`ss_initiator.c`). Observed directly on the bench 2026-09-03: the same board
pair read `clk_off = -3 ppm` one way and `+3 ppm` the other, with a 78 mm
reciprocity gap — 0.25 ppm of residual, well above `dwt_readclockoffset()`'s
own ~0.015 ppm (4.7 mm) quantisation, so estimator bias or thermal drift
rather than resolution.

The conclusion this paragraph existed for is unchanged and still the point: a
small `max_reciprocal_mm` says nothing about the common-mode bias every anchor
shares — which is the part calibration fixes and the part that moves an
absolute distance. **A perfectly reciprocal mesh can be uniformly wrong by
metres.**

`docs/antenna-delay-calibration.md` is the
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

### The survey has now RUN on hardware (2026-08-26), with THREE anchors in 2D

First successful end-to-end `apos run` + `apos apply`, on a gateway plus three
anchors (`0x0001`, `0x0002`, `0x0004`; `0x0003` was powered off). It took four
attempts and a silent stack overflow to get there — see the hard-won fact on
`struct gw_core_ctx` below, which is the single most expensive thing this
project has learned about the survey path.

**Met, and not to be re-run to prove:**

- `apos enum` → 3 peers with distinct EUI-64s and short addresses.
- `apos run` → `missing_pairs:0`, `placed:3/3`, `ambiguous:0`, `iters:15`.
- **The beacon stayed on time throughout**, including the ranging phase, the
  solve and the NVS persist: the `gw_sf` heartbeat came out at exactly 200.0 ms
  (systime delta 49 923 544 hi32 x 4.006 ns) with not one `"beacon started but
  TXFRS never completed"`. So `APOS_GW_SOLVE_BUDGET_UUS` (150000) and the
  survey-persist gate that reuses it are now **timed on hardware** and hold.
- `apos apply` → `ok:3, failed:0, skipped:0, persisted:1`, every anchor
  acknowledging its coordinates.
- The ranging quality numbers that are real on this array:
  `max_reciprocal_mm: 5` (largest |A→B − B→A| across the whole mesh) and
  `max_sd_mm: 40`. Reciprocity of 5 mm means the antenna-delay ASYMMETRY is
  negligible — it says nothing about the common-mode bias, see below.
- `gauge_collinearity` came back at **1.831** against the "thin" threshold of
  `APOS_ACCEPT_GAUGE_COLLINEARITY` (0.10), i.e. 18x clear. Higher is better
  here (the ratio is |plane.y| / |xaxis.x|); the gauge triangle was
  well-conditioned, not marginal.

**Still outstanding, and the first two are the ones that matter:**

- **Tape measure.** The solved geometry was
  `0x0001 (0.000, 0.000)`, `0x0002 (1.263, 0.000)`, `0x0004 (0.914, 2.312)`,
  i.e. edges 1.263 / 2.486 / 2.338 m. The `anchor pos` values the same boards
  were carrying implied 2.020 / 2.388 / 1.301 — up to **1037 mm** different,
  and suspiciously like a PERMUTATION of the same three lengths, which is the
  `anchor id` / coordinate mismatch this branch exists to remove. Either the
  boards moved since those values were typed, or one of the two is badly wrong.
  Nothing in the survey report can settle it: `rms_mm` was 0 by construction
  (3 edges against 3 free parameters) and reciprocity cancels asymmetry, not
  the common-mode antenna-delay bias every anchor shares. **Measure it.**
- **Antenna-delay calibration** (§1 above). One anchor has a -8 mm `cal ref`
  result; the other two have no recorded result, and no board has been through
  the repeat run or the `cal peer` cross-check. So the absolute scale of those
  three edges is unverified — and `max_reciprocal_mm: 5` does not verify it, for
  the reason spelled out at the end of §1.
- Survival of `kernel reboot cold`.
- A FOUR-anchor run. With three anchors in 2D the mesh is isostatic and
  `rms_mm` is identically zero; with four it is 6 edges against 5 free
  parameters and `rms_mm` becomes a real (if weak) signal. `0x0003` did not
  answer enumeration in three consecutive attempts and needs looking at.
- `apos ref` set, and the retained anchors payload carrying the surveyed
  COORDINATES. The anchor names stay `ANC-LOBBY-00N` on purpose — the customer
  platform may key its records on `name`, so a survey changes coordinates only
  (`src/pos_json.c`).
- The point of the branch: a tag ranging four surveyed anchors, with `0xEA`
  `residual` under ~0.1 m and `(x, y)` stable between fixes.

### 3. TDoA migration (Phase 2/3): the measurement chain works, capacity does not yet

Separate track from §1/§2 above, and the reason the two must not be
conflated: this is about **how many tags the network can serve**, not
whether any one tag's coordinate is trustworthy. Full history in
`docs/superpowers/plans/2026-08-25-fase3-tdoa.md` and
`docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md`; this is the
short version.

**Phase 2 gate: ran, and failed against its own original threshold — Phase 3
proceeded anyway, by a deliberate product decision, not because the hardware
met the bar it was asked to meet.** Anchor-to-anchor timestamp jitter over CCP
measured **782 ps at 30 cm** (`marginal`) and **1.44–1.53 ns at 3 m**
(**`fail`**) against the 1 ns target (`docs/anchor-sync-measurement.md`
§4.1). That converts to ~0.43–0.46 m of implied position error at the 3 m
figure. The accuracy target was consciously re-derived from 10–30 cm to
**~45 cm**, and the worst measured case lands right at that number — proceed
at that accuracy, not at the original one, and do not cite a 10–30 cm figure
for this system without re-checking that decision first.

**Phase 3, Task 7 (the tag emits BLINK instead of ranging): ran on hardware
2026-08-30.** The whole observation chain — tag emits `0xF0` BLINK, every
anchor that hears it stamps and converts the timestamp via
`sync_model_to_master()`, the gateway groups (`tdoa_collect`), rebases and
bounds the 40-bit timestamps (`tdoa_dtu`), solves (`tdoa_solve`), and
publishes through the same `pos_sink_publish()`/MQTT path as before — is
verified producing real fixes, including **surviving a `kernel reboot cold`**
after the survey was re-applied. **Read this precisely: what was measured is
precision (repeatability between consecutive fixes), not accuracy.** No
ground-truth position has been taken, so the ~45 cm target above is still a
target, not yet a verified number. `out->residual_m` from `tdoa_solve()` is
zero by construction at `TDOA_MIN_ANCHORS` (3) — it is not evidence of fit
quality here and must not be cited as such (see `src/tdoa_solve.h`'s own
caller contract).

**Task 8 (cleanup) is done, in `tag_testting`, 2026-08-30.** `pos_dbg.c` (the
temporary raw-range BLE log used to tune the tag's own EKF) is retired per
its removal checklist in `tag_testting/spec/2026-08-22-position-filtering-design.md`
— its capture campaign never ran; it is gone because Phase 3 moved
position-solving off the tag entirely, not because the log finished its job.
`pos_solver`/`pos_residual` ownership has NOT yet moved from `tag_testting`'s
`CLAUDE.md` to this one — that transfer is still open and belongs to whoever
next edits both files together, so a diff between the two copies can be
checked at the same time as the ownership note changes.

**What is NOT done, and is the actual blocker on the 100-tags-at-5Hz
objective: Task 4B, capacity.** The BLINK as shipped by Task 7 occupies the
*same* 13.32 ms slot a full TWR exchange used — a 1.223 ms frame sitting in a
slot sized for something 10.9x longer. The tag saves battery; **the network
gains zero capacity** and still serves the same ~11 tags TWR always did. This
is now fully designed — `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md`
resolves all five open questions the original Task 4B stub left (slot
assignment is `slot_index(seat_id) = seat_id`, the CFP is repartitioned
rather than extended so `T_SUPERFRAME_UUS` stays untouched, a cell runs
TWR-mode or blink-mode but never both) — and planned,
`docs/superpowers/plans/2026-08-30-blink-slotted-mac.md`, six tasks across
both repos.

**Status 2026-09-01: Tasks 1, 2, 3, 4 and 5 are written/measured and
host-test/build clean; Task 6 (end-to-end hardware verification) has NOT been
run — nothing below has been verified as a complete BLINK-mode cell on a
board, only its pieces.**

- **Task 2** (tag TX-arm jitter, hardware, measured 2026-09-01): done.
  `tools/blink_jitter.py` against `COM7_2026_09_01.12.43.19.533.txt`, three
  real tags forced to seat_id 120/121/122 (`CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT`
  — see the Kconfig entry below) so the measurement actually covered the
  known-risky tail of the seat table, not just the low seat ids a 3-tag bench
  fleet lands on by default. n=873/822/733, all well past the plan's ≥200
  gate. **Max spread 39.2 ns** — four orders of magnitude below the
  already-known 64 us gateway TX-arm jitter, so the tag is not the dominant
  jitter source and does not need its own margin term to speak of. A first
  capture (`COM7_...12.17.37.616.txt`) was discarded: one address showed a
  bimodal ~10 ms spread, the signature of a tag RESCAN (new join, same
  address window) mixing two different slot assignments in one log.
  `BLINK_SLOT_GUARD_UUS` moved from the 200 us provisional value to
  **100 us** (sum of the two known terms, 64 + 8 us, rounded up for margin —
  the tag's own 39.2 ns term is noise against that sum), recorded in
  `src/blink_sched.h` and
  `docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md` §1.2
  together, per that section's own no-diverge rule.
- **Task 3** (`src/blink_sched.{c,h}`, ANCLA, host-tested): done, and its
  headline number MOVED with Task 2's measured guard.
  `blink_sched_n_slots()` on the frozen PHY with the now-measured
  `BLINK_SLOT_GUARD_UUS = 100` gives **`BLINK_N_SLOTS = 144`** (`tests/blink_sched/`,
  PASSED) — up from the 134 the 200 us provisional guard gave, and now
  **greater than `GW_MAX_SEATS` (128)**: the seat table's fixed array size,
  not the blink schedule, is the binding cap on this build. The overflow-band
  admission policy described in the design spec §1.1
  (`gw_core_join()` refusing a seat_id `>= BLINK_N_SLOTS` in BLINK mode) is
  therefore hypothetical on this build, not exercised — it would only trigger
  again with a looser guard or a larger `GW_MAX_SEATS`. `tests/blink_sched/`
  still pins the order of magnitude (100-150), not the exact number.
- **Task 4** (`src/gw_core.{c,h}`, `src/uwb_gateway.c`,
  `src/uwb_frame_802_15_4z.h`, ANCLA): done. `UWB_PROTO_VER` is now **4**.
  Mode selection is `anchor cell <twr|blink>` (persisted, GATEWAY-only,
  mirrors `anchor mode`'s existing pattern) rather than a Kconfig or a new
  MQTT trigger — the simplest option that needed no new persistence
  mechanism. `gw_core_set_blink_mode(ctx, n_slots)` bounds a FRESH join's
  seat-id scan to `n_slots` in BLINK mode (`tests/gw_core/` covers both the
  rejection and the `n_slots == 0` = TWR-unrestricted case);
  `gw_core_build_slotmap()` and `reschedule()` are untouched, per the design
  — in BLINK mode `tx_beacon()` simply stops reading their output and writes
  `sched[]` reserved/zero instead.
- **Task 1** (`tag_testting/src/uwb_net_runner.c`,
  `tag_testting/src/uwb_ss_initiator.{c,h}`): written, NOT hardware-verified.
  This is the tag's first-ever use of `DWT_START_TX_DELAYED` — flagged in the
  design as a genuinely new risk, not a copy-paste from the anchor side's
  existing delayed-TX code, and the two traps this project already paid for
  once (arm deadline measured against the preamble, not the RMARKER; a
  `dwt_forcetrxoff()` immediately before arming) are applied defensively
  without a bench cycle to confirm they transfer to this port. One thing NOT
  in the plan's own text but required for correctness once written: the
  bounded post-`dwt_starttx()` poll `blink_publish()` shares with
  `position_publish()` was sized for an immediate TX's ~1.3 ms airtime; a
  delayed BLINK can legitimately not fire for up to ~146 ms (the last blink
  slot), so `blink_publish()` now sleeps (`k_sleep`, not busy-wait) most of
  that gap and only busy-polls the last `BLINK_POLL_MARGIN_MS` (5 ms) —
  without that, every seat past the first would have its BLINK force-aborted
  before it ever transmitted.
- **Task 5** (same tag files, plus `UWB_NET_PROTO_VER` → 4 in
  `tag_testting/src/uwb_net.h`): written, NOT hardware-verified. The tag
  computes its BLINK slot as `seat_id` (the identity), inlined rather than
  copying ANCLA's `blink_sched.h` verbatim — that header pulls in
  `mac_budget.h`'s whole airtime model, which this tag build has no other
  use for. **Known gap, not fixed, and its numbers are now STALE:** the
  tag's fixed per-superframe overhead (`T_BEACON_MS`/`T_GUARD_MS`/
  `T_MINISLOT_MS`) is rounded UP to whole milliseconds, a TWR-era choice
  harmless against a 24 ms ranging slot and NOT harmless against a BLINK
  slot. The "roughly 132-133 of 134" figure this bullet used to carry was
  computed against the 200 us provisional guard; Task 2 moved the guard to
  100 us and `BLINK_N_SLOTS` to 144 (see above), which changes both the
  per-slot airtime and the superframe slack the tag's ~3.8 ms of extra
  rounding eats into. **Re-deriving the affected seat-id range in
  `tag_testting` against the new 100 us/144-slot numbers is unstarted
  work**, not done and not carried over automatically — do not keep citing
  132-133 of 134 now that the inputs it was computed from have changed.
  Fixing the underlying rounding means touching constants the TWR sweep and
  beacon prediction also share, so it is left as a known limit either way.
- **Task 6** (end-to-end hardware verification): **partially run
  (2026-09-01), ceiling-seat correctness confirmed, DENSITY STILL UNTESTED.**
  Production image, `CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT=y` /
  `CONFIG_ANCLA_DEBUG_DUMMY_SEATS=125` (see the Kconfig entry below), 3 real
  tags landed on seats 125/126/127 — the true ceiling this build can reach
  (`GW_MAX_SEATS`-bound, not 143). Confirmed on hardware: join, range and
  publish at seat 127; a lease timeout correctly freeing a ceiling seat after
  ~15 s of inactivity; the freed ceiling seat immediately re-granted to a
  fresh joiner; `Tid` staying stable across that rejoin (`tid=2082962887`
  before and after, under two different short addresses) with the documented
  `"no live seat, Tid falls back to short address"` fallback firing exactly
  as designed in the gap between expiry and rejoin; two `kernel reboot cold`
  cycles reproducing identical `seats_used:125/seats_free:3`; and no
  `"beacon started but TXFRS never completed"` anywhere in ~5 minutes of
  3-tag load. The rejoining tag's battery read 4% shortly after — likely
  brownout, not a MAC fault, but not confirmed either way.

  **What this run does NOT establish, and is the reason Task 6 stays open:
  DENSITY.** Three tags at low airtime (128-180 of 275 slot-superframes
  used) proves correctness at the seat-id ceiling, not that BLINK mode
  actually buys more capacity than TWR under real concurrent load — the
  entire point of this migration (§ "URGENT next work" is elsewhere in this
  file; the number to beat is TWR's ~11-tag ceiling). Closing Task 6 needs a
  density test: either more physical tags approaching the product target, or
  a deliberate synthetic/high-tier load (e.g. several tags forcing FAST tier
  simultaneously) that actually saturates `GW_SCHED_CAPACITY` (275) the way
  three IDLE-mostly tags here never did. Until that runs, "100 tags at 5 Hz"
  remains a schedule-math claim, not a measured one.

### Precisión y suavizado de la posición TDoA (Tasks 1-4 done 2026-09-02, Task 5 open)

Separate branch (`feat/tdoa-accuracy-filter`, from `feat/rtls-scale-tdoa`) and
separate plan/spec
(`docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md`,
`docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md`) from
everything above: this is about making the position a given fix reports LESS
NOISY and TEMPORALLY COHERENT, not about capacity or the ~45 cm accuracy
target. **Suavidad no es exactitud** — nothing here can move that number,
which is still bounded by the array's 1.2-2.5 m GDOP and the uncalibrated RX
antenna delay (item 8, still untouched).

- **Task 1** (`src/sync_model.h`, host-tested): a `SYNC_PHASE_EMA_SHIFT`
  sweep, {3,4,5,6} x seven jitter levels x 12 seeds, is a NEGATIVE finding,
  recorded with its real numbers next to the constant — worst cell across
  the table improves at most ~8.8% (shift 6 vs 3) against a 20% bar, and the
  algebra in the design spec's section 2.1 predicted exactly this. **Shift
  stays 3, unchanged.**
- **Task 2** (`src/tdoa_collect.{c,h}`, host-tested): `tdoa_collect_set_expected()`
  replaces the hardcoded `POS_MAX_ANCHORS` early-release threshold with the
  survey's actual anchor count (`apos_store_get()->n_nodes`, wired in
  `tdoa_gw_step()`), fixing a real defect — on this project's 3-anchor
  deployment, no group had EVER released early, paying the full 150 ms
  window plus up to 200 ms more every single fix.
- **Task 3** (`src/pos_ekf.{c,h}`, host-tested): the tag's own constant-velocity
  EKF gained `pos_ekf_update_tdoa()` (range-DIFFERENCE sequential scalar
  updates) and a `r_tdoa` config field (0.6 m default, sqrt(2)-derived from
  the Fase 2 jitter). **The design spec's premise that this file was dead
  code on the tag is WRONG** — `tag_testting/src/uwb_net_runner.c` still
  runs the range-based half of it on its TWR path — so this followed the
  `pos_solver.c`/`pos_residual.c` verbatim-copy precedent instead of the
  spec's "move" instruction; see `pos_ekf.{c,h}`'s own entry above for the
  full correction. On synthetic noisy data the filter's tracking RMS beat
  `tdoa_solve()`'s raw per-fix RMS by roughly 2x (0.216 m vs 0.397 m in the
  host test) — the actual point of adding it.
- **Task 4** (`src/tdoa_gw.{c,h}`, `CMakeLists.txt`): the filter wired into
  `tdoa_gw_step()`, `dt` from the reference anchor's absolute 40-bit `t_dtu`
  rather than the gateway loop's quantized `now_ms`, the 10 m jump gate
  preserved on every path that seeds. Builds clean for BOTH the production
  and calibration images; `dram0_0_seg` +1720 B. See `tdoa_gw.{c,h}`'s own
  entry above for the full flow and the new `blink stats` counters.

**Task 5 (the full protocol) has NOT run — but an opportunistic smoke test
has, 2026-09-02, on this project's own LIVE deployed gateway** (found already
running on the bench USB port, not a spare board — flashing it interrupted
whatever it was serving at the time). What that smoke test actually covered,
and its limits:

- Flashed clean, booted clean: real survey (4 nodes, `apos show`), WiFi/MQTT
  connected, a real tag joined, CCP master healthy (`sent` climbing,
  `dropped:0`), no `"beacon started but TXFRS never completed"`.
- **Caught a real bug the moment it was exercised on hardware, that host
  tests and code review both missed**: `tdoa_gw_ekf_stats()` was declared in
  `tdoa_gw.h` and documented as wired into `blink stats`'s third JSON line,
  but the function was never actually DEFINED or called from
  `blink_shell.c` — a genuine gap between what got reported as done and what
  shipped. Fixed, rebuilt, reflashed, confirmed the third line
  (`{"tdoa_ekf":{...}}`) now prints real counters on the live board.
- With that fixed, real fixes were flowing (`{"tdoa":{"ingested":82,
  "fixes":7}}`) and `n_dt_invalid` (4) dominated `n_filtered` (2) for the one
  live tag — investigated with TEMPORARY diagnostic logging (added, used,
  then REMOVED once root-caused, not left in-tree) rather than guessed at.
  Two real, already-correctly-handled causes, recorded as their own
  hard-won fact next to `TDOA_DT_MAX_MS` in `tdoa_gw.h`: reordering across
  `tdoa_collect_take_ready()`'s table-order (not time-order) scan when a
  tag has more than one group pending, and multi-second gaps between the
  rare blinks that actually reached `TDOA_MIN_ANCHORS` anchors for that
  specific tag (a coverage/RF fact about that tag right then, not a timing
  bug). Neither is evidence against Task 4's dt logic — the gate routing
  both to a fresh solve instead of a bogus predict is exactly correct.

**Second round, same day, same gateway, operator running one stationary tag
plus one on their arm: caught a SECOND real bug, worse than the first.**
`pos_sink`'s console line showed the stationary tag (`0x0100`) publishing the
exact same `(x, y)` to two decimals across THREE timestamps minutes apart
(`9.10, 3.74` at `00:14:32`, `00:16:23`, `00:16:43`) — a live filter re-solving
real data cannot reproduce an identical float twice. Root cause: see
`src/pos_ekf.{c,h}`'s entry above (`n_no_update`) — a stale-state republish
bug in `solve_one()`, fixed the same session. Confirmed by the arithmetic
`blink stats` itself provided: `tdoa.fixes` (114) exceeded
`seeded + filtered + reseed` (25 + 86 + 0 = 111) by exactly 3, matching the
three stale republishes visible in the log. Fixed, reflashed, reverified on
the same live gateway: `seeded + filtered + reseed` matched `fixes` exactly
in the next capture window (7 + 24 + 0 = 31 = 31), `n_no_update:0`.

**What this smoke-testing round DOES establish, that the first round did
not**: the mechanism runs stably under two real tags (one stationary, one
worn) without crashing, without a single `reseed` (the filter never diverged
past `gate_streak >= reset_after`), and — now that the republish bug is
fixed — every published fix corresponds to an actual computation that cycle,
not leftover state. It also caught a second real defect that neither host
tests nor code review found, the same way the first round did — worth
noting as a pattern, not a coincidence: this filter's failure modes only
show up under real, uncontrolled multi-tag traffic.

**Third round, same day, after the fix and a `kernel reboot cold`: clean.**
Fresh boot, filter reseeded from nothing with no crash;
`seeded(8) + filtered(19) + reseed(0) = 27` matched `tdoa.fixes:27` exactly
and `n_no_update:0` throughout — the republish fix holds across a cold
reboot, not just a warm one. One tag walking, one stationary (roles
identified by `Tid`, not short address, which reassigned across the reboot
as designed):

- **Walking tag, `Tid=693116308`**: `00:00:23.080`-`00:00:26.880`,
  `(1.31,2.78) -> (1.34,2.69) -> (1.43,2.67) -> (1.58,2.56) -> (1.65,2.52) ->
  (1.66,2.48) -> (1.60,2.44) -> (1.93,2.20)`, every consecutive 200 ms step
  under ~15 cm (~0.75 m/s, a plausible walking speed), curving direction
  tracked continuously. This is the trace-continuity acceptance criterion,
  met.
- **Stationary tag, `Tid=2082962887`**: `(3.41,2.63) -> (3.83,2.70) ->
  (4.17,2.76) -> (4.12,2.73)` over ~65 s (only 4 samples -- likely IDLE-tier
  reporting since it never moved). ~0.8 m of x-drift for something that
  should not move: no catastrophic jump this time (contrast the pre-fix
  3.5 m -> 9.1 m lock-in), but still real dispersion, above the ~45 cm
  target, and too few samples to call it a settled number.
- `solve_fail:0`, `jump:0`, `no_anchor:0`, `implausible:0` throughout; both
  one-time residual warnings (`warned_blind_residual`,
  `warned_filtered_residual`) fired exactly once as designed.

**What Task 5 has now established**: the mechanism is stable across a cold
reboot, two real tags, and both bugs this session found; the walking-trace
continuity criterion is met on real data; the stationary dispersion is
measurably better than the pre-fix failure mode but not yet at the ~45 cm
target, on a 4-sample count too small to be a real number. **What it still
has not established**: a proper before/after comparison against a
pre-session baseline (none was recorded), a settled stationary-dispersion
figure (needs a longer undisturbed capture), and which physical anchors were
actually contributing. Operator has accepted this round as sufficient to
close out this session's verification; a longer stationary-only capture is
the natural next step whenever revisited, not a blocker on what shipped
here.

**In one sentence, for whoever picks this up next:** the TDoA measurement
path is real and hardware-verified; the accuracy number is a target, not yet
checked against ground truth; and the capacity number the whole migration was
for (100 tags) has a finished design and zero lines of implementation.

## Build & flash

```powershell
$env:ZEPHYR_BASE = "C:\Users\Menay\zephyrproject\zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = "C:\Users\Menay\zephyrproject\zephyr-sdk-1.0.1"
west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

`$env:ZEPHYR_BASE` is **required** — this project lives outside the
`zephyrproject` west workspace. Zephyr 4.4.x.

**The SDK is INSIDE the workspace, at
`C:\Users\Menay\zephyrproject\zephyr-sdk-1.0.1`** — NOT at
`~/zephyr-sdk-1.0.1`, which an earlier version of this section claimed and
which does not exist on this machine. Setting `ZEPHYR_SDK_INSTALL_DIR`
explicitly is what makes a build work without relying on whatever the CMake
package registry happens to remember from a previous one.

**There is a `.venv` in `zephyrproject` and it is NOT the build
environment.** It holds only `pip` and `python` — no `west`, no Zephyr Python
requirements. The `west` that works is the SYSTEM Python's
(`AppData\Local\Programs\Python\Python312\Scripts\west.exe`), and activating
that venv changes nothing except to make it look like it should have. That
system Python is also the one whose broken TLS cert path makes
`west blobs fetch` fail, which is a separate problem with its own `curl.exe`
workaround — do not go looking for a venv to fix either of them.

There is also a separate **calibration image**, selected with an
`EXTRA_CONF_FILE` overlay rather than a runtime mode:

```powershell
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf"
west flash -d build_cal
```

Two details there are load-bearing, and both are the same ones the debug image
below documents. `-d build_cal` keeps this out of the default `build/`, so
building the calibration image does not destroy the production build — with
`--pristine` and no `-d`, it does. And the `-D` must be QUOTED, or PowerShell
splits it at the dot and the build fails looking for a file named `cal`.

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
apos tagz <metres>             the TAG plane's z in the survey frame, NEGATIVE
                               when tags sit below ceiling anchors. NOT zoff:
                               that moves the survey's z=0, this is the
                               anchor-to-tag vertical separation the TDoA solve
                               uses as dz. Persists immediately, applies to the
                               next observation. **Default 0.0 and UNMEASURED
                               on every site so far** — see below
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

The `blink` tree reports the Phase 3 observation path. Registered
unconditionally on every role and in the calibration image too, and it prints a
`role` field for the same reason `sync` does: on a role that never exercises a
counter, a static zero reads exactly like a dead link.

```
blink stats                    the TDoA observation path as JSON — the
                               anchor's stamping counters (`rx`, `no_sync`,
                               `bad`, `stamped`) and the uplink's
                               publish/subscribe counters (`published`,
                               `pub_drop`, `received`, `rx_drop` plus its
                               three-way breakdown, `sub_fail`). `no_sync ==
                               rx` is the CCP link, not the BLINK path —
                               read `sync stats` and check the gateway is on
                               USB-C. `sub_fail` climbing means the gateway
                               is DEAF: check the broker ACL.
```

The `sync` tree reports the CCP sync gate. It is registered unconditionally —
in the **production** image on every role, AND in the **calibration** image
too (`ccp_slave.c`/`ccp_master.c` compile into both; only `cal_run.c`'s own
loop never drives either). It only reports; it transmits nothing and has no
configuration commands. `stats` and `master` each print a `role` field for
exactly this reason: on a role that does not exercise them (a GATEWAY reading
`stats`, a SLAVE reading `master`, or either command on the cal image) the
underlying counters simply never moved, so a static "no-lock" or `sent:0,
dropped:0` reads exactly like a dead link unless the role is checked first.

```
sync stats                     receive half — the Phase 2 gate as JSON, with a
                               verdict; meaningful on a SLAVE. Read
                               `jitter_est`, NOT `rms` — the residual
                               differences two noisy timestamps and its RMS
                               is ~1.55x the real jitter
sync reset                     clear the residual statistics without
                               touching the lock or the baseline
sync master                    transmit half — CCP sent/dropped counts as
                               JSON; meaningful on a GATEWAY
```

## Layout

- `boards/innovaforce/ancla_esp32s3/` — out-of-tree board definition, added to
  `BOARD_ROOT` from the top-level `CMakeLists.txt`. In-tree deliberately, so it
  is version-controlled with the firmware and immune to `west update`.
- `modules/dw3000-decadriver/` — **vendored third-party** Zephyr module
  (br101/zephyr-dw3000-decadriver @ `6208d99`, containing Qorvo dwt_uwb_driver
  08.02.02), registered via `ZEPHYR_EXTRA_MODULES`. Carries three deliberate
  local deltas plus a fourth found 2026-08-30: `platform/dw3000_spi.c`
  included `"version.h"` directly, which Zephyr moved to
  `include/generated/zephyr/version.h`; only surfaced building against a
  newer Zephyr than the one this module was last touched under. Fixed by
  including `<zephyr/version.h>`. A naive upstream re-pull will silently
  drop all four.
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
  (`T_SUPERFRAME_UUS`, `BEACON_OCCUPANCY_UUS`, `BEACON_GUARD_UUS`, and
  `T_GUARD_UUS`) that the gateway schedules the beacon from and the slaves
  predict it against; no header, no code file. Now also carries the two
  `BUILD_ASSERT`s that check those hand-set constants against the airtime
  `mac_budget.h` derives — see the entry below, and note that
  `T_GUARD_UUS` (488, the 0.5 ms superframe partition guard from the MAC
  contract §2) is **not** `BEACON_GUARD_UUS` (1500, the slave's TX-suppression
  window around the beacon). Confusing the two triples the charged overhead.
- `src/blink_rx.{c,h}` — the anchor side of TDoA: stamp a tag's BLINK and hand
  the observation to the uplink. **Receive-only — it adds no UWB transmission
  to any node**, which is what makes Phase 3 survivable on hardware whose PA
  supply cannot sustain a second frame per superframe (see the battery/CCP
  entry under Hard-won facts). It holds no arithmetic of its own, the same
  split `ccp_slave.c` makes: the conversion into the master's time base is
  `sync_model_to_master()`, host-tested, and what lives here is the part a
  host test cannot reach — a real RX timestamp from a real DW3220 and the
  decision to **DISCARD** when the local clock cannot be expressed in the
  master's base. **Discarding is the correct outcome, not a failure**: an
  observation with no common time base is noise, and fed to `tdoa_solve()` it
  would move the reported position by however far the two anchors' clocks
  happen to be apart, indistinguishable from a real path difference. It is
  dropped and counted (`blink_rx_stats()`'s `n_no_sync`), never published with
  a local or guessed timestamp. `n_rx > 0` with `n_no_sync == n_rx` and
  `n_sent` flat is a board hearing tags fine while its CCP link is down —
  read `sync stats` (and check the gateway is on USB-C) before suspecting the
  radio. Dispatched from `uwb_slave.c` immediately after `ccp_slave_on_rx()`
  and before the responder chain: the CCP is what makes a BLINK convertible,
  and nothing between the timestamp read and its conversion should be allowed
  to spend time. No host suite on purpose — everything it calls is already
  host-tested and the only thing a test could reach is the counters; its
  verification is on hardware.
- `src/blink_shell.c` — the `blink` command tree (`blink stats`), the ONLY
  console surface on the observation path. Registered unconditionally and
  prints a `role` field, same precedent and same reason as `src/sync_shell.c`.
  It exists because every failure on that path is otherwise invisible and
  reads as "no tags": an anchor whose CCP link is down (`no_sync == rx`), a
  gateway whose broker ACL refused the subscription (`sub_fail`, i.e. DEAF),
  a publisher too new for this gateway's `POS_JSON_BLINK_MAX_LEN`
  (`rx_drop_oversize`), and a gateway loop too slow to drain `obs_q`
  (`rx_drop_evict`). The receive drops are split three ways on purpose —
  conflated, a format incompatibility is indistinguishable from saturation,
  and the two need opposite responses. It prints the two verdicts worth
  stating rather than leaving them to be spotted.
- `src/blink_frame.{c,h}` — the BLINK wire format for Phase 3 TDoA: a tag
  emits this instead of running a ranging sweep, every anchor that hears it
  timestamps it, and the gateway solves. Function code `0xF0`, the first code
  outside the now-full `0xEx` range (see the allocation table in
  `src/ccp_frame.h`). Deliberately NOT in `uwb_frame_802_15_4z.c`, same
  byte-identity rule as `ccp_frame.c`/`apos_frame.c` — the tag carries a
  byte-identical copy, this repo is the source of truth for it. Pure C,
  host-tested in `tests/blink_frame/`.
- `src/tdoa_collect.{c,h}` — groups the observations different anchors make of
  the SAME tag BLINK before `tdoa_solve()` ever sees them, keyed on
  `(tag_addr, blink_seq)`. Pure C, no radio and no clock of its own —
  `now_ms` is supplied by the caller on every call, and every comparison
  against it is a signed difference, so a wrapping or repeated millisecond
  counter cannot corrupt state. `blink_seq` wraps at 256, which even at 5 Hz
  is 51.2 s between two blinks sharing a value — far longer than
  `TDOA_COLLECT_WINDOW_MS` (150 ms, deliberately under one 200 ms
  superframe), so a group always closes or times out long before its `seq`
  could recur; no extra disambiguator is needed. A group releases as soon as
  it is COMPLETE (`POS_MAX_ANCHORS` distinct anchors reported) or, short of
  that, once its window has expired with at least `TDOA_MIN_ANCHORS`
  gathered — fewer than that is not resolvable in 2D and the group is
  silently discarded. A duplicate report from the same `anchor_id` for the
  same blink (an MQTT redelivery, a retry) is rejected, not folded in twice —
  doing so would hand the solver a zero-difference equation against itself
  and make its normal matrix singular. **Slot exhaustion (a 17th distinct blink
  while all `TDOA_COLLECT_SLOTS` (16) are open) evicts the LEAST COMPLETE
  group, ties broken by oldest — never the newest arrival, and never a
  group that has already reached `TDOA_MIN_ANCHORS`.** A first revision of
  this module evicted strictly by age on the theory that the oldest group
  was "furthest from completion" — backwards: the oldest group has had the
  MOST time to fill, so it is the group MOST LIKELY to already be
  releasable, and evicting it can silently destroy a fix the gateway could
  already have produced while making room for a blink that may never
  complete. Fixed in code review before this ever shipped. A group already
  RELEASABLE (`n >= TDOA_MIN_ANCHORS`) is therefore protected from eviction
  while any less-complete group exists; if every slot already holds a
  releasable group, the new observation is REJECTED rather than displacing
  one of them, since all of them are about to be drained by the next
  `tdoa_collect_take_ready()` call regardless. The `net_uplink_submit()`
  precedent for "evict to make room" is real but only partially transfers:
  that queue holds exclusively already-completed fixes, so its worst case
  is losing one finished reading, whereas this module's groups range from
  zero observations to fully resolvable — which is exactly why
  completeness, not age, has to be the primary eviction key here.
  `O(TDOA_COLLECT_SLOTS)` linear search per call, bounded and
  allocation-free, safe to call from the `K_PRIO_COOP(0)` gateway loop.
  Host-tested in `tests/tdoa_collect/`, including an end-to-end test that
  feeds a collected group straight into `tdoa_solve()`, and a dedicated
  test that proves a releasable group survives slot exhaustion that would
  have evicted it under the old, age-only policy.
- `src/tdoa_dtu.{c,h}` — turns a blink group's ABSOLUTE 40-bit master-base
  timestamps into signed differences, and bounds them. `struct tdoa_meas.t_dtu`
  as it leaves `blink_rx.c` wraps every ~17.2 s, and `tdoa_solve()` consumes
  only `t_i - t_0`, so two observations of the SAME blink on opposite sides of
  that wrap differ by ~2^40 DTU — about **5.16 million km** of path difference —
  which the solver cannot tell from a measurement. Neither `tdoa_collect` (which
  only groups) nor `tdoa_solve` (delivered and tested against rebased inputs) may
  fix it, so `tdoa_gw` calls `tdoa_dtu_rebase()` between
  `tdoa_collect_take_ready()` and `tdoa_solve()`: the project's global
  "every timestamp comparison is a signed difference" rule at modulo 2^40
  instead of 2^32. `tdoa_dtu_plausible()` then bounds every rebased `|t_dtu|` by
  `TDOA_DTU_MAX_SPREAD` (32768 DTU = 153.7 m of path difference) — a
  deliberately generous PHYSICAL bound from the deployment (apos edges measured
  1.2–2.5 m), **not** a sync-quality filter: it catches the gross case (an
  uncorrected wrap, a corrupt `t_dtu`, a clock model re-baselined mid-group) and
  is the only sanity check in front of a solver whose own residual is zero by
  construction at 3 anchors. Tightening it to, say, 20 m would make it an
  underived sync-quality gate in disguise. Pure C, host-tested in
  `tests/tdoa_dtu/`, including that an UNREBASED wrapped group is rejected.
- `src/tdoa_gw.{c,h}` — the GATEWAY consumer of the observation topic, and the
  point where Phase 3 first produces a coordinate: drain `net_uplink_get_obs()`,
  position each observation from the applied survey, group it with
  `tdoa_collect`, rebase and bound it with `tdoa_dtu`, solve with `tdoa_solve()`
  and hand a `struct pos_fix` to `pos_sink_publish()`. **Nothing downstream
  changes** — same `struct pos_fix`, same `Tid` derivation
  (`gw_core_find_eui()` + `tag_id_from_eui()`, fallback and phantom-record cost
  included), same `pos_json_fix()` payload, same frozen platform contract; the
  measurement model changed, the telemetry did not. `tdoa_gw_step()` runs **once
  per superframe** from the top of the gateway's OUTER loop, just after a beacon
  went out, and is bounded twice over (`TDOA_GW_INGEST_MAX` and
  `TDOA_GW_SOLVE_MAX`, whose values and the tag capacity they buy are below —
  named here rather than restated, so the two halves of this entry cannot
  disagree the way they did once already): it never blocks
  (`net_uplink_get_obs()` is
  `K_NO_WAIT`), never transmits and never writes flash — the four requirements
  of anything on that `K_PRIO_COOP(0)` loop. Deliberately NOT in the inner RX
  loop: observations arrive on the uplink thread's 50 ms cadence, not on RX
  events. All module state is `static` (`struct tdoa_collect` alone is 1792 B
  against a 4096 B main stack that already peaks at 1748 B in apos's
  `do_solve()`) — the same reason `gw_core_ctx` is. **`tdoa_collect_add()`
  returning false is counted APART into `reject_dup` and `reject_shed`**: a
  duplicate anchor report is harmless, whereas load shedding (every slot already
  holding a releasable group) means this module is not draining fast enough and
  otherwise looks at the console exactly like anchors going quiet. Honours
  `tdoa_solve.h`'s caller contract by **zeroing `residual_m` at
  `TDOA_MIN_ANCHORS`** rather than forwarding a number that is zero by
  construction, with one warning per boot saying so. A GATEWAY contributes no
  observation of its own (`blink_rx_init()` is called only from `uwb_slave.c`) —
  correct, not a gap: it holds reserved address `0x0000`, is not in the survey,
  and its observation could not be positioned. No host suite: its arithmetic all
  lives in `tdoa_dtu`, `tdoa_collect`, `tdoa_solve` and (since 2026-09-02)
  `pos_ekf`, which have theirs.
  Carries the one piece of new POLICY in Phase 3 that can silently DISCARD a
  valid fix, recorded here rather than left in the source: a per-tag seed memo
  (`TDOA_GW_SEED_SLOTS`, 16, LRU) remembers each tag's last fix and battery
  reading, and a fix landing more than **`TDOA_GW_MAX_JUMP_M` (10 m)** from a
  seed younger than `TDOA_GW_SEED_AGE_MS` (1000 ms) is REJECTED and counted in
  `jump`. 10 m in 1 s is 36 km/h; the gate exists because `tdoa_solve.h` warns
  that a tag outside the anchor hull can converge on the MIRROR branch and
  still report `valid`, with a residual that cannot say so. The memo therefore
  has TWO timestamps: `pos_ms` (written only with x/y, and what seed freshness
  is judged on) and `last_ms` (touched by every observation, and what LRU
  eviction uses), plus a `has_pos` flag. A single shared timestamp — which is
  what this task's brief specified — makes a tag's FIRST observation look like
  a fresh previous fix at **(0, 0)**, so that tag's first real fix is measured
  against the origin and, more than 10 m away, rejected forever with nothing in
  the log but "jumps more than 10.0 m". Caught in review before it shipped, and
  the reason this policy is documented here at all.
  `TDOA_GW_INGEST_MAX` (32) and `TDOA_GW_SOLVE_MAX` (8) sustain **8 tags at
  5 Hz over 4 anchors** — observations per superframe = anchors x blink_rate x
  tags x 0.2, groups per superframe = blink_rate x tags x 0.2, and
  `tdoa_collect_take_ready()` releases at most ONE group per call. That is the
  same ceiling `OBS_QUEUE_DEPTH` (32) imposes, so all three move together or
  not at all; past it the loss lands UPSTREAM as `rx_drop_evict`, never as
  `reject_shed`. Every per-observation and per-fix `LOG_WRN` here is **once per
  boot** (counters carry the magnitudes): this runs on the `K_PRIO_COOP(0)`
  loop, where `CONFIG_LOG_MODE_OVERFLOW` lets a steady condition destroy the
  very records an operator is reading.
  Counters read from the console with `blink stats` (second JSON line).

  **Since 2026-09-02 (`docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md`),
  every tag also carries a `pos_ekf` in its seed memo** — the same struct
  `tag_testting/src/uwb_net_runner.c` runs for TWR, moved here for TDoA use
  (see `src/pos_ekf.{c,h}`'s own entry for why this is a verbatim copy, not
  a full ownership move, contradicting what the design spec assumed). The
  memo's existing `x`/`y`/`pos_ms`/`has_pos` are UNCHANGED in purpose — the
  last PUBLISHED position, used only to seed a fresh `tdoa_solve()` and to
  jump-gate it — and are synchronised to the filter's output only at publish
  time; the running filter itself is a second, independent piece of state.
  Per group: `dt` for `pos_ekf_predict()` comes from the reference anchor's
  ABSOLUTE 40-bit `t_dtu`, captured in `solve_one()` BEFORE
  `tdoa_dtu_rebase()` converts `m[]` to signed differences, differenced
  against the tag's own last-processed value with a **local `sdelta40()`**
  (same 40-bit signed-difference discipline as `sync_model.c`'s and
  `ccp_slave.c`'s copies — no shared header for it, same precedent) rather
  than any gateway-loop timestamp: `now_ms` is quantized to the superframe
  and battles the collector's own release cadence, and feeding that to a
  constant-velocity filter would fabricate velocity that does not exist. A
  `dt` that is non-positive or exceeds `TDOA_DT_MAX_MS` (2000, ten blinks at
  5 Hz — same order as `TDOA_GW_SEED_AGE_MS`) is treated as no dt at all: a
  fresh `tdoa_solve()` reseeds instead of predicting through it. Flow, per
  the design spec's own diagram: an already-seeded filter with a valid `dt`
  gets `pos_ekf_predict()` + `pos_ekf_update_tdoa()`; anything else (cold
  start, or a dt failure) runs a fresh `tdoa_solve()` — through the SAME
  `resolve_and_seed()` helper that also handles `pos_ekf_needs_reseed()`
  recovery, so the mirror-branch jump gate (`TDOA_GW_MAX_JUMP_M`) covers
  every path that ever calls `pos_ekf_seed()`, cold start and recovery
  alike — the filter's own statistical innovation gate is what covers
  everything else, and it does not exist until a seed does. The published
  fix is always `pos_ekf_get()`'s result; `residual_m` is the solve's
  (zeroed per the `TDOA_MIN_ANCHORS` rule above) whenever a solve actually
  ran this cycle, and a flat `0.0f` with its own one-time warning on a
  purely-filtered cycle, since `pos_ekf` performs no least-squares fit for
  that number to come from. Process noise (`sigma_a_move` vs
  `sigma_a_still`) is scheduled from a hysteresis (`TDOA_GW_MOVING_ENTER_MPS`
  / `_EXIT_MPS`) on the filter's OWN velocity estimate — there is no
  accelerometer on this path — a closed loop on its own output that the
  design spec's section 4.6 accepts can stick on "moving" under high noise;
  first-cut numbers, unchecked against hardware. Six counters
  (`n_seeded`, `n_reseed`, `n_filtered`, `n_dt_invalid`, `n_gate_rejected`,
  `n_no_update` — see `tdoa_gw_ekf_stats()`'s own doc comment for exactly
  what each counts) are the THIRD `blink stats` JSON line, for the same
  reason the first two exist: without them the filter is a black box on
  the bench, no way to tell "no tags" from "the filter rejects
  everything". **`n_no_update` exists because of a real bug caught live on
  hardware, 2026-09-02**, not written in from the start: `pos_ekf_get()`
  succeeds as soon as a filter has EVER been seeded, so a cycle where `dt`
  is invalid AND the solve+seed fallback also fails (`solve_fail` or the
  mirror-branch jump gate) used to fall straight through to publishing the
  filter's UNCHANGED prior state as a "fresh" fix — caught because a test
  tag published the exact same `(x, y)` to two decimals, minutes apart, and
  confirmed independently because `blink stats`' `fixes` count exceeded
  `seeded + filtered + reseed` by precisely the number of these silent
  republishes. `solve_one()` now tracks whether the filter's state actually
  changed each cycle and suppresses the publish (counting it in
  `n_no_update`, a SUBSET of `n_dt_invalid`) when it did not. Verified
  building clean for both the production and calibration images
  (`pos_ekf.c` is in the unconditional `target_sources` block, same as
  `tdoa_gw.c` itself); `dram0_0_seg` moved 270440 -> 272160 B (+1720 B, one
  `struct pos_ekf` per `TDOA_GW_SEED_SLOTS` plus the new per-tag and static
  fields — close to but a little over the plan's ~1.3 kB estimate, which did
  not fully account for the per-tag `last_ref_t_dtu`/`has_ref_t`/`ekf_moving`
  fields alongside it). **Not yet run on hardware** — see the TDoA migration
  section's own status for what Task 5 still has to verify: this build
  result is compile-and-link correctness, not behaviour.
- `src/tdoa_solve.{c,h}` — the Phase 3 position solver: hyperbolic 2D
  multilateration by Gauss-Newton over range DIFFERENCES (`r_i - r_0`) rather
  than ranges, with anchor 0 as the reference — same closed-form 2x2 normal
  equations and determinant test as `src/pos_solver.c`, whose iteration
  constants (`POS_GN_MAX_ITERS`, `POS_GN_CONVERGE_M`, `POS_GN_DET_EPS`) it
  copies rather than re-picks. Consumes `struct tdoa_meas` (anchor position,
  `dz`, and a timestamp already converted into the common time base by
  `sync_model_to_master()`) and produces the same `struct pos_result`
  `pos_solver.h` defines. **`n` anchors give `n-1` range-difference equations
  against 2 unknowns** — the same isostatic trap `CLAUDE.md` already documents
  for the anchor survey. At `n == TDOA_MIN_ANCHORS` (3) that is exactly
  determined, so Gauss-Newton re-fits any timestamps exactly and
  `out->residual_m` reads (numerically) zero regardless of how wrong the
  input is; `out->residual_m` only becomes a real (if weak) quality signal
  from `n_used == 4`, where there is one spare equation. There is no field to
  flag this — `struct pos_result` is the tag's own copy and this task must
  not touch it — so the rule is stated as a hard caller contract in
  `tdoa_solve.h` instead: never read `out->residual_m` as evidence of fit
  quality when `out->n_used == TDOA_MIN_ANCHORS`.
  `tests/tdoa_solve/test_three_anchor_residual_is_structurally_zero()`
  demonstrates it directly: a 2 m timestamp error moves the fix 2.4 m off the
  true position while `residual_m` stays under a millimetre. Also computes
  `d_i = (t_i - t_0) * TDOA_M_PER_DTU` with the `int64_t` subtraction done
  BEFORE any float conversion — these are raw DW3220 device times up to
  ~2^40, and a float32's 24-bit mantissa only resolves ~65536 DTU (307 m) at
  that magnitude, so subtracting after converting to float would silently
  destroy the measurement while still returning a plausible-looking answer.
  Pure C, no CMSIS-DSP, host-tested in `tests/tdoa_solve/`.
- `src/ccp_frame.{c,h}` — the clock-calibration-packet wire format, function
  code `0xEF`. Deliberately NOT in `uwb_frame_802_15_4z.c` (byte-identical with
  the tag, which has no use for CCPs), same precedent `apos_frame.c` set. Pure
  C, host-tested in `tests/ccp_frame/`. **`0xEF` is the LAST free code in the
  `0xEx` range** — the allocation table is in its header, and anything after
  this needs a subtype byte under an existing code rather than a new one.
- `src/ccp_sched.h` — where the CCP falls in the superframe. Header-only, no
  `.c`, same pattern as `uwb_mac.h`; host-tested in `tests/ccp_sched/` where
  **including the header is the test**. Carries the design-history writeup for
  why the original delayed-TX schedule (a fixed `CCP_OFFSET_UUS` after the
  beacon's RMARKER, plus two `BUILD_ASSERT`s proving both edges of that
  schedule cleared the guard window) was abandoned: it failed 100% on
  hardware, because a delayed TX's arm deadline is measured against its
  RMARKER while the physical constraint is against its PREAMBLE, a whole SHR
  (1050194 ns) earlier — see the hard-won fact below for the general form of
  that trap. The now-current design (immediate TX, deferred announced
  timestamp — see `ccp_master.{c,h}`) has no scheduled RMARKER left to police
  at compile time, so `CCP_OFFSET_UUS` and the two old asserts are GONE, not
  merely unused — a live-looking constant nothing honours is worse than none.
  What remains is `CCP_SCHED_MAX_ARM_NS` (348300 ns) — the whole post-beacon
  guard window minus the CCP's own full airtime minus the beacon's own
  airtime — and one `BUILD_ASSERT` that it is positive, i.e. that the guard
  window is even wide enough to host the CCP's whole frame at all. That is a
  genuine compile-time property of the PHY and the MAC contract's guard
  sizing; whether any one arm sequence is fast enough to use the resulting
  margin is now a measured, runtime question — see `ccp_master.c`'s
  `arm_cost_ns_last`, checked against this same constant every superframe.
- `src/ccp_master.{c,h}` — the CCP transmitter on the GATEWAY, root of the sync
  tree (hop 0). On the gateway deliberately: it already schedules one delayed
  TX per superframe (the beacon), and the clock that beacon is scheduled from
  **is** the time base. A master role on a slave would have added an
  unsolicited TX path to a production image. Transmits each CCP with
  `DWT_START_TX_IMMEDIATE` right after the beacon and reads back its ACTUAL TX
  timestamp once TXFRS confirms it — the deferred-timestamp design: since a
  transmitter cannot know its own TX timestamp before transmitting, that
  timestamp travels in the FOLLOWING CCP instead, so a CCP with sequence S
  announces sequence S-1's measured timestamp. `prev_tx_dtu`/`have_prev_tx` is
  advanced ONLY on a confirmed transmit — a dropped CCP costs the announcement
  it would have carried too, not just itself, so the next CCP re-announces the
  same previous timestamp rather than one for a frame that never existed. The
  first CCP after boot sends `tx_dtu = 0` as a sentinel meaning "nothing to
  announce yet" (a genuine 40-bit timestamp landing on exactly 0 is a 2⁻⁴⁰
  event costing at most one skipped observation); the "sync txtest" probe
  reuses the same sentinel for the same reason, which is a happy accident: it
  makes a probe frame automatically inert to a receiver with no special-casing
  needed on either side. Still calls `dwt_forcetrxoff()` before every transmit
  despite there being no delayed time to arm — this is the ONLY transmit site
  in the tree that follows another TX (the beacon) rather than an RX or a cold
  start, and a completed TX, unlike a completed RX, leaves the Transmit
  Sequencing Engine in `DW_SYS_STATE_TXERR` until `CMD_TXRXOFF` clears it (see
  the hard-won fact below). Tracks `offset_ns_{min,max,last}` — where the
  CCP's ACTUAL RMARKER landed relative to the beacon's, a signed hi32
  difference, replacing the old "lateness against a schedule" instrument that
  stopped meaning anything once nothing is scheduled — and `arm_cost_ns_last`,
  the same offset with the CCP's own SHR and the beacon's own post-RMARKER
  airtime subtracted, checked against `ccp_sched.h`'s `CCP_SCHED_MAX_ARM_NS`
  with a rate-limited `LOG_WRN` (not a `BUILD_ASSERT` — the quantity is
  measured, not known at compile time) if a CCP's frame end reaches the
  earliest legitimate slave CAP preamble. Per-event drop detail logs at
  `LOG_DBG`; a rate-limited `LOG_WRN` summary of `sent`/`dropped`/the offset
  figures every `CCP_MASTER_STATS_LOG_PERIOD_SF` superframes is what actually
  survives `ANCLA_LOG_LEVEL == LOG_LEVEL_INF` in production, firing only when
  the drop count has moved. Also readable live via `sync master`.
- `src/ccp_slave.{c,h}` — the CCP receiver on the SLAVE: owns the single
  `struct sync_model` and implements the deferred-timestamp pairing protocol.
  Because a CCP with sequence S announces sequence S-1's measured timestamp,
  an observation now LAGS the frame it describes by one superframe: this
  module remembers the last frame it actually RECEIVED (`prev_seq`/
  `prev_rx_ts`) and, on receiving sequence S, checks whether that remembered
  frame IS sequence S-1 before pairing S's announcement with S-1's local RX
  time into a `sync_model_observe()` call. A CCP lost in the air therefore
  costs its receiver two things, not one: the missed frame itself, AND the
  announcement that would have completed the PRIOR frame's observation (which
  travelled only in the frame that was lost) — so `n_rx` (every CCP received)
  and the paired-observation count (`sync_model_residual_count()`, surfaced
  alongside `n_paired`/`n_no_announce` via `ccp_slave_stats_ex()`) can
  legitimately diverge on a healthy link, not just a faulty one. `tx_dtu == 0`
  is the reserved "no announcement" sentinel (boot, or a "sync txtest" probe)
  and is handled identically: remembered as the new `prev`, nothing paired.
  Detects missed CCPs from gaps between PAIRED observations (not between
  receptions) and calls `sync_model_miss()` for each one — except an interval
  large enough to look like a sequence number moving BACKWARDS (over 128; the
  ordinary trigger is a gateway reboot, which re-seeds `ccp_seq` at 0 without
  changing `root_id`), where it calls `sync_model_miss()` for none of them,
  re-baselines instead, and counts the frame in `n_reject` rather than
  `n_gap` — the whole point being that `n_gap` is the counter an operator
  reads as "the link is losing CCPs", and a benign reboot must not make it
  lie. A gateway reboot does not always wrap the interval above 128, though: a
  reboot that hits while the slave's own sequence state is itself large enough
  wraps the apparent interval back down into the ordinary 1..128 range,
  indistinguishable from a genuine forward gap **by sequence number alone**.
  So every interval in that range is also checked against the SLAVE's own
  local DW3220 clock — a genuine gap of `n` intervals must show roughly
  `n * SYNC_CCP_INTERVAL_DTU` of real elapsed local time
  (`CCP_SLAVE_GAP_TOL_FACTOR`, a deliberately generous 4x band, orders of
  magnitude past what crystal drift alone could produce), and a reboot's
  elapsed local time has no relationship to that arithmetic at all. A mismatch
  re-baselines and counts in `n_reject` the same as the large-gap case. Every
  re-baseline path — this one, the large-interval one, and a root change —
  calls `sync_model_init()`, which clears the residual statistics along with
  the rate estimate: `count` restarts at 0 and the verdict returns to
  `"insufficient"` even though `sync_model.h` documents those statistics as
  surviving "everything except an explicit reset" — true of `sync_model.c`'s
  own API, but `ccp_slave.c` is a caller that invokes `sync_model_init()`
  itself mid-life rather than only at `ccp_slave_init()`, so a re-baseline is
  a second, implicit reset from this module's callers' point of view. No
  arithmetic of its own beyond that discontinuity check — everything that
  could be wrong about the estimator lives in `sync_model.c`, which is pure C
  and host-tested and was NOT modified for this redesign: only the sequencing
  and pairing logic around it changed, never `sync_model_observe()`'s
  contract. `ccp_slave_model()` returns a `const` pointer: on a SLAVE,
  `main()` stays at the default preemptible priority (see `main.c`), so the
  shell can be preempted by the loop's own writes at any point, and read-only
  access through a const pointer is what makes that safe. The one write
  reachable from the shell (`sync reset`'s residual clear) goes through
  `ccp_slave_residual_reset()`, fenced with `k_sched_lock()`/
  `k_sched_unlock()` instead.
- `src/sync_shell.c` — the `sync` command tree. Prints the gate's verdict
  itself, because the natural mistake here (reading `rms` as if it were the
  jitter) rejects hardware that passes. Registered unconditionally, including
  in the **calibration image** — `ccp_slave.c`/`ccp_master.c` compile into
  every image regardless of `CONFIG_ANCLA_CAL_MODE`, only `cal_run.c`'s loop
  never calls either module's init or per-superframe entry point — so both
  `sync stats` (the receive half, meaningful on a SLAVE) and `sync master`
  (the transmit half, meaningful on a GATEWAY) print a `role` field: on any
  other role the underlying counters simply never moved, and a static
  `"no-lock"` / `sent:0, dropped:0` is otherwise indistinguishable from a dead
  link.
- `src/sync_model.{c,h}` — anchor clock synchronisation: converts a local
  DW3220 timestamp into a master anchor's time base from a stream of CCPs.
  Integer-only, pure C, host-tested in `tests/sync_model/`. **This is the Phase
  2 de-risk for TDoA and the whole migration turns on it** — the MAC contract
  §1 assumed sub-ns wireless sync was unachievable. Long baseline for the rate
  (copying `tag_testting/src/beacon_sched_core.c`'s 1/n scaling) plus a residual
  EMA for the phase, because the phase reference's own noise is the limiting
  term here, not the drift. The header carries the measured error tables and the
  one number the hardware gate reduces to.
- `src/mac_budget.{c,h}` — the airtime and capacity model: frame airtime, the
  SS-TWR exchange span, the turnaround floor, the SFD timeout, the cell budget
  and tag capacity, all in integer picoseconds so the expressions can live in a
  `BUILD_ASSERT`. Exists so `N_CFP`, `POLL_RX_TO_RESP_TX_DLY_UUS` and
  `BEACON_OCCUPANCY_UUS` stop being magic numbers. The macros are the contract;
  the functions are wrappers so `tests/mac_budget/` can sweep parameters. Byte
  counts **exclude** the FCS, matching `UWB_FRAME_LEN_*`. See
  `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md` §3.
- `src/gw_core.{c,h}` — seat table, leases, tag address pool, and the
  per-superframe slot schedule. Pure C, host-tested in `tests/gw_core/`. No
  longer "ported unchanged from the nRF5 gateway": **seats and slots are now
  separate things.** `seats[GW_MAX_SEATS]` (128) is indexed by seat id, and the
  beacon's slot map is an earliest-deadline-first schedule over them, rebuilt
  every superframe inside `gw_core_superframe_tick()`. Admission control counts
  slot-superframes against `GW_SCHED_CAPACITY` (275), the same accounting
  `mac_budget.h` uses. `gw_core_debug_fill_seats(c, n_dummy)` marks
  `seats[0 .. n_dummy-1]` as occupied, IDLE-tier, effectively-infinite-lease
  dummy seats before any real join, so the next real `gw_core_join()` lands at
  seat_id `n_dummy` instead of 0 — a bench aid for reaching a high seat id
  with only a handful of physical tags (used for Task 6's ceiling-seat run,
  see the TDoA migration section). The function itself is unconditional and
  host-test-safe; only its call site in `src/uwb_gateway.c`, gated on
  `CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT`, is bench-only and must never ship.
  **While active, a dummy seat's `short_addr` is deliberately outside both the
  anchor range and the real tag address pool — seat_id and short_addr are
  not a 1:1 pair on this build**, so do not assume the two can be read off
  each other from the console while dummy fill is on.
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
- **Part 2 of the TDoA accuracy work is BUILD-VERIFIED, not hardware-verified
  (2026-09-03).** Both images link clean with zero compiler warnings:
  production `dram0_0_seg` 272160 -> **272840 B (+680 B)** for the whole of
  part 2, and the calibration image links too, which is the check that
  matters there because `net_uplink.c` and `blink_shell.c` compile into it
  with `CONFIG_NETWORKING=n` while consuming the structs part 2 changed. The
  +680 B is smaller than it looks like it should be because `sigma_m` lands
  in padding `struct tdoa_meas` already had (sizeof stays 24); what actually
  grew is `tdoa_collect`'s `aid[]` (+136 B), one bool per memo slot, and a
  counter. **Nothing in part 2 has run on a board** — Tasks 1 (stationary
  baseline) and 6 (end-to-end) are bench work, and the deploy order is
  gateways -> anchors -> tags because a proto-4 gateway rejects EVERY
  proto-5 observation.

- **A 3-anchor TDoA cell has spots where y is UNOBSERVABLE, and the first
  stationary baseline was taken in one of them.** Measured 2026-09-03 on the
  deployed geometry -- origin (0,0), apex (1.752,0.920), xaxis (2.385,0) -- with
  the tag inside the triangle on the base line between origin and xaxis. The
  two range-difference equations there have `d/dy` of **-0.647** (vs the apex)
  and **-0.001** (vs the xaxis anchor): the origin/xaxis pair carries NO y
  information at all for a tag on that baseline, because moving perpendicular
  to it changes both ranges almost equally. All of y hangs on one weak
  equation. Amplification (metres of position error per metre of
  range-difference bias, largest singular value of the Jacobian's
  pseudo-inverse) is **2.09** at that spot, **1.16** at the triangle's centre,
  and up to **4.21** near the origin corner. Consequences worth keeping: a
  baseline capture on the base line measures the worst point in the array, not
  the array; **1 DTU (4.69 mm) of bias on the apex equation moves y by 8.0 mm**,
  so the -1.504 m offset that run reported needs only ~187 DTU (2.9 ns, ~0.88 m)
  of range-difference bias; and raising the apex from 0.920 m to ~1.840 m takes
  the area's worst-case amplification from 4.21 to 1.73, with tripling it adding
  almost nothing (1.33 vs 1.49) -- doubling captures nearly all of the available
  gain. That physical change is larger than any software fix on this path.
  The ~187 DTU itself is NOT attributed by this measurement: the uncalibrated RX
  antenna delay (which TDoA cannot cancel, at 4.69 mm/DTU, and whose per-board
  spread the 2026-08-28 laser campaign measured at ~207 units) and the still
  tape-unvalidated survey geometry (up to 1037 mm of disagreement recorded
  2026-08-26) are both sufficient on their own. Measuring the three edges with a
  tape is free and settles it. What this DOES rule out as the main cause is the
  height model: with this geometry a real 1.6 m separation solved at `dz = 0`
  costs 0.222 m and moves y by only -0.042, in the opposite direction to the
  one observed.

- **A console capture is not the fix stream, and the shortfall looks exactly
  like a tag going idle.** The 2026-09-03 stationary run had the gateway count
  **1424** fixes while the console delivered **291** (20.4%), the loss
  concentrated in the tail -- which reads as a still tag dropping to a slow
  reporting tier and is nothing of the kind: `ingested / anchors` shows it
  blinked at ~2.3 Hz for the whole 687 s. Zephyr logs no
  `--- N messages dropped ---` because the loss is below its accounting (the
  USB-JTAG console and the terminal program). Every cadence and gap figure read
  off such a capture is a property of the CONSOLE; dispersion and hull figures
  survive because they are per-fix, but they are a non-uniform sample.
  `tools/pos_trace.py` cross-checks its own line count against `tdoa.fixes` and
  says so, because this misreading was one sentence away from being recorded as
  a finding about dt gaps. The gateway's own `dt_invalid` (180 of 1424 = 12.6%
  on that run) IS real -- it comes off the DTU clock, not the log.

- **Per-anchor TDoA weighting publishes the MEASURED jitter, not the assumed
  one, and that distinction is a factor of ~15.** Proto 5 adds `sigma_dtu` to
  the observation: each anchor's own 1-sigma timestamp jitter, which
  `pos_ekf_update_tdoa()` turns into a per-equation `R = sigma_0^2 + sigma_k^2`
  (a range DIFFERENCE carries two independent timestamps, so the variances add
  -- the same sqrt(2) behind `r_tdoa`'s default, applied per anchor instead of
  once for all). The trap the obvious implementation walks into: the design
  spec said to publish `sync_model_error_dtu()`, but that function is computed
  from `SYNC_JITTER_DTU`, a hardcoded **assumption** of ~100 ps, while Fase 2
  measured 782 ps to 1.53 ns on real hardware. Publishing it would have made
  the filter roughly 15x overconfident in every observation -- worse than the
  flat fallback it replaced. `sync_model_jitter_est_dtu()` is the measured one,
  derived from that anchor's own CCP residuals, and is what ships. It is
  per-ANCHOR and slowly varying, NOT per-blink: it discriminates a weak or
  distant link from a good one, not one blink from the next. `sigma_m <= 0` or
  non-finite means UNKNOWN and falls back to `cfg->r_tdoa` **per anchor**, so a
  mixed-firmware fleet still gets real weighting from the anchors that report
  one; the `isfinite` half of that check is not decorative, since an
  uninitialised `struct tdoa_meas` on the stack is exactly how a NaN sigma
  arrives (it broke a host test on the way in). Weighting by `quality`
  (`ipatovAccumCount`) is deliberately NOT done: whether it carries any
  information at fixed PLEN is unmeasured, and measuring it needs a real
  capture nobody has taken yet.

- **`BLINK_FLAG_MOVING` is the tag's accelerometer, and its ABSENCE is
  indistinguishable from "still".** Proto 5 (2026-09-03) puts the LIS2HH12's
  activity verdict on the air in `blink_frame.flags` bit 1, so the gateway can
  apply `pos_ekf_zupt()` to a stationary tag -- which `pos_ekf.h` calls the
  single largest visual improvement available, because a motionless tag is the
  common case. It REPLACED a heuristic that inferred motion from the filter's
  own velocity output; that heuristic is DELETED, not kept as a fallback, since
  two criteria competing for one process-noise parameter is worse than either
  alone. The trap: an anchor still on proto 4 sends no `f` field, the parser
  leaves it 0, and 0 reads as "not moving" -- so a version-skewed fleet applies
  a ZUPT on EVERY cycle and looks exactly like a healthy stationary one.
  `blink stats`' `zupt` counter exists for that, and the shell warns when
  `zupt == filtered`. The gateway forwards the whole flags BYTE rather than a
  decoded boolean, so `BLINK_FLAG_ALERT` became visible for free and the next
  flag needs no new field.

- **The TDoA height model (`apos tagz`) is a PER-SITE number and nobody has
  measured one yet.** `tdoa_gw.c` sets each observation's `dz` to
  `anchor z - apos_store's tag_z_m`, defaulting to 0.0 — i.e. tags assumed to
  be in the anchors' own plane, which is exactly what the code did before the
  setting existed, so an unconfigured gateway is unchanged. The trap is that a
  uniform `dz` looks like it should cancel in a range DIFFERENCE and **does
  not**: `sqrt(rho^2 + dz^2)` is nonlinear, so an unmodelled separation pulls
  every reported position INWARD toward the anchor centroid. Measured
  2026-09-03 (`tests/tdoa_solve/test_height_model`): on a 10 m array a 1.4-1.5 m
  separation is a 2.5% effect (0.098 m), but on **this project's 1.2-2.5 m
  array it is ~23% — 0.230 m at 1 m from centre**, the same order as the whole
  ~45 cm accuracy target. An earlier draft of this work asserted the bias went
  OUTWARD, in three places; it was measurement, not reasoning, that settled the
  direction. Setting a wrong value is a NEW bias, which is why the default is
  0.0 rather than a plausible ceiling figure: somebody has to put a tape
  measure on the site.

- `src/pos_solver.{c,h}` and `src/pos_residual.{c,h}` — the tag's own
  range-based position solver (`pos_solve()`) and RMS residual helper
  (`pos_residual_rms()`), copied verbatim from `tag_testting/src/` for Phase 3
  TDoA work: the gateway needs the same `struct pos_result` and
  `pos_residual_rms()` the tag already uses, so the tag's copy owns the
  algorithm and this one must not drift from it — same rule as
  `uwb_frame_802_15_4z.c` and `cal_math.{c,h}`. The `cal_math.c` drift incident
  above is the reason this rule exists and not a formality: the tag's own copy
  later grew `CAL_MAX_STEP_UNITS`, and that one-sided change silently made this
  project's `-ERANGE` guard in `cal_solve.c` unreachable, returning a "success"
  antenna delay ~4.7 m wrong with nothing failing loudly. Whoever re-copies
  `pos_solver.c`/`pos_residual.c` from the tag later must diff both files
  first and treat any divergence as a decision, not a paste. The tag owns
  these files while it still solves its own fix; ownership moves to this repo
  once the gateway takes over solving (Phase 3, later task). Pure C
  (`<math.h>`, `sqrtf`/`fabsf` — the deliberate float exception to the "no
  float on time/scheduling paths" rule, since this is geometry, not a clock),
  host-tested in `tests/pos_solver/` and `tests/pos_residual/`.
- `src/pos_ekf.{c,h}` — constant-velocity EKF (state `[x,y,vx,vy]`, sequential
  scalar updates, no matrix inverse), copied verbatim from
  `tag_testting/src/` for the 2026-09-02 TDoA accuracy/smoothing work, same
  rule and same reason as `pos_solver.c`/`pos_residual.c` just above:
  **the design spec that authorised this copy assumed the tag's own copy was
  dead (Phase 3 having moved solving to the gateway), and that assumption is
  WRONG** — `tag_testting/src/uwb_net_runner.c` still runs the full
  `pos_ekf_seed()`/`predict()`/`update_ranges()`/`zupt()`/`needs_reseed()`
  sequence on its TWR-ranging path; Phase 3 added a blink-only path
  alongside it, it did not replace it. So this follows the identical
  precedent to `pos_solver.c`: the tag owns the range-based half of this
  file while it still solves its own TWR fix, ownership has not moved, and
  `tag_testting/CLAUDE.md` was deliberately left untouched. `pos_ekf_seed()`,
  `predict()`, `update_ranges()`, `zupt()`, `needs_reseed()`, `get()`,
  `pos_sigma()` and `cfg_defaults()`'s range-related fields must not drift
  from the tag's copy; diff both files before ever re-copying, same
  discipline the `cal_math.c` drift incident above exists to enforce.
  `pos_ekf_update_tdoa()` and `struct pos_ekf_cfg`'s `r_tdoa` field are new,
  added only here: a range-DIFFERENCE update over `struct tdoa_meas` with the
  same sequential-scalar mechanics as `update_ranges()`, `r_tdoa` defaulted
  to 0.6 m (derived from the Fase 2 hardware sync jitter via sqrt(2), since a
  range difference is two independent noisy timestamps rather than one — see
  the field's own comment in `pos_ekf.h`), and the same int64-before-float
  timestamp-subtraction discipline `tdoa_solve.c` documents twice, applied
  here as its own trap since it does not inherit automatically to a second
  consumer of `struct tdoa_meas`. Pure C (`<math.h>`, no CMSIS-DSP, no
  Zephyr), host-tested in `tests/pos_ekf/` — the tag's original suite,
  migrated unchanged, plus new tests for the TDoA update, the int64
  subtraction-order trap, and a direct RMS comparison against `tdoa_solve()`
  on the same noisy synthetic data (the actual point of adding this filter).
- `src/tag_id.{c,h}` — FNV-1a 32-bit hash used to derive a tag's stable
  platform identity (`Tid`) from its EUI. Pure C, host-tested in
  `tests/tag_id/`.
- `src/pos_json.{c,h}` — MQTT payload formatting. Pure C, host-tested in
  `tests/pos_json/`. The position payload is a **fixed contract** with the
  downstream consumer:
  `{"Tid":<decimal>,"x":...,"y":...,"z":0,"batt":<int>,"chg":<0|1>}`.
  `batt` is 0..100 or **-1** when the tag sent no percentage (never 255,
  which is a legal byte and reads as a real reading; never `null`, so the
  column type never varies); `chg` is 1 in exactly that case. Both are
  derived from the SAME sentinel (`UWB_FRAME_POS_SOC_CONNECTED`), so `chg`
  carries no information of its own and **also reads 1 for a failed fuel
  gauge** — the tag distinguishes the causes internally but the wire does
  not. `Tid` is
  `fix->tag_id` (`src/tag_id.c`'s `tag_id_from_eui()` of the tag's EUI, a
  **stable per-physical-device id**, resolved from the gateway's seat table
  at dispatch time — see the "stable tag identity" entry below for why this
  is not `fix->src_addr`) as a **plain decimal number** (not hex, not a
  string; e.g. `0x1234` → `4660`), `z` is the integer `0`, there is no
  `zoneName` (the
  consumer gets the zone from the anchors topic), and `residual`/`n_anchors`
  are deliberately absent. `pos_json_anchors()` takes a
  `const struct apos_survey *` and publishes the **surveyed** geometry when one
  has been applied — one entry per surveyed anchor, `node[0]` (the gauge origin)
  carrying the `apos ref` lat/long and the rest local-only in metres relative to
  it. With no survey it falls back to the original four-anchor stub
  (`ANC-LOBBY-001..004` at the corners of a 2 m × 2 m square) so an unsurveyed
  gateway still publishes a schema-valid document. The stub's coordinates are
  placeholders; its schema is the contract.
  Since Phase 3 it also carries the **TDoA observation payload** —
  `POS_JSON_TOPIC_BLINK`, `struct pos_blink_obs`, `pos_json_blink()` and
  `pos_json_blink_parse()`, host-tested separately in `tests/pos_json_blink/`
  because its consumer is our own gateway, not the customer platform. Its `ts`
  field (the 40-bit RX timestamp in the master's time base) is emitted as a
  **quoted decimal string on purpose**: the `Tid`-to-`int32` truncation above is
  what an unquoted large integer invites from a consumer, and a `t_dtu` reaches
  ~512x `INT32_MAX`. The parser is a minimal `"key":` scanner, not a JSON
  parser, and it tolerates unknown fields so a newer publisher cannot break an
  older gateway.
- `src/net_config.{c,h}` — WiFi and MQTT settings. Pure C, host-tested in
  `tests/net_config/`. Explicitly initialised from `main()`, **not** lazily like
  `uwb_config_get()`, because `net_uplink` is a second thread.
- `src/net_store.{c,h}` — the above persisted under the `net/` settings subtree.
- `src/net_shell.c` — the `net` console command tree.
- `src/net_uplink.{c,h}` — the WiFi + MQTT uplink thread and the bounded fix
  queue. **No longer GATEWAY-only: since Phase 3 it starts in BOTH modes**, and
  the header's old "a slave has nothing to publish" claim is rewritten rather
  than left standing — an anchor's BLINK observations ARE the TDoA measurement
  and there is no UWB backhaul for them. Two directions, gated on one
  `is_gateway` flag read once at thread start: a GATEWAY publishes fixes, keeps
  publishing the retained anchor map, and SUBSCRIBES to
  `POS_JSON_TOPIC_BLINK` (`net_uplink_get_obs()` drains what arrives, from the
  gateway loop); a SLAVE publishes observations (`net_uplink_submit_blink()`)
  and deliberately does NOT publish the retained map — four anchors
  overwriting one retained document would be a real fault, not redundancy.
  The subscribe lives at the end of `mqtt_bring_up()` on purpose: every
  reconnect passes through it, so re-subscribing is automatic instead of a
  forgettable special case, and a missing SUBACK aborts the connection rather
  than leaving a gateway that publishes but is silently **deaf**. Both queues
  drop the OLDEST entry when full, counted in `net_uplink_obs_stats()`.
  Note this file is in the UNCONDITIONAL `target_sources` block, so it also
  compiles into the CALIBRATION image where `CONFIG_NETWORKING=n`: every
  public symbol needs a stub in the `#else` half or the cal image fails to
  LINK.
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
  defines `CONFIG_ANCLA_CAL_MODE`, `CONFIG_ANCLA_RANGING_DEBUG`, and the
  bench-only `CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT` /
  `CONFIG_ANCLA_DEBUG_DUMMY_SEATS` pair.
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
- `docs/anchor-sync-measurement.md` — the Phase 2 gate for the TDoA migration,
  reduced to one measurement: per-observation timestamp jitter between two
  anchors, read off `sync stats`. Under ~0.5 ns Phase 3 proceeds, over ~1 ns it
  does not. Needs no reference instrument (the model measures its own input
  noise) and **does not depend on antenna calibration** — that is a bias, this
  is noise — so it runs in parallel with the calibration campaign. Also records
  what is not yet implemented: the radio glue.
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
- `tools/blink_jitter.py` — the Task 2 measurement script: parses a
  `simple_rx.c` sniffer capture and, for one tag's short address, subtracts
  each BLINK (`0xF0`) frame's RX timestamp from the beacon (`0xE5`, src
  `0x0000`) immediately preceding it in the same log, giving the tag's real
  RMARKER offset from the beacon — the quantity `BLINK_SLOT_GUARD_UUS` is
  provisional against. Uses only the sniffer's own DW3000 clock (no PC/UART
  timing, and no 40-bit wrap risk: one superframe is ~200 ms, far under the
  ~17.2 s wrap period), so plain hex subtraction is safe as long as the
  beacon precedes its blink in the log. This is what produced the **39.2 ns**
  max-spread figure behind Task 2's `BLINK_SLOT_GUARD_UUS = 100` (see the TDoA
  migration section's Task 2 bullet).
- `tools/tag_density.py` — a standalone capacity estimator: how many tags the
  MAC can track at a given update rate under TWR vs BLINK/TDoA mode, with
  every PHY/MAC constant overridable on the CLI (defaulting to today's frozen
  `uwb_phy.h`/`uwb_mac.h`/`gw_core.h`/`blink_sched.h` contract) so it stays
  useful if a future hardware revision changes the PHY. **It is a hand-written
  Python reimplementation of `src/mac_budget.{c,h}`, `src/blink_sched.{c,h}`
  and the tier-cost constants in `src/gw_core.h` — NOT wired to those C files
  and NOT host-tested against them**, the same drift risk class CLAUDE.md
  already flags for `src/cal_math.c` and `src/pos_solver.c`: if
  `mac_budget.c`'s formula, or `GW_MAX_SEATS`/`GW_SCHED_WINDOW_SF`/the tier
  table change, this file must be updated by hand or it quietly reports a
  stale number. No build-time or test-time check ties the two together.

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
- **A delayed TX's arm deadline is `DX_TIME − SHR`, not `DX_TIME` — measuring
  lateness against the RMARKER instead of the preamble start hid a 100%
  CCP-drop bug for two bench cycles.** The clock-calibration-packet (CCP,
  `0xEF`) master originally scheduled each frame with a DELAYED TX at a fixed
  offset after the beacon's RMARKER, and `dwt_starttx()` returned `DWT_ERROR`
  for every single one on the bench. The instrumentation that finally
  explained it measured the arm sequence completing 614654-678485 ns after
  the BEACON's RMARKER — comfortably inside the CCP's own ~1.538 ms offset, so
  by that measure the arm was never late. It was late anyway, because a
  frame's RMARKER sits at the END of its own SHR (1050194 ns at PLEN_1024, the
  preamble+SFD this project runs), so the deadline that actually matters is
  the PREAMBLE start, a whole SHR earlier than the RMARKER the old instrument
  was comparing against. Against that true deadline the arm was routinely
  ~190 us LATE, while simultaneously reading ~860 us EARLY against the
  RMARKER — both true at once, because they are distances to two different
  instants a full SHR apart. `ull_starttx()`'s two failure branches (the
  HPDWARN deadline check, and `DW_SYS_STATE_TXERR` — the Transmit Sequencing
  Engine left mid-TX-to-IDLE by whatever transmitted immediately before this
  arm) both collapse to the same plain `DWT_ERROR` return; there is no public
  accessor that says which one fired, and `sys_status_lo:0x00000000` /
  `hpdwarn_seen:0` on every failure was the only way to infer TXERR rather
  than HPDWARN from outside the driver. Raising the offset could not have
  fixed this either way: solving both edges of the guard window for a legal
  offset left a window narrower than the observed arm-completion jitter
  alone. The eventual fix (`src/ccp_master.c`, `src/ccp_sched.h`) does not
  raise the offset — it removes the deadline: `DWT_START_TX_IMMEDIATE` instead
  of a scheduled time, with the frame's actual TX timestamp read back after
  TXFRS and carried in the FOLLOWING frame (`src/ccp_slave.c` pairs them).
  Immune to arm jitter by construction, since there is no longer a deadline
  for jitter to miss. The general lesson: on this part, ANY reasoning about a
  delayed TX's timing must be anchored to the PREAMBLE, not the RMARKER the
  API itself schedules against — an SHR's worth of margin (over 1 ms at
  PLEN_1024) can be silently spent or silently available depending on which
  edge a given measurement is actually taken from.
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
- **A tag missing from the beacon's slot map means "not your turn", NOT "seat
  reclaimed" — since proto_ver 3.** The map used to be an ownership table and
  `seats[]` was indexed by CFP slot, so 11 slots meant 11 tags
  (`gw_core_join()` returned false for the 12th) and every seat was named in
  every superframe whatever its tier. That is why the MAC contract's §5.1 claim
  that low tiers "hand airtime back" was never true: tiers saved tag battery
  and no network capacity at all. Now the map is a schedule, 100 tags share 11
  slots, and the tag sleeps through superframes it is not named in. The wire
  FORMAT did not change — only its meaning, which is exactly why `proto_ver`
  had to move: a v2 tag reads one absence as a reclaim and tears down.
- **`gw_core`'s monotonic short-address pool is a QUARANTINE, not an oversight
  — do not turn it into prompt reuse.** `0x0100..0xFFFD` is 65278 values, so a
  freed address is not handed out again for that many joins. That gap is what
  protects the `Tid` fallback: a POS frame arriving just after its sender's
  lease expired finds no seat, falls back to `tag_id = src_addr`, and creates
  one uninformative phantom record — documented and accepted. Reuse the address
  promptly and another tag now holds it, so `gw_core_find_eui()` returns the
  WRONG tag's EUI and the straggler is attributed to a real live device
  instead. Hashing the EUI does not help: the point is that the EUI looked up
  belongs to someone else. Full reasoning is in `alloc_short_addr()`.
- **Function codes are allocated across BOTH repos, and `0xEB` was handed out
  twice.** The tag defined `UWB_FRAME_TYPE_ALERT = 0xEB` while this project had
  been using the same byte as `APOS_FRAME_TYPE` for the survey since before
  ALERT existed. The collision was exact, not approximate: `APOS_LEN_ENUM_RSP`
  is 34 bytes, identical to `UWB_FRAME_LEN_ALERT`, with the discriminating byte
  at the same offset 10 — ALERT's `state` against apos's subtype. It never bit,
  but only via two single-point saves: an `ENUM_RSP` reaching a tag is rejected
  solely because subtype `0x02` trips `UWB_ALERT_STATE_RESERVED_MASK` by one
  bit, and a HELP alert's `state = 0x01` **is** `APOS_SUB_SURVEY_BEGIN` and
  survives only on a length mismatch. ALERT moved to `0xEE` on 2026-08-25.
  Current allocation: `0xE0`–`0xEA` as before, `0xEB` apos, `0xEC`/`0xED`
  reserved for CONFIG_SET/CONFIG_ACK, `0xEE` ALERT. **Check both repos before
  claiming a code is free** — the tag's codec is the same file but its
  neighbours are not.
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
- **The ANCLA boards' ~25 dB TX deficit was a PA soldering defect, not
  firmware — RESOLVED (2026-08-25).** The symptom: measured with the sniffer
  0.5 m from the gateway, beacons arrived at −84 dBm where ~−57 dBm is
  predicted, while the tag at ~3 m arrived at −74.5 dBm against ~−72.5 dBm
  predicted — i.e. the tag matched physics and the ANCLA boards were ~25 dB
  down. That ceiling is why early range tests died at 2–3 m. **Root cause was
  bad solder joints on the QM14070 PA**, found and corrected in rework; it was
  never a firmware or configuration fault. Two corollaries worth keeping:
  (1) the `dwt_setfinegraintxseq(0)` / `dwt_setlnapamode()` ordering fix
  documented above is still **required** (the API forbids fine-grain
  sequencing with an external PA), but it was never the cause of this deficit
  and its contribution was never separately measured — do not cite this
  entry as evidence either way; (2) any board showing a large unexplained TX
  deficit should be reflow-inspected at the PA before firmware is suspected,
  since this failure looked exactly like a configuration bug from the console.
  A link budget measured before the rework is not valid — re-measure on a
  reworked board.
- **`cal_solve.c` needs TWO guards, and the newer one exists because the other
  one silently stopped working.** `cal_solve_tx_delay()` has always rejected a
  solved delay outside `CAL_TX_DLY_MIN..MAX` with `-ERANGE`, and its header
  contract says the caller must not persist such a value. Then `cal_math.c` --
  which is a **verbatim copy owned by the tag** -- gained `CAL_MAX_STEP_UNITS`
  (2000), bounding one iteration's correction so a corrupted sample cannot swing
  the delay ~32000 units at once. Individually both are right. Together, the
  clamp **defeated the range check**: a saturated step no longer overshoots the
  window, it lands comfortably inside it. Concretely
  `cal_solve_tx_delay(12000, 2000, 16385, 16385, &tx)` is a 10 m measurement
  error that used to return `-ERANGE`; with the clamp it returned **0 with
  tx = 18385**, a success carrying a delay ~4.7 m wrong, which the calling
  procedure would have written to NVS. Verified by neutralising the guard and
  watching `CHECK(tx != 18385)` fail. So `cal_solve.c` now checks
  `|measured_mm - ref_mm| > CAL_MAX_STEP_MM` (4680 mm, **derived** from
  `CAL_MAX_STEP_UNITS * CAL_MM_PER_UNIT_X1000` rather than restated, since the
  tag may change those) **before** the range test, and reports the bound the
  measurement was pushing toward. The general lesson, worth more than the fix: a
  bound added in a copied dependency can make a caller's own validity check
  unreachable, and nothing fails loudly when it does — the anchor's
  `tests/cal_solve/` went on passing because it only asserted the tag's
  selftest vectors. When re-copying `cal_math` from the tag, re-check that
  `CAL_MAX_STEP_MM` still bounds what the solver actually saturates at.
- **An SS-TWR exchange is NOT `poll_airtime + turnaround + resp_airtime` — that
  form double-counts both SHRs and inflates the span by ~1194 us per exchange
  at this PHY.** `POLL_RX_TO_RESP_TX_DLY_UUS` is measured **RMARKER to
  RMARKER**, and the RMARKER sits at the *end* of its own SHR, so the poll's
  payload and the whole response SHR both fall **inside** the turnaround
  window rather than beside it. The correct span is `poll SHR + turnaround +
  (response PHR + payload + FCS)` = **3330.9 us**, so a 4-anchor sweep is
  **13.32 ms**, not the 18.1 ms the naive form gives. This matters because the
  wrong figure makes `N_CFP = 11` look like it barely fits (10.5 slots) when
  the budget actually affords **14** and the shipped constant is conservative
  with 3 slots of headroom. Encoded in `MAC_SSTWR_EXCHANGE_PS()`
  (`src/mac_budget.h`) and pinned both ways — correct and naive — by
  `test_sstwr_exchange()`. The MAC contract's own hand estimate of
  `T_slot ~= 15 ms` was right; it was the design spec's first draft that got
  this wrong, and the model is what caught it.
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
- **Stable tag identity (Tid) is derived from the tag's EUI, not its MAC
  short address — the two are not interchangeable, and this was a real
  field bug.** `gw_core_superframe_tick()` wipes a seat's entire record,
  EUI included, the instant its lease ages to zero, and `alloc_short_addr()`
  is a bare monotonic counter with no memory of previously-issued
  addresses. So any rejoin after a lease gap — including a plain battery
  disconnect, which guarantees the old lease can't be renewed in time —
  got a brand-new, permanently higher short address, and the platform saw
  one physical tag as several different `Tid` values over its lifetime
  (`0x0101`, `0x0102`, `0x0103`, ... observed directly on the bench).
  Fixed without touching the wire format or the tag firmware at all: the
  gateway already learns a tag's full EUI at JOIN (`struct gw_seat.eui`),
  and a tag can only be transmitting a POS frame while some gateway is
  tracking it, so `uwb_gateway.c`'s POS dispatch looks up the sender's EUI
  from the seat table (`gw_core_find_eui()`) and hashes it (FNV-1a 32-bit,
  `tag_id_from_eui()`) into `fix.tag_id`, which `pos_json_fix()` now
  publishes as `Tid` instead of `fix.src_addr`. A single EUI half (either
  32-bit `NRF_FICR->DEVICEID` word alone) was deliberately rejected as the
  source: Nordic only guarantees the *combined* 64-bit value unique, and a
  single half can plausibly repeat within one production batch since these
  words are typically wafer/lot/die-position derived — hash the whole
  8-byte EUI, never a slice of it. **`gw_core`'s POS dispatch is
  deliberately not gated on seat state** (`uwb_gateway.c`'s existing
  comment on the `uwb_frame_is_pos` branch) — a fix can arrive after its
  sender's lease, and therefore its seat and the EUI in it, has already
  expired. `gw_core_find_eui()` returning false is this documented case,
  not a new error: the fallback is `tag_id = src_addr` for that one
  straggler fix, logged, and the fix is still published — never dropped.
  The **per-frame value** of that fallback matches today's pre-fix
  behavior exactly, not a regression — but the **system-level
  consequence** is genuinely new, and small: under the old code every fix
  from a given tag carried the same `Tid` (its `src_addr`), so a straggler
  after lease expiry was still correctly attributed. Under the new code,
  that straggler's `Tid` (`src_addr`) differs from every other fix that
  same tag has ever sent (`hash(EUI)`), so the platform sees a one-record
  "phantom device" for that single frame — a narrow, accepted cost, not a
  strict narrowing of an existing gap. This is why `Tid`'s stability is a
  strong guarantee for a live tag and a best-effort one for a fix that
  lands in the same superframe as a lease expiry, not an absolute one.
  Three more honesty notes worth keeping on hand rather than re-deriving:
  (1) the accepted hash-collision bound — the space is **2^31, not 2^32**
  (see the int32 entry below), so P(any collision) is ~44% at 50,000
  devices, ~2.3% at 10,000 and ~0.02% at 1,000 — "negligible" holds for a
  single site or a few-thousand-unit fleet and is an accepted tradeoff at
  large scale, not a mathematical guarantee; (2) a fallback `Tid` (`= src_addr`, which lives in
  `0x0100..0xFFFD`) has roughly a 1.5×10⁻⁵ per-tag chance of coincidentally
  landing on some other real tag's hashed `tag_id` — in that rare case the
  straggler would be silently misattributed to a real (wrong) device
  rather than merely creating an uninformative phantom record, the one
  path by which the fallback can be silently wrong about a real tag, not
  just uninformative; (3) an all-zero EUI (unprogrammed FICR) is accepted
  by `gw_core_join()` and would hash to a fixed constant value shared by
  every such tag — not a new hole this fix introduces (the old code had
  the identical issue via `src_addr` for that scenario), but still true
  under the new scheme.

- **`Tid` must fit a POSITIVE signed 32-bit integer — the platform's column
  is `int32`, not `uint32`, and it drops anything above `INT32_MAX` in
  silence.** Diagnosed on the bench 2026-08-24 with three tags connected:
  the two at `Tid` 693116308 and 2082962887 appeared on the platform and
  the one at 2728562623 (`0xA2A28FBF`, the only one with bit 31 set) never
  did. **The gateway was blameless and proving that first is what made this
  quick** — `struct pos_fix.tag_id` is `uint32_t`, `uwb_gateway.c` fills it
  from `tag_id_from_eui()`, `pos_sink.c`'s console line and
  `pos_json_fix()` both format `%u`, and `net_uplink.c` publishes the
  buffer verbatim; compiling `pos_json.c` on the host and running that
  exact value through it emits `{"Tid":2728562623,...}`, correct and
  unsigned. So the truncation is downstream, in a consumer this project
  does not own and cannot change. Not a float64/2^53 rounding problem
  either — all three values are exactly representable as doubles; the
  boundary is specifically 2^31. Fixed at the only place that can enforce
  it structurally: `tag_id_from_eui()` now returns `hash & 0x7FFFFFFF`.
  **Masking, not remapping, and that choice is load-bearing** — the mask is
  the identity for every hash that already fits, so tags already visible on
  the platform keep their existing `Tid` and only the broken ones move
  (2728562623 → 581078975). A modulo or a fold would have renumbered every
  tag at once and orphaned every existing platform record. The other
  `fix.tag_id` writer, the `= src_addr` straggler fallback, is a `uint16_t`
  and can never exceed the bound, so both assignment sites are covered by
  construction. Cost is the halved id space, already folded into the
  collision bound above. `tests/tag_id/` pins the range as a contract, and
  its FNV-1a reference vectors are written as `<published value> &
  0x7FFFFFFF` so they still check the hash itself rather than only the
  masking layered on top.

- **A 2588-byte `struct gw_core_ctx` as a stack AUTOMATIC overflowed the
  4096-byte main stack, silently, and only while `do_solve()` ran.** This cost
  four bench sessions and three wrong instruments, so the whole shape is worth
  keeping. `uwb_gateway_run()` declared `struct gw_core_ctx ctx;` as an
  ordinary local. That was ~236 bytes when the anchor survey was written and
  reviewed — `seats[]` was indexed by CFP slot, so `GW_N_CFP` (11) of them.
  The seat/schedule split for 100-tag capacity made it `seats[GW_MAX_SEATS]`
  (128) and `sizeof` went to **2588**. Measured afterwards with
  `CONFIG_THREAD_ANALYZER`: `main` peaks at 1220 B idle and **1748 B** once
  `do_solve()` → `apos_geom_solve()` → `apos_geom_refine()` has run. So with
  `ctx` on the stack the real figures were 3808/4096 at rest — **fits, 288 B
  spare** — and **4336/4096 during the solve, an overflow of 240 bytes.** That
  is exactly the observed behaviour: the board ran for a minute and died the
  instant the last ranging pair completed, because that is when `do_solve()`
  runs at all. Fixed by making it `static`, which is *correct* and not merely
  roomier: `uwb_gateway_run()` is called once from `main()` and never returns,
  so there is one instance either way (confirmed at the link: production dram
  259440 → 262032 B, +2592, the struct moving to `.bss`). Three lessons that
  generalise past this bug:
  **(a) The absence of a `k_timer` ISR report is evidence AGAINST scheduler
  starvation, not a gap in the instrument.** Three instruments were spent
  chasing the theory that the `K_PRIO_COOP(0)` loop was starving the
  preemptible shell and log threads. An ISR preempts a busy-wait at any
  priority, so an ISR that stops reporting means corrupted kernel state or a
  halt — never starvation. That datum was in hand and was read as a hole in the
  tooling.
  **(b) In deferred logging, an absent line is NOT an absent step.** `LOG_INF`
  enqueues and returns; if the thread dies before its next yield, the record
  sits in a pool the preemptible log thread never drains, and
  `CONFIG_LOG_MODE_OVERFLOW` then overwrites it. The console showed pair 5 of 6
  as the last event when the firmware had gone considerably further. Do not
  count missing lines as steps that did not execute.
  **(c) Host tests cannot see this class of bug and neither can a code review
  of either change.** `gw_core` is host-tested where stacks are megabytes, the
  survey had never been run on hardware, and the two changes lived on different
  branches. `CONFIG_THREAD_ANALYZER` (in `debug.conf`) is the only thing that
  answers it, because arithmetic cannot: Xtensa's windowed ABI spills register
  windows on deep call chains on top of every declared frame, so the true
  high-water mark is always above what `-fstack-usage` can show.
- **`CONFIG_LOG_MODE_IMMEDIATE` is unusable on this build — do not re-add it.**
  Tried in `debug.conf` to get the last line out before a freeze. It failed
  twice, each worse than the problem: (1) a synchronous console write from the
  `K_PRIO_COOP(0)` loop costs milliseconds against a 5 ms
  `BEACON_ARM_MARGIN_UUS`, so **every** delayed beacon missed its slot and the
  re-base dragged the cadence to ~210.7 ms against a 200 ms
  `T_SUPERFRAME_UUS`; (2) it crashed the board ~1.8 s into boot as WiFi
  associated — `EXCCAUSE 20`, `PC 0x640`, current thread `idle` — a garbage PC
  from a corrupted stack, most likely immediate mode's inline `cbprintf`
  running in a context borrowing the 1024-byte `CONFIG_IDLE_STACK_SIZE` while
  the WiFi blob logged heavily. Deferred mode only memcpys from the caller,
  which is why production never sees it. The thing immediate mode was reached
  for is served instead by the stall watchdog's deliberate `k_msleep(1)`: it
  enqueues its report and then sleeps, which is what lets the preemptible log
  thread drain. Note `k_msleep`, **not** `k_yield()` — yielding only reaches
  threads at this priority or above, and every thread that could print the line
  or take a console command is below it.
  `log_panic()` is declared in `zephyr/logging/log_ctrl.h`, not `log.h`.
- **`sysworkq` peaked at 948 of the default 1024 bytes (92 %) with `dwt_isr()`
  on it.** Measured on the gateway under an `apos run`. `br101` runs the DW3000
  interrupt handler from the system workqueue rather than the GPIO callback, and
  `dwt_isr()` reads CIA diagnostics over SPI, so this is the radio path's stack.
  76 bytes spare is not a margin, and it is the same failure class as the
  `gw_core_ctx` overflow above. `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE` is now 2048
  in `prj.conf`. Neither `CONFIG_STACK_SENTINEL` nor hardware stack protection
  is on in production (Xtensa has no `ARCH_HAS_STACK_PROTECTION` here), so an
  overflow anywhere in this firmware is silent — `debug.conf` turns the software
  sentinel on for exactly that reason.
- **A user-facing warning that hardcodes a formula will be wrong the first time
  the other mode runs.** `apos apply`'s unverified-survey warning said "usable
  edges minus **3N-6**" unconditionally, so the first real 2D survey printed the
  3D free-parameter formula — in the single warning an operator is most likely to
  check the arithmetic of. `apos_gw.c`'s two JSON warnings already selected on
  `res.dim`; only the shell copy did not. Fixed by reading
  `apos_gw_result()->dim`. Worth generalising: `res.dim` now has three consumers
  that must agree, and the shell one is the easiest to forget.
- **A delayed TX armed immediately after another TX fails `dwt_starttx()`
  deterministically, and the fix is `dwt_forcetrxoff()` before arming, not a
  timing change.** `ccp_master_after_beacon()` (`src/ccp_master.c`) arms the
  clock-calibration-packet's delayed TX right after `tx_beacon()`'s beacon TX
  completes, and on the bench `dwt_starttx(DWT_START_TX_DELAYED)` returned
  `DWT_ERROR` for 100% of CCPs — `sent:0`, `dropped` climbing by one every
  superframe, forever. The first hypothesis was a missed arm deadline; the
  lateness instrumentation added specifically to test that (Change 2 in the
  same function) refuted it outright: `nonpositive_late` equalled `dropped`
  on all 249 samples with `late_ns_max:0`, meaning the arm completed at or
  before the scheduled hi32 every single time. The real cause is in
  `ull_starttx()` (`modules/dw3000-decadriver/dwt_uwb_driver/dw3000/dw3000_device.c`
  ~line 5061), which has **two** distinct failure branches behind one
  `DWT_ERROR` return: HPDWARN (a genuinely missed deadline, which the
  lateness measurement ruled out), and — the one actually firing here —
  `SYS_STATE_LO` reading `DW_SYS_STATE_TXERR` (`0xD0000`, documented in
  `dw3000_deca_vals.h:154` as "TSE is in TX but TX is in IDLE in SYS_STATE_LO
  register"). That is exactly the state the chip sits in immediately after a
  transmission completes and before `CMD_TXRXOFF` has returned the Transmit
  Sequencing Engine to IDLE. Every other delayed-TX site in this tree already
  calls `dwt_forcetrxoff()` before arming (`src/anchor_respond.c:133`,
  `src/apos_gw.c:207`, `src/apos_node.c:136`, `src/ss_initiator.c:99`) —
  `tx_beacon()` in `src/uwb_gateway.c` is the only OTHER exception, and it is
  safe only because a completed **RX**, not a TX, always precedes it, and RX
  leaves the TSE in IDLE on its own. The CCP is the one delayed TX in the
  whole tree armed directly after another TX, so it inherited TXERR every
  time. Fixed by adding the same `dwt_forcetrxoff()` call the other sites
  already have, immediately before `dwt_setdelayedtrxtime()`. It is cheap,
  not a new wait: `ull_forcetrxoff()` only issues `CMD_TXRXOFF`, and skips
  even that write when the part is already idle (documented at
  `src/ss_initiator.c:99-101`). The general lesson: `ull_starttx()`'s two
  failure paths are indistinguishable from the bare `DWT_ERROR` return —
  there is no public accessor in `deca_device_api.h` that reports which one
  fired or exposes `SYS_STATE_LO` directly — so a zero-lateness, nonzero-drop
  signature is currently the only way from outside the driver to tell TXERR
  apart from a real missed deadline; do not assume a `DWT_ERROR` on a
  delayed TX is automatically a timing problem.
- **A battery-powered gateway cannot sustain the CCP's second PA-driven TX per
  superframe — this is a deployment constraint, not a bug, and it finally
  isolates a mechanism an older entry here only suspected.** Controlled
  bench comparison, same firmware and same board, only the supply changed:
  on a 3.6 V LiPo the gateway emits beacons only and no CCPs ever go out; on
  USB-C both transmit normally. The CCP (`0xEF`) is a second PA-driven
  transmission ~289 us after the beacon's own — see the "arm deadline"
  entries just above for why it is armed immediately after the beacon TX —
  and that back-to-back double-TX burst is exactly the case the older
  hard-won fact "A board that transmits exactly one frame per boot and then
  goes silent is a wedged DW3220" (above, under "This project's own GATEWAY
  mode") flagged as its prime suspect ("the supply sagging under the PA's
  draw at the first TX") without ever isolating it — that episode was closed
  by swapping the board, not by measuring the rail. This experiment isolates
  it: same board, same code, only the power source differs, and only the
  second TX is missing. **Every Phase 2 sync measurement is therefore
  supply-dependent** — a `jitter_est` or a `sent`/`dropped` count means
  nothing unless the power source is recorded alongside it (see
  `docs/anchor-sync-measurement.md` §4.1). The fix is hardware — bulk
  capacitance, LiPo ESR, regulator droop under the ~289 us double-TX burst —
  not firmware; the diagnostic is a scope on the supply rail across the
  beacon-then-CCP pair, not a log line. **The operator trap: on battery this
  presents as `sync master`'s `sent:0` with `dropped` climbing every
  superframe**, which is indistinguishable at the console from the TXERR /
  missed-arm-deadline failures the entries above describe — check the power
  source before chasing either of those as a firmware fault.
- **The Phase 2 sync gate was run on hardware (2026-08-26) and FAILED against
  its original 1 ns target; the target was consciously re-derived rather than
  the hardware being called adequate.** Both readings on USB-C power, after
  `sync reset`: 30 cm gave `jitter_est` 50 DTU = 782 ps (`marginal`, n=436);
  3 m gave 92-98 DTU = 1.44-1.53 ns (`fail`, n=221 and n=357). `gaps:0` and
  `rejected:0` across ~2000 receptions throughout. Fitting
  `sigma^2 = floor^2 + (k*d)^2` to the two points gives a floor of ~49 DTU
  (~772 ps) — already above the 32 DTU pass threshold on its own — with the
  link term only ~8 DTU at 30 cm rising to ~81 DTU at 3 m, i.e. **97% of the
  30 cm variance is a floor, not the link budget**; a purely SNR-limited
  jitter would have risen ~10x for the 10x distance change and it only rose
  1.9x. This is a two-point fit, not a proven decomposition. `sync reset`
  matters: an un-reset reading came back 188 DTU (3.7x too high) because the
  residual sum still carried observations from before the rate estimate
  converged (`SYNC_BASELINE_USEFUL` is 10) — skipping it would read as a
  hardware failure well beyond the one actually measured. Decision (see
  `docs/anchor-sync-measurement.md` §4.1 for the full readout): the 1 ns
  target targeted 10-30 cm TDoA accuracy; at ~1.5 ns the honest range error
  is ~45 cm (1 ns ~= 30 cm). Product decision: **~45 cm is accepted for now,
  Phase 3 proceeds at that accuracy, and it is intended to improve.** Two
  levers are identified and neither has been run: (1) send the CCP every 2
  superframes instead of every 1 and re-measure at 30 cm — if the floor moves
  proportionally with the interval it is crystal wander over that interval
  (§5 remedy 2), if it does not move it is per-observation timestamp noise
  and a hardware limit; deliberately lengthening rather than shortening,
  because shortening needs a second TX per superframe and the entry above
  says the supply already cannot sustain the one it has; (2) the link term
  above matters at deployment range and is addressable through TX power,
  antennas and anchor spacing — CLAUDE.md's own "~25 dB TX deficit" entry
  above records that both these boards' PA rework status has not been
  confirmed, and an unreworked board would inflate this term. §5 remedy 3
  (raising `SYNC_PHASE_EMA_SHIFT`) has its own trap: `SYNC_RESIDUAL_TO_JITTER`
  (1550) is empirical for shift = 3, so raising the shift lowers RMS while
  making the reported `jitter_est` wrong unless the constant is re-derived —
  `tests/sync_model/` pins the relation. The roles-swapped second direction
  in `docs/anchor-sync-measurement.md` §3 has **not** been run.

- **A four-anchor survey plus a laser distance meter calibrates antenna delay
  BETTER than `cal ref` does, and it is the only method here that is
  falsifiable.** Measured 2026-08-28 on the four-anchor array, in full LOS,
  after every board had already been through `cal ref` and passed a `cal peer`
  cross-check. `apos run` still returned `accepted:0` with `rms_mm` 65 and
  edges disagreeing with a laser by -130 to +519 mm. Fitting the per-node model
  `error_ij = b_i + b_j` to the six laser-referenced edges is **6 equations
  against 4 unknowns — overdetermined, so it can fail**. It did not: the RMS
  collapsed from **312.5 mm to 24.1 mm** (13x), every residual inside +/-33 mm,
  which is the level of the per-edge `sd` (16-68 mm) already being reported.
  So the error was per-BOARD antenna delay, not geometry, not NLOS (the array
  was in clear line of sight), and not clock offset (`max_reciprocal_mm` 24
  bounds the residual CFO at ~0.04 ppm against a +/-20 ppm crystal). The solved
  biases were `0x0001` +321 mm, `0x0003` +231 mm, `0x0002` +67 mm, `0x0004`
  -164 mm -- and `0x0001`/`0x0003` were exactly the two boards the operator had
  independently noticed "always differ", which is what turned a hunch into a
  number. **Conversion: 2.3459 mm of range per unit of antenna delay**
  (`c * 15.65 ps / 2`; one DTU is 4.69 mm of path but only half of it lands in
  an SS-TWR range -- the same "half a tick per unit" the antenna-delay bullet
  in "URGENT next work" derives). Reading LONG means the configured delay is
  too SMALL, so the correction ADDS.
  **Why `cal ref` cannot reach this answer, structurally**: it drives
  `(b_anchor + b_ref)` to zero and can never observe `b_ref` alone, so the
  reference node's own delay error transfers into every board it calibrates,
  invisibly and identically. Two consequences worth keeping. (1) Every anchor
  calibrated against ONE reference should end up at `b = -b_ref`, i.e. all
  EQUAL, and pairwise errors should then be a CONSTANT on every pair. A
  measured SPREAD between boards -- 485 mm here -- therefore proves the
  persisted `ant_delay_tx` values are not what calibration should have
  produced, whatever the console said at the time; check the live values with
  `anchor show` before re-deriving anything. (2) A laser gives ABSOLUTE
  distances, so the per-node fit is gauge-free and needs no reference node at
  all: `e_ij = b_i + b_j` over K4 has no null space (adding `c` to every `b`
  moves every edge by `2c`), so the four biases come out absolutely, not
  relatively.
  **This calibrates only the SUM `ant_tx + ant_rx`, which is all SS-TWR can
  observe and all TWR ranging needs. It constrains the TX/RX SPLIT not at
  all** -- and the split is exactly what Phase 3 TDoA depends on, since an
  anchor there only ever receives and nothing cancels: the observable carries
  `(dR_i - dR_0)`, the DIFFERENCE in RX-delay error between anchors, at
  **4.69 mm per DTU with nothing to cancel it**. Every board here still has
  `ant_delay_rx` pinned at the factory 16385 while the whole correction went
  into TX, so that difference is raw uncalibrated manufacturing spread. No TWR
  measurement of any kind can constrain it. Recorded here because the TDoA
  plan does not yet account for it.
  Two things this does NOT establish, stated so they are not assumed: the
  corrected delays had not been applied or re-surveyed when this was written,
  and the fit's own residual (24 mm) is a floor on what the method can resolve,
  not a proven accuracy.

- **A collector that releases groups in TABLE order can make a
  constant-velocity filter WORSE than no filter, and the console gives no
  hint of it.** Measured 2026-09-03 with the `blink trace` ring on a
  stationary tag over 128 cycles: FILTERED fixes had dispersion RMS
  **0.512 m** against the raw `tdoa_solve()` output's **0.345 m**. The
  filter was amplifying, not smoothing. Two defects, harmless alone:
  `tdoa_collect_take_ready()` returned the first releasable group its table
  scan reached (slot-allocation order, unrelated to time) while
  `tdoa_gw_step()` drains up to `TDOA_GW_SOLVE_MAX` (8) groups per
  superframe, so several came out scrambled; and `solve_one()` advanced
  `mm->last_ref_t_dtu` unconditionally, **including for a group whose dt came
  back negative**, so an out-of-order group rewound the dt reference and the
  next group's dt then spanned time already integrated. The trace shows it
  directly — three groups at one `t_ms` with dt 0.400 / 0.200 / 0.600 s, i.e.
  **1.2 s of prediction charged for 200 ms of elapsed time**, and the
  position ramping 2.45 -> 3.02 over three cycles before collapsing 1.6 m.
  Fixed by releasing oldest-first (now a documented part of `take_ready()`'s
  CONTRACT, since the caller's dt depends on it) and by discarding a
  small-negative-dt group outright while HOLDING the reference (counted as
  `reorder` on `blink stats`). Three general lessons: (a) `first_ms` is
  gateway ARRIVAL, not tag emission, so oldest-first is strictly better than
  table order but is **not** a total order — the caller keeps a guard for the
  residue; (b) **a large negative dt is the opposite case and must NOT be
  treated as reordering** — it is a forward gap that aliased through
  `sdelta40()`'s +/-8.6 s boundary, and the same capture shows real 9-15 s
  gaps (slow reporting tier) reading as -7.007 / -8.007 / -8.415 s; treating
  those as reordering wedges the reference permanently, a regression worse
  than the defect, which is what `TDOA_DT_REORDER_MAX_MS` (1000) separates;
  (c) the hypothesis this replaced — that the overconfident published sigma
  closed the innovation gate and the filter coasted — was **killed by the
  same capture**, which reported `gate_rejected: 0`. Host tests and code
  review had both missed this; the instrument that found it was the temporary
  trace ring, third time in this migration that a defect was only reachable
  from real multi-tag traffic.

- **RX antenna delay is calibrated from the raw blink topic against
  tape-measured tag positions — and the fit is falsifiable, which is the
  whole reason it is worth trusting.** `cal ref`/`cal peer` calibrate the SUM
  `ant_tx + ant_rx` because that is all SS-TWR observes; TDoA depends on the
  SPLIT, which they constrain not at all. The observable: the DW3220
  SUBTRACTS `ant_rx` from every RX timestamp, and `T_emit` (with the tag's
  own TX delay) is common to every anchor hearing one blink, so it cancels in
  a range difference, leaving
  `(tau_k - tau_0)*4.69mm - (r_k - r_0) = -(delta_k - delta_0)*4.69mm` — a
  **constant, independent of where the tag is**. So the fit is a plain
  per-anchor mean, no matrix; and estimated independently at several
  tape-measured positions, a genuine offset must come out IDENTICAL at every
  one. Drift with position means geometry or tag height instead, so
  `tools/rx_cal.py` prints the cross-spot spread and refuses to emit an apply
  command when it is large — verified both ways on synthetic data (+150 DTU
  recovered as +147.9 with 33 DTU of timestamp noise and 220 blinks x 4
  spots; the same data fitted with a wrong `--tag-z` turns a 5-8 DTU spread
  into 136-162 and is correctly refused). Three things worth not
  re-deriving. **(1) No firmware is needed at either end**: the input is
  already published on `uwb/anchor/blink/<zone>` (`"a"` and `"ts"` are the
  anchor id and its master-base RX timestamp) and `anchor ant` already
  applies a correction. **(2) Apply by holding the SUM and moving only the
  SPLIT** (`ant_rx += b`, `ant_tx -= b`): SS-TWR sees only the sum, so TWR
  ranging, the `apos` survey and the existing `cal ref` result all come out
  unchanged — which is what makes the campaign safe on an already-calibrated
  fleet, and is why it does NOT contradict CLAUDE.md's standing rule that
  changing `ant_delay_rx` alone decalibrates a board. **(3) The sensitivity
  is asymmetric and easy to get backwards**: one unit of `ant_rx` is a FULL
  4.69 mm in TDoA (a one-way timestamp) but 2.35 mm in SS-TWR (half of each
  unit lands in the round trip). Only DIFFERENCES are observable, which is
  not a limitation — a common RX offset is invisible to TDoA — so the
  lowest-id anchor is pinned as the gauge and left untouched. Procedure:
  `docs/rx-antenna-delay-calibration.md`. **Not yet run on hardware.**

## Reporting progress on multi-task work

The operator asked (2026-08-28) to be given, for any task being worked on, its
**percentage complete, its elapsed duration, and an estimated finish** — and for
those estimates to be grounded rather than invented. Convention, so the numbers
mean the same thing every time:

- **Percent** is stages finished out of the stages a task actually has, not a
  feeling. A task under the SDD loop has four: implement, review, fix round,
  scoped re-review. A task with a clean first review has three and skips
  straight from review to done. Say which denominator is in use, because a task
  at "2 of 3" and one at "2 of 4" are not equally far along.
- **Duration** is wall-clock agent time, taken from the actual run, never
  estimated after the fact.
- **Estimated finish** is the per-stage medians below multiplied by the stages
  remaining. Label it an estimate every time. It is a median of a sample of two
  tasks, so treat it as an order of magnitude, not a commitment.
- **Report the estimate again when it moves**, not only at the start. An
  estimate quietly left stale is worse than none.

Measured on this plan (Phase 3, Tasks 5 and 6, one agent per stage):

| stage             | T5     | T6     | median |
|-------------------|--------|--------|--------|
| implement         | 17.7 m | 16.8 m | 17.3 m |
| review            | 11.7 m | 11.3 m | 11.5 m |
| fix round         | died   |  7.9 m |  7.9 m |
| scoped re-review  |  5.9 m |  6.8 m |  6.3 m |
| **full cycle**    | —      | **42.8 m** | **~43 m** |

Three things that distort those figures and should be said out loud rather than
buried in a margin:

- **Orchestration between stages is not counted.** Building the review package,
  ruling on findings, verifying claims independently and writing the ledger all
  happen between agents and add materially to wall clock.
- **Agents die.** Three subagent deaths on the org monthly spend limit happened
  during this plan alone, one of them mid-fix-round. A dead agent is not lost
  work — it had already committed, and the recovery is to VERIFY and resume,
  never to restart — but it does add an unpredictable stage.
- **A task that needs no fix round finishes in about 35 minutes, not 43.** The
  fix round is the variance, and whether a task needs one is not knowable in
  advance.

Above all: these are estimates of when the CODE is written and reviewed. They
say nothing about when it WORKS, because nothing here is verified on hardware
until a board runs it. Do not let a schedule figure be read as a completion
figure.

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

gcc -Wall -Wextra -Isrc -o tests/pos_json_blink/test_pos_json_blink.exe tests/pos_json_blink/test_pos_json_blink.c src/pos_json.c -lm
./tests/pos_json_blink/test_pos_json_blink.exe  # pos_json_blink: ALL TESTS PASSED, exits 0

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

gcc -Wall -Wextra -Isrc -o tests/tag_id/test_tag_id.exe tests/tag_id/test_tag_id.c src/tag_id.c
./tests/tag_id/test_tag_id.exe                  # OK, exits 0

gcc -Wall -Wextra -Isrc -o tests/mac_budget/test_mac_budget.exe tests/mac_budget/test_mac_budget.c src/mac_budget.c
./tests/mac_budget/test_mac_budget.exe          # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/sync_model/test_sync_model.exe tests/sync_model/test_sync_model.c src/sync_model.c
./tests/sync_model/test_sync_model.exe          # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/ccp_frame/test_ccp_frame.exe tests/ccp_frame/test_ccp_frame.c src/ccp_frame.c
./tests/ccp_frame/test_ccp_frame.exe            # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/blink_frame/test_blink_frame.exe tests/blink_frame/test_blink_frame.c src/blink_frame.c
./tests/blink_frame/test_blink_frame.exe        # blink_frame: ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim -o tests/mac_budget/test_uwb_mac_asserts.exe tests/mac_budget/test_uwb_mac_asserts.c src/mac_budget.c
./tests/mac_budget/test_uwb_mac_asserts.exe     # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim -o tests/ccp_sched/test_ccp_sched.exe tests/ccp_sched/test_ccp_sched.c src/mac_budget.c
./tests/ccp_sched/test_ccp_sched.exe            # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/pos_residual/test_pos_residual.exe tests/pos_residual/test_pos_residual.c src/pos_residual.c -lm
./tests/pos_residual/test_pos_residual.exe      # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/pos_solver/test_pos_solver.exe tests/pos_solver/test_pos_solver.c src/pos_solver.c src/pos_residual.c -lm
./tests/pos_solver/test_pos_solver.exe          # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/tdoa_collect/test_tdoa_collect.exe tests/tdoa_collect/test_tdoa_collect.c src/tdoa_collect.c src/tdoa_solve.c src/pos_residual.c -lm
./tests/tdoa_collect/test_tdoa_collect.exe      # tdoa_collect: ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/tdoa_dtu/test_tdoa_dtu.exe tests/tdoa_dtu/test_tdoa_dtu.c src/tdoa_dtu.c -lm
./tests/tdoa_dtu/test_tdoa_dtu.exe              # tdoa_dtu: ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/tdoa_solve/test_tdoa_solve.exe tests/tdoa_solve/test_tdoa_solve.c src/tdoa_solve.c src/pos_residual.c -lm
./tests/tdoa_solve/test_tdoa_solve.exe          # tdoa_solve: ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/pos_ekf/test_pos_ekf.exe tests/pos_ekf/test_pos_ekf.c src/pos_ekf.c src/pos_solver.c src/pos_residual.c src/tdoa_solve.c -lm
./tests/pos_ekf/test_pos_ekf.exe                # ALL TESTS PASSED, exits 0
```

`-lm` is required by the suites that link `apos_geom.c`, `pos_solver.c`,
`pos_residual.c` or `tdoa_solve.c` — they call `sqrtf`/`fabsf`. The others do
not need it. `tests/tdoa_solve/` links `pos_residual.c` only because its build
line matches the sibling `pos_solver` suite's shape for consistency; the
solver itself does not call into it — see the entry on `src/tdoa_solve.c`
above for why it computes its own range-DIFFERENCE residual instead.

`tests/mac_budget/test_uwb_mac_asserts.c` needs `-Itests/mac_budget/shim`, which
supplies a `zephyr/sys/util.h` defining `BUILD_ASSERT` as `_Static_assert`. That
is the whole point of that test: **including `src/uwb_mac.h` is the test**, so a
budget that no longer holds fails to *compile* under plain gcc instead of
waiting for a Zephyr build. `tests/ccp_sched/` needs the same shim for the same
reason: it includes `src/ccp_sched.h`, whose `BUILD_ASSERT`s check where the CCP
sits against the beacon's own frame length. Same shim pattern as the tag's
`tests/uwb_radio_owner/shim`.

## Repo

`origin` is `https://github.com/JoseLara03/ANCLA_ESP32S3.git`. Work happens on
feature branches (`feat/rtls-scale-tdoa` as of 2026-08-31), pushed to `origin`;
`master` is the base branch. This corrects an earlier version of this section
that said no remote was configured — true once, not since.

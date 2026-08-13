# Antenna-delay calibration

**Date:** 2026-08-13
**Status:** design approved, not yet implemented
**Branch:** `cal/antenna-delay` off `master`
**Addresses:** URGENT next work item 1 in `CLAUDE.md`

---

## 1. Motivation

Every ANCLA board still runs the factory seed `ant_tx = ant_rx = 16385`
(`UWB_ANT_DELAY_DEFAULT`), never trimmed on this hardware. The measured effect is
recorded in `CLAUDE.md`: solving over a 1 m anchor geometry produced residuals of
1.9–2.0 m in one bench session and 0.48–0.73 m in another, with the reported
`(x, y)` wandering ~0.7 m between consecutive fixes with nothing moving. A
residual larger than the array itself means the three ranges disagree wildly.

The chain works end to end — a tag joins, discovers, ranges, solves, reports
`0xEA`, and the gateway republishes over MQTT — but until the delays are trimmed,
no coordinate this system emits means anything.

This spec produces a dedicated calibration firmware image and the console
procedure that trims each anchor against an OTP-calibrated DWM3001CDK, plus an
independent inter-anchor cross-check that proves the result rather than assuming
it.

## 2. Scope

**In scope:** a separate `ANCLA_CAL_MODE` build; an SS-TWR *initiator* role for
the anchor; batched ranging with outlier rejection; the delay solve; persistence
through the existing `uwb_store`; a two-command `cal` console tree; a host test;
the operator procedure; and the correction to the `RX_ANT_DLY` claim in
`CLAUDE.md` (§8).

**Out of scope:** anchor auto-positioning (URGENT item 2 — coordinates stay
hand-entered and still silently default to `(0,0)`); any change to production
SLAVE or GATEWAY firmware behaviour; tag firmware changes of any kind; and DS-TWR
(§4.4). Tag calibration is a sequel, not part of this branch (§9).

## 3. The physics, restated correctly

This section exists because `CLAUDE.md` currently states the reason wrongly, and
the wrong reason will mislead whoever next touches `ant_delay_rx`.

### 3.1 Only the sum `ant_tx + ant_rx` is observable

`CLAUDE.md` says *"Only TX needs trimming: `RX_ANT_DLY` cancels in `RTD_resp`, so
it drops out of the SS-TWR result."* The first half is true and the second is
not.

`RTD_resp` **as a number** is indeed independent of `RX_ANT_DLY`.
`anchor_respond.c:138-141` computes

```
resp_tx_time = (poll_rx_ts + POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8
resp_tx_ts   = ((resp_tx_time & 0xFFFFFFFE) << 8) + cfg->ant_delay_tx
```

so `resp_tx_ts - poll_rx_ts` is exactly `D + ant_delay_tx`. But the delayed TX
time is *derived from* `poll_rx_ts`, which the hardware has already shifted by
`RX_ANT_DLY`. Raising `ant_delay_rx` by one tick therefore makes the anchor
physically transmit one tick **earlier**, which shortens the initiator's
`rtd_init` by one tick, which moves the ToF estimate by half a tick — exactly as
much as raising `ant_delay_tx` by one tick moves it in the same direction by
growing `rtd_resp`.

Writing `e_tx = a_tx - τ_tx` and `e_rx = a_rx - τ_rx` for the programmed-minus-
physical error at each node, the full SS-TWR result is

```
ToF_est = ToF_true - (E_initiator + E_responder) / 2,   E = e_tx + e_rx
```

Both roles contribute the same combined quantity. The tag already models it this
way: `cal_math.h:53` splits a single *combined* delay into TX and RX halves.

**Consequences that this design depends on:**

1. Putting the whole correction into `ant_delay_tx` while pinning `ant_delay_rx`
   at 16385 is *equivalent*, not merely conventional. It is safe.
2. `ant_delay_rx` is not a free parameter. It is part of the calibrated quantity
   and must not be changed independently after calibration.
3. A delay measured while a board is **initiating** is the same delay that
   applies when it later **responds** to the tag. This is what makes the
   anchor-to-anchor cross-check in §6.3 a valid test of the tag-facing path.

### 3.2 Slope

One unit of combined delay is `c × DWT_TIME_UNITS / 2 ≈ 2.34 mm` of reported
distance, negative-going: increasing the delay decreases the reported distance.
The tag stores this as `CAL_MM_PER_UNIT_X1000 = 2340` (`cal_math.h:15`); the
sibling ESP-IDF project stores the reciprocal as `COUNTS_PER_MM ≈ 0.42653`
(`anchor_cal.c:35`). They agree. This spec uses the tag's constant (§5.1).

### 3.3 Clock-offset correction is mandatory

SS-TWR range error from crystal offset is `c/2 × ppm × T_reply`. At this
project's fixed 2000 µus responder turnaround, 5 ppm is ~1.5 m — larger than the
antenna-delay error being measured. Uncorrected, the calibration would be
garbage.

Both existing implementations already correct for it and the correction ports
unchanged:

- tag: `uwb_ss_initiator.c:237-246`
- sibling ESP-IDF anchor: `ranging.c:301-305`

```c
double clk_off = ((double)dwt_readclockoffset()) / (double)(1U << 26);
double tof = ((rtd_init - rtd_resp * (1.0 - clk_off)) / 2.0) * DWT_TIME_UNITS;
```

## 4. Approach

### 4.1 Reference node

The DWM3001CDK carries a factory-calibrated antenna delay in OTP. It will run
Qorvo's stock `ss_twr_responder` example, PHY-matched to this project's contract
(channel 5, PLEN_1024, PAC32, code 9, 850 kbps, SFD_IEEE_4Z, STS/PDoA off), and
configured to load the OTP delay rather than the example's hardcoded `16385`.

**This is a dependency outside this repository.** If the OTP value turns out not
to be loaded, or the PHY is not matched, the cross-check in §6.3 is what will
reveal it — see §7.

### 4.2 Why not the three-node mesh

With three mutually-uncalibrated anchors and no reference, a two-node
measurement only ever yields the *sum* of two unknowns, and the escape is the
Qorvo APS014 three-node solve: three pairwise measurements at known distances,
`err_ij = D_i + D_j`, solved as `D_A = (err_AB + err_AC − err_BC)/2`.

A trustworthy reference dissolves that problem into three independent two-node
calibrations. The mesh solver is therefore **not built**. It remains the fallback
if §4.1's assumption fails.

### 4.3 Why the sibling's `cal self` cannot simply be ported

`CLAUDE.md` recommends porting the sibling's procedure rather than re-deriving
it. That holds for the *statistics and the solve*, but two things do not carry
over:

- Its cal path reads response timestamps at bytes **10/14** (`ranging.c:81-82`),
  the stock Qorvo layout. The ANCLA's own VEWA response puts them at **11/15**,
  because `anchor_respond.c:67` inserts an anchor-id byte at index 10. An
  initiator that must talk to both peers has to handle both layouts (§5.2).
- Its `CAL_INIT_RX_AFTER_TX_UUS`/`CAL_INIT_RESP_TIMEOUT_UUS` (300/2000) are sized
  for a ~450 µus stock responder turnaround, not the ANCLA's 2000 µus.

### 4.4 Why SS-TWR and not DS-TWR

`CLAUDE.md` lists DS-TWR between anchors as the approach to evaluate for
*auto-positioning* (URGENT item 2, Phase 5 of the sibling's roadmap). It is
deliberately not used here. SS-TWR is what the tag actually uses, so the delay
this procedure solves is the delay that matters on the production path, and the
existing WAVE responder is already a complete, bench-confirmed peer for it. With
clock-offset correction applied, SS-TWR's known weakness is removed.

## 5. Components

### 5.1 `src/cal_math.{c,h}` — copied verbatim from the tag

Copied byte-for-byte from `tag_testting/src/cal_math.{c,h}`, on the same
reasoning `CLAUDE.md` gives for `uwb_frame_802_15_4z.c`: both ends of this link
should solve with the identical slope constant and the identical outlier
rejection, and a stripped fork will drift.

Used here:

- `cal_filtered_mean(samples, n, &mean, &kept)` — mean after median/MAD outlier
  rejection, capped at `CAL_MAX_SAMPLES` (128).
- `cal_solve_step(measured_mm, ref_mm, cur_total_dly)` — new **combined** delay.

Deliberately unused: the `cal_record` persistence struct, `cal_crc32`,
`cal_record_finalize`, `cal_record_valid`, `cal_split_dly`. The anchor persists
through `uwb_store` and pins `ant_delay_rx` (§5.5), so it needs neither the
record nor the split. The file is kept whole rather than forked.

Anchor-specific arithmetic goes in `src/cal_solve.{c,h}` (§5.5) rather than being
appended here, so this file stays a verbatim copy.

### 5.2 `src/cal_initiator.{c,h}` — one SS-TWR exchange

```c
/* Returns distance in mm, or INT32_MIN on no/invalid response. */
int32_t cal_initiator_range(uint8_t peer_wire_id);
void    cal_initiator_enter(void);   /* forcetrxoff, mask, rx-after-tx, timeout */
void    cal_initiator_leave(void);   /* restore responder RX state */
```

**Poll frame — one format serves both peers.** 11 payload bytes: the 10 stock
Qorvo header bytes `41 88 <seq> CA DE 'W' 'A' 'V' 'E' E0` plus a peer-id byte at
index 10.

- The DWM's stock responder compares only the first `ALL_MSG_COMMON_LEN = 10`
  bytes and ignores the eleventh.
- The ANCLA responder *requires* it and filters on it
  (`anchor_respond.c:124,134`), so exactly one peer ever answers and the third
  board stays silent. No responder code changes.

**Response — discriminated by payload length.**

| Peer | Response payload | `poll_rx_ts` idx | `resp_tx_ts` idx |
|---|---|---|---|
| DWM3001CDK, stock Qorvo responder | 18 B | 10 | 14 |
| ANCLA anchor (`anchor_respond.c:73`) | 27 B | 11 | 15 |

Payload length is `dwt_getframelength() - FCS_LEN` (`CLAUDE.md`: the frame length
includes the FCS). Anything that is neither 18 nor 27, or whose first 10 bytes do
not match the VEWA reference, is discarded as an error sample.

**Radio state.** `dwt_setrxaftertxdelay(300)` and `dwt_setrxtimeout(4000)` (µus)
cover both the DWM's ~450 µus and the ANCLA's 2000 µus turnaround. Poll is sent
with `DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED`.

**Interrupt mask.** The initiator needs TXFRS *and* the RX events, unlike the
production responder which deliberately keeps TXFRS out of the mask
(`uwb_slave.c:188`). `cal_initiator_enter()`/`_leave()` own that transition, and
the initiator waits on callbacks rather than polling TXFRS, so the hazard
`CLAUDE.md` records for `tx_delayed()` does not apply. Every wait is bounded
(10 ms for TXFRS, 25 ms for the RX result, matching the sibling).

**Clock offset** applied per §3.3.

### 5.3 `src/cal_run.c` — the cal-image main loop

Responder by default, calling the production `anchor_respond_wave_poll()` so the
responder logic is not duplicated. Wakes on a shell-posted request, calls
`cal_initiator_enter()`, runs the batch, calls `cal_initiator_leave()`, re-arms
RX.

It needs its own event loop (~60 lines) rather than reusing `uwb_slave_run()`,
for two reasons:

- `uwb_slave_run()` blocks on `k_sem_take(&rx_sem, K_FOREVER)`
  (`uwb_slave.c:201`) with no injection point, and adding one would edit a
  bench-confirmed production path.
- Running the exchange on the shell thread instead would put two threads on the
  SPI bus to the DW3220 concurrently.

`beacon_guard` is passed as `NULL`: there is no gateway on air during
calibration, and `beacon_guard_tx_allowed()` returns `true` while unlocked
(`beacon_guard.c:37`) in any case.

The batch runs at the main thread's default priority, **not** `K_PRIO_COOP(0)`.
Cooperative priority is a GATEWAY-mode requirement (`main.c:71`) and would starve
the console for the duration of the batch.

### 5.4 `src/cal_shell.c` — two commands

```
cal ref  <mm>        128 exchanges against the DWM reference at <mm>,
                     solve, apply and persist
cal peer <id> <mm>   128 exchanges against ANCLA anchor <id>, report only —
                     never persists
```

No `cal show` or `cal apply`: `anchor show` already prints the active delays and
`anchor ant <tx> <rx>` already provides manual override.

Output per run: sample count, kept count after outlier rejection, mean measured
mm, reference mm, error mm, and for `cal ref` the old and new `ant_delay_tx`.

### 5.5 `src/cal_solve.{c,h}` and persistence

The one piece of arithmetic this project adds on top of the tag's solver: convert
a solved *combined* delay back to a TX-only value with `ant_delay_rx` pinned.
Pure C, no Zephyr, no driver headers — so it is host-testable like every other
pure module in this project.

```c
/* Solve a new ant_delay_tx from a batch mean, holding cur_rx fixed.
 * Returns 0 on success, -ERANGE if the result had to be clamped
 * (reported as a failure by the caller, not silently accepted). */
int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm,
                       uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx);
```

Body: `new_total = cal_solve_step(measured_mm, ref_mm, cur_tx + cur_rx)`, then
`*out_tx = new_total - cur_rx`, clamped to `[14000, 19000]` — the sibling's range
(`anchor_cal.h:23`), about ±5.9 m of correction.

`cal ref` calls it and stores the result through the existing
`uwb_config_set_ant()` + `uwb_store` path that `anchor ant` uses, leaving
`ant_delay_rx` at 16385.

Applied on the next boot, consistent with the documented "every setter persists
immediately, but changes take effect only on reboot" contract. `uwb_slave_run()`
snapshots the config at mode entry precisely so a live edit cannot desynchronise
`resp_tx_ts` arithmetic from the programmed antenna delay (`uwb_slave.c:168-176`);
the cal image follows the same rule.

## 6. Build and procedure

### 6.1 Build separation

A separate image rather than a runtime mode: a deployed anchor must never
transmit unsolicited polls, and the cal image has no use for WiFi, MQTT or the
gateway.

- New root `Kconfig` — the project has none today — declaring `ANCLA_CAL_MODE`
  and sourcing `Kconfig.zephyr`.
- New `cal.conf` overlay: `CONFIG_ANCLA_CAL_MODE=y`, WiFi and MQTT off.
- `CMakeLists.txt` gains a `target_sources_ifdef(CONFIG_ANCLA_CAL_MODE app
  PRIVATE ...)` block; the existing unconditional list is untouched.
- `main.c` gains one `#ifdef CONFIG_ANCLA_CAL_MODE` branch dispatching to
  `cal_run(cfg)` ahead of the existing mode check at `main.c:58`. No production
  path is modified.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu -- -DEXTRA_CONF_FILE=cal.conf
west flash
```

All three anchors run the cal image during a session: the board being calibrated
initiates, the others sit as ordinary WAVE responders.

### 6.2 Physical setup

DWM3001CDK at a tape-measured distance from the anchor under test, **≥ 2 m**,
clear line of sight, antennas at equal height and matched orientation, clear of
metal.

Measure **antenna phase centre to antenna phase centre**, not board edge to board
edge. Tape error enters the calibration 1:1 — 5 mm of sloppiness is 5 mm of
permanent bias on that anchor.

Tags powered off, so nothing else polls the air.

### 6.3 Sequence

1. For each anchor in turn: `cal ref <mm>`, then `cal ref <mm>` again. The second
   run's reported error is the residual.
2. After all three are trimmed, cross-check every anchor pair with
   `cal peer <id> <mm>` at tape-measured distances. These pairs were never used
   to calibrate anything, so their residual is independent evidence.

## 7. Success criteria

1. `tests/cal_solve/` passes under host gcc, following the project's existing
   pattern:
   ```powershell
   gcc -Wall -Wextra -Isrc -o tests/cal_solve/test_cal_solve.exe tests/cal_solve/test_cal_solve.c src/cal_solve.c src/cal_math.c
   ./tests/cal_solve/test_cal_solve.exe
   ```
   Invokes the tag's `cal_math_selftest()` vectors, then covers §5.5's
   combined-to-TX-only conversion: sign of the correction in both directions, the
   2.34 mm/unit slope, `cur_rx` held fixed, and `-ERANGE` at both clamp edges.
2. The second `cal ref` run on each anchor reports `|error| < 15 mm`.
3. **Acceptance test.** `cal peer` on each of the three anchor pairs reports
   `|error| < 30 mm`, on pairs never used to calibrate anything.
4. End to end: the `residual` field in the tag's `0xEA` frames — logged by
   `pos_sink` — collapses from the 0.48–2.0 m recorded in `CLAUDE.md` to under
   ~0.1 m, and consecutive fixes stop wandering with nothing moving.

Criterion 3 is the one that matters. It is what distinguishes a real calibration
from three anchors that agree with a bad reference: if the DWM's OTP delay were
not actually loaded, criterion 2 would still pass on every board while criterion
3 failed by roughly twice the reference's own error.

If 3 passes but 4 does not, the remaining error is geometry — URGENT item 2, not
this branch.

## 8. Documentation changes

- `CLAUDE.md`: replace the `RX_ANT_DLY cancels in RTD_resp` claim with §3.1. The
  conclusion it supports ("trim TX only") is correct and stays; the reason is
  wrong and the corrected statement — only the sum is observable, so
  `ant_delay_rx` is not free — is what a future reader needs.
- `CLAUDE.md`: record the calibration build, the `cal` console tree, and the new
  host test alongside the others.
- A short operator procedure doc under `docs/`, so §6 is runnable without reading
  this spec.

## 9. Follow-on work, not in this branch

- **Tag calibration.** Once one anchor is trimmed, the tag calibrates against it
  with its existing BLE `cal <mm>` command (`tag_testting/src/cal.h:33`) — no
  firmware work on either side. This should be done immediately after this
  branch lands, because criterion 4 above measures the *pair*.
- **Anchor auto-positioning** (URGENT item 2) is untouched. Coordinates remain
  hand-entered and still silently default to `(0,0)`, and the retained
  `uwb/anchor/setup/<zone>` payload remains the `ANC-LOBBY-001..004` stub.
- **The three-node mesh solver** if §4.1's reference assumption fails.

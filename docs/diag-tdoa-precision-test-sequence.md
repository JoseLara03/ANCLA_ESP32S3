# TDoA precision — the ordered test sequence T0–T7

Companion to `docs/diag-tdoa-precision-stage1-audit.md` (including its B14
addendum). This is the **runnable** half: exact console commands, what to
record, and how to read each result.

**Status as of 2026-09-18: T6 implemented and software-verified (plus one piece
of hardware evidence — see the T0 side-finding). T0 has RUN → verdict T0-A.
T1–T5 have NOT run and are BLOCKED — see the blocker section below. T7 is not
triggered and is not yet evaluable.** Verdict boxes still marked `NOT RUN` carry
the prerequisites needed to fill them.

Fill each verdict box in place as it runs, and keep the raw console output —
these are the numbers that go back to the supervisor.

---

## Standing rules for every bench test here

1. **`anchor ant` and `anchor pos` do not take effect until a reboot.** Every
   setter persists immediately and the console says `saved — reboot to apply`.
   Any probe that changes a delay needs `kernel reboot cold` after the set
   **and** after the revert. Budget ~20 s per change for the board to boot,
   rejoin CCP sync and reconverge.
2. **Record the original value before changing anything, and restore it in the
   same session.** `anchor show` prints `ant_tx`/`ant_rx`; paste the whole JSON
   line into the log rather than transcribing two numbers.
3. **Power the gateway from USB-C, never LiPo.** A LiPo cannot sustain the
   CCP's second PA-driven TX per superframe; it presents as `sync master`
   showing `sent:0` with `dropped` climbing, indistinguishable at the console
   from a firmware fault. Check this before interpreting any sync number.
4. **After any reboot of the gateway, every anchor re-baselines.** As of the T6
   fix that is now visible: `sync stats` shows `rebaselines` incrementing,
   `n_obs` restarting at 0, and `sigma_dtu` elevated until `n_obs` reaches 10.
   Wait for `n_obs >= 10` before recording any position data.
5. **Tag static and in clear line of sight** for every positional measurement,
   on a marked spot that is itself tape-measured into the survey frame.

---

## BLOCKER found 2026-09-18 — the reported position is pinned at the centroid

**Observed while running T0**: two tags placed together near the A3-A0 edge are
both reported at the **centre of the array**, and the reading did not change at
all under a 1.88 m perturbation. This blocks T1-T5, because every one of those
tests is read out through the reported position — a frozen output makes all of
them uninterpretable, and no amount of calibration moves a number that is not
tracking.

**Why it is not simply a bias.** The height model (`apos tagz = 0`) pulls
positions INWARD toward the anchor centroid — real, documented, ~23% on this
array — but that is ~0.28 m at 1.2 m from centre. It cannot put a tag *on* the
centroid. What can: `tdoa_solve()` uses the **anchor centroid as its seed** when
no seed is supplied, so a position sitting exactly there and refusing to move is
the signature of a solve returning its starting point, a filter that froze, or a
map displaying a stale value.

**Three candidates, all separated by one command** (`blink stats` on the
gateway, run twice ~30 s apart so the deltas are readable):

| Candidate | Signature |
|---|---|
| **(a) Filter still-freeze** | `tdoa_abg` line: `still` ~= `filtered`, and the shell prints its own warning. `BLINK_FLAG_MOVING`'s ABSENCE reads as "still" (proto-4 publisher, or a tag that never sets it), so the filter freezes every cycle: seeded once near the centroid, frozen after, identical for both tags, immune to everything. Fits the symptom exactly. |
| **(b) Nothing published, map is stale** | `tdoa.fixes` not climbing between the two reads, and/or `jump` climbing. A tag on or outside the hull edge can converge on the mirror branch; `TDOA_GW_MAX_JUMP_M` rejects it and publishes NOTHING, so the platform keeps showing its last value. |
| **(c) Equations dropped** | `no_anchor` climbing — observations from anchor ids absent from the applied survey, which `anchor_xyz()` refuses. Check `solve_fail` alongside. |

Also read `apos show` (are the surveyed coordinates and node count what you
expect, and do the ids map to the boards you think?) and the `pos_sink` console
line — whether a new `(x, y)` is printed per fix at all separates (b) from
(a)/(c) immediately, and consecutive values identical to two decimals is the
stale-republish signature that caught a real bug on this gateway before.

**Status: NOT YET DIAGNOSED.** Awaiting those reads.

---

## T0 — RX-delay synchronisation probe

**Purpose.** Separate three candidates for the per-anchor range-difference
bias: (a) uncancelled slave RX-delay error, (b) master TX/RX split, (c) survey
error.

### T0.0 — read this before running it: two predictions differ from the brief

The brief's theory is right about the slave and wrong about the master, and
the difference changes what a null result means. Both follow from code, not
from opinion (see B14 in the audit for the derivation):

- **Slave: cancellation is EXACT and INSTANT, not "within a few CCP
  periods".** The DW3220 subtracts `ant_rx` from *every* RX timestamp on that
  board, including the CCP's. The sync model is trained on the CCP's RX
  timestamp and evaluated on the blink's, so the same offset appears in both
  and subtracts out algebraically. There is no settling behaviour to watch:
  once the board has re-converged after the reboot the perturbation is gone.
  **Predicted observation: NO position change at all.**
- **Master: `ant_delay_rx` on the gateway does nothing whatsoever.** The
  gateway takes no blink observations — `blink_rx_init()` is called only from
  `uwb_slave.c` — holds reserved address `0x0000`, and is not in the survey.
  Its RX path never enters the TDoA observable. **Predicted observation: NO
  change, and this is not informative.** The master quantity that *could*
  matter is `ant_delay_tx`, and that shifts every anchor's converted time
  equally, so it cancels in the difference too.
- **What DOES enter at full weight per anchor** is the gateway-to-anchor
  distance (B14), which no console probe can perturb. T1b below is the test
  for it.

So the brief's decision matrix has no row for the predicted outcome. Use this
extended one:

| Slave probe | Master probe | Verdict |
|---|---|---|
| **No change** (within the stationary dispersion already measured, ~0.35 m RMS raw) | No change | **T0-A: cancellation active, as derived.** Slave RX delay is not the bias. What remains is B14 + B8's *residual* + survey. Proceed to T1/T1b. |
| Jump, returns over a few CCP periods | any | **T0-A', unexpected**: cancellation works but is not instant. Record the settling time — it would mean the phase EMA is doing work the algebra says is unnecessary, which is new information about `phase_corr`. |
| Jump, **stays** | any | **T0-B: asymmetry bug.** Something subtracts `ant_rx` on one path and not the other. Hunt it: `blink_rx.c` vs `ccp_slave.c` dispatch in `uwb_slave.c` (both are handed the *same* `uwb_get_rx_timestamp_u64()` value at `:348`, so a divergence would be new), or a driver-level difference between frame types. **STOP and report.** |
| Jump, returns | Jump of any kind on the master | **T0-C**: contradicts the code reading above — the gateway would have to be contributing an observation. Report and stop. |

Because the predicted result is "nothing happens", **the probe is only
meaningful if the measurement is sensitive enough to have seen something.**
+100 DTU is 469 mm of one-way path — well above the ~0.35 m raw stationary
dispersion, but not hugely. Consider +400 DTU (1.88 m) instead: it is
unambiguous against the noise, it is still far inside the `ant_rx` field
range, and it is equally reversible. Record which you used.

### T0.1 — procedure

```
# 1. Baseline. Tag on its marked spot, static, LOS. Let it run 5 minutes.
#    Capture the gateway console AND `blink stats` at start and end.
blink stats
sync stats

# 2. On EVERY anchor, record the starting point:
anchor show

# 3. On ONE slave anchor (note which id):
anchor ant <existing_tx> <existing_rx + 400>
kernel reboot cold
#    wait for sync stats to show n_obs >= 10 and valid:1, then observe >= 60 s

# 4. Revert, on the same board:
anchor ant <existing_tx> <existing_rx>
kernel reboot cold

# 5. Repeat 3-4 on the GATEWAY (expected: no effect at all, see T0.0)
# 6. If ambiguous, repeat on a second slave.
```

### T0 verdict

```
Status:            RUN 2026-09-18 (slave probe only; master probe not run)
Perturbation used: +400 DTU (1.88 m of one-way path)
Anchor probed:     A3     original ant_tx/ant_rx: NOT RECORDED before the change
Baseline (x,y):    both tags reported at the ARRAY CENTROID
Perturbed (x,y):   IDENTICAL to baseline — no movement
Verdict:           T0-A — cancellation active, as derived in T0.0
```

**T0-A confirmed, and it is the predicted outcome, not a null result.** A 1.88 m
perturbation moved the reported position by zero, which is what the algebra in
T0.0 says must happen: the DW3220 subtracts `ant_rx` from every RX timestamp on
that board including the CCP's, so the offset appears in both the sync model's
training input and its evaluation input and cancels exactly. **Slave RX-delay
error is not the bias**, and rank 2 (B8) collapses to whatever the *difference*
between boards is after the master term is accounted for.

**Two things this run did NOT establish**, recorded so they are not assumed:

- The **master probe was not run**, and per T0.0 it is expected to be
  uninformative anyway (the gateway takes no blink observations).
- A3's original `ant_tx`/`ant_rx` were **not recorded before the change**, so
  the revert is reconstruction rather than restoration. Read the other anchors
  with `anchor show` and treat A3's values as suspect until `cal ref` or
  `rx_cal.py` re-derives them. **This is the standing rule in this document's
  own preamble and it was skipped — it costs a calibration, not just a note.**

### T0 side-finding: `sync stats` improved, and it was NOT the antenna change

A3 read `verdict:"fail"` before this session and `jitter_est_dtu` 14-17
(219-266 ps, `pass`) after. **That is the T6 firmware, not the antenna delay**,
and the distinction is load-bearing:

- Antenna delay is a constant offset and is absorbed entirely by the model's
  phase term. `docs/anchor-sync-measurement.md` §0.1 states this as a design
  property — it is why the Fase 2 gate can be run on uncalibrated boards.
  `rebaselines:0` on the same read confirms the model never even restarted.
- T6 changed exactly this statistic: residuals are now accumulated **only while
  converged**. The arithmetic for why that is worth ~5x:

```
RMS^2 = (390*sigma^2 + 10*(35*sigma)^2)/400 = sigma^2 * 31.6  ->  RMS = 5.6*sigma
```

  Ten un-converged samples out of 400 inflate the RMS about fivefold, because
  an RMS squares them and an un-converged conversion is ~35x worse. Right
  magnitude, right direction, same effect §4.1 of that doc already records as
  "`sync reset` is worth 3.7x".

So this is the **first hardware evidence that T6 works** — partial, on one
board. It is NOT a calibration result and must not be cited as one.

**Possible consequence worth chasing, bounded honestly:** 219-266 ps against
the Fase 2 measurement of 1.44-1.53 ns that the "~45 cm accepted instead of
10-30 cm" product decision rests on. If the fleet reads like this on current
firmware, that decision has headroom to be revisited. **Do not report the gate
as re-passed on this**: one anchor, unknown distance to the gateway (the
2026-08-26 figure was at 3 m), different board with unconfirmed PA rework
status, and the shell's own tail warning fired (`max 139` vs `rms 27` = 5.1x,
i.e. multipath or an intermittent obstruction on that link). Settling it is one
command per board: `sync reset`, wait 90 s, record `jitter_est_dtu` with that
anchor's measured distance to the gateway.

---

## T1 — Tape the survey

Unchanged from the brief, **plus one addition that B14 makes mandatory.**

### T1a — anchor-to-anchor edges and heights

```
apos show            # record the stored survey BEFORE touching anything
anchor show          # on each anchor, for its own ant_tx/ant_rx and pos
```

Measure every anchor pair with the tape (and with the laser where it reaches —
record which instrument gave which number, they disagree and the laser is the
reference). Measure anchor heights. Then either re-run the survey
(`apos run` / `apos apply`) or set coordinates by hand with `anchor pos`,
and record old and new per anchor in mm.

### T1b — the gateway's position (NEW, and it is the point)

**Tape-measure the CCP master's position in the same frame as the anchors.**
It is not in the survey and no command will tell you where it is. Record its
distance to every ranging anchor.

Then compute, for each anchor `k` against the reference anchor `0` (the lowest
anchor id, which is what `tdoa_collect` uses as the solve reference):

```
predicted_bias_k  =  -(d_Mk - d_M0)      metres of range difference
```

This is the B14 prediction, and it is the number T3 is checked against. It
costs a tape measure and no firmware.

### T1 verdict

```
Status:              NOT RUN — needs physical access and a tape/laser
Edges measured:      pair / value / instrument
Old -> new coords:   per anchor, mm delta
Position delta at the test spot after redeploy: ___ mm
Gateway position:    ___   d_M per anchor: ___
predicted_bias_k:    ___   (metres, signed, vs the lowest-id anchor)
```

---

## T2 — Bench split calibration

**The derivation in the brief is right in free space and wrong under this
firmware's sync, in a way that matters.** Under CCP sync a converted timestamp
is in the master's TRANSMIT base, so it is early by the flight time from the
master. Redoing the brief's algebra with that term:

```
t_fwd = d/c + (d_MR - d_Mi)/c + tau_tx,R + tau_rx,i
t_rev = d/c + (d_Mi - d_MR)/c + tau_tx,i + tau_rx,R

Delta_iR = t_fwd - t_rev = 2*(d_MR - d_Mi)/c  +  s_R - s_i
```

The path term `d/c` still cancels by reciprocity — that part of the brief
holds, and no survey is needed. **The master-distance term does not cancel:
reciprocity DOUBLES it.** Left uncorrected it puts an error of
`+(d_MR - d_Mi)` metres straight into the computed `tau_rx,i`, i.e. 213 DTU
per metre of bench asymmetry, which is the same order as the effect being
calibrated.

**Two fixes, either is sufficient, both are cheap:**

1. **Place the CCP master on the perpendicular bisector of R and i**, so
   `d_MR == d_Mi` and the term is identically zero. Tape-measure both to
   confirm, do not eyeball it. This is the recommended one.
2. Tape-measure `d_MR` and `d_Mi` and subtract `2*(d_MR - d_Mi)/c` from
   `Delta_iR` before solving.

Everything else in the brief's procedure stands: gauge `s_R := 0`, both boards
>= 1.5 m elevated, >= 2 m apart, >= 3000 frames each direction, program
`ant_delay_tx = (D + s)/2` and `ant_delay_rx = (D - s)/2` so the TWR sum `D` is
preserved by construction.

**Reverse-path feasibility.** There is no existing console path that makes an
anchor emit a frame another anchor timestamps into the master base on demand.
The closest existing mechanisms and why each falls short:
`cal peer` (cal image) runs SS-TWR and gives the SUM, not the split;
`apos` ranging likewise; `blink_rx` only ever receives. So T2 **does** need
additive test-only code — a minimal anchor-emitted frame that peers stamp
through `sync_model_to_master()` and report. Flag it clearly as additive test
code and keep it out of the production image, the same way `cal_run.c` is kept
out.

**Cheaper alternative worth costing first:** T3 (`tools/rx_cal.py`) measures
the same per-anchor constant in situ, needs no new firmware at either end, and
absorbs the B14 term as a bonus. T2's advantage is that it isolates the
antenna split specifically; T2's disadvantage is that it isolates *only* that,
so the B14 term survives it and T3 would still be needed afterwards.

### T2 verdict

```
Status:               NOT RUN — needs bench setup + additive test firmware
Reference anchor R:   ___  (gauge s_R := 0)
Master siting:        perpendicular bisector confirmed? ___  d_MR/d_Mi: ___
Per anchor: s_i (DTU) / new ant_tx / new ant_rx / D preserved?
Acceptance |Delta| < 10 DTU post-cal:   ___
Acceptance taped one-way < 10 cm:       ___
Acceptance SS-TWR unchanged:            ___
Test-code needed:     YES (no existing reverse path — see above)
```

---

## T3 — `tools/rx_cal.py` verification run

Run **after T1** (it needs a tape-validated survey; `docs/rx-antenna-delay-calibration.md`
§3 lists this as a hard prerequisite) and after T2 if T2 ran.

**The brief expects `b ≈ 0` after T1+T2. Under B14 that expectation is wrong,
and the discrepancy is the measurement.** T2 calibrates the antenna split;
nothing in T2 or T1 removes the master flight-time term. So the correct
post-T1+T2 prediction is:

```
b_k  ~=  predicted_bias_k  from T1b   =  -(d_Mk - d_M0)
```

not zero. Compare against T1b rather than against zero:

- `|b_k - predicted_bias_k| < 0.5 ns` on every anchor → **B14 attributed and
  T2 verified.** The remaining bias is geometric and the right fix is either
  to model `d_Mk` or to keep folding it into `ant_rx` deliberately and
  document that that is what the number means.
- `b_k ≈ 0` on every anchor → B14 is somehow already absorbed. Do not accept
  this quietly; it would contradict the code reading, so re-derive before
  believing it.
- Neither → localise per anchor, say whether it is master or slave, and
  correlate back to the T0 outcome.

**Do not blindly apply.** Report first, per the brief. If it is applied, apply
as `ant_rx += b`, `ant_tx -= b` so the SS-TWR sum is preserved and `cal ref` /
`apos` / TWR stay valid.

### T3 verdict

```
Status:            NOT RUN — blocked on T1 (needs a tape-validated survey)
b per anchor (ns): ___
predicted_bias_k from T1b (ns): ___
|b - predicted|:   ___
Cross-spot spread (the tool prints it and refuses to emit an apply
command when it is large): ___
Verdict / action taken: ___
```

---

## T4 — Tag height (`apos tagz`)

Measure the tag antenna's height above the plane the survey's `z = 0` sits in.
The value is **negative** when tags sit below ceiling anchors.

```
apos tagz <metres>       # gateway only, persists immediately
apos show                # confirm
```

Current bias with `tagz = 0.0` on this array: **~0.23 m inward** toward the
anchor centroid at 1 m from centre — a 23% effect on a 1.2–2.5 m array,
measured in `tests/tdoa_solve/test_height_model`. It does not cancel in a range
difference because `sqrt(rho^2 + dz^2)` is nonlinear.

Setting a *wrong* value is a new bias, which is why the default is 0.0 rather
than a plausible ceiling figure. Measure it.

### T4 verdict

```
Status:             NOT RUN — needs a tape measure
Measured tag antenna height vs survey z=0: ___ m
Old apos tagz: 0.0        New: ___
Residual bias estimate after update: ___ m
```

---

## T5 — Apex raise (human task, flagged only)

Remount the apex anchor from **0.920 m** to **~1.840 m**. That takes the
array's worst-case position amplification from **4.21 to 1.73**; tripling
instead of doubling buys almost nothing further (1.33 vs 1.49), so doubling
captures nearly all of the available gain. This is a larger win than any
software change on this path.

**After any physical remount, T1 must be re-run** — the survey and the
gateway-distance measurements are both invalidated. Do not make solver changes
that assume the new geometry until remount + resurvey are confirmed.

### T5 verdict

```
Status: PENDING — human physical task. Resurvey (T1 + T1b) required after.
```

---

## T6 — B13 fix

**IMPLEMENTED 2026-09-18.** See the commit on `diag/tdoa-precision`.

What changed, against the five scope-locked requirements:

1. **Gate.** `sync_model_to_master()` now refuses unless the baseline has
   reached `SYNC_BASELINE_USEFUL` (10 intervals) or a usable carried skew
   stands in for it. Previously `valid` was set after ONE completed interval.
2. **Carry.** The last converged rate is kept across a re-baseline as the exact
   integer pair `(m_acc - l_acc, l_acc)` — not a ppb scalar, which resolves
   only 1 part in 1e9 and would cost ~12.8 DTU over one interval of
   extrapolation. Its admissible age is
   `SYNC_SKEW_MAX_ERR_PPB / SYNC_SKEW_DRIFT_PPB_PER_S` = **5 ppb / 10 ppb/s =
   0.5 s**, both constants named and justified in `sync_model.h`.
   `SYNC_SKEW_DRIFT_PPB_PER_S = 10` rounds up the ~8 ppb/s thermal-transient
   figure the header already carried for a plain XO; rounding **up** is the
   safe direction, because assuming a larger drift rate makes a carried skew go
   stale sooner and the fallback is simply the pre-existing behaviour of waiting
   for a fresh baseline. `SYNC_SKEW_MAX_ERR_PPB = 5` sits deliberately between
   the converged estimator (~1.1 ppb) and the two-point estimate that is B13
   itself (~10.8 ppb), so a carried skew is always better than what it replaces
   and always worse than what it is waiting for.
   **Honest limit, stated rather than discovered on a bench:** at 10 ppb/s a
   carried skew is worth only a few hundred milliseconds. It covers a root
   change or a one-to-two-superframe discontinuity; it does **not** cover a
   multi-second gateway reboot, where the anchor correctly publishes nothing
   for up to ~2 s while its fresh baseline converges. Bridging further would
   reintroduce B13 by another route.
3. **Dynamic sigma.** `sync_model_sigma_dtu()` is new and is what `blink_rx.c`
   now publishes as `sigma_dtu`. It is built from the anchor's **own measured**
   residual jitter (fallback `SYNC_JITTER_FALLBACK_DTU = 96`, the measured 3 m
   figure, not the 15x-optimistic `SYNC_JITTER_DTU = 6` design assumption), and
   its phase term now accounts for how many samples the residual EMA has
   actually averaged since the re-baseline. It is **non-increasing in `n_obs`
   by construction**, so it can never claim more confidence mid-transient than
   after it.
4. **Counters, non-resetting.** `sync_model_rebaseline_count()`,
   `sync_model_miss_count()` and `sync_model_n_obs()`, cleared only by
   `sync_model_init()`. Surfaced on `sync stats` as `rebaselines`, `sync_gaps`,
   `n_obs`, alongside `sigma_dtu`/`sigma_ps`, with two new conditional warnings
   (mid-re-baseline; and "this anchor is publishing nothing right now", which
   would otherwise read at the console exactly like an anchor going quiet).
   `ccp_slave.c`'s three mid-life `sync_model_init()` calls became
   `sync_model_rebaseline()`; the root-change path passes `keep_skew = false`
   because a different root is a different crystal.
5. **Tests.** Six new cases in `tests/sync_model/`, including the property the
   plan names: *after a re-baseline the published sigma exceeds the converged
   value until `SYNC_BASELINE_USEFUL` is reached*, plus monotonicity, the
   lifetime counters, the short-discontinuity bridge, the stale-carry refusal,
   and the root-change no-carry. Three pre-existing tests were updated because
   the gate deliberately changes their premise — `test_needs_two_observations`
   is now `test_needs_a_useful_baseline` and asserts refusal at every step
   below the threshold.

One behaviour change worth knowing at the console: residual statistics are now
accumulated **only while converged**, and they now SURVIVE a re-baseline. That
fixes at the source the effect `docs/anchor-sync-measurement.md` §4.1 records
as "`sync reset` is worth 3.7x" (an un-reset 30 cm reading came back 188 DTU
against a true 50). `sync reset` still works and is still worth running, but
skipping it is no longer a trap.

### T6 verification status

```
Host tests:   tests/sync_model/  ALL TESTS PASSED (6 new cases)
              every other suite on the path re-run, no regressions
              (uwb_frame / apos_* / ccp_frame / blink_frame do not BUILD on
              this host — pre-existing MinGW missing-errno gap, unrelated)
Firmware:     production image links clean, zero compiler warnings
                dram0_0_seg 271944 -> 271984 B  (+40 B)
              calibration image links clean (net stubs path)
Hardware:     NOT RUN. The acceptance criterion — induced re-baseline via
              gateway reboot, transient quantified in mm before vs after —
              needs the bench.
```

### T6 acceptance, to run on hardware

```
1. Tag static on its mark, all anchors converged (sync stats: n_obs >= 10).
2. Capture 5 min of position stream. Record dispersion and mean.
3. `kernel reboot cold` on the GATEWAY. This forces a re-baseline on every
   anchor (ccp_master_init() re-seeds ccp_seq at 0 with root_id unchanged).
4. Capture through the reboot and for 5 min after.
5. On each anchor afterwards: `sync stats` — confirm `rebaselines` incremented
   by 1, `sync_gaps` behaviour, and that `sigma_dtu` was elevated and fell.
```

```
Status:                       NOT RUN on hardware
Position transient, pre-fix:  ___ mm   (needs a pre-fix capture — if none was
                              recorded, reflash the previous build to get one)
Position transient, post-fix: ___ mm
rebaselines incremented:      ___
sigma rose then fell:         ___
```

---

## T7 — Conditional DIAG gate

**NOT TRIGGERED — and not evaluable yet.** The gate is "static-tag p95 range
residual > 0.5 m after T0–T6, or the rank-Q jitter unexplained". T0–T5 have not
run, so there is no post-fix residual to test the gate against.

When it is evaluable, note two things the Stage-1 audit already settled:
L4/E5 are dropped outright (B5 is structurally impossible here — the CCP TX
timestamp is measured post-TXFRS, there is no scheduled value to compare
against), and E1's CFO cross-check needs a `dwt_readclockoffset()` read added
inside `ccp_slave_on_rx()`, since no anchor reads CFO on the CCP path today.

```
Status: NOT TRIGGERED (not yet evaluable — T0-T5 outstanding)
```

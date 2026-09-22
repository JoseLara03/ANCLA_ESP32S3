# TDoA precision bug-hunt — Stage 1 static audit

Branch `diag/tdoa-precision`, audited at `dca969c` (tree of
`feat/tdoa-accuracy-filter`). **Read-only: no code changed in this pass.**

## 0. Two corrections to the plan's own premises

Both change what Stage 2/3 should measure, so they come first.

### 0.1 There is no carrier integrator on this part

The plan's constant table gives `0.587 ppm/count (CH5)` via
`FREQ_OFFSET_MULTIPLIER x HERTZ_TO_PPM_MULTIPLIER_CHAN_5`. Those macros and
`dwt_readcarrierintegrator()` are **DW1000**. This is a DW3220 (DEV_ID
`0xDECA0312`) on the Qorvo `dwt_uwb_driver` 08.02.02, whose equivalent is
`dwt_readclockoffset()`, returning a **pre-scaled ratio**: 1 LSB = 2^-26 =
**14.9 ppb**, not a Hz count needing a channel multiplier.

Evidence: `src/ss_initiator.c:294` — `((double)dwt_readclockoffset()) /
(double)(1U << 26)` — and `src/sync_model.h:28`.

### 0.2 This is not a jam-sync system

The plan's B3 assumes "offset frozen between SYNCs, no skew extrapolation".
This firmware runs a **CCP-based rate+phase model** (`src/sync_model.{c,h}`,
host-tested in `tests/sync_model/`) that estimates a drift rate over a
multi-interval baseline and extrapolates it to the blink's own arrival
timestamp. B3 as written does not apply. A related but *different* defect
does — see **B13** below, which is new and not in the plan's registry.

---

## 1. Audit table A1–A7

| Check | Bug | Verdict | Evidence |
|---|---|---|---|
| A1 width | B1 | **REJECTED** | all sync-path stamps are full 40-bit |
| A1 wrap | B2 | **REJECTED** | `sdelta40()` / modulo-fold everywhere |
| A1 float | B9 | **REJECTED** | int64 subtraction precedes every float cast |
| A2 skew | B3 | **REJECTED** | rate estimated and extrapolated at blink time |
| A3 CFO | B4 | **REJECTED as a defect** | absent by design, see 1.4 |
| A4 TX ts | B5 | **REJECTED** | measured post-TXFRS, not scheduled |
| A5 solver | B6 | **REJECTED** | keyed on `(tag_addr, blink_seq)`, one timebase |
| A6 PHY | B7 | **REJECTED** | one `dwt_configure()` at boot, never re-run |
| A7 ant dly | B8 | **CONFIRMED** | `ant_delay_rx` is 16385 on every board |

### 1.1 A1 — timestamp width, wrap, and float (B1, B2, B9)

Every timestamp call site in `src/`:

| Site | Call | Width | Downstream |
|---|---|---|---|
| `uwb_dwtime.c:18` | `dwt_readrxtimestamp()` | **40** | `uint64_t`, all RX paths |
| `uwb_dwtime.c:31` | `dwt_readtxtimestamp()` | **40** | `uint64_t`, CCP announce |
| `ss_initiator.c:284-285` | `...lo32()` | 32 | SS-TWR only, **not** sync path |
| `apos_node.c:128`, `uwb_gateway.c:577..780` | `...systimestamphi32()` | hi32 | beacon schedule only |

`uwb_slave.c:348` takes one `uwb_get_rx_timestamp_u64()` and dispatches the
same 40-bit value to `ccp_slave_on_rx()` (`:362`) and `blink_rx_on_rx()`
(`:375`). No lo32 anywhere on the sync or blink path → **B1 REJECTED**.

Wrap handling, every layer:

- `sync_model.c:20` `sdelta40()` — sign-extends the masked difference from
  bit 39. Used for every `l`/`m` difference and for the residual.
- `ccp_slave.c:267` its own `ccp_slave_sdelta40()`, same discipline.
- `tdoa_dtu.c:28` normalises to `[0, 2^40)` **before** folding to
  `[-2^39, 2^39)` — the two-step form, which is the one that works both ways
  round the wrap.
- `tdoa_gw.c` local `sdelta40()` for the per-tag `dt` reference.

→ **B2 REJECTED.**

Float: `tdoa_solve.c` is float32, but `m[i].t_dtu - m[0].t_dtu` is an
**`int64_t` subtraction evaluated before the cast** at `:96`, `:133`, `:203`,
and by then `tdoa_dtu_rebase()` has bounded every value to
`+/- TDOA_DTU_MAX_SPREAD` (32768 DTU), which float32 carries exactly. The
accumulators `l_acc`/`m_acc`/`phase_corr` are `int64_t`; `sync_model.c` has no
`<math.h>` at all (`:31`). Transport is `strtoll` on a quoted decimal string
(`pos_json.c:266`), not a double. → **B9 REJECTED.**

### 1.2 A2 — sync algorithm classification (B3)

**(a) Is a skew term estimated?** Yes. `sync_model_observe()` accumulates
`l_acc`/`m_acc` over a baseline up to `SYNC_BASELINE_MAX` (128) intervals,
rolled onto its own midpoint past that so thermal drift is tracked rather
than averaged away.

**(b) Extrapolated or frozen at blink arrival?** **Extrapolated.**
`blink_rx.c:63` calls `sync_model_to_master(ccp_slave_model(), rx_ts, ...)`
with the blink's **own** RX timestamp, and `sync_model.c:176-193` computes
`d = sdelta40(l_dtu, m->l_raw)` then `m_raw + d + d*(m_acc-l_acc)/l_acc +
phase_corr`. The skew is applied over exactly the elapsed `d`.

→ **B3 REJECTED.** The split-correction form is also overflow-safe by
construction and documented as such at `sync_model.c:178-190`.

### 1.3 A4 — SYNC TX timestamp provenance (B5)

`ccp_master.c` uses `DWT_START_TX_IMMEDIATE` (`:260`) — there is **no**
`dwt_setdelayedtrxtime()` on this path at all (`:256`) — waits for TXFRS,
then reads `uwb_get_tx_timestamp_u64()` (`:299`) and stores it masked to 40
bits (`:301`). That measured value is announced in the **following** CCP
(`:206`, deferred-timestamp design), and `prev_tx_dtu` advances **only on a
confirmed transmit**, so a dropped CCP re-announces the same value instead of
one for a frame that never existed.

→ **B5 REJECTED.** E5 is unnecessary: there is no scheduled value to compare
against.

**Doc rot found (not a bug):** `ccp_frame.h:69` still says
`tx_dtu  40-bit scheduled RMARKER time of THIS frame`. Two things wrong with
that line under the current design — it is **measured**, not scheduled, and it
describes the **previous** frame, not this one. Fix in a cleanup pass, not
here.

### 1.4 A3 — clock-offset usage (B4)

`dwt_readclockoffset()` appears **once**, in `ss_initiator.c:294`, on the
SS-TWR path. It is **not** used in the sync model, and that is a documented
decision rather than an omission (`sync_model.h:37-43`): single-shot CFO is
~15 ppb/LSB, while two CCP timestamps 200 ms apart give 15.65 ps / 200 ms =
**78 ppt** — two orders of magnitude better. CFO is called out there as "an
acquisition aid and a cross-check, not the estimator".

→ **B4 REJECTED as a defect.** But note the consequence for Stage 3: **E1's
"CI vs SYNC-delta skew cross-check" cannot be run as written**, because no
anchor reads or logs CFO on the CCP path. Getting it means adding a
`dwt_readclockoffset()` read inside `ccp_slave_on_rx()` — additive, DIAG-only,
legal under this phase's rules, and worth doing purely as an independent
check on `sync_model_drift_ppb()`.

### 1.5 A5 — solver association and timebase (B6)

- **(a) grouping by blink sequence:** yes — `tdoa_collect` is keyed on
  `(tag_addr, blink_seq)`; `blink_seq` wraps at 256 = 51.2 s at 5 Hz against a
  150 ms collection window, so a value cannot recur inside a group. A duplicate
  `anchor_id` for the same blink is **rejected**, not folded in twice (which
  would hand the solver a zero-difference equation against itself).
- **(b) conversion at blink arrival:** yes — see 1.2.
- **(c) conversion parameters logged per blink:** **no.** Nothing records which
  `l_raw`/`l_acc`/`m_acc`/`phase_corr` produced a given `converted_ts40`. This
  is the single biggest observability gap and is exactly what L2 in Stage 2
  exists to close. Not itself a bug.
- **(d) per-anchor calibration subtracted in the difference:** **no, and there
  is nothing to subtract** — see B8.

→ **B6 REJECTED** as a correctness defect; (c) and (d) recorded as gaps.

Anchor position lookup (`tdoa_gw.c:196 anchor_xyz()`) resolves
`UWB_ANCHOR_ADDR_BASE + anchor_id` against the applied survey and **refuses**
(counted `no_anchor`) when absent, so a stale or missing survey cannot produce
a silently wrong coordinate.

### 1.6 A6 — PHY configuration consistency (B7)

One `dwt_config_t phy_cfg = UWB_PHY_CONFIG_INITIALIZER` (`uwb_radio.c:24`),
one `dwt_configure()` (`:87`), called once at boot from `main()`. No frame
type reconfigures the PHY; BCN, CCP and BLINK all share it. The contract is
compile-time (`uwb_phy.h`): channel 5, PLEN_1024, PAC32, code 9, SFD type 3,
850 kbps, STS off, PDOA M0.

→ **B7 REJECTED.**

### 1.7 A7 — antenna delays (B8) — **CONFIRMED**

`uwb_radio.c:97-98` applies `dwt_settxantennadelay(cfg->ant_delay_tx)` and
`dwt_setrxantennadelay(cfg->ant_delay_rx)` from per-board NVS. `ant_delay_tx`
is per-board and trimmed. **`ant_delay_rx` is not.**

Exhaustive list of writers of `ant_delay_rx` in the whole tree:

```
src/uwb_config.c:30   c->ant_delay_rx = UWB_ANT_DELAY_DEFAULT;   /* 16385 */
src/uwb_config.c:83   c->ant_delay_rx = (uint16_t)rx;            /* `anchor ant` console only */
```

No calibration path writes it. `cal_solve` **holds it fixed by construction**
(`cal_solve.h:8`), because SS-TWR observes only the sum `ant_tx + ant_rx`.
The whole `cal ref` correction went into TX.

**Why this is fatal specifically for TDoA and invisible to TWR:** an anchor in
TDoA only ever receives. The DW3220 subtracts `ant_rx` from every RX
timestamp, and the observable carries `-(delta_k - delta_0) x 4.69 mm` — the
**difference in RX-delay error between anchors**, with no round trip and
nothing to cancel it. At **4.69 mm/DTU** (one-way), not the 2.35 mm/DTU that
applies in SS-TWR.

The one existing measurement of per-board spread in the TWR-observable
quantity is the 2026-08-28 laser campaign: **~207 units**. Applied one-way
that is **~971 mm of range-difference bias** — before geometry amplifies it.

See `docs/rx-antenna-delay-calibration.md`, which already records a **3.11 m**
position shift when a fourth anchor was added while the tag never moved. That
is the same magnitude as the symptom under investigation.

---

## 2. New finding not in the plan's registry

### B13 — the model publishes with a one-interval baseline after every re-baseline

`sync_model_observe()` sets `m->valid = true` as soon as `l_acc != 0`
(`sync_model.c:157`), i.e. after the **first** completed interval.
`sync_model_to_master()` gates only on `valid && l_acc != 0 && misses <=
SYNC_MISS_MAX` — **nothing anywhere gates on `SYNC_BASELINE_USEFUL`**, and the
header says so explicitly ("not as a threshold anything switches on",
`sync_model.h`).

At n_obs = 1 the two-point rate error is `jitter*sqrt(2)/T`. With the measured
hardware jitter (92–98 DTU at 3 m) that is ~10.8 ppb, which over a 200 ms
extrapolation is **~2.2 ns ≈ 0.65 m** of range-difference error — against
~4 DTU (19 mm) once the baseline is past ~5 observations. A factor of ~35.

Every path in `ccp_slave.c` that calls `sync_model_init()` mid-life re-enters
this state: a root change, a sequence discontinuity over 128 (gateway reboot),
and the local-clock mismatch check. Meanwhile the `sigma_dtu` an anchor
publishes is `sync_model_jitter_est_dtu()` — a **long-run** residual statistic
that does not shrink or grow with baseline length — so a freshly re-baselined
anchor reports its usual confidence while converting ~35x worse.

**Status: CONFIRMED as a code property; contribution UNQUANTIFIED.** On the
2026-08-26 hardware run `gaps:0, rejected:0` across ~2000 receptions, so this
may be rare in practice. L1/L3 in Stage 2 (`sync_miss_count`, plus an added
`n_obs`) settle it; it is cheap to log and cheap to fix if it fires.

---

## 3. Ranked contributions to the observed ~3 m

Magnitudes are range-difference bias/noise multiplied by this array's measured
position amplification (largest singular value of the Jacobian pseudo-inverse):
**1.16** at the triangle centre, **2.09** on the origin–xaxis baseline,
**4.21** near the origin corner (recorded 2026-09-03; geometry origin (0,0),
apex (1.752,0.920), xaxis (2.385,0)).

| Rank | Cause | Range-diff error | Position error | Kind | Status |
|---|---|---|---|---|---|
| 1 | **B8** RX delay split uncalibrated | ~971 mm | **1.1 – 4.1 m** | bias | CONFIRMED (static) |
| 2 | **B11** geometry amplification | — | **x1.16 – x4.21** | multiplier | CONFIRMED (survey) |
| 3 | survey geometry never tape-validated | up to 1037 mm | 1.2 – 4.4 m | bias | OPEN (not a code bug) |
| 4 | sync jitter 1.44–1.53 ns | ~450 mm | 0.52 – 1.9 m | **noise** | measured, Fase 2 |
| 5 | `apos tagz` = 0.0, unmeasured | — | ~0.23 m inward | bias | CONFIRMED (static) |
| 6 | **B10** NLOS ungated | — | unknown | spikes | UNTESTED |
| 7 | **B13** short-baseline publish | ~0.65 m transient | 0.75 – 2.7 m | transient | CONFIRMED (code) |

B8 x B11 alone reproduces the symptom, and is the only entry with an
independent observation of the right magnitude already on record (3.11 m).

Ranks 1 and 3 are **not separable by any measurement this firmware currently
makes** — both are constant per anchor pair, both move absolute position, and
`max_reciprocal_mm` bounds neither (it measures residual clock offset, which is
antisymmetric; antenna delay is exactly symmetric in SS-TWR). Rank 4 is the
only entry that is noise rather than bias, and the only one a longer stationary
capture separates out on its own.

---

## 4. Recommended order — NO FIXES IMPLEMENTED

1. **Tape-measure the three survey edges.** Free, needs no firmware, and it is
   the only thing that separates rank 3 from rank 1. Everything downstream is
   uninterpretable until this is done.
2. **Run `docs/rx-antenna-delay-calibration.md`** — the procedure and
   `tools/rx_cal.py` already exist and the tool is verified against synthetic
   data. Needs no firmware change at either end; apply as `ant_rx += b`,
   `ant_tx -= b` so the SS-TWR sum is preserved and TWR / `apos` / `cal ref`
   stay valid. **This is rank 1 and it is already written.**
3. **Measure `apos tagz`** with the same tape. Rank 5, one console command.
4. Only then build the Stage 2 DIAG image. Its remaining value after 1–3 is
   E1 (quantify residual p95), E4 (B10, untested), and B13's counters — the
   parts no existing procedure covers. L4/E5 can be dropped outright (B5 is
   structurally impossible here); the E1 CFO cross-check needs a
   `dwt_readclockoffset()` read added to `ccp_slave_on_rx()` under DIAG.
5. Raising the apex from 0.920 m to ~1.840 m takes worst-case amplification
   from 4.21 to 1.73. That is a bigger win than any software change on this
   path, and tripling it adds almost nothing beyond doubling (1.33 vs 1.49).

**Exit criterion not yet met:** E1's p95 residual is unquantified, and ranks 1
and 3 are not yet separated. But the audit alone identifies a dominant root
cause (B8) with a magnitude consistent with the observed 3 m, and its fix
procedure already exists and requires no firmware change — which reorders the
plan: run the calibration before building the instrumentation, not after.

---

# Addendum, 2026-09-18 — B14, found while preparing the T0-T7 test sequence

**B14: the CCP master-to-anchor propagation delay is never subtracted, so every
anchor's converted timestamp carries a per-anchor constant equal to its own
distance from the gateway.** CONFIRMED statically. It is structurally
indistinguishable from B8, has a larger available magnitude on this
deployment, and it changes what three of the ordered tests can conclude.

## 1. The derivation

`sync_model_observe()` is fed the pair (master's **transmit** timestamp,
this anchor's **receive** timestamp of that same frame) — `ccp_slave.c`'s
`sync_model_observe(&model, f.tx_dtu, prev_rx_ts)`, where `f.tx_dtu` is the
value `ccp_master.c` read back with `uwb_get_tx_timestamp_u64()` after TXFRS.
So the model is trained to map *this anchor's local RX instant* onto *the
master's TX instant*. Those two instants are not the same instant: they are
separated by the flight time `d_Mi/c`.

`sync_model_to_master()` therefore returns master-TX-base time, which for any
later event at anchor *i* is the true master-clock time **minus** `d_Mi/c`:

```
converted_i  =  T_emit  +  r_i/c  -  delta_i  -  d_Mi/c
```

where `r_i` is tag-to-anchor distance, `delta_i` the RX antenna-delay error
(B8), and `d_Mi` the **gateway-to-anchor** distance. `T_emit` cancels in a
difference; `d_Mi` does not:

```
(converted_i - converted_0) * c  =  (r_i - r_0)  -  (delta_i - delta_0)*4.69mm
                                                 -  (d_Mi - d_M0)
```

`tdoa_solve()` fits `r_i - r_0 = (t_i - t_0) * TDOA_M_PER_DTU` with **no
constant term**. Grep confirms there is nothing to correct it with: the
gateway is not in the survey (`apos` covers the four ranging anchors;
`tdoa_gw.c:anchor_xyz()` resolves `UWB_ANCHOR_ADDR_BASE + anchor_id` only),
`struct apos_survey` has no master position field, and `struct ccp_frame`
carries only `seq`/`hop`/`tx_dtu`/`root_id`.

## 2. Magnitude

**One metre of gateway-distance difference is one metre of range-difference
bias** — one-for-one, not halved, because this is a one-way path with no round
trip. Before geometry amplification (1.16x at the triangle centre, 4.21x near
the origin corner).

The gateway on this deployment sits at its own spot, metres from the anchors,
and has never been surveyed or tape-measured. Distances differing by the array
size (1.2-2.5 m) are the default expectation, which puts B14 at **1.2-2.5 m of
range-difference bias, 1.4-10 m of position error** — larger than B8's
~971 mm, and enough on its own to produce the observed symptom.

It also reproduces the observation the audit above cites as B8's best
evidence: the **3.11 m** position shift recorded in
`docs/rx-antenna-delay-calibration.md` when a fourth anchor was added and the
tag never moved. Adding an anchor adds an equation carrying its own
un-modelled `-d_M4/c`. B14 explains that with no manufacturing spread at all.

## 3. Why the audit above missed it, and what corrects it

A1-A7 asked whether the sync arithmetic was *self-consistent*. It is — the
model does exactly what its header says. B14 is a missing term in the
measurement equation, not a defect in the estimator, so no check in that
table could have reached it.

Corrections to two things the audit and the ordered plan assume:

- **`docs/rx-antenna-delay-calibration.md` section 1 is missing this term.**
  Its model is `tau_k = T_emit + r_k/c - delta_k + noise`; the true model has
  `- d_Mk/c` as well. The practical consequence is mild and in the useful
  direction: `- d_Mk/c` is a **per-anchor constant**, exactly the same shape
  as `delta_k`, so `tools/rx_cal.py`'s per-anchor mean **absorbs it** and the
  correction it computes is still the right correction to apply. What is
  wrong is only the LABEL: the fitted `b_k` is
  `(RX antenna delay difference) + (master flight-time difference)`, not RX
  antenna delay, and writing it into `ant_rx` is a bookkeeping fiction that
  happens to produce the right numbers. It also means the tool's own
  consistency check (the same `b_k` at every tag spot) still holds, because
  both components are tag-position-independent.
- **The published `sigma_dtu` and the Phase 2 jitter figure are unaffected.**
  B14 is a constant; it is absorbed by the phase term and contributes nothing
  to the residual spread.

## 4. Updated ranking

| Rank | Cause | Range-diff error | Position error | Kind | Status |
|---|---|---|---|---|---|
| 1 | **B14** master flight time unmodelled | 1.2-2.5 m (est.) | **1.4 - 10 m** | bias | CONFIRMED (static), magnitude UNMEASURED |
| 2 | **B8** RX delay split uncalibrated | ~971 mm | 1.1 - 4.1 m | bias | CONFIRMED (static) |
| 3 | **B11** geometry amplification | — | x1.16 - x4.21 | multiplier | CONFIRMED (survey) |
| 4 | survey geometry never tape-validated | up to 1037 mm | 1.2 - 4.4 m | bias | OPEN |
| 5 | sync jitter (CCP path) | ~450 mm | 0.52 - 1.9 m | **noise** | measured, Fase 2 — see the provenance note in the T-report |
| 6 | `apos tagz` = 0.0, unmeasured | — | ~0.23 m inward | bias | CONFIRMED (static) |
| 7 | **B10** NLOS ungated | — | unknown | spikes | UNTESTED |
| 8 | **B13** short-baseline publish | ~0.65 m transient | 0.75 - 2.7 m | transient | **FIXED 2026-09-18** |

Ranks 1, 2 and 4 are all per-anchor constants and **no measurement this
firmware makes can separate them from each other.** What separates rank 1 is
that it is the only one that is *predictable from geometry*: tape-measure the
gateway's position, compute `-(d_Mi - d_M0)`, and compare it against the
per-anchor offsets `tools/rx_cal.py` fits. Agreement within the fit's own
spread attributes the bias; disagreement leaves the remainder to ranks 2 and 4.
That check costs one tape measure and no firmware.

---

## B14 addendum 2: CONFIRMED ON HARDWARE, 2026-09-22 — root cause, measured

B14 is no longer a static reading. It was **predicted, then falsified twice,
then fixed by moving one board.** The record, because the method is reusable.

### The measurement

Two tag spots on the same applied survey (anchors at `0x0001 (0,0)`,
`0x0002 (3.143,0)`, `0x0003 (3.564,1.924)`, `0x0004 (-0.105,2.192)`):

| spot | tag truth | reported | error |
|---|---|---|---|
| P1 centre | (1.650, 1.029) | (-0.481, 1.225), n=107 | **2.141 m** |
| P2 edge | (3.354, 0.962) | (1.520, 1.230) | **1.885 m** |

The reported position **tracked** the tag at 117 % of its true motion, so this
was never a collapse to the seed. Converting both errors into per-anchor
range-difference bias gave the same numbers at both spots, 1.7 m apart:

```
0x0002  +2.642 vs +2.585 m   (agree to  56 mm)
0x0003  +2.622 vs +2.708 m   (agree to  86 mm)
0x0004  -0.440 vs -0.249 m   (agree to 191 mm)
```

A constant per-anchor bias. The two large ones agree to ~2 % of their own
magnitude.

### The prediction, made from radio alone

B14 requires those three biases to equal `-(d_Mk - d_M0)`, i.e. distances from
ONE point. Three equations, two unknowns — **overdetermined, so it could
fail.** Fitting gave the gateway at **(3.54, 0.90)**, rms 84 mm.

Tape-measured afterwards: `GW->0x0001 3.393, ->0x0002 1.091, ->0x0003 1.253,
->0x0004 3.702`, which trilaterates to **(3.313, 0.877)** — **228 mm** from the
radio-only prediction. The operator had put the gateway between two anchors
because that is where the laptop cable reached.

Decomposition against that tape:

```
0x0002  bias +2.614   B14 explains +2.302  (88 %)   left over  +66 DTU
0x0003  bias +2.665   B14 explains +2.140  (80 %)   left over +112 DTU
0x0004  bias -0.345   B14 explains -0.309  (90 %)   left over   -8 DTU
```

### The fix, and the second confirmation

**Moving the gateway is the whole fix**, because the observable carries the
*difference* `d_Mk - d_M0`: place it equidistant and the term cancels. For this
array the optimum is ~(1.62, 1.26), spread 94 mm against the 2611 mm it had.

Moved to roughly the array centre on a longer cable, same survey, same anchors:

```
error      2.141 m -> 0.634 m   (-70 %)
dispersion 0.230 m -> 0.162 m
```

And the residual bias re-fits a gateway at **(1.88, 0.81), rms 38 mm** — 0.53 m
off the optimum, leaving 913 mm of spread, which is most of the 0.634 m that
remains. Second overdetermined fit, second small residual.

### What this changes

- **Rank 1 is confirmed and is the dominant term**, at 80-90 % of a 2.1 m
  error on this deployment.
- **The cheapest fix is physical and needs no firmware**: put the gateway
  equidistant from every anchor. Practically — move it until the tape reads the
  same distance to all four.
- **It is a design gap, not an operator error.** `ccp_slave.c` pairs the
  master's TX timestamp with this anchor's RX timestamp of that same frame, so
  each anchor's time base is early by its own flight time, and `tdoa_solve()`
  fits with no constant term. The proper fix subtracts `d_Mk/c` in the solve,
  which needs the gateway's position stored and is firmware work.
- **`tools/rx_cal.py` absorbs whatever is left**, B8 and residual B14 together,
  which is why its fitted `b_k` must not be called "RX antenna delay".

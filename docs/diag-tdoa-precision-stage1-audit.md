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

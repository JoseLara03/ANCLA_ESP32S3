# Alpha-beta-gamma position filter for TDoA fixes — design

**Date:** 2026-09-06
**Branch:** `feat/tdoa-accuracy-part2` (implemented 2026-09-06 per the plan;
hardware evaluation status in CLAUDE.md)
**Supersedes:** the per-tag EKF of
`docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md` §4 and its
part-2 extension. Items 1–3 and 5–7 of that spec (early release, the height
model, `tdoa_solve()` hardening, `sigma_dtu` on the wire, the MOVING bit) are
unaffected and stay.

---

## 0. The decision this document implements, and what it does not reopen

The maintainer's decision (2026-09-06): **drop the constant-velocity EKF and
smooth the solved position with an alpha-beta-gamma (α-β-γ) filter instead.**

Why, stated once so it is not re-litigated:

- The tag carries a 3-axis accelerometer and no gyroscope. Its only usable
  output is a still/moving verdict. That is a mode discriminator, not motion
  input, so the EKF's motion model had nothing to steer by and its
  "tightly-coupled" measurement update bought nothing visible over the raw
  solve.
- On this site's thin 3-anchor triangle a single accepted range-difference
  equation per cycle kept the EKF's divergence recovery disarmed while it ran
  17 m outside a 2.4 m array (part-2 plan, Task 7). The release-order half of
  that defect was real and is fixed in `tdoa_collect`; the filter half is
  moot once the filter is gone.
- The visible problem the operator has is **jumps between consecutive
  fixes** — from anchor coordinates that are still not tape-validated and
  from CCP synchronisation that is sometimes poor — and the tool for that is
  a filter that damps the *published position*, judged by eye on the
  platform's map. That is what this is.

What this document does **not** claim: that smoothing improves accuracy.
**Suavidad no es exactitud.** The ~45 cm accuracy figure is bounded by GDOP
and the uncalibrated RX antenna delay, and this filter cannot move it. It
makes a wrong-by-0.5 m position stay put instead of dancing; it does not make
it right.

Trade-off accepted with eyes open: filtering the least-squares OUTPUT discards
the geometry. The raw solve's noise is not Gaussian, it scales with GDOP
(amplification 1.16 at the triangle's centre, 4.21 at a corner — CLAUDE.md,
"A 3-anchor TDoA cell has spots where y is UNOBSERVABLE"), and a fixed-gain
filter treats every fix alike. The EKF was supposed to be better precisely
here, and on hardware it was not. Simple and inspectable wins.

---

## 1. Goals and non-goals

**Goals**

1. Consecutive published fixes of one tag are temporally coherent: noise
   visibly reduced on a stationary tag, a walking tag's trace continuous.
2. A single gross outlier (mirror-branch solve, a bad CCP cycle, a group with
   one badly stamped anchor) does **not** appear on the map, and does not
   drag the estimate.
3. A tag that genuinely relocates (carried away, or the filter was tracking
   the wrong branch) is followed within a bounded number of cycles.
4. Everything the gateway does per fix stays bounded, allocation-free and
   float-only-in-geometry, on the `K_PRIO_COOP(0)` loop, inside the budget
   `tdoa_gw.h` already documents.
5. Every decision the filter takes is a counter on `blink stats`. Nothing on
   this path may be a black box on the bench — three defects in this
   migration were only reachable through counters and a trace.

**Non-goals**

- Accuracy. See §0.
- A measurement model. The filter sees `(x, y)` from `tdoa_solve()` and
  nothing else — no ranges, no range differences, no per-anchor `sigma_m`.
- Velocity output to the platform. The payload contract is frozen
  (`{"Tid","x","y","z":0}`); `vx`/`vy` stay internal.
- Tuning against ground truth. The gains are derived from a single tracking
  index and then judged by eye and by the stationary-dispersion number;
  ground-truth tuning needs a survey that has been tape-validated, which
  this site's has not.

---

## 2. Where it sits

```
uplink obs_q ──► ingest_one() ──► tdoa_collect ──► take_ready() (oldest first)
                                                         │
                                                         ▼
                       solve_one():  rebase ─► plausible? ─► reorder check
                                                         │
                                                         ▼
                                        resolve_one(): tdoa_solve() + 10 m jump gate
                                                         │  raw (x, y)
                                                         ▼
                                  ┌──────────── pos_abg_step() ─────────────┐
                                  │  predict(dt) → gate → correct | reseed │
                                  └───────────────────┬─────────────────────┘
                                                      │ filtered (x, y), or nothing
                                                      ▼
                                             pos_sink_publish()
```

Everything above the filter is unchanged from the post-removal tree. In
particular:

- **A solve runs on every group.** The EKF skipped the solve on filtered
  cycles; this design cannot, since the solve IS its measurement. Cost:
  `TDOA_GW_SOLVE_MAX` (8) Gauss-Newton fits per superframe, which
  `tdoa_gw.h`'s budget paragraph already prices at ~100 µs each and which
  was the load before 2026-09-02. Nothing new to measure, but the `gw_sf`
  heartbeat is still the instrument that confirms it.
- **The seed for `tdoa_solve()` and the mirror-branch jump gate stay on the
  RAW solve**, against the last PUBLISHED (filtered) position, at
  `TDOA_GW_MAX_JUMP_M` (10 m) within `TDOA_GW_SEED_AGE_MS` (1 s). That gate
  answers "is this solution on the wrong hyperbola branch"; the filter's own
  gate below answers "is this measurement consistent with the track". Two
  gates, two questions, deliberately not merged.
- **dt** comes from the reference anchor's absolute 40-bit `t_dtu`, captured
  before `tdoa_dtu_rebase()`, differenced with `sdelta40()` against the tag's
  `last_ref_t_dtu` — never from `now_ms`. Same source, same reason, same
  reorder discard as before (CLAUDE.md records why; the 2026-09-03 rewind
  defect must not be re-learned).

---

## 3. The filter (`src/pos_abg.{c,h}`)

Pure C, `<math.h>` only, no Zephyr, host-tested in `tests/pos_abg/`. One
instance per tracked tag, living in `tdoa_gw.c`'s `struct tag_memo`.

### 3.1 State and configuration

```c
struct pos_abg {
	float   x, y;        /* position, m */
	float   vx, vy;      /* velocity, m/s */
	float   ax, ay;      /* acceleration, m/s^2 */
	bool    init;        /* false until pos_abg_seed() */
	uint8_t reject_streak;
};

struct pos_abg_cfg {
	float   alpha, beta, gamma;  /* gains, designed at dt = POS_ABG_T_NOM_S */
	float   alpha_still;         /* position gain used while the tag says still */
	float   gate_m;              /* innovation gate on |z - x_pred|, metres */
	uint8_t reset_after;         /* consecutive gated cycles before reseeding on z */
};
```

`reject_streak` saturates at 255 (`uint8_t`), as `pos_ekf`'s did; the
comparison against `reset_after` is `>=`, so saturation cannot un-arm it.

### 3.2 One cycle: `pos_abg_step()`

```c
enum pos_abg_result {
	POS_ABG_ACCEPTED,   /* predicted and corrected; publish */
	POS_ABG_REJECTED,   /* predicted, correction gated; do NOT publish */
	POS_ABG_RESEEDED,   /* state replaced by z (streak reached); publish */
	POS_ABG_BAD_INPUT,  /* unseeded filter, dt <= 0, or non-finite dt/z:
	                     * state untouched, caller's bug */
};

enum pos_abg_result pos_abg_step(struct pos_abg *f, const struct pos_abg_cfg *c,
				 float dt_s, float zx, float zy, bool still);
```

Per axis, with `T = dt_s`, `r = z − x_p`:

```
predict:   x_p = x + vx·T + ½·ax·T²        vx_p = vx + ax·T        ax_p = ax
gate:      d   = hypot(zx − x_p, zy − y_p)
           g   = gate_m · min(POS_ABG_GATE_DT_CAP, max(1, T / T_nom))
           if d > g:        x,vx,ax ← predicted values; reject_streak++;
                            if reject_streak >= reset_after: seed(z); return RESEEDED
                            return REJECTED
correct:   x  = x_p  + α·r
           vx = vx_p + (β / T)·r
           ax = ax_p + (γ / T²)·r
           reject_streak = 0
still:     if still:  α ← alpha_still for the position correction above,
                      and after it  vx = vy = ax = ay = 0
```

Decisions folded into that pseudo-code, each with its reason:

- **Predict always, correct only when accepted.** A rejected group still
  advances the clock (`last_ref_t_dtu` moves in the caller), so the state
  must be propagated to that instant or the next accepted group's dt would
  cover time the filter never integrated — the same rewind class of defect
  as 2026-09-03, from the other side. On a stationary tag the prediction is
  the same point, so coasting costs nothing; on a walking tag it coasts at
  the last velocity for at most `reset_after` cycles.
- **Nothing is published on a rejected cycle.** No evidence arrived that
  cycle. Publishing the prediction would republish an unchanged estimate on
  a stationary tag, which is exactly the "stale republish looks like a live
  fix" bug of 2026-09-02, and this time it would be by design. Counted.
- **The gate is Euclidean, one threshold, not per axis — and it grows with
  dt.** The solve's error is anisotropic (y is the weak axis here), but the
  gate is protection against gross outliers, not a statistical test:
  `gate_m` sits ~4× the raw stationary RMS (0.345 m, 2026-09-03 trace) and
  well above a walking displacement per cycle (≤ 0.3 m at 1.5 m/s, 200 ms).
  **Default 1.5 m at the nominal 200 ms.** The prediction's own uncertainty
  grows with the interval it spans, so the effective gate is
  `gate_m × min(POS_ABG_GATE_DT_CAP, max(1, dt/T_nom))` — 3.0 m at 0.4 s,
  capped at 4.5 m from 0.6 s up (`POS_ABG_GATE_DT_CAP = 3`). Measured in
  simulation (§3.3, "Finding"): with the fixed gate a walking tag under this
  site's dt spread was rejected on 499 of 10⁵ cycles and reseeded 24 times;
  with the scaled gate 3 rejections and 0 reseeds, at the same RMS. Below
  nominal dt the gate stays at `gate_m` (the caller drops sub-50 ms groups
  anyway).
- **Reseed on the streak, not on a hull bound.** After `reset_after`
  consecutive rejections the measurement wins: the state is replaced by the
  latest `z`, velocity and acceleration zeroed. This is the kidnapped-tag
  recovery and also the "we were tracking the mirror branch" recovery.
  **Default 5** (1 s at 5 Hz). `reset_after = 0` means NEVER reseed on the
  streak (the gate then only ever drops), the same convention `pos_ekf` used
  and tested. The part-2 plan's proposed physical hull bound is not needed:
  with the streak bounded, the state cannot drift further than `reset_after`
  cycles of coasting from the last accepted fix.
- **`still` freezes velocity and acceleration after the correction, and
  softens the position gain.** This is the ZUPT-equivalent and, as before,
  the single largest visual win for the common case of a motionless tag.
  With `vx = vy = ax = ay = 0` the next prediction is the current point, and
  `alpha_still` (**default 0.15**) turns the filter into a slow exponential
  average of the solves. The MOVING bit's ABSENCE reads as still (proto-4
  anchors); the `still` counter and its shell warning exist for exactly
  that, as `zupt` did.
- **Fixed gains at variable dt.** `alpha`/`beta`/`gamma` are designed at the
  nominal `POS_ABG_T_NOM_S = 0.2` s and applied with the actual dt in the
  `β/T`, `γ/T²` divisions. For dt longer than nominal (gaps of 0.4–1.0 s are
  routine here, p90 = 0.8 s measured 2026-09-03) the velocity and
  acceleration corrections shrink, which is the conservative direction. dt
  shorter than nominal cannot occur for distinct blinks of one tag (the tag
  blinks at most once per 200 ms superframe); the caller guarantees
  `POS_ABG_DT_MIN_S <= dt <= TDOA_DT_MAX_MS/1000` and the filter itself only
  refuses `dt <= 0`, a non-finite dt or z, or an unseeded state, with
  `POS_ABG_BAD_INPUT`, state untouched.

### 3.3 Gains: one dial, derived, not picked

The three gains are a function of one number, the **tracking index**
`λ = σ_w·T² / σ_v` (Kalata), with `σ_v` the measurement noise (the raw
solve's per-fix RMS) and `σ_w` the acceleration random-walk increment per
sample. Gray & Murray's closed form gives the α-β-γ gains that make the
filter the steady-state Kalman filter for that model:

```
b = λ/2 − 3     c = λ/2 + 3     d = −1
p = c − b²/3    q = 2b³/27 − b·c/3 + d     v = sqrt(q² + 4p³/27)
s = −cbrt((q+v)/2) − cbrt((q−v)/2) − b/3
α = 1 − s²      β = 2(1 − s)²      γ = β² / (2α)
```

**Convention check, because it is the easy thing to get wrong by a factor of
two:** with the update written as `ax += (γ/T²)·r` (§3.2), the γ above equals
the steady-state Kalman gain `K[2]·T²`. Verified numerically on 2026-09-06
against a 3-state (x, v, a) discrete Kalman filter with
`Q = σ_w²·[T²/2, T, 1][T²/2, T, 1]ᵀ`: α and β match to three figures and γ
matches only under this convention — writing the update as `2γ/T²` (the
other common form) makes the same formula deliver twice the intended γ.
`pos_abg_cfg_from_index(cfg, lambda)` implements the closed form so an
operator tunes ONE number; the host test pins it against the Kalman
recursion at three λ values.

Numbers at `T = 0.2 s`, from a scalar simulation (white measurement noise,
one-sample outlier just under the 1.5 m gate):

| λ    | α     | β     | γ      | σ_out/σ_in | 1 m step: overshoot, settle | 1.4 m one-sample outlier: peak |
|------|-------|-------|--------|------------|-----------------------------|--------------------------------|
| 0.01 | 0.350 | 0.075 | 0.0081 | 0.55       | 25 %, 27 steps (5.4 s)      | 0.49 m                         |
| 0.02 | 0.419 | 0.113 | 0.0152 | 0.60       | 23 %, 21 steps (4.2 s)      | 0.59 m                         |
| 0.03 | 0.463 | 0.143 | 0.0220 | 0.64       | 23 %, 18 steps (3.6 s)      | 0.65 m                         |
| 0.05 | 0.521 | 0.190 | 0.0346 | 0.68       | 22 %, 15 steps (3.0 s)      | 0.73 m                         |
| 0.08 | 0.578 | 0.245 | 0.0520 | 0.72       | 20 %, 13 steps (2.6 s)      | 0.81 m                         |

**Default λ = 0.02** (α ≈ 0.42, β ≈ 0.11, γ ≈ 0.015): noise down to ~60 %, a
1 m relocation settled in ~4 s, a sub-gate outlier attenuated to ~40 % of its
size. That is a starting point for a visual judgement, not a tuned number —
the plan's hardware task sweeps λ ∈ {0.01, 0.02, 0.05} on the live gateway
and the maintainer picks by eye. Two things the table already says that
should temper expectations: an α-β-γ filter *overshoots* a step by ~20–25 %
whatever λ is (that is the γ term doing its job on a real acceleration, and
it is the price of following a tag that starts walking), and its noise
reduction is modest (0.55–0.72) because the third-order model must stay
responsive. If the map still looks too lively at λ = 0.01, the honest next
step is `alpha_still` and the gate, not a smaller λ.

Stability: `0 < α < 1`, `0 < β < 2`, `γ ≥ 0`, plus a settled step response
in the host test (impulse response decays; 1 m step settles within 5 % in
under 40 steps at every λ in the table). The host test asserts the
simulation, not a closed-form inequality.

#### Finding (simulation, 2026-09-06): the γ term is a liability under this deployment's dt spread — flagged, not silently overridden

The table above is at a CONSTANT 200 ms. This site does not deliver that:
the measured dt between a tag's consecutive solvable groups has p90 = 0.8 s
(2026-09-03, `tools/pos_trace.py`). Re-running the same scalar simulation
with dt drawn uniformly from [0.2, 1.0] s, σ_v = 0.35 m, a 4 m outlier every
997 cycles, the default λ = 0.02, `reset_after` = 5, 10⁵ cycles, dt-scaled
gate (all figures: gated cycles / streak reseeds / worst estimate error /
RMS error over σ_v):

| variant, dt ∈ [0.2, 1.0] s      | stationary tag        | walking 1 m/s         | half the cycles `still` |
|---------------------------------|-----------------------|-----------------------|-------------------------|
| α-β-γ, gains/dt (this spec)     | 95 / 2 / **8.6 m** / 1.14 | 21 / 0 / 1.8 m / 1.12 | 54 / 0 / 2.4 m / 0.84 |
| α-β-γ, gains at T_nom           | 1315 / 156 / 23.7 m / 2.77 | 1226 / 151 / 23.3 m / 2.66 | 121 / 8 / 20.7 m / 1.26 |
| **α-β (γ = 0), gains/dt**       | 58 / 0 / 2.3 m / **0.89** | 3 / 0 / 1.2 m / **0.88** | 48 / 0 / 2.3 m / **0.79** |
| α-β (γ = 0), gains at T_nom     | 98 / 6 / 10.7 m / 1.09 | 7 / 0 / 1.3 m / 1.03 | 76 / 4 / 10.6 m / 0.99 |
| *reference: constant dt 0.2 s*  | α-β-γ 416 / 7 / 2.0 m / 0.87 · α-β 302 / 0 / 1.3 m / 0.83 | | |

(The RMS column includes the injected outliers, which is why the constant-dt
reference reads 0.87 rather than the clean 0.60 of the table above.)

Read across: **with γ on, the filter is WORSE than the raw solve
(RMS ratio > 1) on both a stationary and a walking tag, and its worst
excursion reaches 8.6 m** — the acceleration state random-walks on
measurement noise and a single long dt then contributes ½·a·dt², up to 25×
the nominal step, which pushes the prediction past the gate; the state coasts
until the streak reseeds. **With γ = 0 the same filter is robust in every
column**: RMS ratio 0.79–0.89, worst error ≈ 2.3 m, zero reseeds. Scaling the
gains by the actual dt (this spec) is right and the alternative (gains fixed
at T_nom) is catastrophic for both variants. A gate that grows with dt helps
both, and is adopted (§3.2).

**What this spec does with that.** The maintainer asked for α-β-γ, and this
is a simulation with an assumed noise model, not a measurement — so the
structure stays α-β-γ, the compiled-in default keeps the λ-derived γ, and
three things make the finding impossible to miss on the bench rather than
deciding it in advance:

1. `gamma` is a first-class runtime knob (`blink abg gamma 0` makes the
   filter α-β on the live gateway with no reflash, §4.5).
2. The first hardware comparison in the plan is γ = default vs γ = 0 at the
   same λ, BEFORE the λ sweep — if the bench agrees with the table, the λ
   sweep runs on α-β and the default flips to γ = 0 in
   `pos_abg_cfg_defaults()`.
3. A host test pins the finding (`test_gamma_zero_is_more_robust_under_dt_jitter`):
   under the [0.2, 1.0] s dt draw, the worst excursion with γ = 0 must be
   smaller than with the default γ. If somebody later "fixes" the filter and
   that test starts failing, the finding has been re-measured, not lost.

**Recommendation, stated plainly:** unless the bench contradicts the table,
ship with γ = 0. An α-β filter with a dt-scaled gate is the configuration
that met every goal in §1 in simulation; the γ term did not, under the dt
distribution this site actually produces.

### 3.4 API

```c
#define POS_ABG_T_NOM_S      0.2f
#define POS_ABG_DT_MIN_S     0.05f
#define POS_ABG_GATE_DT_CAP  3.0f   /* effective gate = gate_m * min(CAP, max(1, dt/T_nom)) */

void pos_abg_cfg_defaults(struct pos_abg_cfg *c);            /* lambda 0.02, alpha_still 0.15, gate 1.5 m, reset_after 5 */
void pos_abg_cfg_from_index(struct pos_abg_cfg *c, float lambda); /* alpha/beta/gamma only; other fields untouched */
void pos_abg_reset(struct pos_abg *f);                       /* init = false */
void pos_abg_seed(struct pos_abg *f, float x, float y);      /* v = a = 0, streak = 0, init = true */
enum pos_abg_result pos_abg_step(struct pos_abg *f, const struct pos_abg_cfg *c,
				 float dt_s, float zx, float zy, bool still);
bool pos_abg_get(const struct pos_abg *f, float *x, float *y, float *vx, float *vy);
```

`pos_abg_get()` returns false until seeded, out-pointers untouched, any may be
NULL — same contract as `pos_ekf_get()` had, so the caller's "never read an
unseeded filter" habit carries over. There is no `pos_sigma()`: a fixed-gain
filter carries no covariance, and inventing one would be the overconfident
`sigma` story of the EKF again.

---

## 4. Integration in `tdoa_gw.c`

### 4.1 Per-tag memo

`struct tag_memo` gains `struct pos_abg abg;` (28 B) and `bool tag_moving;`
(latched from `obs.flags & BLINK_FLAG_MOVING` in `ingest_one()`, as before —
every observation of one blink carries the same byte). `x`/`y`/`pos_ms`/
`has_pos` keep their meaning: the last PUBLISHED position, used to seed
`tdoa_solve()` and to jump-gate it. `last_ref_t_dtu`/`has_ref_t` are
unchanged. One shared `static struct pos_abg_cfg abg_cfg`, set in
`tdoa_gw_init()`. Memory: `16 × 28 B` + config, under 0.5 kB against the
EKF's 1.7 kB.

### 4.2 `solve_one()` flow, per group

```
take_ready → fix_t_dtu = m[0].t_dtu → rebase → plausible? (drop: implausible)
memo_claim
if has_ref_t:
    raw_dt = sdelta40(fix_t_dtu, last_ref_t_dtu);  dt_s = raw_dt × 15.65 ps
    small negative  (|dt| <= TDOA_DT_REORDER_MAX_MS)        → drop, hold reference   [reorder]
    0 < dt < POS_ABG_DT_MIN_S (50 ms)                        → drop, hold reference   [reorder]  (a duplicate group of one blink)
    dt > TDOA_DT_MAX_MS (1000 ms), or large negative (alias) → dt_ok = false          [dt_reseed below]
    otherwise                                                → dt_ok = true
last_ref_t_dtu = fix_t_dtu; has_ref_t = true
resolve_one() → raw (zx, zy), or return (solve_fail / jump already counted)   -- publish nothing
if !pos_abg_get(): seed(z); publish                                            [seeded]
elif !dt_ok:       seed(z); publish                                            [dt_reseed]
else: switch pos_abg_step(dt_s, z, !tag_moving):
        ACCEPTED → publish                                                     [filtered]  (+[still] if the still branch ran)
        REJECTED → publish nothing                                             [gate_rejected]
        RESEEDED → publish                                                     [reseed]
```

Constants, all in `tdoa_gw.h` next to `TDOA_DT_REORDER_MAX_MS`:

- `TDOA_DT_MAX_MS` returns at **1000**, not the EKF's 2000: p90 of the
  measured dt distribution is 0.8 s, so 1 s keeps ~90 % of cycles on the
  filtered path while refusing to predict a walking tag through more than a
  second (1.5 m at walking pace). The part-2 plan's 600 ms candidate would
  reseed on >10 % of cycles, i.e. reintroduce jumps at exactly the cadence
  the filter exists to hide.
- `POS_ABG_DT_MIN_S = 0.05` in `pos_abg.h`: the floor under which `β/T` and
  `γ/T²` stop being meaningful, and which no pair of distinct blinks can
  produce.

**Publish rules, stated as invariants the counters must satisfy:**

```
fixes == seeded + dt_reseed + filtered + reseed
still <= filtered
gate_rejected cycles publish nothing; reorder groups are never solved
```

The first line is the check that caught the stale-republish bug on
2026-09-02; `tools/pos_trace.py` re-learns it for the new line.

### 4.3 What is published

`fix.x/y` = `pos_abg_get()` after the step (on ACCEPTED/RESEEDED/seed);
`fix.n_anchors` = the group's `n`; `fix.residual_m` = the solve's, zeroed at
`TDOA_MIN_ANCHORS` per `tdoa_solve.h`'s contract — every published fix has
a solve behind it now, so the EKF's "filtered fixes have no residual" warning
does not come back. `Tid` derivation unchanged. The memo's `x/y/pos_ms` follow
the published position.

### 4.4 `blink stats`

The `tdoa` line keeps `reorder`. A third line returns, named for what it is:

```
{"tdoa_abg":{"role":"gateway","seeded":N,"dt_reseed":N,"filtered":N,
             "gate_rejected":N,"reseed":N,"still":N}}
```

Two verdicts printed by the shell, same style as the existing ones:

- `still == filtered && filtered > 0` → the MOVING bit never arrives; check
  the anchors' firmware before trusting a still fleet (the proto-4 trap).
- `gate_rejected > filtered / 4` → the gate is rejecting a quarter or more of
  the evidence; either `gate_m` is too tight for this site's raw dispersion
  or the solve is producing outliers at a rate no filter should hide — read
  `implausible`, `jump` and `sync stats` before loosening the gate.

`tdoa_gw_abg_stats()` returns the six counters; `tools/pos_trace.py` parses
the new line and checks the `fixes` identity above.

### 4.5 Console knob, so the λ sweep does not cost a reflash per value

The whole evaluation of this filter is visual, on the live platform, and §3.3
asks for three λ values to be compared. A rebuild-and-reflash per value on
the deployed gateway is a half-hour per data point and interrupts service, so
the config is settable at runtime, **not persisted**, GATEWAY-only:

```
blink abg                      print the live cfg as JSON
                               {"abg":{"lambda":0.020,"alpha":..,"beta":..,"gamma":..,
                                       "alpha_still":0.15,"gate_m":1.50,"reset_after":5}}
blink abg lambda <value>       recompute alpha/beta/gamma from the tracking index
blink abg gamma <value>        set gamma directly; 0 makes the filter alpha-beta
                               (the §3.3 finding's comparison, without a reflash)
blink abg gate <metres>        set gate_m (at nominal dt; it scales with dt, §3.2)
blink abg still <alpha>        set alpha_still
blink abg reset <n>            set reset_after (0 = never reseed on the streak)
```

Not persisted on purpose: these are experiment values until the maintainer
has picked them, and the pick then goes into `pos_abg_cfg_defaults()` as the
compiled-in truth, which is what a reboot returns to. Persisting them would
mean a board carrying a tuning nobody can see in the source.

Concurrency: the shell thread writes `abg_cfg` while the `K_PRIO_COOP(0)`
gateway loop reads it field by field. The loop cannot be preempted by the
shell, but the shell CAN be preempted by the loop between two field writes,
so a `lambda` change (three fields) is fenced with `k_sched_lock()` /
`k_sched_unlock()` — the same fence `ccp_slave_residual_reset()` uses for the
one shell-reachable write in that module. Every filter uses the same shared
cfg, so a change applies to all tags on the next cycle; no per-tag state is
touched, so no reseed is needed.

---

## 5. Error handling and edge cases

| case | behaviour | counter |
|------|-----------|---------|
| first group ever for a tag (fresh memo) | solve, seed, publish | `seeded` |
| memo slot evicted (LRU) while a tag is live | next group is a cold start | `seeded` |
| `tdoa_solve()` fails or jump-gated | nothing published, filter untouched, clock reference advanced | `solve_fail` / `jump` |
| dt ≤ 0 small | group dropped, reference held | `reorder` |
| 0 < dt < 50 ms | group dropped, reference held | `reorder` |
| dt > 1 s or aliased large negative | seed on z, publish | `dt_reseed` |
| innovation > `gate_m` | predict only, nothing published | `gate_rejected` |
| `reset_after` consecutive rejections | seed on z, publish | `reseed` |
| MOVING bit absent (proto-4 anchor) | every cycle takes the still branch; shell warns | `still` |
| NaN/inf from the solve | `tdoa_solve()` already reports invalid; `pos_abg_step()` additionally refuses non-finite z with `POS_ABG_BAD_INPUT`, state untouched, counted as `solve_fail` | `solve_fail` |
| gateway reboot | all memos zeroed; every tag cold-starts | `seeded` |

Note on the row above: because the clock reference advances but the filter
does not, the NEXT successful cycle's dt can be inflated by however long the
failure lasted — see the comment in `tdoa_gw.c` above the `resolve_one()` call
for the full reasoning.

Warnings follow the module's rule: once per boot per condition, counters
carry magnitudes; the `K_PRIO_COOP(0)` loop and `CONFIG_LOG_MODE_OVERFLOW`
are why.

---

## 6. Testing

**Host (`tests/pos_abg/`, plain gcc, `-lm`):**

1. `cfg_from_index` matches the steady-state Kalman gains at λ ∈ {0.01,
   0.02, 0.08} to 1e-3 (the recursion is written into the test).
2. Step response settles within 5 % in < 40 steps for every λ in §3.3's
   table; impulse response decays (stability by simulation).
3. Stationary tag with white noise: output RMS / input RMS within ±0.05 of
   the table (0.60 at λ = 0.02).
4. Constant-velocity track: steady-state lag bounded (α-β-γ has zero lag on
   constant velocity; assert |error| < 2 cm after settling).
5. One-sample 1.4 m outlier on a stationary tag: peak deviation < 0.65 m at
   λ = 0.02 and back within 5 cm in 30 steps.
6. Gate: a 3 m outlier is REJECTED, state equals the prediction, `init`
   still true, nothing else moved. The gate scales with dt: at dt = 0.4 s a
   2.5 m innovation is accepted; at dt = 1.0 s the cap holds the gate at
   4.5 m (4.4 accepted, 4.6 rejected).
7. Streak: 5 consecutive 3 m "outliers" → RESEEDED on the fifth at the new
   position, velocity zero; 4 followed by a return → no reseed, streak back
   to 0.
8. Still: with `still = true`, velocity and acceleration are exactly zero
   after the step and the position moves by `alpha_still × r`.
9. dt: `dt <= 0` and NaN return `POS_ABG_BAD_INPUT` with state bit-identical;
   dt = 1.0 s (five nominal steps) does not blow up the corrections.
10. Long run (10⁵ steps, random dt in [0.2, 1.0], noise, occasional
    outliers): every state field finite throughout, and streak reseeds
    under 1 % of cycles.
11. The §3.3 finding, pinned: the same long run with `gamma = 0` has a
    smaller worst excursion than with the default γ (and no more reseeds).

**Integration (`tdoa_gw.c`, hardware — no host suite, same reasoning as
before: everything it calls is host-tested and the only thing left is the
counters):**

- Counter identity `fixes == seeded + dt_reseed + filtered + reseed` holds
  over a 10-minute two-tag run (one still, one walking), read by
  `tools/pos_trace.py`.
- Stationary dispersion (RMS about the mean of the published fixes over
  ≥ 5 min, tag placed at the triangle's centre — NOT on the base line, see
  CLAUDE.md's unobservable-y note) is below the raw solve's on the same
  capture (the raw figure comes from `blink stats`-free reasoning: run once
  with `alpha = 1, beta = gamma = 0`, which makes the filter the identity).
- Walking trace: every consecutive 200 ms step under ~30 cm, no
  back-steps, no gap-then-teleport inside the anchor hull.
- Injected outlier: with one anchor's `anchor pos` deliberately moved 2 m and
  restored during the run, the map shows no excursion beyond `gate_m` and
  the `gate_rejected` counter moves by the number of affected cycles.
- `kernel reboot cold` and re-join: cold start counted, no crash, identity
  holds again from zero.
- `gw_sf` heartbeat at exactly 200.0 ms throughout, no `"beacon started but
  TXFRS never completed"`.

---

## 7. Budget

- RAM: `struct pos_abg` 28 B × `TDOA_GW_SEED_SLOTS` (16) + `bool` per memo +
  cfg ≈ 0.5 kB `.bss`, versus the EKF's +1720 B. Net shrink.
- CPU per group: one solve (unchanged from the pre-EKF baseline) plus ~30
  flops. Nothing new to measure; `gw_sf` confirms.
- Stack: `pos_abg_step()` is a leaf with a dozen floats. `solve_one()`'s
  frame is unchanged apart from removing the EKF's locals.

---

## 8. Open, and deliberately left open

- **λ, `alpha_still`, `gate_m`, `reset_after` are starting points**, chosen
  from measured dispersion and cadence, to be judged by eye on the platform
  and then by the stationary number. They are not tuned against ground truth
  and cannot be until the survey is tape-validated.
- **Whether γ stays on is the maintainer's call, and the simulation in §3.3
  says it should not.** The default honours the request (λ-derived γ); the
  knob, the host test and the first bench comparison exist so that call is
  made on hardware evidence, not on a name.
- **Per-anchor `sigma_m` has no consumer** after this. It stays on the wire;
  a weighted `tdoa_solve()` is its natural home and is separate work.
- **The height model (`apos tagz`) and RX antenna-delay calibration** are
  where the accuracy is. This filter changes nothing about either and must
  not be read as progress on them.

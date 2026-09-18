/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor clock synchronisation model: converts a local DW3220 timestamp into a
 * master anchor's time base, from a stream of clock-calibration packets (CCPs).
 *
 * This is the Phase 2 de-risk for the TDoA migration, and the whole migration
 * turns on it. TDoA needs every anchor that hears a blink to timestamp it on a
 * COMMON clock to sub-nanosecond accuracy; nothing else in this firmware needs
 * that, and the MAC contract section 1 explicitly assumed it was unachievable
 * wirelessly. See docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md
 * section 4.2 for why that assumption is wrong on this hardware.
 *
 * Deliberately pure C with no Zephyr and no driver dependency, so the maths can
 * be host-tested to exhaustion BEFORE any radio work happens. If the numbers do
 * not hold here they will not hold on air, and finding that out costs a test run
 * rather than a bench session.
 *
 * ---- Where the precision comes from -------------------------------------
 *
 * Two primitives already exist and one is already in production:
 *
 *   dwt_readrxtimestamp()  40-bit RX timestamp, 1 LSB = 1 DTU = 15.65 ps.
 *                          That is 64 LSBs per nanosecond -- the resolution was
 *                          never the problem.
 *   dwt_readclockoffset()  carrier frequency offset, ~14.9 ppb per LSB, already
 *                          used by src/ss_initiator.c.
 *
 * Two independent crystals at +/-20 ppm drift 40 ppm relative, which over a
 * 200 ms CCP interval is 8 us -- five orders of magnitude past the target. So
 * the model must estimate the DRIFT RATE, not just the offset. Estimating it
 * from consecutive CCP timestamps beats CFO by a wide margin:
 *
 *   via CFO                ~15 ppb per LSB single-shot
 *   via two CCP timestamps 15.65 ps / 200 ms = 78 ppt
 *
 * CFO is therefore an acquisition aid and a cross-check, not the estimator.
 *
 * ---- Error budget, and why the phase is averaged ------------------------
 *
 * A two-point rate estimate over a baseline spanning `n` intervals of `T` has a
 * rate error of about jitter*sqrt(2)/(n*T) -- the 1/n scaling that
 * tag_testting/src/beacon_sched_core.c exists for, and which this module copies.
 * Extrapolating `tau` past the last observation, total error is
 *
 *     (jitter * sqrt(2) / (n * T)) * tau   +   jitter_of_the_phase_reference
 *
 * With T = tau = 200 ms the first term collapses fast, and past roughly ten
 * intervals it stops mattering:
 *
 *     jitter    n=1       n=10      dominant term at n=10
 *     100 ps    ~240 ps   ~114 ps   the phase reference
 *     1 ns      ~2.4 ns   ~1.14 ns  the phase reference
 *
 * That is the design-shaping result. beacon_sched_core uses its most recent raw
 * arrival as the phase reference, which is fine there because its target is a
 * millisecond-scale RX window. Here the reference's own noise IS the limiting
 * term, so the phase has to be FILTERED rather than taken raw -- hence the
 * residual EMA below.
 *
 * ---- What the simulation actually measures ------------------------------
 *
 * tests/sync_model/ simulates two crystals 40 ppm apart with timestamp noise.
 * Worst conversion error over the interval following the last CCP, averaged
 * over 12 seeds, at the assumed jitter:
 *
 *     observations   2     5     10    25    100   300
 *     error, DTU     14    4     4     5     4     3
 *
 * So the plateau is real and arrives by about five observations -- one second.
 * SYNC_BASELINE_USEFUL is set to 10 as a conservative statement of that. It IS
 * now the threshold the conversion gate switches on -- see its own definition
 * below for why that changed. Worth knowing: a SINGLE seed is far too
 * noisy to see this, and reading a trend off one gave 2 DTU at n=5 and 10 DTU
 * at n=10, which looks like the plateau does not exist.
 *
 * The deterministic floor with zero jitter is 0 DTU: the integer arithmetic
 * here is exact, and every DTU of error is noise, not rounding.
 *
 * ---- The Phase 2 gate ---------------------------------------------------
 *
 * SYNC_JITTER_DTU is an ASSUMPTION pending measurement on hardware. Sweeping
 * it gives the crossover the hardware measurement has to be compared against:
 *
 *     jitter    0     0.09  0.50  1.00  5.00  10.00 ns
 *     error     0     0.02  0.55  1.23  3.63  8.52  ns
 *
 * The target is crossed between roughly 0.5 ns and 1 ns of per-observation
 * timestamp jitter. That is the whole Phase 2 gate reduced to one number:
 * measure anchor-to-anchor CCP timestamp jitter, and if it comes in under
 * ~0.5 ns the TDoA migration proceeds. Above ~1 ns it does not, and the
 * remedies are a faster CCP rate or averaging more aggressively, both of which
 * cost airtime that section 3.2's budget would have to be re-run for.
 */

#ifndef SYNC_MODEL_H
#define SYNC_MODEL_H

#include <stdint.h>
#include <stdbool.h>

/* DW3220 device time units. 1 DTU = 1 / (499.2 MHz * 128) = 15.65 ps. */
#define SYNC_DTU_PER_NS       64u        /* 63.8976, rounded for budgeting */
#define SYNC_DTU_BITS         40u
#define SYNC_DTU_MASK         0xFFFFFFFFFFULL

/* Nominal CCP interval, one superframe. 200 ms in DTU. */
#define SYNC_CCP_INTERVAL_DTU 12780000000ULL

/* Per-observation timestamp jitter, DTU, 1 sigma. **ASSUMPTION** -- 100 ps is
 * an optimistic-but-plausible figure for a clean LOS anchor-to-anchor link and
 * is what the error table above is built on. Measuring it is the Phase 2 gate
 * (task A7); a value near 1 ns changes the achievable accuracy but not the
 * structure of this module. */
#define SYNC_JITTER_DTU       6u         /* ~100 ps */

/* Baseline past which the rate estimate stops being the dominant error term,
 * per the table above. Kept as a named constant because it is a CONCLUSION,
 * not a tuning knob: growing it does not improve accuracy once the phase
 * reference dominates.
 *
 * ---- 2026-09-18: this IS now a threshold something switches on ----------
 *
 * An earlier version of this comment said it was "not as a threshold anything
 * switches on", and that was exactly the defect (B13,
 * docs/diag-tdoa-precision-stage1-audit.md section 2): sync_model_observe()
 * used to set `valid` as soon as one interval had completed, so every
 * re-baseline published conversions built on a TWO-POINT rate estimate. At the
 * jitter this hardware actually measures (92-98 DTU at 3 m) that is a ~10.8 ppb
 * rate error, which over one 200 ms extrapolation is ~2.2 ns ~= 0.65 m -- about
 * 35x the converged state -- while the anchor went on publishing its long-run
 * sigma as if nothing had happened.
 *
 * It is now the gate: a model that has never converged converts NOTHING, and a
 * model that has been re-baselined converts on the CARRIED skew (below) with a
 * sigma that says so. */
#define SYNC_BASELINE_USEFUL  10u

/* ---- Carrying a converged skew across a re-baseline --------------------
 *
 * A re-baseline (root change, sequence discontinuity, gateway reboot) throws
 * away the rate estimate but not the physics: the two crystals' relative rate
 * did not change just because the sequence numbering did. So the last
 * CONVERGED ratio is kept and used until the fresh baseline reaches
 * SYNC_BASELINE_USEFUL -- which is what makes the gate above affordable
 * instead of a 2-second blackout after every discontinuity.
 *
 * How long a carried skew may be trusted is a physics question, and the answer
 * is the drift RATE bound below times its age. Both constants are named rather
 * than folded into one number because they are assumptions with different
 * provenance and a future measurement may move either one.
 */

/* Bound on how fast the RELATIVE rate of two crystals can change, ppb per
 * second. The figure quoted for a plain XO earlier in this header is ~8 ppb/s
 * (a thermal-transient figure -- a settled crystal in a settled room is three
 * orders of magnitude quieter, and the same sentence's "1.6 ppt per 200 ms"
 * is that settled regime, so the header carries both). 10 rounds the transient
 * figure UP, which is the safe direction here: assuming a LARGER drift rate
 * makes a carried skew go stale SOONER, and the fallback when it does is
 * simply the pre-existing behaviour of waiting for a fresh baseline. Assuming
 * a smaller one would let a genuinely stale skew keep converting. */
#define SYNC_SKEW_DRIFT_PPB_PER_S  10u

/* Admissible rate error, ppb, for a carried skew -- the budget its age is
 * spent against, and the figure the published sigma charges for the whole
 * un-converged window.
 *
 * 5 ppb sits between the two states it has to beat and be beaten by:
 *
 *   converged estimator  ~1.1 ppb  (98 DTU * 15.65 ps * sqrt(2) /
 *                                   (SYNC_BASELINE_USEFUL * 200 ms))
 *   CARRIED, this bound   5.0 ppb  -> 1.0 ns over a 200 ms extrapolation
 *   two-point estimate   ~10.8 ppb -> 2.2 ns, which is B13 itself
 *
 * So a carried skew is always better than the state it replaces and always
 * worse than the state it is waiting for, which is exactly the ordering the
 * published sigma has to express. */
#define SYNC_SKEW_MAX_ERR_PPB      5u

/* The two above, resolved into an age limit. SYNC_SKEW_MAX_ERR_PPB /
 * SYNC_SKEW_DRIFT_PPB_PER_S seconds, expressed in DTU (there are 5 CCP
 * intervals in a second at SYNC_CCP_INTERVAL_DTU).
 *
 * At today's numbers this is 0.5 s -- two and a half CCP intervals -- and the
 * consequence is worth stating plainly rather than discovering on a bench.
 *
 * The carry covers a discontinuity with no real outage behind it (a root
 * change, a local-clock mismatch on an otherwise consecutive frame, a reject
 * spanning one or two superframes). It does NOT cover a multi-second gateway
 * reboot: the age is re-evaluated at every observation, so a carried skew goes
 * stale part-way through the un-converged window and the anchor then publishes
 * NOTHING until its fresh baseline converges. That silence is the correct
 * outcome and is the gate doing its job -- at 10 ppb/s a skew carried across
 * several seconds would be worse than the two-point estimate it is standing in
 * for, so bridging further would reintroduce B13 by another route.
 *
 * Read that together: the bulk of the B13 fix is the GATE and the dynamic
 * sigma. The carry is a bounded refinement that removes the common,
 * sub-second discontinuities from the gate's cost, not a way to make a reboot
 * free. Carrying further would mean asserting a drift-rate bound nobody on
 * this project has measured. */
#define SYNC_SKEW_MAX_AGE_DTU                                                  \
	(((uint64_t)SYNC_CCP_INTERVAL_DTU * 5ULL *                             \
	  (uint64_t)SYNC_SKEW_MAX_ERR_PPB) /                                   \
	 (uint64_t)SYNC_SKEW_DRIFT_PPB_PER_S)

/* Per-observation jitter, DTU, assumed by the error estimate before this
 * anchor has gathered enough of its OWN residuals to measure it.
 *
 * Deliberately NOT SYNC_JITTER_DTU. That constant is the design-time
 * ASSUMPTION (~100 ps) the tables above are built on; the Fase 2 hardware
 * measurement came in at 782 ps to 1.53 ns, so using it as a fallback would
 * hand a consumer a sigma ~15x too confident during exactly the window where
 * the model is least trustworthy. 96 DTU is the measured 3 m figure. The
 * fallback errs pessimistic on purpose: a sigma that is too large costs some
 * down-weighting, a sigma that is too small is silent overconfidence. */
#define SYNC_JITTER_FALLBACK_DTU   96u

/* Residual samples below which sync_model_jitter_est_dtu() is too noisy to
 * stand in for the fallback above. Matches the baseline threshold rather than
 * inventing a second number. */
#define SYNC_JITTER_EST_MIN_N      SYNC_BASELINE_USEFUL

/* Hard cap on baseline length, after which it is rolled forward onto its own
 * midpoint. Bounding it is what lets a slow change in either crystal (thermal
 * drift) be tracked rather than averaged away over the anchor's uptime -- the
 * same reason beacon_sched_core bounds BSCHED_BASELINE_MAX. */
#define SYNC_BASELINE_MAX     128u

/* Residual EMA shift: correction = residual >> SYNC_PHASE_EMA_SHIFT. A shift of
 * 3 averages the phase over ~8 observations, cutting reference noise by ~sqrt(8)
 * while still following a real step within a second. Integer shift rather than a
 * fractional gain so no float and no division appear on this path.
 *
 * Wrapped in #ifndef, not a bare #define, so a host test can override it with
 * -DSYNC_PHASE_EMA_SHIFT=N to sweep it -- the same reason tools/tag_density.py
 * exposes its constants on the CLI: a value that is a measured CONCLUSION has
 * to be re-measurable by whoever re-derives it.
 *
 * ---- 2026-09-02 sweep result: NEGATIVE, shift stays at 3 -----------------
 *
 * Swept {3, 4, 5, 6} x jitter {0, 6, 32, 64, 320, 640, 3200} DTU, 12 seeds
 * each, via tests/sync_model/test_sync_model.c's
 * test_shift_sensitivity_sweep() (worst_error(), SYNC_BASELINE_USEFUL*2
 * observations, mean of 12 seeds per cell, each shift built separately with
 * -DSYNC_PHASE_EMA_SHIFT=N since it is a compile-time constant). Worst cell
 * in the whole table is the highest jitter (3200 DTU), and it is also the
 * cell where a real gain would show up most, since the phase term this
 * constant controls scales with jitter:
 *
 *     shift   jitter=3200 DTU, mean worst error   vs shift=3
 *     3       1928 DTU                            --
 *     4       1831 DTU                             -5.0%
 *     5       1772 DTU                             -8.1%
 *     6       1759 DTU                             -8.7%
 *
 * Checked again at the cell closest to the actual Fase 2 hardware jitter
 * measurement (~92-98 DTU, between the 64 and 320 DTU rows): at jitter=64,
 * shift 3->4/5/6 is 40->37 DTU (-7.5%); at jitter=320, 205->198/196/196 DTU
 * (-3.4% to -4.4%). All figures, at every jitter level, land far under the
 * design spec's 20% bar (section 2.5) -- **shift stays 3, unchanged**.
 *
 * Confirms the algebra in that section (section 2.1): the EMA already
 * contributes only ~6% of the converted error at shift=3 (the dominant term
 * is the single noisy phase-reference sample, which no amount of averaging
 * on the RESIDUAL side of the loop can touch), so raising it buys almost
 * nothing. The simulation's feedback loop (section 2.2) narrows the gap
 * between "EMA-limited" and "phase-reference-limited" slightly versus a pure
 * quadrature-sum guess, but not by enough to change the verdict.
 *
 * Raising the shift also has a real cost that was NOT charged against it
 * above: SYNC_RESIDUAL_TO_JITTER (below) is empirical for shift=3, and the
 * same sweep run showed the shift-4/5/6 builds under-report the true jitter
 * (inferred 340/334/332 DTU against a true sigma of 369 DTU at amplitude 640,
 * versus 350 at shift=3) -- exactly the trap the design spec's section 2.4
 * warns about, and it would have to be re-derived for zero measured benefit.
 *
 * **Do not re-propose raising this constant without new evidence** -- e.g. a
 * hardware jitter figure with a qualitatively different noise shape than the
 * uniform noise this simulation assumes. */
#ifndef SYNC_PHASE_EMA_SHIFT
#define SYNC_PHASE_EMA_SHIFT  3
#endif

/* Consecutive missed CCPs after which the model stops claiming validity.
 * Coasting is safe for a while -- the residual after drift correction is second
 * order (thermal change in the drift RATE, ~8 ppb/s for a plain XO, i.e. 1.6 ppt
 * per 200 ms interval) -- but not forever, and a stale model that still reports
 * valid would silently poison every position it converts. */
#define SYNC_MISS_MAX         25u

struct sync_model {
    /* Raw 40-bit stamps of the most recent observation, for differencing. */
    uint64_t l_raw;
    uint64_t m_raw;
    /* Cumulative local and master time since the baseline started, as signed
     * 64-bit sums of per-interval deltas. Accumulating rather than storing raw
     * endpoints is what removes the 40-bit wrap (17.2 s) as a limit on baseline
     * length -- otherwise a baseline over ~86 intervals could not be
     * represented at all. */
    int64_t  l_acc;
    int64_t  m_acc;
    /* Phase reference: master time at the last observation, plus a filtered
     * correction. See the residual EMA discussion above. */
    int64_t  phase_corr;
    uint32_t n_obs;          /* observations in the current baseline */
    uint32_t misses;         /* consecutive missed CCPs */
    bool     have_raw;       /* l_raw/m_raw usable for differencing */
    bool     valid;          /* sync_model_to_master() would succeed */
    /* Last CONVERGED rate, carried across a re-baseline. Stored as the same
     * (numerator, denominator) pair the conversion uses -- (m_acc - l_acc,
     * l_acc) -- rather than as a ppb scalar, because a ppb integer resolves
     * only 1 part in 1e9 and one CCP interval of extrapolation turns that into
     * ~12.8 DTU of error. Keeping the exact rational costs 17 bytes and loses
     * nothing. `skew_l_raw` is the local timestamp it was last refreshed at,
     * which is what SYNC_SKEW_MAX_AGE_DTU is measured against. */
    int64_t  skew_num;
    int64_t  skew_den;
    uint64_t skew_l_raw;
    bool     have_skew;
    /* Residual statistics. See sync_model_residual_rms_dtu() -- these are what
     * turn the Phase 2 gate into a self-measurement rather than a bench setup
     * with external equipment. Survive everything except an explicit reset
     * (sync_model_residual_reset()) or a full sync_model_init(); in particular
     * they now survive a sync_model_rebaseline(), because the link's jitter is
     * a property of the link and not of the sequence numbering. Only
     * CONVERGED observations are folded in -- see sync_model_observe(). */
    uint64_t res_sq_sum;
    uint32_t res_n;
    uint32_t res_max;
    /* Lifetime counters. Cleared ONLY by sync_model_init(), so they survive
     * every re-baseline -- which is the whole point: a counter that a
     * re-baseline resets cannot measure re-baselines. */
    uint32_t n_rebaseline;
    uint32_t n_miss_total;
};

void sync_model_init(struct sync_model *m);

/* Start a fresh baseline WITHOUT forgetting everything the model has learned.
 *
 * This is what a caller reaches for when the sequence numbering became
 * meaningless (a gateway reboot, a sequence discontinuity, a root change) but
 * the hardware did not: the rate estimate built against the old numbering
 * describes nothing, while the link's measured jitter and the two crystals'
 * relative rate are unchanged.
 *
 * So the baseline (l_acc/m_acc/n_obs/phase_corr/have_raw) is cleared, and:
 *   - the residual statistics are KEPT (they measure the link, not the
 *     baseline -- and a caller that wants them gone has
 *     sync_model_residual_reset());
 *   - the lifetime counters are KEPT and n_rebaseline is incremented;
 *   - the last converged skew is KEPT when `keep_skew`, so conversion can
 *     continue through the un-converged window -- see SYNC_SKEW_MAX_AGE_DTU.
 *
 * `keep_skew` MUST be false when the master itself changed (a different
 * root_id is a different clock, so the carried ratio describes the wrong pair
 * of crystals) and true when only this node's view of the sequence broke. */
void sync_model_rebaseline(struct sync_model *m, bool keep_skew);

/* Feed one CCP. `m_dtu` is the master's transmit time as carried in the CCP
 * payload; `l_dtu` is our own RX timestamp of it. Both are raw 40-bit DW3220
 * values and may wrap freely. */
void sync_model_observe(struct sync_model *m, uint64_t m_dtu, uint64_t l_dtu);

/* Record that an expected CCP did not arrive. */
void sync_model_miss(struct sync_model *m);

/* Convert a local 40-bit timestamp into the master's time base. Returns false
 * (leaving *out untouched) when the model cannot convert, which is any of:
 *
 *   - no phase reference yet (nothing observed since the last re-baseline);
 *   - coasted past SYNC_MISS_MAX;
 *   - the current baseline is shorter than SYNC_BASELINE_USEFUL **and** there
 *     is no usable carried skew to stand in for it. That second clause is the
 *     B13 gate: a two-point rate estimate is ~35x worse than a converged one
 *     at this hardware's measured jitter, and publishing conversions built on
 *     one -- which is what this function used to do -- is worse than
 *     publishing nothing, because nothing downstream could tell. */
bool sync_model_to_master(const struct sync_model *m, uint64_t l_dtu,
                          uint64_t *out);

/* Estimated 1-sigma conversion error, DTU, for a timestamp `ahead_dtu` after
 * the last observation. This is the figure the Phase 2 hardware gate measures
 * against, so it is exposed rather than left implicit. Returns UINT32_MAX when
 * the model is not valid.
 *
 * DYNAMIC, since 2026-09-18, in two ways that both matter:
 *
 *   - the jitter it is built from is this anchor's OWN measured residual
 *     statistic once it has SYNC_JITTER_EST_MIN_N of them, not the design-time
 *     SYNC_JITTER_DTU assumption (which is ~15x optimistic against what this
 *     hardware measures). Before then it is SYNC_JITTER_FALLBACK_DTU, which
 *     errs pessimistic.
 *   - the PHASE term now accounts for how many observations the residual EMA
 *     has actually averaged. Straight after a re-baseline the phase reference
 *     is one unfiltered sample, not 2^SYNC_PHASE_EMA_SHIFT of them, and the
 *     old fixed phase term said otherwise.
 *
 * The result is non-increasing in n_obs by construction, which is the property
 * a consumer needs and tests/sync_model/ pins: it can never claim to be more
 * confident during the un-converged window than it is once converged. */
uint32_t sync_model_error_dtu(const struct sync_model *m, uint32_t ahead_dtu);

/* The 1-sigma conversion error over a FULL CCP interval -- the worst case
 * within a superframe, and therefore the figure to publish to a consumer that
 * will weight this anchor's observation against another's.
 *
 * Exists as its own call rather than sync_model_error_dtu(m, ONE_INTERVAL)
 * because SYNC_CCP_INTERVAL_DTU (1.278e10) does not fit that function's
 * uint32_t `ahead_dtu`.
 *
 * Returns 0 when the model cannot convert. Zero is also the wire sentinel for
 * "unknown" on the blink observation topic, which is the correct reading here:
 * an anchor that cannot convert publishes no observation at all, so the value
 * never travels. When the model CAN convert the result is clamped to a minimum
 * of 1, so a valid sigma can never be mistaken for the sentinel. */
uint32_t sync_model_sigma_dtu(const struct sync_model *m);

/* ---- Lifetime counters, for the console -------------------------------
 *
 * These exist because B13 was invisible: every symptom of a re-baseline
 * (conversions built on a two-point rate, a phase reference averaged over one
 * sample) is indistinguishable at the console from a healthy anchor, and the
 * one statistic that was published -- the long-run residual RMS -- does not
 * move during the transient at all. `sync stats` surfaces all three. */
uint32_t sync_model_rebaseline_count(const struct sync_model *m);
uint32_t sync_model_miss_count(const struct sync_model *m);
uint32_t sync_model_n_obs(const struct sync_model *m);

/* Relative drift of the local clock against the master, in parts per billion,
 * signed. POSITIVE means the local clock runs fast. Diagnostics and the CFO
 * cross-check; not used by the conversion, which works in exact integer ratios
 * instead. */
int32_t sync_model_drift_ppb(const struct sync_model *m);

/* ---- Residual statistics: the Phase 2 gate, self-measured --------------
 *
 * Every observation is predicted before it is folded in, and the residual is
 * (actual master time) - (predicted master time). The actual value is exact --
 * it is the master's own declared transmit time -- so the residual is a
 * measurement of the local RX timestamp noise. That makes the Phase 2 gate a
 * self-measurement: flash two anchors, let them run, read one number. No
 * reference instrument, no known distance, no external clock.
 *
 * But the residual is NOT equal to the jitter, and an earlier version of this
 * comment claimed it was. The prediction consumes TWO noisy local timestamps --
 * the new observation's, and the phase reference's, through the `d = l - l_raw`
 * term -- so the residual differences two independent samples and its RMS sits
 * at sqrt(2) * jitter as a floor, with the phase EMA and the rate term adding a
 * little more. Measured across five noise amplitudes in tests/sync_model/ the
 * factor is about 1.55.
 *
 * That factor matters at the gate in the direction that misleads: an operator
 * reading 64 DTU of RMS and taking it for 1 ns of jitter would be looking at
 * about 0.65 ns of real jitter and rejecting hardware that passes.
 * sync_model_jitter_est_dtu() applies the conversion, and it is the value to
 * compare against the sweep table -- not the raw RMS.
 *
 * SYNC_RESIDUAL_TO_JITTER is EMPIRICAL, from the simulation. It should be
 * re-derived against hardware if the observed RMS and an independently measured
 * jitter ever disagree; the tests pin the relation so a change is visible.
 *
 * All in DTU. rms returns 0 before there are any observations. */
/* Ratio of residual RMS to true timestamp jitter, x1000. See above: sqrt(2) is
 * the floor from differencing two independent samples, and 1.55 is what the
 * simulation measures once the phase EMA and rate term are included. */
#define SYNC_RESIDUAL_TO_JITTER  1550u

uint32_t sync_model_residual_rms_dtu(const struct sync_model *m);
uint32_t sync_model_residual_max_dtu(const struct sync_model *m);
uint32_t sync_model_residual_count(const struct sync_model *m);
void     sync_model_residual_reset(struct sync_model *m);

/* Per-observation timestamp jitter inferred from the residuals. THIS is the
 * number to compare against the sweep table at the top of this header, and the
 * one the Phase 2 gate turns on. */
uint32_t sync_model_jitter_est_dtu(const struct sync_model *m);

#endif /* SYNC_MODEL_H */

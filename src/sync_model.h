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
 * SYNC_BASELINE_USEFUL is set to 10 as a conservative statement of that, not
 * as a threshold anything switches on. Worth knowing: a SINGLE seed is far too
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
 * reference dominates. */
#define SYNC_BASELINE_USEFUL  10u

/* Hard cap on baseline length, after which it is rolled forward onto its own
 * midpoint. Bounding it is what lets a slow change in either crystal (thermal
 * drift) be tracked rather than averaged away over the anchor's uptime -- the
 * same reason beacon_sched_core bounds BSCHED_BASELINE_MAX. */
#define SYNC_BASELINE_MAX     128u

/* Residual EMA shift: correction = residual >> SYNC_PHASE_EMA_SHIFT. A shift of
 * 3 averages the phase over ~8 observations, cutting reference noise by ~sqrt(8)
 * while still following a real step within a second. Integer shift rather than a
 * fractional gain so no float and no division appear on this path. */
#define SYNC_PHASE_EMA_SHIFT  3

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
    bool     valid;          /* a usable rate estimate exists */
};

void sync_model_init(struct sync_model *m);

/* Feed one CCP. `m_dtu` is the master's transmit time as carried in the CCP
 * payload; `l_dtu` is our own RX timestamp of it. Both are raw 40-bit DW3220
 * values and may wrap freely. */
void sync_model_observe(struct sync_model *m, uint64_t m_dtu, uint64_t l_dtu);

/* Record that an expected CCP did not arrive. */
void sync_model_miss(struct sync_model *m);

/* Convert a local 40-bit timestamp into the master's time base. Returns false
 * (leaving *out untouched) while the model has no rate estimate or has coasted
 * past SYNC_MISS_MAX. */
bool sync_model_to_master(const struct sync_model *m, uint64_t l_dtu,
                          uint64_t *out);

/* Estimated 1-sigma conversion error, DTU, for a timestamp `ahead_dtu` after
 * the last observation. This is the figure the Phase 2 hardware gate measures
 * against, so it is exposed rather than left implicit. Returns UINT32_MAX when
 * the model is not valid. */
uint32_t sync_model_error_dtu(const struct sync_model *m, uint32_t ahead_dtu);

/* Relative drift of the local clock against the master, in parts per billion,
 * signed. Diagnostics and the CFO cross-check; not used by the conversion,
 * which works in exact integer ratios instead. */
int32_t sync_model_drift_ppb(const struct sync_model *m);

#endif /* SYNC_MODEL_H */

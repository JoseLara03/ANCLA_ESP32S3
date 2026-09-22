/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See sync_model.h for the error budget this implements and why the phase is
 * filtered separately from the rate.
 */

#include "sync_model.h"

/* Signed difference between two 40-bit DW3220 timestamps.
 *
 * The counter wraps every 2^40 DTU = 17.2 s, so an unsigned compare is wrong
 * across the wrap and the failure is rare, timing-dependent, and looks like a
 * radio fault. Same discipline beacon_guard.c applies to hi32, one field wider:
 * sign-extend the masked difference from bit 39. Correct for any interval under
 * 2^39 DTU (~8.6 s), which is 43 CCP intervals -- far past anything this module
 * differences in one step. */
static int64_t sdelta40(uint64_t a, uint64_t b)
{
    uint64_t d = (a - b) & SYNC_DTU_MASK;

    if (d & (1ULL << (SYNC_DTU_BITS - 1))) {
        return (int64_t)d - (int64_t)(1ULL << SYNC_DTU_BITS);
    }
    return (int64_t)d;
}

/* Integer square root, for the error estimate. No <math.h>: this header is
 * compiled into a Zephyr image where float is soft-ABI, and the estimate needs
 * no more than integer precision. */
static uint32_t isqrt32(uint64_t v)
{
    uint64_t r = 0;
    uint64_t bit = 1ULL << 32;

    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* ---- Which rate the conversion is entitled to use ----------------------
 *
 * Three states, and keeping them in one place is what stops sync_model_observe(),
 * sync_model_to_master() and sync_model_error_dtu() from disagreeing about
 * whether this model is usable -- which is a class of bug that would present
 * as a sigma published for a conversion that never happened, or worse, the
 * reverse.
 */

static bool baseline_converged(const struct sync_model *m)
{
    return m->n_obs >= SYNC_BASELINE_USEFUL && m->l_acc != 0;
}

/* Is the skew carried over from before the last re-baseline still young enough
 * to stand in for a converged baseline?
 *
 * Age is measured at the most recent OBSERVATION (m->l_raw), not at whatever
 * timestamp is being converted, so that this predicate cannot flip between two
 * blinks inside one superframe.
 *
 * sdelta40() is correct to +/-8.6 s and SYNC_SKEW_MAX_AGE_DTU is a fraction of
 * a second, so an outage long enough to alias the 40-bit difference yields a
 * value that is either negative or far past the limit. Both are rejected --
 * which is the safe direction: an aliased age can only ever cause a usable
 * skew to be DISCARDED, never a stale one to be trusted. */
static bool carried_skew_usable(const struct sync_model *m)
{
    if (!m->have_skew || m->skew_den == 0) {
        return false;
    }

    int64_t age = sdelta40(m->l_raw, m->skew_l_raw);

    return age >= 0 && (uint64_t)age <= SYNC_SKEW_MAX_AGE_DTU;
}

/* The (numerator, denominator) the conversion should apply, or false when the
 * model may not convert at all. */
static bool effective_skew(const struct sync_model *m, int64_t *num,
                           int64_t *den)
{
    if (!m->have_raw || m->misses > SYNC_MISS_MAX) {
        return false;
    }
    if (baseline_converged(m)) {
        *num = m->m_acc - m->l_acc;
        *den = m->l_acc;
        return true;
    }
    if (carried_skew_usable(m)) {
        *num = m->skew_num;
        *den = m->skew_den;
        return true;
    }
    return false;
}

/* Clear the BASELINE only: everything that describes the current run of
 * observations, and nothing that describes the link or the hardware. Shared by
 * sync_model_init() and sync_model_rebaseline() so the two cannot drift apart
 * about what a baseline is. */
static void clear_baseline(struct sync_model *m)
{
    m->l_raw      = 0;
    m->m_raw      = 0;
    m->l_acc      = 0;
    m->m_acc      = 0;
    m->phase_corr = 0;
    m->n_obs      = 0;
    m->misses     = 0;
    m->have_raw   = false;
    m->valid      = false;
}

void sync_model_init(struct sync_model *m)
{
    clear_baseline(m);
    m->skew_num     = 0;
    m->skew_den     = 0;
    m->skew_l_raw   = 0;
    m->have_skew    = false;
    m->res_sq_sum   = 0;
    m->res_n        = 0;
    m->res_max      = 0;
    m->n_rebaseline = 0;
    m->n_miss_total = 0;
    /* Deterministic LEGACY (tau=0) rather than whatever an uninitialised
     * caller's stack held. A caller that owns a configured tau (ccp_slave.c)
     * must call sync_model_set_tau() again after this -- see tau_dtu's own
     * doc comment in sync_model.h. */
    m->tau_dtu      = 0;
}

void sync_model_rebaseline(struct sync_model *m, bool keep_skew)
{
    clear_baseline(m);
    if (!keep_skew) {
        m->skew_num   = 0;
        m->skew_den   = 0;
        m->skew_l_raw = 0;
        m->have_skew  = false;
    }
    if (m->n_rebaseline < UINT32_MAX) m->n_rebaseline++;
}

uint32_t sync_model_residual_rms_dtu(const struct sync_model *m)
{
    if (m->res_n == 0) return 0;
    return isqrt32(m->res_sq_sum / m->res_n);
}

uint32_t sync_model_jitter_est_dtu(const struct sync_model *m)
{
    /* Divide out SYNC_RESIDUAL_TO_JITTER. The residual differences two noisy
     * timestamps, so the raw RMS overstates the per-observation jitter by that
     * factor -- taking the RMS for the jitter would reject hardware that
     * actually passes the gate. */
    return (sync_model_residual_rms_dtu(m) * 1000u) / SYNC_RESIDUAL_TO_JITTER;
}

uint32_t sync_model_residual_max_dtu(const struct sync_model *m)
{
    return m->res_max;
}

uint32_t sync_model_residual_count(const struct sync_model *m)
{
    return m->res_n;
}

void sync_model_residual_reset(struct sync_model *m)
{
    m->res_sq_sum = 0;
    m->res_n      = 0;
    m->res_max    = 0;
}

/* Roll the baseline forward onto its own midpoint, halving its span while
 * keeping a rate estimate. Without this the baseline grows without bound and a
 * slow thermal change in either crystal is averaged against observations from
 * minutes ago -- the model would report a rate the hardware no longer has. */
static void roll_baseline(struct sync_model *m)
{
    m->l_acc /= 2;
    m->m_acc /= 2;
    m->n_obs = m->n_obs / 2u;
}

void sync_model_observe(struct sync_model *m, uint64_t m_dtu, uint64_t l_dtu)
{
    m_dtu &= SYNC_DTU_MASK;
    l_dtu &= SYNC_DTU_MASK;

    /* B14 fix: train against the master's TX time as it would read at THIS
     * anchor's antenna, not at the master's -- see sync_model_set_tau()'s
     * doc comment in sync_model.h for the full derivation. tau_dtu is 0 for
     * every caller that has not opted in (LEGACY, or a host test that never
     * touches it), so this is a no-op until then. */
    m_dtu = (uint64_t)((int64_t)m_dtu + (int64_t)m->tau_dtu) & SYNC_DTU_MASK;

    if (!m->have_raw) {
        m->l_raw    = l_dtu;
        m->m_raw    = m_dtu;
        m->have_raw = true;
        m->misses   = 0;
        return;                      /* one point is not a baseline */
    }

    /* Predict this observation BEFORE folding it in, so the residual measures
     * the model against an arrival it has not yet seen. Folding first would
     * make the residual partly self-referential and the EMA would chase its own
     * tail instead of averaging reference noise. */
    {
        uint64_t pred = 0;

        if (sync_model_to_master(m, l_dtu, &pred)) {
            int64_t residual = sdelta40(m_dtu, pred);
            uint64_t mag = (uint64_t)(residual < 0 ? -residual : residual);

            m->phase_corr += residual >> SYNC_PHASE_EMA_SHIFT;

            /* Accumulated BEFORE the correction is applied, so the statistic
             * measures the model's open-loop prediction error rather than what
             * is left after it has already corrected for it. The latter would
             * report a number that shrinks with the EMA gain and says nothing
             * about the hardware.
             *
             * Only while CONVERGED, and that condition is new (2026-09-18).
             * The residual statistics are the long-run measurement of this
             * link's jitter, and they now survive a re-baseline
             * (sync_model_rebaseline()) so that survival must not come at the
             * cost of poisoning them: during the un-converged window the rate
             * is a carried skew and the phase reference is barely filtered, so
             * those residuals measure the TRANSIENT, not the link. This is the
             * same effect docs/anchor-sync-measurement.md section 4.1 records
             * as "`sync reset` is worth 3.7x" -- an un-reset reading at 30 cm
             * came back 188 DTU against a true 50 -- fixed at the source
             * instead of left to an operator procedure. */
            if (baseline_converged(m)) {
                m->res_sq_sum += mag * mag;
                m->res_n++;
                if (mag > m->res_max) m->res_max = (uint32_t)mag;
            }
        }
    }

    int64_t dl = sdelta40(l_dtu, m->l_raw);
    int64_t dm = sdelta40(m_dtu, m->m_raw);

    m->l_acc += dl;
    m->m_acc += dm;
    m->l_raw  = l_dtu;
    m->m_raw  = m_dtu;
    m->misses = 0;

    if (m->n_obs < UINT32_MAX) m->n_obs++;

    if (m->n_obs >= SYNC_BASELINE_MAX) roll_baseline(m);

    /* Refresh the skew that a future re-baseline will carry, but ONLY from a
     * converged baseline -- carrying a two-point estimate across a
     * discontinuity would preserve exactly the error B13 is about instead of
     * bridging over it. */
    if (baseline_converged(m)) {
        m->skew_num   = m->m_acc - m->l_acc;
        m->skew_den   = m->l_acc;
        m->skew_l_raw = m->l_raw;
        m->have_skew  = true;
    }

    {
        int64_t num, den;

        m->valid = effective_skew(m, &num, &den);
    }
}

void sync_model_miss(struct sync_model *m)
{
    int64_t num, den;

    if (m->misses < UINT32_MAX) m->misses++;
    if (m->n_miss_total < UINT32_MAX) m->n_miss_total++;
    m->valid = effective_skew(m, &num, &den);
}

bool sync_model_to_master(const struct sync_model *m, uint64_t l_dtu,
                          uint64_t *out)
{
    int64_t num, den;

    /* The whole gate lives in effective_skew(): no phase reference, coasted
     * past SYNC_MISS_MAX, or a baseline shorter than SYNC_BASELINE_USEFUL with
     * no usable carried skew behind it. */
    if (!effective_skew(m, &num, &den)) {
        return false;
    }

    int64_t d = sdelta40(l_dtu & SYNC_DTU_MASK, m->l_raw);

    /* The correction is carried SEPARATELY from the identity rather than
     * applying the ratio dm/dl to d directly.
     *
     *   naive: m_raw + d * m_acc / l_acc
     *   here:  m_raw + d + d * (m_acc - l_acc) / l_acc
     *
     * They are algebraically the same, but the naive form overflows: d is up to
     * one CCP interval (1.28e10 DTU) and m_acc up to 128 of them (1.64e12), so
     * the product reaches 2.1e22 against an int64 ceiling of 9.2e18. In this
     * form the multiplier is (m_acc - l_acc), which is drift * l_acc and so at
     * 40 ppm reaches only 6.5e7 -- the product tops out near 8.3e17, eleven
     * times inside the ceiling. Splitting it also means the identity term is
     * exact and only the small correction is ever rounded.
     *
     * The same bound holds for a CARRIED skew: skew_num/skew_den are a copy of
     * a (m_acc - l_acc, l_acc) pair from a converged baseline, so they are the
     * same magnitudes this argument already covers. */
    int64_t corr = (d * num) / den;

    *out = ((uint64_t)((int64_t)m->m_raw + d + corr + m->phase_corr)) &
           SYNC_DTU_MASK;
    return true;
}

/* The per-observation jitter to build the error estimate from: this anchor's
 * OWN measured residual statistic once there are enough residuals to believe
 * it, and a pessimistic fallback before that.
 *
 * NOT SYNC_JITTER_DTU. That is the design-time assumption (~100 ps) the tables
 * in the header are built on; the hardware measured 782 ps to 1.53 ns, so
 * building a published sigma on it would be ~15x overconfident -- the same
 * trap src/blink_rx.c already documents for sync_model_error_dtu() itself, and
 * the reason that file reached past this function for jitter_est_dtu(). It no
 * longer has to. */
static uint32_t working_jitter_dtu(const struct sync_model *m)
{
    if (m->res_n >= SYNC_JITTER_EST_MIN_N) {
        uint32_t j = sync_model_jitter_est_dtu(m);

        if (j > 0u) {
            return j;
        }
    }
    return SYNC_JITTER_FALLBACK_DTU;
}

/* Shared body of sync_model_error_dtu() and sync_model_sigma_dtu(), taking
 * `ahead` as a uint64_t so a full CCP interval (1.278e10, past uint32_t) can be
 * asked for. */
static uint32_t error_dtu_ahead(const struct sync_model *m, uint64_t ahead_dtu)
{
    int64_t num, den;

    if (!effective_skew(m, &num, &den)) {
        return UINT32_MAX;
    }

    uint32_t jit = working_jitter_dtu(m);
    uint64_t rate_term;

    if (baseline_converged(m)) {
        /* Rate term: the two-point rate error is jitter*sqrt(2)/span, and
         * extrapolating `ahead` scales it by ahead/span. Computed as
         * jitter * sqrt(2) * ahead / span with the sqrt(2) folded in as 1.4142
         * via 14142/10000 to stay in integers.
         *
         * Overflow: jit is bounded by the uint16 the wire carries (65535), so
         * the widest product here is 65535 * 14142 * 1.278e10 = 1.19e19,
         * inside the 1.84e19 uint64 ceiling. */
        uint64_t span = (uint64_t)(m->l_acc < 0 ? -m->l_acc : m->l_acc);

        rate_term = ((uint64_t)jit * 14142ULL * ahead_dtu) /
                    (10000ULL * span);
    } else {
        /* Running on a CARRIED skew. Its error is not an estimator variance --
         * it was estimated over a long converged baseline -- it is STALENESS,
         * and carried_skew_usable() has already bounded that at
         * SYNC_SKEW_MAX_ERR_PPB by refusing anything older than
         * SYNC_SKEW_MAX_AGE_DTU. So the rate term is that bound applied over
         * `ahead`, which is larger than the converged term by construction:
         * the published sigma can never claim more confidence during the
         * un-converged window than after it. */
        rate_term = (ahead_dtu * (uint64_t)SYNC_SKEW_MAX_ERR_PPB) /
                    1000000000ULL;
    }

    /* Phase term: the reference's own noise, reduced by the EMA's effective
     * averaging length -- at most 2^SYNC_PHASE_EMA_SHIFT observations, but only
     * as many as it has actually SEEN since the last re-baseline. The old fixed
     * form claimed the full window from the first observation onward, which is
     * the half of B13 that survives even with a carried skew: the rate may be
     * bridged over, the phase reference cannot be. n_obs counts intervals, so
     * the number of samples the EMA has folded in is n_obs + 1. */
    uint32_t avg_n = 1u << SYNC_PHASE_EMA_SHIFT;

    if (m->n_obs + 1u < avg_n) {
        avg_n = m->n_obs + 1u;
    }
    /* jitter / sqrt(avg_n), in integers: scale the numerator by 1000 and take
     * the root of avg_n scaled by 1000^2 so the two scalings cancel. */
    uint32_t phase_term = (uint32_t)(((uint64_t)jit * 1000ULL) /
                                     isqrt32((uint64_t)avg_n * 1000000ULL));

    /* Independent terms add in quadrature. */
    uint64_t sq = rate_term * rate_term +
                  (uint64_t)phase_term * (uint64_t)phase_term;

    return isqrt32(sq);
}

uint32_t sync_model_error_dtu(const struct sync_model *m, uint32_t ahead_dtu)
{
    return error_dtu_ahead(m, (uint64_t)ahead_dtu);
}

uint32_t sync_model_sigma_dtu(const struct sync_model *m)
{
    uint32_t e = error_dtu_ahead(m, SYNC_CCP_INTERVAL_DTU);

    if (e == UINT32_MAX) {
        return 0u;          /* cannot convert -- nothing will be published */
    }
    return (e == 0u) ? 1u : e;   /* 0 is the wire's "unknown" sentinel */
}

uint32_t sync_model_rebaseline_count(const struct sync_model *m)
{
    return m->n_rebaseline;
}

uint32_t sync_model_miss_count(const struct sync_model *m)
{
    return m->n_miss_total;
}

uint32_t sync_model_n_obs(const struct sync_model *m)
{
    return m->n_obs;
}

void sync_model_set_tau(struct sync_model *m, int32_t new_tau_dtu)
{
    int64_t delta = (int64_t)new_tau_dtu - (int64_t)m->tau_dtu;

    m->tau_dtu = new_tau_dtu;

    /* Feed-forward (FR-5): shift the running absolute reference by the same
     * delta a fresh observation would have applied, so the NEXT
     * sync_model_to_master() call already reflects the new tau -- no wait
     * for the next CCP, no re-baseline. l_acc/m_acc/skew_num/skew_den need
     * no correction: they are sums of differences between two m_dtu values
     * that both carried the SAME tau at the time they were observed, so a
     * change here does not touch anything already folded into them (see the
     * header comment for the algebra). Left untouched when there is no
     * observation yet (!have_raw) -- there is nothing to shift, and
     * sync_model_observe() applies the new tau to its very first m_dtu on
     * its own. */
    if (m->have_raw) {
        m->m_raw = (uint64_t)((int64_t)m->m_raw + delta) & SYNC_DTU_MASK;
    }
}

int32_t sync_model_get_tau(const struct sync_model *m)
{
    return m->tau_dtu;
}

int32_t sync_model_drift_ppb(const struct sync_model *m)
{
    if (!m->valid || m->m_acc == 0) return 0;

    /* (l_acc - m_acc) / m_acc, scaled to parts per billion.
     *
     * Sign convention: POSITIVE means the local clock runs FAST relative to the
     * master, which is what "drift of the local clock against the master"
     * reads as. Note this is the opposite sign from the `num` the conversion
     * uses -- that one is (m_acc - l_acc), because it corrects local time
     * TOWARD master time. Getting these two confused is easy, so the tests pin
     * both directions.
     *
     * Denominator is m_acc, NOT l_acc. Dividing by the local span yields
     * d/(1+d) rather than d -- referred to the wrong timebase. It is a
     * second-order difference and harmless in isolation, but it is a real 2 ppb
     * discrepancy at 40 ppm and it made a test that asked for 40000 ppb read
     * back 39998. The conversion in sync_model_to_master() is unaffected either
     * way: it works in exact integer ratios and never uses this value.
     *
     * The numerator is small (drift * m_acc) so the 1e9 scaling cannot
     * overflow: at 40 ppm over a 128-interval baseline it is 6.5e7, and
     * 6.5e7 * 1e9 = 6.5e16, well inside int64. */
    int64_t num = (m->l_acc - m->m_acc) * 1000000000LL;

    return (int32_t)(num / m->m_acc);
}

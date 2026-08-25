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

void sync_model_init(struct sync_model *m)
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
    if (m->valid) {
        uint64_t pred = 0;

        if (sync_model_to_master(m, l_dtu, &pred)) {
            int64_t residual = sdelta40(m_dtu, pred);

            m->phase_corr += residual >> SYNC_PHASE_EMA_SHIFT;
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

    /* A baseline needs a nonzero local span before a ratio means anything. */
    if (m->l_acc != 0) m->valid = true;

    if (m->n_obs >= SYNC_BASELINE_MAX) roll_baseline(m);
}

void sync_model_miss(struct sync_model *m)
{
    if (m->misses < UINT32_MAX) m->misses++;
    if (m->misses > SYNC_MISS_MAX) m->valid = false;
}

bool sync_model_to_master(const struct sync_model *m, uint64_t l_dtu,
                          uint64_t *out)
{
    if (!m->valid || m->l_acc == 0 || m->misses > SYNC_MISS_MAX) {
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
     * exact and only the small correction is ever rounded. */
    int64_t num  = m->m_acc - m->l_acc;
    int64_t corr = (d * num) / m->l_acc;

    *out = ((uint64_t)((int64_t)m->m_raw + d + corr + m->phase_corr)) &
           SYNC_DTU_MASK;
    return true;
}

uint32_t sync_model_error_dtu(const struct sync_model *m, uint32_t ahead_dtu)
{
    if (!m->valid || m->l_acc == 0 || m->misses > SYNC_MISS_MAX) {
        return UINT32_MAX;
    }

    /* Rate term: the two-point rate error is jitter*sqrt(2)/span, and
     * extrapolating `ahead` scales it by ahead/span. Computed as
     * jitter * sqrt(2) * ahead / span with the sqrt(2) folded in as 1.4142
     * via 14142/10000 to stay in integers. */
    uint64_t span = (uint64_t)(m->l_acc < 0 ? -m->l_acc : m->l_acc);
    uint64_t rate_term =
            ((uint64_t)SYNC_JITTER_DTU * 14142ULL * (uint64_t)ahead_dtu) /
            (10000ULL * span);

    /* Phase term: the reference's own noise, reduced by the EMA's effective
     * averaging length of 2^SYNC_PHASE_EMA_SHIFT observations. This is the term
     * that dominates once the baseline is past SYNC_BASELINE_USEFUL, which is
     * the whole reason the EMA exists. */
    uint32_t avg_n = 1u << SYNC_PHASE_EMA_SHIFT;
    /* jitter / sqrt(avg_n), in integers: scale the numerator by 1000 and take
     * the root of avg_n scaled by 1000^2 so the two scalings cancel. */
    uint32_t phase_term = (SYNC_JITTER_DTU * 1000u) /
                          isqrt32((uint64_t)avg_n * 1000000ULL);

    /* Independent terms add in quadrature. */
    uint64_t sq = rate_term * rate_term +
                  (uint64_t)phase_term * (uint64_t)phase_term;

    return isqrt32(sq);
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

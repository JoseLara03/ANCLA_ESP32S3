#ifndef POS_EKF_H
#define POS_EKF_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "pos_solver.h"   /* struct pos_meas */
#include "tdoa_solve.h"   /* struct tdoa_meas, TDOA_M_PER_DTU */

/*
 * Tightly-coupled EKF, over raw UWB ranges or over TDoA range DIFFERENCES.
 *
 * State is [x, y, vx, vy], constant velocity with white-noise acceleration.
 * Measurements are the individual ranges (or range differences), NOT the
 * solved position: filtering the least-squares output discards the geometry,
 * whose error is non-Gaussian, correlated with GDOP, and changes character
 * every time the anchor count or set changes. Fusing ranges directly also
 * gets the dz model and per-measurement gating for free.
 *
 * Updates are sequential and scalar -- one range (or range difference) at a
 * time, a 4x1 gain and one scalar divide. There is no matrix inverse
 * anywhere, which is why this module needs neither CMSIS-DSP nor Zephyr and
 * is fully host-testable.
 *
 * The accelerometer contributes ZUPT and process-noise scheduling only. The
 * LIS2HH12 is a 3-axis accelerometer, so yaw is unobservable and body->room
 * rotation is impossible; it is a mode discriminator, not an inertial sensor.
 *
 * ---- Ownership --------------------------------------------------------------
 *
 * Copied verbatim from tag_testting/src/pos_ekf.{c,h} on 2026-09-02, same
 * precedent and same rule as pos_solver.c/pos_residual.c in this directory
 * (see this project's own CLAUDE.md entry on those two files):
 * pos_ekf_update_ranges() and everything above it in this file is the tag's
 * original range-based filter, UNCHANGED. pos_ekf_update_tdoa() below, and
 * the r_tdoa field on struct pos_ekf_cfg, are new -- added only here, for
 * the gateway's TDoA use, and the tag has no need of either.
 *
 * The 2026-09-02 design spec (section 4.1) that authorised this copy assumed
 * pos_ekf was already dead code on the tag, on the premise that "Phase 3
 * moved position-solving off the tag onto the gateway". That premise does
 * NOT hold: tag_testting/src/uwb_net_runner.c still calls the full
 * pos_ekf_seed()/predict()/update_ranges()/zupt()/needs_reseed() sequence on
 * its TWR-ranging path (the `UWB_ACT_RUN_SWEEP` branch that is not blink
 * mode) -- Phase 3 added a SECOND, blink-only path alongside the original
 * one, it did not replace it. So, exactly as with pos_solver.c/pos_residual.c
 * already in this file's own project: *the tag still owns the base file
 * while it still solves its own range-based fix*, ownership has NOT moved,
 * and tag_testting/CLAUDE.md was deliberately left untouched rather than
 * being edited to claim otherwise. Whoever next re-copies the range-based
 * half of this file from the tag must diff it first and treat any
 * divergence as a decision -- the same warning CLAUDE.md already carries for
 * cal_math.c, whose divergence incident is exactly what an unreviewed extra
 * field or extra function on one side and not the other can cause.
 *
 * See spec/2026-08-22-position-filtering-design.md for the original
 * range-based design, and
 * docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md section 4
 * for the TDoA extension.
 */

struct pos_ekf_cfg {
    float sigma_a_still;  /* process noise accel std dev, still   (m/s^2) */
    float sigma_a_move;   /* process noise accel std dev, walking (m/s^2) */
    float r_range;        /* range measurement std dev            (m)     */
    /* TDoA range-DIFFERENCE measurement std dev (m). NOT r_range: a range
     * difference is two independent noisy timestamps subtracted, so its sigma
     * is ~sqrt(2) times a single timestamp's. Default 0.6 m is derived from
     * the Phase 2 hardware sync jitter measurement (~1.44-1.53 ns per anchor,
     * ~45 cm 1-sigma range error -- docs/anchor-sync-measurement.md section
     * 4.1) via that sqrt(2), and is coupled to it: if a future SYNC_PHASE_EMA_
     * SHIFT sweep (sync_model.h) ever lowers the measured jitter, re-derive
     * this default alongside it. Reusing r_range here would tell the filter
     * to trust a TDoA measurement far more than it deserves, which reads as
     * LESS smooth, not more. */
    float r_tdoa;
    float r_zupt;         /* zero-velocity pseudo-measurement std dev (m/s) */
    float gate_k;         /* innovation gate, in sigmas (e.g. 3.0)        */
    float v_max;          /* speed clamp                          (m/s)   */
    uint8_t reset_after;  /* consecutive all-gated fixes before a reset   */
};

struct pos_ekf {
    float   x[4];        /* x, y, vx, vy */
    float   P[16];       /* row-major 4x4 covariance */
    bool    init;        /* false until pos_ekf_seed() */
    uint8_t gate_streak; /* consecutive fixes in which every range was gated */
};

/* Compiled-in starting point; tuned against captured data (see the `dbg` log
 * in the design). Callers may override any field. */
void pos_ekf_cfg_defaults(struct pos_ekf_cfg *c);

/* Drop all state. The filter is uninitialised until the next pos_ekf_seed(). */
void pos_ekf_reset(struct pos_ekf *f);

/* Initialise position from a snapshot fix, zero velocity, inflated covariance.
 * Also used for the divergence recovery path. */
void pos_ekf_seed(struct pos_ekf *f, float x, float y);

/* Constant-velocity prediction over dt_s seconds. `moving` selects
 * sigma_a_move vs sigma_a_still. dt_s must be the ACTUAL elapsed time -- it
 * varies with tier and with skipped superframes and must never be assumed. */
void pos_ekf_predict(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                     float dt_s, bool moving);

/*
 * Fuse `n` ranges as sequential scalar updates, each gated at gate_k sigma of
 * (H P H^T + R). Returns the number of ranges accepted.
 *
 * When every range is gated the internal streak counter advances; once it
 * reaches cfg->reset_after the caller should re-seed from a snapshot solve
 * (query with pos_ekf_needs_reseed()). This is the kidnapped-tag case, and the
 * case where the tag was carried while the accelerometer reported still.
 */
int pos_ekf_update_ranges(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                          const struct pos_meas *m, size_t n);

/*
 * Fuse `n` TDoA observations as n-1 sequential scalar range-DIFFERENCE
 * updates against reference m[0], each gated at gate_k sigma -- the same
 * mechanics, same gate_streak bookkeeping and same speed clamp as
 * pos_ekf_update_ranges(), with the measurement model swapped for a
 * hyperbolic one:
 *
 *   h = r_i - r_0
 *   H = [ (x-x_i)/r_i - (x-x_0)/r_0 ,  (y-y_i)/r_i - (y-y_0)/r_0 , 0, 0 ]
 *   z = (t_i - t_0) * TDOA_M_PER_DTU
 *
 * `m[k].dz` follows struct pos_meas's convention (anchor z minus tag z), the
 * same convention tdoa_solve.c's slant() uses. Returns the number of range
 * differences accepted, out of n-1.
 */
int pos_ekf_update_tdoa(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                        const struct tdoa_meas *m, size_t n);

/* Zero-velocity update: two scalar pseudo-measurements vx = 0, vy = 0. Apply
 * when the LIS2HH12 reports inactive. This is the single largest visual
 * improvement available, because a stationary tag is the common case. */
void pos_ekf_zupt(struct pos_ekf *f, const struct pos_ekf_cfg *c);

/* True once the all-gated streak has reached cfg->reset_after. */
bool pos_ekf_needs_reseed(const struct pos_ekf *f, const struct pos_ekf_cfg *c);

/* Current estimate. Returns false if the filter has never been seeded; the
 * out-pointers are then untouched. Any of them may be NULL. */
bool pos_ekf_get(const struct pos_ekf *f, float *x, float *y,
                 float *vx, float *vy);

/* Position standard deviation, sqrt(P[0,0] + P[1,1]) -- a scalar spread figure
 * for diagnostics and for deciding whether a fix is worth publishing.
 *
 * Returns 0.0f for an unseeded filter, because P is zeroed until the first
 * seed. That reads as *maximal* confidence, so a bare `if (sigma < thresh)
 * publish()` would publish garbage from a filter that has never had a fix.
 * Always gate on pos_ekf_get()'s return value first. */
float pos_ekf_pos_sigma(const struct pos_ekf *f);

#endif /* POS_EKF_H */

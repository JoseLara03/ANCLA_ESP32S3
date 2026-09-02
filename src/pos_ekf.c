#include "pos_ekf.h"
#include <math.h>
#include <string.h>

/*
 * Sequential-scalar EKF -- see the "The filter: tightly-coupled EKF over raw
 * ranges" section of spec/2026-08-22-position-filtering-design.md for the
 * pseudocode this file follows line for line. No CMSIS-DSP, no Zephyr:
 * every update is a 4x1 gain and a scalar divide, never a matrix inverse, so
 * the whole module is plain C99 and host-testable.
 *
 * P is stored row-major, P[i*4+j]. State order is fixed: x, y, vx, vy.
 */

/* Seed covariance. pos_ekf_seed() takes no cfg (see pos_ekf.h), so these are
 * fixed constants rather than derived from a caller's v_max.
 *
 * Position: (1.5 m)^2. The seed is always fed a snapshot fix -- either the
 * very first pos_solve() at cold start, or the divergence-recovery re-seed --
 * so it is an untrusted position, not "we know nothing about it". 1.5 m std
 * dev is generous next to r_range (0.12 m default) so the first few range
 * updates are not fighting an overconfident prior, while still tight enough
 * that the 3-sigma gate (~4.5 m radius against a single 0.12 m range) does
 * not itself reject the first few real measurements.
 *
 * Velocity: (1.0 m/s)^2, half of the default v_max (2.0 m/s). Both cold start
 * and reseed assert vx = vy = 0 with no actual evidence of it -- reseed in
 * particular follows a run of gated-out ranges, which is exactly the
 * kidnapped-tag/carried-while-still case where the old velocity estimate is
 * worthless. A prior loose enough to reach v_max within 2 sigma lets a
 * genuinely moving tag's first couple of range updates pull velocity toward
 * the truth quickly, without being so loose that one noisy early range flings
 * velocity to the clamp on a single bad gain.
 */
#define POS_EKF_SEED_POS_VAR   (1.5f * 1.5f)
#define POS_EKF_SEED_VEL_VAR   (1.0f * 1.0f)

/* dt guard. Non-positive dt means no time passed -- skip prediction rather
 * than divide the state by a meaningless transition. The upper clamp guards
 * against a caller passing a bogus elapsed time (e.g. a wrapped/uninitialised
 * clock read): Q grows as dt^4, so an absurd dt would otherwise blow P up to
 * numbers that stop being numerically useful in a single step. 120 s is
 * roughly 2x the longest planned IDLE-tier skip in the rate ladder
 * (design doc: 300 superframes * ~202 ms =~ 60 s), with margin for a future
 * tier that skips further than today's defaults.
 */
#define POS_EKF_DT_MAX  120.0f

/* Symmetrize P in place. Sequential scalar updates in float drift asymmetric
 * over many iterations (the (I - K H) P form is not symmetry-preserving under
 * rounding even though it is exact in real arithmetic), and an asymmetric P
 * is how these filters silently go non-positive-definite. Cheap to redo after
 * every predict/update, so it is done unconditionally rather than only when
 * something looks wrong. */
static void ekf_symmetrize(struct pos_ekf *f)
{
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            float avg = 0.5f * (f->P[i * 4 + j] + f->P[j * 4 + i]);
            f->P[i * 4 + j] = avg;
            f->P[j * 4 + i] = avg;
        }
    }
}

/* Clamp |v| to v_max, preserving direction. Applied after every update batch
 * (range fuse or ZUPT) per the design's "speed gate". */
static void ekf_clamp_speed(struct pos_ekf *f, float v_max)
{
    float vx = f->x[2];
    float vy = f->x[3];
    float speed = sqrtf(vx * vx + vy * vy);

    if (v_max > 0.0f && speed > v_max) {
        float scale = v_max / speed;
        f->x[2] = vx * scale;
        f->x[3] = vy * scale;
    }
}

/*
 * One sequential scalar Kalman update: innov = z - h, with row vector H
 * (only 4 elements, most callers have two of them zero) and scalar
 * measurement variance R.
 *
 * If gate_k > 0 the update is gated: it is skipped (returns false, state
 * untouched) when innov^2 > gate_k^2 * S. Passing gate_k <= 0 disables
 * gating -- this is how pos_ekf_zupt() reuses this exact path: the design
 * calls for ZUPT to always apply, never gate.
 *
 * This is the single update path referenced throughout pos_ekf.h's
 * comments ("Same scalar update path -- reuse it, do not duplicate it").
 */
static bool ekf_scalar_update(struct pos_ekf *f, const float H[4],
                              float innov, float R, float gate_k)
{
    float PHt[4];

    for (int i = 0; i < 4; i++) {
        float s = 0.0f;
        for (int j = 0; j < 4; j++) {
            s += f->P[i * 4 + j] * H[j];
        }
        PHt[i] = s;
    }

    float S = R;
    for (int i = 0; i < 4; i++) {
        S += H[i] * PHt[i];
    }

    if (S <= 0.0f) {
        return false; /* degenerate covariance -- refuse rather than divide */
    }

    if (gate_k > 0.0f && innov * innov > (gate_k * gate_k) * S) {
        return false; /* innovation gate */
    }

    float K[4];
    for (int i = 0; i < 4; i++) {
        K[i] = PHt[i] / S;
    }

    for (int i = 0; i < 4; i++) {
        f->x[i] += K[i] * innov;
    }

    /* P = (I - K H) P = P - K (H P). H P is recomputed explicitly (rather
     * than reusing PHt, which is P H^T) so this is correct even if P has
     * drifted slightly asymmetric since the last symmetrize. */
    float HP[4];
    for (int j = 0; j < 4; j++) {
        float s = 0.0f;
        for (int i = 0; i < 4; i++) {
            s += H[i] * f->P[i * 4 + j];
        }
        HP[j] = s;
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            f->P[i * 4 + j] -= K[i] * HP[j];
        }
    }

    ekf_symmetrize(f);
    return true;
}

void pos_ekf_cfg_defaults(struct pos_ekf_cfg *c)
{
    c->sigma_a_still = 0.05f;
    c->sigma_a_move  = 1.2f;
    /* ASSUMPTION pending the measurement campaign (design doc "Temporary
     * instrumentation" section) -- the static soak has not run yet, so 0.12 m
     * is the design's stated placeholder, not a fitted value. */
    c->r_range       = 0.12f;
    /* Derived from the Fase 2 hardware sync jitter (~1.44-1.53 ns per anchor,
     * ~45 cm 1-sigma range error) via sqrt(2) for the range-DIFFERENCE this
     * measures -- see pos_ekf.h's comment on this field for the full
     * derivation and its dependency on the sync_model.h shift sweep. */
    c->r_tdoa        = 0.6f;
    c->r_zupt        = 0.02f;
    c->gate_k        = 3.0f;
    c->v_max         = 2.0f;
    c->reset_after   = 3;
}

void pos_ekf_reset(struct pos_ekf *f)
{
    memset(f->x, 0, sizeof(f->x));
    memset(f->P, 0, sizeof(f->P));
    f->init        = false;
    f->gate_streak = 0;
}

void pos_ekf_seed(struct pos_ekf *f, float x, float y)
{
    memset(f->P, 0, sizeof(f->P));

    f->x[0] = x;
    f->x[1] = y;
    f->x[2] = 0.0f;
    f->x[3] = 0.0f;

    f->P[0 * 4 + 0]  = POS_EKF_SEED_POS_VAR;
    f->P[1 * 4 + 1]  = POS_EKF_SEED_POS_VAR;
    f->P[2 * 4 + 2]  = POS_EKF_SEED_VEL_VAR;
    f->P[3 * 4 + 3]  = POS_EKF_SEED_VEL_VAR;

    f->init        = true;
    f->gate_streak = 0;
}

void pos_ekf_predict(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                     float dt_s, bool moving)
{
    if (!f->init) {
        return; /* nothing to predict before the first seed */
    }
    if (dt_s <= 0.0f) {
        return; /* no elapsed time -- see the dt guard comment above */
    }
    if (dt_s > POS_EKF_DT_MAX) {
        dt_s = POS_EKF_DT_MAX;
    }

    float x0  = f->x[0];
    float y0  = f->x[1];
    float vx0 = f->x[2];
    float vy0 = f->x[3];

    f->x[0] = x0 + dt_s * vx0;
    f->x[1] = y0 + dt_s * vy0;
    /* vx, vy unchanged -- constant velocity */

    /*
     * P = F P F^T + Q, closed form for the sparse
     *     F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
     * rather than a generic 4x4 matmul: FP's bottom two rows equal P's
     * (F leaves vx,vy rows alone), and its top two rows are P's row plus
     * dt times the corresponding velocity row. The second F^T multiply
     * folds the same way over columns.
     */
    float FP[16];
    for (int j = 0; j < 4; j++) {
        FP[0 * 4 + j] = f->P[0 * 4 + j] + dt_s * f->P[2 * 4 + j];
        FP[1 * 4 + j] = f->P[1 * 4 + j] + dt_s * f->P[3 * 4 + j];
        FP[2 * 4 + j] = f->P[2 * 4 + j];
        FP[3 * 4 + j] = f->P[3 * 4 + j];
    }

    float P_new[16];
    for (int i = 0; i < 4; i++) {
        P_new[i * 4 + 0] = FP[i * 4 + 0] + dt_s * FP[i * 4 + 2];
        P_new[i * 4 + 1] = FP[i * 4 + 1] + dt_s * FP[i * 4 + 3];
        P_new[i * 4 + 2] = FP[i * 4 + 2];
        P_new[i * 4 + 3] = FP[i * 4 + 3];
    }

    float sigma_a = moving ? c->sigma_a_move : c->sigma_a_still;
    float q       = sigma_a * sigma_a;
    float dt2     = dt_s * dt_s;
    float dt3     = dt2 * dt_s;
    float dt4     = dt3 * dt_s;

    P_new[0 * 4 + 0] += q * dt4 / 4.0f;
    P_new[0 * 4 + 2] += q * dt3 / 2.0f;
    P_new[2 * 4 + 0] += q * dt3 / 2.0f;
    P_new[2 * 4 + 2] += q * dt2;

    P_new[1 * 4 + 1] += q * dt4 / 4.0f;
    P_new[1 * 4 + 3] += q * dt3 / 2.0f;
    P_new[3 * 4 + 1] += q * dt3 / 2.0f;
    P_new[3 * 4 + 3] += q * dt2;

    memcpy(f->P, P_new, sizeof(f->P));
    ekf_symmetrize(f);
}

int pos_ekf_update_ranges(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                          const struct pos_meas *m, size_t n)
{
    if (!f->init || n == 0) {
        return 0;
    }

    int accepted = 0;

    for (size_t k = 0; k < n; k++) {
        float dx = f->x[0] - m[k].x;
        float dy = f->x[1] - m[k].y;
        float dz = m[k].dz;
        float h  = sqrtf(dx * dx + dy * dy + dz * dz);

        if (h <= 0.0f) {
            continue; /* degenerate geometry -- Jacobian undefined, skip */
        }

        float H[4]   = { dx / h, dy / h, 0.0f, 0.0f };
        float innov  = m[k].range_m - h;
        float R      = c->r_range * c->r_range;

        if (ekf_scalar_update(f, H, innov, R, c->gate_k)) {
            accepted++;
        }
    }

    if (accepted > 0) {
        f->gate_streak = 0;
    } else if (f->gate_streak < UINT8_MAX) {
        /* Saturate rather than let a long out-of-coverage run wrap an
         * 8-bit counter back through 0, which would falsely clear
         * pos_ekf_needs_reseed(). */
        f->gate_streak++;
    }

    ekf_clamp_speed(f, c->v_max);
    return accepted;
}

int pos_ekf_update_tdoa(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                        const struct tdoa_meas *m, size_t n)
{
    if (!f->init || n == 0) {
        return 0;
    }

    int accepted = 0;

    for (size_t k = 1; k < n; k++) {
        float dx0 = f->x[0] - m[0].x;
        float dy0 = f->x[1] - m[0].y;
        float dz0 = m[0].dz;
        float r0  = sqrtf(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);

        float dxk = f->x[0] - m[k].x;
        float dyk = f->x[1] - m[k].y;
        float dzk = m[k].dz;
        float rk  = sqrtf(dxk * dxk + dyk * dyk + dzk * dzk);

        if (r0 <= 0.0f || rk <= 0.0f) {
            continue; /* degenerate geometry -- Jacobian undefined, skip */
        }

        float H[4] = { dxk / rk - dx0 / r0, dyk / rk - dy0 / r0, 0.0f, 0.0f };

        /* TRAP: subtract the two 40-bit device timestamps in int64_t BEFORE
         * any float conversion. They run up to ~2^40 and a float32's 24-bit
         * mantissa only resolves ~65536 DTU (307 m) at that magnitude --
         * converting to float first and subtracting after silently destroys
         * the measurement while still returning a plausible-looking number.
         * Same trap tdoa_solve.c documents twice against tdoa_meas; it does
         * not inherit automatically to this second consumer of the struct. */
        int64_t dt_dtu = m[k].t_dtu - m[0].t_dtu;
        float   z      = (float)dt_dtu * TDOA_M_PER_DTU;
        float   h      = rk - r0;
        float   innov  = z - h;
        float   R      = c->r_tdoa * c->r_tdoa;

        if (ekf_scalar_update(f, H, innov, R, c->gate_k)) {
            accepted++;
        }
    }

    if (accepted > 0) {
        f->gate_streak = 0;
    } else if (f->gate_streak < UINT8_MAX) {
        /* Saturate, same reasoning as pos_ekf_update_ranges(). */
        f->gate_streak++;
    }

    ekf_clamp_speed(f, c->v_max);
    return accepted;
}

void pos_ekf_zupt(struct pos_ekf *f, const struct pos_ekf_cfg *c)
{
    if (!f->init) {
        return;
    }

    float R = c->r_zupt * c->r_zupt;
    const float Hx[4] = { 0.0f, 0.0f, 1.0f, 0.0f };
    const float Hy[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

    /* gate_k = 0.0f -- ZUPT is never gated (design: "if !moving, apply
     * vx = 0 and vy = 0 as two scalar updates" unconditionally). Reuses
     * ekf_scalar_update(), the exact same path pos_ekf_update_ranges() uses. */
    (void)ekf_scalar_update(f, Hx, 0.0f - f->x[2], R, 0.0f);
    (void)ekf_scalar_update(f, Hy, 0.0f - f->x[3], R, 0.0f);

    ekf_clamp_speed(f, c->v_max);
}

bool pos_ekf_needs_reseed(const struct pos_ekf *f, const struct pos_ekf_cfg *c)
{
    if (c->reset_after == 0) {
        /* 0 means "never auto-reseed", not "reseed immediately". A caller
         * that zeroed or memset the config did not ask for hair-trigger
         * reseeding, and gate_streak >= 0 is trivially true for every
         * freshly seeded filter -- treating 0 literally would loop. */
        return false;
    }
    return f->gate_streak >= c->reset_after;
}

bool pos_ekf_get(const struct pos_ekf *f, float *x, float *y,
                 float *vx, float *vy)
{
    if (!f->init) {
        return false;
    }

    if (x)  { *x  = f->x[0]; }
    if (y)  { *y  = f->x[1]; }
    if (vx) { *vx = f->x[2]; }
    if (vy) { *vy = f->x[3]; }
    return true;
}

float pos_ekf_pos_sigma(const struct pos_ekf *f)
{
    if (!f->init) {
        return 0.0f;
    }

    float var = f->P[0 * 4 + 0] + f->P[1 * 4 + 1];
    if (var < 0.0f) {
        var = 0.0f; /* guard float noise; variance cannot be negative */
    }
    return sqrtf(var);
}

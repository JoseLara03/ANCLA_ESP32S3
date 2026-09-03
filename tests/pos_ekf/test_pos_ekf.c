#include "pos_ekf.h"
#include "pos_solver.h"
#include "tdoa_solve.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Anchors at the corners of an 8x8 m room, ceiling-mounted 1.6 m above the
 * tag -- the same dz the design doc's worked example uses. Non-collinear,
 * so trilateration is well-posed for every test below. */
static const float ANCHOR_XY[4][2] = {
    { 0.0f, 0.0f }, { 8.0f, 0.0f }, { 0.0f, 8.0f }, { 8.0f, 8.0f },
};
#define ANCHOR_DZ  1.6f

/* Mirrors POS_EKF_DT_MAX, which is file-local to pos_ekf.c. If that
 * constant moves, test_dt_guards() fails loudly rather than silently
 * stopping testing the clamp. */
#define POS_EKF_DT_MAX_EXPECTED  120.0f

/* Build exact (noise-free) ranges from the 3D slant model for a true (x, y).
 * Exact ranges let each test assert a tight numeric bound instead of a fuzzy
 * one, while still exercising the real dz-carrying measurement model. */
static void make_ranges(float x, float y, struct pos_meas *out, size_t n)
{
    for (size_t i = 0; i < n && i < 4; i++) {
        float dx = x - ANCHOR_XY[i][0];
        float dy = y - ANCHOR_XY[i][1];

        out[i].x       = ANCHOR_XY[i][0];
        out[i].y       = ANCHOR_XY[i][1];
        out[i].dz      = ANCHOR_DZ;
        out[i].range_m = sqrtf(dx * dx + dy * dy + ANCHOR_DZ * ANCHOR_DZ);
    }
}

static int all_finite_state(const struct pos_ekf *f)
{
    for (int i = 0; i < 4; i++) {
        if (!isfinite(f->x[i])) return 0;
    }
    for (int i = 0; i < 16; i++) {
        if (!isfinite(f->P[i])) return 0;
    }
    return 1;
}

/* -------------------------------------------------------------------- */

static void test_converges_static_from_poor_seed(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);

    /* True position (4.0, 3.0); poor seed ~3.6 m away. */
    pos_ekf_seed(&f, 1.0f, 1.0f);

    for (int i = 0; i < 60; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x, y;
    CHECK(pos_ekf_get(&f, &x, &y, NULL, NULL));
    CHECK(fabsf(x - 4.0f) < 0.05f);
    CHECK(fabsf(y - 3.0f) < 0.05f);
    CHECK(all_finite_state(&f));
}

static void test_tracks_constant_velocity(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    const float dt = 0.2f;
    const float vx_true = 0.5f;
    const float vy_true = 0.3f;
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    /* 100 steps @ 0.2 s = 20 s of straight-line walking. */
    for (int i = 0; i < 100; i++) {
        tx += vx_true * dt;
        ty += vy_true * dt;

        pos_ekf_predict(&f, &c, dt, true);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x, y, vx, vy;
    CHECK(pos_ekf_get(&f, &x, &y, &vx, &vy));
    CHECK(fabsf(x - tx) < 0.15f);
    CHECK(fabsf(y - ty) < 0.15f);
    CHECK(fabsf(vx - vx_true) < 0.1f);
    CHECK(fabsf(vy - vy_true) < 0.1f);
    CHECK(all_finite_state(&f));
}

static void test_outlier_gated(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 3.9f, 3.1f); /* close seed */

    /* Converge P tight with 30 good, exact fixes at the true point (4, 3). */
    for (int i = 0; i < 30; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x_before, y_before;
    pos_ekf_get(&f, &x_before, &y_before, NULL, NULL);

    /* One fix with anchor 0's range inflated by exactly 1 m. */
    pos_ekf_predict(&f, &c, 0.2f, false);
    make_ranges(4.0f, 3.0f, m, 4);
    m[0].range_m += 1.0f;
    int accepted = pos_ekf_update_ranges(&f, &c, m, 4);

    CHECK(accepted == 3); /* anchor 0 gated, three others accepted */

    float x_after, y_after;
    pos_ekf_get(&f, &x_after, &y_after, NULL, NULL);

    /* The estimate barely moves -- well under the size of the outlier. */
    CHECK(fabsf(x_after - x_before) < 0.05f);
    CHECK(fabsf(y_after - y_before) < 0.05f);
    CHECK(fabsf(x_after - 4.0f) < 0.05f);
    CHECK(fabsf(y_after - 3.0f) < 0.05f);
}

/* Shared walk-then-stop scenario for isolating ZUPT's own contribution.
 * `apply_zupt` selects whether pos_ekf_zupt() is called during the stopped
 * phase; everything else (seed, walk, dt, stopped-phase ranges) is
 * identical between the two runs so any velocity difference at the end is
 * attributable to ZUPT alone, not to the range updates converging on their
 * own. `speed_before_out` (walking-phase exit speed) may be NULL. */
static void run_walk_then_stop(bool apply_zupt, float *speed_before_out,
                                float *speed_after_out)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    /* Walk for a bit so velocity is genuinely nonzero. */
    for (int i = 0; i < 40; i++) {
        tx += 0.4f * 0.2f;
        ty += 0.2f * 0.2f;
        pos_ekf_predict(&f, &c, 0.2f, true);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    if (speed_before_out) {
        float vx, vy;
        pos_ekf_get(&f, NULL, NULL, &vx, &vy);
        *speed_before_out = sqrtf(vx * vx + vy * vy);
    }

    /* Now the tag stops: static ranges every fix, still-mode process noise,
     * and (only in the apply_zupt run) the ZUPT pseudo-measurement. */
    for (int i = 0; i < 40; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        if (apply_zupt) {
            pos_ekf_zupt(&f, &c);
        }
    }

    CHECK(all_finite_state(&f));

    float vx, vy;
    pos_ekf_get(&f, NULL, NULL, &vx, &vy);
    *speed_after_out = sqrtf(vx * vx + vy * vy);
}

static void test_zupt_zeroes_velocity(void)
{
    float speed_before, speed_with_zupt, speed_without_zupt;

    run_walk_then_stop(true, &speed_before, &speed_with_zupt);
    CHECK(speed_before > 0.1f); /* sanity: motion actually built up velocity */
    CHECK(speed_with_zupt < 0.02f);

    run_walk_then_stop(false, NULL, &speed_without_zupt);

    /* Isolate ZUPT's own contribution: with byte-for-byte identical inputs
     * otherwise, applying the ZUPT pseudo-measurement must drive residual
     * velocity down well past what the range updates alone achieve. If
     * pos_ekf_zupt()'s body were deleted, the two runs would be numerically
     * identical and this comparison would fail. */
    CHECK(speed_with_zupt < 0.5f * speed_without_zupt);
}

static void test_speed_clamp_engages(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    c.v_max = 1.0f;
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* Force an outrageous velocity directly into the state -- far past
     * v_max -- then run one range update matching the current position
     * exactly (so the correction itself barely touches x/y and, since P is
     * still diagonal from the seed, leaves vx/vy completely alone) purely
     * to exercise ekf_clamp_speed() at the end of the batch. */
    f.x[2] = 10.0f;
    f.x[3] = 0.0f;

    make_ranges(4.0f, 3.0f, m, 4);
    pos_ekf_update_ranges(&f, &c, m, 4);

    float vx, vy;
    pos_ekf_get(&f, NULL, NULL, &vx, &vy);
    float speed = sqrtf(vx * vx + vy * vy);

    /* Clamped to exactly v_max (10.0 * (v_max/10.0)), not left at 10 and
     * not zeroed -- direction (pure +x) is preserved too. */
    CHECK(fabsf(speed - c.v_max) < 1e-3f);
    CHECK(fabsf(vx - c.v_max) < 1e-3f);
    CHECK(fabsf(vy) < 1e-4f);
}

static void test_stationary_variance_falls(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.3f, 2.6f); /* seeded near, not at, the true point */

    float sigma_first = pos_ekf_pos_sigma(&f);

    float sigma_at_5 = 0.0f, sigma_at_50 = 0.0f;
    for (int i = 1; i <= 50; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        pos_ekf_zupt(&f, &c);

        if (i == 5)  { sigma_at_5  = pos_ekf_pos_sigma(&f); }
        if (i == 50) { sigma_at_50 = pos_ekf_pos_sigma(&f); }
    }

    /* The averaging effect: sigma shrinks from the seed, and keeps shrinking
     * (or holds at its still-mode floor) as more stationary fixes arrive. */
    CHECK(sigma_at_5 < sigma_first);
    CHECK(sigma_at_50 < sigma_at_5);
    CHECK(sigma_at_50 < 0.05f);
}

static void test_pos_sigma_uses_both_diag_terms(void)
{
    struct pos_ekf f;

    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 0.0f, 0.0f);

    /* Asymmetric position variance -- P[0][0] != P[1][1] -- so an
     * implementation that only reads P[0][0] (e.g. sqrtf(P[0][0]) alone) is
     * numerically distinguishable from the documented
     * sqrt(P[0,0] + P[1,1]). */
    f.P[0 * 4 + 0] = 4.0f;
    f.P[1 * 4 + 1] = 9.0f;

    float sigma = pos_ekf_pos_sigma(&f);
    CHECK(fabsf(sigma - sqrtf(13.0f)) < 1e-4f);
    CHECK(fabsf(sigma - 2.0f) > 1e-3f);  /* what sqrtf(P[0][0]) alone gives */
    CHECK(fabsf(sigma - 3.0f) > 1e-3f);  /* what sqrtf(P[1][1]) alone gives */
}

static void test_gate_streak_and_reseed(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* A couple of good fixes first, to look like a running filter. */
    for (int i = 0; i < 5; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }
    CHECK(!pos_ekf_needs_reseed(&f, &c));

    /* Now feed garbage: ranges consistent with a point 50 m away, on every
     * anchor, for reset_after fixes running. */
    for (uint8_t i = 0; i < c.reset_after; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);
        int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(accepted == 0);
    }

    CHECK(pos_ekf_needs_reseed(&f, &c));

    /* One good fix clears the streak. */
    pos_ekf_predict(&f, &c, 0.2f, false);
    make_ranges(4.0f, 3.0f, m, 4);
    int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
    CHECK(accepted > 0);
    CHECK(!pos_ekf_needs_reseed(&f, &c));
}

static void test_gate_streak_saturates(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* Drive the counter to just below the wrap point directly rather than by
     * running 255 all-gated fixes. That longer route does not actually work,
     * and the reason is worth recording: with nothing ever accepted, Q keeps
     * inflating P every predict, so S = HPH^T + R grows without bound and the
     * 3-sigma gate eventually reopens and admits a range. That is CORRECT --
     * it is how the filter recovers from divergence when no caller reseeds it
     * -- so a test must not depend on the gate staying shut indefinitely.
     * What is under test here is only the counter's saturation. */
    f.gate_streak = UINT8_MAX - 5u;

    for (int i = 0; i < 20; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);   /* ~70 m away: gated */
        (void)pos_ekf_update_ranges(&f, &c, m, 4);
    }

    /* Saturated, not wrapped. Without the < UINT8_MAX guard the counter rolls
     * through 0 and back up through every small value, so needs_reseed()
     * would falsely clear for reset_after fixes after each wrap -- the tag
     * would stop asking for the reseed that is its only way out. */
    CHECK(f.gate_streak == UINT8_MAX);
    CHECK(pos_ekf_needs_reseed(&f, &c));

    /* And one accepted range still clears it. */
    make_ranges(4.0f, 3.0f, m, 4);
    CHECK(pos_ekf_update_ranges(&f, &c, m, 4) > 0);
    CHECK(f.gate_streak == 0u);
    CHECK(!pos_ekf_needs_reseed(&f, &c));
}

static void test_reset_after_zero_never_reseeds(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    c.reset_after = 0;
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* A freshly seeded, perfectly healthy filter has gate_streak == 0. With
     * reset_after == 0 that must NOT be read as "streak has reached the
     * threshold" -- 0 means "never auto-reseed" (the safe reading for a
     * caller that zeroed/memset the config), not "reseed immediately". */
    CHECK(!pos_ekf_needs_reseed(&f, &c));

    /* Even a long all-gated run must never report needing a reseed. */
    for (int i = 0; i < 10; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);
        int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(accepted == 0);
        CHECK(!pos_ekf_needs_reseed(&f, &c));
    }
}

static void test_q_position_and_cross_terms(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    const float dt = 0.5f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 0.0f, 0.0f);

    /* Start from a clean diagonal P so the whole of the resulting
     * position-velocity covariance comes from Q, not from the seed. */
    memset(f.P, 0, sizeof(f.P));

    pos_ekf_predict(&f, &c, dt, true);

    const float q  = c.sigma_a_move * c.sigma_a_move;
    const float pp = q * dt * dt * dt * dt / 4.0f;   /* dt^4/4 */
    const float pv = q * dt * dt * dt / 2.0f;        /* dt^3/2 */
    const float vv = q * dt * dt;                    /* dt^2   */

    /* Pins the dt^4/4 scale: a mutation to dt^4 quadruples this. */
    CHECK(fabsf(f.P[0 * 4 + 0] - pp) < 1e-6f);
    CHECK(fabsf(f.P[1 * 4 + 1] - pp) < 1e-6f);

    /* Pins the dt^3/2 cross terms, which are what make position and velocity
     * co-vary. Deleting them leaves the diagonal correct and the filter
     * quietly worse, so assert them directly rather than via behaviour. */
    CHECK(f.P[0 * 4 + 2] > 0.0f);
    CHECK(f.P[1 * 4 + 3] > 0.0f);
    CHECK(fabsf(f.P[0 * 4 + 2] - pv) < 1e-6f);
    CHECK(fabsf(f.P[2 * 4 + 0] - pv) < 1e-6f);
    CHECK(fabsf(f.P[1 * 4 + 3] - pv) < 1e-6f);
    CHECK(fabsf(f.P[3 * 4 + 1] - pv) < 1e-6f);

    CHECK(fabsf(f.P[2 * 4 + 2] - vv) < 1e-6f);
    CHECK(fabsf(f.P[3 * 4 + 3] - vv) < 1e-6f);

    /* No cross-axis coupling: x must not co-vary with vy. */
    CHECK(fabsf(f.P[0 * 4 + 3]) < 1e-9f);
    CHECK(fabsf(f.P[1 * 4 + 2]) < 1e-9f);
}

static void test_dt_guards(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f, ref;

    pos_ekf_cfg_defaults(&c);

    /* Non-positive dt must be a no-op, not a negative-time propagation.
     * Reachable in practice from a wrapped or equal clock read. */
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 2.0f, 5.0f);
    f.x[2] = 0.7f;
    f.x[3] = -0.4f;
    ref = f;

    pos_ekf_predict(&f, &c, 0.0f, true);
    CHECK(memcmp(&f, &ref, sizeof(f)) == 0);

    pos_ekf_predict(&f, &c, -3.0f, true);
    CHECK(memcmp(&f, &ref, sizeof(f)) == 0);

    /* An absurd dt must be clamped, not propagated. Without the clamp the
     * dt^4/4 term grows as the fourth power, so 10x the limit is 10000x the
     * position variance -- assert against what the clamp permits. */
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 2.0f, 5.0f);
    memset(f.P, 0, sizeof(f.P));
    pos_ekf_predict(&f, &c, 1200.0f, true);

    const float q      = c.sigma_a_move * c.sigma_a_move;
    const float capped = q * POS_EKF_DT_MAX_EXPECTED * POS_EKF_DT_MAX_EXPECTED
                           * POS_EKF_DT_MAX_EXPECTED * POS_EKF_DT_MAX_EXPECTED
                           / 4.0f;

    CHECK(fabsf(f.P[0] - capped) < capped * 1e-4f);
    CHECK(all_finite_state(&f));
}

static void test_dt_variation_stable(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 3.5f, 3.5f);

    /* Alternate a fast-tier dt (0.2 s) with a deep-skip dt (5 s) at a fixed
     * true point; the filter must not destabilise either way. */
    for (int i = 0; i < 40; i++) {
        float dt = (i % 2 == 0) ? 0.2f : 5.0f;
        pos_ekf_predict(&f, &c, dt, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(all_finite_state(&f));
    }

    float x, y;
    pos_ekf_get(&f, &x, &y, NULL, NULL);
    CHECK(fabsf(x - 4.0f) < 0.2f);
    CHECK(fabsf(y - 3.0f) < 0.2f);
}

static void test_covariance_symmetric_positive_diag(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    for (int i = 0; i < 500; i++) {
        bool moving = (i % 7) < 4;
        float dt = 0.2f + 0.1f * (float)(i % 5);

        if (moving) {
            tx += 0.3f * dt;
            ty += 0.15f * dt;
        }

        pos_ekf_predict(&f, &c, dt, moving);
        make_ranges(tx, ty, m, 4);

        /* Every 11th fix, corrupt one anchor to exercise the gate path too. */
        if ((i % 11) == 0) {
            m[i % 4].range_m += 2.0f;
        }
        pos_ekf_update_ranges(&f, &c, m, 4);

        if (!moving) {
            pos_ekf_zupt(&f, &c);
        }

        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                /* Tight on purpose: 1e-3 was looser than the asymmetry
                 * 500 sequential scalar updates accumulate in float, so
                 * it passed with ekf_symmetrize() deleted. P must be
                 * symmetric to rounding, not merely nearly so. */
                float diff = fabsf(f.P[r * 4 + col] - f.P[col * 4 + r]);
                CHECK(diff <= 1e-7f * (1.0f + fabsf(f.P[r * 4 + col])));
            }
            CHECK(f.P[r * 4 + r] > 0.0f);
        }
        CHECK(all_finite_state(&f));
    }
}

static void test_get_before_seed_and_null_pointers(void)
{
    struct pos_ekf f;
    pos_ekf_reset(&f);

    float x = -999.0f, y = -999.0f, vx = -999.0f, vy = -999.0f;
    bool ok = pos_ekf_get(&f, &x, &y, &vx, &vy);

    CHECK(!ok);
    /* Out-pointers must be left untouched before the first seed. */
    CHECK(x == -999.0f);
    CHECK(y == -999.0f);
    CHECK(vx == -999.0f);
    CHECK(vy == -999.0f);

    /* NULL out-pointers must be accepted, seeded or not. */
    CHECK(pos_ekf_get(&f, NULL, NULL, NULL, NULL) == false);

    pos_ekf_seed(&f, 2.0f, 2.0f);
    CHECK(pos_ekf_get(&f, NULL, NULL, NULL, NULL) == true);
    CHECK(pos_ekf_get(&f, &x, NULL, NULL, NULL) == true);
    CHECK(fabsf(x - 2.0f) < 1e-6f);
}

static void test_no_nan_inf_long_run(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 0.5f, ty = 7.5f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    for (int i = 0; i < 1000; i++) {
        bool moving = ((i / 20) % 2) == 0;
        float dt = 0.1f + 0.05f * (float)(i % 100); /* sweeps 0.1..5.1 s */

        if (moving) {
            tx += 0.25f * dt;
            ty -= 0.10f * dt;
            if (tx > 7.5f) tx = 0.5f;
            if (ty < 0.5f) ty = 7.5f;
        }

        pos_ekf_predict(&f, &c, dt, moving);

        size_t n = (size_t)3 + (size_t)(i % 2); /* alternate n=3/n=4 */
        make_ranges(tx, ty, m, 4);
        if ((i % 13) == 0) {
            m[0].range_m += 3.0f; /* occasional gross outlier */
        }
        pos_ekf_update_ranges(&f, &c, m, n);

        if (!moving && (i % 3) == 0) {
            pos_ekf_zupt(&f, &c);
        }

        if (pos_ekf_needs_reseed(&f, &c)) {
            /* Exercise the caller-driven reseed path itself. */
            pos_ekf_seed(&f, tx, ty);
        }

        CHECK(all_finite_state(&f));
    }
}

/* ---- Task 3: pos_ekf_update_tdoa() -------------------------------------- */

/* Same 10x10 m square used elsewhere in this file, ceiling-mounted 1.5 m
 * above the tag. Anchor 0 is the TDoA reference in every test below. */
static const float TDOA_AX[4] = { 0.0f, 10.0f, 10.0f,  0.0f };
static const float TDOA_AY[4] = { 0.0f,  0.0f, 10.0f, 10.0f };
#define TDOA_DZ  1.5f
#define TDOA_T0  ((int64_t)0x1000000000LL)

/* Exact (noise-free) TDoA observations for true point (x, y), n anchors. */
static void make_tdoa(float x, float y, size_t n, struct tdoa_meas *out)
{
    for (size_t a = 0; a < n && a < 4; a++) {
        float dx = x - TDOA_AX[a], dy = y - TDOA_AY[a];
        float r  = sqrtf(dx * dx + dy * dy + TDOA_DZ * TDOA_DZ);

        out[a].x     = TDOA_AX[a];
        out[a].y     = TDOA_AY[a];
        out[a].dz    = TDOA_DZ;
        /* 0 = "this anchor reported no sigma", so the filter falls back to
         * cfg->r_tdoa. Set EXPLICITLY: callers build struct tdoa_meas on the
         * stack, and leaving this field uninitialised feeds
         * pos_ekf_update_tdoa() garbage as a measurement variance. That is
         * not hypothetical -- it is what this helper did until 2026-09-03,
         * and it made test_tdoa_degenerate_geometry_no_nan() pass or fail on
         * whatever happened to be on the stack. */
        out[a].sigma_m = 0.0f;
        out[a].t_dtu = TDOA_T0 + (int64_t)llroundf(r / TDOA_M_PER_DTU);
    }
}

/* TRAP 1: (m[k].t_dtu - m[0].t_dtu) must be computed in int64_t before any
 * float conversion. t_dtu sits at TDOA_T0 = 2^36 here, where a float32's
 * 24-bit mantissa (ULP = 2^13 = 8192 DTU at this magnitude, half-ULP
 * ~19.2 m of path difference) can no longer resolve an ~11 m
 * range-difference: converting each raw timestamp to float FIRST and
 * subtracting after rounds both to the SAME representable float, silently
 * zeroing a real ~11 m difference. That corrupted zero is itself the
 * discriminator: fed as the measurement, it disagrees with the model's own
 * ~11 m prediction by more than even a freshly-seeded (loose, 1.5 m sigma)
 * filter's 3-sigma gate tolerates, so a float-first build REJECTS this
 * update while a correct int64-first build ACCEPTS it -- this assertion is
 * what actually fails if someone "simplifies" the subtraction order. A
 * smaller geometry (as in make_tdoa()'s 10x10 m square) does not
 * discriminate: its few-metre differences stay inside the gate whichever
 * way the subtraction is done, which is why this test uses its own
 * deliberately larger, asymmetric baseline instead. */
static void test_trap1_int64_subtraction_order_matters(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct tdoa_meas m[2];
    const float ax0 = 0.0f, ay0 = 0.0f, dz0 = TDOA_DZ;
    const float ax1 = 20.0f, ay1 = 0.0f, dz1 = TDOA_DZ;
    const float tx = 4.0f, ty = 3.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    float r0 = sqrtf((tx - ax0) * (tx - ax0) + (ty - ay0) * (ty - ay0) +
                     dz0 * dz0);
    float r1 = sqrtf((tx - ax1) * (tx - ax1) + (ty - ay1) * (ty - ay1) +
                     dz1 * dz1);

    m[0] = (struct tdoa_meas){ .x = ax0, .y = ay0, .dz = dz0,
                                .t_dtu = TDOA_T0 };
    m[1] = (struct tdoa_meas){ .x = ax1, .y = ay1, .dz = dz1,
                                .t_dtu = TDOA_T0 +
                                         (int64_t)llroundf((r1 - r0) /
                                                            TDOA_M_PER_DTU) };

    int accepted = pos_ekf_update_tdoa(&f, &c, m, 2);
    CHECK(accepted == 1);
}

/* Deterministic PRNG, same shape as tests/sync_model's: reproducibility
 * matters more than statistical quality for a host test. */
static uint32_t g_rng_state = 0xA5A5A5A5u;

static int32_t rng_noise_dtu(int32_t amplitude)
{
    if (amplitude == 0) return 0;
    g_rng_state = g_rng_state * 1664525u + 1013904223u;
    return (int32_t)((g_rng_state >> 8) % (uint32_t)(2 * amplitude + 1)) -
           amplitude;
}

/* The point of Task 3, not a side effect: on the SAME noisy data, the EKF's
 * tracking RMS must beat tdoa_solve()'s per-fix RMS, because the filter has
 * memory (a process model that matches this constant-velocity truth exactly)
 * and the raw per-group solve does not. ~136 DTU (~0.3 m) of independent
 * per-anchor timestamp noise gives a range-difference noise of roughly
 * 0.3 * sqrt(2) =~ 0.42 m, the same order as r_tdoa's 0.6 m default (derived
 * in pos_ekf.h from the Fase 2 hardware jitter via the same sqrt(2)). */
static void test_tdoa_filter_beats_raw_solve(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    const float dt = 0.2f;
    const float vx_true = 0.3f, vy_true = 0.2f;
    const int   n_steps = 100;
    const int32_t noise_amp_dtu = 136;
    float tx = 1.0f, ty = 1.0f;
    bool seeded = false;
    double sq_ekf = 0.0, sq_solve = 0.0;
    int n_scored = 0;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);

    for (int i = 0; i < n_steps; i++) {
        struct tdoa_meas m[4];

        tx += vx_true * dt;
        ty += vy_true * dt;

        make_tdoa(tx, ty, 4, m);
        for (int a = 0; a < 4; a++) {
            m[a].t_dtu += (int64_t)rng_noise_dtu(noise_amp_dtu);
        }

        if (!seeded) {
            pos_ekf_seed(&f, tx, ty); /* first fix seeds, like tdoa_gw.c */
            seeded = true;
        } else {
            pos_ekf_predict(&f, &c, dt, true);
            pos_ekf_update_tdoa(&f, &c, m, 4);
        }

        struct pos_result r;
        bool solved = tdoa_solve(m, 4, NULL, &r);

        /* Score once both estimates exist and the filter has had a chance
         * to converge past its 1.5 m seed prior. */
        if (i >= 20 && solved && r.valid) {
            float ex, ey;

            CHECK(pos_ekf_get(&f, &ex, &ey, NULL, NULL));

            float de = ex - tx, df = ey - ty;
            float ds = r.x - tx, dr = r.y - ty;

            sq_ekf   += (double)(de * de + df * df);
            sq_solve += (double)(ds * ds + dr * dr);
            n_scored++;
        }
    }

    CHECK(n_scored > 50);

    double rms_ekf   = sqrt(sq_ekf / n_scored);
    double rms_solve = sqrt(sq_solve / n_scored);

    printf("  tdoa filter vs raw solve: RMS ekf=%.4f m, solve=%.4f m"
           " (n=%d)\n", rms_ekf, rms_solve, n_scored);

    CHECK(rms_ekf < rms_solve);
}

/* A stationary tag fed exact, noise-free TDoA observations must neither
 * drift in position nor accumulate spurious velocity. */
static void test_tdoa_stationary_does_not_drift(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    const float tx = 5.0f, ty = 5.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    for (int i = 0; i < 60; i++) {
        struct tdoa_meas m[4];

        make_tdoa(tx, ty, 4, m);
        pos_ekf_predict(&f, &c, 0.2f, false);
        pos_ekf_update_tdoa(&f, &c, m, 4);
    }

    float x, y, vx, vy;

    CHECK(pos_ekf_get(&f, &x, &y, &vx, &vy));
    CHECK(fabsf(x - tx) < 0.05f);
    CHECK(fabsf(y - ty) < 0.05f);
    CHECK(fabsf(vx) < 0.05f);
    CHECK(fabsf(vy) < 0.05f);
}

/* A gross outlier on one anchor's timestamp must be gated, leaving the
 * estimate essentially untouched -- mirrors test_outlier_gated() for the
 * range path. Only the (anchor 1 vs reference anchor 0) equation is
 * affected, so exactly 2 of the 3 range-difference equations accept. */
static void test_tdoa_gate_rejects_outlier(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    const float tx = 5.0f, ty = 5.0f;
    struct tdoa_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    /* Converge P tight first with 30 good, exact fixes. */
    for (int i = 0; i < 30; i++) {
        make_tdoa(tx, ty, 4, m);
        pos_ekf_predict(&f, &c, 0.2f, false);
        pos_ekf_update_tdoa(&f, &c, m, 4);
    }

    float x_before, y_before;

    CHECK(pos_ekf_get(&f, &x_before, &y_before, NULL, NULL));

    /* One fix with anchor 1's timestamp corrupted by ~50 m of path
     * difference: a gross outlier against the default r_tdoa = 0.6 m. */
    make_tdoa(tx, ty, 4, m);
    m[1].t_dtu += (int64_t)llroundf(50.0f / TDOA_M_PER_DTU);

    pos_ekf_predict(&f, &c, 0.2f, false);
    int accepted = pos_ekf_update_tdoa(&f, &c, m, 4);

    CHECK(accepted == 2); /* anchor 1's equation gated; the other two accept */

    float x_after, y_after;

    CHECK(pos_ekf_get(&f, &x_after, &y_after, NULL, NULL));
    CHECK(fabsf(x_after - x_before) < 0.1f);
    CHECK(fabsf(y_after - y_before) < 0.1f);
}

/* Seeding exactly on the reference anchor makes r_0 == 0, undefined for
 * every range-difference Jacobian against it -- must be refused cleanly
 * (accepted == 0) rather than producing a NaN/Inf state. */
static void test_tdoa_degenerate_geometry_no_nan(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct tdoa_meas m[3];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, TDOA_AX[0], TDOA_AY[0]); /* exactly on anchor 0 */

    make_tdoa(4.0f, 3.0f, 3, m); /* some other true point's geometry */

    int accepted = pos_ekf_update_tdoa(&f, &c, m, 3);
    CHECK(accepted == 0);
    CHECK(all_finite_state(&f));
}

/*
 * ZUPT on the TDoA path: a stationary tag whose accelerometer says so.
 *
 * The failure this guards against is a constant-velocity model integrating
 * measurement noise into a drift that does not exist -- which is exactly what
 * a motionless tag looks like on a live gateway, and is the ~0.8 m dispersion
 * the 2026-09-02 hardware round left open. pos_ekf.h calls a zero-velocity
 * update the single largest visual improvement available for that reason.
 */
static void test_tdoa_zupt_pins_velocity(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f_zupt, f_free;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f_zupt);
    pos_ekf_reset(&f_free);
    pos_ekf_seed(&f_zupt, 5.0f, 5.0f);
    pos_ekf_seed(&f_free, 5.0f, 5.0f);

    /* The same noisy observations of a tag that never moves, fed to two
     * filters that differ ONLY in whether ZUPT is applied. */
    for (unsigned int k = 0; k < 60u; k++) {
        struct tdoa_meas m[4];

        make_tdoa(5.0f, 5.0f, 4, m);
        for (int a = 0; a < 4; a++) {
            m[a].t_dtu += (int64_t)rng_noise_dtu(64);
        }

        pos_ekf_predict(&f_zupt, &c, 0.2f, false);
        pos_ekf_update_tdoa(&f_zupt, &c, m, 4);
        pos_ekf_zupt(&f_zupt, &c);

        pos_ekf_predict(&f_free, &c, 0.2f, true);
        pos_ekf_update_tdoa(&f_free, &c, m, 4);
    }

    float zx, zy, zvx, zvy, fx, fy, fvx, fvy;

    CHECK(pos_ekf_get(&f_zupt, &zx, &zy, &zvx, &zvy));
    CHECK(pos_ekf_get(&f_free, &fx, &fy, &fvx, &fvy));

    float zs = sqrtf(zvx * zvx + zvy * zvy);
    float fs = sqrtf(fvx * fvx + fvy * fvy);

    printf("  zupt: |v| %.4f m/s (with) vs %.4f m/s (without); "
           "pos err %.3f vs %.3f m\n",
           (double)zs, (double)fs,
           (double)sqrtf((zx - 5.0f) * (zx - 5.0f) +
                         (zy - 5.0f) * (zy - 5.0f)),
           (double)sqrtf((fx - 5.0f) * (fx - 5.0f) +
                         (fy - 5.0f) * (fy - 5.0f)));

    /* The point of the update: velocity is pinned near zero. */
    CHECK(zs < 0.05f);
    /* And it is doing something -- an unconstrained filter on the same data
     * carries real velocity. A test that only checked zs could pass on a
     * filter that never moves for unrelated reasons. */
    CHECK(zs < fs);
}

/*
 * Per-anchor weighting (proto 5): an anchor that reports a LARGE sigma must
 * move the state less than one reporting a small sigma, given the same
 * innovation.
 *
 * Run twice over identical geometry and identical corrupted timestamps,
 * changing ONLY the sigmas. Any difference in how far the state moves is
 * attributable to the weighting and to nothing else.
 */
static void test_tdoa_per_anchor_weighting(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f_trusted, f_doubted;
    struct tdoa_meas m[4];
    unsigned int k;

    pos_ekf_cfg_defaults(&c);
    c.gate_k = 0.0f;   /* gating off: this test is about the GAIN, not the gate */

    pos_ekf_reset(&f_trusted);
    pos_ekf_reset(&f_doubted);
    pos_ekf_seed(&f_trusted, 5.0f, 5.0f);
    pos_ekf_seed(&f_doubted, 5.0f, 5.0f);

    /* A real geometry, then one anchor's timestamp pushed well off. */
    make_tdoa(6.5f, 4.0f, 4, m);
    m[1].t_dtu += (int64_t)llroundf(1.0f / TDOA_M_PER_DTU);

    /* Run A: every anchor claims a tight 0.05 m sigma. */
    for (k = 0; k < 4u; k++) {
        m[k].sigma_m = 0.05f;
    }
    pos_ekf_update_tdoa(&f_trusted, &c, m, 4);

    /* Run B: identical data, every anchor claims a loose 5 m sigma. */
    for (k = 0; k < 4u; k++) {
        m[k].sigma_m = 5.0f;
    }
    pos_ekf_update_tdoa(&f_doubted, &c, m, 4);

    float tx, ty, dx, dy;

    CHECK(pos_ekf_get(&f_trusted, &tx, &ty, NULL, NULL));
    CHECK(pos_ekf_get(&f_doubted, &dx, &dy, NULL, NULL));

    float moved_trusted = sqrtf((tx - 5.0f) * (tx - 5.0f) +
                                (ty - 5.0f) * (ty - 5.0f));
    float moved_doubted = sqrtf((dx - 5.0f) * (dx - 5.0f) +
                                (dy - 5.0f) * (dy - 5.0f));

    printf("  weighting: tight sigma moved %.3f m, loose sigma moved %.3f m\n",
           (double)moved_trusted, (double)moved_doubted);
    CHECK(moved_trusted > moved_doubted);
    CHECK(all_finite_state(&f_trusted));
    CHECK(all_finite_state(&f_doubted));
}

/*
 * sigma_m <= 0, non-finite, or absent must fall back to cfg->r_tdoa rather
 * than be taken literally -- 0 would read as infinite confidence and NaN
 * would poison R. The fallback is PER ANCHOR, so a mixed-firmware deployment
 * still gets real weighting from the anchors that do report one.
 */
static void test_tdoa_missing_sigma_falls_back(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f_zero, f_flat;
    struct tdoa_meas m[4];
    unsigned int k;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f_zero);
    pos_ekf_reset(&f_flat);
    pos_ekf_seed(&f_zero, 5.0f, 5.0f);
    pos_ekf_seed(&f_flat, 5.0f, 5.0f);

    make_tdoa(6.5f, 4.0f, 4, m);

    /* Nobody reports a sigma: must behave exactly like the flat r_tdoa. */
    for (k = 0; k < 4u; k++) {
        m[k].sigma_m = 0.0f;
    }
    pos_ekf_update_tdoa(&f_zero, &c, m, 4);

    /* The same thing said explicitly. sqrtf(2)/2 * r_tdoa per anchor gives
     * R = r_tdoa^2 across the pair, which is the flat case by construction. */
    for (k = 0; k < 4u; k++) {
        m[k].sigma_m = c.r_tdoa * 0.70710678f;
    }
    pos_ekf_update_tdoa(&f_flat, &c, m, 4);

    float zx, zy, fx, fy;

    CHECK(pos_ekf_get(&f_zero, &zx, &zy, NULL, NULL));
    CHECK(pos_ekf_get(&f_flat, &fx, &fy, NULL, NULL));
    /* Not asserted equal: the fallback uses r_tdoa for BOTH anchors, giving
     * R = 2 * r_tdoa^2, so the two differ by a known factor. What must hold
     * is that neither blew up and both stayed sane. */
    CHECK(all_finite_state(&f_zero));
    CHECK(all_finite_state(&f_flat));
    CHECK(fabsf(zx - 5.0f) < 5.0f && fabsf(zy - 5.0f) < 5.0f);

    /* A NaN sigma must not reach R. Uninitialised stack is how this arrives. */
    for (k = 0; k < 4u; k++) {
        m[k].sigma_m = 0.0f / 0.0f;
    }
    pos_ekf_reset(&f_zero);
    pos_ekf_seed(&f_zero, 5.0f, 5.0f);
    pos_ekf_update_tdoa(&f_zero, &c, m, 4);
    CHECK(all_finite_state(&f_zero));
}


/*
 * ---- Task 7: an ATTEMPT at reproducing the runaway that does NOT ------
 *
 * READ THIS BEFORE TRUSTING IT. This test was written to fail against HEAD
 * and reproduce the hardware runaway. IT PASSES against HEAD, so it does not
 * reproduce it. It is kept because what it DOES pin is a real property --
 * the filter stays bounded on the deployed thin geometry under unbiased
 * noise -- and because the attempt narrows the mechanism, which is worth
 * more in the file than in a session log.
 *
 * What it establishes by failing to fail: unbiased measurement noise on this
 * geometry does NOT make the filter escape. Worst excursion measured 0.54 m
 * with +/-64 DTU of noise over 300 cycles, needs_reseed never asked, no
 * all-gated cycle. So the hardware runaway needs an ingredient this
 * scenario lacks -- and the two measured candidates are the per-anchor BIAS
 * (~0.88 m of range difference, which displaces the solution the equations
 * agree on) and the outlier TAIL (`sync stats` reported max_dtu 633 = 2.97 m
 * against an RMS of 52, which the shell itself calls non-Gaussian).
 *
 * The runaway itself, measured on hardware 2026-09-03
 * (COM15_2026_09_03.11.58.45.067.txt): a single tag left the 2.4 m array and
 * ran to 17.24 m outside it over 30.2 s and 137 consecutive fixes,
 * monotonically, at ~1.5 m/s -- while `blink stats` reported reseed:0 for the
 * entire episode. Full evidence in
 * docs/superpowers/plans/2026-09-03-tdoa-accuracy-filter-part2.md, Task 7.
 *
 * And the arithmetic that says the recovery was never even ARMED, which is
 * why the next step is instrumenting the gateway rather than tuning this
 * test until it fails: gate_streak advances only on a cycle where every
 * equation is gated, and `gate_rejected:70` over 304 filtered cycles of 2
 * equations each allows at most 35 such cycles -- while reset_after needs 3
 * CONSECUTIVE ones. So the filter was accepting at least one equation on
 * almost every cycle of the escape, which is a different situation from
 * "the gate closed and it coasted".
 *
 * THE MECHANISM this was built to exercise:
 * pos_ekf_needs_reseed() fires on gate_streak >= cfg->reset_after, and
 * gate_streak only ADVANCES when accepted == 0 -- every equation gated. At
 * TDOA_MIN_ANCHORS (3) pos_ekf_update_tdoa() produces just 2 equations, so a
 * single acceptance resets the streak to zero. And one range-difference
 * equation constrains ONE direction, so the filter can accept an update every
 * cycle -- streak permanently zero, recovery never armed -- while running
 * away along the direction that equation does not constrain.
 *
 * The geometry below is the deployed one, and it is what makes that direction
 * exist: with the tag on the base line between origin and xaxis, the
 * origin/xaxis equation has d/dy = -0.001 (measured) because moving
 * perpendicular to that base changes both ranges almost equally. All of y
 * rests on the apex equation's gain of 0.647.
 */

/* The surveyed geometry from that capture. Thin on purpose -- a square array
 * does NOT reproduce this, which is why the existing tests miss it. */
static const float RUN_AX[3] = { 0.000f, 1.752f, 2.385f };
static const float RUN_AY[3] = { 0.000f, 0.920f, 0.000f };

static void make_runaway_obs(struct tdoa_meas *m, float tx, float ty,
                             int32_t noise_dtu)
{
    for (int i = 0; i < 3; i++) {
        float dx = tx - RUN_AX[i], dy = ty - RUN_AY[i];
        float r = sqrtf(dx * dx + dy * dy);

        m[i].x = RUN_AX[i];
        m[i].y = RUN_AY[i];
        m[i].dz = 0.0f;          /* apos tagz is 0.0 on that site */
        /* The sigma the anchors actually publish: sync stats reported
         * jitter_est_dtu 33 on a deployed anchor, i.e. 0.155 m. Used here
         * BECAUSE it is overconfident against the ~0.5 m residual the
         * 4-anchor run measured -- an R that small is what closes the 3-sigma
         * gate on real measurements and leaves the filter predicting. */
        m[i].sigma_m = 33.0f * TDOA_M_PER_DTU;
        m[i].t_dtu = 500000 + (int64_t)llroundf(r / TDOA_M_PER_DTU)
                     + rng_noise_dtu(noise_dtu);
    }
}

static float dist_outside_triangle(float x, float y)
{
    float best = 1e9f;
    int neg = 0, pos = 0;

    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        float ax = RUN_AX[i], ay = RUN_AY[i];
        float bx = RUN_AX[j], by = RUN_AY[j];
        float cr = (bx - ax) * (y - ay) - (by - ay) * (x - ax);

        if (cr < 0.0f) neg = 1;
        if (cr > 0.0f) pos = 1;

        float ex = bx - ax, ey = by - ay;
        float L2 = ex * ex + ey * ey;
        float t = (L2 <= 0.0f) ? 0.0f
                  : ((x - ax) * ex + (y - ay) * ey) / L2;

        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float d = sqrtf((x - (ax + t * ex)) * (x - (ax + t * ex)) +
                        (y - (ay + t * ey)) * (y - (ay + t * ey)));

        if (d < best) best = d;
    }
    return (neg && pos) ? best : 0.0f;   /* 0 when inside */
}

static void test_thin_geometry_stays_bounded_under_noise(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct tdoa_meas m[3];
    const float tx = 0.665f, ty = 0.0f;   /* the spot the tag actually was */
    const float dt = 0.2f;
    float worst_out = 0.0f;
    int reseed_asked = 0;
    int cycles_gated_all = 0;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    g_rng_state = 20260903u;
    pos_ekf_seed(&f, tx, ty);

    /* 300 cycles at 5 Hz = 60 s, the order of the observed 30.2 s episode.
     * `moving` is true throughout: the hardware runaway happened while the
     * tag's accelerometer reported motion, so no ZUPT pins the velocity.
     * That is the case to reproduce -- ZUPT working is already covered by
     * test_tdoa_zupt_pins_velocity(). */
    for (int k = 0; k < 300; k++) {
        make_runaway_obs(m, tx, ty, 64);   /* +/-64 DTU = +/-0.30 m */

        pos_ekf_predict(&f, &c, dt, true);

        int accepted = pos_ekf_update_tdoa(&f, &c, m, 3);

        if (accepted == 0) {
            cycles_gated_all++;
        }
        if (pos_ekf_needs_reseed(&f, &c)) {
            reseed_asked++;
            /* What tdoa_gw.c does on that signal: reseed from a fresh
             * solve. Modelled as a seed at the truth, which is the most
             * favourable version of it -- if even that does not keep the
             * filter bounded, the recovery is not the problem. */
            pos_ekf_seed(&f, tx, ty);
        }

        float x, y;

        if (pos_ekf_get(&f, &x, &y, NULL, NULL)) {
            float out = dist_outside_triangle(x, y);

            if (out > worst_out) worst_out = out;
        }
    }

    printf("  thin geometry, unbiased noise: worst %.2f m outside; "
           "needs_reseed fired %d time(s), all-gated cycles %d\n",
           (double)worst_out, reseed_asked, cycles_gated_all);

    /* A tag cannot be 3 m outside a 2.4 m array. This bound HOLDS against
     * HEAD, which is the finding: unbiased noise alone does not produce the
     * hardware escape. Keep the assertion anyway -- it is a real property and
     * a future change that breaks it is a regression worth catching. */
    CHECK(worst_out < 3.0f);
    CHECK(all_finite_state(&f));

    /* And the diagnosis, pinned separately so a future change that fixes the
     * distance by luck rather than by arming the recovery still shows up:
     * if the filter DID escape, the recovery must have been asked. */
    if (worst_out >= 3.0f) {
        CHECK(reseed_asked > 0);
    }
}

int main(void)
{
    test_thin_geometry_stays_bounded_under_noise();
    test_tdoa_per_anchor_weighting();
    test_tdoa_missing_sigma_falls_back();
    test_tdoa_zupt_pins_velocity();
    test_converges_static_from_poor_seed();
    test_tracks_constant_velocity();
    test_outlier_gated();
    test_zupt_zeroes_velocity();
    test_speed_clamp_engages();
    test_stationary_variance_falls();
    test_pos_sigma_uses_both_diag_terms();
    test_gate_streak_and_reseed();
    test_gate_streak_saturates();
    test_reset_after_zero_never_reseeds();
    test_dt_variation_stable();
    test_q_position_and_cross_terms();
    test_dt_guards();
    test_covariance_symmetric_positive_diag();
    test_get_before_seed_and_null_pointers();
    test_no_nan_inf_long_run();
    test_trap1_int64_subtraction_order_matters();
    test_tdoa_filter_beats_raw_solve();
    test_tdoa_stationary_does_not_drift();
    test_tdoa_gate_rejects_outlier();
    test_tdoa_degenerate_geometry_no_nan();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}

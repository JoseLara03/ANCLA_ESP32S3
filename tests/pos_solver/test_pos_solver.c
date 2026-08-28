#include "pos_solver.h"
#include "pos_residual.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Rectangle anchor layout: a 6 m x 4 m room, anchors in the corners,
 * ceiling-mounted 1.6 m above a waist-worn tag -- the exact scenario worked
 * through in spec/2026-08-22-position-filtering-design.md's slant-range
 * table ("anchors at 2.7 m and the tag at 1.1 m, dz = 1.6 m"). */
static const float AX[4] = { 0.0f, 6.0f, 0.0f, 6.0f };
static const float AY[4] = { 0.0f, 0.0f, 4.0f, 4.0f };
#define ROOM_DZ  1.6f

/* Fill m[0..3] with the exact 3D slant range from (tx, ty) to each corner
 * anchor, at vertical offset dz. Passing dz = 0 here (with the anchor x/y
 * unchanged) is how test_planar_model_is_visibly_wrong_3d_is_not() feeds a
 * genuine slant range into a planar model -- exactly the pre-2026-08-22
 * defect. */
static void make_measurements(float tx, float ty, float dz, struct pos_meas m[4])
{
    for (int i = 0; i < 4; i++) {
        float dx = tx - AX[i];
        float dy = ty - AY[i];
        m[i].x = AX[i];
        m[i].y = AY[i];
        m[i].dz = dz;
        m[i].range_m = sqrtf(dx * dx + dy * dy + dz * dz);
    }
}

static void make_slant_measurements(float tx, float ty, struct pos_meas m[4])
{
    make_measurements(tx, ty, ROOM_DZ, m);
}

static void test_exact_recovery_3d(void)
{
    struct pos_meas m[4];
    make_slant_measurements(3.0f, 2.0f, m);

    struct pos_result out;
    bool ok = pos_solve(m, 4, NULL, &out);

    CHECK(ok);
    CHECK(out.valid);
    CHECK(fabsf(out.x - 3.0f) < 1e-3f);
    CHECK(fabsf(out.y - 2.0f) < 1e-3f);
    CHECK(out.residual_m < 1e-3f);
    CHECK(out.n_used == 4);
    CHECK(out.dropped_idx == POS_NO_DROP);
}

static void test_cold_start_converges_from_various_positions(void)
{
    /* Deliberately includes the two shapes of geometry the design flags as
     * hard: right up against a wall (small horizontal separation from two
     * anchors) and almost directly under an anchor (that anchor's Jacobian
     * row nearly vanishes, so it contributes little directional
     * information and the fit leans on the other three). */
    struct { float tx, ty; } cases[] = {
        { 3.00f, 2.00f }, /* room centre                */
        { 0.10f, 2.00f }, /* hugging the x = 0 wall     */
        { 5.90f, 2.00f }, /* hugging the x = 6 wall     */
        { 3.00f, 0.10f }, /* hugging the y = 0 wall     */
        { 0.05f, 0.05f }, /* almost under anchor 0      */
        { 5.95f, 3.95f }, /* almost under anchor 3      */
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        struct pos_meas m[4];
        make_slant_measurements(cases[i].tx, cases[i].ty, m);

        struct pos_result out;
        bool ok = pos_solve(m, 4, NULL, &out);

        CHECK(ok);
        CHECK(out.valid);
        CHECK(fabsf(out.x - cases[i].tx) < 1e-2f);
        CHECK(fabsf(out.y - cases[i].ty) < 1e-2f);
    }
}

/* The whole point of the 2026-08-22 change: feeding a real slant range into
 * a planar model (dz forced to 0, mirroring the pre-change estimator)
 * produces a position visibly off the truth, while the 3D model recovers it
 * essentially exactly from the same underlying ranges. */
static void test_planar_model_is_visibly_wrong_3d_is_not(void)
{
    struct pos_meas slant[4];
    make_slant_measurements(1.0f, 0.5f, slant); /* near a wall: worst dz/range ratio */

    struct pos_result out3d;
    bool ok3d = pos_solve(slant, 4, NULL, &out3d);
    CHECK(ok3d);
    CHECK(out3d.valid);
    float err3d = hypotf(out3d.x - 1.0f, out3d.y - 0.5f);
    CHECK(err3d < 0.01f);

    struct pos_meas planar[4];
    for (int i = 0; i < 4; i++) {
        planar[i] = slant[i];
        planar[i].dz = 0.0f; /* same range_m -- only the model changes */
    }

    struct pos_result outp;
    bool okp = pos_solve(planar, 4, NULL, &outp);

    /* Either the planar solve fails outright, or it converges well off the
     * true position -- an inconsistent set of circles either has no clean
     * intersection or has one far from where the 3D model puts it. A planar
     * solve landing within centimetres of the truth would mean the defect
     * this design fixes was never real, so that is the one outcome that
     * must NOT happen. */
    float errp = (okp && outp.valid) ? hypotf(outp.x - 1.0f, outp.y - 0.5f) : 1e9f;
    CHECK(!okp || !outp.valid || errp > 0.3f);
}

static void test_three_anchor_case(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);

    struct pos_result out;
    bool ok = pos_solve(m, 3, NULL, &out); /* only the first 3 corners */

    CHECK(ok);
    CHECK(out.valid);
    CHECK(fabsf(out.x - 2.0f) < 1e-3f);
    CHECK(fabsf(out.y - 1.0f) < 1e-3f);
    CHECK(out.n_used == 3);
    CHECK(out.dropped_idx == POS_NO_DROP);
}

static void test_three_anchor_never_attempts_rejection(void)
{
    /* n == 3 has no spare anchor -- per pos_solver.h, rejection must not
     * even be attempted, so a bad range biases the fit instead of being
     * dropped. */
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);
    m[1].range_m += 1.0f;

    struct pos_result out;
    bool ok = pos_solve(m, 3, NULL, &out);

    CHECK(ok);
    CHECK(out.valid);
    CHECK(out.n_used == 3);
    CHECK(out.dropped_idx == POS_NO_DROP);
}

/* All four anchors collinear (sharing y = 0): the reference-anchor
 * subtraction linear_seed() uses collapses to a rank-1 system (every row's
 * y-component is 0), so it falls back to the anchor centroid -- which for a
 * collinear layout sits exactly ON the line. Gauss-Newton's own Jacobian at
 * a point ON that line is then also singular by construction (dh/dy = 0 for
 * every anchor), so the degeneracy is caught on the very first iteration.
 * See derive_seed()'s comment in pos_solver.c. */
static void test_collinear_anchors_are_rejected(void)
{
    const float cx[4] = { 0.0f, 2.0f, 4.0f, 6.0f };
    const float cy[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    struct pos_meas m[4];
    for (int i = 0; i < 4; i++) {
        float dx = 3.0f - cx[i];
        float dy = 2.0f - cy[i];
        m[i].x = cx[i];
        m[i].y = cy[i];
        m[i].dz = ROOM_DZ;
        m[i].range_m = sqrtf(dx * dx + dy * dy + ROOM_DZ * ROOM_DZ);
    }

    struct pos_result out;
    bool ok = pos_solve(m, 4, NULL, &out);

    CHECK(!ok);
    CHECK(!out.valid);
}

static void test_single_outlier_is_dropped_and_fix_stays_accurate(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);
    const int bad = 1;
    m[bad].range_m += 1.0f; /* NLOS-style bias, design doc: +0.3-2 m */

    struct pos_result out;
    bool ok = pos_solve(m, 4, NULL, &out);

    CHECK(ok);
    CHECK(out.valid);
    CHECK(out.n_used == 3);
    CHECK(out.dropped_idx == (uint8_t)bad);
    CHECK(fabsf(out.x - 2.0f) < 1e-2f);
    CHECK(fabsf(out.y - 1.0f) < 1e-2f);
    CHECK(out.residual_m < 0.05f); /* the 3 surviving ranges are exact */
}

/* out->residual_m must describe the anchors actually USED (3, after the
 * drop), not the full set -- rebuild that subset independently and compare
 * against a fresh pos_residual_rms() call. */
static void test_residual_field_covers_only_used_anchors(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);
    m[1].range_m += 1.0f;

    struct pos_result out;
    CHECK(pos_solve(m, 4, NULL, &out));
    CHECK(out.valid);
    CHECK(out.n_used == 3);

    struct pos_meas used[3];
    size_t k = 0;
    for (size_t i = 0; i < 4; i++) {
        if (i != out.dropped_idx) {
            used[k++] = m[i];
        }
    }
    float expect = pos_residual_rms(used, 3, out.x, out.y);
    CHECK(fabsf(out.residual_m - expect) < 1e-5f);
}

static void test_clean_four_anchor_fix_is_never_dropped(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);

    /* Small, realistic per-range noise (a couple of cm), well under
     * POS_OUTLIER_RES_FLOOR_M -- ordinary measurement jitter, not an
     * outlier. A good 4-anchor fix must never lose an anchor to this. */
    const float jitter[4] = { 0.02f, -0.015f, 0.01f, -0.02f };
    for (int i = 0; i < 4; i++) {
        m[i].range_m += jitter[i];
    }

    struct pos_result out;
    bool ok = pos_solve(m, 4, NULL, &out);

    CHECK(ok);
    CHECK(out.valid);
    CHECK(out.n_used == 4);
    CHECK(out.dropped_idx == POS_NO_DROP);
}

static void test_nonfinite_input_is_rejected(void)
{
    struct pos_meas base[4];
    make_slant_measurements(2.0f, 1.0f, base);

    struct pos_meas m_nan[4];
    memcpy(m_nan, base, sizeof(base));
    m_nan[1].range_m = NAN;
    struct pos_result out1;
    CHECK(!pos_solve(m_nan, 4, NULL, &out1));
    CHECK(!out1.valid);

    struct pos_meas m_inf[4];
    memcpy(m_inf, base, sizeof(base));
    m_inf[2].x = INFINITY;
    struct pos_result out2;
    CHECK(!pos_solve(m_inf, 4, NULL, &out2));
    CHECK(!out2.valid);

    struct pos_meas m_neg[4];
    memcpy(m_neg, base, sizeof(base));
    m_neg[0].range_m = -1.0f; /* physically impossible, must also be rejected */
    struct pos_result out3;
    CHECK(!pos_solve(m_neg, 4, NULL, &out3));
    CHECK(!out3.valid);

    float bad_seed[2] = { NAN, 0.0f };
    struct pos_result out4;
    CHECK(!pos_solve(base, 4, bad_seed, &out4));
    CHECK(!out4.valid);
}

static void test_n_out_of_range_is_rejected(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.0f, 1.0f, m);

    struct pos_result out;
    CHECK(!pos_solve(m, 2, NULL, &out));
    CHECK(!out.valid);
    CHECK(!pos_solve(m, 5, NULL, &out)); /* > POS_MAX_ANCHORS; m[] is never
                                           * touched for an out-of-range n */
    CHECK(!out.valid);
}

static void test_seeded_solve_agrees_with_cold_solve(void)
{
    struct pos_meas m[4];
    make_slant_measurements(2.5f, 1.5f, m);

    struct pos_result cold;
    CHECK(pos_solve(m, 4, NULL, &cold));
    CHECK(cold.valid);

    float seed[2] = { 2.4f, 1.4f }; /* a plausible previous fix, nearby */
    struct pos_result seeded;
    CHECK(pos_solve(m, 4, seed, &seeded));
    CHECK(seeded.valid);

    CHECK(fabsf(cold.x - seeded.x) < 1e-3f);
    CHECK(fabsf(cold.y - seeded.y) < 1e-3f);
}


/* A seed deliberately far from the answer, so a converged result can only
 * have come from the iteration -- not from derive_seed(), which is exact on
 * noise-free data and would otherwise let a gutted gn_solve() pass. */
static void solve_from_far_seed(const struct pos_meas *m, size_t n,
                                struct pos_result *out)
{
    const float far_seed[2] = { -40.0f, 55.0f };

    (void)pos_solve(m, n, far_seed, out);
}

static void test_gauss_newton_actually_iterates(void)
{
    struct pos_meas m[4];
    struct pos_result out;

    /* Kills the mutant "replace gn_solve()'s body with *ox = x0; *oy = y0;
     * return true;". Every accuracy assertion elsewhere in this file seeds
     * from derive_seed(), which is already exact on noise-free data, so they
     * all pass with Gauss-Newton removed entirely. This one cannot. */
    make_slant_measurements(2.0f, 3.0f, m);
    solve_from_far_seed(m, 4, &out);

    CHECK(out.valid);
    CHECK(fabsf(out.x - 2.0f) < 1e-2f);
    CHECK(fabsf(out.y - 3.0f) < 1e-2f);
}

static void test_solver_side_dz_is_load_bearing(void)
{
    struct pos_meas m[4];
    struct pos_result out;

    /* Kills the mutant "drop + dz*dz from the solver's h, leaving
     * pos_residual.c intact". The sibling planar test varies the DATA (dz = 0
     * in the measurements) so it cannot see a solver that ignores a dz that
     * IS present. Seeding far away forces the answer through the Jacobian,
     * where the dropped term lives. */
    make_slant_measurements(1.5f, 1.0f, m);
    solve_from_far_seed(m, 4, &out);

    CHECK(out.valid);
    CHECK(fabsf(out.x - 1.5f) < 1e-2f);
    CHECK(fabsf(out.y - 1.0f) < 1e-2f);
}

static void test_collinear_rejected_even_with_a_seed(void)
{
    struct pos_meas m[4];
    struct pos_result out;

    /* Collinear anchors along y = 0. The mirror pair (x, +y) and (x, -y) fit
     * identically, so the problem has no unique answer regardless of how good
     * the ranges are.
     *
     * This is the case the runner now hits by default: it seeds every solve
     * from the filter, and under a seed Gauss-Newton converges contentedly to
     * whichever mirror the seed is nearer. det(J^T J) does NOT catch it --
     * measured, collinear geometry reaches 2.23 against a legitimate-room
     * minimum of 1.89 -- so rejection has to come from the anchor spread. */
    for (int i = 0; i < 4; i++) {
        m[i].x  = 2.0f * (float)i;
        m[i].y  = 0.0f;
        m[i].dz = ROOM_DZ;
    }
    for (int i = 0; i < 4; i++) {
        float dx = 3.0f - m[i].x;
        float dy = 2.0f - m[i].y;
        m[i].range_m = sqrtf(dx * dx + dy * dy + ROOM_DZ * ROOM_DZ);
    }

    const float above[2] = { 3.0f,  1.9f };
    const float below[2] = { 3.0f, -1.9f };

    CHECK(!pos_solve(m, 4, above, &out));
    CHECK(!out.valid);
    CHECK(!pos_solve(m, 4, below, &out));
    CHECK(!out.valid);
    CHECK(!pos_solve(m, 4, NULL, &out));

    /* Near-collinear must go too: a 1 cm bow is survey noise, not geometry. */
    m[1].y = 0.01f;
    CHECK(!pos_solve(m, 4, above, &out));

    /* ...but a genuinely spread set with the same seed must still solve, so
     * the check is rejecting degeneracy rather than everything. */
    make_slant_measurements(3.0f, 1.9f, m);
    CHECK(pos_solve(m, 4, above, &out));
    CHECK(out.valid);
}

static void test_valid_implies_a_stationary_point(void)
{
    unsigned st = 99u;
    int checked = 0;

    /* The contract "a seed must never make a converged solve report success
     * at a point the ranges do not support" is really a statement that a
     * valid result is a stationary point of the least-squares problem, so
     * assert exactly that -- over adversarial random input rather than one
     * hand-built case.
     *
     * Constructing the pathology by hand is a trap: the obvious attempt
     * (contradictory ranges, seeded at the room centre) fails because the
     * centre of a symmetric anchor set genuinely IS the least-squares
     * solution, large residual and all. A large residual is not a defect --
     * it is the caller's signal that the ranges disagree. Returning a point
     * where the gradient is NOT zero is the defect. */
    for (int k = 0; k < 20000; k++) {
        struct pos_meas m[4];
        struct pos_result out;
        float seed[2];

#define RND ((float)((st = st * 1664525u + 1013904223u) >> 8 & 0xFFFFu) / 65536.0f)
        for (int i = 0; i < 4; i++) {
            m[i].x       = RND * 8.0f;
            m[i].y       = RND * 6.0f;
            m[i].dz      = RND * 2.0f;
            m[i].range_m = RND * 12.0f;
        }
        seed[0] = -20.0f + RND * 40.0f;
        seed[1] = -20.0f + RND * 40.0f;
#undef RND

        if (!pos_solve(m, 4, seed, &out)) {
            continue;
        }
        checked++;

        /* Gradient of 0.5 * sum(r_i^2) at the reported point, over the
         * anchors actually USED. Skipping the dropped one matters: after a
         * rejection the answer is the subset's optimum, so the FULL-set
         * gradient there is non-zero by construction and testing it would be
         * asserting the wrong invariant. */
        float g0 = 0.0f, g1 = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (out.dropped_idx != POS_NO_DROP && i == (int)out.dropped_idx) {
                continue;
            }
            float dx = out.x - m[i].x;
            float dy = out.y - m[i].y;
            float h  = sqrtf(dx * dx + dy * dy + m[i].dz * m[i].dz);

            if (h < 1e-3f) { h = 1e-3f; }
            float r = h - m[i].range_m;
            g0 += (dx / h) * r;
            g1 += (dy / h) * r;
        }
        CHECK(sqrtf(g0 * g0 + g1 * g1) < 0.05f);
    }
    /* Also pins the backtracking line search, which is otherwise invisible:
     * on benign room geometry it changes nothing (100 % accurate solves with
     * or without it, even from seeds 60 m outside the room), so only wild
     * geometry shows what it buys. Measured over this exact loop: 86.9 % of
     * cases converge with the line search, 60.4 % without it. The bound sits
     * between the two, so deleting the line search fails here. */
    CHECK(checked > 15000);
    CHECK(checked <= 20000);
}

static void test_clean_fix_survives_realistic_noise(void)
{
    struct pos_meas m[4];
    struct pos_result out;
    unsigned st = 7u;
    int drops = 0, solved = 0;

    /* The sibling "never dropped" test uses 2 cm jitter -- 6x tighter than the
     * 12 cm sigma the design actually assumes, so it verified the guarantee
     * only where it was never in doubt. At the real assumed sigma the old
     * RMS-ratio threshold discarded a good anchor on 3.8 % of clean fixes.
     * Deterministic LCG so a failure is reproducible. */
    for (int k = 0; k < 4000; k++) {
        float tx = 0.5f + 5.0f * (float)((st = st * 1664525u + 1013904223u)
                                         >> 8 & 0xFFFFu) / 65536.0f;
        float ty = 0.5f + 3.0f * (float)((st = st * 1664525u + 1013904223u)
                                         >> 8 & 0xFFFFu) / 65536.0f;

        make_slant_measurements(tx, ty, m);
        for (int i = 0; i < 4; i++) {
            /* Sum of 4 uniforms ~ Gaussian, scaled to sigma = 0.12 m. */
            float u = 0.0f;
            for (int j = 0; j < 4; j++) {
                u += (float)((st = st * 1664525u + 1013904223u) >> 8 & 0xFFFFu)
                     / 65536.0f - 0.5f;
            }
            m[i].range_m += 0.12f * u * 1.732f;
        }

        if (pos_solve(m, 4, NULL, &out)) {
            solved++;
            if (out.dropped_idx != POS_NO_DROP) {
                drops++;
            }
        }
    }

    CHECK(solved > 3900);
    CHECK(drops * 200 < solved);   /* well under 0.5 % */
}

static void test_rejection_contract_holds_under_stress(void)
{
    unsigned st = 4242u;

    /* Pins the documented shape of the rejection result -- at most one anchor
     * dropped, n_used consistent with dropped_idx, dropped_idx indexing the
     * CALLER's array -- across noisy, multi-outlier and contradictory inputs.
     *
     * Deliberately does NOT assert that two bad anchors produce no drop. With
     * three anchors and two unknowns the fit has one degree of freedom, so a
     * remaining outlier can be partly absorbed; demanding detection there
     * would be asserting a guarantee the geometry cannot give, which is the
     * same reason pos_solver.h does not attempt rejection at n == 3 at all. */
    for (int k = 0; k < 20000; k++) {
        struct pos_meas m[4];
        struct pos_result out;

#define RND ((float)((st = st * 1664525u + 1013904223u) >> 8 & 0xFFFFu) / 65536.0f)
        /* Separate statements: two RND uses in one expression would modify
         * `st` twice without a sequence point. */
        float tx = 0.5f + RND * 5.0f;
        float ty = 0.5f + RND * 3.0f;

        make_slant_measurements(tx, ty, m);
        for (int i = 0; i < 4; i++) {
            m[i].range_m += 0.12f * (RND - 0.5f) * 3.5f;
        }
        int nbad = (int)(RND * 3.0f);
        for (int b2 = 0; b2 < nbad; b2++) {
            int idx = (int)(RND * 4.0f);
            if (idx > 3) { idx = 3; }
            m[idx].range_m += 0.4f + RND * 2.0f;
        }
#undef RND

        if (!pos_solve(m, 4, NULL, &out)) {
            continue;
        }
        if (out.dropped_idx == POS_NO_DROP) {
            CHECK(out.n_used == 4);
        } else {
            CHECK(out.dropped_idx < 4);
            CHECK(out.n_used == 3);
        }
        CHECK(out.valid);
        CHECK(isfinite(out.x) && isfinite(out.y) && isfinite(out.residual_m));
        CHECK(out.residual_m >= 0.0f);
    }
}


static void test_leverage_correction_is_load_bearing(void)
{
    unsigned st = 31337u;
    int solved = 0, dropped = 0;

    /* Least squares absorbs part of a single bad range by moving the fix, so
     * the leftover residual understates the error by the leverage factor
     * (1 - h_ii) -- about half with four anchors and two unknowns. Dividing it
     * out is what takes +1 m NLOS detection from ~72 % to ~95 %.
     *
     * Without that correction this loop drops an anchor far less often, so the
     * bound below fails. Measured with the correction: ~95 % dropped. */
    for (int k = 0; k < 6000; k++) {
        struct pos_meas m[4];
        struct pos_result out;

#define RND ((float)((st = st * 1664525u + 1013904223u) >> 8 & 0xFFFFu) / 65536.0f)
        float tx = 0.6f + RND * 4.8f;
        float ty = 0.6f + RND * 2.8f;

        make_slant_measurements(tx, ty, m);
        for (int i = 0; i < 4; i++) {
            m[i].range_m += 0.12f * (RND - 0.5f) * 3.46f;
        }
        int bad = (int)(RND * 4.0f);
#undef RND
        if (bad > 3) { bad = 3; }
        m[bad].range_m += 1.0f;          /* a 1 m NLOS hit */

        if (pos_solve(m, 4, NULL, &out)) {
            solved++;
            if (out.dropped_idx != POS_NO_DROP) {
                dropped++;
            }
        }
    }

    CHECK(solved > 5800);
    CHECK(dropped * 100 > solved * 85);   /* >85 % caught */
}

int main(void)
{
    test_exact_recovery_3d();
    test_gauss_newton_actually_iterates();
    test_solver_side_dz_is_load_bearing();
    test_cold_start_converges_from_various_positions();
    test_planar_model_is_visibly_wrong_3d_is_not();
    test_three_anchor_case();
    test_three_anchor_never_attempts_rejection();
    test_collinear_anchors_are_rejected();
    test_collinear_rejected_even_with_a_seed();
    test_valid_implies_a_stationary_point();
    test_single_outlier_is_dropped_and_fix_stays_accurate();
    test_residual_field_covers_only_used_anchors();
    test_clean_four_anchor_fix_is_never_dropped();
    test_clean_fix_survives_realistic_noise();
    test_leverage_correction_is_load_bearing();
    test_rejection_contract_holds_under_stress();
    test_nonfinite_input_is_rejected();
    test_n_out_of_range_is_rejected();
    test_seeded_solve_agrees_with_cold_solve();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}

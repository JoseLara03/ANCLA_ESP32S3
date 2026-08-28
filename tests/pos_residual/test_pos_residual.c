#include "pos_residual.h"
#include "pos_solver.h"
#include <math.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Anchors on the unit square; true position is its centre, (0.5, 0.5), which
 * is sqrt(0.5) from every corner in the plane (dz = 0 here -- these tests
 * are checking the planar part of the model, so every measurement's dz is
 * explicitly 0). */
#define D_CENTRE  0.70710678f

static void test_consistent_geometry_has_zero_residual(void)
{
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, 0.0f, D_CENTRE }, { 1.0f, 1.0f, 0.0f, D_CENTRE },
    };
    CHECK(fabsf(pos_residual_rms(m, 4, 0.5f, 0.5f)) < 1e-5f);
}

static void test_one_bad_range_shows_up(void)
{
    /* One range inflated by exactly 1 m. Three errors are 0, one is 1, so
     * the RMS is sqrt(1/4) = 0.5 exactly. */
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, 0.0f, D_CENTRE }, { 1.0f, 1.0f, 0.0f, D_CENTRE + 1.0f },
    };
    CHECK(fabsf(pos_residual_rms(m, 4, 0.5f, 0.5f) - 0.5f) < 1e-4f);
}

static void test_three_anchor_case(void)
{
    /* Three anchors, consistent. n == 3 is the minimum the solver accepts. */
    struct pos_meas m[3] = {
        { 0.0f, 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, 0.0f, D_CENTRE },
    };
    CHECK(fabsf(pos_residual_rms(m, 3, 0.5f, 0.5f)) < 1e-5f);
}

static void test_offset_position_is_penalised(void)
{
    /* Same consistent ranges, but evaluated at the wrong position: the
     * residual must be clearly non-zero. Guards against a stub that always
     * returns 0. */
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, 0.0f, D_CENTRE }, { 1.0f, 1.0f, 0.0f, D_CENTRE },
    };
    CHECK(pos_residual_rms(m, 4, 2.0f, 2.0f) > 0.5f);
}

static void test_zero_measurements(void)
{
    struct pos_meas m[1] = { { 0.0f, 0.0f, 0.0f, 1.0f } };
    CHECK(pos_residual_rms(m, 0, 0.0f, 0.0f) == 0.0f);
}

/* dz is part of the model, not an afterthought: a slant range with a large
 * vertical offset must be recognised as consistent with the true horizontal
 * position once dz is supplied, and the *same* range data must show up as
 * badly inconsistent if evaluated as if it were planar (dz = 0) -- that gap
 * is the entire reason struct pos_meas grew a dz field
 * (spec/2026-08-22-position-filtering-design.md, "Slant range fed into a
 * planar model"). */
static void test_dz_is_part_of_the_model(void)
{
    const float dz = 1.6f; /* ceiling anchor minus waist-height tag, per spec */
    const float tx = 1.0f, ty = 0.0f;

    struct pos_meas m[4];
    const float ax[4] = { 0.0f, 3.0f, 0.0f, 3.0f };
    const float ay[4] = { 0.0f, 0.0f, 2.0f, 2.0f };

    for (int i = 0; i < 4; i++) {
        float dx = tx - ax[i];
        float dy = ty - ay[i];
        m[i].x = ax[i];
        m[i].y = ay[i];
        m[i].dz = dz;
        m[i].range_m = sqrtf(dx * dx + dy * dy + dz * dz); /* true slant range */
    }

    /* Evaluated with dz honoured, the true horizontal position is exactly
     * consistent with every slant range. */
    CHECK(fabsf(pos_residual_rms(m, 4, tx, ty)) < 1e-4f);

    /* The same ranges, treated as planar (dz forced to 0, as the estimator
     * did before this change), are NOT consistent with that position -- the
     * slant range overshoots the planar distance by a dz-dependent amount at
     * every anchor. */
    struct pos_meas planar[4];
    for (int i = 0; i < 4; i++) {
        planar[i] = m[i];
        planar[i].dz = 0.0f;
    }
    CHECK(pos_residual_rms(planar, 4, tx, ty) > 0.3f);
}

int main(void)
{
    test_consistent_geometry_has_zero_residual();
    test_one_bad_range_shows_up();
    test_three_anchor_case();
    test_offset_position_is_penalised();
    test_zero_measurements();
    test_dz_is_part_of_the_model();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}

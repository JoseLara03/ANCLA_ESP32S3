#ifndef POS_SOLVER_H
#define POS_SOLVER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Maximum anchors per solve (matches the static anchor list). 2D needs >=3. */
#define POS_MAX_ANCHORS  4

/* Sentinel for pos_result.dropped_idx when no anchor was rejected. */
#define POS_NO_DROP  0xFFu

/*
 * One anchor measurement.
 *
 * `dz` is the vertical offset (anchor z - tag z) in metres, so the model range
 * is the 3D slant distance:
 *
 *     h_i = sqrt((x - x_i)^2 + (y - y_i)^2 + dz_i^2)
 *
 * The DW3000 measures a slant range to a ceiling-mounted anchor. Feeding that
 * into a planar model (the pre-2026-08-22 behaviour, dz implicitly 0) makes the
 * range circles mutually inconsistent, which leaves the least-squares problem
 * with a flat minimum the solution slides along -- the "big jump from small
 * motion" symptom. Carrying dz inside the measurement model, rather than
 * pre-projecting the range onto the horizontal plane, is deliberate: projection
 * needs a clamp when range < dz and its error diverges as dz/r_h near an
 * anchor, whereas the model form makes the Jacobian shrink exactly where the
 * horizontal information does.
 *
 * See spec/2026-08-22-position-filtering-design.md.
 */
struct pos_meas {
    float x;        /* anchor x, metres */
    float y;        /* anchor y, metres */
    float dz;       /* anchor z - tag z, metres */
    float range_m;  /* measured slant range, metres */
};

/* Solved 2D position. valid == false means no solution was produced. */
struct pos_result {
    float   x;
    float   y;
    float   residual_m;   /* RMS range residual over the USED anchors; 0 if !valid */
    bool    valid;
    uint8_t n_used;       /* anchors that contributed (n, or n-1 after a drop) */
    uint8_t dropped_idx;  /* index into the caller's array, or POS_NO_DROP */
};

/*
 * 2D trilateration by Gauss-Newton on the 3D range model, with single-outlier
 * rejection.
 *
 * `seed_xy` is an optional 2-element {x, y} starting point (typically the
 * previous fix). Pass NULL to let the solver derive its own seed. A seed only
 * affects convergence speed and which local minimum is reached; it must never
 * be able to make a converged solve report success at a point the ranges do not
 * support.
 *
 * Outlier rejection: with n == POS_MAX_ANCHORS the solver also evaluates every
 * 3-anchor subset and adopts the best one when it beats the full-set residual
 * by a clear margin, reporting the excluded anchor in out->dropped_idx. With
 * n == 3 there is no redundancy and no rejection is attempted.
 *
 * Returns false (and sets out->valid = false) when n is out of range, the
 * geometry is degenerate (collinear anchors -> singular normal matrix), or the
 * iteration fails to converge.
 *
 * Pure: no CMSIS-DSP, no Zephyr. The normal equations are 2x2 and inverted in
 * closed form, which is both cheaper than arm_mat_inverse_f32 and lets the
 * determinant be tested explicitly for the degenerate case. This is what makes
 * the solver host-testable, which it previously was not.
 */
bool pos_solve(const struct pos_meas *m, size_t n, const float *seed_xy,
               struct pos_result *out);

#endif /* POS_SOLVER_H */

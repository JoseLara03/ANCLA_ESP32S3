#include "pos_solver.h"
#include "pos_residual.h"
#include <math.h>

/*
 * Gauss-Newton on the raw nonlinear 3D range model. See
 * spec/2026-08-22-position-filtering-design.md "Snapshot solver".
 *
 * No CMSIS-DSP: the normal equations are always 2x2 (two unknowns, x and y),
 * so a closed-form inverse (det = a*d - b*c) is both cheaper than
 * arm_mat_inverse_f32 and lets the degenerate case be tested explicitly
 * against an epsilon instead of trusting a library return code. It is also
 * what makes this module host-testable at all -- arm_math.h does not build
 * outside the Zephyr/CMSIS toolchain.
 */

/* Iteration cap for the Gauss-Newton loop. The problem is only mildly
 * nonlinear once the seed is within the room (see derive_seed()), so
 * convergence is normally a handful of iterations; the cap bounds the worst
 * case (e.g. a centroid seed near a far wall) rather than reflecting typical
 * behaviour. */
#define POS_GN_MAX_ITERS      20

/* Backtracking halvings tried per Gauss-Newton step before giving up on that
 * direction. 2^-8 of a metre-scale step is a few millimetres -- finer than
 * the convergence threshold below needs, and cheap: each halving is one more
 * residual evaluation over at most 4 anchors. */
#define POS_GN_MAX_HALVINGS   8

/* Converged when a step moves (x, y) by less than this. An order of
 * magnitude below the range-measurement precision the design assumes
 * (centimetres), so it bounds iteration count, not achievable accuracy. */
#define POS_GN_CONVERGE_M     1e-4f

/* Division-by-zero guard for dh/dx = dx/h, dh/dy = dy/h. Only reachable if an
 * anchor's horizontal position coincides with the estimate AND dz is
 * (mis-)configured as 0; real anchors are ceiling-mounted, so dz alone is
 * normally >1 m. 1 mm is far below any physically real slant range. */
#define POS_H_MIN_M           1e-3f

/* Degenerate-geometry threshold on det(J^T J). Each Jacobian row is a unit
 * direction cosine (bounded by 1 in each component), so for anchors spread
 * around the estimate det is O(0.1-1); it collapses toward 0 only when the
 * anchors and the estimate all line up (see the collinear-anchor note on
 * derive_seed() below). 1e-6 sits far under real geometry and far over
 * float32 rounding noise on an O(1) quantity. */
#define POS_GN_DET_EPS        1e-6f

/* Convergence is ultimately judged on the GRADIENT of the least-squares cost,
 * ||J^T r||, not on whether the line search managed a step.
 *
 * A stalled line search is not the same thing as a stationary point: the
 * Gauss-Newton direction can be poor while the gradient is still large, and
 * treating "no step improved the fit" as "converged" is what let this
 * function return the caller's own seed with valid = true. The gradient is
 * the definition of the thing being claimed, so it is what gets tested.
 *
 * J's rows are direction cosines, so the gradient carries the units of the
 * residual: 5 mm is tight against a range sigma of ~0.12 m and loose against
 * float32 noise on an O(1) sum. */
#define POS_GN_GRAD_EPS       5e-3f

/* Assumed per-range 1-sigma, metres. This is the single most load-bearing
 * unverified number in the file: the outlier threshold below is derived from
 * it. The design's static-soak campaign exists to measure it -- until then it
 * is the design doc's estimate, not data. */
#define POS_RANGE_SIGMA_M     0.12f

/* A residual beyond this many sigma is not explainable as measurement noise.
 * Used as an ABSOLUTE test on the largest residual rather than as a ratio
 * between the full-set and subset RMS: RMS is not comparable across different
 * anchor counts (a clean 3-anchor fit has 1 degree of freedom against the
 * 4-anchor fit's 2, so its RMS is ~0.82x the 4-anchor RMS for the same noise),
 * which is what made the original ratio test both fire on clean fixes and miss
 * real outliers. Measured behaviour is in the comment on pos_solve(). */
#define POS_OUTLIER_K_SIGMA   3.8f

/* Smallest twice-triangle-area (m^2) any three anchors may span before the
 * geometry is called degenerate.
 *
 * det(J^T J) does NOT catch collinear anchors, which is worth stating plainly
 * because it is the obvious thing to reach for: with the tag OFF the anchor
 * line the direction cosines still fan out, so the normal matrix stays well
 * conditioned while the problem has two mirror solutions of identical
 * residual. Measured over this room model, collinear anchors reach
 * det(J^T J) = 2.23 against a legitimate-geometry MINIMUM of 1.89 -- the two
 * are not separable by that metric at all, in either direction. Collinearity
 * is a property of the anchors, so it is tested on the anchors.
 *
 * 0.5 m^2 of twice-area is a triangle of 0.25 m^2 -- far thinner than any
 * real anchor placement, far thicker than survey error. */
#define POS_MIN_SPREAD_M2     0.5f

/*
 * Cold-start seed: the pre-2026-08-22 estimator's reference-anchor
 * linearization, applied to the *horizontally projected* range
 * r_h^2 = max(r^2 - dz^2, 0) rather than the raw slant range. This function
 * only has to land Gauss-Newton in the right basin, not be accurate, so
 * treating the problem as planar here (and correctly as 3D everywhere else)
 * is fine -- see derive_seed()'s centroid fallback for when even this linear
 * system is singular.
 *
 * Returns false when the reference-anchor subtraction is itself degenerate
 * (anchors collinear, or the reference anchor duplicated), which happens
 * before any Gauss-Newton iteration runs.
 */
static bool linear_seed(const struct pos_meas *m, size_t n, float *sx, float *sy)
{
    /* Componentwise magnitude: A's rows are 2*(coordinate difference), which
     * for a real room are O(1-20) m, so a meaningful det is O(1-400); this
     * catches the case where it collapses to (near) exactly 0 without ever
     * being close for a legitimate room-scale layout. */
    const float SEED_DET_EPS = 1e-3f;

    const float xk = m[0].x, yk = m[0].y;
    const float rk_h2 = fmaxf(m[0].range_m * m[0].range_m - m[0].dz * m[0].dz, 0.0f);
    const float ck = xk * xk + yk * yk;

    float A00 = 0.0f, A01 = 0.0f, A11 = 0.0f, b0 = 0.0f, b1 = 0.0f;

    for (size_t i = 1; i < n; i++) {
        const float xi = m[i].x, yi = m[i].y;
        const float ri_h2 = fmaxf(m[i].range_m * m[i].range_m - m[i].dz * m[i].dz, 0.0f);
        const float ci = xi * xi + yi * yi;

        const float a0 = 2.0f * (xk - xi);
        const float a1 = 2.0f * (yk - yi);
        const float b  = (ri_h2 - rk_h2) + (ck - ci);

        A00 += a0 * a0;
        A01 += a0 * a1;
        A11 += a1 * a1;
        b0  += a0 * b;
        b1  += a1 * b;
    }

    const float det = A00 * A11 - A01 * A01;
    if (fabsf(det) < SEED_DET_EPS) {
        return false;
    }

    *sx = ( A11 * b0 - A01 * b1) / det;
    *sy = (-A01 * b0 + A00 * b1) / det;
    return true;
}

/*
 * Seed used when the caller does not supply a previous fix. Falls back to
 * the anchor centroid when linear_seed()'s own normal equations are
 * singular.
 *
 * Note on collinear anchors: if every anchor shares one line (the degenerate
 * geometry pos_solve() must reject), the centroid also lies exactly on that
 * line. That is not a bug to work around -- it is what makes the rejection
 * deterministic: gn_solve()'s very first Jacobian, evaluated on the line,
 * has a zero column (dh/dy = (y-yi)/h = 0 for every anchor when y equals
 * every anchor's y), so det(J^T J) is exactly 0 and gn_solve() reports the
 * degeneracy on iteration 0 rather than silently converging to one arm of a
 * mirror ambiguity.
 */
static void derive_seed(const struct pos_meas *m, size_t n, float *sx, float *sy)
{
    if (linear_seed(m, n, sx, sy)) {
        return;
    }

    float cx = 0.0f, cy = 0.0f;
    for (size_t i = 0; i < n; i++) {
        cx += m[i].x;
        cy += m[i].y;
    }
    *sx = cx / (float)n;
    *sy = cy / (float)n;
}

/*
 * Twice the largest triangle area spanned by any three anchors -- zero when
 * they all lie on one line, and also zero when fewer than three are distinct,
 * so it covers the "any two anchors share coordinates" case pos_solver.h calls
 * out as well.
 */
static float anchor_spread(const struct pos_meas *m, size_t n)
{
    float best = 0.0f;

    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            for (size_t k = j + 1; k < n; k++) {
                const float cross = (m[j].x - m[i].x) * (m[k].y - m[i].y)
                                  - (m[j].y - m[i].y) * (m[k].x - m[i].x);
                const float a = fabsf(cross);

                if (a > best) {
                    best = a;
                }
            }
        }
    }
    return best;
}

/*
 * Largest residual over the set, LEVERAGE-CORRECTED, in metres.
 *
 * A plain residual badly understates a single bad range, because least
 * squares absorbs part of the error by moving the fix. The absorbed fraction
 * is the leverage h_ii = J_i (J^T J)^-1 J_i^T, and the leftover residual is
 * only (1 - h_ii) of the true error. With 4 anchors and 2 unknowns the
 * leverages sum to 2, so h_ii averages 0.5 and a 1 m NLOS bias shows up as
 * roughly 0.5 m of residual -- which is why a raw test at 3 sigma caught only
 * 72 % of 1 m biases and picked the wrong anchor in 13 % of cases.
 *
 * Dividing by sqrt(1 - h_ii) undoes exactly that shrinkage and makes the
 * residuals comparable to each other and to sigma, which is what the
 * threshold is expressed in. This is the standard studentized-residual
 * outlier test; it is affordable here only because the normal matrix is 2x2.
 */
static float max_abs_residual(const struct pos_meas *m, size_t n,
                              float x, float y)
{
    float jtj00 = 0.0f, jtj01 = 0.0f, jtj11 = 0.0f;
    float jx[POS_MAX_ANCHORS], jy[POS_MAX_ANCHORS], e[POS_MAX_ANCHORS];

    for (size_t i = 0; i < n; i++) {
        const float dx = x - m[i].x;
        const float dy = y - m[i].y;
        float h = sqrtf(dx * dx + dy * dy + m[i].dz * m[i].dz);

        if (h < POS_H_MIN_M) {
            h = POS_H_MIN_M;
        }
        jx[i] = dx / h;
        jy[i] = dy / h;
        e[i]  = h - m[i].range_m;

        jtj00 += jx[i] * jx[i];
        jtj01 += jx[i] * jy[i];
        jtj11 += jy[i] * jy[i];
    }

    const float det = jtj00 * jtj11 - jtj01 * jtj01;
    float worst = 0.0f;

    for (size_t i = 0; i < n; i++) {
        float scaled = fabsf(e[i]);

        if (fabsf(det) >= POS_GN_DET_EPS) {
            /* h_ii = J_i (J^T J)^-1 J_i^T, with the 2x2 inverse in closed
             * form. Clamped away from 1: a point of leverage 1 is fitted
             * exactly by construction, so its residual carries no information
             * and the division would blow up rather than say anything. */
            const float m00 =  jtj11 / det;
            const float m01 = -jtj01 / det;
            const float m11 =  jtj00 / det;
            float lev = jx[i] * (m00 * jx[i] + m01 * jy[i])
                      + jy[i] * (m01 * jx[i] + m11 * jy[i]);

            if (lev > 0.95f) { lev = 0.95f; }
            if (lev < 0.0f)  { lev = 0.0f;  }
            scaled = fabsf(e[i]) / sqrtf(1.0f - lev);
        }

        if (scaled > worst) {
            worst = scaled;
        }
    }
    return worst;
}

/*
 * Gauss-Newton on the 3D range model, from seed (x0, y0):
 *
 *   h_i(x,y) = sqrt((x-x_i)^2 + (y-y_i)^2 + dz_i^2)
 *   r_i      = h_i - range_i
 *   J_i      = [ (x-x_i)/h_i, (y-y_i)/h_i ]
 *   dp       = -(J^T J)^-1 J^T r
 *
 * Backtracking line search (accept the step only if it reduces the RMS
 * residual, else halve it) guards against plain Gauss-Newton overshooting
 * from a poor seed on this nonlinear problem -- the closed-form 2x2 solve
 * still gives the step direction, so this stays cheap and exact.
 *
 * Returns false on a degenerate normal matrix or a non-finite result.
 */
static bool gn_solve(const struct pos_meas *m, size_t n, float x0, float y0,
                      float *ox, float *oy)
{
    float x = x0, y = y0;

    for (int iter = 0; iter < POS_GN_MAX_ITERS; iter++) {
        float jtj00 = 0.0f, jtj01 = 0.0f, jtj11 = 0.0f;
        float jtr0 = 0.0f, jtr1 = 0.0f;

        for (size_t i = 0; i < n; i++) {
            const float dx = x - m[i].x;
            const float dy = y - m[i].y;
            const float dz = m[i].dz;

            float h = sqrtf(dx * dx + dy * dy + dz * dz);
            if (h < POS_H_MIN_M) {
                h = POS_H_MIN_M;
            }
            const float r  = h - m[i].range_m;
            const float jx = dx / h;
            const float jy = dy / h;

            jtj00 += jx * jx;
            jtj01 += jx * jy;
            jtj11 += jy * jy;
            jtr0  += jx * r;
            jtr1  += jy * r;
        }

        const float det = jtj00 * jtj11 - jtj01 * jtj01;
        if (fabsf(det) < POS_GN_DET_EPS) {
            return false;
        }

        const float dpx = -( jtj11 * jtr0 - jtj01 * jtr1) / det;
        const float dpy = -(-jtj01 * jtr0 + jtj00 * jtr1) / det;

        /* Stationarity is judged on the UNDAMPED Newton step, before the
         * line search touches it. Two reasons: a seed that is already the
         * solution produces dp ~ 0 and no improving step, which must read as
         * converged rather than as a stall; and testing the damped step
         * instead would let eight halvings of a genuine 2.5 cm step fall under
         * the threshold and "converge" short of a stationary point. */
        if (dpx * dpx + dpy * dpy
                < POS_GN_CONVERGE_M * POS_GN_CONVERGE_M) {
            break;
        }

        const float cost0 = pos_residual_rms(m, n, x, y);
        float step = 1.0f;
        float nx = x, ny = y;
        bool improved = false;

        for (int ls = 0; ls < POS_GN_MAX_HALVINGS; ls++) {
            nx = x + step * dpx;
            ny = y + step * dpy;
            const float cost1 = pos_residual_rms(m, n, nx, ny);
            if (isfinite(cost1) && cost1 < cost0) {
                improved = true;
                break;
            }
            step *= 0.5f;
        }

        if (!improved) {
            /* Every halving made the fit worse (or non-finite). On the FIRST
             * iteration that means no step was ever accepted, so the "answer"
             * would be the seed handed in by the caller -- reporting success
             * there is precisely what pos_solver.h forbids, and it is not
             * hypothetical: over 200k adversarial cases this path returned the
             * seed verbatim with valid = true and a residual up to 113 m.
             * Whether this point is nonetheless a usable answer is decided
             * below by the gradient, not here: a stall near a minimum is
             * fine, a stall anywhere else is not. */
            break;
        }

        x = nx;
        y = ny;
    }

    if (!isfinite(x) || !isfinite(y)) {
        return false;
    }

    /* Final gate: is this actually a stationary point? Covers both exits that
     * are not an explicit convergence test -- the stalled line search and the
     * exhausted iteration budget -- with one criterion that means what the
     * contract says. A large residual here is fine and is the caller's signal
     * that the ranges disagree; a large GRADIENT means the ranges were never
     * fitted at all. */
    {
        float g0 = 0.0f, g1 = 0.0f;

        for (size_t i = 0; i < n; i++) {
            const float dx = x - m[i].x;
            const float dy = y - m[i].y;
            float h = sqrtf(dx * dx + dy * dy + m[i].dz * m[i].dz);

            if (h < POS_H_MIN_M) {
                h = POS_H_MIN_M;
            }
            const float r = h - m[i].range_m;

            g0 += (dx / h) * r;
            g1 += (dy / h) * r;
        }
        if (!isfinite(g0) || !isfinite(g1) ||
            sqrtf(g0 * g0 + g1 * g1) > POS_GN_GRAD_EPS) {
            return false;
        }
    }

    *ox = x;
    *oy = y;
    return true;
}

bool pos_solve(const struct pos_meas *m, size_t n, const float *seed_xy,
               struct pos_result *out)
{
    out->valid       = false;
    out->residual_m  = 0.0f;
    out->n_used      = 0;
    out->dropped_idx = POS_NO_DROP;
    out->x = 0.0f;
    out->y = 0.0f;

    if (n < 3 || n > POS_MAX_ANCHORS) {
        return false;
    }

    /* Ranges arrive over the air; a corrupted or missing frame can produce
     * NaN/Inf rather than merely a wrong number, and NaN compares false
     * against everything -- including the degenerate-geometry det checks
     * above -- so it must be screened before any arithmetic runs on it. */
    for (size_t i = 0; i < n; i++) {
        if (!isfinite(m[i].x) || !isfinite(m[i].y) || !isfinite(m[i].dz) ||
            !isfinite(m[i].range_m) || m[i].range_m < 0.0f) {
            return false;
        }
    }
    if (seed_xy != NULL && (!isfinite(seed_xy[0]) || !isfinite(seed_xy[1]))) {
        return false;
    }

    /* Degenerate anchor geometry, tested on the anchors themselves so the
     * verdict does not depend on the seed. It used to be caught only as a
     * side effect of the cold-start path: derive_seed() happens to land on
     * the anchor line for collinear anchors, where the Jacobian does collapse.
     * With a caller seed -- which is now the normal case, since the runner
     * seeds every solve from the filter -- Gauss-Newton instead converges
     * happily to whichever of the two mirror solutions the seed is nearer,
     * and returned valid = true for a position that is a reflection of the
     * truth. */
    if (anchor_spread(m, n) < POS_MIN_SPREAD_M2) {
        return false;
    }

    float sx, sy;
    if (seed_xy != NULL) {
        sx = seed_xy[0];
        sy = seed_xy[1];
    } else {
        derive_seed(m, n, &sx, &sy);
    }

    float fx, fy;
    if (!gn_solve(m, n, sx, sy, &fx, &fy)) {
        return false;
    }
    const float full_res = pos_residual_rms(m, n, fx, fy);

    out->x           = fx;
    out->y           = fy;
    out->residual_m  = full_res;
    out->n_used      = (uint8_t)n;
    out->dropped_idx = POS_NO_DROP;

    /* Single-outlier rejection needs the one spare anchor a full set
     * provides -- n == 3 has no redundancy to spend on it, per pos_solver.h.
     *
     * The test is absolute, on the largest residual, in units of the assumed
     * range sigma: accept the full set when every anchor agrees to within
     * POS_OUTLIER_K_SIGMA, otherwise look for the single anchor whose removal
     * brings the rest back inside that bound. An anchor is dropped only when
     * doing so actually EXPLAINS the inconsistency -- if no subset comes back
     * clean, the geometry or the noise model is wrong, not one anchor, and
     * dropping one would just discard information.
     *
     * Measured, 200k clean fixes and 50k single-NLOS fixes, 8x6 m room with
     * corner anchors, sigma = 0.12 m, correct-anchor rate in brackets:
     *
     *                          clean false-drop   +0.5 m NLOS   +1.0 m NLOS
     *   old RMS-ratio test      3.81 %            67 % (47 %)   99 % (79 %)
     *   this, leverage-corrected 0.057 %          21 % (16 %)   95 % (76 %)
     *
     * i.e. a 67x cut in false drops for 4 points of 1 m sensitivity. The old
     * test was simultaneously too eager on clean data and too blunt on real
     * outliers because it compared RMS across different anchor counts.
     *
     * Being conservative is the right bias here: the EKF downstream gates
     * every range at 3 sigma of its own innovation covariance, so a missed
     * outlier gets a second chance, while a wrongly dropped anchor is
     * information thrown away that nothing downstream can recover.
     *
     * A false drop is expected at roughly 4 * P(|N(0,1)| > K) once the
     * residuals are leverage-corrected -- that is where 3.8 comes from, and it
     * is why the constant is meaningful rather than tuned. */
    const float accept = POS_OUTLIER_K_SIGMA * POS_RANGE_SIGMA_M;

    if (n == POS_MAX_ANCHORS &&
        max_abs_residual(m, n, fx, fy) > accept) {
        float best_worst = accept;
        int   best_drop  = -1;
        float best_x = fx, best_y = fy;

        for (size_t drop = 0; drop < n; drop++) {
            struct pos_meas sub[POS_MAX_ANCHORS - 1];
            size_t k = 0;

            for (size_t j = 0; j < n; j++) {
                if (j != drop) {
                    sub[k++] = m[j];
                }
            }

            /* Seed every subset from the same point as the full solve (the
             * previous fix, or its cold-start derivation): the ambiguity is
             * which *range* is bad, not where the tag is. */
            float subx, suby;
            if (!gn_solve(sub, n - 1, sx, sy, &subx, &suby)) {
                continue;
            }

            const float subworst = max_abs_residual(sub, n - 1, subx, suby);

            if (subworst < best_worst) {
                best_worst = subworst;
                best_drop  = (int)drop;
                best_x     = subx;
                best_y     = suby;
            }
        }

        if (best_drop >= 0) {
            out->x           = best_x;
            out->y           = best_y;
            out->residual_m  = pos_residual_rms(m, n, best_x, best_y);
            out->n_used      = (uint8_t)(n - 1);
            out->dropped_idx = (uint8_t)best_drop;

            /* Report the residual over the anchors actually USED, per
             * pos_solver.h -- recomputed on the subset, not on all n. */
            struct pos_meas sub[POS_MAX_ANCHORS - 1];
            size_t k = 0;
            for (size_t j = 0; j < n; j++) {
                if (j != (size_t)best_drop) {
                    sub[k++] = m[j];
                }
            }
            out->residual_m = pos_residual_rms(sub, n - 1, best_x, best_y);
        }
    }

    if (!isfinite(out->x) || !isfinite(out->y) || !isfinite(out->residual_m)) {
        out->valid = false;
        return false;
    }

    out->valid = true;
    return true;
}

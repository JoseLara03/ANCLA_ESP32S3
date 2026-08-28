#ifndef TDOA_SOLVE_H
#define TDOA_SOLVE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "pos_solver.h"     /* struct pos_result, POS_MAX_ANCHORS */

/* Metres of path-length difference per DTU: c / (499.2 MHz x 128).
 * 299792458 x 1.565004e-11 = 4.691764e-3, which rounds to the value below to
 * 0.000% -- verified, not assumed. One DTU is 4.69 mm, and 1 ns of sync error
 * (64 DTU) is ~30 cm, which is exactly how the Phase 2 gate
 * (docs/anchor-sync-measurement.md) translates into position error. */
#define TDOA_M_PER_DTU  4.69175e-3f

/* 2D needs 2 independent range differences, and n anchors give n-1 -- see the
 * n_used contract below for what that costs the residual at n == 3. */
#define TDOA_MIN_ANCHORS  3u

/*
 * One TDoA observation: where the anchor is, and when it heard the tag's
 * BLINK, already converted into the common (master) time base by
 * sync_model_to_master() -- t_dtu is NOT a raw per-anchor local timestamp.
 * `dz` follows the same convention as struct pos_meas: anchor z minus tag z,
 * in metres, carried inside the model rather than projected out beforehand.
 */
struct tdoa_meas {
	float   x;
	float   y;
	float   dz;
	int64_t t_dtu;
};

/*
 * Hyperbolic 2D multilateration by Gauss-Newton over range DIFFERENCES, with
 * anchor 0 as the reference:
 *
 *   r_i(x,y)  = sqrt((x-x_i)^2 + (y-y_i)^2 + dz_i^2)
 *   f_i(x,y)  = (r_i(x,y) - r_0(x,y)) - d_i
 *   d_i       = (t_i - t_0) * TDOA_M_PER_DTU
 *   df_i/dx   = (x-x_i)/r_i - (x-x_0)/r_0
 *   df_i/dy   = (y-y_i)/r_i - (y-y_0)/r_0
 *
 * for every anchor i >= 1. Same skeleton as pos_solve(): the 2x2 normal
 * equations are inverted in closed form, with the determinant tested
 * explicitly for degenerate (e.g. collinear) anchor geometry rather than
 * trusting a near-zero divide.
 *
 * `seed_xy` is an optional 2-element {x, y} starting point; NULL uses the
 * anchor centroid.
 *
 * DEGREES OF FREEDOM, AND WHAT out->residual_m DOES NOT TELL YOU: n anchors
 * give n-1 equations against 2 unknowns (x, y). At n == TDOA_MIN_ANCHORS (3)
 * that is exactly determined -- 2 equations, 2 unknowns, zero spare -- so
 * Gauss-Newton re-fits ANY set of timestamps exactly and out->residual_m
 * comes back at (numerically) zero *by construction*, regardless of how
 * wrong the input measurements are. This is the identical situation
 * CLAUDE.md documents for the anchor survey's isostatic 4-anchor 3D case and
 * its 3-anchor 2D case: a residual of zero there means "nothing contradicted
 * the input", not "the fit is good". Only at n >= 4 (>= 3 equations, >= 1
 * spare) does out->residual_m carry any information at all, and even then it
 * is a diff-residual (residual on RANGE DIFFERENCES), not a range residual --
 * see the comment on diff_residual() in tdoa_solve.c for why
 * pos_residual_rms() must not be reused here. A caller MUST check
 * out->n_used before treating out->residual_m as a quality signal: at
 * n_used == TDOA_MIN_ANCHORS it is not one.
 *
 * KNOWN LIMITATION -- "converged" is not always "correct": convergence here
 * is judged purely by step size (the iteration stops once |dx|,|dy| fall
 * under CONV_EPS_M), unlike pos_solve(), which additionally rejects a
 * converged point whose cost-function GRADIENT is still large -- see
 * gn_solve()'s final gate in pos_solver.c. A hyperbolic TDoA system
 * genuinely has a second branch (the mirror solution on the far side of the
 * reference anchor's hyperbola), and for a tag well outside the anchor hull
 * Gauss-Newton can settle there and still report valid = true. At n == 4
 * (one spare equation) out->residual_m is too weak a signal to reliably
 * discriminate the wrong branch from the right one. This is deliberately
 * NOT fixed by adding a gradient gate in this revision -- that would be a
 * change to the convergence criterion itself, which needs its own
 * validation rather than being folded into a review fix. A caller feeding
 * this solver a bad or unbounded seed should treat a "valid" result as
 * provisional until it is corroborated (e.g. against a previous fix, or
 * plausibility-checked against the deployment's known geometry).
 *
 * Returns false (and sets out->valid = false) if n < TDOA_MIN_ANCHORS, if
 * n > POS_MAX_ANCHORS, if the geometry is degenerate (singular normal
 * matrix, e.g. collinear anchors, or the estimate exactly on anchor 0), or if
 * the iteration fails to converge within its budget.
 */
bool tdoa_solve(const struct tdoa_meas *m, size_t n, const float *seed_xy,
		struct pos_result *out);

#endif /* TDOA_SOLVE_H */

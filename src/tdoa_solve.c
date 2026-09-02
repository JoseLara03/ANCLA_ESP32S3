#include "tdoa_solve.h"
#include <math.h>

/*
 * Constants copied from src/pos_solver.c's gn_solve() rather than re-chosen:
 * POS_GN_MAX_ITERS, POS_GN_CONVERGE_M and POS_GN_DET_EPS are already proven
 * on hardware for the same 2x2-normal-equations shape, and picking different
 * numbers here would introduce a subtle behavioural divergence between the
 * two solvers for no reason.
 */
#define MAX_ITER     20
#define CONV_EPS_M   1e-4f
#define DET_EPS      1e-6f

/* Backtracking halvings tried per Gauss-Newton step before giving up on that
 * direction. Same value and same reasoning as POS_GN_MAX_HALVINGS: each
 * halving is one more diff_residual() evaluation over n-1 equations, and
 * 2^-8 of a metre-scale step is finer than CONV_EPS_M needs. */
#define MAX_HALVINGS 8

/*
 * Stationarity threshold on the gradient of the least-squares cost,
 * ||J^T f||. This is what makes "converged" mean "this is a stationary
 * point" rather than "the line search ran out of ideas" -- see the block
 * comment on the final gate in tdoa_solve() below for why that distinction
 * is load-bearing HERE in particular.
 *
 * DERIVED, NOT COPIED from pos_solver.c's POS_GN_GRAD_EPS (5e-3), and the
 * difference matters because the threshold carries units:
 *
 *   - pos_solver's Jacobian rows are direction cosines, bounded by 1 in each
 *     component; ours are DIFFERENCES of two direction cosines
 *     ((x-x_i)/r_i - (x-x_0)/r_0), bounded by 2. For the same displacement
 *     from a stationary point the gradient here is about twice as large.
 *   - pos_solver's residual is a RANGE residual against a range sigma of
 *     ~0.12 m (POS_RANGE_SIGMA_M); ours is a range-DIFFERENCE residual
 *     against ~0.6 m (pos_ekf.h's r_tdoa, itself derived from the Fase 2
 *     sync jitter via sqrt(2)).
 *
 * So: 5e-3 / 0.12 ~= 4% of sigma, kept as the fraction; 0.04 * 0.6 * 2 =
 * 0.048, rounded to 5e-2. Tight against a 0.6 m measurement sigma and loose
 * against float32 noise on an O(1) sum, which is the same pair of bounds
 * pos_solver.c states for its own value.
 *
 * Copying 5e-3 verbatim would have imported a number dimensioned for a
 * different measurement model -- roughly 10x too tight here, which would
 * reject good fits as non-stationary.
 */
#define GRAD_EPS     5e-2f

/* Division-by-zero guard on r_i = sqrt(...). Same VALUE as POS_H_MIN_M in
 * pos_solver.c, but a different BEHAVIOUR: pos_solver.c clamps h to that
 * floor and continues (max_abs_residual()/gn_solve()), whereas here the
 * solve is refused outright rather than clamped. Only reachable if the
 * estimate lands exactly on an anchor's horizontal position AND dz is
 * (mis-)configured as 0. */
#define R_MIN_M      1e-3f

/* Slant distance from (x, y) to anchor a's horizontal position, with dz
 * carried inside the model rather than projected out beforehand -- same
 * convention as struct pos_meas, see pos_solver.h. */
static float slant(const struct tdoa_meas *a, float x, float y)
{
	float dx = x - a->x, dy = y - a->y;

	return sqrtf(dx * dx + dy * dy + a->dz * a->dz);
}

/*
 * RMS of f_i over the n-1 range-DIFFERENCE equations, in metres, at the
 * solved (x, y).
 *
 * Deliberately does NOT call pos_residual_rms(): that function measures
 * disagreement against MEASURED RANGES, and TDoA never observes a range --
 * only differences of arrival time. A perfect TDoA fix has zero residual on
 * these difference equations and an undefined (never computed) residual on
 * absolute range, because absolute range was never part of the measurement
 * model. Reusing pos_residual_rms() here would silently compare against a
 * quantity that does not exist for this sensor.
 *
 * TRAP 1, applied here as well as in the solve loop below: (t_i - t_0) is
 * subtracted in int64_t BEFORE any float conversion. These are raw DW3220
 * device-time values that can run up to ~2^40, and a float32's 24-bit
 * mantissa only resolves ~65536 DTU (307 m) at that magnitude -- converting
 * each timestamp to float first and subtracting would silently destroy the
 * measurement while still producing a plausible-looking number. The
 * DIFFERENCE itself is only ever a few thousand DTU, which float32 carries
 * exactly, so integer-subtract-then-convert is both correct and lossless.
 */
static float diff_residual(const struct tdoa_meas *m, size_t n, float x, float y)
{
	float r0 = slant(&m[0], x, y);
	float acc = 0.0f;

	for (size_t i = 1; i < n; i++) {
		float d_meas = (float)(m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU;
		float f = (slant(&m[i], x, y) - r0) - d_meas;

		acc += f * f;
	}
	return sqrtf(acc / (float)(n - 1));
}

/*
 * Squared norm of the least-squares gradient, ||J^T f||^2, at (x, y).
 *
 * Kept separate from the iteration because the final stationarity gate has to
 * evaluate it at the point the loop actually STOPPED at, which in general is
 * not a point the loop ever built a Jacobian for -- the last accepted step
 * moves (x, y) after the Jacobian that produced it was computed.
 *
 * Returns -1.0f when the geometry is degenerate at (x, y) (some r_i under
 * R_MIN_M); the caller must treat that as a refusal, not as a small gradient.
 * TRAP 1 applies here too: the int64_t subtraction precedes the float
 * conversion, for the reason spelled out on diff_residual() above.
 */
static float grad_norm2(const struct tdoa_meas *m, size_t n, float x, float y)
{
	float r0 = slant(&m[0], x, y);
	float g0 = 0.0f, g1 = 0.0f;

	if (r0 < R_MIN_M) {
		return -1.0f;
	}

	for (size_t i = 1; i < n; i++) {
		float ri = slant(&m[i], x, y);

		if (ri < R_MIN_M) {
			return -1.0f;
		}

		float d_meas = (float)(m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU;
		float f  = (ri - r0) - d_meas;
		float gx = (x - m[i].x) / ri - (x - m[0].x) / r0;
		float gy = (y - m[i].y) / ri - (y - m[0].y) / r0;

		g0 += gx * f;
		g1 += gy * f;
	}
	return g0 * g0 + g1 * g1;
}

bool tdoa_solve(const struct tdoa_meas *m, size_t n, const float *seed_xy,
		struct pos_result *out)
{
	if (!out) {
		return false;
	}

	out->valid = false;
	out->x = 0.0f;
	out->y = 0.0f;
	out->residual_m = 0.0f;
	out->n_used = 0;
	out->dropped_idx = POS_NO_DROP;

	if (!m || n < TDOA_MIN_ANCHORS || n > POS_MAX_ANCHORS) {
		return false;
	}

	float x, y;

	if (seed_xy) {
		x = seed_xy[0];
		y = seed_xy[1];
	} else {
		/* Anchor centroid: interior to the convex hull, which is where the
		 * TDoA geometry is well conditioned. Outside it the hyperbolas
		 * flatten out and the iteration can walk away. */
		x = 0.0f;
		y = 0.0f;
		for (size_t i = 0; i < n; i++) {
			x += m[i].x;
			y += m[i].y;
		}
		x /= (float)n;
		y /= (float)n;
	}

	for (int it = 0; it < MAX_ITER; it++) {
		float jtj00 = 0.0f, jtj01 = 0.0f, jtj11 = 0.0f;
		float jtf0 = 0.0f, jtf1 = 0.0f;
		float r0 = slant(&m[0], x, y);

		/* dz != 0 normally keeps r0 well away from zero, but a deployment
		 * with dz == 0 and the tag directly under the reference anchor can
		 * still reach this. */
		if (r0 < R_MIN_M) {
			return false;
		}

		for (size_t i = 1; i < n; i++) {
			float ri = slant(&m[i], x, y);

			if (ri < R_MIN_M) {
				return false;
			}

			/* TRAP 1: int64_t subtraction first, float conversion after --
			 * see the comment on diff_residual() above for why the reverse
			 * order silently destroys the measurement. */
			float d_meas = (float)(m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU;
			float f  = (ri - r0) - d_meas;
			float gx = (x - m[i].x) / ri - (x - m[0].x) / r0;
			float gy = (y - m[i].y) / ri - (y - m[0].y) / r0;

			jtj00 += gx * gx;
			jtj01 += gx * gy;
			jtj11 += gy * gy;
			jtf0  += gx * f;
			jtf1  += gy * f;
		}

		/*
		 * 2x2 normal equations, inverted in closed form -- same pattern as
		 * pos_solver.c, and for the same reason: cheaper than a generic
		 * inverse, and it lets the determinant be tested explicitly, which
		 * is how degenerate geometry (e.g. collinear anchors) is detected
		 * instead of returning a fabricated point.
		 */
		float det = jtj00 * jtj11 - jtj01 * jtj01;

		if (fabsf(det) < DET_EPS) {
			return false;
		}

		float dx = -( jtj11 * jtf0 - jtj01 * jtf1) / det;
		float dy = -(-jtj01 * jtf0 + jtj00 * jtf1) / det;

		/* Stationarity is judged on the UNDAMPED Newton step, before the
		 * line search below touches it -- same reasoning pos_solver.c's
		 * gn_solve() records, and both halves of it apply here. A seed
		 * that IS already the solution produces dp ~ 0 and no improving
		 * step, which has to read as converged rather than as a stall;
		 * and testing the DAMPED step instead would let eight halvings of
		 * a legitimate 2.5 cm step fall under the threshold and "converge"
		 * short of a stationary point. */
		if (fabsf(dx) < CONV_EPS_M && fabsf(dy) < CONV_EPS_M) {
			break;
		}

		/*
		 * Backtracking line search. The raw Gauss-Newton step is a
		 * direction, not a promise: on the hyperbolic cost surface TDoA
		 * produces it can overshoot badly, and accepting it unconditionally
		 * is how the iteration walks away from a solution it was next to.
		 * diff_residual() is the cost, and it already exists in this file --
		 * it was written for out->residual_m and needed no new arithmetic.
		 */
		float cost0 = diff_residual(m, n, x, y);
		float step = 1.0f;
		float nx = x, ny = y;
		bool improved = false;

		for (int ls = 0; ls < MAX_HALVINGS; ls++) {
			nx = x + step * dx;
			ny = y + step * dy;

			float cost1 = diff_residual(m, n, nx, ny);

			if (isfinite(cost1) && cost1 < cost0) {
				improved = true;
				break;
			}
			step *= 0.5f;
		}

		if (!improved) {
			/* Every halving made the fit worse, or non-finite. On the
			 * FIRST iteration that means no step was ever accepted, so
			 * (x, y) is still the caller's seed -- see the final gate
			 * below for why returning it as a result is the specific
			 * failure this function had to stop being able to commit.
			 * Whether this point is nonetheless a usable answer is
			 * decided there, on the gradient, and not here. */
			break;
		}

		x = nx;
		y = ny;
	}

	if (!isfinite(x) || !isfinite(y)) {
		return false;
	}

	/*
	 * Final gate: is this actually a stationary point?
	 *
	 * Covers the three exits that are not an explicit convergence test --
	 * the stalled line search, the exhausted iteration budget, and a seed
	 * that was never improved on -- with one criterion that means what the
	 * contract says. A large RESIDUAL here is fine and is the caller's
	 * signal that the timestamps disagree; a large GRADIENT means they were
	 * never fitted at all.
	 *
	 * WHY THIS MATTERS MORE HERE THAN IN pos_solver.c, which is where the
	 * criterion comes from: pos_solver.c records that without this gate it
	 * "returned the seed verbatim with valid = true and a residual up to
	 * 113 m" across 200k adversarial cases. In this solver the seed is not
	 * an arbitrary starting guess -- src/tdoa_gw.c passes the tag's LAST
	 * PUBLISHED POSITION on every fix. So a stall here does not return a
	 * random point; it returns the previous answer, reporting success,
	 * which is the stale-republish defect found on hardware on 2026-09-02
	 * reached by a second route. And the arithmetic that caught that one
	 * (fixes == seeded + filtered + reseed, read off `blink stats`) is
	 * BLIND to this route: the fix enters through the seeding path and
	 * tallies as n_seeded, so the totals still balance. This gate is the
	 * only thing standing in front of it.
	 */
	{
		float g2 = grad_norm2(m, n, x, y);

		if (g2 < 0.0f || g2 > GRAD_EPS * GRAD_EPS) {
			return false;
		}

		{
			out->x = x;
			out->y = y;
			out->residual_m = diff_residual(m, n, x, y);
			out->n_used = (uint8_t)n;
			/* TRAP 2, see the contract on tdoa_solve() in tdoa_solve.h:
			 * out->residual_m is not a quality signal at n_used ==
			 * TDOA_MIN_ANCHORS (2 equations, 2 unknowns, zero spare -- the
			 * fit is exact by construction). It only becomes informative
			 * from n_used == 4, where there is one spare equation. That is
			 * a caller-side rule, spelled out in the header rather than
			 * hidden in this comment, because nothing in this struct can
			 * flag it on its own without touching pos_result -- which this
			 * task must not modify.
			 *
			 * Outlier rejection is deliberately NOT done here. At n == 3
			 * there is no redundancy to reject against, and at n == 4
			 * pos_solve()'s subset-rejection criterion is defined on RANGE
			 * residuals, not range-DIFFERENCE residuals, and would need to
			 * be re-derived before it means anything for this model. Future
			 * work, not an oversight. */
			out->dropped_idx = POS_NO_DROP;
			out->valid = true;
			return true;
		}
	}
}

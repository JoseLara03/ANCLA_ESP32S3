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

		x += dx;
		y += dy;

		if (fabsf(dx) < CONV_EPS_M && fabsf(dy) < CONV_EPS_M) {
			if (!isfinite(x) || !isfinite(y)) {
				return false;
			}
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
	return false;   /* did not converge */
}

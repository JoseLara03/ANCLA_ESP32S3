#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "tdoa_solve.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* Anchors at the corners of a 10 m square, tag 1.5 m below them. */
static const float AX[4] = { 0.0f, 10.0f, 10.0f,  0.0f };
static const float AY[4] = { 0.0f,  0.0f, 10.0f, 10.0f };
#define DZ 1.5f

/* Builds EXACT observations for a true position. Times are integers in DTU,
 * as they arrive from hardware, with an arbitrary common offset the solver
 * must cancel -- that is the entire point of TDoA. */
static void make(struct tdoa_meas *m, size_t n, float tx, float ty,
		 int64_t common_offset)
{
	for (size_t i = 0; i < n; i++) {
		float dx = tx - AX[i], dy = ty - AY[i];
		float r = sqrtf(dx * dx + dy * dy + DZ * DZ);

		m[i].x = AX[i];
		m[i].y = AY[i];
		m[i].dz = DZ;
		m[i].t_dtu = common_offset + (int64_t)llroundf(r / TDOA_M_PER_DTU);
	}
}

static void test_recovers_a_known_position(void)
{
	const float pts[][2] = { {5.0f, 5.0f}, {2.0f, 3.0f}, {7.5f, 8.0f},
				 {1.0f, 1.0f}, {9.0f, 5.0f} };

	for (unsigned int k = 0; k < 5; k++) {
		struct tdoa_meas m[4];
		struct pos_result r;

		make(m, 4, pts[k][0], pts[k][1], 0x1234567890LL);
		CHECK(tdoa_solve(m, 4, NULL, &r));
		CHECK(r.valid);
		CHECK(fabsf(r.x - pts[k][0]) < 0.05f);
		CHECK(fabsf(r.y - pts[k][1]) < 0.05f);
		printf("  (%.1f,%.1f) -> (%.3f,%.3f) res %.4f\n",
		       (double)pts[k][0], (double)pts[k][1],
		       (double)r.x, (double)r.y, (double)r.residual_m);
	}
}

/* The common offset is exactly what TDoA cancels by construction: the tag
 * shares no clock with anyone. Changing it must not move the result by a mm. */
static void test_common_offset_cancels(void)
{
	struct tdoa_meas a[4], b[4];
	struct pos_result ra, rb;

	make(a, 4, 4.0f, 6.0f, 0);
	make(b, 4, 4.0f, 6.0f, 0x7FFFFFFFFLL);
	CHECK(tdoa_solve(a, 4, NULL, &ra));
	CHECK(tdoa_solve(b, 4, NULL, &rb));
	CHECK(ra.valid && rb.valid);
	CHECK(fabsf(ra.x - rb.x) < 1e-3f);
	CHECK(fabsf(ra.y - rb.y) < 1e-3f);
}

/* Three anchors is the minimum: 2 differences, 2 unknowns. */
static void test_three_anchors_is_enough(void)
{
	struct tdoa_meas m[3];
	struct pos_result r;

	make(m, 3, 4.0f, 4.0f, 99);
	CHECK(tdoa_solve(m, 3, NULL, &r));
	CHECK(r.valid);
	CHECK(fabsf(r.x - 4.0f) < 0.10f);
	CHECK(fabsf(r.y - 4.0f) < 0.10f);
}

/* With exactly 3 anchors the system is exactly determined -- 2 equations, 2
 * unknowns, zero spare -- so Gauss-Newton re-fits ANY timestamps exactly and
 * residual_m comes back at (numerically) zero regardless of whether the
 * input made physical sense. This is TRAP 2 from the task brief, and it is
 * the identical shape as the isostatic 4-anchor 3D / 3-anchor 2D case
 * CLAUDE.md documents for the anchor survey. A caller must not read
 * n_used == 3's residual_m as evidence of a good fit -- see tdoa_solve.h. */
static void test_three_anchor_residual_is_structurally_zero(void)
{
	struct tdoa_meas m[3];
	struct pos_result r;

	/* Start from a real, consistent 3-anchor geometry, then blow one
	 * timestamp off by an amount no real sync error would produce (2 m of
	 * path error, ~400x the noise floor tested below). With no spare
	 * equation the solver still re-fits it EXACTLY -- to some other (x, y),
	 * not the true (4, 4) -- and residual_m still reads ~0. */
	make(m, 3, 4.0f, 4.0f, 99);
	m[1].t_dtu += (int64_t)llroundf(2.0f / TDOA_M_PER_DTU);

	CHECK(tdoa_solve(m, 3, NULL, &r));
	CHECK(r.valid);
	CHECK(r.n_used == 3);
	printf("  n_used=3, true (4,4), 2 m timestamp error -> (%.3f,%.3f) "
	       "residual_m=%.6f (structurally ~0, proves nothing)\n",
	       (double)r.x, (double)r.y, (double)r.residual_m);
	CHECK(r.residual_m < 1e-3f);
	/* The fix moved well away from the true position, yet the residual did
	 * not notice: that gap is exactly what the header contract warns about. */
	CHECK(fabsf(r.x - 4.0f) > 0.5f || fabsf(r.y - 4.0f) > 0.5f);
}

static void test_too_few_anchors_refused(void)
{
	struct tdoa_meas m[4];
	struct pos_result r;

	make(m, 4, 5.0f, 5.0f, 0);
	CHECK(!tdoa_solve(m, 2, NULL, &r));
	CHECK(!r.valid);
	CHECK(!tdoa_solve(m, 0, NULL, &r));
	CHECK(!tdoa_solve(NULL, 4, NULL, &r));
}

/* Collinear anchors leave the normal matrix singular. Must be reported, not
 * returned as a made-up point -- same as pos_solve. */
static void test_collinear_anchors_refused(void)
{
	struct tdoa_meas m[3];
	struct pos_result r;

	for (int i = 0; i < 3; i++) {
		m[i].x = (float)i * 5.0f;
		m[i].y = 0.0f;
		m[i].dz = DZ;
		m[i].t_dtu = 1000 + i * 100;
	}
	CHECK(!tdoa_solve(m, 3, NULL, &r));
	CHECK(!r.valid);
}

/* Anchor 0 is the reference; r_0 appears in every equation's Jacobian as a
 * denominator. The tag directly under anchor 0's horizontal position, with
 * dz == 0, makes r_0 -> 0 and must be refused rather than dividing through a
 * near-zero term. */
static void test_reference_anchor_singularity_refused(void)
{
	struct tdoa_meas m[4];
	struct pos_result r;
	float seed[2] = { 0.0f, 0.0f };

	make(m, 4, 5.0f, 5.0f, 0);
	m[0].dz = 0.0f;      /* anchor 0 flush with the tag plane */
	m[0].x = 0.0f; m[0].y = 0.0f;

	/* Seed the iteration exactly on anchor 0's horizontal position so the
	 * very first r_0 evaluation is the degenerate one. */
	CHECK(!tdoa_solve(m, 4, seed, &r));
	CHECK(!r.valid);
}

/* seed_xy may be NULL -- the solver must derive its own seed (the anchor
 * centroid) and still converge to the same answer a supplied seed would. */
static void test_null_seed_converges(void)
{
	struct tdoa_meas m[4];
	struct pos_result r_null, r_seeded;
	float seed[2] = { 5.0f, 5.0f };

	make(m, 4, 6.0f, 4.0f, 42);
	CHECK(tdoa_solve(m, 4, NULL, &r_null));
	CHECK(tdoa_solve(m, 4, seed, &r_seeded));
	CHECK(r_null.valid && r_seeded.valid);
	CHECK(fabsf(r_null.x - r_seeded.x) < 1e-2f);
	CHECK(fabsf(r_null.y - r_seeded.y) < 1e-2f);
	CHECK(fabsf(r_null.x - 6.0f) < 0.05f);
	CHECK(fabsf(r_null.y - 4.0f) < 0.05f);
}

/* A seed far outside any sane basin (and far from the anchors) must not make
 * the solver report success at a point the ranges do not support -- either
 * it converges to the true answer anyway, or it honestly fails. It must
 * never silently return the seed itself. */
static void test_bad_seed_does_not_fabricate_a_result(void)
{
	struct tdoa_meas m[4];
	struct pos_result r;
	float seed[2] = { 5000.0f, -5000.0f };

	make(m, 4, 5.0f, 5.0f, 0);
	if (tdoa_solve(m, 4, seed, &r)) {
		CHECK(r.valid);
		CHECK(fabsf(r.x - 5.0f) < 0.5f);
		CHECK(fabsf(r.y - 5.0f) < 0.5f);
	} else {
		CHECK(!r.valid);
	}
}

/* One DTU is 4.69 mm of path difference: the resolution sync has to sustain,
 * and the constant that converts the Phase 2 gate into position error. */
static void test_dtu_scale(void)
{
	CHECK(TDOA_M_PER_DTU > 0.00469f && TDOA_M_PER_DTU < 0.00470f);
	/* 1 ns of sync error = 64 DTU = ~30 cm. */
	CHECK(fabsf(64.0f * TDOA_M_PER_DTU - 0.300f) < 0.01f);
}

/* Sync noise at gate level: 0.5 ns = 32 DTU on each timestamp. The resulting
 * position error is what decides whether the product meets 10-30 cm, and this
 * test PRINTS it rather than asserting it -- the number is the deliverable.
 *
 * `solved` is counted and asserted separately from `worst`, deliberately: a
 * solver that fails every trial leaves `worst` at its initial 0.0f, which
 * looks exactly like a perfect result on the printed line and would let this
 * test pass vacuously while reporting a fabricated "worst error 0.000 m" --
 * this is not hypothetical, see the negative-control transcript in
 * task-3-report.md. 190/200 is not a tuned tolerance for failure rate, only a
 * floor that catches "the solver stopped solving"; at this noise level 200
 * trials solve essentially every time in practice (see the printed count). */
static void test_error_under_gate_level_noise(void)
{
	float worst = 0.0f;
	unsigned int solved = 0;
	uint32_t rng = 12345u;

	for (unsigned int trial = 0; trial < 200u; trial++) {
		struct tdoa_meas m[4];
		struct pos_result r;

		make(m, 4, 5.0f, 5.0f, 1000000);
		for (int i = 0; i < 4; i++) {
			rng = rng * 1664525u + 1013904223u;
			m[i].t_dtu += (int64_t)((rng >> 8) % 65u) - 32;   /* +/-32 DTU */
		}
		if (!tdoa_solve(m, 4, NULL, &r) || !r.valid) continue;
		solved++;
		float e = sqrtf((r.x - 5.0f) * (r.x - 5.0f) +
				(r.y - 5.0f) * (r.y - 5.0f));
		if (e > worst) worst = e;
	}
	printf("  with +/-32 DTU (0.5 ns) noise: worst error %.3f m (%u/200 solved)\n",
	       (double)worst, solved);
	CHECK(solved > 190u);
	CHECK(worst < 1.0f);
}

/*
 * Gradient of the least-squares cost, computed INDEPENDENTLY of the solver's
 * own grad_norm2() so the tests below check the property rather than echoing
 * the implementation.
 */
static float grad_mag(const struct tdoa_meas *m, size_t n, float x, float y)
{
	float dx0 = x - m[0].x, dy0 = y - m[0].y;
	float r0 = sqrtf(dx0 * dx0 + dy0 * dy0 + m[0].dz * m[0].dz);
	float g0 = 0.0f, g1 = 0.0f;

	for (size_t i = 1; i < n; i++) {
		float dxi = x - m[i].x, dyi = y - m[i].y;
		float ri = sqrtf(dxi * dxi + dyi * dyi + m[i].dz * m[i].dz);
		float d_meas = (float)(m[i].t_dtu - m[0].t_dtu) * TDOA_M_PER_DTU;
		float f = (ri - r0) - d_meas;

		g0 += (dxi / ri - dx0 / r0) * f;
		g1 += (dyi / ri - dy0 / r0) * f;
	}
	return sqrtf(g0 * g0 + g1 * g1);
}

/*
 * THE regression test for this solver's worst failure mode.
 *
 * src/pos_solver.c records that without a gradient gate its own Gauss-Newton
 * "returned the seed verbatim with valid = true and a residual up to 113 m"
 * over 200k adversarial cases. This solver had neither that gate nor a line
 * search, and since 2026-09-02 src/tdoa_gw.c hands it the tag's LAST
 * PUBLISHED POSITION as the seed on every fix -- so a stalled solve does not
 * return a random point, it returns the previous answer while reporting
 * success. That is the stale-republish defect found on hardware, by a route
 * the `fixes == seeded + filtered + reseed` check is blind to (it enters
 * through the seeding path and tallies as n_seeded, so the totals balance).
 *
 * The signature is exact: valid = true with the output bit-for-bit equal to
 * the seed means no step was ever accepted. Asserted on float equality
 * deliberately -- an approximate test would also flag legitimate cases where
 * the seed happens to be very near the solution.
 */
static void test_never_returns_the_seed_verbatim(void)
{
	uint32_t rng = 987654321u;
	unsigned int valid_count = 0, trials = 0;

	for (unsigned int trial = 0; trial < 20000u; trial++) {
		struct tdoa_meas m[4];
		struct pos_result r;
		float seed[2];

		make(m, 4, 5.0f, 5.0f, 1000000);

		/* Corrupt the timestamps hard enough that the equations are
		 * mutually inconsistent -- there is no (x, y) that fits them, so
		 * Gauss-Newton has every reason to thrash. */
		for (int i = 0; i < 4; i++) {
			rng = rng * 1664525u + 1013904223u;
			m[i].t_dtu += (int64_t)((rng >> 8) % 4001u) - 2000;
		}
		/* ...and seed it from somewhere unhelpful, including well outside
		 * the anchor hull, which is where tdoa_solve.h warns the mirror
		 * branch lives. */
		rng = rng * 1664525u + 1013904223u;
		seed[0] = (float)((rng >> 8) % 400u) * 0.25f - 45.0f;
		rng = rng * 1664525u + 1013904223u;
		seed[1] = (float)((rng >> 8) % 400u) * 0.25f - 45.0f;

		trials++;
		if (!tdoa_solve(m, 4, seed, &r) || !r.valid) {
			continue;
		}
		valid_count++;

		/* The defect, stated exactly. */
		CHECK(!(r.x == seed[0] && r.y == seed[1]));

		/* And the general form of it: anything reported valid must be a
		 * stationary point. The bound is deliberately looser than the
		 * solver's own GRAD_EPS (5e-2) so this test checks the property,
		 * not the constant. */
		CHECK(grad_mag(m, 4, r.x, r.y) < 0.1f);
	}
	printf("  adversarial seeds: %u/%u returned valid, none verbatim\n",
	       valid_count, trials);
	/* A solver that refused everything would pass the checks above
	 * vacuously -- same trap test_error_under_gate_level_noise() documents. */
	CHECK(valid_count > 0u);
}

/*
 * The line search earns its keep on a seed far outside the anchor hull, where
 * the undamped Gauss-Newton step on a hyperbolic surface overshoots. Data is
 * CLEAN here, so there is a right answer and the only question is whether the
 * iteration reaches it.
 */
/*
 * The seed basin: how far a seed can start from the true position and still
 * converge, swept over distance and direction on CLEAN data.
 *
 * This test PRINTS the table rather than asserting a shape, in the same
 * spirit as test_error_under_gate_level_noise() -- the measured basin is the
 * deliverable. Two assertions only, and both are properties rather than
 * tuned thresholds:
 *
 *   1. Near seeds must converge from EVERY direction. "Near" is set from the
 *      OPERATING regime, not from where the measured table happens to break:
 *      tdoa_gw.c seeds from the tag's last published position, at most one
 *      blink (200 ms) old, and a walking tag moves ~0.15 m in that time. So
 *      the real seed error is centimetres. 5 m is already ~30x that, and is
 *      the bar asserted. (TDOA_GW_MAX_JUMP_M's 10 m is the outer bound of the
 *      mirror-branch jump gate, not a typical seed error -- and at 10 m the
 *      measured basin is 7/8, recorded here rather than asserted, because
 *      tuning the assertion to the measurement is how a test stops testing
 *      anything.)
 *   2. THE INVARIANT: whenever the solver reports valid, the answer is the
 *      true position. It may legitimately refuse -- far outside the hull the
 *      direction cosines to every anchor converge on each other, so their
 *      DIFFERENCES (which is what this Jacobian's rows are) collapse and
 *      det(J^T J) falls under DET_EPS. Refusing there is correct: the
 *      geometry carries no information and an answer would be fabricated.
 *      What it must never do is report a confident wrong one.
 *
 * Measured 2026-09-03, tag at (6, 4) in a 10 m square, 8 directions per ring,
 * before and after adding the line search:
 *
 *     seed distance   5    10   15   20-40  45     50-60
 *     before          8/8  2/8  0/8  0/8    0/8    0/8
 *     after           8/8  7/8  4/8  3/8    2/8    1/8
 *
 * The line search is what widened that, and the "before" row is why a raw
 * Gauss-Newton step is not adequate on a hyperbolic cost surface.
 */
static void test_seed_basin_and_never_wrong_when_valid(void)
{
	printf("  seed basin (tag at 6,4; 8 directions per ring):\n");

	for (float d = 5.0f; d <= 60.0f; d += 5.0f) {
		unsigned int ok = 0;

		for (unsigned int k = 0; k < 8u; k++) {
			float a = (float)k * 3.14159265f / 4.0f;
			struct tdoa_meas m[4];
			struct pos_result r;
			float seed[2] = { 6.0f + d * cosf(a),
					  4.0f + d * sinf(a) };

			make(m, 4, 6.0f, 4.0f, 500000);
			if (!tdoa_solve(m, 4, seed, &r) || !r.valid) {
				continue;
			}
			ok++;
			/* Invariant 2. */
			CHECK(fabsf(r.x - 6.0f) < 0.05f);
			CHECK(fabsf(r.y - 4.0f) < 0.05f);
			CHECK(grad_mag(m, 4, r.x, r.y) < 0.1f);
		}
		printf("    %4.0f m: %u/8\n", (double)d, ok);

		/* Invariant 1, only for the rings that matter operationally. */
		if (d <= 5.0f) {
			CHECK(ok == 8u);
		}
	}
}

/* Build observations for a tag whose TRUE vertical separation from the anchor
 * plane is `true_h`, but hand the solver a model dz of `model_dz`. With the
 * two equal this is the honest case; with model_dz = 0 it is what the gateway
 * did before `apos tagz` existed. */
static void make_h(struct tdoa_meas *m, size_t n, float tx, float ty,
		   float true_h, float model_dz, const float *ax,
		   const float *ay)
{
	for (size_t i = 0; i < n; i++) {
		float dx = tx - ax[i], dy = ty - ay[i];
		float r = sqrtf(dx * dx + dy * dy + true_h * true_h);

		m[i].x = ax[i];
		m[i].y = ay[i];
		m[i].dz = model_dz;
		m[i].t_dtu = 500000 + (int64_t)llroundf(r / TDOA_M_PER_DTU);
	}
}

/*
 * The height model (`apos tagz`, apos_store.h's tag_z_m).
 *
 * The tempting simplification is that with every anchor at one height the dz
 * is a common term that cancels in a range DIFFERENCE. It does not:
 * sqrt(rho^2 + dz^2) is nonlinear, so a uniform dz COMPRESSES the differences,
 * and a model that assumes dz = 0 has to move the estimate to somewhere the
 * horizontal differences are smaller -- i.e. TOWARD the anchor centroid.
 *
 * Direction measured, not reasoned: the bias is consistently INWARD. An
 * earlier draft of this work asserted "outward" in three places and was simply
 * wrong; the numbers below are what settled it.
 *
 * Magnitude, measured 2026-09-03, tag on the mid-line, error at the worst
 * point tested:
 *
 *     10.0 m array, H = 1.5 m   ->  0.098 m   (offset from centre 4.000 -> 3.902)
 *      2.5 m array, H = 1.4 m   ->  0.230 m   (offset from centre 1.000 -> 0.770)
 *      2.5 m array, H = 0.0 m   ->  0.002 m   (no-op, as the default must be)
 *
 * The middle row is the one that matters: THIS project's array measures
 * 1.2-2.5 m per edge, so an unmodelled 1.4 m separation shrinks the reported
 * offset from centre by ~23% -- the same order as the whole ~45 cm accuracy
 * target. On a 10 m array the same H is a 2.5% effect and easy to dismiss.
 */
static void test_height_model(void)
{
	/* A 2.5 m square, this deployment's scale rather than the 10 m one the
	 * rest of this file uses. */
	static const float SX[4] = { 0.0f, 2.5f, 2.5f, 0.0f };
	static const float SY[4] = { 0.0f, 0.0f, 2.5f, 2.5f };
	const float H = 1.4f;
	const float tx = 2.25f, ty = 1.25f;
	const float cx = 1.25f;          /* array centre, x */
	struct tdoa_meas m[4];
	struct pos_result r;

	/* 1. Modelled correctly: the true position comes back. */
	make_h(m, 4, tx, ty, H, H, SX, SY);
	CHECK(tdoa_solve(m, 4, NULL, &r));
	CHECK(r.valid);
	CHECK(fabsf(r.x - tx) < 0.02f);
	CHECK(fabsf(r.y - ty) < 0.02f);

	/* 2. Modelled as dz = 0 against a real 1.4 m separation: a real error,
	 *    and it pulls the estimate INWARD. Both parts asserted -- a test
	 *    that only checked the magnitude would pass on a bias in either
	 *    direction. */
	make_h(m, 4, tx, ty, H, 0.0f, SX, SY);
	CHECK(tdoa_solve(m, 4, NULL, &r));
	CHECK(r.valid);
	{
		float err = sqrtf((r.x - tx) * (r.x - tx) +
				  (r.y - ty) * (r.y - ty));

		printf("  height model: true (%.2f,%.2f), dz=0 gives "
		       "(%.3f,%.3f), err %.3f m\n",
		       (double)tx, (double)ty, (double)r.x, (double)r.y,
		       (double)err);
		CHECK(err > 0.15f);                       /* the bias is real */
		CHECK(fabsf(r.x - cx) < fabsf(tx - cx));  /* and it is INWARD */
	}

	/* 3. dz = 0 is a NO-OP when the tags really are in the anchor plane.
	 *    This is what makes apos_store's 0.0 default safe to ship: an
	 *    unconfigured gateway reports exactly what it reported before. */
	make_h(m, 4, tx, ty, 0.0f, 0.0f, SX, SY);
	CHECK(tdoa_solve(m, 4, NULL, &r));
	CHECK(r.valid);
	CHECK(fabsf(r.x - tx) < 0.01f);
	CHECK(fabsf(r.y - ty) < 0.01f);
}

int main(void)
{
	test_dtu_scale();
	test_recovers_a_known_position();
	test_common_offset_cancels();
	test_three_anchors_is_enough();
	test_three_anchor_residual_is_structurally_zero();
	test_too_few_anchors_refused();
	test_collinear_anchors_refused();
	test_reference_anchor_singularity_refused();
	test_null_seed_converges();
	test_bad_seed_does_not_fabricate_a_result();
	test_error_under_gate_level_noise();
	test_never_returns_the_seed_verbatim();
	test_seed_basin_and_never_wrong_when_valid();
	test_height_model();
	if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
	printf("tdoa_solve: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

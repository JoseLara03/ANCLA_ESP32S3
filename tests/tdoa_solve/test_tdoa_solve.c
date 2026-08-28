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
 * test PRINTS it rather than asserting it -- the number is the deliverable. */
static void test_error_under_gate_level_noise(void)
{
	float worst = 0.0f;
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
		float e = sqrtf((r.x - 5.0f) * (r.x - 5.0f) +
				(r.y - 5.0f) * (r.y - 5.0f));
		if (e > worst) worst = e;
	}
	printf("  with +/-32 DTU (0.5 ns) noise: worst error %.3f m\n",
	       (double)worst);
	CHECK(worst < 1.0f);
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
	if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
	printf("tdoa_solve: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

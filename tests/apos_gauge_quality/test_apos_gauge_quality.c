/*
 * The 3D gauge conditioning gate against the real solver and realistic ranging
 * noise. The point is the square-root law documented on
 * APOS_GAUGE_UP_NOISE_K: a coplanar gauge plus ranging noise reconstructs with
 * a non-zero apparent height, so "volume != 0" cannot be the test.
 */
#include "apos_geom.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static uint32_t rng = 12345u;

static float urand(void)
{
	rng = rng * 1664525u + 1013904223u;
	return ((float)(rng >> 8) + 0.5f) / 16777216.0f;
}

static float grand(void)
{
	float u1 = urand(), u2 = urand();

	return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static float dist3(const float a[3], const float b[3])
{
	float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];

	return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* 4 anchors + a gateway (node 4), every pair, noise sigma on each edge. */
static int run(const float xyz[5][3], float sigma, struct apos_result *r,
	       struct apos_gauge_quality *q, float range_err)
{
	struct apos_edge e[10];
	uint16_t k = 0;
	struct apos_gauge g = {.origin = 0, .xaxis = 1, .plane = 2, .up = 3,
			       .dim = APOS_GEOM_3D};

	for (uint8_t i = 0; i < 5; i++) {
		for (uint8_t j = (uint8_t)(i + 1); j < 5; j++) {
			e[k].i = i;
			e[k].j = j;
			e[k].d_m = dist3(xyz[i], xyz[j]) + sigma * grand();
			e[k].sd_m = 0.03f;
			k++;
		}
	}
	int rc = apos_geom_solve(e, k, 5, &g, r);

	if (rc) {
		return rc;
	}
	return apos_geom_gauge_conditioning(r, &g, range_err, q);
}

#define TRIALS 400

/* Four ceiling anchors at one height (z = 0 plane), gateway off the plane.
 * This is the install the plan says must be refused. */
static void test_coplanar_gauge_is_refused_under_noise(void)
{
	const float xyz[5][3] = {
		{0, 0, 0}, {2.4f, 0, 0}, {0, 2.0f, 0}, {2.4f, 2.0f, 0},
		{1.2f, 1.0f, -1.4f}, /* gateway, far off the plane: must NOT save it */
	};
	int accepted = 0, nsolve = 0;

	for (int t = 0; t < TRIALS; t++) {
		struct apos_result r;
		struct apos_gauge_quality q;

		if (run(xyz, 0.03f, &r, &q, 0.03f) != 0) {
			continue;
		}
		nsolve++;
		if (!q.coplanar && !q.collinear) {
			accepted++;
		}
	}
	printf("coplanar: %d/%d accepted\n", accepted, nsolve);
	CHECK(nsolve > TRIALS / 2);
	CHECK(accepted * 100 <= nsolve); /* <= 1 % false accepts */
}

/* A genuinely 3D gauge: `up` 0.8 m above the plane of the other three. */
static void test_tall_gauge_is_accepted_under_noise(void)
{
	const float xyz[5][3] = {
		{0, 0, 0}, {2.4f, 0, 0}, {0, 2.0f, 0}, {2.4f, 2.0f, 0.8f},
		{1.2f, 1.0f, -1.4f},
	};
	int rejected = 0, nsolve = 0;

	for (int t = 0; t < TRIALS; t++) {
		struct apos_result r;
		struct apos_gauge_quality q;

		if (run(xyz, 0.03f, &r, &q, 0.03f) != 0) {
			continue;
		}
		nsolve++;
		if (q.coplanar || q.collinear) {
			rejected++;
		}
	}
	printf("tall: %d/%d rejected\n", rejected, nsolve);
	CHECK(nsolve > TRIALS / 2);
	CHECK(rejected * 100 <= nsolve * 2); /* <= 2 % false rejects */
}

/* A nearly-collinear origin/xaxis/plane triangle is refused on its own. */
static void test_collinear_triangle_is_refused(void)
{
	const float xyz[5][3] = {
		{0, 0, 0}, {3.0f, 0, 0}, {1.5f, 0.1f, 0}, {1.0f, 1.0f, 1.5f},
		{1.2f, 1.0f, -1.4f},
	};
	struct apos_result r;
	struct apos_gauge_quality q;

	CHECK(run(xyz, 0.0f, &r, &q, 0.03f) == 0);
	CHECK(q.collinear);
}

static void test_exact_regular_tetra_values(void)
{
	const float xyz[5][3] = {
		{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 1.0f}, {1, 1, -1},
	};
	struct apos_result r;
	struct apos_gauge_quality q;

	CHECK(run(xyz, 0.0f, &r, &q, 0.03f) == 0);
	CHECK(fabsf(q.up_height_m - 1.0f) < 0.01f);
	CHECK(fabsf(q.collinearity - 1.0f) < 0.01f); /* |a x b| = 4, |a|^2 = 4 */
	CHECK(!q.coplanar && !q.collinear);
}

static void test_rejects_2d_and_null(void)
{
	struct apos_result r;
	struct apos_gauge_quality q;
	struct apos_gauge g2 = {.origin = 0, .xaxis = 1, .plane = 2, .up = 0,
				.dim = APOS_GEOM_2D};

	memset(&r, 0, sizeof(r));
	CHECK(apos_geom_gauge_conditioning(NULL, &g2, 0.03f, &q) == -22);
	CHECK(apos_geom_gauge_conditioning(&r, &g2, 0.03f, &q) == -22);
}

int main(void)
{
	test_exact_regular_tetra_values();
	test_collinear_triangle_is_refused();
	test_coplanar_gauge_is_refused_under_noise();
	test_tall_gauge_is_accepted_under_noise();
	test_rejects_2d_and_null();
	if (g_fail) {
		printf("%d CHECK(s) FAILED\n", g_fail);
		return 1;
	}
	printf("apos_gauge_quality: ALL TESTS PASSED\n");
	return 0;
}

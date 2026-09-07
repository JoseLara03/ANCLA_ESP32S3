#include "pos_abg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		g_fail++; \
	} \
} while (0)

/* ---- Steady-state Kalman filter for the jerk model, the reference the
 * closed-form gains must match. State [x, v, a], F = [[1,T,T^2/2],[0,1,T],
 * [0,0,1]], Q = sw^2 * g g^T with g = [T^2/2, T, 1], H = [1 0 0], R = sv^2.
 * Iterated to convergence; returns alpha = K0, beta = K1*T, gamma = K2*T^2
 * (the LAST one is the convention check -- see pos_abg.h). */
static void kalman_ss_gains(double sw, double sv, double T,
			    double *alpha, double *beta, double *gamma)
{
	double F[3][3] = {{1, T, T * T / 2}, {0, 1, T}, {0, 0, 1}};
	double g[3] = {T * T / 2, T, 1};
	double P[3][3] = {{10, 0, 0}, {0, 10, 0}, {0, 0, 10}};
	double K[3] = {0, 0, 0};

	for (int it = 0; it < 5000; it++) {
		double FP[3][3], Pp[3][3];
		int i, j, k;

		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
			FP[i][j] = 0;
			for (k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
		}
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
			Pp[i][j] = sw * sw * g[i] * g[j];
			for (k = 0; k < 3; k++) Pp[i][j] += FP[i][k] * F[j][k];
		}
		double S = Pp[0][0] + sv * sv;
		for (i = 0; i < 3; i++) K[i] = Pp[i][0] / S;
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
			P[i][j] = Pp[i][j] - K[i] * Pp[0][j];
	}
	*alpha = K[0];
	*beta  = K[1] * T;
	*gamma = K[2] * T * T;
}

static void test_gains_match_steady_state_kalman(void)
{
	const double T = 0.2, sv = 0.35;
	const double lams[3] = {0.01, 0.02, 0.08};

	for (int i = 0; i < 3; i++) {
		struct pos_abg_cfg c;
		double sw = lams[i] * sv / (T * T);
		double a, b, g;

		memset(&c, 0, sizeof(c));
		pos_abg_cfg_from_index(&c, (float)lams[i]);
		kalman_ss_gains(sw, sv, T, &a, &b, &g);
		CHECK(fabs(c.alpha - a) < 1e-3);
		CHECK(fabs(c.beta  - b) < 1e-3);
		CHECK(fabs(c.gamma - g) < 1e-3);   /* the 2x convention trap */
		/* from_index must not touch the other fields */
		CHECK(c.alpha_still == 0.0f && c.gate_m == 0.0f &&
		      c.reset_after == 0u);
	}
}

static void test_defaults(void)
{
	struct pos_abg_cfg c, ref;

	pos_abg_cfg_defaults(&c);
	memset(&ref, 0, sizeof(ref));
	pos_abg_cfg_from_index(&ref, 0.02f);
	CHECK(c.alpha == ref.alpha && c.beta == ref.beta && c.gamma == ref.gamma);
	CHECK(fabsf(c.alpha - 0.419f) < 0.005f);   /* spec §3.3 table, lambda 0.02 */
	CHECK(fabsf(c.beta  - 0.113f) < 0.005f);
	CHECK(fabsf(c.gamma - 0.0152f) < 0.001f);
	CHECK(c.alpha_still == 0.15f);
	CHECK(c.gate_m == 1.5f);
	CHECK(c.reset_after == 5u);
}

static void test_get_before_seed_and_null_pointers(void)
{
	struct pos_abg f;
	float x = 7.0f, y = 7.0f;

	pos_abg_reset(&f);
	CHECK(!pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(x == 7.0f && y == 7.0f);   /* untouched */

	pos_abg_seed(&f, 1.0f, 2.0f);
	CHECK(pos_abg_get(&f, NULL, NULL, NULL, NULL));
	CHECK(pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(x == 1.0f && y == 2.0f);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	CHECK(f.reject_streak == 0u);
}

/* Filter that is the identity (alpha 1, beta 0, gamma 0) must return z. */
static void test_identity_gains_pass_measurement_through(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	float x, y;

	pos_abg_cfg_defaults(&c);
	c.alpha = 1.0f; c.beta = 0.0f; c.gamma = 0.0f; c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 3.0f, -4.0f, false) == POS_ABG_ACCEPTED);
	CHECK(pos_abg_get(&f, &x, &y, NULL, NULL));
	CHECK(fabsf(x - 3.0f) < 1e-6f && fabsf(y + 4.0f) < 1e-6f);
}

/* A 1 m step in x: settles within 5 % in under 40 steps at every lambda in
 * the spec's table, and the impulse response decays -- stability by
 * simulation, the spec's stated test. */
static void test_step_response_settles(void)
{
	const float lams[5] = {0.01f, 0.02f, 0.03f, 0.05f, 0.08f};

	for (int i = 0; i < 5; i++) {
		struct pos_abg_cfg c;
		struct pos_abg f;
		float hist[400];
		int settle = -1;

		pos_abg_cfg_defaults(&c);
		pos_abg_cfg_from_index(&c, lams[i]);
		c.gate_m = 1e9f;
		pos_abg_reset(&f);
		pos_abg_seed(&f, 0.0f, 0.0f);
		for (int k = 0; k < 400; k++) {
			CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 0.0f, false) ==
			      POS_ABG_ACCEPTED);
			hist[k] = f.x;
		}
		for (int k = 0; k < 400 && settle < 0; k++) {
			int ok = 1;
			for (int j = k; j < 400; j++) {
				if (fabsf(hist[j] - 1.0f) >= 0.05f) { ok = 0; break; }
			}
			if (ok) settle = k;
		}
		CHECK(settle >= 0 && settle < 40);
		CHECK(fabsf(hist[399] - 1.0f) < 1e-3f);
		CHECK(fabsf(f.vx) < 1e-3f && fabsf(f.ax) < 1e-3f);
	}
}

/* xorshift32, so the noise is reproducible across platforms. */
static uint32_t g_rng = 0xA5A5A5A5u;
static float noise(float sigma)
{
	/* sum of 12 uniforms -> approx N(0,1) */
	float s = 0.0f;
	for (int i = 0; i < 12; i++) {
		g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
		s += (float)(g_rng & 0xFFFFu) / 65535.0f;
	}
	return (s - 6.0f) * sigma;
}

/* Stationary tag, white noise: output RMS / input RMS ~ 0.60 at lambda 0.02
 * (spec §3.3 table), checked to +/-0.05. */
static void test_stationary_noise_reduction(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	const int N = 20000, warm = 500;
	const float sv = 0.35f;
	double acc = 0.0;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < N; k++) {
		pos_abg_step(&f, &c, 0.2f, noise(sv), noise(sv), false);
		if (k >= warm) acc += (double)f.x * f.x;
	}
	double ratio = sqrt(acc / (N - warm)) / sv;
	CHECK(ratio > 0.55 && ratio < 0.65);
}

/* Constant velocity: an alpha-beta-gamma filter has ZERO steady-state lag
 * on a constant-velocity target. |error| < 2 cm after settling. */
static void test_constant_velocity_no_lag(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	const float v = 1.2f, T = 0.2f;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 1; k <= 300; k++) {
		float t = (float)k * T;
		pos_abg_step(&f, &c, T, v * t, 0.0f, false);
		if (k > 100) {
			CHECK(fabsf(f.x - v * t) < 0.02f);
			CHECK(fabsf(f.vx - v) < 0.05f);
		}
	}
}

static void test_bad_input_leaves_state_untouched(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f, before;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 1.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(!f.init);

	pos_abg_seed(&f, 1.0f, 2.0f);
	f.vx = 0.3f; f.ax = -0.1f;
	before = f;
	CHECK(pos_abg_step(&f, &c, 0.0f, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, -0.2f, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, NAN, 1.0f, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, 0.2f, NAN, 2.0f, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, INFINITY, false) == POS_ABG_BAD_INPUT);
	CHECK(memcmp(&f, &before, sizeof(f)) == 0);
}

/* dt of 1.0 s (five nominal steps) must not blow up beta/dt, gamma/dt^2. */
static void test_long_dt_is_bounded(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	c.gate_m = 1e9f;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 1.0f, 0.0f, false) == POS_ABG_ACCEPTED);
	CHECK(f.x > 0.0f && f.x <= 1.0f);
	CHECK(fabsf(f.vx) <= c.beta / 1.0f * 1.0f + 1e-6f);
	CHECK(fabsf(f.ax) <= c.gamma / 1.0f * 1.0f + 1e-6f);
}

int main(void)
{
	test_gains_match_steady_state_kalman();
	test_defaults();
	test_get_before_seed_and_null_pointers();
	test_identity_gains_pass_measurement_through();
	test_step_response_settles();
	test_stationary_noise_reduction();
	test_constant_velocity_no_lag();
	test_bad_input_leaves_state_untouched();
	test_long_dt_is_bounded();

	if (g_fail) {
		printf("pos_abg: %d FAILED\n", g_fail);
		return 1;
	}
	printf("pos_abg: ALL TESTS PASSED\n");
	return 0;
}

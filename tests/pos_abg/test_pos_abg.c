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

/* A single 3 m outlier is REJECTED: state equals the prediction (which for a
 * stationary seeded filter is the seed), init still true, streak 1. */
static void test_gate_rejects_outlier_and_holds_prediction(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 1.0f, 1.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 4.0f, 1.0f, false) == POS_ABG_REJECTED);
	CHECK(f.init);
	CHECK(f.x == 1.0f && f.y == 1.0f);
	CHECK(f.vx == 0.0f && f.ax == 0.0f);
	CHECK(f.reject_streak == 1u);

	/* A moving filter coasts on its velocity while rejecting. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	f.vx = 1.0f;
	CHECK(pos_abg_step(&f, &c, 0.2f, 10.0f, 0.0f, false) == POS_ABG_REJECTED);
	CHECK(fabsf(f.x - 0.2f) < 1e-6f);
	CHECK(f.vx == 1.0f);
}

/* Exactly at the gate is accepted (the test is `>`), just past it rejected. */
static void test_gate_boundary(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.5f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.5001f, 0.0f, false) == POS_ABG_REJECTED);
	/* Euclidean, not per-axis: 1.2 in x and 1.2 in y is 1.70 m. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.2f, 1.2f, false) == POS_ABG_REJECTED);
}

/* The gate grows with dt: 2x at 0.4 s, capped at 3x (4.5 m) from 0.6 s up.
 * Below nominal dt it stays at gate_m. */
static void test_gate_scales_with_dt(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.4f, 2.5f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.4f, 3.1f, 0.0f, false) == POS_ABG_REJECTED);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 4.4f, 0.0f, false) == POS_ABG_ACCEPTED);
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 1.0f, 4.6f, 0.0f, false) == POS_ABG_REJECTED);

	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.1f, 1.6f, 0.0f, false) == POS_ABG_REJECTED);
}

/* Five consecutive far measurements: RESEEDED on the fifth at the new spot,
 * velocity zero, streak 0. Four then a return: no reseed, streak back to 0. */
static void test_streak_reseeds_after_reset_after(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	int k;

	pos_abg_cfg_defaults(&c);
	CHECK(c.reset_after == 5u);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (k = 0; k < 4; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
		CHECK(f.reject_streak == (uint8_t)(k + 1));
	}
	CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_RESEEDED);
	CHECK(f.x == 5.0f && f.y == 5.0f);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	CHECK(f.reject_streak == 0u);

	pos_abg_seed(&f, 0.0f, 0.0f);
	for (k = 0; k < 4; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
	}
	CHECK(pos_abg_step(&f, &c, 0.2f, 0.1f, 0.0f, false) == POS_ABG_ACCEPTED);
	CHECK(f.reject_streak == 0u);
	CHECK(f.x > 0.0f && f.x < 0.1f);
}

/* reset_after == 0 means never reseed on the streak; the streak saturates
 * at 255 instead of wrapping to 0. */
static void test_reset_after_zero_never_reseeds_and_streak_saturates(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	c.reset_after = 0u;
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 300; k++) {
		CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 5.0f, false) == POS_ABG_REJECTED);
	}
	CHECK(f.reject_streak == 255u);
	CHECK(f.x == 0.0f && f.y == 0.0f);
}

/* still: velocity and acceleration are exactly zero after the step, and the
 * position moves by alpha_still * r, not alpha * r. */
static void test_still_freezes_velocity_and_uses_alpha_still(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	f.vx = 0.5f; f.ax = 0.2f;   /* pretend it had been moving */
	CHECK(pos_abg_step(&f, &c, 0.2f, 1.0f, 0.0f, true) == POS_ABG_ACCEPTED);
	CHECK(f.vx == 0.0f && f.vy == 0.0f && f.ax == 0.0f && f.ay == 0.0f);
	/* prediction was 0 + 0.5*0.2 + 0.5*0.2*0.04 = 0.104; correction 0.15*(1-0.104) */
	CHECK(fabsf(f.x - (0.104f + 0.15f * (1.0f - 0.104f))) < 1e-5f);

	/* A gated cycle while still still counts as rejected, not accepted. */
	pos_abg_seed(&f, 0.0f, 0.0f);
	CHECK(pos_abg_step(&f, &c, 0.2f, 5.0f, 0.0f, true) == POS_ABG_REJECTED);
}

/* One-sample 1.4 m outlier (just under the gate) on a stationary tag at the
 * default lambda: peak output deviation < 0.65 m and back within 5 cm in 30
 * steps -- spec §3.3 table row for lambda 0.02 says 0.59 m. */
static void test_sub_gate_outlier_is_attenuated(void)
{
	struct pos_abg_cfg c;
	struct pos_abg f;
	float peak = 0.0f;

	pos_abg_cfg_defaults(&c);
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 60; k++) {
		float z = (k == 5) ? 1.4f : 0.0f;

		CHECK(pos_abg_step(&f, &c, 0.2f, z, 0.0f, false) == POS_ABG_ACCEPTED);
		if (fabsf(f.x) > peak) peak = fabsf(f.x);
		if (k >= 35) CHECK(fabsf(f.x) < 0.05f);
	}
	CHECK(peak > 0.5f && peak < 0.65f);
}

/* The spec §3.3 "Finding" scenario: a stationary tag, 10^5 steps of random
 * dt in [0.2, 1.0] s, sigma 0.35 m noise, a 4 m outlier every 997 cycles,
 * never `still` (a worn tag). Returns the worst |estimate| and counts
 * reseeds. Shared by the two tests below. */
static float long_run(const struct pos_abg_cfg *c, uint32_t *reseeds_out)
{
	struct pos_abg f;
	float worst = 0.0f;
	uint32_t reseeds = 0u;

	g_rng = 0xC0FFEE11u;   /* same draw for every caller */
	pos_abg_reset(&f);
	pos_abg_seed(&f, 0.0f, 0.0f);
	for (int k = 0; k < 100000; k++) {
		g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
		float dt = 0.2f + 0.8f * (float)(g_rng & 0xFFu) / 255.0f;
		float zx = noise(0.35f), zy = noise(0.35f);

		if ((k % 997) == 0) { zx += 4.0f; }       /* gross outlier */
		if (pos_abg_step(&f, c, dt, zx, zy, false) == POS_ABG_RESEEDED) {
			reseeds++;
		}
		CHECK(isfinite(f.x) && isfinite(f.y) && isfinite(f.vx) &&
		      isfinite(f.vy) && isfinite(f.ax) && isfinite(f.ay));
		if (fabsf(f.x) > worst) worst = fabsf(f.x);
		if (fabsf(f.y) > worst) worst = fabsf(f.y);
	}
	*reseeds_out = reseeds;
	return worst;
}

/* Default config: finite throughout, streak reseeds under 1 % of cycles.
 * NOTE the bound this test does NOT make: the estimate is not asserted to
 * stay near the truth, because with the default gamma it does not (spec
 * §3.3 table: worst 8.6 m). That is the finding the next test pins. */
static void test_long_run_stays_finite(void)
{
	struct pos_abg_cfg c;
	uint32_t reseeds = 0u;

	pos_abg_cfg_defaults(&c);
	(void)long_run(&c, &reseeds);
	CHECK(reseeds < 1000u);
}

/* Spec §3.3 "Finding", pinned: under this dt spread the alpha-beta filter
 * (gamma = 0) has a smaller worst excursion than the default alpha-beta-gamma,
 * and no more reseeds. If this starts failing, the finding has been
 * re-measured -- update spec §3.3 with the new numbers, do not delete this. */
static void test_gamma_zero_is_more_robust_under_dt_jitter(void)
{
	struct pos_abg_cfg c_abg, c_ab;
	uint32_t r_abg = 0u, r_ab = 0u;
	float w_abg, w_ab;

	pos_abg_cfg_defaults(&c_abg);
	c_ab = c_abg;
	c_ab.gamma = 0.0f;
	w_abg = long_run(&c_abg, &r_abg);
	w_ab  = long_run(&c_ab, &r_ab);
	printf("  finding: worst |x| abg %.2f m (%u reseeds) vs ab %.2f m "
	       "(%u reseeds)\n", (double)w_abg, r_abg, (double)w_ab, r_ab);
	CHECK(w_ab < w_abg);
	CHECK(r_ab <= r_abg);
	CHECK(w_ab < 3.0f);   /* spec table: ~2.3 m */
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
	test_gate_rejects_outlier_and_holds_prediction();
	test_gate_boundary();
	test_gate_scales_with_dt();
	test_streak_reseeds_after_reset_after();
	test_reset_after_zero_never_reseeds_and_streak_saturates();
	test_still_freezes_velocity_and_uses_alpha_still();
	test_sub_gate_outlier_is_attenuated();
	test_long_run_stays_finite();
	test_gamma_zero_is_more_robust_under_dt_jitter();

	if (g_fail) {
		printf("pos_abg: %d FAILED\n", g_fail);
		return 1;
	}
	printf("pos_abg: ALL TESTS PASSED\n");
	return 0;
}

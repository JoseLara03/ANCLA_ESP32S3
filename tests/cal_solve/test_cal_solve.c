#include "cal_solve.h"
#include "cal_math.h"

#include <errno.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* The tag's own vectors travel with the copied file. If these fail, the copy is
 * wrong -- not this project's arithmetic. */
static void test_tag_selftest_vectors_pass(void)
{
    CHECK(cal_math_selftest() == 0);
}

/* 234 mm too far at 2.34 mm/unit is +100 units of combined delay, and with
 * cur_rx pinned the whole +100 lands on tx. */
static void test_measuring_too_far_increases_tx(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2234, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16485);
}

/* Symmetric: measuring short pulls tx down by the same 100 units. */
static void test_measuring_too_short_decreases_tx(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(1766, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16285);
}

/* A perfect measurement must not move the delay at all. */
static void test_zero_error_is_a_fixed_point(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2000, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16385);
}

/* cur_rx is held fixed, so an already-asymmetric pair keeps its rx and moves
 * only tx -- the correction is computed on the SUM but applied to tx alone. */
static void test_rx_is_held_fixed_and_correction_lands_on_tx(void)
{
    uint16_t tx = 0;

    /* Combined 16000 + 16385 = 32385, +100 units -> 32485, minus rx -> 16100. */
    CHECK(cal_solve_tx_delay(2234, 2000, 16000, 16385, &tx) == 0);
    CHECK(tx == 16100);
}

/* Out-of-range results are a reported failure, not a silent clamp: a 10 m error
 * means the setup is wrong (wrong peer, wrong tape, reflection), and quietly
 * pinning the delay at the rail would hide that. */
static void test_clamp_high_is_reported(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(12000, 2000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx == CAL_TX_DLY_MAX);
}

static void test_clamp_low_is_reported(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(0, 10000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx == CAL_TX_DLY_MIN);
}

/* The batch statistics the shell feeds cal_solve_tx_delay: a gross outlier from
 * a reflection or a half-decoded frame must not drag the mean. */
static void test_filtered_mean_rejects_a_gross_outlier(void)
{
    int32_t s[] = {2001, 1999, 2000, 2002, 1998, 9000};
    int32_t mean = 0;
    size_t kept = 0;

    CHECK(cal_filtered_mean(s, 6, &mean, &kept));
    CHECK(kept == 5);
    CHECK(mean >= 1998 && mean <= 2002);
}

/* CAL_MAX_STEP_MM must track cal_math.h's exported constants, not a literal.
 * If cal_math changes on the tag side and is re-copied, this is what notices. */
static void test_max_step_mm_is_derived(void)
{
    CHECK(CAL_MAX_STEP_MM == 4680);
    CHECK(CAL_MAX_STEP_MM ==
          (int32_t)(((int32_t)CAL_MAX_STEP_UNITS * CAL_MM_PER_UNIT_X1000) / 1000));
}

/* The regression that motivated the guard. A 10 m measurement error used to be
 * refused because the unclamped solve overshot CAL_TX_DLY_MAX. Once
 * cal_solve_step() gained its +/-CAL_MAX_STEP_UNITS bound, the saturated result
 * landed at 18385 -- inside the accepted window -- and cal_solve_tx_delay()
 * returned 0. That is a SUCCESS carrying a delay ~4.7 m wrong, which the
 * calling procedure would then have persisted to NVS.
 *
 * The value the guard reports matters as much as the rejection: 18385 must
 * never be handed back as if it were a solution. */
static void test_saturated_step_is_rejected_not_reported_as_success(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(12000, 2000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx != 18385);
    CHECK(tx == CAL_TX_DLY_MAX);

    /* Same in the other direction. */
    tx = 0;
    CHECK(cal_solve_tx_delay(2000, 12000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx == CAL_TX_DLY_MIN);
}

/* The guard must not be over-eager: an error of exactly CAL_MAX_STEP_MM needs
 * exactly CAL_MAX_STEP_UNITS, which the solver applies without saturating. So
 * the boundary itself is a legitimate calibration and must succeed. */
static void test_boundary_error_is_accepted(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2000 + CAL_MAX_STEP_MM, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx > 16385);

    /* One millimetre past it is not. */
    tx = 0;
    CHECK(cal_solve_tx_delay(2000 + CAL_MAX_STEP_MM + 1, 2000, 16385, 16385,
                             &tx) == -ERANGE);
}

/* A realistic bad-but-not-absurd reading still has to work: half a metre of
 * error is well inside one step and is exactly what `cal ref` exists to fix. */
static void test_ordinary_correction_still_succeeds(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2500, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx > 16385 && tx < CAL_TX_DLY_MAX);
}

/* ---- link statistics (the range test) ---------------------------------- */

static void test_link_stats_basic(void)
{
	/* mean 30000, deviations -20,-10,0,+10,+20 -> population var 200,
	 * sd = sqrt(200) = 14.14 -> 14 by integer truncation. */
	int32_t s[] = { 29980, 29990, 30000, 30010, 30020 };
	struct cal_link_stats st;

	CHECK(cal_link_stats_compute(s, 5, &st));
	CHECK(st.mean_mm == 30000);
	CHECK(st.sd_mm == 14);
	CHECK(st.min_mm == 29980);
	CHECK(st.max_mm == 30020);
}

static void test_link_stats_single_and_empty(void)
{
	int32_t s[] = { 1234 };
	struct cal_link_stats st;

	CHECK(cal_link_stats_compute(s, 1, &st));
	CHECK(st.mean_mm == 1234 && st.sd_mm == 0);
	CHECK(st.min_mm == 1234 && st.max_mm == 1234);

	CHECK(!cal_link_stats_compute(s, 0, &st));
	CHECK(!cal_link_stats_compute(NULL, 1, &st));
	CHECK(!cal_link_stats_compute(s, 1, NULL));
}

/* The integer sqrt must be exact on perfect squares and must never overshoot:
 * Newton's iteration can settle one either side, and both directions are
 * corrected explicitly. Swept rather than spot-checked, because an off-by-one
 * here reads as a plausible sd rather than as a failure. */
static void test_link_stats_isqrt_never_overshoots(void)
{
	struct cal_link_stats st;
	int k;

	for (k = 0; k < 400; k++) {
		/* Two samples at +/-k about 0 give population variance k*k,
		 * so sd must come out EXACTLY k. */
		int32_t s[2];

		s[0] = -k;
		s[1] = k;
		CHECK(cal_link_stats_compute(s, 2, &st));
		if (st.sd_mm != k) {
			printf("  isqrt: k=%d gave sd=%d\n", k, st.sd_mm);
			CHECK(st.sd_mm == k);
			return;
		}
	}
	printf("  isqrt exact for k=0..399\n");
}

/* A spread wide enough that squaring a deviation would overflow int32 if the
 * accumulation were not done in int64. 100 km in mm is 1e8; its square is
 * 1e16, which is ~4.6 million times INT32_MAX. */
static void test_link_stats_no_int32_overflow(void)
{
	int32_t s[] = { 0, 100000000 };
	struct cal_link_stats st;

	CHECK(cal_link_stats_compute(s, 2, &st));
	CHECK(st.mean_mm == 50000000);
	CHECK(st.sd_mm == 50000000);
}

/* ---- RX level ----------------------------------------------------------- */

static void test_rx_level_formula(void)
{
	/* level = 10*log10(C * 2^21 / N^2) - 121.7.
	 * With C = 12000 and N = 1024: 12000 * 2097152 = 2.5166e10,
	 * / 1048576 = 24000, 10*log10 = 43.802, - 121.7 = -77.898 -> -779. */
	CHECK(cal_rx_level_dbm_x10(12000u, 1024u) == -779);

	/* Doubling the accumulation count at fixed power drops the level by
	 * 6.02 dB (N is squared), which is the property worth pinning rather
	 * than a second magic number. */
	int32_t a = cal_rx_level_dbm_x10(12000u, 512u);
	int32_t b = cal_rx_level_dbm_x10(12000u, 1024u);

	CHECK(a - b == 60 || a - b == 61);
}

/* A CIA that had not finished when the diagnostics were read reports zeros.
 * That must NOT come back as a very low signal level -- on a polled,
 * interrupt-free receive path (ss_initiator.c) it is a routine outcome, and
 * reporting it as -infinity dBm would look exactly like a link at its floor. */
static void test_rx_level_invalid_is_not_a_weak_signal(void)
{
	CHECK(cal_rx_level_dbm_x10(0u, 1024u) == CAL_RX_LEVEL_INVALID);
	CHECK(cal_rx_level_dbm_x10(12000u, 0u) == CAL_RX_LEVEL_INVALID);
}

int main(void)
{
	test_link_stats_basic();
	test_link_stats_single_and_empty();
	test_link_stats_isqrt_never_overshoots();
	test_link_stats_no_int32_overflow();
	test_rx_level_formula();
	test_rx_level_invalid_is_not_a_weak_signal();
    test_tag_selftest_vectors_pass();
    test_measuring_too_far_increases_tx();
    test_measuring_too_short_decreases_tx();
    test_zero_error_is_a_fixed_point();
    test_rx_is_held_fixed_and_correction_lands_on_tx();
    test_clamp_high_is_reported();
    test_clamp_low_is_reported();
    test_filtered_mean_rejects_a_gross_outlier();
    test_max_step_mm_is_derived();
    test_saturated_step_is_rejected_not_reported_as_success();
    test_boundary_error_is_accepted();
    test_ordinary_correction_still_succeeds();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}

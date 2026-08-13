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

int main(void)
{
    test_tag_selftest_vectors_pass();
    test_measuring_too_far_increases_tx();
    test_measuring_too_short_decreases_tx();
    test_zero_error_is_a_fixed_point();
    test_rx_is_held_fixed_and_correction_lands_on_tx();
    test_clamp_high_is_reported();
    test_clamp_low_is_reported();
    test_filtered_mean_rejects_a_gross_outlier();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}

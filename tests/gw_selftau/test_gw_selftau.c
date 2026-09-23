/*
 * Host test for gw_selftau_mm_to_dtu() -- the one piece of gw_selftau.h that
 * has no Zephyr/radio dependency. Everything else in gw_selftau.c (the
 * request/step state machine) drives ss_initiator_range() and belongs to
 * hardware verification, same as blink_rx.c and apos_node.c have no host
 * suite for the same reason.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/gw_selftau/test_gw_selftau.exe \
 *       tests/gw_selftau/test_gw_selftau.c
 *
 * No .c to link: gw_selftau_mm_to_dtu() is `static inline` in the header, so
 * including gw_selftau.h (which pulls in ss_initiator.h for
 * SS_INITIATOR_MM_PER_DTU_X1000) is the whole test, same precedent as
 * ccp_sched.h's suite.
 */

#include "gw_selftau.h"

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                          \
	do {                                                                  \
		if (!(cond)) {                                                \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);\
			failures++;                                           \
		}                                                             \
	} while (0)

static void test_zero_is_zero(void)
{
	CHECK(gw_selftau_mm_to_dtu(0) == 0);
}

/* SS_INITIATOR_MM_PER_DTU_X1000 mm IS one DTU by construction -- this is the
 * conversion factor's own definition, and if it stops holding here the
 * constant was edited without checking this test, which is the point. */
static void test_one_dtu_round_trips(void)
{
	CHECK(gw_selftau_mm_to_dtu(SS_INITIATOR_MM_PER_DTU_X1000) == 1000);
	CHECK(gw_selftau_mm_to_dtu(2 * SS_INITIATOR_MM_PER_DTU_X1000) == 2000);
}

/* Hand-computed against SS_INITIATOR_MM_PER_DTU_X1000 == 4690: 1000 mm is
 * 213.219... DTU (rounds down to 213), 1002 mm is 213.646... DTU (rounds up
 * to 214) -- picked specifically to straddle a rounding boundary, so a
 * regression from round-to-nearest to plain truncation would flip the
 * second case but not the first. */
static void test_rounds_to_nearest_not_truncates(void)
{
	CHECK(SS_INITIATOR_MM_PER_DTU_X1000 == 4690);
	CHECK(gw_selftau_mm_to_dtu(1000) == 213);
	CHECK(gw_selftau_mm_to_dtu(1002) == 214);
}

static void test_monotonic(void)
{
	int32_t prev = gw_selftau_mm_to_dtu(0);

	for (int32_t mm = 1; mm <= 20000; mm += 37) {
		int32_t v = gw_selftau_mm_to_dtu(mm);

		CHECK(v >= prev);
		prev = v;
	}
}

int main(void)
{
	test_zero_is_zero();
	test_one_dtu_round_trips();
	test_rounds_to_nearest_not_truncates();
	test_monotonic();

	if (failures) {
		printf("FAILED (%d)\n", failures);
		return 1;
	}
	printf("gw_selftau: ALL TESTS PASSED\n");
	return 0;
}

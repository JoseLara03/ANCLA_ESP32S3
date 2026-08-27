/* Including ccp_sched.h IS the test: under tests/mac_budget/shim its
 * BUILD_ASSERTs are _Static_assert, so a budget that stops holding fails to
 * COMPILE under plain gcc instead of waiting for a Zephyr build. Same pattern
 * as tests/mac_budget/test_uwb_mac_asserts.c.
 *
 * The printed figures are not decoration: they are the numbers the design
 * decision in docs/superpowers/plans/2026-08-26-fase2-ccp-sync.md D2 was made
 * from, so a reader can confirm the table rather than trust it. */
#include "ccp_sched.h"

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);  \
			failures++;                                            \
		}                                                              \
	} while (0)

/* The offset is BEACON_OCCUPANCY_UUS by DERIVATION, not by coincidence. If
 * someone retunes it to a literal, this fails and says why. */
static void test_offset_is_the_beacon_occupancy(void)
{
	CHECK(CCP_OFFSET_UUS == BEACON_OCCUPANCY_UUS);
	CHECK(CCP_OFFSET_UUS == 1500u);
}

/* Both edges, with their margins pinned. A change that leaves the asserts
 * holding but eats the margin is worth seeing. */
static void test_ccp_fits_the_post_beacon_guard(void)
{
	uint32_t earliest = CCP_SCHED_BEACON_END_NS + CCP_SCHED_SHR_NS;
	uint32_t ends_at = CCP_SCHED_AT_NS +
			   CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	CHECK(CCP_SCHED_AT_NS >= earliest);
	CHECK(ends_at <= CCP_SCHED_GUARD_END_NS);

	/* Measured 2026-08-26. Exact values, not bounds: these are constant
	 * arithmetic over frozen PHY parameters, so anything that moves them
	 * moves the airtime model and should be read, not silently absorbed. */
	CHECK(CCP_SCHED_SHR_NS == 1050194u);
	CHECK(CCP_SCHED_BEACON_END_NS == 389411u);
	CHECK(earliest == 1439605u);
	CHECK(CCP_SCHED_AT_NS == 1538461u);
	CHECK(ends_at == 1777284u);
	CHECK(CCP_SCHED_GUARD_END_NS == 3076922u);

	CHECK(CCP_SCHED_AT_NS - earliest == 98856u);
	CHECK(CCP_SCHED_GUARD_END_NS - ends_at == 1299638u);
}

/* The CCP's total airtime and its share of a superframe. Quoted in
 * docs/anchor-sync-measurement.md section 2, so pin it here rather than letting
 * the doc drift. */
static void test_airtime_share_is_recorded(void)
{
	uint32_t full = CCP_SCHED_SHR_NS +
			CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	CHECK(full == 1289017u);
	/* 0.645% of a 200 ms superframe, to a tenth of a per mille. */
	CHECK((uint32_t)((uint64_t)full * 10000u /
			 MAC_UUS_TO_NS(T_SUPERFRAME_UUS)) == 64u);
}

int main(void)
{
	test_offset_is_the_beacon_occupancy();
	test_ccp_fits_the_post_beacon_guard();
	test_airtime_share_is_recorded();

	if (failures) {
		printf("FAILED %d\n", failures);
		return 1;
	}
	printf("ALL TESTS PASSED\n");
	return 0;
}

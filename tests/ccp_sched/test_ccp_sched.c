/* Including ccp_sched.h IS the test: under tests/mac_budget/shim its
 * BUILD_ASSERT is _Static_assert, so a budget that stops holding fails to
 * COMPILE under plain gcc instead of waiting for a Zephyr build. Same pattern
 * as tests/mac_budget/test_uwb_mac_asserts.c.
 *
 * The printed figures are not decoration: they are the numbers the design
 * decision in docs/superpowers/plans/2026-08-26-fase2-ccp-sync.md D2 was made
 * from, so a reader can confirm the table rather than trust it. Rewritten for
 * the immediate-TX design (Option B): the old asserts checked a delayed TX's
 * scheduled RMARKER against both edges of the guard window, which no longer
 * exists as a scheduled quantity. What is still checkable at compile time is
 * that the guard window is wide enough to host one whole CCP frame at all,
 * after the beacon's own airtime -- CCP_SCHED_MAX_ARM_NS. */
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

/* The airtime figures themselves, pinned so nothing here silently drifts.
 * These are constant arithmetic over frozen PHY parameters and the MAC
 * contract's guard sizing -- unrelated to how any one arm sequence performs
 * on hardware. Measured 2026-08-26 (unchanged by the Option B redesign: the
 * PHY and the guard geometry did not move, only how the CCP is scheduled). */
static void test_derived_quantities_are_pinned(void)
{
	/* PLEN_256 (2026-10-12): SHR = 264 * 1017.63 = 268654 ns (was 1050194
	 * at PLEN_1024). The beacon's post-RMARKER airtime and the guard
	 * window do not depend on the preamble, so they are unchanged;
	 * CAP_PREAMBLE = 3076922 - 268654 = 2808268 (was 2026728). */
	CHECK(CCP_SCHED_SHR_NS == 268654u);
	CHECK(CCP_SCHED_BEACON_END_NS == 389411u);
	CHECK(CCP_SCHED_GUARD_END_NS == 3076922u);
	CHECK(CCP_SCHED_CAP_PREAMBLE_NS == 2808268u);
}

/* The number this whole redesign turns on: how much of the post-beacon guard
 * window is left over for the CCP's own frame plus the immediate-TX arm
 * sequence, after subtracting the beacon's own airtime and the CCP's whole
 * SHR-through-FCS airtime. Sized against CCP_FRAME_LEN_V2 (30 bytes), not
 * the original CCP_FRAME_LEN (21): B14 Fase B (2026-09-24) made v2 the
 * ROUTINE per-superframe frame (src/ccp_master.c), so the budget this test
 * pins must reflect what is actually transmitted, not the smaller v1
 * frame. Was 348300 ns against v1; the v1 figure is still pinned just
 * below for anyone building a v1-only frame, unchanged since it is real
 * arithmetic over a real (if no longer routine) frame length. */
static void test_max_arm_ns_is_1826674(void)
{
	/* 2808268 - (268654 + 323529) - 389411 = 1826674 at PLEN_256
	 * (263594 at PLEN_1024; was 348300 against v1 originally). */
	CHECK(CCP_SCHED_MAX_ARM_NS == 1826674);

	uint32_t full_ccp = CCP_SCHED_SHR_NS +
			    CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN_V2);

	CHECK(CCP_SCHED_CAP_PREAMBLE_NS - CCP_SCHED_BEACON_END_NS - full_ccp ==
	      CCP_SCHED_MAX_ARM_NS);
}

/* The v1 frame's own airtime, pinned on its own terms (not part of
 * CCP_SCHED_MAX_ARM_NS anymore, see above) since ccp_frame_build() can
 * still build one and it is worth having a number to check it against. */
static void test_v1_airtime_share_is_recorded(void)
{
	uint32_t full = CCP_SCHED_SHR_NS +
			CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN);

	/* 268654 + 238823 (PHR + 23 B * 8 bit at 850k) = 507477 ns. */
	CHECK(full == 507477u);
	/* 0.254% of a 200 ms superframe, to a tenth of a per mille. */
	CHECK((uint32_t)((uint64_t)full * 10000u /
			 MAC_UUS_TO_NS(T_SUPERFRAME_UUS)) == 25u);
}

/* The v2 frame's airtime and its share of a superframe -- what is ACTUALLY
 * sent every superframe since B14 Fase B. Quoted in
 * docs/anchor-sync-measurement.md section 2, so pin it here rather than
 * letting the doc drift. */
static void test_v2_airtime_share_is_recorded(void)
{
	uint32_t full = CCP_SCHED_SHR_NS +
			CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN_V2);

	/* 268654 + 323529 (PHR + 32 B * 8 bit at 850k) = 592183 ns. */
	CHECK(full == 592183u);
	/* 0.296% of a 200 ms superframe, to a tenth of a per mille. */
	CHECK((uint32_t)((uint64_t)full * 10000u /
			 MAC_UUS_TO_NS(T_SUPERFRAME_UUS)) == 29u);
}

/* The deliberate delay between the beacon's RMARKER and the CCP's preamble
 * (2026-10-13). Pinned with its hi32 conversion, and with the margin the two
 * BUILD_ASSERTs in ccp_sched.h leave: the CCP frame must end before the first
 * slave CAP preamble, with the slowest measured slave re-arm (871 us) still
 * ahead of the preamble. */
static void test_ccp_delay_is_pinned(void)
{
	CHECK(CCP_TX_PREAMBLE_DELAY_NS == 1200000u);
	/* 1200000 ns / 4.006 ns per hi32 tick, truncated. */
	CHECK(CCP_TX_DELAY_HI32 == 299550u);

	uint32_t frame_end = CCP_TX_PREAMBLE_DELAY_NS + CCP_SCHED_SHR_NS +
			     CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN_V2);

	/* 1200000 + 268654 + 323529 = 1792183 ns after the beacon's RMARKER. */
	CHECK(frame_end == 1792183u);
	/* ~1.016 ms of room before the first slave CAP preamble. */
	CHECK(CCP_SCHED_CAP_PREAMBLE_NS - frame_end == 1016085u);
	/* Preamble starts after the slowest slave re-arm measured (871 us). */
	CHECK(CCP_TX_PREAMBLE_DELAY_NS > 871000u);
}

int main(void)
{
	test_derived_quantities_are_pinned();
	test_ccp_delay_is_pinned();
	test_max_arm_ns_is_1826674();
	test_v1_airtime_share_is_recorded();
	test_v2_airtime_share_is_recorded();

	if (failures) {
		printf("FAILED %d\n", failures);
		return 1;
	}
	printf("ALL TESTS PASSED\n");
	return 0;
}

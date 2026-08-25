/*
 * Host test for the budget BUILD_ASSERTs in src/uwb_mac.h.
 *
 * Including uwb_mac.h at all is the test: its two BUILD_ASSERTs become
 * _Static_assert via the shim, so a budget that no longer holds fails to
 * COMPILE here rather than waiting for a Zephyr build. The runtime checks
 * below then pin the derived slot count, so a change that keeps the asserts
 * satisfied but moves the number still gets noticed.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -Itests/mac_budget/shim \
 *       -o tests/mac_budget/test_uwb_mac_asserts.exe \
 *       tests/mac_budget/test_uwb_mac_asserts.c src/mac_budget.c
 */

#include <stdio.h>
#include <stdlib.h>
#include "uwb_mac.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

int main(void)
{
	/* The superframe really is 200.0 ms. */
	CHECK(MAC_UUS_TO_NS(T_SUPERFRAME_UUS) > 199990000u);
	CHECK(MAC_UUS_TO_NS(T_SUPERFRAME_UUS) < 200010000u);

	/* T_GUARD_UUS is 0.5 ms, and is NOT BEACON_GUARD_UUS. The two are
	 * different quantities that were easy to confuse; pinned so a future
	 * edit that unifies them has to argue with a test. */
	CHECK(MAC_UUS_TO_NS(T_GUARD_UUS) > 499000u);
	CHECK(MAC_UUS_TO_NS(T_GUARD_UUS) < 502000u);
	CHECK(T_GUARD_UUS != BEACON_GUARD_UUS);

	/* One CFP slot = a 4-anchor SS-TWR sweep, ~13.3 ms. */
	CHECK(UWB_MAC_CFP_SLOT_NS > 13300000u);
	CHECK(UWB_MAC_CFP_SLOT_NS < 13350000u);

	/* ~191.8 ms left for the CFP after beacon, guards and CAP. */
	CHECK(UWB_MAC_CFP_USABLE_NS > 191000000u);
	CHECK(UWB_MAC_CFP_USABLE_NS < 193000000u);

	/* The budget affords 14 ranging slots; the wire carries 11. The shipped
	 * constant is therefore conservative, with 3 slots of headroom -- and
	 * the BUILD_ASSERT above is what keeps it that way. */
	CHECK(UWB_MAC_CFP_SLOTS_FEASIBLE == 14u);
	CHECK(UWB_FRAME_N_CFP == 11u);
	CHECK(UWB_FRAME_N_CFP < UWB_MAC_CFP_SLOTS_FEASIBLE);

	printf("uwb_mac budget: slot=%u ns  usable=%u ns  feasible=%u  N_CFP=%u\n",
	       (unsigned)UWB_MAC_CFP_SLOT_NS, (unsigned)UWB_MAC_CFP_USABLE_NS,
	       (unsigned)UWB_MAC_CFP_SLOTS_FEASIBLE, (unsigned)UWB_FRAME_N_CFP);

	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("uwb_mac asserts: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

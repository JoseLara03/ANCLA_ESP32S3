/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host test for blink_sched.c. See
 * docs/superpowers/plans/2026-08-30-blink-slotted-mac.md Task 3.
 */

#include "blink_sched.h"
#include "mac_budget.h"

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                           \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

/* ---- Constants copied from uwb_mac.h / uwb_frame_802_15_4z.h -------------
 * Same convention tests/mac_budget/test_mac_budget.c uses: these live in
 * Zephyr-dependent headers this host test cannot include, and the
 * BUILD_ASSERTs in uwb_mac.h are what catches a divergence on target. */
#define T_SUPERFRAME_UUS        195000u
#define T_GUARD_NS              500000u    /* T_GUARD_UUS = 488, MAC contract */
#define UWB_FRAME_N_CFP         11u
#define UWB_FRAME_LEN_BEACON    (15u + 2u * UWB_FRAME_N_CFP)
#define UWB_FRAME_LEN_KEEPALIVE 12u
#define N_CAP                   4u

static void frozen_cell(struct mac_cell *c, const struct mac_phy *phy)
{
	c->superframe_ns = MAC_UUS_TO_NS(T_SUPERFRAME_UUS);
	c->beacon_ns     = mac_frame_ns(phy, UWB_FRAME_LEN_BEACON);
	c->guard_ns      = T_GUARD_NS;
	c->minislot_ns   = mac_frame_ns(phy, UWB_FRAME_LEN_KEEPALIVE) + 100000u;
	c->n_cap         = N_CAP;
}

static void test_slot_index_is_identity(void)
{
	for (int i = 0; i <= 127; i++) {
		CHECK(blink_sched_slot_index((uint8_t)i) == (uint8_t)i);
	}
}

static void test_admissible_below_n_slots(void)
{
	CHECK(blink_sched_seat_admissible(0, 100) == true);
	CHECK(blink_sched_seat_admissible(99, 100) == true);   /* k = n - 1 */
	CHECK(blink_sched_seat_admissible(100, 100) == false); /* k = n */
	CHECK(blink_sched_seat_admissible(127, 100) == false);
	CHECK(blink_sched_seat_admissible(0, 0) == false);
}

static void test_n_slots_matches_current_estimate(void)
{
	struct mac_phy phy;
	struct mac_cell cell;

	mac_phy_frozen(&phy);
	frozen_cell(&cell, &phy);

	uint16_t n = blink_sched_n_slots(&cell);

	printf("blink_sched_n_slots() = %u (BLINK_SLOT_GUARD_UUS=%u)\n",
	       (unsigned int)n, (unsigned int)BLINK_SLOT_GUARD_UUS);

	/* Deliberately a range, not an exact number -- pins the order of
	 * magnitude without coupling the test to a one-unit guard change.
	 *
	 * The design doc (section 1.4) estimated 96-106 for this guard before
	 * this function existed; running the actual mac_budget.h model (the
	 * same frozen_cell() shape tests/mac_budget/test_mac_budget.c already
	 * validates against hardware-informed numbers) gives 134. Per the
	 * plan's own instruction ("anotar el numero real... es la cifra de
	 * BLINK_N_SLOTS para el resto de este plan"), the measured figure is
	 * what counts, not the doc's pre-code guess -- so the range here is
	 * centered on the measured value, not the earlier estimate. */
	CHECK(n >= 100u && n <= 150u);
}

static void test_n_slots_shrinks_with_more_guard(void)
{
	struct mac_phy phy;
	struct mac_cell cell;

	mac_phy_frozen(&phy);
	frozen_cell(&cell, &phy);

	uint32_t base_slot_ns = blink_sched_slot_ns();
	uint16_t base_n       = mac_cell_max_slots(&cell, base_slot_ns);
	uint16_t more_guard_n = mac_cell_max_slots(&cell, base_slot_ns + 500000u);

	CHECK(more_guard_n <= base_n);
}

static void test_at_least_the_product_target(void)
{
	struct mac_phy phy;
	struct mac_cell cell;

	mac_phy_frozen(&phy);
	frozen_cell(&cell, &phy);

	CHECK(blink_sched_n_slots(&cell) >= 100u);
}

int main(void)
{
	test_slot_index_is_identity();
	test_admissible_below_n_slots();
	test_n_slots_matches_current_estimate();
	test_n_slots_shrinks_with_more_guard();
	test_at_least_the_product_target();

	printf(failures ? "FAILED (%d)\n" : "PASSED\n", failures);
	return failures ? 1 : 0;
}

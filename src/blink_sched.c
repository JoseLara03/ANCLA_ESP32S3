/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See blink_sched.h.
 */

#include "blink_sched.h"
#include "blink_frame.h"

uint32_t blink_sched_slot_ns(void)
{
	struct mac_phy phy;

	mac_phy_frozen(&phy);
	return mac_frame_ns(&phy, BLINK_FRAME_LEN) +
	       MAC_UUS_TO_NS(BLINK_SLOT_GUARD_UUS);
}

uint16_t blink_sched_n_slots(const struct mac_cell *cell)
{
	return mac_cell_max_slots(cell, blink_sched_slot_ns());
}

uint8_t blink_sched_slot_index(uint8_t seat_id)
{
	return seat_id;
}

bool blink_sched_seat_admissible(uint8_t seat_id, uint16_t n_slots)
{
	return (uint16_t)seat_id < n_slots;
}

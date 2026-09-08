/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "disc_schedule.h"

uint32_t disc_resp_delay_uus(uint8_t anchor_id)
{
	return DISC_BASE_UUS + (uint32_t)anchor_id * DISC_SLOT_UUS;
}

bool disc_group_match(uint8_t anchor_id, uint8_t group, uint8_t n_groups)
{
	if (n_groups == 0) {
		return false;
	}
	return (uint8_t)(anchor_id % n_groups) == group;
}

bool disc_resp_delay_uus_grouped(uint8_t anchor_id, uint8_t n_groups, uint32_t *delay_uus)
{
	if (n_groups == 0) {
		return false;
	}
	uint32_t rank = (uint32_t)anchor_id / n_groups;

	if (rank > DISC_MAX_RANK) {
		return false;
	}
	*delay_uus = DISC_BASE_UUS + rank * DISC_SLOT_UUS;
	return true;
}

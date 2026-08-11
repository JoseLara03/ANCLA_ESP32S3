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

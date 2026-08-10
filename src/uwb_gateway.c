/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATEWAY mode: TDMA beacon plus CAP seat granting.
 * Stub — the beacon and gw_core land in spec D.
 */

#include "uwb_modes.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uwb_gateway, LOG_LEVEL_INF);

void uwb_gateway_run(const uwb_config_t *cfg)
{
	if (!cfg->position_valid) {
		LOG_ERR("gateway not positioned — set `anchor pos <x> <y> <z>` first");
	}

	LOG_INF("GATEWAY mode, anchor_id=%u — beacon not implemented yet",
		cfg->anchor_id);

	while (1) {
		k_sleep(K_FOREVER);
	}
}

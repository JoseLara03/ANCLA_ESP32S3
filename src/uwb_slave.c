/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SLAVE mode: SS-TWR responder that also observes the gateway beacon.
 * Stub — the responder lands in spec C.
 */

#include "uwb_modes.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uwb_slave, LOG_LEVEL_INF);

void uwb_slave_run(const uwb_config_t *cfg)
{
	LOG_INF("SLAVE mode, anchor_id=%u — responder not implemented yet",
		cfg->anchor_id);

	while (1) {
		k_sleep(K_FOREVER);
	}
}

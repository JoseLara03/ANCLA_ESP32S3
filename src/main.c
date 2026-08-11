/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor boot: load the persisted configuration, bring the DW3220 up with it,
 * and hand the main thread to the configured mode.
 *
 * A radio failure deliberately does not halt the system — the shell stays
 * alive so a mis-set antenna delay or a wiring fault is recoverable over USB
 * without a reflash.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "uwb_config.h"
#include "uwb_modes.h"
#include "uwb_radio.h"
#include "uwb_store.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void log_config(const uwb_config_t *cfg)
{
	LOG_INF("{\"mode\":\"%s\",\"id\":%u,\"short_addr\":\"0x%04X\","
		"\"ant_tx\":%u,\"ant_rx\":%u,"
		"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		uwb_config_short_addr(cfg),
		cfg->ant_delay_tx, cfg->ant_delay_rx,
		(double)cfg->x, (double)cfg->y, (double)cfg->z,
		cfg->position_valid ? 1u : 0u);
}

int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	uwb_store_init();
	log_config(cfg);

	ret = uwb_radio_init(cfg);
	if (ret) {
		LOG_ERR("radio bring-up failed (%d) — shell stays up, not entering a mode",
			ret);
		return ret;
	}

	if (cfg->mode == UWB_MODE_GATEWAY) {
		uwb_gateway_run(cfg);
	} else {
		uwb_slave_run(cfg);
	}

	return 0;
}

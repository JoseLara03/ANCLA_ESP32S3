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

#include "apos_store.h"
#include "disc_schedule.h"
#include "net_config.h"
#include "net_uplink.h"
#include "uwb_config.h"
#include "uwb_modes.h"
#include "uwb_radio.h"
#include "uwb_store.h"

#ifdef CONFIG_ANCLA_CAL_MODE
#include "cal_run.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void log_config(const uwb_config_t *cfg)
{
	/* disc_delay_uus is derived from anchor_id, not stored -- printed because
	 * it is the number that actually decides whether two boards collide.
	 * Three consoles side by side must show three different values here; two
	 * boards sharing one answer the tag's DISCOVERY broadcast in the same
	 * slot, and neither board can detect that locally. */
	LOG_INF("{\"mode\":\"%s\",\"id\":%u,\"short_addr\":\"0x%04X\","
		"\"ant_tx\":%u,\"ant_rx\":%u,"
		"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u,"
		"\"disc_delay_uus\":%u}",
		uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		uwb_config_short_addr(cfg),
		cfg->ant_delay_tx, cfg->ant_delay_rx,
		(double)cfg->x, (double)cfg->y, (double)cfg->z,
		cfg->position_valid ? 1u : 0u,
		disc_resp_delay_uus(cfg->anchor_id));
}

int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	/* Before uwb_store_init(): its settings_load() runs the "net" handler,
	 * which writes into this singleton. Explicit rather than lazy because
	 * net_uplink is a second thread (see net_config.h). */
	net_config_init();

	uwb_store_init();
	/* After uwb_store_init(): its settings_load() is what runs this
	 * module's handler and fills the cache. Before net_uplink_start():
	 * the uplink publishes the anchors payload on connect and reads it. */
	apos_store_init();
	log_config(cfg);

	ret = uwb_radio_init(cfg);
	if (ret) {
		LOG_ERR("radio bring-up failed (%d) — shell stays up, not entering a mode",
			ret);
		return ret;
	}

#ifdef CONFIG_ANCLA_CAL_MODE
	/* The calibration image ignores the configured mode entirely: it is
	 * neither a SLAVE nor a GATEWAY, and it must not beacon. */
	cal_run(cfg);
#else
	if (cfg->mode == UWB_MODE_GATEWAY) {
		/* Run the beacon loop cooperatively so no network thread can
		 * preempt a delayed-TX arm. The loop already yields at its
		 * k_sem_take(), which is where every other thread gets time.
		 *
		 * This makes any unbounded busy-wait on this path fatal: no
		 * lower-priority thread -- including the shell -- can ever run
		 * again. Every spin here must be bounded (see
		 * uwb_wait_for_sysstatus_lo).
		 *
		 * SLAVE mode is deliberately left at the default priority: it
		 * runs no network thread, so the change would buy nothing and
		 * would perturb a bench-confirmed path. */
		k_thread_priority_set(k_current_get(), K_PRIO_COOP(0));
		net_uplink_start();
		uwb_gateway_run(cfg);
	} else {
		uwb_slave_run(cfg);
	}
#endif

	return 0;
}

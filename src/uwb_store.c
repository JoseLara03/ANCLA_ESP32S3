/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-field settings keys under the "anchor" subtree, rather than one struct
 * blob: specs C and D will add fields, and a blob would force a version byte
 * plus a wipe-to-defaults on every layout change. Unknown keys are ignored
 * and their fields keep their defaults.
 *
 * A stored value that fails validation falls back to its default and is
 * logged; the other stored fields are still applied.
 */

#include "uwb_store.h"
#include "uwb_config.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(uwb_store, LOG_LEVEL_INF);

#define KEY_MODE   "anchor/mode"
#define KEY_ID     "anchor/id"
#define KEY_ANT_TX "anchor/ant_tx"
#define KEY_ANT_RX "anchor/ant_rx"
#define KEY_POS    "anchor/pos"

/* Written as one record so x, y, z and the valid flag can never disagree. */
struct stored_pos {
	float   x;
	float   y;
	float   z;
	uint8_t valid;
};

static int read_val(settings_read_cb read_cb, void *cb_arg, void *dst, size_t len)
{
	ssize_t n = read_cb(cb_arg, dst, len);

	if (n != (ssize_t)len) {
		return -EINVAL;
	}
	return 0;
}

static int anchor_settings_set(const char *key, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(len);

	if (strcmp(key, "mode") == 0) {
		uint8_t v;

		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_mode(cfg, v)) {
			LOG_WRN("stored mode %u invalid — keeping %s", v,
				uwb_config_mode_name(cfg->mode));
		}
		return 0;
	}

	if (strcmp(key, "id") == 0) {
		uint8_t v;

		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_id(cfg, v)) {
			LOG_WRN("stored id %u invalid — keeping %u", v, cfg->anchor_id);
		}
		return 0;
	}

	if (strcmp(key, "ant_tx") == 0) {
		return read_val(read_cb, cb_arg, &cfg->ant_delay_tx,
				sizeof(cfg->ant_delay_tx));
	}

	if (strcmp(key, "ant_rx") == 0) {
		return read_val(read_cb, cb_arg, &cfg->ant_delay_rx,
				sizeof(cfg->ant_delay_rx));
	}

	if (strcmp(key, "pos") == 0) {
		struct stored_pos p;

		if (read_val(read_cb, cb_arg, &p, sizeof(p))) {
			return -EINVAL;
		}
		if (p.valid) {
			uwb_config_set_pos(cfg, p.x, p.y, p.z);
		}
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(anchor, "anchor", NULL, anchor_settings_set,
			       NULL, NULL);

int uwb_store_init(void)
{
	int ret;

	ret = settings_subsys_init();
	if (ret) {
		LOG_WRN("settings_subsys_init failed (%d) — running on defaults", ret);
		return ret;
	}

	ret = settings_load();
	if (ret) {
		LOG_WRN("settings_load failed (%d) — running on defaults", ret);
		return ret;
	}

	return 0;
}

static void save_one(const char *key, const void *val, size_t len)
{
	int ret = settings_save_one(key, val, len);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", key, ret);
	}
}

void uwb_store_save_mode(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_MODE, &cfg->mode, sizeof(cfg->mode));
}

void uwb_store_save_id(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_ID, &cfg->anchor_id, sizeof(cfg->anchor_id));
}

void uwb_store_save_ant(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_ANT_TX, &cfg->ant_delay_tx, sizeof(cfg->ant_delay_tx));
	save_one(KEY_ANT_RX, &cfg->ant_delay_rx, sizeof(cfg->ant_delay_rx));
}

void uwb_store_save_pos(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	struct stored_pos p = {
		.x     = cfg->x,
		.y     = cfg->y,
		.z     = cfg->z,
		.valid = cfg->position_valid ? 1u : 0u,
	};

	save_one(KEY_POS, &p, sizeof(p));
}

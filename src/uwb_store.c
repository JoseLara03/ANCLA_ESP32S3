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

#define KEY_MODE      "anchor/mode"
#define KEY_ID        "anchor/id"
#define KEY_CELL_MODE "anchor/cell_mode"
#define KEY_ANT_TX    "anchor/ant_tx"
#define KEY_ANT_RX    "anchor/ant_rx"
#define KEY_POS       "anchor/pos"
#define KEY_SYNC_TAU      "anchor/sync_tau"
#define KEY_SYNC_TAU_MODE "anchor/sync_tau_mode"
#define KEY_SYNC_LINK     "anchor/sync_link"

/* Static sync link: valid(1) + generation(4, LE) + parent EUI(8) = 13 bytes,
 * serialised by hand so struct padding never decides the stored layout. A
 * separate key from sync_tau on purpose -- see uwb_store_save_sync_link(). */
#define SYNC_LINK_REC_LEN 13u

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

	if (strcmp(key, "mode") == 0) {
		uint8_t v;

		if (len != sizeof(v)) {
			LOG_WRN("stored mode size %u invalid — expected %u, keeping %s",
				(unsigned int)len, (unsigned int)sizeof(v),
				uwb_config_mode_name(cfg->mode));
			return -EINVAL;
		}
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

		if (len != sizeof(v)) {
			LOG_WRN("stored id size %u invalid — expected %u, keeping %u",
				(unsigned int)len, (unsigned int)sizeof(v), cfg->anchor_id);
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_id(cfg, v)) {
			LOG_WRN("stored id %u invalid — keeping %u", v, cfg->anchor_id);
		}
		return 0;
	}

	if (strcmp(key, "cell_mode") == 0) {
		uint8_t v;

		if (len != sizeof(v)) {
			LOG_WRN("stored cell_mode size %u invalid — expected %u, keeping %s",
				(unsigned int)len, (unsigned int)sizeof(v),
				uwb_config_cell_mode_name(cfg->cell_mode));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_cell_mode(cfg, v)) {
			LOG_WRN("stored cell_mode %u invalid — keeping %s", v,
				uwb_config_cell_mode_name(cfg->cell_mode));
		}
		return 0;
	}

	if (strcmp(key, "ant_tx") == 0) {
		if (len != sizeof(cfg->ant_delay_tx)) {
			LOG_WRN("stored ant_tx size %u invalid — expected %u, keeping %u",
				(unsigned int)len, (unsigned int)sizeof(cfg->ant_delay_tx),
				cfg->ant_delay_tx);
			return -EINVAL;
		}
		return read_val(read_cb, cb_arg, &cfg->ant_delay_tx,
				sizeof(cfg->ant_delay_tx));
	}

	if (strcmp(key, "ant_rx") == 0) {
		if (len != sizeof(cfg->ant_delay_rx)) {
			LOG_WRN("stored ant_rx size %u invalid — expected %u, keeping %u",
				(unsigned int)len, (unsigned int)sizeof(cfg->ant_delay_rx),
				cfg->ant_delay_rx);
			return -EINVAL;
		}
		return read_val(read_cb, cb_arg, &cfg->ant_delay_rx,
				sizeof(cfg->ant_delay_rx));
	}

	if (strcmp(key, "pos") == 0) {
		struct stored_pos p;

		if (len != sizeof(p)) {
			LOG_WRN("stored pos size %u invalid — expected %u, keeping defaults",
				(unsigned int)len, (unsigned int)sizeof(p));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &p, sizeof(p))) {
			return -EINVAL;
		}
		if (p.valid) {
			uwb_config_set_pos(cfg, p.x, p.y, p.z);
		}
		return 0;
	}

	if (strcmp(key, "sync_tau") == 0) {
		if (len != sizeof(cfg->sync_tau_dtu)) {
			LOG_WRN("stored sync_tau size %u invalid — expected %u, keeping %d",
				(unsigned int)len, (unsigned int)sizeof(cfg->sync_tau_dtu),
				cfg->sync_tau_dtu);
			return -EINVAL;
		}
		return read_val(read_cb, cb_arg, &cfg->sync_tau_dtu,
				sizeof(cfg->sync_tau_dtu));
	}

	if (strcmp(key, "sync_tau_mode") == 0) {
		uint8_t v;

		if (len != sizeof(v)) {
			LOG_WRN("stored sync_tau_mode size %u invalid — expected %u, keeping %s",
				(unsigned int)len, (unsigned int)sizeof(v),
				uwb_config_sync_tau_mode_name(cfg->sync_tau_mode));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		bool migrated;
		uint8_t use = uwb_config_migrate_sync_tau_mode(v, &migrated);

		/* TWR/AUTO were retired 2026-10-12. A record written by older
		 * firmware is read for MIGRATION only: it becomes SURVEY in RAM, the
		 * stored tau is kept (it was a survey value all along), and nothing
		 * on the air is started on its account. Not re-persisted here -- a
		 * settings write inside the load callback is not safe -- so it is
		 * migrated again, identically, on every boot until the next
		 * `sync tau mode` or `apos apply` rewrites it. */
		if (migrated) {
			LOG_WRN("stored sync_tau_mode %u is retired or invalid — using "
				"survey (the stored tau is kept; run `apos apply` to "
				"install a surveyed link)", v);
		}
		if (!uwb_config_set_sync_tau_mode(cfg, use)) {
			LOG_WRN("stored sync_tau_mode %u invalid — keeping %s", use,
				uwb_config_sync_tau_mode_name(cfg->sync_tau_mode));
		}
		return 0;
	}

	if (strcmp(key, "sync_link") == 0) {
		uint8_t rec[SYNC_LINK_REC_LEN];

		if (len != sizeof(rec)) {
			LOG_WRN("stored sync_link size %u invalid — expected %u, "
				"treating as no installed link",
				(unsigned int)len, (unsigned int)sizeof(rec));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, rec, sizeof(rec))) {
			return -EINVAL;
		}
		if (rec[0] == 1u) {
			uint32_t gen = (uint32_t)rec[1] | ((uint32_t)rec[2] << 8) |
				       ((uint32_t)rec[3] << 16) |
				       ((uint32_t)rec[4] << 24);

			uwb_config_set_sync_link(cfg, gen, &rec[5]);
		} else {
			uwb_config_clear_sync_link(cfg);
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

static int save_one(const char *key, const void *val, size_t len)
{
	int ret = settings_save_one(key, val, len);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", key, ret);
	}
	return ret;
}

int uwb_store_save_mode(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	return save_one(KEY_MODE, &cfg->mode, sizeof(cfg->mode));
}

int uwb_store_save_id(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	return save_one(KEY_ID, &cfg->anchor_id, sizeof(cfg->anchor_id));
}

int uwb_store_save_cell_mode(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	return save_one(KEY_CELL_MODE, &cfg->cell_mode, sizeof(cfg->cell_mode));
}

int uwb_store_save_ant(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret_tx, ret_rx;

	/* Both keys are always attempted, even if the first fails, so a bad
	 * tx write does not skip the rx write. Report the first failure. */
	ret_tx = save_one(KEY_ANT_TX, &cfg->ant_delay_tx, sizeof(cfg->ant_delay_tx));
	ret_rx = save_one(KEY_ANT_RX, &cfg->ant_delay_rx, sizeof(cfg->ant_delay_rx));

	return ret_tx ? ret_tx : ret_rx;
}

int uwb_store_save_pos(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	struct stored_pos p = {
		.x     = cfg->x,
		.y     = cfg->y,
		.z     = cfg->z,
		.valid = cfg->position_valid ? 1u : 0u,
	};

	return save_one(KEY_POS, &p, sizeof(p));
}

int uwb_store_save_sync_tau(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	return save_one(KEY_SYNC_TAU, &cfg->sync_tau_dtu, sizeof(cfg->sync_tau_dtu));
}

int uwb_store_save_sync_tau_mode(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	return save_one(KEY_SYNC_TAU_MODE, &cfg->sync_tau_mode, sizeof(cfg->sync_tau_mode));
}

int uwb_store_save_sync_link(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	uint8_t rec[SYNC_LINK_REC_LEN];

	rec[0] = cfg->sync_link_valid ? 1u : 0u;
	rec[1] = (uint8_t)cfg->sync_link_gen;
	rec[2] = (uint8_t)(cfg->sync_link_gen >> 8);
	rec[3] = (uint8_t)(cfg->sync_link_gen >> 16);
	rec[4] = (uint8_t)(cfg->sync_link_gen >> 24);
	memcpy(&rec[5], cfg->sync_parent_eui, 8);

	return save_one(KEY_SYNC_LINK, rec, sizeof(rec));
}

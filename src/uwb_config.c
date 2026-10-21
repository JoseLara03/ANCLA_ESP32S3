/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Config defaults, validation and the single active instance.
 */

#include "uwb_config.h"

#include <string.h>

static const char *const mode_names[UWB_MODE_COUNT] = {
	[UWB_MODE_SLAVE]   = "slave",
	[UWB_MODE_GATEWAY] = "gateway",
};

static const char *const cell_mode_names[UWB_CELL_MODE_COUNT] = {
	[UWB_CELL_TWR]   = "twr",
	[UWB_CELL_BLINK] = "blink",
};

static const char *const sync_tau_mode_names[SYNC_TAU_MODE_COUNT] = {
	[SYNC_TAU_MODE_LEGACY] = "legacy",
	[SYNC_TAU_MODE_SURVEY] = "survey",
	[SYNC_TAU_MODE_TWR]    = "twr",
	[SYNC_TAU_MODE_AUTO]   = "auto",
};

void uwb_config_set_defaults(uwb_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->mode         = UWB_MODE_SLAVE;
	/* No id until a gateway assigns one (`apos assign`) or the console sets
	 * one: a factory-fresh board must not claim id 0, which a second
	 * factory-fresh board would claim too. */
	c->anchor_id    = UWB_ANCHOR_ID_UNASSIGNED;
	c->cell_mode    = UWB_CELL_TWR;
	c->ant_delay_tx = UWB_ANT_DELAY_DEFAULT;
	c->ant_delay_rx = UWB_ANT_DELAY_DEFAULT;
	/* x/y/z zeroed above; position_valid stays false so a GATEWAY that has
	 * never been positioned refuses to beacon rather than beaconing from
	 * a bogus origin. */
	c->sync_tau_dtu  = 0;
	/* SURVEY, not LEGACY: an unconfigured board (sync_tau_dtu == 0) behaves
	 * identically to LEGACY either way, but this is the mode an operator
	 * expects to already be selected once the survey tool pushes a real
	 * value -- FR-11's rollback path is `sync tau mode legacy`, not the
	 * factory default. */
	c->sync_tau_mode = SYNC_TAU_MODE_SURVEY;
}

uwb_config_t *uwb_config_get(void)
{
	static uwb_config_t cfg;
	static bool initialised;

	if (!initialised) {
		uwb_config_set_defaults(&cfg);
		initialised = true;
	}
	return &cfg;
}

bool uwb_config_set_mode(uwb_config_t *c, uint8_t mode)
{
	if (mode >= UWB_MODE_COUNT) {
		return false;
	}
	c->mode = mode;
	return true;
}

bool uwb_config_set_id(uwb_config_t *c, uint8_t id)
{
	if (id >= UWB_MAX_ANCHORS && id != UWB_ANCHOR_ID_UNASSIGNED) {
		return false;
	}
	c->anchor_id = id;
	return true;
}

bool uwb_config_is_assigned(const uwb_config_t *c)
{
	return c->anchor_id < UWB_MAX_ANCHORS;
}

bool uwb_config_set_cell_mode(uwb_config_t *c, uint8_t cell_mode)
{
	if (cell_mode >= UWB_CELL_MODE_COUNT) {
		return false;
	}
	c->cell_mode = cell_mode;
	return true;
}

bool uwb_config_set_ant(uwb_config_t *c, uint32_t tx, uint32_t rx)
{
	/* Both are validated before either is written, so a rejected pair
	 * leaves the config exactly as it was. */
	if (tx > UINT16_MAX || rx > UINT16_MAX) {
		return false;
	}
	c->ant_delay_tx = (uint16_t)tx;
	c->ant_delay_rx = (uint16_t)rx;
	return true;
}

void uwb_config_set_pos(uwb_config_t *c, float x, float y, float z)
{
	c->x = x;
	c->y = y;
	c->z = z;
	c->position_valid = true;
}

void uwb_config_set_sync_tau(uwb_config_t *c, int32_t tau_dtu)
{
	c->sync_tau_dtu = tau_dtu;
}

bool uwb_config_set_sync_tau_mode(uwb_config_t *c, uint8_t mode)
{
	/* TWR and AUTO are retired (2026-10-12): there is no on-air source left to
	 * track, and accepting them would let a stored or typed value select a
	 * mode that silently behaves as SURVEY. */
	if (mode != SYNC_TAU_MODE_LEGACY && mode != SYNC_TAU_MODE_SURVEY) {
		return false;
	}
	c->sync_tau_mode = mode;
	return true;
}

bool uwb_config_mode_from_name(const char *name, uint8_t *out)
{
	for (uint8_t i = 0; i < UWB_MODE_COUNT; i++) {
		if (strcmp(name, mode_names[i]) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

const char *uwb_config_mode_name(uint8_t mode)
{
	if (mode >= UWB_MODE_COUNT) {
		return "unknown";
	}
	return mode_names[mode];
}

bool uwb_config_cell_mode_from_name(const char *name, uint8_t *out)
{
	for (uint8_t i = 0; i < UWB_CELL_MODE_COUNT; i++) {
		if (strcmp(name, cell_mode_names[i]) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

const char *uwb_config_cell_mode_name(uint8_t cell_mode)
{
	if (cell_mode >= UWB_CELL_MODE_COUNT) {
		return "unknown";
	}
	return cell_mode_names[cell_mode];
}

bool uwb_config_sync_tau_mode_from_name(const char *name, uint8_t *out)
{
	for (uint8_t i = 0; i < SYNC_TAU_MODE_COUNT; i++) {
		if (strcmp(name, sync_tau_mode_names[i]) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

const char *uwb_config_sync_tau_mode_name(uint8_t mode)
{
	if (mode >= SYNC_TAU_MODE_COUNT) {
		return "unknown";
	}
	return sync_tau_mode_names[mode];
}

int32_t uwb_config_sync_tau_effective(const uwb_config_t *c)
{
	switch (c->sync_tau_mode) {
	case SYNC_TAU_MODE_LEGACY:
		return 0;
	case SYNC_TAU_MODE_SURVEY:
	case SYNC_TAU_MODE_TWR:  /* Fase B stub: no TWR source yet, degrade */
	case SYNC_TAU_MODE_AUTO: /* to the stored survey value -- see the   */
		return c->sync_tau_dtu; /* enum's own comment.               */
	default:
		return 0;
	}
}

uint16_t uwb_config_short_addr(const uwb_config_t *c)
{
	/* A gateway's real address is the reserved 0x0000, not base+anchor_id --
	 * anchor_id on a gateway config is meaningless (never set from the
	 * console) and defaults to 0, which previously mapped to the SAME
	 * address as a genuine anchor id 0. Every caller of this function
	 * (net_uplink.c's MQTT client id, `anchor show`, ...) needs the real
	 * address regardless of role, so the branch belongs here once rather
	 * than being duplicated at each call site. */
	if (c->mode == UWB_MODE_GATEWAY) {
		return UWB_ADDR_GATEWAY_RESERVED;
	}
	/* Explicitly, never through the sum: BASE + 0xFF is the first tag
	 * address. */
	if (!uwb_config_is_assigned(c)) {
		return UWB_ANCHOR_ADDR_UNASSIGNED;
	}
	return (uint16_t)(UWB_ANCHOR_ADDR_BASE + c->anchor_id);
}

void uwb_config_set_sync_link(uwb_config_t *c, uint32_t generation,
			      const uint8_t parent_eui[8])
{
	c->sync_link_valid = true;
	c->sync_link_gen = generation;
	memcpy(c->sync_parent_eui, parent_eui, sizeof(c->sync_parent_eui));
}

void uwb_config_clear_sync_link(uwb_config_t *c)
{
	c->sync_link_valid = false;
	c->sync_link_gen = 0;
	memset(c->sync_parent_eui, 0, sizeof(c->sync_parent_eui));
}

uint8_t uwb_config_migrate_sync_tau_mode(uint8_t stored, bool *migrated)
{
	bool m = false;
	uint8_t out = stored;

	if (stored != SYNC_TAU_MODE_LEGACY && stored != SYNC_TAU_MODE_SURVEY) {
		out = SYNC_TAU_MODE_SURVEY;
		m = true;
	}
	if (migrated) {
		*migrated = m;
	}
	return out;
}

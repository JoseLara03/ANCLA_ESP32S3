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

void uwb_config_set_defaults(uwb_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->mode         = UWB_MODE_SLAVE;
	c->anchor_id    = 0;
	c->cell_mode    = UWB_CELL_TWR;
	c->ant_delay_tx = UWB_ANT_DELAY_DEFAULT;
	c->ant_delay_rx = UWB_ANT_DELAY_DEFAULT;
	/* x/y/z zeroed above; position_valid stays false so a GATEWAY that has
	 * never been positioned refuses to beacon rather than beaconing from
	 * a bogus origin. */
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
	if (id >= UWB_MAX_ANCHORS) {
		return false;
	}
	c->anchor_id = id;
	return true;
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
	return (uint16_t)(UWB_ANCHOR_ADDR_BASE + c->anchor_id);
}

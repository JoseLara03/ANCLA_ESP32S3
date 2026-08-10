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

void uwb_config_set_defaults(uwb_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->mode         = UWB_MODE_SLAVE;
	c->anchor_id    = 0;
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

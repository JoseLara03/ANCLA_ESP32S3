/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See node_eui.h.
 */

#include "node_eui.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(node_eui, LOG_LEVEL_INF);

static uint8_t eui[NODE_EUI_LEN];
static char eui_hex[NODE_EUI_HEX_LEN + 1u];
static bool valid;
static bool done;

void node_eui_init(void)
{
	if (done) {
		return;
	}
	done = true;

	memset(eui, 0, sizeof(eui));

	ssize_t n = hwinfo_get_device_id(eui, sizeof(eui));

	if (n <= 0) {
		memset(eui, 0, sizeof(eui));
		valid = false;
		LOG_ERR("hwinfo_get_device_id failed (%d) — this board has no "
			"unique id: it cannot take part in a survey, be assigned an "
			"address, or publish under its own identity", (int)n);
	} else if (node_eui_is_zero(eui)) {
		valid = false;
		LOG_ERR("hwinfo returned an all-zero id — unprogrammed eFuse? This "
			"board would share its identity with every other such board");
	} else {
		valid = true;
	}
	node_eui_fmt(eui, eui_hex);
}

bool node_eui_valid(void)
{
	node_eui_init();
	return valid;
}

const uint8_t *node_eui(void)
{
	node_eui_init();
	return eui;
}

const char *node_eui_hex(void)
{
	node_eui_init();
	return eui_hex;
}

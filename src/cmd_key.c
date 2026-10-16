/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "cmd_key.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "hmac_sha256.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(cmd_key, LOG_LEVEL_INF);

#define KEY_NAME "cmd/key"

static uint8_t key_buf[CMD_KEY_MAX];
static size_t  key_len;

const uint8_t *cmd_key_get(size_t *len)
{
	if (len != NULL) {
		*len = key_len;
	}
	return key_len == 0u ? NULL : key_buf;
}

bool cmd_key_is_set(void)
{
	return key_len != 0u;
}

void cmd_key_fingerprint(char out[9])
{
	uint8_t h[32];

	if (key_len == 0u) {
		out[0] = 0;
		return;
	}
	ancla_sha256(key_buf, key_len, h);
	snprintf(out, 9, "%02x%02x%02x%02x", h[0], h[1], h[2], h[3]);
}

static int cmd_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(key, "key") != 0) {
		return 0;                       /* a field from a newer firmware: ignore */
	}
	if (len < CMD_KEY_MIN || len > CMD_KEY_MAX) {
		LOG_WRN("stored command key has size %u — ignoring it (commands stay disabled)",
			(unsigned int)len);
		return -EINVAL;
	}
	if (read_cb(cb_arg, key_buf, len) != (ssize_t)len) {
		return -EINVAL;
	}
	key_len = len;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(cmd, "cmd", NULL, cmd_settings_set, NULL, NULL);

int cmd_key_set(const uint8_t *key, size_t len)
{
	int ret;

	if (key == NULL || len < CMD_KEY_MIN || len > CMD_KEY_MAX) {
		return -EINVAL;
	}
	ret = settings_save_one(KEY_NAME, key, len);
	if (ret != 0) {
		LOG_ERR("failed to persist the command key (%d)", ret);
		return ret;
	}
	memcpy(key_buf, key, len);
	key_len = len;
	return 0;
}

int cmd_key_clear(void)
{
	int ret = settings_delete(KEY_NAME);

	if (ret != 0) {
		LOG_ERR("failed to erase the command key (%d)", ret);
		return ret;
	}
	memset(key_buf, 0, sizeof(key_buf));
	key_len = 0u;
	return 0;
}

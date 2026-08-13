/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_config.h"

#include <string.h>

static net_config_t g_cfg;

/* Copy `src` into a field of capacity `cap` (excluding NUL) if its length is
 * within [min_len, cap]. Returns false without touching `dst` otherwise, which
 * is what gives every setter its all-or-nothing contract. */
static bool set_str(char *dst, size_t cap, const char *src, size_t min_len)
{
	size_t n;

	if (src == NULL) {
		return false;
	}

	n = strlen(src);
	if (n < min_len || n > cap) {
		return false;
	}

	memcpy(dst, src, n);
	dst[n] = '\0';
	return true;
}

void net_config_set_defaults(net_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->port = NET_MQTT_PORT_DEFAULT;
}

void net_config_init(void)
{
	net_config_set_defaults(&g_cfg);
}

net_config_t *net_config_get(void)
{
	return &g_cfg;
}

bool net_config_set_ssid(net_config_t *c, const char *ssid)
{
	return set_str(c->ssid, NET_SSID_MAX, ssid, 1);
}

bool net_config_set_psk(net_config_t *c, const char *psk)
{
	return set_str(c->psk, NET_PSK_MAX, psk, NET_PSK_MIN);
}

bool net_config_set_broker(net_config_t *c, const char *host, uint32_t port)
{
	char scratch[NET_BROKER_MAX + 1];

	/* Validate both fields before writing either: a rejected port must not
	 * leave a half-applied broker behind. */
	if (port == 0u || port > 65535u) {
		return false;
	}
	if (!set_str(scratch, NET_BROKER_MAX, host, 1)) {
		return false;
	}

	memcpy(c->broker, scratch, sizeof(scratch));
	c->port = (uint16_t)port;
	return true;
}

bool net_config_set_user(net_config_t *c, const char *user)
{
	return set_str(c->mqtt_user, NET_MQTT_USER_MAX, user, 0);
}

bool net_config_set_mqtt_pass(net_config_t *c, const char *pass)
{
	return set_str(c->mqtt_pass, NET_MQTT_PASS_MAX, pass, 0);
}

bool net_config_is_provisioned(const net_config_t *c)
{
	return c->ssid[0] != '\0' && c->broker[0] != '\0' && c->port != 0u;
}

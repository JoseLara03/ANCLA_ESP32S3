/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_store.h"
#include "net_config.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(net_store, LOG_LEVEL_INF);

#define KEY_SSID     "net/ssid"
#define KEY_PSK      "net/psk"
#define KEY_BROKER   "net/broker"
#define KEY_PORT     "net/port"
#define KEY_USER     "net/user"
#define KEY_MQTTPASS "net/mqttpass"

/* Read a stored NUL-terminated string into `dst` via the validating setter
 * `apply`. Rejects anything that does not fit the scratch buffer or that the
 * setter refuses, leaving the field at its default. */
static int load_str(settings_read_cb read_cb, void *cb_arg, size_t len,
		    const char *name,
		    bool (*apply)(net_config_t *, const char *))
{
	char scratch[NET_PSK_MAX + 2]; /* the longest field plus NUL */
	ssize_t n;

	if (len == 0u || len > sizeof(scratch)) {
		LOG_WRN("stored %s size %u invalid — keeping the default",
			name, (unsigned int)len);
		return -EINVAL;
	}

	n = read_cb(cb_arg, scratch, len);
	if (n != (ssize_t)len) {
		return -EINVAL;
	}

	scratch[len - 1] = '\0'; /* defend against a stored value with no NUL */

	if (!apply(net_config_get(), scratch)) {
		LOG_WRN("stored %s rejected by validation — keeping the default",
			name);
	}
	return 0;
}

static bool apply_broker_host(net_config_t *c, const char *host)
{
	/* The port is a separate key and may not have been loaded yet, so reuse
	 * whatever port the config currently holds (the 1883 default at worst).
	 * Settings keys arrive in an unspecified order, so neither key may
	 * depend on the other having been seen. */
	return net_config_set_broker(c, host, c->port);
}

static int net_settings_set(const char *key, size_t len,
			    settings_read_cb read_cb, void *cb_arg)
{
	net_config_t *cfg = net_config_get();

	if (strcmp(key, "ssid") == 0) {
		return load_str(read_cb, cb_arg, len, "ssid", net_config_set_ssid);
	}
	if (strcmp(key, "psk") == 0) {
		return load_str(read_cb, cb_arg, len, "psk", net_config_set_psk);
	}
	if (strcmp(key, "broker") == 0) {
		return load_str(read_cb, cb_arg, len, "broker", apply_broker_host);
	}
	if (strcmp(key, "user") == 0) {
		return load_str(read_cb, cb_arg, len, "user", net_config_set_user);
	}
	if (strcmp(key, "mqttpass") == 0) {
		return load_str(read_cb, cb_arg, len, "mqttpass",
				net_config_set_mqtt_pass);
	}

	if (strcmp(key, "port") == 0) {
		uint16_t v;

		if (len != sizeof(v)) {
			LOG_WRN("stored port size %u invalid — keeping %u",
				(unsigned int)len, cfg->port);
			return -EINVAL;
		}
		if (read_cb(cb_arg, &v, sizeof(v)) != (ssize_t)sizeof(v)) {
			return -EINVAL;
		}
		/* Validated inline rather than through net_config_set_broker():
		 * that setter requires a non-empty host, and settings keys
		 * arrive in an unspecified order, so the broker key may not have
		 * been seen yet. The port range is the whole rule here. */
		if (v == 0u) {
			LOG_WRN("stored port 0 invalid — keeping %u", cfg->port);
			return 0;
		}
		cfg->port = v;
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(net, "net", NULL, net_settings_set, NULL, NULL);

static int save_one(const char *key, const void *val, size_t len)
{
	int ret = settings_save_one(key, val, len);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", key, ret);
	}
	return ret;
}

int net_store_save_ssid(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_SSID, c->ssid, strlen(c->ssid) + 1u);
}

int net_store_save_psk(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_PSK, c->psk, strlen(c->psk) + 1u);
}

int net_store_save_broker(void)
{
	const net_config_t *c = net_config_get();
	int first, ret;

	/* Two keys, both always attempted; report the first failure. */
	first = save_one(KEY_BROKER, c->broker, strlen(c->broker) + 1u);
	ret = save_one(KEY_PORT, &c->port, sizeof(c->port));

	return first ? first : ret;
}

int net_store_save_user(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_USER, c->mqtt_user, strlen(c->mqtt_user) + 1u);
}

int net_store_save_mqtt_pass(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_MQTTPASS, c->mqtt_pass, strlen(c->mqtt_pass) + 1u);
}

int net_store_save_all(void)
{
	int first = 0, ret;

	ret = net_store_save_ssid();
	if (ret && !first) { first = ret; }
	ret = net_store_save_psk();
	if (ret && !first) { first = ret; }
	ret = net_store_save_broker();
	if (ret && !first) { first = ret; }
	ret = net_store_save_user();
	if (ret && !first) { first = ret; }
	ret = net_store_save_mqtt_pass();
	if (ret && !first) { first = ret; }

	return first;
}

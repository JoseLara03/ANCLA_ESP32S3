/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gateway network configuration: WiFi credentials and MQTT broker details.
 * Pure C with no Zephyr dependency so the validation is host-testable; the
 * persistence lives in net_store.c and the console in net_shell.c.
 *
 * Two passwords live here and they must never be confused: `psk` is the WiFi
 * WPA2 passphrase, `mqtt_pass` is the MQTT password. The console commands
 * (`net pass` / `net mqttpass`), the NVS keys and the JSON keys all agree with
 * these field names.
 */

#ifndef NET_CONFIG_H
#define NET_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Maximum useful lengths, all excluding the NUL terminator. */
#define NET_SSID_MAX      32  /* 802.11 SSID                    */
#define NET_PSK_MAX       63  /* WPA2 passphrase; minimum is 8  */
#define NET_PSK_MIN        8
#define NET_BROKER_MAX    63  /* hostname or IPv4 literal       */
#define NET_MQTT_USER_MAX 32
#define NET_MQTT_PASS_MAX 63

#define NET_MQTT_PORT_DEFAULT 1883u

typedef struct {
	char     ssid[NET_SSID_MAX + 1];
	char     psk[NET_PSK_MAX + 1];
	char     broker[NET_BROKER_MAX + 1];
	uint16_t port;
	char     mqtt_user[NET_MQTT_USER_MAX + 1];
	char     mqtt_pass[NET_MQTT_PASS_MAX + 1];
} net_config_t;

/* Overwrite *c with the documented defaults: everything empty, port 1883. */
void net_config_set_defaults(net_config_t *c);

/* Initialise the singleton to defaults.
 *
 * MUST be called from main() before anything else can reach net_config_get().
 * Unlike uwb_config_get(), this has no lazy first-caller-wins initialiser:
 * net_uplink is a second thread, which breaks the assumption that makes the
 * lazy pattern safe over there (see CLAUDE.md). */
void net_config_init(void);

/* The single active instance. Undefined before net_config_init(). */
net_config_t *net_config_get(void);

/* Validating setters. Each returns true and mutates on success, or returns
 * false and leaves *c completely untouched on a rejected value. */
bool net_config_set_ssid(net_config_t *c, const char *ssid);
bool net_config_set_psk(net_config_t *c, const char *psk);
bool net_config_set_broker(net_config_t *c, const char *host, uint32_t port);
bool net_config_set_user(net_config_t *c, const char *user);
bool net_config_set_mqtt_pass(net_config_t *c, const char *pass);

/* True when there is enough configuration to attempt an uplink at all: an
 * SSID to join and a broker to publish to. MQTT credentials may legitimately
 * be empty (anonymous broker), so they are not required here. */
bool net_config_is_provisioned(const net_config_t *c);

#endif /* NET_CONFIG_H */

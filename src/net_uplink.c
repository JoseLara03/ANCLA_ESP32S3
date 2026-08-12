/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_uplink.h"

#include "net_config.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/* net_if.h must precede dhcpv4.h: in this Zephyr version dhcpv4.h does not
 * include net_if.h itself, so "struct net_if" would otherwise get a
 * prototype-scoped forward declaration in dhcpv4.h that is incompatible with
 * the real tag net_if.h defines -- a hard compile error, not a style nit. */
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <string.h>

LOG_MODULE_REGISTER(net_uplink, LOG_LEVEL_INF);

/* Below the WiFi driver's threads (capped at ESP32_WIFI_MAX_THREAD_PRIORITY,
 * default 7) so the driver drains its packet queues before we hand it more,
 * and far below the promoted gateway loop at K_PRIO_COOP(0). */
#define UPLINK_PRIO       10
#define UPLINK_STACK_SIZE 4096

/* Reconnect backoff ladder, in seconds. */
#define BACKOFF_START_S 1u
#define BACKOFF_MAX_S   32u

enum uplink_state {
	ST_UNCONFIGURED = 0,
	ST_WIFI_CONNECTING,
	ST_WIFI_CONNECTED,
	ST_MQTT_CONNECTING,
	ST_CONNECTED,
};

static const char *const state_names[] = {
	[ST_UNCONFIGURED]    = "unconfigured",
	[ST_WIFI_CONNECTING] = "wifi-connecting",
	[ST_WIFI_CONNECTED]  = "wifi-connected",
	[ST_MQTT_CONNECTING] = "mqtt-connecting",
	[ST_CONNECTED]       = "connected",
};

static enum uplink_state g_state = ST_UNCONFIGURED;

static K_SEM_DEFINE(ip_sem, 0, 1);
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void wifi_evt(struct net_mgmt_event_callback *cb, uint64_t event,
		     struct net_if *iface)
{
	ARG_UNUSED(iface);

	switch (event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		const struct wifi_status *st = (const struct wifi_status *)cb->info;

		if (st->status) {
			LOG_WRN("WiFi association failed (status %d)", st->status);
		} else {
			LOG_INF("WiFi associated");
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_WRN("WiFi disconnected");
		g_state = ST_WIFI_CONNECTING;
		break;
	default:
		break;
	}
}

static void ipv4_evt(struct net_mgmt_event_callback *cb, uint64_t event,
		     struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (event == NET_EVENT_IPV4_ADDR_ADD) {
		k_sem_give(&ip_sem);
	}
}

bool net_uplink_get_ip(char *buf, size_t len)
{
	struct net_if *iface = net_if_get_first_wifi();
	struct net_if_ipv4 *ipv4;

	if (iface == NULL || iface->config.ip.ipv4 == NULL) {
		return false;
	}

	ipv4 = iface->config.ip.ipv4;
	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (ipv4->unicast[i].ipv4.addr_state != NET_ADDR_PREFERRED) {
			continue;
		}
		if (net_addr_ntop(AF_INET, &ipv4->unicast[i].ipv4.address.in_addr,
				  buf, len) == NULL) {
			return false;
		}
		return true;
	}
	return false;
}

const char *net_uplink_state_str(void)
{
	return state_names[g_state];
}

/* Associate and wait for DHCP. Returns true once an IPv4 address is assigned. */
static bool wifi_connect(const net_config_t *cfg)
{
	struct wifi_connect_req_params params = {0};
	struct net_if *iface = net_if_get_first_wifi();
	int ret;

	if (iface == NULL) {
		LOG_ERR("no WiFi interface — is the &wifi node enabled?");
		return false;
	}

	params.ssid        = (const uint8_t *)cfg->ssid;
	params.ssid_length = strlen(cfg->ssid);
	params.psk         = (const uint8_t *)cfg->psk;
	params.psk_length  = strlen(cfg->psk);
	params.security    = WIFI_SECURITY_TYPE_PSK;
	params.channel     = WIFI_CHANNEL_ANY;
	params.band        = WIFI_FREQ_BAND_2_4_GHZ;
	params.mfp         = WIFI_MFP_OPTIONAL;

	g_state = ST_WIFI_CONNECTING;
	LOG_INF("associating with \"%s\"", cfg->ssid);

	/* The request fails while the interface is still coming up after boot,
	 * so retry rather than treating the first failure as fatal -- this is
	 * what samples/net/cloud/tagoio_http_post/src/wifi.c does, and skipping
	 * it makes the first association after every cold boot fail. */
	for (int tries = 10; tries > 0; tries--) {
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params,
			       sizeof(params));
		if (ret == 0) {
			break;
		}
		LOG_DBG("connect request failed (%d) — waiting for the iface", ret);
		k_msleep(500);
	}

	if (ret) {
		LOG_WRN("NET_REQUEST_WIFI_CONNECT failed (%d)", ret);
		return false;
	}

	net_dhcpv4_start(iface);

	/* 30 s covers association plus a DHCP exchange on a slow AP. Failing
	 * here is not fatal -- the caller backs off and retries. */
	if (k_sem_take(&ip_sem, K_SECONDS(30)) != 0) {
		LOG_WRN("no IPv4 address within 30 s");
		return false;
	}

	g_state = ST_WIFI_CONNECTED;
	return true;
}

static void uplink_thread(void *a, void *b, void *c)
{
	const net_config_t *cfg = net_config_get();
	uint32_t backoff_s = BACKOFF_START_S;
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	if (!net_config_is_provisioned(cfg)) {
		/* Logged exactly once. An unprovisioned gateway is a normal
		 * state, not an error to repeat forever. */
		LOG_INF("no network configuration — uplink idle "
			"(set `net ssid` and `net broker`, then reboot)");
		g_state = ST_UNCONFIGURED;
		return;
	}

	net_mgmt_init_event_callback(&wifi_cb, wifi_evt,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, ipv4_evt, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	while (1) {
		if (!wifi_connect(cfg)) {
			LOG_WRN("retrying in %u s", backoff_s);
			k_sleep(K_SECONDS(backoff_s));
			backoff_s = MIN(backoff_s * 2u, BACKOFF_MAX_S);
			continue;
		}

		backoff_s = BACKOFF_START_S;

		if (net_uplink_get_ip(ip, sizeof(ip))) {
			LOG_INF("{\"wifi\":\"connected\",\"ip\":\"%s\"}", ip);
		}

		/* Task 5 replaces this with the MQTT connect and poll loop. */
		while (g_state == ST_WIFI_CONNECTED) {
			k_sleep(K_SECONDS(1));
		}
	}
}

K_THREAD_STACK_DEFINE(uplink_stack, UPLINK_STACK_SIZE);
static struct k_thread uplink_thread_data;

void net_uplink_start(void)
{
	k_thread_create(&uplink_thread_data, uplink_stack, UPLINK_STACK_SIZE,
			uplink_thread, NULL, NULL, NULL,
			UPLINK_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&uplink_thread_data, "net_uplink");
}

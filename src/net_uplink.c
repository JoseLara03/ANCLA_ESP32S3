/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_uplink.h"

#include "net_config.h"
#include "pos_json.h"
#include "uwb_config.h"

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
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>

#include <errno.h>
#include <stdio.h>
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

#define MQTT_RX_BUF_SIZE 256
#define MQTT_TX_BUF_SIZE 256

/* Poll timeout while connected. Short enough that a queued fix waits at most
 * this long (Task 6) -- Zephyr cannot wait on a socket and a k_msgq in one
 * call without an eventfd, so the queue is drained on this cadence instead.
 * 50 ms against a 200 ms superframe is not a meaningful added latency. */
#define POLL_TIMEOUT_MS 50

#define LOCATION_TOPIC "testtopic/1/position"
#define ANCHOR_TOPIC   "testtopic/1/anchors"

static struct mqtt_client client;
static struct sockaddr_storage broker_addr;
static uint8_t mqtt_rx_buf[MQTT_RX_BUF_SIZE];
static uint8_t mqtt_tx_buf[MQTT_TX_BUF_SIZE];
static char    client_id[16];
static char    payload_buf[POS_JSON_MAX_LEN];
static uint16_t next_msg_id = 1u;
static bool    mqtt_connected;
static bool    connack_seen;

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

static void mqtt_evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	ARG_UNUSED(c);

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_WRN("CONNACK refused (%d)", evt->result);
			break;
		}
		connack_seen = true;
		mqtt_connected = true;
		LOG_INF("MQTT connected");
		break;
	case MQTT_EVT_DISCONNECT:
		LOG_WRN("MQTT disconnected (%d)", evt->result);
		mqtt_connected = false;
		break;
	case MQTT_EVT_PUBACK:
		break;
	default:
		break;
	}
}

/* Resolve the broker into broker_addr. Accepts a hostname or an IPv4 literal;
 * getaddrinfo() handles both, which is why CONFIG_DNS_RESOLVER is enabled --
 * a literal-only implementation would make a moved broker a reflash. */
static bool resolve_broker(const net_config_t *cfg)
{
	struct zsock_addrinfo hints = {
		.ai_family   = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct zsock_addrinfo *res = NULL;
	struct sockaddr_in *in = (struct sockaddr_in *)&broker_addr;
	char port_str[6];
	int ret;

	snprintf(port_str, sizeof(port_str), "%u", cfg->port);

	ret = zsock_getaddrinfo(cfg->broker, port_str, &hints, &res);
	if (ret != 0 || res == NULL) {
		LOG_WRN("cannot resolve broker \"%s\" (%d)", cfg->broker, ret);
		return false;
	}

	memcpy(in, res->ai_addr, sizeof(*in));
	in->sin_port = htons(cfg->port);
	in->sin_family = AF_INET;
	zsock_freeaddrinfo(res);
	return true;
}

static int publish(const char *topic, const char *msg, enum mqtt_qos qos,
		   bool retain)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.qos          = qos;
	p.message.topic.topic.utf8   = (uint8_t *)topic;
	p.message.topic.topic.size   = strlen(topic);
	p.message.payload.data       = (uint8_t *)msg;
	p.message.payload.len        = strlen(msg);
	p.message_id                 = next_msg_id++;
	p.dup_flag                   = 0u;
	p.retain_flag                = retain ? 1u : 0u;

	/* message_id 0 is reserved by the protocol. */
	if (next_msg_id == 0u) {
		next_msg_id = 1u;
	}

	return mqtt_publish(&client, &p);
}

/* Connect and wait for CONNACK. Returns true with mqtt_connected set. */
static bool mqtt_bring_up(const net_config_t *cfg)
{
	static struct mqtt_utf8 user, pass;
	const uwb_config_t *ucfg = uwb_config_get();
	int64_t deadline;
	int ret;

	if (!resolve_broker(cfg)) {
		return false;
	}

	g_state = ST_MQTT_CONNECTING;
	connack_seen = false;
	mqtt_connected = false;

	snprintf(client_id, sizeof(client_id), "uwb-gw-%04X",
		 uwb_config_short_addr(ucfg));

	mqtt_client_init(&client);

	client.broker           = &broker_addr;
	client.evt_cb           = mqtt_evt_handler;
	client.client_id.utf8   = (uint8_t *)client_id;
	client.client_id.size   = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.transport.type   = MQTT_TRANSPORT_NON_SECURE;
	client.rx_buf           = mqtt_rx_buf;
	client.rx_buf_size      = sizeof(mqtt_rx_buf);
	client.tx_buf           = mqtt_tx_buf;
	client.tx_buf_size      = sizeof(mqtt_tx_buf);

	/* Empty credentials mean an anonymous broker: leave the pointers NULL
	 * rather than sending zero-length fields. */
	if (cfg->mqtt_user[0] != '\0') {
		user.utf8 = (uint8_t *)cfg->mqtt_user;
		user.size = strlen(cfg->mqtt_user);
		client.user_name = &user;
	} else {
		client.user_name = NULL;
	}
	if (cfg->mqtt_pass[0] != '\0') {
		pass.utf8 = (uint8_t *)cfg->mqtt_pass;
		pass.size = strlen(cfg->mqtt_pass);
		client.password = &pass;
	} else {
		client.password = NULL;
	}

	ret = mqtt_connect(&client);
	if (ret) {
		LOG_WRN("mqtt_connect failed (%d)", ret);
		return false;
	}

	/* Wait for CONNACK, pumping the socket. 10 s is generous for a LAN
	 * broker and still bounded. */
	deadline = k_uptime_get() + 10000;
	while (!connack_seen && k_uptime_get() < deadline) {
		struct zsock_pollfd fds = {
			.fd = client.transport.tcp.sock,
			.events = ZSOCK_POLLIN,
		};

		if (zsock_poll(&fds, 1, POLL_TIMEOUT_MS) > 0) {
			if (mqtt_input(&client) != 0) {
				break;
			}
		}
	}

	if (!connack_seen) {
		LOG_WRN("no CONNACK within 10 s");
		mqtt_abort(&client);
		return false;
	}

	g_state = ST_CONNECTED;
	return true;
}

/* Publish the stubbed zone/anchor map. Retained and QoS 1: it is slow-changing
 * state that a late subscriber needs, which is exactly the opposite of a
 * position fix. */
static void publish_anchor_stub(void)
{
	int n = pos_json_anchors(payload_buf, sizeof(payload_buf));

	if (n < 0) {
		LOG_ERR("anchors payload does not fit POS_JSON_MAX_LEN");
		return;
	}

	if (publish(ANCHOR_TOPIC, payload_buf, MQTT_QOS_1_AT_LEAST_ONCE, true)) {
		LOG_WRN("anchors publish failed");
	} else {
		LOG_INF("published retained anchor map");
	}
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

		if (!mqtt_bring_up(cfg)) {
			LOG_WRN("MQTT retry in %u s", backoff_s);
			k_sleep(K_SECONDS(backoff_s));
			backoff_s = MIN(backoff_s * 2u, BACKOFF_MAX_S);
			continue;
		}

		backoff_s = BACKOFF_START_S;
		publish_anchor_stub();

		while (mqtt_connected) {
			struct zsock_pollfd fds = {
				.fd = client.transport.tcp.sock,
				.events = ZSOCK_POLLIN,
			};

			if (zsock_poll(&fds, 1, POLL_TIMEOUT_MS) > 0 &&
			    mqtt_input(&client) != 0) {
				break;
			}

			/* Drives the keepalive PINGREQ. */
			if (mqtt_live(&client) != 0 && errno != EAGAIN) {
				break;
			}

			/* Task 6 drains the fix queue here. */
		}

		LOG_WRN("MQTT connection lost — reconnecting");
		mqtt_abort(&client);
		mqtt_connected = false;
		g_state = ST_WIFI_CONNECTED;
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

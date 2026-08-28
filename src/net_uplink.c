/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_uplink.h"

#include <zephyr/sys/util.h> /* ARG_UNUSED, used by both branches below */

/* Every symbol below is WiFi/MQTT-stack shaped and has zero fallback of its
 * own, so this whole file compiles only when the net stack it depends on is
 * actually present. The calibration image (cal.conf) turns CONFIG_NETWORKING
 * off on purpose -- it never runs GATEWAY mode and has nothing to publish --
 * and the stub block below keeps the public API linkable (net_shell.c and
 * pos_sink.c call it unconditionally) without dragging in net_if.h et al. */
#ifdef CONFIG_NETWORKING

#include "apos_store.h"
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
#include <zephyr/net/net_compat.h>
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

/* Both topics are zone-scoped, and the trailing element is the same zone id the
 * anchors payload already carries as its "name" field -- so it is composed from
 * POS_JSON_ZONE_NAME rather than repeated as a literal here. Changing zone is
 * then one edit in pos_json.h, and the topic can never disagree with the
 * payload published on it. */
#define LOCATION_TOPIC "uwb/response/position/" POS_JSON_ZONE_NAME
#define ANCHOR_TOPIC   "uwb/anchor/setup/" POS_JSON_ZONE_NAME

static struct mqtt_client client;
static struct sockaddr_storage broker_addr;
static uint8_t mqtt_rx_buf[MQTT_RX_BUF_SIZE];
static uint8_t mqtt_tx_buf[MQTT_TX_BUF_SIZE];
static char    client_id[16];
static char    payload_buf[POS_JSON_MAX_LEN];
static uint16_t next_msg_id = 1u;
static bool    mqtt_connected;
static bool    connack_seen;
static bool    suback_seen;

/* ~1.5 superframes at GW_N_CFP (11) seats: enough to absorb a publish stalling
 * behind one TCP retransmit, small enough that a real outage discards rather
 * than accumulates. */
#define FIX_QUEUE_DEPTH 16

K_MSGQ_DEFINE(fix_q, sizeof(struct pos_fix), FIX_QUEUE_DEPTH, 4);

/* Two queues, not one: the directions are different and live in different
 * modes. 32 observations are ~4 tags x 4 anchors x 2 superframes of slack on
 * the anchor, and on the gateway ~8 complete 4-anchor blinks -- above
 * TDOA_COLLECT_SLOTS (16 groups), so the queue is never the bottleneck before
 * the collector is. DRAM cost: 32 x 24 B = 768 B each. */
#define OBS_QUEUE_DEPTH 32

K_MSGQ_DEFINE(blink_q, sizeof(struct pos_blink_obs), OBS_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(obs_q,   sizeof(struct pos_blink_obs), OBS_QUEUE_DEPTH, 4);

static uint32_t n_obs_pub;
static uint32_t n_obs_pub_drop;
static uint32_t n_obs_rx;
static uint32_t n_obs_rx_drop;
static uint32_t n_obs_sub_fail;

/* Receive buffer for one PUBLISH. static, not an automatic: the uplink thread
 * has 4096 B of stack and pos_json_blink_parse() already puts 96 B of its own
 * there. */
static uint8_t obs_payload[POS_JSON_BLINK_MAX_LEN];

/* true in GATEWAY mode. Read once when the thread starts and never changes:
 * the mode comes from NVS and only changes with a reboot. */
static bool is_gateway;

static uint32_t dropped_fixes;
static int64_t  last_drop_warn_ms;

/* A sustained outage drops ~55 fixes per second. An unthrottled warning would
 * flood the very console being used to diagnose it. */
#define DROP_WARN_INTERVAL_MS 10000

static K_SEM_DEFINE(ip_sem, 0, 1);
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

/* Ground truth for "is the link actually up", set only from the WiFi
 * association result/disconnect events and from a successful wifi_connect().
 * uplink_thread()'s MQTT-level retry loop polls this to decide whether an
 * MQTT failure should be retried in place (WiFi still up) or whether it must
 * fall back to the outer loop and re-associate. g_state is a *display*
 * string and must not be trusted for this decision -- see wifi_evt(). */
static volatile bool wifi_associated;

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
		/* Cleared here, not just left to be inferred: this is the one
		 * authoritative signal that the link actually dropped, as
		 * opposed to an MQTT-only failure while WiFi stays up. */
		wifi_associated = false;
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

void net_uplink_submit(const struct pos_fix *fix)
{
	struct pos_fix discarded;

	/* Called from the gateway's dispatch path: never block, never allocate,
	 * never touch a socket. */
	if (k_msgq_put(&fix_q, fix, K_NO_WAIT) == 0) {
		return;
	}

	/* Full: drop the OLDEST and take the newest. A stale position is
	 * worthless in an RTLS, so the newest sample is always the one worth
	 * keeping. */
	(void)k_msgq_get(&fix_q, &discarded, K_NO_WAIT);
	if (k_msgq_put(&fix_q, fix, K_NO_WAIT) != 0) {
		/* Only reachable if the uplink thread raced us to the slot; the
		 * fix is dropped either way. */
	}

	dropped_fixes++;

	int64_t now = k_uptime_get();

	if (now - last_drop_warn_ms >= DROP_WARN_INTERVAL_MS) {
		last_drop_warn_ms = now;
		LOG_WRN("uplink queue full — %u fixes dropped so far", dropped_fixes);
	}
}

void net_uplink_submit_blink(const struct pos_blink_obs *o)
{
	struct pos_blink_obs discarded;

	if (o == NULL) {
		return;
	}
	if (k_msgq_put(&blink_q, o, K_NO_WAIT) == 0) {
		return;
	}

	/* Full: drop the OLDEST, same rule as net_uplink_submit(). */
	(void)k_msgq_get(&blink_q, &discarded, K_NO_WAIT);
	n_obs_pub_drop++;
	if (k_msgq_put(&blink_q, o, K_NO_WAIT) != 0) {
		n_obs_pub_drop++;
	}
}

bool net_uplink_get_obs(struct pos_blink_obs *out)
{
	if (out == NULL) {
		return false;
	}
	return k_msgq_get(&obs_q, out, K_NO_WAIT) == 0;
}

void net_uplink_obs_stats(uint32_t *n_pub, uint32_t *n_pub_drop, uint32_t *n_rx,
			  uint32_t *n_rx_drop, uint32_t *n_sub_fail)
{
	if (n_pub != NULL)      { *n_pub = n_obs_pub; }
	if (n_pub_drop != NULL) { *n_pub_drop = n_obs_pub_drop; }
	if (n_rx != NULL)       { *n_rx = n_obs_rx; }
	if (n_rx_drop != NULL)  { *n_rx_drop = n_obs_rx_drop; }
	if (n_sub_fail != NULL) { *n_sub_fail = n_obs_sub_fail; }
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
	wifi_associated = true;
	return true;
}

/* Fold one received observation PUBLISH into obs_q. GATEWAY side. */
static void handle_publish(struct mqtt_client *c,
			   const struct mqtt_publish_param *p)
{
	struct pos_blink_obs obs;
	struct pos_blink_obs discarded;
	uint32_t len = p->message.payload.len;

	/* The payload is ALWAYS drained from the socket, whether it fits or
	 * not: leaving bytes unread desynchronises the MQTT stream and takes
	 * down the whole connection, not just this message. */
	if (len == 0u || len >= sizeof(obs_payload)) {
		while (len > 0u) {
			uint32_t chunk = MIN(len, (uint32_t)sizeof(obs_payload));

			if (mqtt_readall_publish_payload(c, obs_payload,
							 chunk) < 0) {
				mqtt_connected = false;
				return;
			}
			len -= chunk;
		}
		n_obs_rx_drop++;
		return;
	}

	if (mqtt_readall_publish_payload(c, obs_payload, len) < 0) {
		mqtt_connected = false;
		return;
	}

	/* QoS 0 on the subscription, so there is no PUBACK to return here. */
	if (pos_json_blink_parse((const char *)obs_payload, (size_t)len,
				 &obs) != 0) {
		n_obs_rx_drop++;
		return;
	}

	n_obs_rx++;

	if (k_msgq_put(&obs_q, &obs, K_NO_WAIT) == 0) {
		return;
	}
	(void)k_msgq_get(&obs_q, &discarded, K_NO_WAIT);
	n_obs_rx_drop++;
	if (k_msgq_put(&obs_q, &obs, K_NO_WAIT) != 0) {
		n_obs_rx_drop++;
	}
}

static void mqtt_evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
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
	case MQTT_EVT_SUBACK:
		if (evt->result != 0) {
			LOG_WRN("SUBACK refused (%d)", evt->result);
			break;
		}
		suback_seen = true;
		break;
	case MQTT_EVT_PUBLISH:
		handle_publish(c, &evt->param.publish);
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

/* Subscribe to the observation topic. GATEWAY only.
 *
 * Called from mqtt_bring_up(), which is what makes RE-subscribing after a
 * reconnect automatic instead of a forgettable special case: every reconnect
 * goes through here. A gateway that reconnects without re-subscribing is
 * silently DEAF -- it keeps publishing fixes it can no longer compute, and
 * nothing in the log says so.
 *
 * A missing or refused SUBACK returns false, which mqtt_bring_up() turns into
 * an mqtt_abort() and a backoff retry. Deliberately NOT carrying on without a
 * subscription: a connection that publishes but never receives is exactly the
 * undetectable state this gate exists to prevent. */
static bool subscribe_observations(void)
{
	struct mqtt_topic topic = {
		.topic = {
			.utf8 = (uint8_t *)POS_JSON_TOPIC_BLINK,
			.size = sizeof(POS_JSON_TOPIC_BLINK) - 1u,
		},
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
	};
	struct mqtt_subscription_list list = {
		.list        = &topic,
		.list_count  = 1u,
		.message_id  = next_msg_id++,
	};
	int ret;

	if (next_msg_id == 0u) {
		next_msg_id = 1u;
	}

	suback_seen = false;
	ret = mqtt_subscribe(&client, &list);
	if (ret != 0) {
		LOG_WRN("mqtt_subscribe failed (%d)", ret);
		n_obs_sub_fail++;
		return false;
	}

	/* Up to 10 s, the same scale as the CONNACK wait. Bounded: this thread
	 * is preemptible, but an unbounded loop here would leave the uplink
	 * hung forever without saying why. */
	for (int i = 0; i < 100 && !suback_seen && mqtt_connected; i++) {
		struct zsock_pollfd fds = {
			.fd     = client.transport.tls.sock,
			.events = ZSOCK_POLLIN,
		};

		if (zsock_poll(&fds, 1, 100) > 0 && mqtt_input(&client) != 0) {
			break;
		}
	}

	if (!suback_seen) {
		LOG_WRN("no SUBACK for %s within 10 s - the gateway would be "
			"deaf; reconnecting", POS_JSON_TOPIC_BLINK);
		n_obs_sub_fail++;
		return false;
	}

	LOG_INF("subscribed to %s", POS_JSON_TOPIC_BLINK);
	return true;
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
	client.rx_buf           = mqtt_rx_buf;
	client.rx_buf_size      = sizeof(mqtt_rx_buf);
	client.tx_buf           = mqtt_tx_buf;
	client.tx_buf_size      = sizeof(mqtt_tx_buf);

	/* Hosted brokers (CloudAMQP included) only expose MQTT over TLS
	 * externally -- a plain-TCP CONNECT gets the socket closed by the
	 * broker's TLS-terminating proxy. No CA certificate is provisioned:
	 * peer verification is off, matching how this broker is reached from
	 * MQTTX today (SSL/TLS on, certificate verification off). sec_tag_list
	 * stays empty since nothing is being verified. hostname is still set
	 * for SNI, in case the broker's TLS front end routes by server name. */
	client.transport.type                    = MQTT_TRANSPORT_SECURE;
	client.transport.tls.config.peer_verify   = TLS_PEER_VERIFY_NONE;
	client.transport.tls.config.cipher_count  = 0;
	client.transport.tls.config.cipher_list   = NULL;
	client.transport.tls.config.sec_tag_count = 0;
	client.transport.tls.config.sec_tag_list  = NULL;
	client.transport.tls.config.hostname      = cfg->broker;

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
			.fd = client.transport.tls.sock,
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

	if (is_gateway && !subscribe_observations()) {
		mqtt_abort(&client);
		mqtt_connected = false;
		return false;
	}

	return true;
}

/* Publish the zone/anchor map: the surveyed geometry if one has been applied,
 * otherwise the stub. Retained and QoS 1: it is slow-changing state that a
 * late subscriber needs, which is exactly the opposite of a position fix. */
static void publish_anchor_map(void)
{
	int n = pos_json_anchors(payload_buf, sizeof(payload_buf),
				  apos_store_get());

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

/* Publish everything queued. QoS 0 and not retained: the topic is flat, so
 * retaining would only preserve whichever tag published last, and QoS 0
 * matches the lossy UWB link underneath -- a fix needing a retransmit is
 * stale by the time it lands. Returns false if the connection died. */
static bool drain_fix_queue(void)
{
	struct pos_fix fix;

	while (k_msgq_get(&fix_q, &fix, K_NO_WAIT) == 0) {
		int n = pos_json_fix(payload_buf, sizeof(payload_buf), &fix);

		if (n < 0) {
			/* A formatting bug, not a network problem. Never publish
			 * a truncated JSON document. */
			LOG_ERR("position payload truncated — dropping fix from 0x%04X",
				fix.src_addr);
			continue;
		}

		if (publish(LOCATION_TOPIC, payload_buf,
			    MQTT_QOS_0_AT_MOST_ONCE, false) != 0) {
			LOG_WRN("position publish failed — reconnecting");
			return false;
		}
	}
	return true;
}

/* Publish every queued observation. QoS 0 and not retained, for the same
 * reasons as a fix: the topic is flat, and an observation that needs a
 * retransmit is already past TDOA_COLLECT_WINDOW_MS by the time it lands.
 * Returns false if the connection died. */
static bool drain_blink_queue(void)
{
	struct pos_blink_obs obs;

	while (k_msgq_get(&blink_q, &obs, K_NO_WAIT) == 0) {
		int n = pos_json_blink(payload_buf, sizeof(payload_buf), &obs);

		if (n < 0) {
			/* A formatting bug, not a network problem. Never
			 * publish truncated JSON. */
			LOG_ERR("observation payload truncated - dropping blink "
				"from 0x%04X", obs.tag_addr);
			n_obs_pub_drop++;
			continue;
		}

		if (publish(POS_JSON_TOPIC_BLINK, payload_buf,
			    MQTT_QOS_0_AT_MOST_ONCE, false) != 0) {
			LOG_WRN("observation publish failed - reconnecting");
			return false;
		}
		n_obs_pub++;
	}
	return true;
}

static void uplink_thread(void *a, void *b, void *c)
{
	const net_config_t *cfg = net_config_get();
	uint32_t wifi_backoff_s = BACKOFF_START_S;
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	is_gateway = (uwb_config_get()->mode == UWB_MODE_GATEWAY);

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

	/* Outer loop: WiFi-level connectivity. Only entered/re-entered when
	 * the link is believed to be down (cold start, wifi_connect()
	 * failure, or a genuine NET_EVENT_WIFI_DISCONNECT_RESULT clearing
	 * wifi_associated). Every iteration re-associates and resets both
	 * backoff ladders. */
	while (1) {
		if (!wifi_connect(cfg)) {
			LOG_WRN("retrying in %u s", wifi_backoff_s);
			k_sleep(K_SECONDS(wifi_backoff_s));
			wifi_backoff_s = MIN(wifi_backoff_s * 2u, BACKOFF_MAX_S);
			continue;
		}

		wifi_backoff_s = BACKOFF_START_S;

		if (net_uplink_get_ip(ip, sizeof(ip))) {
			LOG_INF("{\"wifi\":\"connected\",\"ip\":\"%s\"}", ip);
		}

		uint32_t mqtt_backoff_s = BACKOFF_START_S;

		/* Inner loop: MQTT-level connectivity, retried in place while
		 * WiFi is believed to still be associated. This is the fix
		 * for the coupling bug -- an MQTT-only failure (broker down,
		 * connection dropped, publish failure) must NOT call
		 * wifi_connect() again: the ESP32 driver rejects a second
		 * connect request with -EALREADY while already associated
		 * and raises a fake CONN_FAIL event for it, which used to
		 * burn the whole WiFi backoff ladder and then get stuck
		 * because mqtt_bring_up() was never retried. */
		while (wifi_associated) {
			if (!mqtt_bring_up(cfg)) {
				LOG_WRN("MQTT retry in %u s", mqtt_backoff_s);
				k_sleep(K_SECONDS(mqtt_backoff_s));
				mqtt_backoff_s = MIN(mqtt_backoff_s * 2u,
						      BACKOFF_MAX_S);
				continue;
			}

			mqtt_backoff_s = BACKOFF_START_S;

			/* Only the gateway publishes the retained map. Four
			 * anchors overwriting the same retained document would
			 * be a real defect, not redundancy: an anchor does not
			 * know the whole survey. */
			if (is_gateway) {
				publish_anchor_map();
			}

			while (mqtt_connected && wifi_associated) {
				struct zsock_pollfd fds = {
					.fd = client.transport.tls.sock,
					.events = ZSOCK_POLLIN,
				};

				if (zsock_poll(&fds, 1, POLL_TIMEOUT_MS) > 0 &&
				    mqtt_input(&client) != 0) {
					break;
				}

				/* Drives the keepalive PINGREQ. mqtt_live()
				 * returns -EAGAIN directly as its return value
				 * on the "not yet time to ping" path -- it
				 * never touches errno -- so the return value
				 * itself must be checked, not errno (which
				 * could hold anything from an unrelated call
				 * and would make this branch fire almost
				 * every poll). */
				int live_ret = mqtt_live(&client);

				if (live_ret != 0 && live_ret != -EAGAIN) {
					break;
				}

				if (is_gateway) {
					if (!drain_fix_queue()) {
						break;
					}
				} else if (!drain_blink_queue()) {
					break;
				}
			}

			LOG_WRN("MQTT connection lost — reconnecting");
			mqtt_abort(&client);
			mqtt_connected = false;

			/* Only claim wifi-connected if WiFi is actually still
			 * up. If wifi_associated went false while we were in
			 * the connected loop above, wifi_evt() already set
			 * g_state = ST_WIFI_CONNECTING for the real reason --
			 * don't stomp on that with a stale "connected". */
			if (wifi_associated) {
				g_state = ST_WIFI_CONNECTED;
			}
		}

		/* Reached only once wifi_associated is false: a genuine WiFi
		 * disconnect. Fall back to the top of the outer loop, which
		 * re-associates and resets mqtt_backoff_s (re-declared each
		 * outer iteration) back to BACKOFF_START_S. */
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

#else /* !CONFIG_NETWORKING */

void net_uplink_start(void)
{
}

void net_uplink_submit(const struct pos_fix *fix)
{
	ARG_UNUSED(fix);
}

/* The observation path is stubbed here for the same reason as the rest of this
 * block: net_uplink.c is in the UNCONDITIONAL target_sources list, so the
 * calibration image (CONFIG_NETWORKING=n) compiles it too, and blink_rx.c --
 * also unconditional -- calls net_uplink_submit_blink(). Without these the cal
 * image fails to LINK, not to compile. */
void net_uplink_submit_blink(const struct pos_blink_obs *o)
{
	ARG_UNUSED(o);
}

bool net_uplink_get_obs(struct pos_blink_obs *out)
{
	ARG_UNUSED(out);
	return false;
}

void net_uplink_obs_stats(uint32_t *n_pub, uint32_t *n_pub_drop, uint32_t *n_rx,
			  uint32_t *n_rx_drop, uint32_t *n_sub_fail)
{
	if (n_pub != NULL)      { *n_pub = 0u; }
	if (n_pub_drop != NULL) { *n_pub_drop = 0u; }
	if (n_rx != NULL)       { *n_rx = 0u; }
	if (n_rx_drop != NULL)  { *n_rx_drop = 0u; }
	if (n_sub_fail != NULL) { *n_sub_fail = 0u; }
}

const char *net_uplink_state_str(void)
{
	return "networking-disabled";
}

bool net_uplink_get_ip(char *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return false;
}

#endif /* CONFIG_NETWORKING */

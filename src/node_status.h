/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NODE_STATUS_H
#define NODE_STATUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The periodic status report every node publishes over MQTT.
 *
 * Why it exists: anchors are unreachable once mounted -- no USB, no console -- so
 * everything `sync stats`, `sync tau show` and `net show` print had to be read
 * on a bench, which is exactly where the field failures do not happen. Each node
 * now publishes the same numbers on uwb/anchor/status/<zone>/<EUI16> (retained, so a
 * late subscriber sees the last report) every NODE_STATUS_PERIOD_S, with an MQTT
 * last-will that flips `online` to 0 when the node drops off.
 *
 * This module is PURE: it formats a plain struct. Collecting the numbers from the
 * radio, WiFi and sync modules is net_uplink.c's job. That split is what makes the
 * JSON host-testable (tests/node_status/), which matters because a report that is
 * silently truncated or mis-typed is read by a consumer that drops what it cannot
 * parse -- the same reason pos_json.c is pure.
 */

#define NODE_STATUS_PERIOD_S 10u

/* Fields that only exist for one role are guarded by has_*; they are OMITTED from
 * the JSON rather than sent as zero, because a zero reads as a measurement. */
struct node_status {
	bool        is_gateway;
	/* The node's identity on every topic (node_eui_hex(), or the short-address
	 * fallback when the board has no EUI). NULL is omitted. */
	const char *eui;
	bool        assigned;        /* slave only: holds a gateway-given id */
	uint8_t     anchor_id;       /* slave only, and only when assigned */
	uint16_t    short_addr;
	uint32_t    uptime_s;
	const char *reset;           /* "power", "brownout", ... never NULL */
	const char *fw;              /* build stamp, never NULL */

	/* WiFi / MQTT */
	bool        has_rssi;
	int32_t     rssi_dbm;
	uint32_t    wifi_up;         /* lifetime association count */
	uint32_t    mqtt_up;         /* lifetime MQTT connect count */
	uint32_t    obs_pub;
	uint32_t    obs_drop;

	/* Sync (slave) -- the numbers `sync stats` prints */
	bool        has_sync;
	bool        sync_valid;
	uint32_t    sigma_dtu;
	uint32_t    jitter_dtu;
	uint32_t    rms_dtu;
	uint32_t    max_dtu;
	uint32_t    n_obs;
	uint32_t    rebaselines;
	uint32_t    sync_gaps;
	uint32_t    outliers;        /* rejected by the gross-outlier gate */
	uint32_t    ccp_rx;
	uint32_t    ccp_rejected;
	int32_t     drift_ppb;
	bool        clock_fresh;
	int32_t     tau_used_dtu;
	const char *tau_mode;        /* "legacy" | "survey" */
	bool        tau_link;        /* a survey installed the link */

	/* BLINK reception (slave) */
	uint32_t    blink_rx;
	uint32_t    blink_no_sync;
	uint32_t    blink_stale;
	uint32_t    blink_sent;

	/* Remote commands: whether they are enabled (a key is stored) and the challenge a command
	 * must be signed against. Not secret -- it only has to be fresh and single-use. */
	bool        cmd_enabled;
	const char *challenge;       /* NULL or "" when none */

	/* CCP master (gateway) */
	bool        has_master;
	uint32_t    ccp_sent;
	uint32_t    ccp_dropped;
};

/* Writes one compact JSON object. Returns its length, or -1 if `len` is too small
 * -- never a truncated document, which would poison the retained topic. */
int node_status_json(char *buf, size_t len, const struct node_status *s);

/* The last-will payload: the node is gone. `eui` may be NULL (omitted); `id` is
 * printed only for an assigned anchor. */
int node_status_offline_json(char *buf, size_t len, bool is_gateway,
			     const char *eui, bool assigned, uint8_t id);

#endif /* NODE_STATUS_H */

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The WiFi + MQTT uplink thread. Runs in BOTH modes since Phase 3.
 *
 * Isolation from the UWB beacon is by PRIORITY, not by core: the ESP32 WiFi
 * driver is `depends on !SMP`, so there is exactly one core. This thread runs
 * preemptible and below both the promoted gateway loop and the WiFi driver's
 * own threads, and both directions cross a bounded queue that never blocks the
 * caller.
 *
 * ---- Two modes, two directions -------------------------------------------
 *
 * GATEWAY: publishes position fixes and the retained anchor map, and
 * SUBSCRIBES to POS_JSON_TOPIC_BLINK, feeding what arrives to
 * net_uplink_get_obs() for the gateway loop to drain.
 *
 * SLAVE: publishes TDoA observations (net_uplink_submit_blink()) and nothing
 * else. It does NOT publish the retained anchor map -- that document is the
 * gateway's, and four anchors overwriting each other's retained copy of it
 * would be a real fault, not redundancy.
 *
 * Until Phase 3 this header said "GATEWAY mode only -- a slave has nothing to
 * publish and starting WiFi there would cost ~50 KB of heap for no benefit."
 * The first half stopped being true (an anchor's BLINK observations ARE the
 * TDoA measurement, and there is no UWB backhaul for them); the second half is
 * still a real cost and it is now paid deliberately, to be measured on
 * hardware rather than assumed -- see the plan's Task 5 Step 14. Anchor DRAM
 * was at ~66 % before this change.
 */

#ifndef NET_UPLINK_H
#define NET_UPLINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pos_json.h"
#include "pos_sink.h"

/* Start the uplink thread. Called in BOTH modes since Phase 3: a gateway
 * publishes fixes and subscribes to observations, an anchor publishes
 * observations. Safe to call when unprovisioned: the thread logs once and
 * idles. */
void net_uplink_start(void);

/* Hand one decoded fix to the uplink. Non-blocking: enqueues and returns, so
 * it is safe from the gateway's dispatch path. Drops the OLDEST queued fix if
 * the queue is full -- a stale position is worthless in an RTLS. */
void net_uplink_submit(const struct pos_fix *fix);

/* Hand one TDoA observation to the uplink, ANCHOR side. Non-blocking: enqueues
 * and returns, so it is safe from the SLAVE loop's dispatch path. Drops the
 * OLDEST queued observation when the queue is full -- same rule as
 * net_uplink_submit(), and for the same reason: a stale observation is past
 * TDOA_COLLECT_WINDOW_MS by the time it lands and the gateway would discard it
 * anyway. */
void net_uplink_submit_blink(const struct pos_blink_obs *o);

/* Take one received observation, GATEWAY side. Returns false immediately when
 * none is queued -- never blocks, so it is safe to call from the
 * K_PRIO_COOP(0) gateway loop, which is the only intended caller (Task 6).
 *
 * The queue is the thread boundary, deliberately: the MQTT callback runs on
 * THIS thread (priority 10) and struct tdoa_collect is owned by the gateway
 * loop, so parsing here and folding there means tdoa_collect needs no lock at
 * all. */
bool net_uplink_get_obs(struct pos_blink_obs *out);

/* Observation-path counters, for `net show`. `n_pub`/`n_pub_drop` are the
 * anchor side, `n_rx`/`n_rx_drop` the gateway side, and `n_sub_fail` counts
 * subscriptions that never got a SUBACK -- a gateway with n_sub_fail climbing
 * is DEAF, and that is otherwise invisible from the console. */
void net_uplink_obs_stats(uint32_t *n_pub, uint32_t *n_pub_drop, uint32_t *n_rx,
			  uint32_t *n_rx_drop, uint32_t *n_sub_fail);

/* Human-readable connection state for `net show`. Never NULL. */
const char *net_uplink_state_str(void);

/* Copy the DHCP-assigned IPv4 address into `buf`. Returns false and leaves
 * `buf` untouched when no address is assigned. */
bool net_uplink_get_ip(char *buf, size_t len);

#endif /* NET_UPLINK_H */

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's WiFi + MQTT uplink thread.
 *
 * Isolation from the UWB beacon is by PRIORITY, not by core: the ESP32 WiFi
 * driver is `depends on !SMP`, so there is exactly one core. This thread runs
 * preemptible and below both the promoted gateway loop and the WiFi driver's
 * own threads, and the gateway hands work over through a bounded queue that
 * never blocks the caller.
 */

#ifndef NET_UPLINK_H
#define NET_UPLINK_H

#include <stdbool.h>
#include <stddef.h>

#include "pos_sink.h"

/* Start the uplink thread. GATEWAY mode only -- a slave has nothing to publish
 * and starting WiFi there would cost ~50 KB of heap for no benefit. Safe to
 * call when unprovisioned: the thread logs once and idles. */
void net_uplink_start(void);

/* Hand one decoded fix to the uplink. Non-blocking: enqueues and returns, so
 * it is safe from the gateway's dispatch path. Drops the OLDEST queued fix if
 * the queue is full -- a stale position is worthless in an RTLS. */
void net_uplink_submit(const struct pos_fix *fix);

/* Human-readable connection state for `net show`. Never NULL. */
const char *net_uplink_state_str(void);

/* Copy the DHCP-assigned IPv4 address into `buf`. Returns false and leaves
 * `buf` untouched when no address is assigned. */
bool net_uplink_get_ip(char *buf, size_t len);

#endif /* NET_UPLINK_H */

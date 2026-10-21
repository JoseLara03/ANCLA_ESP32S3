/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NODE_CMD_H
#define NODE_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Executes remote MQTT commands on a node (anchor or gateway). The wire format, the HMAC and the
 * challenge live in cmd_proto.c (pure, host-tested); this file is the Zephyr half: it holds the
 * challenge state, applies the rate limit, and performs the whitelisted action through the same
 * setters the shell commands use, so a remote `tau_set` and a console `sync tau set` cannot
 * diverge.
 *
 * Commands (anchor):  status, sync_reset, tau_set a, tau_mode a, ant_set a b, reboot
 * Commands (gateway): status, reboot            -- the rest answer "not_supported"
 *
 * Everything is refused unless a key is stored (`net cmdkey`).
 */

struct node_cmd_result {
	int  ack_len;        /* bytes written to the ack buffer, or -1 */
	bool status_now;     /* publish a status report immediately */
	bool reboot;         /* cold reboot AFTER the ack has been sent */
};

/* `node` is the name commands are signed against and that names this node's
 * topics: the EUI's 16 hex digits on an anchor, "gw" on the gateway (copied). */
void node_cmd_init(bool is_gateway, const char *node);

/* The challenge a command must be signed against; 16 hex characters. Rotated by
 * node_cmd_rotate() with every status report. */
const char *node_cmd_challenge(void);
void node_cmd_rotate(void);

/* Handles one received command payload (not NUL terminated). Always produces an ack -- including
 * for rejected commands, so the sender learns why. */
struct node_cmd_result node_cmd_handle(const char *payload, size_t len, char *ack, size_t ack_cap);

#endif /* NODE_CMD_H */

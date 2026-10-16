/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CMD_PROTO_H
#define CMD_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Remote commands over MQTT: the wire format, the authentication and the challenge. Pure C, no
 * Zephyr, host-tested in tests/cmd_proto/ (and cross-checked against the Python signer in
 * edge/tools/nodecmd.py with a shared test vector).
 *
 * Why this exists and why it is written defensively: a mounted anchor has no console, so every
 * operation that `sync`/`anchor` shell commands offer has to be reachable some other way -- and
 * the way is a shared MQTT broker that other platforms of the same company also use. Anyone able
 * to publish to it could otherwise reboot or recalibrate every anchor in the field.
 *
 * Threat model and mechanism:
 *  - A per-site secret key (set once on the bench with `net cmdkey`) signs every command with
 *    HMAC-SHA256. No key stored => every command is refused (fail closed).
 *  - Replay is stopped by a CHALLENGE, not by a counter: the node publishes a fresh random
 *    challenge in its status report; the command's signature covers it; a node only accepts the
 *    challenge it published, and burns it on use. A counter would have to survive a reboot (the
 *    nodes have no RTC and a replayed `reboot` would loop forever); a challenge cannot.
 *  - The command set is a closed whitelist; arguments are range-checked by the executor.
 *
 * Wire format (one flat JSON object, ASCII, no escapes):
 *   {"c":"tau_set","a":312,"b":0,"ch":"a1b2c3d4e5f60718","sig":"<64 hex>"}
 * "a" and "b" are optional signed integers. The signature is
 *   HMAC-SHA256(key, "<zone>|<node>|<ch>|<c>|<a>|<b>")
 * with a/b as decimal, or empty when absent; <node> is "gw" or the decimal anchor id.
 */

#define CMD_NAME_MAX   16u
#define CMD_CH_HEX_LEN 16u              /* 64-bit challenge */
#define CMD_MAX_LEN    256u             /* longest accepted command payload */

enum cmd_id {
	CMD_NONE = 0,
	CMD_STATUS,        /* publish a status report now */
	CMD_SYNC_RESET,    /* clear the sync residual statistics */
	CMD_TAU_SET,       /* a = tau in signed DTU; persisted, applied live */
	CMD_TAU_MODE,      /* a = 0 legacy, 1 survey; persisted, applied live */
	CMD_ANT_SET,       /* a = ant_delay_tx, b = ant_delay_rx; persisted, applies on reboot */
	CMD_REBOOT,        /* acknowledged first, then a cold reboot */
};

struct cmd_msg {
	enum cmd_id id;
	char        name[CMD_NAME_MAX];
	bool        has_a;
	bool        has_b;
	int32_t     a;
	int32_t     b;
	char        ch[CMD_CH_HEX_LEN + 1];
	uint8_t     sig[32];
};

enum cmd_err {
	CMD_OK = 0,
	CMD_E_FORMAT   = -1,    /* not a command object we understand */
	CMD_E_UNKNOWN  = -2,    /* well formed, command not on the whitelist */
};

/* Parses `payload` (not NUL terminated). Strict: unknown keys, escapes, trailing junk and
 * oversize fields are all CMD_E_FORMAT. Never reads past `len`. */
int cmd_parse(const char *payload, size_t len, struct cmd_msg *out);

/* The exact byte string that is signed. Returns its length, or -1 if it does not fit. */
int cmd_canonical(char *buf, size_t len, const char *zone, const char *node,
		  const struct cmd_msg *m);

/* True only if m->sig is the HMAC of the canonical string under `key`. Constant time. */
bool cmd_verify(const struct cmd_msg *m, const uint8_t *key, size_t key_len, const char *zone,
		const char *node);

/* ---- Challenge ------------------------------------------------------------ */

/* The node keeps the CURRENT challenge and the PREVIOUS one. It is rotated with every status
 * report, so a command signed against the report a server just read is still valid for one more
 * period; and it is burned when a command is accepted. */
struct cmd_challenge {
	char cur[CMD_CH_HEX_LEN + 1];
	char prev[CMD_CH_HEX_LEN + 1];
	bool have_cur;
	bool have_prev;
};

void cmd_challenge_init(struct cmd_challenge *c);
/* New challenge from 64 bits of randomness the caller supplies. */
void cmd_challenge_rotate(struct cmd_challenge *c, uint32_t rand_hi, uint32_t rand_lo);
bool cmd_challenge_accepts(const struct cmd_challenge *c, const char *ch_hex);
/* A command was accepted: both challenges are dead, the caller rotates in a new one. */
void cmd_challenge_burn(struct cmd_challenge *c);

/* ---- Replies --------------------------------------------------------------- */

/* {"c":"tau_set","ok":1,"v":312} or {"c":"tau_set","ok":0,"err":"bad_sig"}. `name` is echoed only
 * if it is short and printable, otherwise "?". Returns the length or -1 if it does not fit. */
int cmd_ack_json(char *buf, size_t len, const char *name, bool ok, const char *err,
		 bool has_value, int32_t value);

#endif /* CMD_PROTO_H */

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CMD_KEY_H
#define CMD_KEY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The per-site secret that authenticates remote commands (cmd_proto.h). Stored in NVS under
 * "cmd/key". With no key stored every remote command is refused -- fail closed -- so an
 * unprovisioned node is not remotely controllable by anyone.
 *
 * It is set once, on the bench, with `net cmdkey <hex>`; it is never printed back (`net show`
 * reports only set/unset). Loaded by settings_load() through a static settings handler, so it is
 * available as soon as uwb_store_init() has run. */
#define CMD_KEY_MIN 16u
#define CMD_KEY_MAX 32u

/* Pointer to the key and its length, or NULL / 0 when none is stored. */
const uint8_t *cmd_key_get(size_t *len);
bool cmd_key_is_set(void);

/* 8 hex characters + NUL: the first 4 bytes of SHA-256(key). Not the key -- a fingerprint that lets
 * the bench and the server confirm they hold the same one. Empty string when no key is stored. */
void cmd_key_fingerprint(char out[9]);

/* Validates (16..32 bytes), stores in RAM and persists. 0 on success, negative errno. */
int cmd_key_set(const uint8_t *key, size_t len);
int cmd_key_clear(void);

#endif /* CMD_KEY_H */

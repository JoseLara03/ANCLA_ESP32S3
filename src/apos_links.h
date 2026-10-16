/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The persisted form of the static sync links a survey installed, plus the
 * "apply in progress" marker. Pure C: the byte codec is host-tested, and only
 * apos_store.c touches settings.
 *
 * Why an explicit, versioned codec instead of memcpy of a struct, which is what
 * the older survey blob does (and why that blob is left alone): a struct blob
 * is validated only by its sizeof, so any added field or padding change turns a
 * stored record into garbage that still passes the size check, or into a
 * "treat as no survey" that silently drops a calibrated install. Here every
 * field has a fixed offset, the version byte is checked first, and an unknown
 * version is REJECTED (kept on flash, reported) rather than reinterpreted.
 *
 * Layout, little-endian, all fields fixed-width:
 *
 *   links record   [0]     version (APOS_LINKS_VERSION)
 *                  [1..4]  generation u32
 *                  [5]     n entries (<= APOS_LINKS_MAX)
 *                  then n x 22 bytes:
 *                    child EUI(8) parent EUI(8) tau_dtu i32 valid u8 source u8
 *
 *   pending record [0]     version
 *                  [1..4]  generation u32 that was being applied
 *                  [5]     state (APOS_PENDING_*)
 */

#ifndef APOS_LINKS_H
#define APOS_LINKS_H

#include "apos_table.h"   /* APOS_EUI_LEN, APOS_MAX_NODES */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APOS_LINKS_VERSION 1u
#define APOS_LINKS_MAX     APOS_MAX_NODES

#define APOS_LINK_ENTRY_LEN (APOS_EUI_LEN + APOS_EUI_LEN + 4u + 1u + 1u)
#define APOS_LINKS_HDR_LEN  6u
#define APOS_LINKS_MAX_LEN  (APOS_LINKS_HDR_LEN + APOS_LINKS_MAX * APOS_LINK_ENTRY_LEN)
#define APOS_PENDING_LEN    6u

struct apos_link_rec {
	uint8_t child_eui[APOS_EUI_LEN];
	uint8_t parent_eui[APOS_EUI_LEN];
	int32_t tau_dtu;
	bool    valid;   /* explicit: tau_dtu == 0 is a legal calibrated value */
	uint8_t source;  /* enum sync_link_source */
};

struct apos_links {
	uint32_t generation;
	uint8_t  n;
	struct apos_link_rec rec[APOS_LINKS_MAX];
};

enum apos_pending_state {
	APOS_PENDING_NONE     = 0,
	APOS_PENDING_APPLYING = 1, /* anchors may hold a mix of generations */
};

struct apos_pending {
	uint32_t generation;
	uint8_t  state; /* enum apos_pending_state */
};

/* Returns the bytes written, or -EINVAL (NULL, or n > APOS_LINKS_MAX) /
 * -EMSGSIZE (cap too small). */
int apos_links_encode(const struct apos_links *l, uint8_t *buf, size_t cap);

/* Returns 0, or -EINVAL on NULL, -EBADMSG when the length does not match what
 * the header claims (truncation, trailing bytes, n > max), -ENOTSUP for a
 * version this firmware does not know. *out is untouched on failure. */
int apos_links_decode(const uint8_t *buf, size_t len, struct apos_links *out);

int apos_pending_encode(const struct apos_pending *p, uint8_t *buf, size_t cap);
int apos_pending_decode(const uint8_t *buf, size_t len, struct apos_pending *out);

/* The link whose child is this EUI, or NULL. Lookups are by EUI, never by
 * position in the array or by short address. */
const struct apos_link_rec *apos_links_find(const struct apos_links *l,
					    const uint8_t child_eui[APOS_EUI_LEN]);

#endif /* APOS_LINKS_H */

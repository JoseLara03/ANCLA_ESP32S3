/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's EUI -> short address map, and the policy that decides which
 * board gets which address on `apos assign`. Pure C: the codec and the policy
 * are host-tested (tests/apos_addrmap/), and only apos_store.c touches
 * settings. See docs/superpowers/specs/2026-10-21-anchor-eui-identity-design.md
 * D4 and D6.
 *
 * The map is a PERMANENT reservation, like a DHCP reservation: an entry for an
 * anchor that is off on the day of an assign is kept, so that board does not
 * lose its id and its address is not handed to another board while it is still
 * mounted. Only `apos forget <eui>` frees one. This is the anchor-side cousin of
 * gw_core's tag-address quarantine, in a simpler form: replacing a board is an
 * explicit operator act here, not a timeout.
 *
 * Layout of the persisted record, little-endian, fixed offsets (the same
 * discipline as apos_links.h, for the same reasons):
 *
 *   [0]   version (APOS_ADDRMAP_VERSION)
 *   [1]   n entries (<= APOS_ADDRMAP_MAX)
 *   then n x 10 bytes: EUI(8) addr u16
 */

#ifndef APOS_ADDRMAP_H
#define APOS_ADDRMAP_H

#include "apos_table.h"   /* APOS_EUI_LEN, APOS_MAX_NODES */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APOS_ADDRMAP_VERSION   1u
#define APOS_ADDRMAP_MAX       APOS_MAX_NODES
#define APOS_ADDRMAP_HDR_LEN   2u
#define APOS_ADDRMAP_ENTRY_LEN (APOS_EUI_LEN + 2u)
#define APOS_ADDRMAP_MAX_LEN   (APOS_ADDRMAP_HDR_LEN + \
				APOS_ADDRMAP_MAX * APOS_ADDRMAP_ENTRY_LEN)

struct apos_addrmap_ent {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t addr;
};

struct apos_addrmap {
	uint8_t n;
	struct apos_addrmap_ent e[APOS_ADDRMAP_MAX];
};

/* Returns the bytes written, or -EINVAL (NULL, n > max) / -EMSGSIZE. */
int apos_addrmap_encode(const struct apos_addrmap *m, uint8_t *buf, size_t cap);

/* Returns 0, or -EINVAL on NULL, -ENOTSUP for an unknown version, -EBADMSG for
 * a length that disagrees with n, n > max, an all-zero EUI, or a repeated EUI
 * or address. *out is untouched on failure: a blob that cannot be read is
 * REJECTED and left on flash, never reinterpreted or erased. */
int apos_addrmap_decode(const uint8_t *buf, size_t len, struct apos_addrmap *out);

/* Index of the entry, or -1. */
int apos_addrmap_find_eui(const struct apos_addrmap *m,
			  const uint8_t eui[APOS_EUI_LEN]);
int apos_addrmap_find_addr(const struct apos_addrmap *m, uint16_t addr);

/* Insert or move an entry. 0 on success; -EINVAL for a NULL/all-zero EUI;
 * -EADDRINUSE if a DIFFERENT EUI holds `addr`; -ENOSPC when full. */
int apos_addrmap_put(struct apos_addrmap *m, const uint8_t eui[APOS_EUI_LEN],
		     uint16_t addr);

/* 0, or -ENOENT. Keeps the remaining entries in order. */
int apos_addrmap_forget(struct apos_addrmap *m, const uint8_t eui[APOS_EUI_LEN]);

/* ---- Assignment policy ---- */

/* One enumerated board: its EUI and the address it answered from (possibly
 * UWB_ANCHOR_ADDR_UNASSIGNED, or one another board shares). */
struct apos_plan_peer {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t addr;
};

enum apos_plan_action {
	APOS_PLAN_KEEP    = 0, /* in the map and already at its address */
	APOS_PLAN_ADOPT   = 1, /* not in the map; its own address is kept and
				* goes into the map -- no frame, no reboot */
	APOS_PLAN_ASSIGN  = 2, /* send SETADDR with `addr` */
	APOS_PLAN_NOSPACE = 3, /* only in a -ENOSPC result: could not be placed */
};

struct apos_plan_out {
	uint8_t  action;  /* enum apos_plan_action */
	uint16_t addr;    /* the address it ends up with */
};

/* Decide every peer's address. Rules, in order (spec D6):
 *
 *   1. The map rules. A peer in the map gets its mapped address: KEEP if it
 *      already answers from it, ASSIGN otherwise.
 *   2. Adopt what already works. A peer NOT in the map whose own address is in
 *      [addr_lo, addr_hi), not reserved in the map by another EUI, and claimed
 *      by no other peer that outranks it, is ADOPTed as is -- a hand-numbered
 *      fleet goes through an assign with zero frames.
 *   3. Duplicates the map does not settle: among non-map peers sharing one
 *      address, the LOWEST EUI (byte-wise) keeps it; a mapped peer whose mapped
 *      address it is outranks them all.
 *   4. Everyone else (UNASSIGNED, losers of rule 3, out-of-range addresses)
 *      gets the lowest address in range that is neither in the map nor held by
 *      a peer, handed out in EUI order.
 *   5. No room -- in the address range or in the map -- refuses the WHOLE plan:
 *      returns -ENOSPC with the unplaceable peers marked APOS_PLAN_NOSPACE.
 *      There is no partial assign.
 *
 * Deterministic: the same peers in any order give the same per-peer result.
 * Returns 0, -ENOSPC, or -EINVAL (NULL, n > APOS_ADDRMAP_MAX, an all-zero or
 * repeated EUI among the peers, or addr_lo >= addr_hi). `*n_assign`, if not
 * NULL, receives the number of ASSIGN actions. */
int apos_addr_plan(const struct apos_addrmap *m,
		   const struct apos_plan_peer *peers, uint8_t n,
		   uint16_t addr_lo, uint16_t addr_hi,
		   struct apos_plan_out *out, uint8_t *n_assign);

#endif /* APOS_ADDRMAP_H */

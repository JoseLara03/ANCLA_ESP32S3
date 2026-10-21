/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_addrmap.h.
 */

#include "apos_addrmap.h"

#include <errno.h>
#include <string.h>

static bool eui_is_zero(const uint8_t eui[APOS_EUI_LEN])
{
	for (unsigned int i = 0; i < APOS_EUI_LEN; i++) {
		if (eui[i] != 0u) {
			return false;
		}
	}
	return true;
}

/* ---- Codec ---- */

int apos_addrmap_encode(const struct apos_addrmap *m, uint8_t *buf, size_t cap)
{
	if (!m || !buf || m->n > APOS_ADDRMAP_MAX) {
		return -EINVAL;
	}

	size_t need = APOS_ADDRMAP_HDR_LEN + (size_t)m->n * APOS_ADDRMAP_ENTRY_LEN;

	if (cap < need) {
		return -EMSGSIZE;
	}

	buf[0] = APOS_ADDRMAP_VERSION;
	buf[1] = m->n;

	uint8_t *p = &buf[APOS_ADDRMAP_HDR_LEN];

	for (uint8_t k = 0; k < m->n; k++) {
		memcpy(p, m->e[k].eui, APOS_EUI_LEN);
		p[APOS_EUI_LEN]      = (uint8_t)m->e[k].addr;
		p[APOS_EUI_LEN + 1u] = (uint8_t)(m->e[k].addr >> 8);
		p += APOS_ADDRMAP_ENTRY_LEN;
	}
	return (int)need;
}

int apos_addrmap_decode(const uint8_t *buf, size_t len, struct apos_addrmap *out)
{
	if (!buf || !out) {
		return -EINVAL;
	}
	if (len < APOS_ADDRMAP_HDR_LEN) {
		return -EBADMSG;
	}
	if (buf[0] != APOS_ADDRMAP_VERSION) {
		return -ENOTSUP;
	}

	uint8_t n = buf[1];

	if (n > APOS_ADDRMAP_MAX ||
	    len != APOS_ADDRMAP_HDR_LEN + (size_t)n * APOS_ADDRMAP_ENTRY_LEN) {
		return -EBADMSG;
	}

	struct apos_addrmap tmp;

	memset(&tmp, 0, sizeof(tmp));

	const uint8_t *p = &buf[APOS_ADDRMAP_HDR_LEN];

	for (uint8_t k = 0; k < n; k++) {
		memcpy(tmp.e[k].eui, p, APOS_EUI_LEN);
		tmp.e[k].addr = (uint16_t)(p[APOS_EUI_LEN] |
					   ((uint16_t)p[APOS_EUI_LEN + 1u] << 8));
		p += APOS_ADDRMAP_ENTRY_LEN;

		if (eui_is_zero(tmp.e[k].eui)) {
			return -EBADMSG;
		}
		for (uint8_t j = 0; j < k; j++) {
			if (memcmp(tmp.e[j].eui, tmp.e[k].eui, APOS_EUI_LEN) == 0 ||
			    tmp.e[j].addr == tmp.e[k].addr) {
				return -EBADMSG;
			}
		}
	}
	tmp.n = n;
	*out = tmp;
	return 0;
}

/* ---- Lookups and edits ---- */

int apos_addrmap_find_eui(const struct apos_addrmap *m,
			  const uint8_t eui[APOS_EUI_LEN])
{
	if (!m || !eui) {
		return -1;
	}
	for (uint8_t k = 0; k < m->n; k++) {
		if (memcmp(m->e[k].eui, eui, APOS_EUI_LEN) == 0) {
			return k;
		}
	}
	return -1;
}

int apos_addrmap_find_addr(const struct apos_addrmap *m, uint16_t addr)
{
	if (!m) {
		return -1;
	}
	for (uint8_t k = 0; k < m->n; k++) {
		if (m->e[k].addr == addr) {
			return k;
		}
	}
	return -1;
}

int apos_addrmap_put(struct apos_addrmap *m, const uint8_t eui[APOS_EUI_LEN],
		     uint16_t addr)
{
	if (!m || !eui || eui_is_zero(eui)) {
		return -EINVAL;
	}

	int holder = apos_addrmap_find_addr(m, addr);
	int self = apos_addrmap_find_eui(m, eui);

	if (holder >= 0 && holder != self) {
		return -EADDRINUSE;
	}
	if (self >= 0) {
		m->e[self].addr = addr;
		return 0;
	}
	if (m->n >= APOS_ADDRMAP_MAX) {
		return -ENOSPC;
	}
	memcpy(m->e[m->n].eui, eui, APOS_EUI_LEN);
	m->e[m->n].addr = addr;
	m->n++;
	return 0;
}

int apos_addrmap_forget(struct apos_addrmap *m, const uint8_t eui[APOS_EUI_LEN])
{
	int k = apos_addrmap_find_eui(m, eui);

	if (k < 0) {
		return -ENOENT;
	}
	for (uint8_t j = (uint8_t)k; j + 1u < m->n; j++) {
		m->e[j] = m->e[j + 1u];
	}
	m->n--;
	memset(&m->e[m->n], 0, sizeof(m->e[m->n]));
	return 0;
}

/* ---- Policy ---- */

static bool in_range(uint16_t a, uint16_t lo, uint16_t hi)
{
	return a >= lo && a < hi;
}

/* Byte-wise EUI order: the rule-3 tie-break and the rule-4 hand-out order. */
static int eui_cmp(const uint8_t a[APOS_EUI_LEN], const uint8_t b[APOS_EUI_LEN])
{
	return memcmp(a, b, APOS_EUI_LEN);
}

int apos_addr_plan(const struct apos_addrmap *m,
		   const struct apos_plan_peer *peers, uint8_t n,
		   uint16_t addr_lo, uint16_t addr_hi,
		   struct apos_plan_out *out, uint8_t *n_assign)
{
	/* Per peer: the map entry (rule 1), whether it is decided yet. */
	int  map_idx[APOS_ADDRMAP_MAX];
	bool decided[APOS_ADDRMAP_MAX];
	uint8_t new_entries = 0;
	uint8_t assigns = 0;
	int rc = 0;

	if (!m || !peers || !out || n > APOS_ADDRMAP_MAX ||
	    addr_lo >= addr_hi || m->n > APOS_ADDRMAP_MAX) {
		return -EINVAL;
	}
	for (uint8_t i = 0; i < n; i++) {
		if (eui_is_zero(peers[i].eui)) {
			return -EINVAL;
		}
		for (uint8_t j = 0; j < i; j++) {
			if (eui_cmp(peers[i].eui, peers[j].eui) == 0) {
				return -EINVAL;
			}
		}
	}

	/* Rule 1: the map rules. An entry whose address fell outside the range
	 * (a narrower cap than when it was written) is treated as no entry: the
	 * board is re-placed by rule 4 and its entry moved by the caller. */
	for (uint8_t i = 0; i < n; i++) {
		int k = apos_addrmap_find_eui(m, peers[i].eui);

		decided[i] = false;
		map_idx[i] = (k >= 0 && in_range(m->e[k].addr, addr_lo, addr_hi)) ? k : -1;
		if (map_idx[i] >= 0) {
			uint16_t want = m->e[map_idx[i]].addr;

			out[i].addr = want;
			out[i].action = (peers[i].addr == want) ? APOS_PLAN_KEEP
								: APOS_PLAN_ASSIGN;
			decided[i] = true;
		}
	}

	/* Rules 2 and 3: adopt a non-map peer's own address when nothing
	 * outranks it. Outranked by: a map entry for that address belonging to
	 * ANY other EUI (present or not -- a reservation), or another non-map
	 * peer at the same address with a lower EUI. */
	for (uint8_t i = 0; i < n; i++) {
		if (decided[i] || !in_range(peers[i].addr, addr_lo, addr_hi)) {
			continue;
		}

		int holder = apos_addrmap_find_addr(m, peers[i].addr);
		bool outranked = (holder >= 0 &&
				  eui_cmp(m->e[holder].eui, peers[i].eui) != 0);

		for (uint8_t j = 0; j < n && !outranked; j++) {
			if (j == i || map_idx[j] >= 0 || peers[j].addr != peers[i].addr) {
				continue;
			}
			if (eui_cmp(peers[j].eui, peers[i].eui) < 0) {
				outranked = true;
			}
		}
		/* Adopting needs a map entry; with none left the peer falls
		 * through to rule 4, which reports it as NOSPACE. */
		if (!outranked && m->n + new_entries < APOS_ADDRMAP_MAX) {
			out[i].action = APOS_PLAN_ADOPT;
			out[i].addr = peers[i].addr;
			decided[i] = true;
			new_entries++;
		}
	}

	/* Rule 4: everyone left, in EUI order, gets the lowest free address. Free
	 * means: not in the map (any entry, present or absent) and not already
	 * the outcome of a decided peer. */
	for (;;) {
		int pick = -1;

		for (uint8_t i = 0; i < n; i++) {
			if (!decided[i] &&
			    (pick < 0 || eui_cmp(peers[i].eui, peers[pick].eui) < 0)) {
				pick = i;
			}
		}
		if (pick < 0) {
			break;
		}

		uint16_t a;
		bool found = false;

		for (a = addr_lo; a < addr_hi; a++) {
			bool taken = apos_addrmap_find_addr(m, a) >= 0;

			for (uint8_t j = 0; j < n && !taken; j++) {
				if (decided[j] && out[j].action != APOS_PLAN_NOSPACE &&
				    out[j].addr == a) {
					taken = true;
				}
			}
			if (!taken) {
				found = true;
				break;
			}
		}

		/* A peer whose stale map entry fell out of range already owns a
		 * slot in the map; anyone else needs a new one. */
		bool needs_entry = apos_addrmap_find_eui(m, peers[pick].eui) < 0;

		if (!found || (needs_entry &&
			       m->n + new_entries >= APOS_ADDRMAP_MAX)) {
			out[pick].action = APOS_PLAN_NOSPACE;
			out[pick].addr = peers[pick].addr;
			rc = -ENOSPC;
		} else {
			out[pick].action = APOS_PLAN_ASSIGN;
			out[pick].addr = a;
			if (needs_entry) {
				new_entries++;
			}
		}
		decided[pick] = true;
	}

	for (uint8_t i = 0; i < n; i++) {
		if (out[i].action == APOS_PLAN_ASSIGN) {
			assigns++;
		}
	}
	if (n_assign) {
		*n_assign = assigns;
	}
	return rc;
}

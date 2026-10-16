/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_links.h.
 */

#include "apos_links.h"

#include <errno.h>
#include <string.h>

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int apos_links_encode(const struct apos_links *l, uint8_t *buf, size_t cap)
{
	if (!l || !buf || l->n > APOS_LINKS_MAX) {
		return -EINVAL;
	}

	size_t need = APOS_LINKS_HDR_LEN + (size_t)l->n * APOS_LINK_ENTRY_LEN;

	if (cap < need) {
		return -EMSGSIZE;
	}

	buf[0] = APOS_LINKS_VERSION;
	put_u32(&buf[1], l->generation);
	buf[5] = l->n;

	uint8_t *p = &buf[APOS_LINKS_HDR_LEN];

	for (uint8_t k = 0; k < l->n; k++) {
		memcpy(p, l->rec[k].child_eui, APOS_EUI_LEN);
		memcpy(p + APOS_EUI_LEN, l->rec[k].parent_eui, APOS_EUI_LEN);
		put_u32(p + 2 * APOS_EUI_LEN, (uint32_t)l->rec[k].tau_dtu);
		p[2 * APOS_EUI_LEN + 4] = l->rec[k].valid ? 1u : 0u;
		p[2 * APOS_EUI_LEN + 5] = l->rec[k].source;
		p += APOS_LINK_ENTRY_LEN;
	}
	return (int)need;
}

int apos_links_decode(const uint8_t *buf, size_t len, struct apos_links *out)
{
	if (!buf || !out) {
		return -EINVAL;
	}
	if (len < APOS_LINKS_HDR_LEN) {
		return -EBADMSG;
	}
	if (buf[0] != APOS_LINKS_VERSION) {
		return -ENOTSUP;
	}

	uint8_t n = buf[5];

	if (n > APOS_LINKS_MAX ||
	    len != APOS_LINKS_HDR_LEN + (size_t)n * APOS_LINK_ENTRY_LEN) {
		return -EBADMSG;
	}

	struct apos_links tmp;

	memset(&tmp, 0, sizeof(tmp));
	tmp.generation = get_u32(&buf[1]);
	tmp.n = n;

	const uint8_t *p = &buf[APOS_LINKS_HDR_LEN];

	for (uint8_t k = 0; k < n; k++) {
		/* valid is 0 or 1 and nothing else: a stray byte here is
		 * corruption, not "truthy". */
		if (p[2 * APOS_EUI_LEN + 4] > 1u) {
			return -EBADMSG;
		}
		memcpy(tmp.rec[k].child_eui, p, APOS_EUI_LEN);
		memcpy(tmp.rec[k].parent_eui, p + APOS_EUI_LEN, APOS_EUI_LEN);
		tmp.rec[k].tau_dtu = (int32_t)get_u32(p + 2 * APOS_EUI_LEN);
		tmp.rec[k].valid = p[2 * APOS_EUI_LEN + 4] == 1u;
		tmp.rec[k].source = p[2 * APOS_EUI_LEN + 5];
		p += APOS_LINK_ENTRY_LEN;
	}
	*out = tmp;
	return 0;
}

int apos_pending_encode(const struct apos_pending *p, uint8_t *buf, size_t cap)
{
	if (!p || !buf) {
		return -EINVAL;
	}
	if (cap < APOS_PENDING_LEN) {
		return -EMSGSIZE;
	}
	buf[0] = APOS_LINKS_VERSION;
	put_u32(&buf[1], p->generation);
	buf[5] = p->state;
	return (int)APOS_PENDING_LEN;
}

int apos_pending_decode(const uint8_t *buf, size_t len, struct apos_pending *out)
{
	if (!buf || !out) {
		return -EINVAL;
	}
	if (len != APOS_PENDING_LEN) {
		return -EBADMSG;
	}
	if (buf[0] != APOS_LINKS_VERSION) {
		return -ENOTSUP;
	}
	if (buf[5] > APOS_PENDING_APPLYING) {
		return -EBADMSG;
	}
	out->generation = get_u32(&buf[1]);
	out->state = buf[5];
	return 0;
}

const struct apos_link_rec *apos_links_find(const struct apos_links *l,
					    const uint8_t child_eui[APOS_EUI_LEN])
{
	if (!l || !child_eui) {
		return NULL;
	}
	for (uint8_t k = 0; k < l->n; k++) {
		if (memcmp(l->rec[k].child_eui, child_eui, APOS_EUI_LEN) == 0) {
			return &l->rec[k];
		}
	}
	return NULL;
}

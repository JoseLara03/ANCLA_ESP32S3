/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See blink_frame.h. The field helpers mirror ccp_frame.c's, which in turn
 * mirror apos_frame.c's and uwb_frame_802_15_4z.c's, for the reason
 * apos_frame.c states: the shared codec is byte-identical with the tag's
 * copy and exports none of them, and a few one-line helpers are a smaller
 * price than making it export internals.
 */

#include "blink_frame.h"

#include <errno.h>

#define OFF_SEQ_HDR 2u
#define OFF_DEST    5u
#define OFF_SRC     7u
#define OFF_TYPE    9u

#define ADDR_BCAST  0xFFFFu

/* ---- Little-endian field helpers ---- */
static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

int blink_frame_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
		      uint8_t frame_seq, const struct blink_frame *f)
{
	if (!buf || !f) {
		return -EINVAL;
	}
	if (buf_len < BLINK_FRAME_LEN) {
		return -EMSGSIZE;
	}

	buf[0] = 0x41;
	buf[1] = 0x88;
	buf[OFF_SEQ_HDR] = frame_seq;
	buf[3] = 0xCA;
	buf[4] = 0xDE;
	put_u16(&buf[OFF_DEST], ADDR_BCAST);
	put_u16(&buf[OFF_SRC], src_addr);
	buf[OFF_TYPE] = BLINK_FRAME_TYPE;

	buf[BLINK_OFF_SEQ]   = f->seq;
	buf[BLINK_OFF_SOC]   = f->batt_soc;
	buf[BLINK_OFF_FLAGS] = (uint8_t)(f->flags & ~BLINK_FLAG_RESERVED_MASK);
	buf[BLINK_OFF_RSVD]  = 0;

	return (int)BLINK_FRAME_LEN;
}

bool blink_frame_is_blink(const uint8_t *buf, size_t len)
{
	/* `len >=` rather than `==`: dwt_getframelength() and
	 * cb_data->datalength both INCLUDE the 2-byte FCS, and this project has
	 * been bitten by that before (see CLAUDE.md). A caller that has not
	 * subtracted FCS_LEN still gets a correct answer here. */
	return buf && len >= BLINK_FRAME_LEN && buf[0] == 0x41 && buf[1] == 0x88 &&
	       buf[3] == 0xCA && buf[4] == 0xDE &&
	       buf[OFF_TYPE] == BLINK_FRAME_TYPE;
}

int blink_frame_parse(const uint8_t *buf, size_t len, struct blink_frame *out)
{
	if (!out) {
		return -EINVAL;
	}
	if (!blink_frame_is_blink(buf, len)) {
		return -EINVAL;
	}
	/* Reserved bits must be zero. Accepting them today makes them unusable
	 * tomorrow without breaking compatibility. */
	if (buf[BLINK_OFF_FLAGS] & BLINK_FLAG_RESERVED_MASK) {
		return -EPROTO;
	}
	if (buf[BLINK_OFF_RSVD] != 0) {
		return -EPROTO;
	}

	out->src_addr = get_u16(&buf[OFF_SRC]);
	out->seq      = buf[BLINK_OFF_SEQ];
	out->batt_soc = buf[BLINK_OFF_SOC];
	out->flags    = buf[BLINK_OFF_FLAGS];

	return 0;
}

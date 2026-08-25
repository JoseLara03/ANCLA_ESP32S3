/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See ccp_frame.h. The field helpers mirror apos_frame.c's, which in turn mirror
 * uwb_frame_802_15_4z.c's, for the reason apos_frame.c states: the shared codec
 * is byte-identical with the tag's copy and exports none of them, and a few
 * one-line helpers are a smaller price than making it export internals.
 */

#include "ccp_frame.h"

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

/* 40-bit DW3220 timestamp, little-endian, five bytes. Its own helper rather
 * than a put_u64 because the field width is what makes it a device timestamp:
 * writing eight bytes here would silently overrun root_id. */
static void put_u40(uint8_t *p, uint64_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
	p[4] = (uint8_t)(v >> 32);
}

static uint64_t get_u40(const uint8_t *p)
{
	return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) |
	       ((uint64_t)p[3] << 24) | ((uint64_t)p[4] << 32);
}

bool ccp_hop_adoptable(uint8_t hop)
{
	/* CCP_HOP_UNSYNCED is excluded by the bound, not by a special case: it
	 * is 0xFF and CCP_HOP_MAX is small. Stated anyway because a future
	 * CCP_HOP_MAX of 255 would silently make unsynced masters adoptable. */
	return hop <= CCP_HOP_MAX && hop != CCP_HOP_UNSYNCED;
}

int ccp_frame_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
		    uint8_t frame_seq, const struct ccp_frame *f)
{
	if (!buf || !f) {
		return -EINVAL;
	}
	if (buf_len < CCP_FRAME_LEN) {
		return -EMSGSIZE;
	}
	/* A timestamp with bits above 40 set is a caller bug -- most likely a
	 * raw uint64 counter rather than a masked device timestamp. Rejected
	 * rather than truncated: a truncated timestamp parses cleanly at the
	 * far end and produces a plausible-looking but wrong sync observation,
	 * which is the worst possible failure for this frame. */
	if (f->tx_dtu > 0xFFFFFFFFFFULL) {
		return -EINVAL;
	}

	buf[0] = 0x41;
	buf[1] = 0x88;
	buf[OFF_SEQ_HDR] = frame_seq;
	buf[3] = 0xCA;
	buf[4] = 0xDE;
	put_u16(&buf[OFF_DEST], ADDR_BCAST);
	put_u16(&buf[OFF_SRC], src_addr);
	buf[OFF_TYPE] = CCP_FRAME_TYPE;

	buf[CCP_OFF_SEQ] = f->seq;
	buf[CCP_OFF_HOP] = f->hop;
	put_u40(&buf[CCP_OFF_TX_DTU], f->tx_dtu);
	put_u32(&buf[CCP_OFF_ROOT_ID], f->root_id);

	return (int)CCP_FRAME_LEN;
}

bool ccp_frame_is_ccp(const uint8_t *buf, size_t len)
{
	/* `len >=` rather than `==`: dwt_getframelength() and
	 * cb_data->datalength both INCLUDE the 2-byte FCS, and this project has
	 * been bitten by that before (see CLAUDE.md). A caller that has not
	 * subtracted FCS_LEN still gets a correct answer here. */
	return buf && len >= CCP_FRAME_LEN && buf[0] == 0x41 && buf[1] == 0x88 &&
	       buf[3] == 0xCA && buf[4] == 0xDE &&
	       buf[OFF_TYPE] == CCP_FRAME_TYPE;
}

int ccp_frame_parse(const uint8_t *buf, size_t len, struct ccp_frame *out)
{
	if (!out) {
		return -EINVAL;
	}
	if (!ccp_frame_is_ccp(buf, len)) {
		return -EINVAL;
	}

	uint8_t hop = buf[CCP_OFF_HOP];

	/* Refused here rather than left to the caller. Adopting an unsynced or
	 * too-deep master is the one mistake that corrupts the time base
	 * silently -- every conversion downstream stays plausible -- so the
	 * check lives where it cannot be forgotten. */
	if (!ccp_hop_adoptable(hop)) {
		return -EPROTO;
	}

	out->seq      = buf[CCP_OFF_SEQ];
	out->hop      = hop;
	out->tx_dtu   = get_u40(&buf[CCP_OFF_TX_DTU]);
	out->root_id  = get_u32(&buf[CCP_OFF_ROOT_ID]);
	out->src_addr = get_u16(&buf[OFF_SRC]);
	return 0;
}

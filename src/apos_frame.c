/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_frame.h. The field helpers mirror uwb_frame_802_15_4z.c's rather
 * than being shared with it: that file is byte-identical to the tag's copy and
 * exports none of them, and four one-line helpers are a smaller price than
 * making the shared codec export internals.
 */

#include "apos_frame.h"

#include <errno.h>
#include <string.h>

#define OFF_SEQ     2u
#define OFF_DEST    5u
#define OFF_SRC     7u
#define OFF_TYPE    9u
#define OFF_SUB     10u
#define OFF_PAYLOAD 11u

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

/* memcpy through a uint32_t rather than a cast: type-punning a float* to a
 * uint32_t* is undefined behaviour and gcc does miscompile it at -O2. */
static void put_f32(uint8_t *p, float v)
{
	uint32_t u;

	memcpy(&u, &v, sizeof(u));
	put_u32(p, u);
}

static float get_f32(const uint8_t *p)
{
	uint32_t u = get_u32(p);
	float v;

	memcpy(&v, &u, sizeof(v));
	return v;
}

static void put_xyz(uint8_t *p, float x, float y, float z)
{
	put_f32(p, x);
	put_f32(p + 4, y);
	put_f32(p + 8, z);
}

static void get_xyz(const uint8_t *p, float *x, float *y, float *z)
{
	*x = get_f32(p);
	*y = get_f32(p + 4);
	*z = get_f32(p + 8);
}

static int write_hdr(uint8_t *buf, size_t buf_len, size_t need, uint16_t dest,
		     uint16_t src, uint8_t subtype)
{
	if (!buf) {
		return -EINVAL;
	}
	if (buf_len < need) {
		return -EMSGSIZE;
	}
	buf[0] = 0x41;
	buf[1] = 0x88;
	buf[OFF_SEQ] = 0; /* caller sets via apos_frame_set_seq() */
	buf[3] = 0xCA;
	buf[4] = 0xDE;
	put_u16(&buf[OFF_DEST], dest);
	put_u16(&buf[OFF_SRC], src);
	buf[OFF_TYPE] = APOS_FRAME_TYPE;
	buf[OFF_SUB] = subtype;
	return 0;
}

static bool known_subtype(uint8_t s)
{
	return s >= APOS_SUB_SURVEY_BEGIN && s <= APOS_SUB_SURVEY_END;
}

bool apos_frame_is_apos(const uint8_t *buf, size_t len)
{
	if (!buf || len < APOS_HDR_LEN) {
		return false;
	}
	if (buf[0] != 0x41 || buf[1] != 0x88) {
		return false;
	}
	if (buf[3] != 0xCA || buf[4] != 0xDE) {
		return false;
	}
	if (buf[OFF_TYPE] != APOS_FRAME_TYPE) {
		return false;
	}
	return known_subtype(buf[OFF_SUB]);
}

uint8_t apos_frame_subtype(const uint8_t *buf)
{
	return buf[OFF_SUB];
}

uint16_t apos_frame_src(const uint8_t *buf)
{
	return get_u16(&buf[OFF_SRC]);
}

uint16_t apos_frame_dest(const uint8_t *buf)
{
	return get_u16(&buf[OFF_DEST]);
}

void apos_frame_set_seq(uint8_t *buf, uint8_t seq)
{
	buf[OFF_SEQ] = seq;
}

/* Shared entry check for every parser: an APOS frame of exactly this subtype
 * and exactly this length. Exact, not >=, because a frame length that does not
 * match is either a truncation or an unsubtracted FCS -- both are bugs worth
 * surfacing rather than parsing around. */
static int parse_check(const uint8_t *buf, size_t len, uint8_t subtype,
		       size_t expect)
{
	if (!apos_frame_is_apos(buf, len)) {
		return -EINVAL;
	}
	if (buf[OFF_SUB] != subtype || len != expect) {
		return -EINVAL;
	}
	return 0;
}

/* ---- SURVEY_BEGIN ---- */
int apos_frame_survey_begin_build(uint8_t *buf, size_t buf_len, uint16_t src,
				  uint16_t session, uint16_t window_s)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SURVEY_BEGIN, APOS_ADDR_BCAST,
			   src, APOS_SUB_SURVEY_BEGIN);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], window_s);
	return (int)APOS_LEN_SURVEY_BEGIN;
}

int apos_frame_parse_survey_begin(const uint8_t *buf, size_t len,
				  uint16_t *session, uint16_t *window_s)
{
	int rc = parse_check(buf, len, APOS_SUB_SURVEY_BEGIN,
			     APOS_LEN_SURVEY_BEGIN);

	if (rc || !session || !window_s) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*window_s = get_u16(&buf[OFF_PAYLOAD + 2]);
	return 0;
}

/* ---- ENUM_RSP ---- */
int apos_frame_enum_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			      uint16_t dest, uint16_t session,
			      const uint8_t eui[APOS_EUI_LEN], bool pos_valid,
			      float x, float y, float z)
{
	int rc;

	if (!eui) {
		return -EINVAL;
	}

	rc = write_hdr(buf, buf_len, APOS_LEN_ENUM_RSP, dest, src,
		       APOS_SUB_ENUM_RSP);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	memcpy(&buf[OFF_PAYLOAD + 2], eui, APOS_EUI_LEN);
	buf[OFF_PAYLOAD + 10] = pos_valid ? 1u : 0u;
	put_xyz(&buf[OFF_PAYLOAD + 11], x, y, z);
	return (int)APOS_LEN_ENUM_RSP;
}

int apos_frame_parse_enum_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			      uint8_t eui_out[APOS_EUI_LEN], bool *pos_valid,
			      float *x, float *y, float *z)
{
	int rc = parse_check(buf, len, APOS_SUB_ENUM_RSP, APOS_LEN_ENUM_RSP);

	if (rc) {
		return rc;
	}
	if (!session || !eui_out || !pos_valid || !x || !y || !z) {
		return -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	memcpy(eui_out, &buf[OFF_PAYLOAD + 2], APOS_EUI_LEN);
	*pos_valid = buf[OFF_PAYLOAD + 10] != 0u;
	get_xyz(&buf[OFF_PAYLOAD + 11], x, y, z);
	return 0;
}

/* ---- RANGE_CMD ---- */
int apos_frame_range_cmd_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, uint8_t n_exchanges)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_RANGE_CMD, dest, src,
			   APOS_SUB_RANGE_CMD);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], peer_addr);
	buf[OFF_PAYLOAD + 4] = n_exchanges;
	return (int)APOS_LEN_RANGE_CMD;
}

int apos_frame_parse_range_cmd(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, uint8_t *n_exchanges)
{
	int rc = parse_check(buf, len, APOS_SUB_RANGE_CMD, APOS_LEN_RANGE_CMD);

	if (rc || !session || !peer_addr || !n_exchanges) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*peer_addr = get_u16(&buf[OFF_PAYLOAD + 2]);
	*n_exchanges = buf[OFF_PAYLOAD + 4];
	return 0;
}

/* ---- RANGE_RSP ---- */
int apos_frame_range_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, int32_t mean_mm,
			       uint16_t sd_mm, uint8_t n_ok)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_RANGE_RSP, dest, src,
			   APOS_SUB_RANGE_RSP);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], peer_addr);
	/* Signed-to-unsigned conversion is intentional and well-defined (C
	 * requires wraparound, i.e. two's-complement reinterpretation, for this
	 * cast): the wire carries mean_mm's raw two's-complement bit pattern,
	 * and the parser reverses the same cast on the way back. */
	put_u32(&buf[OFF_PAYLOAD + 4], (uint32_t)mean_mm);
	put_u16(&buf[OFF_PAYLOAD + 8], sd_mm);
	buf[OFF_PAYLOAD + 10] = n_ok;
	return (int)APOS_LEN_RANGE_RSP;
}

int apos_frame_parse_range_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, int32_t *mean_mm,
			       uint16_t *sd_mm, uint8_t *n_ok)
{
	int rc = parse_check(buf, len, APOS_SUB_RANGE_RSP, APOS_LEN_RANGE_RSP);

	if (rc) {
		return rc;
	}
	if (!session || !peer_addr || !mean_mm || !sd_mm || !n_ok) {
		return -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*peer_addr = get_u16(&buf[OFF_PAYLOAD + 2]);
	*mean_mm = (int32_t)get_u32(&buf[OFF_PAYLOAD + 4]);
	*sd_mm = get_u16(&buf[OFF_PAYLOAD + 8]);
	*n_ok = buf[OFF_PAYLOAD + 10];
	return 0;
}

/* ---- SETPOS ---- */
int apos_frame_setpos_build(uint8_t *buf, size_t buf_len, uint16_t src,
			    uint16_t dest, uint16_t session,
			    float x, float y, float z)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SETPOS, dest, src,
			   APOS_SUB_SETPOS);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	return (int)APOS_LEN_SETPOS;
}

int apos_frame_parse_setpos(const uint8_t *buf, size_t len, uint16_t *session,
			    float *x, float *y, float *z)
{
	int rc = parse_check(buf, len, APOS_SUB_SETPOS, APOS_LEN_SETPOS);

	if (rc || !session || !x || !y || !z) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	get_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	return 0;
}

/* ---- SETPOS_ACK ---- */
int apos_frame_setpos_ack_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t dest, uint16_t session,
				float x, float y, float z, bool ok)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SETPOS_ACK, dest, src,
			   APOS_SUB_SETPOS_ACK);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	buf[OFF_PAYLOAD + 14] = ok ? 1u : 0u;
	return (int)APOS_LEN_SETPOS_ACK;
}

int apos_frame_parse_setpos_ack(const uint8_t *buf, size_t len,
				uint16_t *session, float *x, float *y, float *z,
				bool *ok)
{
	int rc = parse_check(buf, len, APOS_SUB_SETPOS_ACK,
			     APOS_LEN_SETPOS_ACK);

	if (rc || !session || !x || !y || !z || !ok) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	get_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	*ok = buf[OFF_PAYLOAD + 14] != 0u;
	return 0;
}

/* ---- SURVEY_END ---- */
int apos_frame_survey_end_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t session)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SURVEY_END, APOS_ADDR_BCAST,
			   src, APOS_SUB_SURVEY_END);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	return (int)APOS_LEN_SURVEY_END;
}

int apos_frame_parse_survey_end(const uint8_t *buf, size_t len,
				uint16_t *session)
{
	int rc = parse_check(buf, len, APOS_SUB_SURVEY_END,
			     APOS_LEN_SURVEY_END);

	if (rc || !session) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	return 0;
}

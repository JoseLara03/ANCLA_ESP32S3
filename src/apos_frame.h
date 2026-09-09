/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Codec for the auto-positioning frames: one message type (0xEB) carrying a
 * subtype byte, rather than seven entries in the 0xE_ type space.
 *
 * Deliberately NOT added to uwb_frame_802_15_4z.c. That file is copied
 * byte-for-byte from the tag and any divergence is a wire-format bug waiting to
 * happen; the survey is anchor-and-gateway-only traffic the tag never parses, so
 * it has no business in the shared codec.
 *
 * The header is the ADDRESSED 10-byte form the module frames use --
 * 0x41 0x88 seq 0xCA 0xDE dest_lo dest_hi src_lo src_hi type -- not the
 * WAVE/VEWA literal-ident form, because the survey needs real src/dest
 * addressing. Byte 10 is the subtype; payload starts at 11.
 *
 * Pure C -- no Zephyr -- so every round-trip is host-testable.
 */

#ifndef APOS_FRAME_H
#define APOS_FRAME_H

#include "apos_table.h"   /* APOS_EUI_LEN */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Continues the 0xE_ series after POS (0xEA). */
#define APOS_FRAME_TYPE 0xEBu

#define APOS_SUB_SURVEY_BEGIN 0x01u
#define APOS_SUB_ENUM_RSP     0x02u
#define APOS_SUB_RANGE_CMD    0x03u
#define APOS_SUB_RANGE_RSP    0x04u
#define APOS_SUB_SETPOS       0x05u
#define APOS_SUB_SETPOS_ACK   0x06u
#define APOS_SUB_SURVEY_END   0x07u

/* 10 common header bytes plus the subtype. */
#define APOS_HDR_LEN 11u

/* Payload lengths, excluding the FCS -- callers add FCS_LEN in
 * dwt_writetxfctrl() exactly as the module frames do. */
#define APOS_LEN_SURVEY_BEGIN (APOS_HDR_LEN + 4u)  /* session, window_s     */
#define APOS_LEN_ENUM_RSP     (APOS_HDR_LEN + 27u) /* session, eui, pv, xyz,
						    * heard_ids            */
#define APOS_LEN_RANGE_CMD    (APOS_HDR_LEN + 5u)  /* session, peer, n_exch */
#define APOS_LEN_RANGE_RSP    (APOS_HDR_LEN + 11u) /* session, peer, mean,
						    * sd, n_ok             */
#define APOS_LEN_SETPOS       (APOS_HDR_LEN + 14u) /* session, xyz          */
#define APOS_LEN_SETPOS_ACK   (APOS_HDR_LEN + 15u) /* session, xyz, ok      */
#define APOS_LEN_SURVEY_END   (APOS_HDR_LEN + 2u)  /* session               */

/* Largest of the above, for RX buffer sizing. 38 bytes + FCS, comfortably
 * inside uwb_slave.c's RX_BUF_LEN (64).
 *
 * ENUM_RSP grew by 4 bytes when the survey scaled to 32 anchors, for the
 * neighbour bitmap below. That is a WIRE CHANGE and the parsers match lengths
 * exactly, so a gateway on this firmware and an anchor on an older one will not
 * enumerate each other at all -- the anchor's reply is refused with a
 * "parse failed" LOG_WRN naming the expected length. Survey traffic is
 * commissioning-only and the whole array is reflashed together, so this is a
 * flag-day change by design rather than an accident; it costs nothing to the
 * tag, which never parses 0xEB.
 *
 * That last clause was briefly FALSE: protocol v3 gave the tag an ALERT frame
 * on 0xEB, colliding exactly with this type -- same code, same 34-byte length
 * as APOS_LEN_ENUM_RSP, same discriminating byte at offset 10 (apos's subtype
 * against ALERT's `state`) -- and uwb_gateway.c's dispatch() tests apos FIRST,
 * so a tag ALERT would have been swallowed by the survey handler. ALERT moved
 * to 0xEE; apos keeps 0xEB because it already has seven subtypes, its own codec
 * and its own host tests. The clause is true again, and must stay true. */
#define APOS_LEN_MAX APOS_LEN_ENUM_RSP

/* Broadcast destination, matching UWB_FRAME_ADDR_BCAST. Redeclared rather than
 * included so this module stays free of the shared codec's header. */
#define APOS_ADDR_BCAST 0xFFFFu

/* True if this is a well-formed APOS frame of at least APOS_HDR_LEN bytes with
 * a known subtype. Every parser below re-checks its own length, so a caller may
 * dispatch on apos_frame_subtype() straight after this returns true. */
bool    apos_frame_is_apos(const uint8_t *buf, size_t len);
uint8_t apos_frame_subtype(const uint8_t *buf);
uint16_t apos_frame_src(const uint8_t *buf);
uint16_t apos_frame_dest(const uint8_t *buf);
void    apos_frame_set_seq(uint8_t *buf, uint8_t seq);

/* Builders return the payload length written (excluding FCS), or a negative
 * errno: -EINVAL on a NULL pointer, -EMSGSIZE when buf_len is too small. */
int apos_frame_survey_begin_build(uint8_t *buf, size_t buf_len, uint16_t src,
				  uint16_t session, uint16_t window_s);
/* heard_ids is a bitmap of ANCHOR IDS (bit k = short address
 * UWB_ANCHOR_ADDR_BASE + k) this board received an ENUM_RSP from during the
 * current survey -- the adjacency evidence the gateway's candidate-pair filter
 * runs on (apos_table_is_candidate()). Zero is legal and means "heard nobody",
 * which the gateway reads as no evidence rather than as isolation. */
int apos_frame_enum_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			      uint16_t dest, uint16_t session,
			      const uint8_t eui[APOS_EUI_LEN], bool pos_valid,
			      float x, float y, float z, uint32_t heard_ids);
int apos_frame_range_cmd_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, uint8_t n_exchanges);
int apos_frame_range_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, int32_t mean_mm,
			       uint16_t sd_mm, uint8_t n_ok);
int apos_frame_setpos_build(uint8_t *buf, size_t buf_len, uint16_t src,
			    uint16_t dest, uint16_t session,
			    float x, float y, float z);
int apos_frame_setpos_ack_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t dest, uint16_t session,
				float x, float y, float z, bool ok);
int apos_frame_survey_end_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t session);

/* Parsers return 0, or -EINVAL on a NULL pointer, a length that does not match
 * the subtype exactly, or a subtype mismatch. Out parameters are untouched on
 * failure, so a caller cannot act on half-parsed values. */
int apos_frame_parse_survey_begin(const uint8_t *buf, size_t len,
				  uint16_t *session, uint16_t *window_s);
int apos_frame_parse_enum_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			      uint8_t eui_out[APOS_EUI_LEN], bool *pos_valid,
			      float *x, float *y, float *z,
			      uint32_t *heard_ids);
int apos_frame_parse_range_cmd(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, uint8_t *n_exchanges);
int apos_frame_parse_range_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, int32_t *mean_mm,
			       uint16_t *sd_mm, uint8_t *n_ok);
int apos_frame_parse_setpos(const uint8_t *buf, size_t len, uint16_t *session,
			    float *x, float *y, float *z);
int apos_frame_parse_setpos_ack(const uint8_t *buf, size_t len,
				uint16_t *session, float *x, float *y, float *z,
				bool *ok);
int apos_frame_parse_survey_end(const uint8_t *buf, size_t len,
				uint16_t *session);

#endif /* APOS_FRAME_H */

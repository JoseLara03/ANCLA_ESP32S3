/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Clock-calibration packet (CCP): the frame a master anchor broadcasts so its
 * neighbours can put their DW3220 timestamps on a common time base. Feeds
 * src/sync_model.{c,h}, which is where the arithmetic and the error budget live.
 *
 * Deliberately NOT added to uwb_frame_802_15_4z.{c,h}. That file must stay
 * byte-identical with the tag's copy and the tag has no use for CCPs, so this
 * follows the precedent apos_frame.{c,h} already set for survey traffic. Pure C,
 * host-tested in tests/ccp_frame/.
 *
 * ---- Function code: 0xEF is the LAST one free ---------------------------
 *
 * Allocation across both repositories, which is the only view that matters --
 * see CLAUDE.md on how 0xEB came to be handed out twice:
 *
 *   0xE0..0xE4  legacy SS-TWR, DISCOVERY, MULTI-POLL, RANGE-RESPONSE
 *   0xE5..0xEA  BEACON, JOIN, GRANT, KEEPALIVE, RELEASE, POS
 *   0xEB        APOS_FRAME_TYPE (survey, seven subtypes)
 *   0xEC, 0xED  reserved: CONFIG_SET / CONFIG_ACK (design spec section 4.4)
 *   0xEE        ALERT
 *   0xEF        CCP  <- this, and the 0xEx range is now FULL
 *   0xF0        BLINK_FRAME_TYPE (src/blink_frame.h) -- first code past 0xEx
 *
 * Anything after this needs a different range or a subtype byte under an
 * existing code. Prefer a subtype: apos already demonstrates that seven
 * messages fit comfortably behind one code, and the 802.15.4 function-code
 * space this project uses is a single byte shared with the tag.
 *
 * ---- Why the payload carries a SCHEDULED transmit time -----------------
 *
 * The receiver needs to pair its own RX timestamp with the master's TX
 * timestamp of the same frame, and a transmitter cannot know its own TX
 * timestamp before transmitting. Two ways out: carry the PREVIOUS frame's
 * actual timestamp, or schedule the transmission and carry the scheduled time.
 *
 * This takes the second. The whole SS-TWR responder in anchor_respond.c already
 * depends on `poll_rx_ts + delay` being the actual on-air RMARKER time, so the
 * assumption is one this firmware relies on everywhere and has bench-confirmed.
 * It costs no extra frame of latency and no cross-frame bookkeeping.
 *
 * It does carry one obligation: CLAUDE.md records that dwt_starttx() can report
 * DWT_SUCCESS for a delayed transmission that never actually happens, which is
 * why every TX site here waits for TXFRS. A CCP whose TXFRS never arrives was
 * never on air, so its scheduled time describes nothing -- the sender MUST NOT
 * count it, and the receiver will simply see a gap in ccp_seq. Feeding a
 * phantom CCP into sync_model would inject a fabricated observation into the
 * one estimator the whole TDoA migration depends on.
 */

#ifndef CCP_FRAME_H
#define CCP_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define CCP_FRAME_TYPE       0xEFu

/* Layout. Offsets are absolute in the frame; lengths exclude the 2-byte FCS,
 * matching the UWB_FRAME_LEN_* convention this project uses everywhere.
 *
 *   0..9    802.15.4 header: FC 0x41 0x88, seq, PANID, dest 0xFFFF, src, type
 *   10      ccp_seq   monotonic, wraps at 256; a gap means a missed CCP
 *   11      hop       0 for the root master, N for N hops from it
 *   12..16  tx_dtu    40-bit scheduled RMARKER time of THIS frame
 *   17..20  root_id   identifies whose time base this is
 */
#define CCP_OFF_SEQ          10u
#define CCP_OFF_HOP          11u
#define CCP_OFF_TX_DTU       12u
#define CCP_OFF_ROOT_ID      17u
#define CCP_FRAME_LEN        21u

/* ---- v2 extension (B14 Fase B, docs/superpowers/plans/2026-09-22-b14-flight-compensation.md
 * FR-7/D7): the master's periodic SS-TWR flight-time measurement to ONE
 * anchor, piggybacked on the next few ordinary CCPs rather than costing a
 * frame of its own. Appended after the v1 layout, not a new function code --
 * `ccp_frame_is_ccp()`'s `len >=` check and `ccp_frame_parse()` never
 * reading past CCP_OFF_ROOT_ID+4 mean a v1 receiver already ignores these
 * bytes with NO code change on its side; that is what makes this
 * "versioned" rather than "a new frame type" -- a v1 anchor on an otherwise
 * upgraded fleet keeps working, it just never receives a TWR-sourced tau.
 *
 *   21      version        0/absent = v1 (21-byte frame); 2 = this extension
 *   22      tier           reserved, always 0 -- see D8's multi-hop hook
 *   23..26  tau_meas_dtu   signed, this anchor's flight time from the
 *                          master, same sign/units as sync_model_set_tau()
 *   27..28  tau_meas_anchor  which anchor this measurement is FOR (short
 *                          addr, e.g. 0x0001..0x0004); 0x0000 (the
 *                          gateway's own reserved address, never a valid
 *                          anchor) is the sentinel for "no measurement
 *                          completed yet" -- same "reserved value as
 *                          absence" convention as CCP's own tx_dtu==0
 *   29      twr_quality    0..255, opaque to the wire format; the filter
 *                          consuming it (FR-8) defines the scale
 */
#define CCP_OFF_VERSION           21u
#define CCP_OFF_TIER              22u
#define CCP_OFF_TAU_MEAS_DTU      23u
#define CCP_OFF_TAU_MEAS_ANCHOR   27u
#define CCP_OFF_TWR_QUALITY       29u
#define CCP_FRAME_LEN_V2          30u

#define CCP_VERSION_V1            0u
#define CCP_VERSION_V2            2u

/* Reserved gateway address (UWB_ADDR_GATEWAY_RESERVED in uwb_config.h, not
 * included here to keep this file dependency-free) repeated as a literal:
 * no anchor can ever hold it, so it is the sentinel for "this CCP carries
 * no TWR measurement yet" -- a cold-booted master, or one that has not
 * completed a first round-robin pass. */
#define CCP_TAU_MEAS_ANCHOR_NONE  0x0000u

/* Hop 0 is the root. A slave adopting a master at hop N publishes N+1, so the
 * tree depth is visible on air without any topology protocol. */
#define CCP_HOP_ROOT         0u

/* Refuse to adopt a master deeper than this. Every hop adds the previous hop's
 * residual error, so depth is the one topology parameter that directly degrades
 * accuracy -- see docs/anchor-sync-measurement.md. Two levels (root, then
 * masters at hop 1, then leaves at hop 2) covers ~100 anchors on one site with
 * WiFi coordinating which anchors take which role out of band, which is what
 * the backhaul decision bought. */
#define CCP_HOP_MAX          2u

/* Marks a hop field that must never be adopted, for a master whose own sync is
 * not valid. Distinct from CCP_HOP_MAX + 1 so the reason is legible on a
 * sniffer rather than looking like an ordinary too-deep frame. */
#define CCP_HOP_UNSYNCED     0xFFu

struct ccp_frame {
    uint8_t  seq;
    uint8_t  hop;
    uint64_t tx_dtu;      /* 40-bit; the top 24 bits of this field are zero */
    uint32_t root_id;
    uint16_t src_addr;    /* filled by the parser from the header */

    /* v2 fields, all zero-value-safe: a caller that never sets them (an
     * ordinary `struct ccp_frame f = {0};`) builds an unchanged v1 frame,
     * because `version` defaults to CCP_VERSION_V1. Parsing a v1 frame
     * leaves these at their CALLER-supplied `out` values untouched except
     * `version`, which parse() always sets -- check `version` before
     * reading any of the rest. */
    uint8_t  version;         /* CCP_VERSION_V1 or CCP_VERSION_V2 */
    uint8_t  tier;            /* reserved, always 0 for now */
    int32_t  tau_meas_dtu;
    uint16_t tau_meas_anchor; /* CCP_TAU_MEAS_ANCHOR_NONE if not present */
    uint8_t  twr_quality;
};

/* Build. Returns bytes written (excl. FCS) or a negative errno. `tx_dtu` is
 * masked to 40 bits; a value with high bits set is a caller bug and is
 * rejected rather than silently truncated, because a truncated timestamp would
 * produce a plausible-looking but wrong sync observation.
 *
 * `f->version` selects the wire length: CCP_VERSION_V1 (the default on a
 * zero-initialized struct) writes the original CCP_FRAME_LEN bytes and
 * touches none of the v2 fields; CCP_VERSION_V2 writes CCP_FRAME_LEN_V2 and
 * requires `buf_len` to cover it. Any other value is rejected -- a version
 * this build doesn't know how to write is a caller bug, not something to
 * silently downgrade. */
int  ccp_frame_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                     uint8_t frame_seq, const struct ccp_frame *f);

/* Cheap type/length test, for a receive dispatcher. */
bool ccp_frame_is_ccp(const uint8_t *buf, size_t len);

/* Parse. Returns 0 or a negative errno. Rejects a hop this node must not adopt,
 * so the caller cannot forget to check.
 *
 * Always sets `out->version`: CCP_VERSION_V2 when `len` covers
 * CCP_FRAME_LEN_V2 and `buf[CCP_OFF_VERSION] == CCP_VERSION_V2`, else
 * CCP_VERSION_V1 -- ANY other on-air value at that offset (a future v3, a
 * corrupt frame, or simply a shorter v1 frame with nothing there) parses as
 * v1, deliberately: the v1 fields it already validated are still correct
 * and safe to use, and refusing the whole frame over an extension this
 * build cannot read would be strictly worse than ignoring the extension.
 * The v2 fields are populated only when `out->version == CCP_VERSION_V2`;
 * a caller must check that before reading `tau_meas_dtu` and friends. */
int  ccp_frame_parse(const uint8_t *buf, size_t len, struct ccp_frame *out);

/* True if `hop` may be adopted as a time source by a node at this depth. */
bool ccp_hop_adoptable(uint8_t hop);

#endif /* CCP_FRAME_H */

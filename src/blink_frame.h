/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLINK: the frame a tag emits under TDoA instead of running a bidirectional
 * ranging sweep. Every anchor that hears it timestamps it and hands the
 * observation to the gateway, which solves the fix -- the tag does no
 * arithmetic at all. Feeds Phase 3 of the TDoA migration; see
 * docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md.
 *
 * Deliberately NOT added to uwb_frame_802_15_4z.{c,h}. That file must stay
 * byte-identical with the tag's copy, and a BLINK is exactly the kind of
 * frame that belongs only on one side of that boundary at build time but
 * must still match byte-for-byte at the wire -- same precedent ccp_frame.h
 * and apos_frame.h already set. The tag carries a byte-identical COPY of
 * this file; this repo is the source of truth for it, same rule as
 * cal_math.c, just in the other direction.
 *
 * ---- Function code: 0xF0, the first code outside the 0xEx range --------
 *
 * The 0xEx range is full as of ccp_frame.h's 0xEF (see the allocation table
 * there, and CLAUDE.md on how 0xEB came to be handed out twice across the
 * two repos). 0xF0 is free in both: this project defines nothing there, and
 * the tag's uwb_frame_802_15_4z.h tops out at 0xEE (ALERT). Checked directly
 * against both repos before use, not assumed.
 */

#ifndef BLINK_FRAME_H
#define BLINK_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define BLINK_FRAME_TYPE  0xF0u

/* Layout. Offsets are absolute in the frame; lengths exclude the 2-byte FCS,
 * matching the UWB_FRAME_LEN_* convention this project uses everywhere.
 *
 *   0..9    802.15.4 header: FC 0x41 0x88, seq, PANID, dest 0xFFFF, src, type
 *   10      blink_seq   monotonic, wraps at 256; the grouping key
 *   11      batt_soc    0..100, or UWB_FRAME_POS_SOC_CONNECTED (0xFF)
 *   12      flags       bit0 = ALERT pending, bit1 = MOVING; 2..7 reserved
 *   13      reserved    must be 0; exists so the length is even
 */
#define BLINK_OFF_SEQ     10u
#define BLINK_OFF_SOC     11u
#define BLINK_OFF_FLAGS   12u
#define BLINK_OFF_RSVD    13u
#define BLINK_FRAME_LEN   14u

#define BLINK_FLAG_ALERT          0x01u

/*
 * BLINK_FLAG_MOVING (proto 5): the tag's accelerometer says it is in motion.
 *
 * The gateway needs this because a stationary tag is the common case and the
 * single largest visual improvement available to the position filter is a
 * zero-velocity update -- pos_ekf_zupt(), whose own header says exactly that.
 * Without a real motion signal the gateway can only infer "still" from the
 * filter's OWN velocity estimate, which is a closed loop on its own output and
 * can stick on "moving" under high noise.
 *
 * The tag already has this bit and already uses it: motion.c sets
 * motion_moving from the LIS2HH12 activity interrupt, and the tag's own TWR
 * path already drives pos_ekf_predict()/pos_ekf_zupt() from it. All this flag
 * does is put it on the air so the TDoA path can use it too.
 *
 * WHAT IT DOES NOT MEAN: "not moving" is the accelerometer's opinion, not
 * ground truth. A tag carried slowly and smoothly can report still, which
 * would feed the filter a zero-velocity update it does not deserve. The
 * defence against that is not in this frame -- it is pos_ekf's gate streak and
 * pos_ekf_needs_reseed(), which the gateway already honours.
 *
 * Adding this bit is why UWB_PROTO_VER moves to 5: blink_frame_parse()
 * rejects any reserved bit, so a proto-4 receiver drops a BLINK carrying it
 * rather than misreading it. That rejection is the whole point of the
 * reserved-bit check, and it is why the mask has to shrink in the same commit
 * on both sides.
 */
#define BLINK_FLAG_MOVING         0x02u

#define BLINK_FLAG_RESERVED_MASK  0xFCu

struct blink_frame {
	uint16_t src_addr;
	uint8_t  seq;
	uint8_t  batt_soc;
	uint8_t  flags;
};

/* Build. Returns bytes written (excl. FCS) or a negative errno. */
int  blink_frame_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
		       uint8_t frame_seq, const struct blink_frame *f);

/* Cheap type/length test, for a receive dispatcher. */
bool blink_frame_is_blink(const uint8_t *buf, size_t len);

/* Parse. Returns 0 or a negative errno. Rejects any reserved bit set, so a
 * receiver on older firmware cannot silently misread a future flag. */
int  blink_frame_parse(const uint8_t *buf, size_t len, struct blink_frame *out);

#endif /* BLINK_FRAME_H */

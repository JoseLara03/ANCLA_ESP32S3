/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The two ranging responders. Each is offered every received frame and ignores
 * what is not addressed to it.
 *
 * IMPORTANT: len is the PAYLOAD length. The radio reports payload + FCS, and
 * the caller must subtract FCS_LEN before calling either function -- the frame
 * module derives field counts from the length and an extra 2 bytes silently
 * corrupts the result.
 */

#ifndef ANCHOR_RESPOND_H
#define ANCHOR_RESPOND_H

#include <stdbool.h>
#include <stdint.h>

#include "beacon_guard.h"
#include "uwb_config.h"

/* If buf is a legacy WAVE/0xE0 poll addressed to this anchor, schedule the
 * delayed VEWA/0xE1 response carrying our id, both timestamps and (x, y).
 * Otherwise no-op.
 *
 * bg may be NULL, which disables suppression. When non-NULL, a response whose
 * scheduled TX would land inside the beacon window is dropped rather than
 * transmitted -- one lost range instead of a corrupted broadcast.
 *
 * allow_unpositioned relaxes the position_valid requirement. An anchor with no
 * position MUST NOT answer a tag: the response encodes (0, 0) and the tag
 * cannot tell that apart from a real coordinate, so three unpositioned anchors
 * produce a confident meaningless fix with no error reported anywhere.
 *
 * But during an anchor survey every anchor is unpositioned and must still
 * answer its PEERS, or the survey can never bootstrap a cold deployment. The
 * caller resolves that: uwb_slave.c passes apos_node_window_open(), so the
 * relaxation lasts exactly as long as the gateway-opened survey window;
 * cal_run.c passes true, because calibrating an unpositioned board is normal
 * and there is no gateway or tag on air during calibration. */
void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len,
			      uint64_t poll_rx_ts, const uwb_config_t *cfg,
			      uint8_t *seq, struct beacon_guard *bg,
			      bool allow_unpositioned);

/* If buf is a DISCOVERY/0xE2 broadcast, schedule an id-staggered
 * RANGE-RESPONSE/0xE4 carrying our short address and CIR metrics. Otherwise
 * no-op.
 *
 * cir_power / cir_quality are captured by the caller from the RX callback --
 * see uwb_slave.c. They are 0 when CIA had not finished for that frame.
 *
 * bg may be NULL, which disables suppression. When non-NULL, a response whose
 * scheduled TX would land inside the beacon window is dropped rather than
 * transmitted -- one lost range instead of a corrupted broadcast. */
void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality,
			      struct beacon_guard *bg);

/* Called once for every successfully parsed beacon (frame_counter is the
 * counter that beacon carried). Checks internally whether it is this
 * anchor's turn in the ANNOUNCE rotation --
 * frame_counter % GW_ANNOUNCE_A == cfg->anchor_id (GW_ANNOUNCE_A in
 * gw_core.h) -- and if so, schedules a delayed ANNOUNCE/0xEC carrying our
 * short address, (x, y, z) and the CIR quality of that beacon, timed to land
 * inside the tag's T_ANNOUNCE_MS listen window (tag_testting's
 * uwb_net_runner.c). Otherwise no-op -- most anchors on most superframes.
 *
 * beacon_rx_ts is the RMARKER timestamp of the beacon just parsed, same units
 * as every other responder's *_rx_ts here.
 *
 * Gated on cfg->position_valid exactly like anchor_respond_wave_poll() with
 * allow_unpositioned == false -- unlike the survey path, there is no window
 * during which an unpositioned board should advertise coordinates it does
 * not have; an unsurveyed anchor simply does not announce.
 *
 * bg may be NULL, which disables suppression, same convention as the other
 * two responders. */
void anchor_respond_announce(uint32_t frame_counter, const uwb_config_t *cfg,
			     uint8_t *seq, uint64_t beacon_rx_ts,
			     int32_t cir_power, uint16_t cir_quality,
			     struct beacon_guard *bg);

/* If buf is a MULTI-POLL/0xE3 naming this anchor's short address among its
 * slots, schedule a delayed, addressed MPOL_RESP/0xED carrying our (x, y, z)
 * at the per-anchor delay the TAG assigned (slots[i].delay_us) -- not one we
 * compute -- so up to four anchors' responses land staggered inside the
 * tag's single multi-poll RX window without colliding. Otherwise no-op: the
 * poll did not name us.
 *
 * poll_rx_ts is this anchor's own RX timestamp of the multipoll, same
 * convention as every other responder's *_rx_ts here -- NOT a value read out
 * of the frame (the frame's own tx_ts field is the tag's bookkeeping, unused
 * on this path).
 *
 * bg may be NULL, which disables suppression, same convention as the other
 * responders. Gated on cfg->position_valid exactly like
 * anchor_respond_wave_poll() -- no allow_unpositioned relaxation, same
 * reasoning as anchor_respond_announce(). */
void anchor_respond_multipoll(const uint8_t *buf, uint16_t len,
			      uint64_t poll_rx_ts, const uwb_config_t *cfg,
			      uint8_t *seq, struct beacon_guard *bg);

#endif /* ANCHOR_RESPOND_H */

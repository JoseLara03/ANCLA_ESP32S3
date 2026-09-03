/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One SS-TWR exchange with this board as the initiator -- the role the tag
 * normally plays. Shared by the calibration image (cal_run.c) and the
 * production image (apos_node.c): it is the exact exchange the antenna delays
 * were calibrated through, so a second copy would be a second thing to
 * calibrate.
 *
 * Compiled into BOTH images, but a production anchor must never poll
 * unsolicited. That safety property is enforced by the CALLER, not here:
 * apos_node.c enters this only while a gateway-opened survey window is live AND
 * a RANGE_CMD for the current session has arrived from 0x0000. See
 * docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md 6.3.
 *
 * Interrupts are disabled for the duration (ss_initiator_enter/_leave) and the
 * exchange polls SYS_STATUS instead. That inverts the production responder's
 * rule, and deliberately: CLAUDE.md's warning against polling TXFRS/CIADONE
 * applies because dwt_isr() clears those bits before the callback runs. With no
 * ISR enabled there is nothing to clear them first, which is also what Qorvo's
 * own ss_twr_initiator example does. Every poll here is bounded.
 */

#ifndef SS_INITIATOR_H
#define SS_INITIATOR_H

#include <stdint.h>

/* Put the radio into initiator mode. Must be paired with ss_initiator_leave();
 * between the two, this board does not answer anything. */
void ss_initiator_enter(void);

/* Restore the RX state the responder loop expects. */
void ss_initiator_leave(void);

/* One poll/response exchange with the peer whose wire id is peer_wire_id.
 * Returns the distance in mm (clock-offset corrected), or INT32_MIN if the
 * response never arrived, was malformed, or came from an unexpected layout. */
int32_t ss_initiator_range(uint8_t peer_wire_id);

/* As ss_initiator_range(), plus the received signal level of the response, in
 * the DW3000's own q8.8 dBm format (divide by 256 for dBm). Added for the
 * range test (docs/range-test.md): a success/fail count is a CLIFF, while
 * received level degrades smoothly, so it says how far from the sensitivity
 * floor a link is rather than only whether it is over it -- and it is the
 * measurement that can settle whether the anchor's post-rework TX power
 * matches its prediction, which nothing has ever checked.
 *
 * The number comes from the vendored driver's own dwt_calculate_rssi(), NOT
 * from a local formula. Reimplementing the datasheet expression by hand is a
 * trap this project already fell into once, on 2026-09-03: the DW3000's
 * signal power is 10*log10(C * 2^21 / N^2) + 6*D - A, and the DGC_DECISION
 * term `6*D` -- worth up to +42 dB, and read from a register, not derivable
 * from the diagnostics struct -- was omitted, so the first field readings
 * came back tens of dB low and looked like a hardware fault. deca_rsl.c
 * carries the real expression and is already compiled into both images.
 *
 * `*rssi_q8` is set to SS_INITIATOR_RSSI_INVALID when the CIA had not
 * finished by the time the diagnostics were read. That is a routine outcome
 * on this interrupt-free polled path, NOT a weak signal, and must never be
 * reported as a very low level. The pointer may be NULL.
 *
 * Reading diagnostics costs a short SPI burst against a ~5 ms exchange.
 * ss_initiator_range() is the unchanged wrapper, so apos_node.c pays none of
 * it. */
#define SS_INITIATOR_RSSI_INVALID  INT16_MIN
int32_t ss_initiator_range_ex(uint8_t peer_wire_id, int16_t *rssi_q8);

/* Per-batch outcome breakdown, valid between ss_initiator_enter() and the
 * next ss_initiator_enter() (which clears it). Distinguishes "no response at
 * all" -- the signature of a link at its range limit -- from "a response
 * arrived and was malformed", which is interference or a peer on the wrong
 * build. Any pointer may be NULL. */
void ss_initiator_diag(uint32_t *ok, uint32_t *tx_start_fail,
		       uint32_t *tx_done_timeout, uint32_t *rx_timeout_or_err,
		       uint32_t *frame_len_bad, uint32_t *header_mismatch,
		       uint32_t *layout_unknown);

#endif /* SS_INITIATOR_H */

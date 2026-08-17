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

#endif /* SS_INITIATOR_H */

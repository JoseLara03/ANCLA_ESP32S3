/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One SS-TWR exchange with this board as the initiator -- the role the tag
 * normally plays. Used only by the calibration image.
 *
 * Interrupts are disabled for the duration (cal_initiator_enter/_leave) and the
 * exchange polls SYS_STATUS instead. That inverts the production responder's
 * rule, and deliberately: CLAUDE.md's warning against polling TXFRS/CIADONE
 * applies because dwt_isr() clears those bits before the callback runs. With no
 * ISR enabled there is nothing to clear them first, which is also what Qorvo's
 * own ss_twr_initiator example does. Every poll here is bounded.
 */

#ifndef CAL_INITIATOR_H
#define CAL_INITIATOR_H

#include <stdint.h>

/* Put the radio into initiator mode. Must be paired with cal_initiator_leave();
 * between the two, this board does not answer anything. */
void cal_initiator_enter(void);

/* Restore the RX state the responder loop expects. */
void cal_initiator_leave(void);

/* One poll/response exchange with the peer whose wire id is peer_wire_id.
 * Returns the distance in mm (clock-offset corrected), or INT32_MIN if the
 * response never arrived, was malformed, or came from an unexpected layout. */
int32_t cal_initiator_range(uint8_t peer_wire_id);

#endif /* CAL_INITIATOR_H */

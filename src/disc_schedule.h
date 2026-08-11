/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Discovery responder turnaround floor and per-anchor stagger. This is what
 * keeps four anchors from answering one broadcast DISCOVERY on top of each
 * other, which is why it is its own host-testable module.
 */

#ifndef DISC_SCHEDULE_H
#define DISC_SCHEDULE_H

#include <stdint.h>

/* Minimum turnaround from DISCOVERY RX to the first anchor's response TX. */
#define DISC_BASE_UUS 2000u

/* Gap between consecutive anchors' responses.
 * TUNE ON HARDWARE: must exceed one 0xE4 reply's air time. 3500 (3.5 ms >
 * 1.3 ms frame airtime + SPI and settle margin) is an estimate carried over
 * from the nRF5 anchor, not a measurement on this board. */
#define DISC_SLOT_UUS 3500u

/* Delay in UWB microseconds from DISCOVERY RX to this anchor's response TX.
 * Takes the 0-based console id, NOT the short address -- keying it on the
 * address would make anchor 0 wait a full slot for no reason. */
uint32_t disc_resp_delay_uus(uint8_t anchor_id);

#endif /* DISC_SCHEDULE_H */

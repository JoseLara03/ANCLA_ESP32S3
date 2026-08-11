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

/* Minimum turnaround from DISCOVERY RX to the first anchor's response TX.
 * TUNE ON HARDWARE, confirmed on the bench at two points so far:
 *   - 2000 (carried over from the nRF5 anchor's bare-metal SPI timing):
 *     failed 100% of the time. ~1700 uus is consumed just getting from RX
 *     to dwt_starttx() on this ANCLA_ESP32S3/Zephyr port (the vendored
 *     driver's ioctl-style dispatch plus bit-banged CS cost more per call
 *     than the bare-metal path 2000 was sized for), leaving ~290 uus margin
 *     at the dwt_starttx() call site -- not enough.
 *   - 2400, after downgrading the per-frame LOG_INF calls in this path to
 *     LOG_DBG (compiled out): still failed 100% of the time. Logging was
 *     NOT the dominant overhead -- the ~1700 uus is elsewhere (the SPI
 *     reads themselves: dwt_readrxdata/readrxtimestamp/readdiagnostics).
 *     Running this close to the deadline also triggered a separate bug: an
 *     unbounded TXFRS wait in tx_delayed() (src/anchor_respond.c) hung the
 *     whole console when dwt_starttx()'s own deadline check raced a
 *     borderline-late TX -- now fixed with a bounded wait, but the near-miss
 *     margin itself is still not something to run in production.
 *   - 6000 confirmed working reliably (~4300 uus margin observed).
 * Back to 6000 pending a real reduction in the ~1700 uus SPI-side overhead
 * (not logging) if a tighter link budget is still required. */
#define DISC_BASE_UUS 6000u

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

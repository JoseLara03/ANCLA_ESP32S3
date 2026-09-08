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

#include <stdbool.h>
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
 *   - 2000 confirmed working after the SPI bus was switched to fast
 *     rate (26.67 MHz). The earlier 2000/2400 failures were the 2 MHz bus,
 *     not the driver's per-call overhead: read_cir()'s 216-byte transfer
 *     alone was 864 uus of pure clock time at 2 MHz. Bench-confirmed with an
 *     external sniffer capturing clean WAVE/DISCOVERY responses and no TX
 *     misses; RX-side profiling at the fast rate measured cir=133-156 uus,
 *     readdata=23-33 uus, readts=20 uus (down from ~944/~1700 uus total at
 *     2 MHz). Not iterated below 2000; the gate only requires holding under
 *     2500 uus. */
#define DISC_BASE_UUS 2000u

/* Gap between consecutive anchors' responses.
 * TUNE ON HARDWARE: must exceed one 0xE4 reply's air time. 3500 (3.5 ms >
 * 1.3 ms frame airtime + SPI and settle margin) is an estimate carried over
 * from the nRF5 anchor, not a measurement on this board. */
#define DISC_SLOT_UUS 3500u

/* Delay in UWB microseconds from DISCOVERY RX to this anchor's response TX.
 * Takes the 0-based console id, NOT the short address -- keying it on the
 * address would make anchor 0 wait a full slot for no reason.
 * Equivalent to disc_resp_delay_uus_grouped(anchor_id, 1) -- kept as its own
 * function rather than a wrapper so callers that never see grouping (host
 * tests, anything predating network-scaling-v3) do not need to know it
 * exists. */
uint32_t disc_resp_delay_uus(uint8_t anchor_id);

/* Highest rank this schedule can stagger inside one DISCOVERY collection
 * window: DISC_BASE_UUS + DISC_MAX_RANK * DISC_SLOT_UUS = 12500 uus, the
 * value TX_COMPLETE_TIMEOUT_MS (anchor_respond.c, 18 ms) is derived from.
 * A group sized so that an anchor's rank inside it exceeds this must not
 * transmit -- see disc_resp_delay_uus_grouped(). */
#define DISC_MAX_RANK 3u

/* True if this anchor should answer a DISCOVERY carrying (group, n_groups):
 * anchor_id mod n_groups == group. n_groups == 0 matches nothing (a
 * malformed frame the caller should already have rejected via
 * uwb_frame_parse_discovery()'s own -EINVAL, but this stays safe either
 * way). */
bool disc_group_match(uint8_t anchor_id, uint8_t group, uint8_t n_groups);

/* Delay in UWB microseconds from DISCOVERY RX to this anchor's response TX,
 * for a grouped round: rank = anchor_id / n_groups, delay = DISC_BASE_UUS +
 * rank * DISC_SLOT_UUS. Grouping is what keeps the collection window bounded
 * as the deployment grows past 4 anchors -- every group holds at most
 * ceil(UWB_MAX_ANCHORS / n_groups) anchors, and at n_groups = 8 for 32
 * anchors that is exactly 4, the same worst case the ungrouped schedule was
 * already sized for.
 *
 * Returns the delay via *delay_uus and true, or false if rank > DISC_MAX_RANK
 * -- an n_groups too small for the deployment (or a stale tag) would
 * otherwise schedule a response past TX_COMPLETE_TIMEOUT_MS, which is
 * silently indistinguishable from a lost frame on air. The caller must
 * refuse to transmit rather than send late. */
bool disc_resp_delay_uus_grouped(uint8_t anchor_id, uint8_t n_groups, uint32_t *delay_uus);

#endif /* DISC_SCHEDULE_H */

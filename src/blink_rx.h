/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The anchor side of TDoA: stamp a tag's BLINK and hand the observation to the
 * uplink.
 *
 * This module holds no arithmetic of its own, deliberately -- the same split
 * ccp_slave.c makes. Everything that could be wrong about the clock conversion
 * lives in sync_model.c (pure C, host-tested); what lives here is the part a
 * host test cannot reach: a real RX timestamp from a real DW3220, and the
 * decision to DISCARD when the local clock cannot be expressed in the master's
 * time base.
 *
 * ---- No transmission, ever ------------------------------------------------
 *
 * A BLINK is answered by nothing. This module reads, converts and enqueues; it
 * never calls dwt_starttx() and never delays the caller. That is what makes it
 * survivable on hardware whose PA supply cannot sustain a second frame per
 * superframe (see CLAUDE.md): Phase 3 adds no UWB transmission to any node.
 *
 * ---- Discarding is the correct outcome, not a failure ---------------------
 *
 * sync_model_to_master() returns false whenever the model has no rate estimate
 * yet or has coasted past SYNC_MISS_MAX. An observation without a common time
 * base is NOISE, not data: fed to tdoa_solve() it would move the reported
 * position by however far the two anchors' clocks happen to be apart, and the
 * solver has no way to tell that from a real path difference. So it is dropped
 * and counted (blink_rx_stats()'s n_no_sync), never published with a local
 * timestamp and never published with a guessed one.
 */

#ifndef BLINK_RX_H
#define BLINK_RX_H

#include "uwb_config.h"

#include <stdbool.h>
#include <stdint.h>

/* Latch this anchor's id and clear the counters. Call once, before the SLAVE
 * loop starts, with the same config the rest of the loop uses. */
void blink_rx_init(const uwb_config_t *cfg);

/* Offer one received frame. Returns true if it WAS a BLINK -- consumed either
 * way, so the caller can stop its dispatch chain -- and false if the frame is
 * for someone else.
 *
 * `rx_ts` is the full 40-bit DW3220 RX timestamp, exactly as
 * uwb_get_rx_timestamp_u64() returns it. Do NOT shift it to hi32: sync_model
 * works in whole DTU and the bottom 8 bits are 255 DTU (~1.2 m of path
 * difference at TDOA_M_PER_DTU) -- the same rule ccp_slave_on_rx() states, and
 * it matters more here, since this timestamp goes straight into a position.
 *
 * `quality` is the CIR quality the caller already read; carried through as
 * diagnostics only. The solver does not use it. */
bool blink_rx_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts,
		    uint16_t quality);

/* Counters. `n_rx` is BLINKs received, `n_no_sync` those dropped because
 * sync_model_to_master() refused, `n_bad` those that parsed as a BLINK type but
 * failed blink_frame_parse() (reserved bits set), and `n_sent` those handed to
 * the uplink. n_rx == n_no_sync with n_sent flat is the signature of a board
 * hearing tags fine while its CCP link is down -- check `sync stats` before
 * suspecting the radio. */
void blink_rx_stats(uint32_t *n_rx, uint32_t *n_no_sync, uint32_t *n_bad,
		    uint32_t *n_sent);

#endif /* BLINK_RX_H */

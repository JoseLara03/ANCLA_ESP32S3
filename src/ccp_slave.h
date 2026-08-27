/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The SLAVE side of anchor clock sync: receive CCPs, feed sync_model, and own
 * the one instance of it.
 *
 * This module holds no arithmetic of its own. Everything that could be wrong
 * about the estimator lives in sync_model.c, which is pure C and host-tested;
 * what lives here is the part a host test cannot reach -- a real RX timestamp
 * from a real DW3220 -- and the sequence bookkeeping that tells the model when
 * an expected CCP did not arrive.
 */

#ifndef CCP_SLAVE_H
#define CCP_SLAVE_H

#include "sync_model.h"

#include <stdbool.h>
#include <stdint.h>

/* Clear to the no-observations state. Call once before the SLAVE loop starts. */
void ccp_slave_init(void);

/* Offer one received frame. Returns true if it WAS a CCP -- consumed either
 * way, so the caller can stop its dispatch chain -- and false if the frame is
 * for someone else.
 *
 * `rx_ts` is the full 40-bit DW3220 RX timestamp, exactly as
 * uwb_get_rx_timestamp_u64() returns it. Do not shift it: sync_model works in
 * whole DTU, and handing it a hi32 value would throw away the bottom 8 bits --
 * 255 DTU, ~4 ns, against a 1 ns gate. */
bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts);

/* The live model, for the `sync` shell command. Non-const because
 * sync_model_residual_reset() takes a mutable pointer.
 *
 * Unsynchronised, and safe today for one reason: the SLAVE loop is the only
 * writer and the shell is strictly lower priority, so a reader never observes a
 * half-updated model -- it sees the previous state or the next one. Same
 * reasoning, and the same fragility, as apos_gw_result(). */
struct sync_model *ccp_slave_model(void);

/* Counters. `n_gap` is expected CCPs that never arrived, `n_reject` is frames
 * that were CCPs but were not usable -- a parse failure, a hop this node may
 * not adopt, or a duplicate sequence number. */
void ccp_slave_stats(uint32_t *n_rx, uint32_t *n_gap, uint32_t *n_reject,
		     uint32_t *root_id);

#endif /* CCP_SLAVE_H */

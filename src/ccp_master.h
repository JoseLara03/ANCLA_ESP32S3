/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's clock-calibration-packet transmitter: the root of the anchor
 * sync tree (hop 0).
 *
 * On the GATEWAY, deliberately. The gateway already schedules one delayed TX
 * per superframe and the clock it schedules from IS the network's time base, so
 * the CCP master and the beacon master being the same board is coherent rather
 * than convenient. A master role on a SLAVE was rejected: it would add an
 * UNSOLICITED transmit path to a production slave image, which is exactly the
 * collision hazard apos_node.c's two gates exist to prevent -- see CLAUDE.md on
 * why ss_initiator.c being in production is safe only because of them.
 *
 * To repeat the measurement with the roles swapped (docs/anchor-sync-
 * measurement.md section 3), swap the BOARDS: `anchor mode gateway` on the
 * other one and `kernel reboot cold`. Same technique CLAUDE.md already
 * prescribes for separating a firmware fault from a board fault.
 */

#ifndef CCP_MASTER_H
#define CCP_MASTER_H

#include <stdint.h>

/* Derives root_id from this board's EUI-64 and clears the counters. Call once
 * before the gateway loop starts. */
void ccp_master_init(void);

/* Transmit one CCP, scheduled CCP_OFFSET_UUS after the beacon's RMARKER.
 *
 * Call ONLY with a beacon_tx_dtu that tx_beacon() actually returned, i.e. only
 * after that beacon's TXFRS was confirmed. A CCP scheduled from a beacon that
 * never transmitted would be scheduled from a time that does not exist.
 *
 * `frame_seq` is the gateway's shared 802.15.4 sequence counter, consumed by
 * reference exactly as tx_beacon() and send_grant() consume it.
 *
 * Bounded: at most one delayed TX and one bounded TXFRS wait
 * (CCP_MASTER_TX_TIMEOUT_MS). Never blocks. Safe on the K_PRIO_COOP(0) loop. */
void ccp_master_after_beacon(uint64_t beacon_tx_dtu, uint8_t *frame_seq);

/* Diagnostics. `dropped` counts CCPs whose transmission was not CONFIRMED --
 * a failed dwt_starttx() or a TXFRS that never arrived. Each one leaves a gap
 * in ccp_seq on purpose, so a receiver sees the miss. */
void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root_id);

#endif /* CCP_MASTER_H */

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
 * Call ONLY with the beacon_hi32 that WAS PROGRAMMED for a beacon that
 * actually transmitted (uwb_gateway.c's `next_beacon`, still un-re-based at
 * the call site -- checked by reading the caller), i.e. only after that
 * beacon's TXFRS was confirmed. A CCP scheduled from a beacon that never
 * transmitted would be scheduled from a time that does not exist.
 *
 * This deliberately takes the PROGRAMMED hi32, not tx_beacon()'s measured TX
 * timestamp: reading that timestamp back costs a ~20 us SPI transaction
 * (mac_budget.h's `readts`) out of a ~98.9 us total arm budget
 * (CCP_SCHED_ARM_BUDGET_NS, ccp_sched.h) that hardware has already shown is
 * too tight -- 100% of CCPs were dropped on the bench. The programmed hi32
 * differs from the true RMARKER by exactly ant_delay_tx, a fixed antenna
 * delay this gate's receiver already treats as a constant bias to be
 * absorbed on the first observation (see the "does NOT carry ant_delay_tx"
 * comment in ccp_master.c) -- so using the programmed value costs this
 * measurement nothing and removes one SPI read from the critical path.
 *
 * `frame_seq` is the gateway's shared 802.15.4 sequence counter, consumed by
 * reference exactly as tx_beacon() and send_grant() consume it.
 *
 * Bounded: at most one delayed TX and one bounded TXFRS wait
 * (CCP_MASTER_TX_TIMEOUT_MS). Never blocks. Safe on the K_PRIO_COOP(0) loop. */
void ccp_master_after_beacon(uint32_t beacon_hi32, uint8_t *frame_seq);

/* Diagnostics. `dropped` counts CCPs whose transmission was not CONFIRMED --
 * a failed dwt_starttx() or a TXFRS that never arrived. Each one leaves a gap
 * in ccp_seq on purpose, so a receiver sees the miss.
 *
 * late_ns_{min,max,last} are read only on the dwt_starttx()-failure path,
 * where a register read costs nothing extra (the CCP is already dropped):
 * how far past the CCP's scheduled hi32 the radio clock already was when
 * dwt_starttx() reported failure, in nanoseconds. 0 means "never observed a
 * failure with a positive lateness" -- see ccp_master.c for what a
 * non-positive reading would mean and why it is reported separately rather
 * than folded into these three. */
void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root_id,
		      int32_t *late_ns_min, int32_t *late_ns_max,
		      int32_t *late_ns_last);

#endif /* CCP_MASTER_H */

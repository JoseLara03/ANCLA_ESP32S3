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
 *
 * ---- Deferred-timestamp design (Option B), replacing the delayed-TX one ---
 *
 * The original design scheduled each CCP with a DELAYED TX at a fixed offset
 * after the beacon's RMARKER, and dwt_starttx() failed for 100% of CCPs on the
 * bench -- the arm sequence could not reliably complete before the CCP's own
 * preamble needed to start (see src/ccp_sched.h's header comment for the full
 * diagnosis and the two race windows that made no offset value work).
 *
 * This module now transmits each CCP with DWT_START_TX_IMMEDIATE -- right
 * after the beacon, whatever moment that turns out to be -- and reads back
 * the frame's ACTUAL TX timestamp once TXFRS is confirmed. Since a
 * transmitter cannot know its own TX timestamp before transmitting, that
 * timestamp is carried in the FOLLOWING CCP instead of in the frame it
 * belongs to. A CCP with sequence S therefore announces the TX timestamp of
 * the CCP with sequence S-1 -- see ccp_slave.c for the receiver side of that
 * pairing. This is immune to arm jitter by construction: there is no deadline
 * to arm against, so there is nothing for jitter to make late.
 */

#ifndef CCP_MASTER_H
#define CCP_MASTER_H

#include <stdbool.h>
#include <stdint.h>

/* Derives root_id from this board's EUI-64 and clears the counters. Call once
 * before the gateway loop starts. */
void ccp_master_init(void);

/* Transmit one CCP with DWT_START_TX_IMMEDIATE, right after the beacon.
 *
 * Call ONLY once the caller's beacon TX for this superframe has been
 * CONFIRMED (a nonzero tx_beacon() return) -- an immediate TX transmitted
 * before the beacon would collide with it, and this function does not itself
 * check that the beacon went out.
 *
 * `beacon_hi32` is the hi32 the beacon was PROGRAMMED to (or, in the caller's
 * naming, `next_beacon` before it is re-based) -- this function no longer
 * needs it to SCHEDULE anything (there is nothing left to schedule), but it
 * is kept as a parameter because the new instrumentation below needs the
 * beacon's own RMARKER to compute where the CCP's ACTUAL RMARKER lands
 * relative to it, which is the number that answers "does this still clear
 * the CAP". Removing the parameter would have cost that measurement for a
 * saving of one argument.
 *
 * The frame this call builds carries `tx_dtu` = the PREVIOUS CCP's measured
 * TX timestamp (0 as a sentinel meaning "no announcement yet" -- see
 * ccp_master.c on why a genuine 40-bit timestamp landing on exactly 0 is a
 * 2^-40 event costing at most one skipped observation), and `seq` = this
 * frame's own sequence number, incremented at BUILD time regardless of
 * whether the transmit succeeds -- a built-but-untransmitted CCP must still
 * leave a visible gap for the receiver.
 *
 * `frame_seq` is the gateway's shared 802.15.4 sequence counter, consumed by
 * reference exactly as tx_beacon() and send_grant() consume it.
 *
 * Bounded: at most one immediate TX and one bounded TXFRS wait
 * (CCP_MASTER_TX_TIMEOUT_MS). Never blocks. Safe on the K_PRIO_COOP(0) loop. */
void ccp_master_after_beacon(uint32_t beacon_hi32, uint8_t *frame_seq);

/* Diagnostics. `dropped` counts CCPs whose transmission was not CONFIRMED --
 * a build failure, a failed dwt_starttx(), or a TXFRS that never arrived.
 * Each one leaves a gap in ccp_seq on purpose, so a receiver sees the miss --
 * AND, under the deferred-timestamp design, costs the announcement the NEXT
 * CCP would otherwise have carried: the previous-timestamp state is only
 * advanced on a CONFIRMED transmit (see ccp_master.c), so a drop never
 * causes a later CCP to announce a timestamp for a frame that never went on
 * air.
 *
 * offset_ns_{min,max,last}: where the CCP's ACTUAL RMARKER landed relative to
 * the BEACON's RMARKER, in nanoseconds, on a CONFIRMED transmit only. This
 * replaces the old delayed-TX "lateness against a schedule" instrument, which
 * is meaningless now that nothing is scheduled -- there is no deadline to be
 * late against, only an actual landing spot to report.
 *
 * arm_cost_ns_last: offset_ns_last minus the CCP's own SHR and minus the
 * beacon's own post-RMARKER airtime (ccp_sched.h's CCP_SCHED_SHR_NS and
 * CCP_SCHED_BEACON_END_NS) -- i.e. how long the arm sequence itself
 * (dwt_forcetrxoff() through dwt_starttx(), five SPI transactions minus the
 * one dwt_setdelayedtrxtime() call an immediate TX no longer needs) took
 * between the beacon's frame ending and this CCP's preamble starting. This is
 * the number to compare directly against ccp_sched.h's CCP_SCHED_MAX_ARM_NS
 * (348300 ns) -- the compile-time-checked budget for exactly this quantity. */
void ccp_master_stats(uint32_t *sent, uint32_t *dropped, uint32_t *root_id,
		      int32_t *offset_ns_min, int32_t *offset_ns_max,
		      int32_t *offset_ns_last, int32_t *arm_cost_ns_last);

/* D3: the "sync txtest" control experiment -- one CCP sent with
 * DWT_START_TX_IMMEDIATE, exactly like every ordinary CCP now, so this is no
 * longer a control against a DIFFERENT transmit mode -- it is a one-shot
 * probe an operator can fire on demand, at an arbitrary point in the
 * superframe, without waiting for the next beacon. Entirely separate state
 * and counters from the normal per-superframe path above -- this can never be
 * confused with, or perturb, ccp_master_stats()'s sent/dropped, and its
 * tx_dtu is always 0 (seq 0xFF marks it, so ccp_slave.c's sentinel handling
 * ignores it as "no announcement" -- see ccp_master.c).
 *
 * ccp_master_request_txtest() only sets a flag and is safe to call from ANY
 * thread, including the shell: it never touches the radio. The gateway loop
 * is K_PRIO_COOP(0) and owns the only radio access in gateway mode, so the
 * actual transmit happens in ccp_master_txtest_step(), which the loop must
 * call once per iteration (uwb_gateway.c does, right after the normal CCP
 * call) -- it is a no-op unless a test is pending, and otherwise performs at
 * most one immediate TX plus one bounded TXFRS wait, the same bound as
 * ccp_master_after_beacon(). Never blocks, never spins. */
void ccp_master_request_txtest(void);

/* True if a "sync txtest" request is queued but the gateway loop has not yet
 * consumed it. Shell-callable. */
bool ccp_master_txtest_pending(void);

/* Consumes a pending txtest request, if any: builds and transmits exactly one
 * CCP with DWT_START_TX_IMMEDIATE and records the outcome. A no-op (returns
 * immediately) when no test is pending. Call ONLY from the gateway loop --
 * this touches the radio. `frame_seq` is the gateway's shared 802.15.4
 * sequence counter, consumed by reference like every other TX site. */
void ccp_master_txtest_step(uint8_t *frame_seq);

/* Outcome of the most recent "sync txtest". `pending` is true if a request is
 * queued but not yet serviced; `done` is true once the gateway loop has acted
 * on one (cleared again by the next ccp_master_request_txtest()); `tx_ok` is
 * whether dwt_starttx(DWT_START_TX_IMMEDIATE) reported success; `txfrs_ok` is
 * whether TXFRS then actually arrived within the bound. */
void ccp_master_txtest_stats(bool *pending, bool *done, bool *tx_ok,
			     bool *txfrs_ok);

#endif /* CCP_MASTER_H */

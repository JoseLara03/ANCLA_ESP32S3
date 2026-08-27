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

#include <stdbool.h>
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

/* D1 + D2 diagnostics for telling ull_starttx()'s two failure branches apart
 * (HPDWARN vs. DW_SYS_STATE_TXERR) -- see ccp_master.c's statics for the full
 * reasoning. All four are read only on the dwt_starttx()-failure path, same
 * as the fields above.
 *
 * late_signed_last / late_signed_min: the SIGNED lateness in nanoseconds,
 * tracked unconditionally (unlike late_ns_min/max above, which only track
 * positive values). Expect roughly -1.05 ms if at_hi32 is sane; a value
 * approaching the ~8.6 s half-period instead means at_hi32 itself is wrong.
 *
 * sys_status_lo / hpdwarn_seen: the raw low system-status word (and whether
 * HPDWARN was set in it) read immediately after the failure, with HPDWARN
 * explicitly cleared right before this dwt_starttx() so a stale HPDWARN
 * cannot be mistaken for a fresh one. NOT proof either way: ull_starttx()
 * issues CMD_TXRXOFF on both of its failure branches before we ever read
 * this, and that command may itself clear HPDWARN -- so a clear reading is
 * consistent with TXERR, not conclusive of it. */
void ccp_master_diag_stats(int32_t *late_signed_last, int32_t *late_signed_min,
			   uint32_t *sys_status_lo, bool *hpdwarn_seen);

/* D3: the "sync txtest" control experiment -- one CCP sent with
 * DWT_START_TX_IMMEDIATE instead of DWT_START_TX_DELAYED, to separate "the
 * delayed-TX machinery is at fault" from "something more basic about
 * transmitting twice in a superframe is wrong". Entirely separate state and
 * counters from the normal per-superframe path above -- this can never be
 * confused with, or perturb, ccp_master_stats()'s sent/dropped.
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

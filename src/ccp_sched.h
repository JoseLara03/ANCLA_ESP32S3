/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Where the CCP sits inside the superframe.
 *
 * Header only, no .c file, same as uwb_mac.h: everything here is compile-time
 * arithmetic, and the BUILD_ASSERT at the bottom IS the deliverable.
 *
 * ---- Why the delayed-TX design this header used to police was abandoned ---
 *
 * The original design scheduled the CCP with a DELAYED TX, CCP_OFFSET_UUS
 * (BEACON_OCCUPANCY_UUS, 1500 UUS) after the beacon's RMARKER, and this header
 * carried two BUILD_ASSERTs proving that a frame armed exactly on schedule
 * would clear both the beacon on one side and the first legitimate slave CAP
 * transmit on the other. That held at compile time and failed 100% on
 * hardware: dwt_starttx() returned DWT_ERROR for every single CCP.
 *
 * Instrumentation (see git history on src/ccp_master.c, commits c4a515b through
 * 2efd93b) settled why. The arm sequence -- dwt_forcetrxoff(),
 * dwt_setdelayedtrxtime(), dwt_writetxdata(), dwt_writetxfctrl(),
 * dwt_starttx() -- completed 614654-678485 ns after the BEACON's RMARKER, i.e.
 * 225243-289074 ns after the beacon's own frame ended
 * (CCP_SCHED_BEACON_END_NS, 389411 ns). That is well inside the ~1.538 ms
 * CCP_OFFSET_UUS gave the arm to complete in -- comfortably early against the
 * CCP's own RMARKER. But a frame's RMARKER sits at the END of its own SHR
 * (CCP_SCHED_SHR_NS, 1050194 ns at PLEN_1024): the CCP's PREAMBLE had to start
 * at 1538461 - 1050194 = 488267 ns after the beacon's RMARKER, and the arm
 * routinely finished 190218 ns (worst case) AFTER that. So the arm was late
 * for the preamble it needed to schedule, by up to ~190 us, while being
 * measured as ~860 us EARLY against the RMARKER -- both true at once, because
 * they are distances to two different instants a full SHR apart. The old
 * instrument measured lateness against the RMARKER and reported "not late";
 * the reference point, not the measurement, was wrong. That is what hid this
 * for two bench cycles.
 *
 * hpdwarn_seen:0 and sys_status_lo:0x00000000 on every failure additionally
 * showed the failing branch was DW_SYS_STATE_TXERR in ull_starttx(), not the
 * HPDWARN deadline check -- TXERR being CAUSED by the late arm, not inherited
 * from the beacon (dwt_forcetrxoff() before the arm, already present, changed
 * nothing, because it cannot fix a preamble that starts too late).
 *
 * Raising CCP_OFFSET_UUS cannot fix this: solving both edges for the offset
 * leaves a legal window of 1685.5-1743.2 UUS, 57.7 UUS (57700 ns) wide --
 * narrower than the ~63831 ns of observed arm-completion jitter
 * (678485 - 614654) alone, before any margin. There is no delayed-TX offset
 * that reliably fits.
 *
 * The fix (docs/superpowers/plans/2026-08-26-fase2-ccp-sync.md, Option B):
 * transmit the CCP with DWT_START_TX_IMMEDIATE right after the beacon, read
 * back its ACTUAL TX timestamp once TXFRS is confirmed, and carry that
 * timestamp in the FOLLOWING CCP rather than announcing a scheduled time for
 * itself. This is immune to arm jitter by construction: there is no deadline
 * to miss, because the announced timestamp is measured after the fact, not
 * predicted before it. src/ccp_master.c and src/ccp_slave.c implement the
 * pairing; this header now checks the one thing that is still compile-time
 * checkable -- that an immediate CCP transmitted right after the beacon
 * physically FITS in the post-beacon guard window at all, with margin to
 * spare, before the first legitimate slave CAP preamble.
 */

#ifndef CCP_SCHED_H
#define CCP_SCHED_H

#include "ccp_frame.h"
#include "mac_budget.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/sys/util.h>

/* ---- Derived quantities, for the assert and for tests/ccp_sched ---- */

/* Preamble + SFD. A frame's RMARKER is at the end of this, so the SHR occupies
 * the air BEFORE the scheduled time. */
#define CCP_SCHED_SHR_NS                                                       \
	MAC_PS_TO_NS(MAC_SHR_PS(MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM))

/* Airtime AFTER the RMARKER: PHR + payload + FCS. `bytes` excludes the FCS,
 * matching the UWB_FRAME_LEN_* convention used everywhere in this project. */
#define CCP_SCHED_POST_RMARKER_NS(bytes)                                       \
	MAC_PS_TO_NS(MAC_BITS_PS(MAC_PHR_BITS, MAC_PHY_BITRATE) +               \
		     MAC_BITS_PS(((uint64_t)(bytes) + MAC_FCS_BYTES) * 8ULL,    \
				 MAC_PHY_BITRATE))

/* Where the beacon's own frame stops occupying the air, relative to its
 * RMARKER. UWB_FRAME_MAX_LEN is the beacon length: it is the longest frame the
 * contract defines, and tx_beacon() builds exactly that. This is also, now,
 * the earliest instant the CCP's own arm sequence can even begin -- it cannot
 * start before tx_beacon() has returned, which happens only once the beacon's
 * TXFRS is confirmed. */
#define CCP_SCHED_BEACON_END_NS                                                \
	CCP_SCHED_POST_RMARKER_NS(UWB_FRAME_MAX_LEN)

/* Where the slaves' suppression window closes, relative to the beacon's
 * RMARKER. */
#define CCP_SCHED_GUARD_END_NS                                                 \
	(MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS) + MAC_UUS_TO_NS(BEACON_GUARD_UUS))

/* Earliest PREAMBLE of the first legitimate slave CAP transmit, relative to
 * the beacon's RMARKER.
 *
 * beacon_guard_tx_allowed() (src/beacon_guard.c) gates a slave's SCHEDULED
 * RMARKER against [next_beacon - guard, next_beacon + occupancy + guard], so
 * the earliest a slave may schedule its RMARKER is CCP_SCHED_GUARD_END_NS. But
 * the collision that matters is airtime, and a frame's PREAMBLE precedes its
 * RMARKER by a whole SHR -- the same trap this header's own top-of-file
 * comment already warns about, applied here to the TRAILING edge instead of
 * the leading one. So the true earliest colliding airtime is one SHR earlier
 * than the guard's own RMARKER bound. */
#define CCP_SCHED_CAP_PREAMBLE_NS                                              \
	(CCP_SCHED_GUARD_END_NS - CCP_SCHED_SHR_NS)

/* The whole guard-window budget available to host one CCP frame, from the
 * instant the beacon's frame ends to the instant the first legitimate slave
 * CAP preamble may begin, MINUS the CCP's own full airtime (SHR through FCS).
 * What is left over is headroom for the immediate-TX arm sequence itself
 * (dwt_forcetrxoff() through dwt_starttx(), now with NO dwt_setdelayedtrxtime()
 * call at all -- an immediate TX needs no scheduled time) to run and for the
 * frame to clear the air before that preamble.
 *
 * This is the honest replacement for the old CCP_SCHED_ARM_BUDGET_NS: that
 * macro bounded a delayed TX's arm-before-a-deadline race, which no longer
 * exists. This bounds something real instead -- that the guard window is even
 * wide enough to fit the CCP's whole frame with the beacon's own airtime
 * subtracted, a genuine compile-time property of the PHY parameters and the
 * MAC contract's guard sizing, independent of how fast any one arm sequence
 * happens to run.
 *
 * Evaluates to 2026728 - 1289017 - 389411 = 348300 today. */
/* Computed in a SIGNED 64-bit type deliberately: every term above is a
 * uint32_t, and an unsigned subtraction that goes negative WRAPS to a huge
 * positive value rather than failing the assert below -- exactly the kind of
 * silent-wraparound trap CLAUDE.md already warns about for hi32 arithmetic,
 * just at compile time instead of on the wire. Widening to int64_t before
 * subtracting is what makes a genuine shortfall show up as a negative number
 * instead of vanishing. */
#define CCP_SCHED_MAX_ARM_NS                                                   \
	((int64_t)CCP_SCHED_CAP_PREAMBLE_NS -                                  \
	 (int64_t)(CCP_SCHED_SHR_NS + CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN)) - \
	 (int64_t)CCP_SCHED_BEACON_END_NS)

/* The guard window must be wide enough to host one whole CCP frame after the
 * beacon's own airtime, with something left over for the arm sequence to run
 * in. A non-positive value here would mean the CCP frame plus the beacon's
 * frame together do not even fit before the first legitimate slave CAP
 * preamble -- a genuine compile-time property, checkable without any
 * hardware measurement at all, unlike the arm-jitter question the old delayed
 * design could not settle this way. */
BUILD_ASSERT(CCP_SCHED_MAX_ARM_NS > 0,
	     "no room left in the post-beacon guard window for the CCP frame");

#endif /* CCP_SCHED_H */

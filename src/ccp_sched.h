/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Where the CCP sits inside the superframe.
 *
 * Header only, no .c file, same as uwb_mac.h: everything here is compile-time
 * arithmetic, and the BUILD_ASSERTs at the bottom ARE the deliverable. There is
 * no runtime behaviour to test, only a budget that must hold.
 *
 * The placement decision, and why it costs nothing:
 *
 * A slave suppresses its own TX inside [T_b - BEACON_GUARD_UUS,
 * T_b + BEACON_OCCUPANCY_UUS + BEACON_GUARD_UUS], where T_b is the beacon's
 * RMARKER -- that is the value beacon_guard_beacon() is fed, since it comes
 * from the beacon's RX timestamp. That window is already reserved and already
 * empty, so a CCP transmitted inside it takes NO airtime away from the CAP or
 * the CFP. The alternative -- a slot of its own -- would have cost 0.645% of
 * every superframe and forced the capacity model in the design spec section 3.2
 * to be re-run.
 *
 * The offset is BEACON_OCCUPANCY_UUS rather than a tuned number, and that is
 * the point: it reads as "immediately after the occupancy the beacon already
 * declares for itself". The two asserts below prove both edges hold.
 *
 * Everything is measured from the beacon's RMARKER, because that is what a
 * delayed TX is programmed against. Note the RMARKER sits at the END of its own
 * SHR, so a frame's preamble PRECEDES its scheduled time -- this is the same
 * trap that made an earlier SS-TWR span estimate double-count both SHRs; see
 * MAC_SSTWR_EXCHANGE_PS in mac_budget.h.
 */

#ifndef CCP_SCHED_H
#define CCP_SCHED_H

#include "ccp_frame.h"
#include "mac_budget.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/sys/util.h>

/* Offset from the beacon's RMARKER to the CCP's RMARKER, in UUS. The caller
 * multiplies by UUS_TO_DWT_TIME; this header stays free of radio headers so the
 * budget is checkable under plain gcc. */
#define CCP_OFFSET_UUS  BEACON_OCCUPANCY_UUS

/* ---- Derived quantities, for the asserts and for tests/ccp_sched ---- */

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

/* The CCP's scheduled RMARKER, relative to the beacon's. */
#define CCP_SCHED_AT_NS         MAC_UUS_TO_NS(CCP_OFFSET_UUS)

/* Where the beacon's own frame stops occupying the air, relative to its
 * RMARKER. UWB_FRAME_MAX_LEN is the beacon length: it is the longest frame the
 * contract defines, and tx_beacon() builds exactly that. */
#define CCP_SCHED_BEACON_END_NS                                                \
	CCP_SCHED_POST_RMARKER_NS(UWB_FRAME_MAX_LEN)

/* Where the slaves' suppression window closes, relative to the beacon's
 * RMARKER. */
#define CCP_SCHED_GUARD_END_NS                                                 \
	(MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS) + MAC_UUS_TO_NS(BEACON_GUARD_UUS))

/* The wall-clock budget to ARM the CCP -- not merely an airtime margin, which
 * is how the comment on assert (a) used to read this number.
 *
 * tx_beacon() returns only once the BEACON's TXFRS has fired, i.e. at
 * CCP_SCHED_BEACON_END_NS after the beacon's RMARKER. The CCP's own PREAMBLE
 * must begin at CCP_SCHED_AT_NS - CCP_SCHED_SHR_NS (its RMARKER minus its own
 * SHR). Between those two instants, ccp_master_after_beacon() must run
 * dwt_writesysstatuslo() -> uwb_get_tx_timestamp_u64() ->
 * dwt_setdelayedtrxtime() -> dwt_writetxdata() -> dwt_writetxfctrl() ->
 * dwt_starttx() -- five SPI transactions -- and have the delayed TX ARMED
 * before this window closes. That is a real scheduling constraint on the
 * K_PRIO_COOP(0) gateway loop, not comfortable headroom: mac_budget.h's own
 * MAC_TURNAROUND_FLOOR_PS carries this project's MEASURED arm-cost figures for
 * a comparable path (cir=133-156 us, readdata=23-33 us, readts=20 us, "so
 * ~200000 ns is the honest planning figure") -- and that path includes a CIR
 * read this one does not, so ~200 us over-estimates the CCP's true cost by an
 * AMOUNT NOBODY HAS MEASURED, not by a known, comfortable margin. Whether
 * 98856 ns is enough is exactly the open question this comment refuses to
 * paper over.
 *
 * This is also HALF of a two-sided legal window for CCP_OFFSET_UUS: this is
 * the lower bound (unmeasured on this exact path), and CCP_SCHED_CAP_PREAMBLE_NS
 * below derives the upper bound (1743 UUS -- see its comment). CCP_OFFSET_UUS
 * must be re-derived from a MEASURED beacon-TXFRS-to-dwt_starttx()-return cost
 * on real hardware and checked against both bounds, never guessed and never
 * retuned as a knob -- CLAUDE.md already records a board wedged once by
 * treating a delayed-TX budget that way (the beacon's own
 * TX_COMPLETE_TIMEOUT_MS regression). */
#define CCP_SCHED_ARM_BUDGET_NS                                                \
	(CCP_SCHED_AT_NS - CCP_SCHED_SHR_NS - CCP_SCHED_BEACON_END_NS)

/* (a) The CCP's PREAMBLE must not start before the beacon's frame has finished.
 * Checking the RMARKER alone would pass while the preamble sat on top of the
 * beacon's payload -- 1050 us of collision that a sniffer would show as a
 * corrupt beacon and nothing would attribute to the CCP.
 *
 * This margin IS CCP_SCHED_ARM_BUDGET_NS -- see its comment above for why that
 * number is a live risk, not a comfortable one. */
BUILD_ASSERT(CCP_SCHED_AT_NS >= CCP_SCHED_BEACON_END_NS + CCP_SCHED_SHR_NS,
	     "CCP preamble would start before the beacon frame ends");

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

/* (b) The CCP must be off the air before the first legitimate slave CAP
 * PREAMBLE, or it collides with it.
 *
 * Comparing against CCP_SCHED_GUARD_END_NS directly -- the RMARKER bound the
 * guard itself is written against -- over-claims the margin by a full SHR
 * (1050194 ns): it says nothing about when a slave's preamble can actually
 * appear, only about where its RMARKER may land. CCP_SCHED_CAP_PREAMBLE_NS
 * corrects for that, and is what this assert must compare against. */
BUILD_ASSERT(CCP_SCHED_AT_NS + CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN) <=
		     CCP_SCHED_CAP_PREAMBLE_NS,
	     "CCP still transmitting when the first slave CAP preamble may begin");

/* The offset ceiling this leaves CCP_OFFSET_UUS: solving assert (b) for
 * CCP_SCHED_AT_NS gives AT_NS <= CCP_SCHED_CAP_PREAMBLE_NS -
 * CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN) = 2026728 - 238823 = 1787905 ns,
 * i.e. AT_NS/MAC_UUS_PS <= 1743 UUS (integer UUS, truncated). Paired with
 * CCP_SCHED_ARM_BUDGET_NS above, CCP_OFFSET_UUS's legal window is two-sided:
 * bounded below by an unmeasured arm cost and above by 1743 UUS. Not encoded
 * as a macro or a third BUILD_ASSERT because CCP_FRAME_LEN's contribution
 * (238823 ns) is specific to today's CCP payload size and would need
 * re-deriving if that ever changes -- the two asserts above already check the
 * one CCP_OFFSET_UUS value that matters. */

#endif /* CCP_SCHED_H */

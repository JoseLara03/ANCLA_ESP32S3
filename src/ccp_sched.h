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

/* (a) The CCP's PREAMBLE must not start before the beacon's frame has finished.
 * Checking the RMARKER alone would pass while the preamble sat on top of the
 * beacon's payload -- 1050 us of collision that a sniffer would show as a
 * corrupt beacon and nothing would attribute to the CCP. */
BUILD_ASSERT(CCP_SCHED_AT_NS >= CCP_SCHED_BEACON_END_NS + CCP_SCHED_SHR_NS,
	     "CCP preamble would start before the beacon frame ends");

/* (b) The CCP must be off the air before the suppression window closes, or it
 * collides with the first legitimate slave transmit of the CAP. */
BUILD_ASSERT(CCP_SCHED_AT_NS + CCP_SCHED_POST_RMARKER_NS(CCP_FRAME_LEN) <=
		     CCP_SCHED_GUARD_END_NS,
	     "CCP still transmitting when the beacon guard closes");

#endif /* CCP_SCHED_H */

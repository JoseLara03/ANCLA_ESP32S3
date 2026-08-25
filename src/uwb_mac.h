/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Superframe constants from the MAC contract
 * (tag_testting/spec/2026-06-17-uwb-mac-protocol-contract.md, section 2.1).
 *
 * Fixed by protocol version and identical on every node, which is why they live
 * in one header rather than beside the code that uses them: the gateway
 * schedules the beacon from these and the slaves predict it from the same
 * values. Two copies would drift, and a slave predicting against a stale period
 * fails as intermittent beacon corruption -- which looks like an RF fault, not a
 * constant.
 *
 * The PHY contract lives separately in uwb_phy.h; this is the MAC layer.
 */

#ifndef UWB_MAC_H
#define UWB_MAC_H

/* T_superframe = 200 ms. 1 UUS = 512/499.2 MHz = 1.0256 us, so 195000 UUS is
 * 200.0 ms. */
#define T_SUPERFRAME_UUS 195000u

/* T_beacon ~1.5 ms, rounded up in UUS.
 *
 * T_guard covers two independent things, not just one:
 *   - Clock drift: two independent +/-20 ppm crystals over
 *     BEACON_GUARD_MAX_MISSES superframes (~800 ms) drift ~32 us relative --
 *     see BEACON_GUARD_MAX_MISSES in beacon_guard.h for that reasoning in
 *     full.
 *   - Physical airtime: both beacon_guard_beacon()'s RX timestamp and
 *     beacon_guard_tx_allowed()'s TX time are referenced to the RMARKER, but
 *     the actual collision window starts at the preamble, which precedes the
 *     RMARKER by ~1042 us at this PHY's PLEN_1024/64 MHz PRF. A guard sized
 *     only for clock drift under-covers the leading edge by that whole
 *     preamble length, letting a small fraction of responses land on the
 *     beacon's preamble even with the guard "working as designed".
 * 1500 rounds up 1042 us of preamble/airtime margin plus headroom; the drift
 * term above is small enough to fold in without pushing this out further.
 * The guard errs wide deliberately either way: suppressing one extra ranging
 * response costs one range, while corrupting one beacon costs every node in
 * the network. */
#define BEACON_OCCUPANCY_UUS 1500u
#define BEACON_GUARD_UUS     1500u

/* T_guard from the MAC contract section 2: the superframe partition guard that
 * precedes the CAP and follows the beacon and the CFP. 0.5 ms.
 *
 * NOT the same thing as BEACON_GUARD_UUS above, and conflating them is a real
 * hazard: BEACON_GUARD_UUS is the slave's TX-suppression window around the
 * beacon (sized for a whole preamble length, hence 1500), while this is the
 * inter-region margin the superframe budget charges three times. Using 1500
 * here would charge 4.6 ms of overhead that does not exist and understate the
 * slot count by ~1. */
#define T_GUARD_UUS          488u   /* 0.5 ms; 488 UUS = 500.5 us */

/* ---- Budget verification --------------------------------------------------
 *
 * N_CFP, BEACON_OCCUPANCY_UUS and POLL_RX_TO_RESP_TX_DLY_UUS were all set by
 * hand. These asserts are the first thing that checks them against the airtime
 * the PHY actually implies (src/mac_budget.h).
 *
 * The direction matters: UWB_FRAME_N_CFP stays an explicit LITERAL on the wire,
 * frozen by proto_ver, because two repositories computing it independently is
 * how the two sides come to disagree about a frame length. The model VERIFIES
 * the literal is feasible; it never supplies it.
 *
 * These live here and not in uwb_frame_802_15_4z.h on purpose: that file is
 * byte-identical with the tag's copy and the tag has no mac_budget.h. See
 * docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md section 8.0.
 */
#include <zephyr/sys/util.h>
#include "mac_budget.h"
#include "uwb_frame_802_15_4z.h"

/* One CFP slot must hold a 4-anchor SS-TWR sweep. */
#define UWB_MAC_CFP_SLOT_NS                                                    \
	(4u * MAC_PS_TO_NS(MAC_SSTWR_EXCHANGE_PS(                              \
		      MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM, UWB_FRAME_LEN_RESP,   \
		      MAC_PHY_BITRATE, 2000u)))

/* Superframe minus beacon, three guards, and the CAP. */
#define UWB_MAC_CFP_USABLE_NS                                                  \
	(MAC_UUS_TO_NS(T_SUPERFRAME_UUS) -                                     \
	 MAC_PS_TO_NS(MAC_FRAME_PS(MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM,          \
				   UWB_FRAME_LEN_BEACON, MAC_PHY_BITRATE)) -   \
	 3u * MAC_UUS_TO_NS(T_GUARD_UUS) -                                     \
	 UWB_FRAME_N_CAP *                                                     \
		 (MAC_PS_TO_NS(MAC_FRAME_PS(MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM, \
					    UWB_FRAME_LEN_KEEPALIVE,           \
					    MAC_PHY_BITRATE)) +                \
		  100000u))

/* The ranging slots the airtime budget actually affords. Currently 14. */
#define UWB_MAC_CFP_SLOTS_FEASIBLE                                             \
	(UWB_MAC_CFP_USABLE_NS / UWB_MAC_CFP_SLOT_NS)

/* If this fires, the beacon does not fit the window every slave reserves for
 * it, and delayed responses will land on the beacon's payload. */
BUILD_ASSERT(MAC_PS_TO_NS(MAC_FRAME_PS(MAC_PHY_PLEN_SYM, MAC_PHY_SFD_SYM,
				       UWB_FRAME_LEN_BEACON, MAC_PHY_BITRATE)) <
		     MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS),
	     "beacon airtime exceeds BEACON_OCCUPANCY_UUS");

/* If this fires, a full CFP overruns the superframe and the beacon arms late --
 * which is a network-wide timing fault, not a dropped frame. See CLAUDE.md on
 * why a late beacon is the worst failure this MAC has. */
BUILD_ASSERT(UWB_FRAME_N_CFP <= UWB_MAC_CFP_SLOTS_FEASIBLE,
	     "UWB_FRAME_N_CFP exceeds what the airtime budget affords");

#endif /* UWB_MAC_H */

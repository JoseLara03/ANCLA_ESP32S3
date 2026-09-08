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

/* ANNOUNCE (0xEC) rotation length (network-scaling-v3, design doc §3.C):
 * announce_id = frame_counter % ANNOUNCE_CYCLE_A, and the anchor whose id
 * matches transmits a passive ANNOUNCE in the window right after the beacon
 * it just heard. No wire byte carries announce_id -- both the gateway (which
 * generates frame_counter) and every anchor (which parses it straight back
 * out of the beacon it just received) already have everything needed to
 * compute the same value locally, so this is derived, not transmitted; the
 * design doc's original "beacon gains one byte" proposal is superseded by
 * this cheaper equivalent, which also does not force UWB_FRAME_LEN_BEACON to
 * grow before UWB_FRAME_N_CFP does (that bump is still gated on hardware --
 * see the anchor plan's Task 10 and the tag plan's Task 9).
 *
 * MUST stay coprime with the gateway's phase-cycle length (GW_CYCLE_C,
 * gw_core.h -- currently 16): a tag awake only on frame_counter % C == p
 * advances frame_counter by exactly C between every beacon it actually
 * receives, so the announce_id it observes cycles through only
 * gcd(ANNOUNCE_CYCLE_A, C) distinct values, not all of ANNOUNCE_CYCLE_A of
 * them. At ANNOUNCE_CYCLE_A == 16 that gcd is 16 (every value shared) and a
 * single-phase tag would see exactly ONE announce_id forever, never
 * discovering any anchor but the one holding that id. 37 is prime and
 * greater than UWB_MAX_ANCHORS (32), so it shares no factor with any C up to
 * 32 either, and by the Chinese Remainder Theorem every phase eventually
 * observes every announce_id, within at most ANNOUNCE_CYCLE_A * C
 * superframes (37 * 16 = 592, ~592 * 200 ms ~= 118 s at today's C). That is
 * fine for steady-state anchor-pool maintenance and useless for cold start,
 * which is why grouped DISCOVERY (disc_schedule.h) exists as the fast path
 * -- do not merge the two mechanisms. tests/gw_core/ proves the
 * coprimality property against GW_CYCLE_C directly rather than trusting the
 * arithmetic in this comment. */
#define ANNOUNCE_CYCLE_A 37u

/* Delay in UWB microseconds from a beacon's own RMARKER (its RX timestamp)
 * to this anchor's ANNOUNCE transmission, for the anchor whose turn it is.
 * TUNE ON HARDWARE, same status as DISC_BASE_UUS (disc_schedule.h) before it
 * was bench-confirmed -- this is a reasoned estimate, not a measurement.
 *
 * Must clear two things and land inside one: BEACON_OCCUPANCY_UUS (1500,
 * the beacon's own airtime from RMARKER) so the ANNOUNCE never starts before
 * the beacon has finished transmitting; the anchor's own RX-to-TX processing
 * turnaround (SPI reads, timestamp capture, frame build -- the same order of
 * magnitude the fast-SPI-bus responders measured at ~200-300 uus, see
 * DISC_BASE_UUS's history); and the tag's T_ANNOUNCE_MS (2000 uus,
 * tag_testting's uwb_net_runner.c) listen window, which the tag arms
 * immediately when ITS OWN beacon reception completes -- the same physical
 * instant as this anchor's, modulo propagation delay, which is negligible at
 * these ranges. 2500 sits centred in the resulting
 * [~1500+turnaround, 1500+2000] uus arrival window with margin on both
 * sides. */
#define T_ANNOUNCE_TX_DELAY_UUS (BEACON_OCCUPANCY_UUS + 1000u)

#endif /* UWB_MAC_H */

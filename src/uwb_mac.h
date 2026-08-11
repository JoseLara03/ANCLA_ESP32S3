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

#endif /* UWB_MAC_H */

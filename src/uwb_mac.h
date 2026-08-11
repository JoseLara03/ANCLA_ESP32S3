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

/* T_beacon ~1.5 ms and T_guard 0.5 ms, rounded up in UUS. The guard errs wide
 * deliberately: suppressing one extra ranging response costs one range, while
 * corrupting one beacon costs every node in the network. */
#define BEACON_OCCUPANCY_UUS 1500u
#define BEACON_GUARD_UUS      500u

#endif /* UWB_MAC_H */

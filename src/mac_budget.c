/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thin wrappers over the macros in mac_budget.h. The macros carry the
 * arithmetic so it can be used in a BUILD_ASSERT; these exist so the host
 * tests can sweep parameters, and so a shell command could report the budget
 * on target later without duplicating the model.
 */

#include "mac_budget.h"

void mac_phy_frozen(struct mac_phy *out)
{
	if (out == NULL) {
		return;
	}
	out->plen_sym    = MAC_PHY_PLEN_SYM;
	out->sfd_sym     = MAC_PHY_SFD_SYM;
	out->pac_sym     = MAC_PHY_PAC_SYM;
	out->bitrate_bps = MAC_PHY_BITRATE;
}

uint32_t mac_shr_ns(const struct mac_phy *p)
{
	return MAC_PS_TO_NS(MAC_SHR_PS(p->plen_sym, p->sfd_sym));
}

uint32_t mac_frame_ns(const struct mac_phy *p, uint16_t payload_bytes)
{
	return MAC_PS_TO_NS(MAC_FRAME_PS(p->plen_sym, p->sfd_sym, payload_bytes,
					 p->bitrate_bps));
}

uint32_t mac_sstwr_exchange_ns(const struct mac_phy *p, uint16_t resp_bytes,
			       uint32_t turnaround_uus)
{
	return MAC_PS_TO_NS(MAC_SSTWR_EXCHANGE_PS(p->plen_sym, p->sfd_sym,
						  resp_bytes, p->bitrate_bps,
						  turnaround_uus));
}

uint32_t mac_turnaround_floor_ns(const struct mac_phy *p, uint16_t rx_bytes,
				 uint32_t overhead_ns)
{
	return MAC_PS_TO_NS(MAC_TURNAROUND_FLOOR_PS(p->plen_sym, p->sfd_sym,
						    rx_bytes, p->bitrate_bps,
						    overhead_ns));
}

uint32_t mac_sfd_timeout_sym(const struct mac_phy *p)
{
	return MAC_SFD_TIMEOUT_SYM(p->plen_sym, p->sfd_sym, p->pac_sym);
}

uint32_t mac_cell_usable_ns(const struct mac_cell *c)
{
	/* Three guards: after the beacon, before the CAP, and after the CFP.
	 * Matching the contract's section 2 diagram rather than inventing a
	 * fourth. */
	uint64_t overhead = (uint64_t)c->beacon_ns +
			    3ULL * (uint64_t)c->guard_ns +
			    (uint64_t)c->n_cap * (uint64_t)c->minislot_ns;

	if (overhead >= (uint64_t)c->superframe_ns) {
		return 0u;
	}
	return (uint32_t)((uint64_t)c->superframe_ns - overhead);
}

uint16_t mac_cell_max_slots(const struct mac_cell *c, uint32_t slot_ns)
{
	uint32_t usable;

	if (slot_ns == 0u) {
		return 0u;
	}
	usable = mac_cell_usable_ns(c);

	/* Truncating division is deliberate: a partial slot is not a slot, and
	 * rounding up here is exactly how a superframe budget overruns and
	 * arms the beacon late. */
	return (uint16_t)(usable / slot_ns);
}

uint32_t mac_capacity_slot_sf(uint16_t n_slots, uint16_t window_sf)
{
	return (uint32_t)n_slots * (uint32_t)window_sf;
}

uint32_t mac_demand_slot_sf(uint16_t n_tags, uint16_t rate_div,
			    uint16_t window_sf)
{
	if (rate_div == 0u) {
		rate_div = 1u;
	}
	/* Participations one tag makes in the window, rounded UP: a tag that
	 * gets 1.4 participations must be budgeted 2, or the busiest superframe
	 * in the window is oversubscribed even though the average fits. */
	uint32_t per_tag = ((uint32_t)window_sf + rate_div - 1u) / rate_div;

	return (uint32_t)n_tags * per_tag;
}

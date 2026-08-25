/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Airtime and capacity model for the UWB MAC.
 *
 * This exists so that N_CFP, POLL_RX_TO_RESP_TX_DLY_UUS, BEACON_OCCUPANCY_UUS
 * and the guard constants stop being magic numbers. Every one of them is
 * derivable from the PHY contract (uwb_phy.h), and this is where the
 * derivation lives -- so a PHY change re-derives them instead of silently
 * invalidating them, and so a proposed capacity number can be checked instead
 * of argued about.
 *
 * Two design rules:
 *
 *   1. Everything is INTEGER arithmetic in picoseconds, with no float. That is
 *      what lets these expressions appear in a BUILD_ASSERT. The macros are
 *      the contract; the functions below are thin wrappers so the host tests
 *      can sweep parameters the macros are instantiated with.
 *
 *   2. Byte counts are payload EXCLUDING the 2-byte FCS, matching the frame
 *      module's UWB_FRAME_LEN_* convention. The FCS is added internally. This
 *      project has been bitten by FCS accounting before (see CLAUDE.md on
 *      dwt_getframelength()), so the convention is stated once, here, and
 *      every entry point follows it.
 *
 * See docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md section 3.
 */

#ifndef MAC_BUDGET_H
#define MAC_BUDGET_H

#include <stdint.h>

/* ---- PHY-derived constants ------------------------------------------------
 *
 * Preamble symbol duration at 64 MHz mean PRF. The 16 MHz PRF figure is
 * 993.59 ns; this network is 64 MHz throughout (uwb_phy.h), so only the one
 * value is carried -- a second constant nothing selects between is a trap.
 */
#define MAC_PREAMBLE_SYM_PS   1017630ULL

/* 1 UUS = 512 / 499.2 MHz = 1.025641 us. The same conversion UUS_TO_DWT_TIME
 * is built on; kept here in ps so this header needs no DW3000 dependency and
 * can be host-compiled. */
#define MAC_UUS_PS            1025641ULL

/* PHY header, standard mode: 19 bits, sent at the data rate when
 * DWT_PHRRATE_STD is selected (which uwb_phy.h does). */
#define MAC_PHR_BITS          19ULL

/* Appended by dwt_writetxfctrl, never counted in UWB_FRAME_LEN_*. */
#define MAC_FCS_BYTES         2ULL

/* ---- Primitive durations, picoseconds ----------------------------------- */

#define MAC_BITS_PS(bits, bps) \
	((uint64_t)(bits) * 1000000000000ULL / (uint64_t)(bps))

/* SHR = preamble + SFD. The RMARKER sits at the END of the SHR, which is the
 * fact the exchange model below turns on. */
#define MAC_SHR_PS(plen_sym, sfd_sym) \
	(((uint64_t)(plen_sym) + (uint64_t)(sfd_sym)) * MAC_PREAMBLE_SYM_PS)

/* Whole frame, SHR through FCS. `bytes` excludes the FCS. */
#define MAC_FRAME_PS(plen_sym, sfd_sym, bytes, bps)                            \
	(MAC_SHR_PS(plen_sym, sfd_sym) + MAC_BITS_PS(MAC_PHR_BITS, bps) +      \
	 MAC_BITS_PS(((uint64_t)(bytes) + MAC_FCS_BYTES) * 8ULL, bps))

#define MAC_PS_TO_NS(ps)    ((uint32_t)((uint64_t)(ps) / 1000ULL))
#define MAC_UUS_TO_NS(uus)  ((uint32_t)((uint64_t)(uus) * MAC_UUS_PS / 1000ULL))

/* ---- SS-TWR exchange ----------------------------------------------------
 *
 * One initiator poll plus one responder reply, measured from the first chip of
 * the poll's SHR to the last chip of the response's FCS.
 *
 * Deliberately NOT `poll_frame + turnaround + resp_frame`. That form
 * DOUBLE-COUNTS both SHRs and inflates the result by ~2.1 ms at PLEN_1024 --
 * it is the error the design spec was first written with. The turnaround
 * constant (POLL_RX_TO_RESP_TX_DLY_UUS) is measured RMARKER to RMARKER, and
 * each RMARKER is already at the end of its own SHR, so the span is:
 *
 *   poll SHR  +  turnaround  +  (response PHR + payload + FCS)
 *
 * with the poll's own payload and the response's SHR both falling INSIDE the
 * turnaround window rather than beside it.
 */
#define MAC_SSTWR_EXCHANGE_PS(plen_sym, sfd_sym, resp_bytes, bps, turn_uus)    \
	(MAC_SHR_PS(plen_sym, sfd_sym) +                                       \
	 (uint64_t)(turn_uus) * MAC_UUS_PS +                                   \
	 MAC_BITS_PS(MAC_PHR_BITS, bps) +                                      \
	 MAC_BITS_PS(((uint64_t)(resp_bytes) + MAC_FCS_BYTES) * 8ULL, bps))

/* ---- Turnaround floor ---------------------------------------------------
 *
 * The smallest legal POLL_RX_TO_RESP_TX_DLY_UUS. After the poll's RMARKER the
 * responder must still (a) receive the rest of the poll, (b) read it out over
 * SPI and arm the delayed TX, and (c) transmit its own SHR, because the
 * response's RMARKER lands only after that SHR has gone out.
 *
 * `overhead_ns` is the measured RX-to-dwt_starttx() cost. At the 26.67 MHz SPI
 * rate this project measured cir=133-156 us, readdata=23-33 us, readts=20 us,
 * so ~200000 ns is the honest planning figure. It was ~1700 us at the old
 * 2 MHz rate -- see CLAUDE.md on why the bus, not the driver, was the cost.
 */
#define MAC_TURNAROUND_FLOOR_PS(plen_sym, sfd_sym, rx_bytes, bps, overhead_ns) \
	(MAC_BITS_PS(MAC_PHR_BITS, bps) +                                      \
	 MAC_BITS_PS(((uint64_t)(rx_bytes) + MAC_FCS_BYTES) * 8ULL, bps) +     \
	 (uint64_t)(overhead_ns) * 1000ULL +                                   \
	 MAC_SHR_PS(plen_sym, sfd_sym))

/* ---- SFD timeout -------------------------------------------------------
 *
 * PLEN + 1 + SFD - PAC. Trivial, and here anyway because the two repos
 * disagreed on it for months: the anchor carried 1001 and the tag 1025 while
 * both ran PAC32 (design spec section 2.8). A named derivation is harder to
 * get wrong than an open-coded sum repeated 20 times.
 */
#define MAC_SFD_TIMEOUT_SYM(plen_sym, sfd_sym, pac_sym) \
	((uint32_t)(plen_sym) + 1u + (uint32_t)(sfd_sym) - (uint32_t)(pac_sym))

/* ---- The frozen PHY ----------------------------------------------------
 *
 * Mirrors uwb_phy.h. Frozen 2026-08-25: TDoA carries the capacity target at
 * PLEN_1024, so there is no reason to spend the 6 dB of preamble integration
 * gain that shortening it would cost. Freezing also keeps every unit's
 * antenna-delay calibration valid.
 */
#define MAC_PHY_PLEN_SYM    1024u
#define MAC_PHY_SFD_SYM     8u
#define MAC_PHY_PAC_SYM     32u
#define MAC_PHY_BITRATE     850000u

/* ---- Runtime / host-test API -------------------------------------------
 *
 * Thin wrappers over the macros so tests can sweep parameters. Pure; no state.
 * All return nanoseconds unless named otherwise.
 */
struct mac_phy {
	uint16_t plen_sym;
	uint8_t  sfd_sym;
	uint8_t  pac_sym;
	uint32_t bitrate_bps;
};

/* The frozen PHY as a struct, for the sweep tests. */
void     mac_phy_frozen(struct mac_phy *out);

uint32_t mac_shr_ns(const struct mac_phy *p);
uint32_t mac_frame_ns(const struct mac_phy *p, uint16_t payload_bytes);
uint32_t mac_sstwr_exchange_ns(const struct mac_phy *p, uint16_t resp_bytes,
			       uint32_t turnaround_uus);
uint32_t mac_turnaround_floor_ns(const struct mac_phy *p, uint16_t rx_bytes,
				 uint32_t overhead_ns);
uint32_t mac_sfd_timeout_sym(const struct mac_phy *p);

/* ---- Cell budget ------------------------------------------------------
 *
 * The superframe overhead a cell pays before any ranging or blink slot fits.
 * Layout per the MAC contract section 2:
 *
 *   [BEACON][g][ CAP: n_cap minislots ][g][ CFP: slots ][g]
 */
struct mac_cell {
	uint32_t superframe_ns;
	uint32_t beacon_ns;
	uint32_t guard_ns;    /* one guard; three are charged */
	uint32_t minislot_ns;
	uint8_t  n_cap;
};

/* Nanoseconds left for CFP / blink slots after beacon, guards and CAP.
 * Returns 0 rather than underflowing if the overhead exceeds the superframe. */
uint32_t mac_cell_usable_ns(const struct mac_cell *c);

/* How many slots of `slot_ns` fit in that usable span. */
uint16_t mac_cell_max_slots(const struct mac_cell *c, uint32_t slot_ns);

/* ---- Capacity ---------------------------------------------------------
 *
 * Capacity is counted in SLOT-SUPERFRAMES over a window, which is what makes
 * tiers comparable: a tag at 5 Hz occupies its slot in every superframe, a tag
 * at 0.2 Hz in one of every 25. `rate_div` is superframes per participation
 * (1 = 5 Hz, 5 = 1 Hz, 25 = 0.2 Hz at a 200 ms superframe).
 *
 * NOTE: this measures what a CORRECT scheduler could serve. Today's gateway
 * cannot -- gw_core_build_slotmap() writes a seat's address into its slot every
 * superframe regardless of tier, so the real capacity is n_slots tags, full
 * stop. That gap is the point of design spec section 4.1.
 */
uint32_t mac_capacity_slot_sf(uint16_t n_slots, uint16_t window_sf);
uint32_t mac_demand_slot_sf(uint16_t n_tags, uint16_t rate_div,
			    uint16_t window_sf);

#endif /* MAC_BUDGET_H */

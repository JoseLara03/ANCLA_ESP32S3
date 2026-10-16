/*
 * Host tests for mac_budget.
 *
 * The point of this suite is NOT that the model is self-consistent -- that is
 * easy and worthless. It is that the model REPRODUCES the constants this
 * firmware already ships and has bench-confirmed, so that when it then says
 * something new (a slot count, a capacity ceiling) that claim inherits the
 * same credibility.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/mac_budget/test_mac_budget.exe \
 *       tests/mac_budget/test_mac_budget.c src/mac_budget.c
 */

#include <stdio.h>
#include <stdlib.h>
#include "mac_budget.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

/* Within `tol` ns of `want`. */
#define CHECK_NEAR(got, want, tol)                                             \
	do {                                                                   \
		long g = (long)(got), w = (long)(want), t = (long)(tol);        \
		if (g - w > t || w - g > t) {                                  \
			printf("FAIL %s:%d: %s = %ld, want %ld +/- %ld\n",      \
			       __FILE__, __LINE__, #got, g, w, t);             \
			failures++;                                            \
		}                                                              \
	} while (0)

/* ---- The constants this model must reproduce ---------------------------
 * Copied, not included: these live in Zephyr-dependent headers. A divergence
 * between these literals and the real headers is caught by the BUILD_ASSERTs
 * in uwb_mac.h, which is where the two meet on target.
 */
#define BEACON_OCCUPANCY_UUS        1500u   /* uwb_mac.h */
#define POLL_RX_TO_RESP_TX_DLY_UUS  2000u   /* anchor_respond.c */
#define T_SUPERFRAME_UUS            195000u /* uwb_mac.h */
#define UWB_FRAME_N_CFP             11u     /* uwb_frame_802_15_4z.h */
#define UWB_FRAME_LEN_BEACON        (15u + 2u * UWB_FRAME_N_CFP) /* 37 */
#define UWB_FRAME_LEN_RESP          20u
#define UWB_FRAME_LEN_KEEPALIVE     12u
#define POS_POLL_LEN                11u     /* WAVE positioning poll */

/* Measured RX-to-dwt_starttx() cost at the 26.67 MHz SPI rate. */
#define RX_TO_TX_OVERHEAD_NS        200000u

/* ---- 1. The PHY primitives ------------------------------------------- */

static void test_preamble_and_frames(void)
{
	struct mac_phy phy;

	mac_phy_frozen(&phy);

	/* PLEN_256 + 8-symbol SFD at 64 MHz PRF: (256 + 8) * 1017.63 ns =
	 * 268654 ns. (It was (1024 + 8) * 1017.63 = 1050194 ns at PLEN_1024,
	 * the "~1.05 ms" that once made a 450 uus turnaround on the
	 * DWM3001CDK physically impossible; see test_turnaround_floor for
	 * what the shorter SHR does to that.) */
	CHECK_NEAR(mac_shr_ns(&phy), 268654, 500);

	/* The beacon must fit inside BEACON_OCCUPANCY_UUS. That constant was
	 * set by hand at 1500 uus; this is the first time it has been checked
	 * against the actual frame. */
	uint32_t beacon_ns = mac_frame_ns(&phy, UWB_FRAME_LEN_BEACON);

	/* SHR 268654 + PHR 22353 + 39 B (312 bit) 367059 ns */
	CHECK_NEAR(beacon_ns, 658066, 1000);
	CHECK(beacon_ns < MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS));

	/* ...and by a sane margin, not by 1 ns. */
	CHECK(MAC_UUS_TO_NS(BEACON_OCCUPANCY_UUS) - beacon_ns > 50000u);

	/* A response frame, for the exchange model below. */
	/* SHR 268654 + PHR 22353 + 22 B (176 bit) 207059 ns */
	CHECK_NEAR(mac_frame_ns(&phy, UWB_FRAME_LEN_RESP), 498066, 1000);
}

/* ---- 2. The SFD-timeout divergence this work found ------------------- */

static void test_sfd_timeout(void)
{
	struct mac_phy phy;

	mac_phy_frozen(&phy);

	/* uwb_phy.h now carries (256 + 1 + 8 - 16) = 249 (PLEN_256/PAC16).
	 * At PLEN_1024/PAC32 it was (1025 + 8 - 32) = 1001; the tag carried
	 * (1024 + 1 + 8 - 8) = 1025 while ALSO running PAC32, which is the
	 * divergence corrected on 2026-08-25. */
	CHECK(mac_sfd_timeout_sym(&phy) == 249u);
	CHECK(MAC_SFD_TIMEOUT_SYM(1024u, 8u, 32u) == 1001u);

	/* The wrong value, pinned so nobody "restores" it: it is what you get
	 * by subtracting PAC8 from a PAC32 configuration. */
	CHECK(MAC_SFD_TIMEOUT_SYM(1024u, 8u, 8u) == 1025u);

	/* PAC really does drive it. */
	CHECK(MAC_SFD_TIMEOUT_SYM(1024u, 8u, 16u) == 1017u);
}

/* ---- 3. The turnaround constant is feasible -------------------------- */

static void test_turnaround_floor(void)
{
	struct mac_phy phy;

	mac_phy_frozen(&phy);

	uint32_t floor_ns = mac_turnaround_floor_ns(&phy, POS_POLL_LEN,
						    RX_TO_TX_OVERHEAD_NS);
	uint32_t actual_ns = MAC_UUS_TO_NS(POLL_RX_TO_RESP_TX_DLY_UUS);

	/* ~613 us: poll PHR+payload+FCS after the RMARKER (~145 us), SPI and
	 * compute (200 us), then the response's own SHR (~269 us), which must
	 * go out before the response RMARKER lands. (1395 us at PLEN_1024.) */
	CHECK_NEAR(floor_ns, 613360, 3000);

	/* The shipped 2000 uus clears it -- this is the first time that has
	 * been demonstrated rather than asserted. */
	CHECK(actual_ns > floor_ns);

	/* Margin, ~1438 us (was ~656 us at PLEN_1024). At PLEN_256 the
	 * 2000 uus constant is no longer tight: 1000 uus (1026 us) would also
	 * clear the floor. The constant is deliberately unchanged here;
	 * shortening it is a separate decision. */
	CHECK_NEAR(actual_ns - floor_ns, 1437922, 4000);

	/* The Qorvo ss_twr_responder stock value cannot work here, which is
	 * exactly what CLAUDE.md says about the DWM3001CDK reference node. */
	CHECK(MAC_UUS_TO_NS(450u) < floor_ns);
}

/* ---- 4. The SS-TWR exchange, and the double-counting trap ----------- */

static void test_sstwr_exchange(void)
{
	struct mac_phy phy;

	mac_phy_frozen(&phy);

	uint32_t exch = mac_sstwr_exchange_ns(&phy, UWB_FRAME_LEN_RESP,
					      POLL_RX_TO_RESP_TX_DLY_UUS);

	/* ~2549 us: poll SHR (268654) + turnaround (2051282) + the response's
	 * PHR/payload/FCS (229412). (3331 us at PLEN_1024.) */
	CHECK_NEAR(exch, 2549348, 3000);

	/* The naive form -- poll_frame + turnaround + resp_frame -- comes out
	 * too high (by exactly the poll's whole frame: ~413 us at PLEN_256,
	 * ~1194 us at PLEN_1024) because it charges BOTH SHRs on top of a
	 * turnaround that already contains one of them. Pinned here because
	 * the design spec was first written with exactly this error, and it
	 * inflated the 4-anchor slot from 13.3 ms to 18.1 ms (at PLEN_1024).
	 */
	uint32_t naive = mac_frame_ns(&phy, POS_POLL_LEN) +
			 MAC_UUS_TO_NS(POLL_RX_TO_RESP_TX_DLY_UUS) +
			 mac_frame_ns(&phy, UWB_FRAME_LEN_RESP);

	CHECK(naive > exch);
	CHECK_NEAR(naive - exch, 413360, 5000);

	/* A 4-anchor sweep, which is what one CFP slot must hold. The MAC
	 * contract section 2.1 estimated T_slot at ~15 ms by hand (PLEN_1024);
	 * the model said 13.3 ms there and says 10.2 ms at PLEN_256. */
	uint32_t slot4 = 4u * exch;

	CHECK_NEAR(slot4, 10197392, 12000);
	CHECK(slot4 < 15000000u);
}

/* ---- 5. N_CFP = 11 is feasible (this is what A2 asserts on target) --- */

static void frozen_cell(struct mac_cell *c, const struct mac_phy *phy)
{
	c->superframe_ns = MAC_UUS_TO_NS(T_SUPERFRAME_UUS);
	c->beacon_ns     = mac_frame_ns(phy, UWB_FRAME_LEN_BEACON);
	/* MAC contract section 2.1: T_guard = 0.5 ms. Distinct from
	 * BEACON_GUARD_UUS (1500), which is the slave's TX-suppression window
	 * around the beacon, not a superframe partition guard. Conflating the
	 * two is a real hazard: it would triple the charged overhead. */
	c->guard_ns      = 500000u;
	c->minislot_ns   = mac_frame_ns(phy, UWB_FRAME_LEN_KEEPALIVE) + 100000u;
	c->n_cap         = 4u;
}

static void test_n_cfp_is_feasible(void)
{
	struct mac_phy phy;
	struct mac_cell cell;

	mac_phy_frozen(&phy);
	frozen_cell(&cell, &phy);

	/* The superframe is 200.0 ms exactly -- 195000 UUS. */
	CHECK_NEAR(cell.superframe_ns, 200000000, 5000);

	uint32_t usable = mac_cell_usable_ns(&cell);
	uint32_t slot   = 4u * mac_sstwr_exchange_ns(&phy, UWB_FRAME_LEN_RESP,
						     POLL_RX_TO_RESP_TX_DLY_UUS);
	uint16_t maxs   = mac_cell_max_slots(&cell, slot);

	CHECK(usable > 190000000u);   /* ~195.75 ms */

	/* 19 slots fit (195.75 ms / 10.197 ms = 19.2; 14 at PLEN_1024). The shipped N_CFP is 11, so the constant is
	 * CONSERVATIVE, not optimistic -- and this is the assertion A2 puts
	 * behind a BUILD_ASSERT in uwb_mac.h. */
	CHECK(maxs == 19u);
	CHECK(UWB_FRAME_N_CFP <= maxs);

	/* Sanity on the other side: 11 slots really do fit with room left. */
	CHECK((uint64_t)UWB_FRAME_N_CFP * slot < usable);

	/* And the model has teeth -- a slot twice as wide would not fit 11. */
	CHECK(mac_cell_max_slots(&cell, slot * 2u) < UWB_FRAME_N_CFP);
}

/* ---- 6. TDoA is what reaches 100 tags; TWR is not ------------------- */

/* Blink: 10-byte header + type + seq + flags/batt. Excludes FCS. */
#define BLINK_LEN            14u
#define BLINK_GUARD_NS      100000u
#define WINDOW_SF            25u    /* the IDLE tier's period */
#define TAGS_TARGET         100u

static void test_capacity_100_tags(void)
{
	struct mac_phy phy;
	struct mac_cell cell;

	mac_phy_frozen(&phy);
	frozen_cell(&cell, &phy);

	/* --- TWR. The slot width itself is checked in
	 * test_n_cfp_is_feasible(); here only the seat count matters, because
	 * the TWR ceiling is N_CFP, not airtime. --- */
	uint32_t twr_cap = mac_capacity_slot_sf(UWB_FRAME_N_CFP, WINDOW_SF);

	CHECK(twr_cap == 275u);

	/* Shift change: all 100 tags moving, every one wanting 5 Hz. */
	uint32_t demand_5hz = mac_demand_slot_sf(TAGS_TARGET, 1u, WINDOW_SF);

	CHECK(demand_5hz == 2500u);

	/* TWR is ~9x short. This is the finding that makes TDoA a product
	 * requirement rather than an optimization. */
	CHECK(demand_5hz > twr_cap * 9u);

	/* Even all-IDLE fits TWR, which is why the current system looks fine
	 * on a bench with a handful of stationary tags. */
	CHECK(mac_demand_slot_sf(TAGS_TARGET, 25u, WINDOW_SF) <= twr_cap);

	/* But all-1 Hz does not. */
	CHECK(mac_demand_slot_sf(TAGS_TARGET, 5u, WINDOW_SF) > twr_cap);

	/* --- TDoA --- */
	uint32_t blink_slot = mac_frame_ns(&phy, BLINK_LEN) + BLINK_GUARD_NS;
	uint16_t blink_max  = mac_cell_max_slots(&cell, blink_slot);

	/* 441595 ns frame (14 B + FCS = 16 B) + 100000 ns guard */
	CHECK_NEAR(blink_slot, 541595, 3000);

	/* 195750845 / 541595 = 361 blink slots per superframe at the frozen
	 * PLEN_256 PHY (~144 at PLEN_1024). */
	CHECK(blink_max >= 355u && blink_max <= 367u);

	uint32_t tdoa_cap = mac_capacity_slot_sf(blink_max, WINDOW_SF);

	/* The shift-change case fits with room to spare. */
	CHECK(demand_5hz < tdoa_cap);

	/* Specifically: 100 tags at 5 Hz uses ~28% of one superframe's slots
	 * (~70% at PLEN_1024), so the headroom is real and not marginal. */
	CHECK(TAGS_TARGET * 100u / blink_max >= 25u);
	CHECK(TAGS_TARGET * 100u / blink_max <= 30u);

	/* TDoA beats TWR by an order of magnitude in simultaneous movers. */
	CHECK(blink_max > UWB_FRAME_N_CFP * 10u);
}

/* ---- 7. PLEN_256 vs the previous PLEN_1024 PHY ---------------------- */

static void test_shortening_plen_buys_capacity(void)
{
	struct mac_phy p256, p1024;
	struct mac_cell c256, c1024;

	/* The frozen PHY is now PLEN_256/PAC16; PLEN_1024/PAC32 is the previous
	 * contract, spelled out explicitly as the comparison point. */
	mac_phy_frozen(&p256);
	CHECK(p256.plen_sym == 256u && p256.pac_sym == 16u);
	p1024 = p256;
	p1024.plen_sym = 1024u;
	p1024.pac_sym  = 32u;

	frozen_cell(&c1024, &p1024);
	frozen_cell(&c256, &p256);

	uint32_t blink_1024 = mac_frame_ns(&p1024, BLINK_LEN) + BLINK_GUARD_NS;
	uint32_t blink_256  = mac_frame_ns(&p256, BLINK_LEN) + BLINK_GUARD_NS;

	uint16_t max_1024 = mac_cell_max_slots(&c1024, blink_1024);
	uint16_t max_256  = mac_cell_max_slots(&c256, blink_256);

	/* Shortening the preamble really does buy a lot of airtime... */
	CHECK(max_256 > max_1024 * 2u);

	/* ...PLEN_1024 already cleared the 100-tag target at 5 Hz, so the
	 * extra capacity is headroom, paid for with ~6 dB of preamble
	 * integration gain against metal and NLOS. */
	CHECK(max_1024 > TAGS_TARGET);
	CHECK(max_256 > TAGS_TARGET);

	/* And the PAC must track PLEN, or the SFD timeout goes wrong the same
	 * way the tag's did. */
	CHECK(mac_sfd_timeout_sym(&p256) == 249u);
	CHECK(mac_sfd_timeout_sym(&p1024) == 1001u);
}

int main(void)
{
	test_preamble_and_frames();
	test_sfd_timeout();
	test_turnaround_floor();
	test_sstwr_exchange();
	test_n_cfp_is_feasible();
	test_capacity_100_tags();
	test_shortening_plen_buys_capacity();

	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("mac_budget: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

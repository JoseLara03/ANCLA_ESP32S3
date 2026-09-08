#include "gw_core.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void mk_eui(uint8_t out[UWB_FRAME_EUI_LEN], uint8_t tag)
{
    for (int i = 0; i < UWB_FRAME_EUI_LEN; i++) out[i] = (uint8_t)(tag + i);
}

static void test_init(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    CHECK(c.frame_counter == 0);
    CHECK(c.next_short_addr == GW_TAG_ADDR_BASE);
    for (int p = 0; p < GW_CYCLE_C; p++)
        for (int s = 0; s < GW_N_CFP; s++)
            CHECK(c.seats[p][s].short_addr == 0);
}

static void test_join_fills_slots_in_order(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (int i = 0; i < 3; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)(0x10 + i));
        CHECK(gw_core_join(&c, eui, 1, &g));
        CHECK(g.phase == 0);              /* phase-major fill: phase 0 first */
        CHECK(g.slot_index == i);
        CHECK(g.short_addr == (uint16_t)(GW_TAG_ADDR_BASE + i));
        CHECK(g.lease == GW_LEASE_SF);
        CHECK(g.tier == 1);
    }
}

/* The interim single-phase JOIN policy (see gw_core_join()'s comment) scans
 * phase-major: it must completely fill phase 0's GW_N_CFP slots before it
 * ever places a seat in phase 1. This pins down exactly where that boundary
 * falls; test_network_full below only proves the policy eventually reaches
 * every phase, not where each one starts. */
static void test_join_advances_to_next_phase_when_full(void)
{
    struct gw_core_ctx c;
    struct gw_grant g;

    gw_core_init(&c);

    for (int s = 0; s < GW_N_CFP; s++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, (uint8_t)s);
        CHECK(gw_core_join(&c, eui, 1, &g));
        CHECK(g.phase == 0);
        CHECK(g.slot_index == s);
    }

    /* Phase 0 is now completely full; the next JOIN must land in phase 1,
     * slot 0 -- not be refused, and not wrap back into phase 0. */
    uint8_t extra[UWB_FRAME_EUI_LEN];
    mk_eui(extra, (uint8_t)GW_N_CFP);
    CHECK(gw_core_join(&c, extra, 1, &g));
    CHECK(g.phase == 1);
    CHECK(g.slot_index == 0);
}

static void test_rejoin_is_idempotent(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant g1, g2, g3;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);

    CHECK(gw_core_join(&c, eui_a, 1, &g1));
    CHECK(gw_core_join(&c, eui_b, 1, &g2));

    /* A repeat JOIN from a known EUI -- a tag that missed its GRANT and
     * retried -- must return the SAME seat, not consume a second one. */
    CHECK(gw_core_join(&c, eui_a, 2, &g3));
    CHECK(g3.phase == g1.phase);
    CHECK(g3.slot_index == g1.slot_index);
    CHECK(g3.short_addr == g1.short_addr);
    CHECK(g3.tier == 2);          /* but the tier is updated */
}

/* The brief's headline acceptance test. With seats now spread across
 * GW_CYCLE_C phases, filling just GW_N_CFP tags no longer exhausts the table
 * (see test_join_advances_to_next_phase_when_full above) -- the network is
 * only truly full at GW_CYCLE_C * GW_N_CFP seats (176 today; 224 once a
 * separate, deferred task grows GW_N_CFP 11 -> 14). Computed here
 * independently of the header's GW_MAX_SEATS macro, rather than trusting it,
 * so a mistake in that macro's own definition can't hide a matching mistake
 * in this test. */
static void test_network_full(void)
{
    struct gw_core_ctx c;
    int total_seats = (int)(GW_CYCLE_C * GW_N_CFP);

    gw_core_init(&c);

    for (int i = 0; i < total_seats; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        CHECK(gw_core_join(&c, eui, 1, &g));
    }

    /* One more distinct tag, after every seat is taken, must be refused
     * cleanly -- not silently overwrite an existing seat or wrap around. */
    uint8_t extra[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(extra, (uint8_t)total_seats);
    CHECK(!gw_core_join(&c, extra, 1, &g));
}

static void test_keepalive(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (unsigned i = 0; i < 10; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.phase][g.slot_index].lease_remaining == GW_LEASE_SF - 10);

    gw_core_keepalive(&c, g.short_addr, 3);
    CHECK(c.seats[g.phase][g.slot_index].lease_remaining == GW_LEASE_SF);
    CHECK(c.seats[g.phase][g.slot_index].tier == 3);

    /* An unknown address must not disturb anything. */
    gw_core_keepalive(&c, 0xBEEF, 1);
    CHECK(c.seats[g.phase][g.slot_index].tier == 3);
}

static void test_lease_expiry(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (unsigned i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.phase][g.slot_index].short_addr != 0);    /* still held */

    gw_core_superframe_tick(&c);
    CHECK(c.seats[g.phase][g.slot_index].short_addr == 0);    /* reclaimed */
    CHECK(c.frame_counter == GW_LEASE_SF);
}

/* The brief's phase-independence requirement: a seat's lease must reach zero
 * after exactly GW_LEASE_SF gw_core_superframe_tick() calls no matter which
 * phase it lives in, because that function ages EVERY phase on every call
 * (see its comment) -- there is no "current phase" that ages while the
 * others wait. Forces two tags into different phases (0 and 2) by fully
 * occupying the phases before the second one, then ticks once short of
 * GW_LEASE_SF and once more, checking both seats reclaim on the exact same
 * call as test_lease_expiry()'s single-phase case above. */
static void test_lease_expiry_same_regardless_of_phase(void)
{
    struct gw_core_ctx c;
    struct gw_grant g_phase0, g_phase2, g;

    gw_core_init(&c);

    /* First tag: lands phase 0, slot 0. */
    {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, 0);
        CHECK(gw_core_join(&c, eui, 1, &g_phase0));
        CHECK(g_phase0.phase == 0);
    }

    /* Fill the rest of phase 0 (tags 1..GW_N_CFP-1) and all of phase 1
     * (tags GW_N_CFP..2*GW_N_CFP-1), so the next JOIN is pushed into phase 2. */
    for (int s = 1; s < GW_N_CFP; s++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, (uint8_t)s);
        CHECK(gw_core_join(&c, eui, 1, &g));
    }
    for (int s = 0; s < GW_N_CFP; s++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, (uint8_t)(GW_N_CFP + s));
        CHECK(gw_core_join(&c, eui, 1, &g));
    }

    {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, (uint8_t)(2 * GW_N_CFP));
        CHECK(gw_core_join(&c, eui, 1, &g_phase2));
        CHECK(g_phase2.phase == 2);
    }

    for (unsigned i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g_phase0.phase][g_phase0.slot_index].short_addr != 0);
    CHECK(c.seats[g_phase2.phase][g_phase2.slot_index].short_addr != 0);

    gw_core_superframe_tick(&c);
    CHECK(c.seats[g_phase0.phase][g_phase0.slot_index].short_addr == 0);
    CHECK(c.seats[g_phase2.phase][g_phase2.slot_index].short_addr == 0);
    CHECK(c.frame_counter == GW_LEASE_SF);
}

static void test_release(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));
    gw_core_release(&c, g.short_addr);
    CHECK(c.seats[g.phase][g.slot_index].short_addr == 0);

    /* Releasing an unknown address is a no-op, not a crash. */
    gw_core_release(&c, 0xBEEF);
}

static void test_slotmap(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    uint16_t map[GW_N_CFP];

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));
    gw_core_build_slotmap(&c, g.phase, map);

    CHECK(map[g.slot_index] == g.short_addr);
    for (int i = 0; i < GW_N_CFP; i++) {
        if (i != g.slot_index) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);
    }

    /* A phase nothing was ever joined into must publish an all-broadcast row
     * -- gw_core_build_slotmap() must not leak the other phase's occupant. */
    uint16_t other_map[GW_N_CFP];
    uint8_t other_phase = (uint8_t)(g.phase == 0 ? 1 : 0);

    gw_core_build_slotmap(&c, other_phase, other_map);
    for (int i = 0; i < GW_N_CFP; i++) CHECK(other_map[i] == UWB_FRAME_ADDR_BCAST);
}

/* "Every phase's map is consistent across a full cycle" (task brief): fills
 * phase 0 completely, adds one seat in phase 1, ticks a handful of
 * superframes (fewer than GW_LEASE_SF, so nothing expires), and checks each
 * phase's slotmap still reflects exactly its own seats -- unperturbed by the
 * other phase's occupancy and unperturbed by the ticks in between. */
static void test_slotmap_isolated_per_phase_across_ticks(void)
{
    struct gw_core_ctx c;
    uint16_t phase0_addrs[GW_N_CFP];
    struct gw_grant g;

    gw_core_init(&c);

    for (int s = 0; s < GW_N_CFP; s++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        mk_eui(eui, (uint8_t)s);
        CHECK(gw_core_join(&c, eui, 1, &g));
        CHECK(g.phase == 0);
        CHECK(g.slot_index == s);
        phase0_addrs[s] = g.short_addr;
    }

    uint8_t extra[UWB_FRAME_EUI_LEN];
    struct gw_grant g_phase1;
    mk_eui(extra, (uint8_t)GW_N_CFP);
    CHECK(gw_core_join(&c, extra, 1, &g_phase1));
    CHECK(g_phase1.phase == 1);
    CHECK(g_phase1.slot_index == 0);

    for (unsigned i = 0; i < 5; i++) gw_core_superframe_tick(&c);

    uint16_t map0[GW_N_CFP], map1[GW_N_CFP], map2[GW_N_CFP];

    gw_core_build_slotmap(&c, 0, map0);
    for (int s = 0; s < GW_N_CFP; s++) CHECK(map0[s] == phase0_addrs[s]);

    gw_core_build_slotmap(&c, 1, map1);
    CHECK(map1[0] == g_phase1.short_addr);
    for (int s = 1; s < GW_N_CFP; s++) CHECK(map1[s] == UWB_FRAME_ADDR_BCAST);

    /* Phase 2 was never touched -- must still be all-broadcast after the
     * ticks above, proving they don't perturb occupancy in any phase. */
    gw_core_build_slotmap(&c, 2, map2);
    for (int s = 0; s < GW_N_CFP; s++) CHECK(map2[s] == UWB_FRAME_ADDR_BCAST);
}

/* Defensive-guard test for gw_core_build_slotmap()'s own bounds check: a
 * phase at/past GW_CYCLE_C must come back all-broadcast, not index outside
 * seats[][] -- even when a live seat exists at phase 0. See the function's
 * header comment: the only real caller always derives phase from
 * frame_counter % GW_CYCLE_C, so this path is a backstop, not a live one. */
static void test_slotmap_out_of_range_phase_is_safe(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    uint16_t map[GW_N_CFP];

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    gw_core_build_slotmap(&c, (uint8_t)GW_CYCLE_C, map);
    for (int i = 0; i < GW_N_CFP; i++) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);
}

static void test_addr_pool_skips_live_seats(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant ga, gb, gc;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);
    CHECK(gw_core_join(&c, eui_a, 1, &ga));
    CHECK(gw_core_join(&c, eui_b, 1, &gb));

    /* Free the FIRST seat, then join a third tag: the pool has moved on, so
     * the new tag must not be handed an address a live seat still holds. */
    gw_core_release(&c, ga.short_addr);
    uint8_t eui_c[UWB_FRAME_EUI_LEN];
    mk_eui(eui_c, 0x30);
    CHECK(gw_core_join(&c, eui_c, 1, &gc));
    CHECK(gc.short_addr != gb.short_addr);
}

static void test_find_eui_by_addr(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(eui, 0x42);

    CHECK(gw_core_join(&c, eui, 1, &g));

    uint8_t out[UWB_FRAME_EUI_LEN];
    CHECK(gw_core_find_eui(&c, g.short_addr, out));
    CHECK(memcmp(out, eui, UWB_FRAME_EUI_LEN) == 0);

    /* An address with no live seat must fail cleanly, not read garbage. */
    uint8_t out2[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, (uint16_t)(g.short_addr + 999), out2));

    /* Address 0 is never a valid seat (0 means "free" in struct gw_seat). */
    uint8_t out3[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, 0, out3));
}

/* The motivating case for the tag_id fallback path (see
 * uwb_gateway.c's dispatch() and CLAUDE.md's "Stable tag identity" entry):
 * a tag joins, its lease ages all the way to expiry, and gw_core_find_eui()
 * must then report false for its former address -- not stale data, not a
 * crash. Mirrors test_lease_expiry()'s tick pattern. */
static void test_find_eui_after_lease_expiry(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    uint8_t out[UWB_FRAME_EUI_LEN];
    CHECK(gw_core_find_eui(&c, g.short_addr, out));    /* live: resolves */

    for (unsigned i = 0; i < GW_LEASE_SF; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.phase][g.slot_index].short_addr == 0);      /* reclaimed */

    uint8_t out2[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, g.short_addr, out2));  /* expired: false */
}

/* Second-seat selectivity: each address must resolve to its OWN eui, never
 * the other tag's -- a lookup that silently returned a neighboring seat's
 * EUI would be worse than a clean miss. */
static void test_find_eui_selectivity(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant ga, gb;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);
    CHECK(gw_core_join(&c, eui_a, 1, &ga));
    CHECK(gw_core_join(&c, eui_b, 1, &gb));

    uint8_t out_a[UWB_FRAME_EUI_LEN], out_b[UWB_FRAME_EUI_LEN];
    CHECK(gw_core_find_eui(&c, ga.short_addr, out_a));
    CHECK(gw_core_find_eui(&c, gb.short_addr, out_b));

    CHECK(memcmp(out_a, eui_a, UWB_FRAME_EUI_LEN) == 0);
    CHECK(memcmp(out_b, eui_b, UWB_FRAME_EUI_LEN) == 0);
    CHECK(memcmp(out_a, eui_b, UWB_FRAME_EUI_LEN) != 0);
    CHECK(memcmp(out_b, eui_a, UWB_FRAME_EUI_LEN) != 0);
}

int main(void)
{
    test_init();
    test_join_fills_slots_in_order();
    test_join_advances_to_next_phase_when_full();
    test_rejoin_is_idempotent();
    test_network_full();
    test_keepalive();
    test_lease_expiry();
    test_lease_expiry_same_regardless_of_phase();
    test_release();
    test_slotmap();
    test_slotmap_isolated_per_phase_across_ticks();
    test_slotmap_out_of_range_phase_is_safe();
    test_addr_pool_skips_live_seats();
    test_find_eui_by_addr();
    test_find_eui_after_lease_expiry();
    test_find_eui_selectivity();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

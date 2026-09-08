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

/* ---- multi-phase helpers ------------------------------------------------ */

/* Every cell holding `addr`, read straight out of the table rather than
 * trusting the grant that produced it: phase bitmask returned, cell count and
 * shared CFP slot out-params. A tag's cells must all share one slot
 * (gw_core.h invariant 1), so *slot_out is the lowest phase's slot and
 * test_one_slot_per_tag below is what actually pins the invariant. */
static uint16_t mask_of(const struct gw_core_ctx *c, uint16_t addr,
                        int *slot_out, int *cells_out)
{
    uint16_t mask = 0;
    int cells = 0, slot = -1;

    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr != addr) continue;
            mask |= (uint16_t)(1u << p);
            if (slot < 0) slot = s;
            cells++;
        }
    }
    if (slot_out) *slot_out = slot;
    if (cells_out) *cells_out = cells;
    return mask;
}

/* The n phases of a grant must sit at a constant GW_CYCLE_C/n spacing.
 * Rebuilt from first principles from the mask's own lowest bit instead of
 * compared against a literal, so this asserts the property the brief asks for
 * at any GW_CYCLE_C rather than at 16 specifically. */
static int is_even_spread(uint16_t mask, int n)
{
    int stride, base = -1;
    uint16_t expect = 0;

    if (n <= 0 || (GW_CYCLE_C % n) != 0) return 0;
    if (gw_phase_count(mask) != (uint8_t)n) return 0;
    stride = GW_CYCLE_C / n;
    for (int p = 0; p < GW_CYCLE_C && base < 0; p++) {
        if (mask & (uint16_t)(1u << p)) base = p;
    }
    if (base < 0 || base >= stride) return 0;   /* not a canonical base */
    for (int k = 0; k < n; k++) expect |= (uint16_t)(1u << (base + k * stride));
    return mask == expect;
}

/* The layout the brief explicitly rules out: n phases in a row. Four of those
 * give a mover four fixes inside 0.8 s and then 2.4 s of nothing, worse for
 * its EKF than an even 1.25 Hz -- so every spread assertion below also checks
 * the mask is NOT this. */
static uint16_t consecutive_mask(uint16_t mask, int n)
{
    uint16_t out = 0;
    int base = -1;

    for (int p = 0; p < GW_CYCLE_C && base < 0; p++) {
        if (mask & (uint16_t)(1u << p)) base = p;
    }
    if (base < 0) return 0;
    for (int k = 0; k < n && base + k < GW_CYCLE_C; k++) {
        out |= (uint16_t)(1u << (base + k));
    }
    return out;
}

/* Park a synthetic occupant in one exact cell. The allocator reads nothing but
 * short_addr to decide whether a cell is claimable, so this is enough to
 * create contention at a (phase, slot) that JOIN's base-major fill cannot be
 * steered to. */
static void occupy(struct gw_core_ctx *c, int phase, int slot, uint16_t addr)
{
    c->seats[phase][slot].short_addr      = addr;
    c->seats[phase][slot].lease_remaining = GW_LEASE_SF;
}

/* Join `n` single-phase filler tags. JOIN's placement is base-major, so this
 * fills phase 0's GW_N_CFP slots, then phase 1's, and so on -- the way every
 * contention test below sets up "phases 0..k are full". Seeds start at
 * `seed0`; callers keep those ranges disjoint from their own tags' EUIs. */
static void join_fillers(struct gw_core_ctx *c, int n, int seed0)
{
    for (int i = 0; i < n; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;

        mk_eui(eui, (uint8_t)(seed0 + i));
        CHECK(gw_core_join(c, eui, GW_TIER_IDLE, &g));
    }
}

/* Total live cells, for the conservation checks: an allocation must never
 * change the count for anyone but the tag being allocated. */
static int occupied_cells(const struct gw_core_ctx *c)
{
    int n = 0;

    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            if (c->seats[p][s].short_addr != 0) n++;
        }
    }
    return n;
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

    gw_core_keepalive(&c, g.short_addr, 3, NULL);
    CHECK(c.seats[g.phase][g.slot_index].lease_remaining == GW_LEASE_SF);
    CHECK(c.seats[g.phase][g.slot_index].tier == 3);

    /* An unknown address must not disturb anything. */
    gw_core_keepalive(&c, 0xBEEF, 1, NULL);
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

/* ---- Task 13: phase allocation by tier --------------------------------- */

/* gw_phase_count() is what every assertion below counts phases with, so pin it
 * first -- an off-by-one there would make the whole file agree with itself and
 * with nothing else. */
static void test_phase_count(void)
{
    CHECK(gw_phase_count(0x0000u) == 0);
    CHECK(gw_phase_count(0x0001u) == 1);
    CHECK(gw_phase_count(0x8000u) == 1);
    CHECK(gw_phase_count(0x5555u) == 8);   /* every even phase */
    CHECK(gw_phase_count(0x1111u) == 4);
    CHECK(gw_phase_count(0xFFFFu) == 16);
}

/* A JOIN is one phase whatever the tag asked for -- a tag that has done
 * nothing but broadcast a JOIN has not proved it will stay, and the phases its
 * tier earns are handed out by the first KEEPALIVE. */
static void test_join_grants_one_phase_at_every_tier(void)
{
    struct gw_core_ctx c;
    const uint8_t tiers[] = { GW_TIER_IDLE, GW_TIER_SLOW, GW_TIER_FAST, 99 };

    gw_core_init(&c);

    for (unsigned i = 0; i < sizeof(tiers) / sizeof(tiers[0]); i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        int cells = 0, slot = -1;

        mk_eui(eui, (uint8_t)(0x40 + i));
        CHECK(gw_core_join(&c, eui, tiers[i], &g));
        CHECK(gw_phase_count(g.phase_mask) == 1);
        CHECK(g.phase_mask == (uint16_t)(1u << g.phase));
        CHECK(mask_of(&c, g.short_addr, &slot, &cells) == g.phase_mask);
        CHECK(cells == 1);
        CHECK(slot == g.slot_index);
    }
}

/* The brief's headline: a FAST tag's phases spread evenly around the cycle.
 * On an empty table the ceiling (GW_PHASES_MAX_FAST == 8) is available, so
 * "8 when the budget allows" resolves to 8 -- at stride GW_CYCLE_C/8 == 2,
 * i.e. every even phase, NOT phases 0..7. */
static void test_keepalive_spreads_8_phases_evenly(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    int cells = 0, slot = -1;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_phase_count(g.phase_mask) == 1);

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(k.short_addr == g.short_addr);
    CHECK(gw_phase_count(k.phase_mask) == 8);
    CHECK(is_even_spread(k.phase_mask, 8));
    CHECK(k.phase_mask != consecutive_mask(k.phase_mask, 8));
    CHECK(k.phase == 0);                       /* base is the lowest phase */
    CHECK(k.phase_mask == 0x5555u);            /* phases 0,2,4,...,14 */

    /* And the table agrees with the grant, at ONE slot index. */
    CHECK(mask_of(&c, g.short_addr, &slot, &cells) == k.phase_mask);
    CHECK(cells == 8);
    CHECK(slot == k.slot_index);
    CHECK(k.slot_index == g.slot_index);       /* grew in place */
}

/* SLOW's ceiling is 2, so a SLOW tag gets exactly two phases half a cycle
 * apart -- 1.6 s between fixes rather than two fixes 200 ms apart. */
static void test_keepalive_spreads_2_phases_evenly(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    int cells = 0, slot = -1;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_SLOW, &g));

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_SLOW, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 2);
    CHECK(is_even_spread(k.phase_mask, 2));
    CHECK(k.phase_mask != consecutive_mask(k.phase_mask, 2));
    CHECK(k.phase_mask == 0x0101u);            /* phases 0 and 8 */
    CHECK(mask_of(&c, g.short_addr, &slot, &cells) == k.phase_mask);
    CHECK(cells == 2);
    CHECK(slot == k.slot_index);
}

/* Four phases -- the third size the brief names -- is not any tier's ceiling,
 * it is what the 8 -> 4 -> 2 -> 1 ladder falls back to. Reached by filling
 * phases 0 and 1: every 8-phase candidate has base 0 or 1 (stride 2) and so
 * needs a cell in one of them, while base 2 at stride 4 does not. This is
 * "FAST 4" and the budget test in one shot: the same tag that got 8 on an
 * empty table gets 4 here, without touching a single filler. */
static void test_keepalive_falls_back_to_4_phases(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    int cells = 0, slot = -1, before;

    gw_core_init(&c);
    join_fillers(&c, 2 * GW_N_CFP, 0x80);      /* phases 0 and 1 full */

    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(g.phase == 2 && g.slot_index == 0);  /* base-major: next free cell */

    before = occupied_cells(&c);
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 4);
    CHECK(is_even_spread(k.phase_mask, 4));
    CHECK(k.phase_mask != consecutive_mask(k.phase_mask, 4));
    CHECK(k.phase_mask == 0x4444u);            /* phases 2,6,10,14 */
    CHECK(k.phase == 2);
    CHECK(mask_of(&c, g.short_addr, &slot, &cells) == k.phase_mask);
    CHECK(cells == 4 && slot == 0);

    /* Exactly three cells were added and none taken from anyone. */
    CHECK(occupied_cells(&c) == before + 3);
    for (int s = 0; s < GW_N_CFP; s++) {
        CHECK(c.seats[0][s].short_addr != 0 && c.seats[0][s].short_addr != g.short_addr);
        CHECK(c.seats[1][s].short_addr != 0 && c.seats[1][s].short_addr != g.short_addr);
    }
}

/* The ladder's middle rung, to show it steps rather than jumping straight to
 * one phase. Phases 0..3 full blocks every 8- and 4-phase candidate (their
 * bases are 0..1 and 0..3), leaving base 4 at stride 8. */
static void test_keepalive_falls_back_to_2_phases(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;

    gw_core_init(&c);
    join_fillers(&c, 4 * GW_N_CFP, 0x80);      /* phases 0..3 full */

    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(g.phase == 4 && g.slot_index == 0);

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 2);
    CHECK(is_even_spread(k.phase_mask, 2));
    CHECK(k.phase_mask == 0x1010u);            /* phases 4 and 12 */
}

/* The non-stranding rule at maximum pressure. Every cell in the table is a
 * separate single-phase tag, then one of them asks for FAST: the ladder must
 * walk 8 -> 4 -> 2 -> 1 and land back on the one cell that tag already owns,
 * because that is the only cell candidate_ok() will accept for it. Nothing may
 * be evicted, so the occupancy and every other tag's footprint are unchanged
 * -- a FAST tag taking a stationary tag's last phase would show up here as a
 * tag whose cell count dropped to zero without a gw_core_release(). */
static void test_contention_never_strands_any_tag(void)
{
    struct gw_core_ctx c;
    int total = (int)(GW_CYCLE_C * GW_N_CFP);
    uint16_t addrs[GW_CYCLE_C * GW_N_CFP];
    struct gw_grant k;

    gw_core_init(&c);
    for (int i = 0; i < total; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;

        mk_eui(eui, (uint8_t)i);
        CHECK(gw_core_join(&c, eui, GW_TIER_IDLE, &g));
        addrs[i] = g.short_addr;
    }
    CHECK(occupied_cells(&c) == total);

    /* The tag in the middle of the table turns into a mover. */
    uint16_t mover = addrs[total / 2];
    int mover_slot_before = -1;
    uint16_t mover_mask_before = mask_of(&c, mover, &mover_slot_before, NULL);

    CHECK(gw_core_keepalive(&c, mover, GW_TIER_FAST, &k) == GW_KEEPALIVE_SAME);
    CHECK(gw_phase_count(k.phase_mask) == 1);
    CHECK(k.phase_mask == mover_mask_before);
    CHECK(k.slot_index == mover_slot_before);

    /* Every tag, mover included, still holds exactly its one cell. */
    CHECK(occupied_cells(&c) == total);
    for (int i = 0; i < total; i++) {
        int cells = 0;

        (void)mask_of(&c, addrs[i], NULL, &cells);
        CHECK(cells == 1);
    }
}

/* Partial contention: a stationary tag's only cell sits inside the phase set a
 * FAST tag would otherwise be granted, and must survive intact. Phases 0 and 1
 * are full, so FAST would take base 2 / stride 4 (phases 2,6,10,14) -- park
 * the stationary tag at (6, 0) and the 4-phase candidate at slot 0 is dead. */
static void test_fast_tag_cannot_take_a_stationary_tags_only_phase(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    const uint16_t sitter = 0x0AAA;

    gw_core_init(&c);
    join_fillers(&c, 2 * GW_N_CFP, 0x80);      /* phases 0 and 1 full */
    occupy(&c, 6, 0, sitter);

    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(g.phase == 2 && g.slot_index == 0);

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);

    /* The sitter is untouched and still holds exactly one cell. */
    int sitter_cells = 0;
    CHECK(mask_of(&c, sitter, NULL, &sitter_cells) == (uint16_t)(1u << 6));
    CHECK(sitter_cells == 1);
    CHECK(c.seats[6][0].short_addr == sitter);

    /* The mover still got four evenly spread phases -- it skipped base 2
     * (phases 2,6,10,14), whose slot-0 set needs the sitter's cell, and took
     * base 3 (phases 3,7,11,15) instead. It gave up nothing for the sitter's
     * sake: it neither shrank nor changed CFP slot, it just used a different
     * base within the same slot. */
    CHECK(gw_phase_count(k.phase_mask) == 4);
    CHECK(is_even_spread(k.phase_mask, 4));
    CHECK(k.phase_mask != consecutive_mask(k.phase_mask, 4));
    CHECK(k.phase == 3);
    CHECK(k.slot_index == 0);
    CHECK(k.phase_mask == 0x8888u);            /* phases 3,7,11,15 */

    /* The property that generalises past this exact layout: whatever set it
     * got, it does not contain the cell the sitter holds. */
    CHECK(!(k.slot_index == 0 && (k.phase_mask & (uint16_t)(1u << 6))));
}

/* The other move case: the tag's own CFP slot can no longer hold a set of the
 * size it now wants, so the allocation moves it to another slot instead of
 * dropping it to fewer phases. Slot 0's every 2-phase base (b, b+8 for
 * b = 0..7) is blocked by a synthetic occupant, while slot 1 is wide open. */
static void test_regrant_moves_slot_rather_than_shrink(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_SLOW, &g));
    CHECK(g.phase == 0 && g.slot_index == 0);

    /* Kill base 0 (needs (8,0)) and bases 1..7 (need (1,0)..(7,0)). */
    occupy(&c, 8, 0, 0x0B00);
    for (int p = 1; p <= 7; p++) occupy(&c, p, 0, (uint16_t)(0x0B00 + p));

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_SLOW, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 2);
    CHECK(is_even_spread(k.phase_mask, 2));
    CHECK(k.slot_index == 1);                  /* moved off slot 0 */
    CHECK(k.phase_mask == 0x0101u);            /* phases 0 and 8, at slot 1 */
    CHECK(c.seats[0][0].short_addr == 0);      /* old cell handed back */

    /* Not one of the synthetic occupants was disturbed. */
    CHECK(c.seats[8][0].short_addr == 0x0B00);
    for (int p = 1; p <= 7; p++) {
        CHECK(c.seats[p][0].short_addr == (uint16_t)(0x0B00 + p));
    }
}

/* A tier drop must hand the excess cells BACK, not merely stop counting them.
 * Proved the only way that really settles it: fill the table completely around
 * a FAST tag holding 8 phases, confirm a JOIN is refused, drop that tag to
 * IDLE, and then show exactly 7 further JOINs succeed -- landing in the cells
 * it gave up -- before the table is full again. */
static void test_tier_drop_returns_phases_to_the_pool(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k, extra;
    int total = (int)(GW_CYCLE_C * GW_N_CFP);
    int seed = 0x00;

    gw_core_init(&c);
    mk_eui(eui, 0xF0);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);
    CHECK(k.phase_mask == 0x5555u);

    /* Fill every remaining cell, one single-phase tag each. */
    for (int i = 0; i < total - 8; i++) {
        uint8_t f[UWB_FRAME_EUI_LEN];
        struct gw_grant fg;

        mk_eui(f, (uint8_t)(seed + i));
        CHECK(gw_core_join(&c, f, GW_TIER_IDLE, &fg));
    }
    CHECK(occupied_cells(&c) == total);
    mk_eui(eui, 0xE0);
    CHECK(!gw_core_join(&c, eui, GW_TIER_IDLE, &extra));   /* full */

    /* The mover goes still: 8 phases -> 1, keeping its base cell. */
    struct gw_grant idle;
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_IDLE, &idle)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(idle.phase_mask) == 1);
    CHECK(idle.phase_mask == 0x0001u);
    CHECK(idle.slot_index == k.slot_index);
    CHECK(occupied_cells(&c) == total - 7);

    /* The seven released cells are genuinely back in the pool: seven JOINs
     * succeed, the eighth is refused, and the first of them lands on a cell
     * the mover used to hold. */
    for (int i = 0; i < 7; i++) {
        uint8_t f[UWB_FRAME_EUI_LEN];
        struct gw_grant fg;

        mk_eui(f, (uint8_t)(0xE1 + i));
        CHECK(gw_core_join(&c, f, GW_TIER_IDLE, &fg));
        CHECK(k.phase_mask & (uint16_t)(1u << fg.phase));  /* a released cell */
        CHECK(fg.slot_index == k.slot_index);
    }
    CHECK(occupied_cells(&c) == total);
    mk_eui(eui, 0xE9);
    CHECK(!gw_core_join(&c, eui, GW_TIER_IDLE, &extra));
}

/* gw_core_release() must free EVERY cell of a multi-phase tag. Clearing only
 * the first one found would leave seven cells published in seven phases' slot
 * maps under an address granted to nobody, each aging out on its own lease. */
static void test_release_frees_every_phase(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    int cells = -1;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);
    CHECK(occupied_cells(&c) == 8);

    gw_core_release(&c, g.short_addr);
    CHECK(mask_of(&c, g.short_addr, NULL, &cells) == 0);
    CHECK(cells == 0);
    CHECK(occupied_cells(&c) == 0);            /* the whole table, not just one */

    /* And the address no longer resolves to an EUI anywhere. */
    uint8_t out[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, g.short_addr, out));
}

/* A multi-phase tag's lease must expire as ONE unit after exactly GW_LEASE_SF
 * ticks -- gw_core_superframe_tick() ages each of its 8 cells independently
 * and they stay in step only because claim() set them all in the same call. A
 * partial expiry would leave a tag ranging in some of its phases and silently
 * absent from the rest. */
static void test_multiphase_lease_expires_as_one_unit(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;
    int cells = -1;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);

    for (unsigned i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(mask_of(&c, g.short_addr, NULL, &cells) == k.phase_mask);
    CHECK(cells == 8);                         /* all eight still held */

    gw_core_superframe_tick(&c);
    CHECK(mask_of(&c, g.short_addr, NULL, &cells) == 0);
    CHECK(cells == 0);                         /* all eight reclaimed at once */
    CHECK(c.frame_counter == GW_LEASE_SF);
}

/* A steady-state KEEPALIVE must not shuffle the tag. Its phase set is already
 * the right size, so the second and third calls report SAME and leave the mask
 * and slot exactly where they were -- otherwise a FAST tag would be moved to a
 * numerically lower base every lease period, changing which superframes it
 * ranges in for no reason. */
static void test_keepalive_is_stable_once_at_tier(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k1, k2, k3;

    gw_core_init(&c);
    join_fillers(&c, GW_N_CFP, 0x80);          /* push the mover off base 0 */
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(g.phase == 1);

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k1)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k1.phase_mask) == 8);
    CHECK(k1.phase_mask == 0xAAAAu);           /* odd phases: base 1, stride 2 */

    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k2)
          == GW_KEEPALIVE_SAME);
    CHECK(k2.phase_mask == k1.phase_mask);
    CHECK(k2.slot_index == k1.slot_index);

    /* Also stable across ticks that do not expire the lease. */
    for (unsigned i = 0; i < 5; i++) gw_core_superframe_tick(&c);
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k3)
          == GW_KEEPALIVE_SAME);
    CHECK(k3.phase_mask == k1.phase_mask);
    CHECK(c.seats[k3.phase][k3.slot_index].lease_remaining == GW_LEASE_SF);
}

/* KEEPALIVE from an address no seat holds must change nothing at all and say
 * so, rather than creating a seat or silently succeeding. */
static void test_keepalive_unknown_addr_is_reported(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));

    CHECK(gw_core_keepalive(&c, 0xBEEF, GW_TIER_FAST, &k) == GW_KEEPALIVE_UNKNOWN);
    CHECK(occupied_cells(&c) == 1);
    CHECK(gw_core_keepalive(&c, 0, GW_TIER_FAST, &k) == GW_KEEPALIVE_UNKNOWN);
    CHECK(occupied_cells(&c) == 1);
}

/* An unrecognised tier byte -- the tag's enum only defines 0..2 and this
 * arrives straight off the air -- must be treated as IDLE. The opposite
 * default would let one malformed frame claim eight phases. */
static void test_unknown_tier_gets_idle_phase_count(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);

    CHECK(gw_core_keepalive(&c, g.short_addr, 0xFF, &k) == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 1);
    CHECK(k.tier == 0xFF);                     /* recorded verbatim */
    CHECK(occupied_cells(&c) == 1);
}

/* A re-JOIN from a tag that already holds 8 phases is a lost GRANT, not a
 * demotion: it must come back with the same address and the same phase set,
 * not be reset to the one phase a fresh JOIN would get. */
static void test_rejoin_preserves_a_multiphase_grant(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k, rj;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);

    for (unsigned i = 0; i < 10; i++) gw_core_superframe_tick(&c);

    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &rj));
    CHECK(rj.short_addr == g.short_addr);
    CHECK(rj.phase_mask == k.phase_mask);
    CHECK(rj.slot_index == k.slot_index);
    CHECK(occupied_cells(&c) == 8);            /* no second seat consumed */

    /* The lease is refreshed on every one of its cells, not just the first. */
    for (int p = 0; p < GW_CYCLE_C; p++) {
        if (rj.phase_mask & (uint16_t)(1u << p)) {
            CHECK(c.seats[p][rj.slot_index].lease_remaining == GW_LEASE_SF);
            CHECK(c.seats[p][rj.slot_index].short_addr == g.short_addr);
        }
    }
}

/* Invariant 1 (gw_core.h): all of one tag's cells share a CFP slot index, so
 * a grant is fully described by one slot plus the phase mask -- which is what
 * a separate, deferred task puts on the wire. Checked across a table churned
 * by joins, growth, a tier drop and a release rather than on one tag. */
static void test_one_slot_per_tag(void)
{
    struct gw_core_ctx c;
    uint16_t movers[6];
    struct gw_grant g;

    gw_core_init(&c);
    join_fillers(&c, GW_N_CFP + 3, 0x80);

    for (unsigned i = 0; i < sizeof(movers) / sizeof(movers[0]); i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];

        mk_eui(eui, (uint8_t)(0x20 + i));
        CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
        movers[i] = g.short_addr;
        CHECK(gw_core_keepalive(&c, movers[i], GW_TIER_FAST, NULL)
              != GW_KEEPALIVE_UNKNOWN);
    }
    CHECK(gw_core_keepalive(&c, movers[2], GW_TIER_SLOW, NULL)
          != GW_KEEPALIVE_UNKNOWN);
    gw_core_release(&c, movers[4]);

    /* Every live address in the table occupies exactly one column. */
    for (int p = 0; p < GW_CYCLE_C; p++) {
        for (int s = 0; s < GW_N_CFP; s++) {
            uint16_t addr = c.seats[p][s].short_addr;
            int slot = -1, cells = 0;
            uint16_t m;

            if (addr == 0) continue;
            m = mask_of(&c, addr, &slot, &cells);
            CHECK(slot == s);
            CHECK(gw_phase_count(m) == (uint8_t)cells);
            CHECK(is_even_spread(m, cells));
        }
    }
}

/* The beacon side of a multi-phase grant: the tag's address appears in the
 * slot map of exactly the phases it holds and in none of the others. This is
 * the only channel that tells the tag when to range today, so it is what makes
 * an 8-phase grant actually produce 8 fixes per cycle. */
static void test_slotmap_reflects_the_whole_phase_mask(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, k;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));
    CHECK(gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &k)
          == GW_KEEPALIVE_REPHASED);
    CHECK(gw_phase_count(k.phase_mask) == 8);

    for (int p = 0; p < GW_CYCLE_C; p++) {
        uint16_t map[GW_N_CFP];
        int found = -1;

        gw_core_build_slotmap(&c, (uint8_t)p, map);
        /* Open-coded rather than via uwb_frame_beacon_find_addr(): this suite
         * links gw_core.c alone, not the frame module. */
        for (int s = 0; s < GW_N_CFP; s++) {
            if (map[s] == k.short_addr) { found = s; break; }
        }
        if (k.phase_mask & (uint16_t)(1u << p)) {
            CHECK(found == k.slot_index);
        } else {
            CHECK(found < 0);
        }
    }
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

    /* Task 13: phase allocation by tier. */
    test_phase_count();
    test_join_grants_one_phase_at_every_tier();
    test_keepalive_spreads_8_phases_evenly();
    test_keepalive_spreads_2_phases_evenly();
    test_keepalive_falls_back_to_4_phases();
    test_keepalive_falls_back_to_2_phases();
    test_contention_never_strands_any_tag();
    test_fast_tag_cannot_take_a_stationary_tags_only_phase();
    test_regrant_moves_slot_rather_than_shrink();
    test_tier_drop_returns_phases_to_the_pool();
    test_release_frees_every_phase();
    test_multiphase_lease_expires_as_one_unit();
    test_keepalive_is_stable_once_at_tier();
    test_keepalive_unknown_addr_is_reported();
    test_unknown_tier_gets_idle_phase_count();
    test_rejoin_preserves_a_multiphase_grant();
    test_one_slot_per_tag();
    test_slotmap_reflects_the_whole_phase_mask();

    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

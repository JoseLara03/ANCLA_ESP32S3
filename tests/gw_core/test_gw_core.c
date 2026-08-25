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
    for (int i = 0; i < GW_N_CFP; i++) CHECK(c.seats[i].short_addr == 0);
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
        CHECK(g.seat_id == i);
        CHECK(g.short_addr == (uint16_t)(GW_TAG_ADDR_BASE + i));
        CHECK(g.lease == GW_LEASE_SF);
        CHECK(g.tier == 1);
    }
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
    CHECK(g3.seat_id == g1.seat_id);
    CHECK(g3.short_addr == g1.short_addr);
    CHECK(g3.tier == 2);          /* but the tier is updated */
}

/* The 12th tag used to be refused, because the seat table was indexed by CFP
 * slot. It is not any more -- that is the whole point of the seat/schedule
 * split -- so this test now asserts the OPPOSITE of what it once did. */
static void test_twelfth_tag_is_admitted(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    /* IDLE so admission is limited by seats, not airtime. */
    for (int i = 0; i < GW_N_CFP + 1; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        CHECK(gw_core_join(&c, eui, GW_TIER_IDLE, &g));
        CHECK(g.seat_id == i);
    }
    CHECK(gw_core_seats_used(&c) == GW_N_CFP + 1);
}

/* The real seat ceiling, and the 100-tag product target inside it. */
static void test_hundred_tags_fit(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (int i = 0; i < 100; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        /* Distinct EUIs: mk_eui is a ramp, so vary the seed widely enough
         * that no two tags collide. */
        eui[7] = (uint8_t)i;
        CHECK(gw_core_join(&c, eui, GW_TIER_IDLE, &g));
    }
    CHECK(gw_core_seats_used(&c) == 100);
    CHECK(gw_core_cost_used(&c) == 100 * gw_core_tier_cost(GW_TIER_IDLE));
    CHECK(gw_core_cost_used(&c) <= GW_SCHED_CAPACITY);
}

static void test_seat_table_full(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (unsigned int i = 0; i < GW_MAX_SEATS; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        eui[7] = (uint8_t)i;
        eui[6] = (uint8_t)(i >> 8);
        CHECK(gw_core_join(&c, eui, GW_TIER_IDLE, &g));
    }

    uint8_t extra[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(extra, 0xF0);
    extra[7] = 0xFE;
    CHECK(!gw_core_join(&c, extra, GW_TIER_IDLE, &g));
}

static void test_keepalive(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (int i = 0; i < 10; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.seat_id].lease_remaining == GW_LEASE_SF - 10);

    /* Tier 3 does not exist. It is normalized to IDLE -- the CHEAPEST tier,
     * never the fastest, so a corrupt or future-versioned request cannot be
     * used to claim airtime. This test previously asserted the raw 3 was
     * stored verbatim, which made the stored tier meaningless as a rate. */
    gw_core_keepalive(&c, g.short_addr, 3, NULL);
    CHECK(c.seats[g.seat_id].lease_remaining == GW_LEASE_SF);
    CHECK(c.seats[g.seat_id].tier == GW_TIER_IDLE);

    /* An unknown address must not disturb anything. */
    gw_core_keepalive(&c, 0xBEEF, 1, NULL);
    CHECK(c.seats[g.seat_id].tier == GW_TIER_IDLE);
}

static void test_lease_expiry(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (unsigned int i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.seat_id].short_addr != 0);    /* still held */

    gw_core_superframe_tick(&c);
    CHECK(c.seats[g.seat_id].short_addr == 0);    /* reclaimed */
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
    CHECK(c.seats[g.seat_id].short_addr == 0);

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
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));

    /* Before the first tick the map is empty: joining grants a SEAT, and the
     * schedule that hands out SLOTS is recomputed once per superframe. */
    gw_core_build_slotmap(&c, map);
    for (int i = 0; i < GW_N_CFP; i++) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);

    /* After a tick the sole tag takes slot 0 -- note NOT necessarily the slot
     * its seat id would imply; seat id and slot index are now unrelated. */
    gw_core_superframe_tick(&c);
    gw_core_build_slotmap(&c, map);
    CHECK(map[0] == g.short_addr);
    for (int i = 1; i < GW_N_CFP; i++) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);
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

    for (unsigned int i = 0; i < GW_LEASE_SF; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.seat_id].short_addr == 0);      /* reclaimed */

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


/* ---- Scheduler: seat != slot ------------------------------------------- */

/* How many of the next `n` superframes name `addr` in the slot map. */
static unsigned int count_grants(struct gw_core_ctx *c, uint16_t addr,
                                 unsigned int n)
{
    unsigned int seen = 0;

    for (unsigned int t = 0; t < n; t++) {
        uint16_t map[GW_N_CFP];

        gw_core_superframe_tick(c);
        gw_core_build_slotmap(c, map);
        for (unsigned int i = 0; i < GW_N_CFP; i++)
            if (map[i] == addr) { seen++; break; }
    }
    return seen;
}

/* Each tier gets the cadence the MAC contract promises. This is the behaviour
 * the old slot-indexed table could not express at all: every seat was named in
 * every superframe regardless of tier, which is why lowering a tier saved tag
 * battery and no network capacity. */
static void test_tier_cadence(void)
{
    const struct { uint8_t tier; unsigned int expect; } cases[] = {
        { GW_TIER_FAST, 25u },      /* every superframe */
        { GW_TIER_SLOW, 5u },       /* every 5th */
        { GW_TIER_IDLE, 1u },       /* every 25th */
    };

    for (unsigned int k = 0; k < 3; k++) {
        struct gw_core_ctx c;
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;

        gw_core_init(&c);
        mk_eui(eui, 0x20);
        CHECK(gw_core_join(&c, eui, cases[k].tier, &g));
        CHECK(g.tier == cases[k].tier);
        /* 25 superframes, deliberately inside GW_LEASE_SF (50): a 50-tick
         * window reclaims the seat on its last tick and the FAST count comes
         * back 49, which is the lease working, not the scheduler failing. */
        CHECK(count_grants(&c, g.short_addr, GW_SCHED_WINDOW_SF) ==
              cases[k].expect);
    }
}

/* Presence is guaranteed; RATE is what runs out. A cell filled with FAST
 * requests admits every tag up to the seat table and grants FAST only to as
 * many as GW_SCHED_UPGRADE_POOL affords -- the rest are seated at a lower tier
 * rather than refused. Before the IDLE-floor policy this test asserted the
 * opposite arrangement, where movers took the airtime greedily and later tags
 * were turned away entirely. */
static void test_presence_is_guaranteed_rate_is_not(void)
{
    struct gw_core_ctx c;
    unsigned int admitted = 0, fast = 0;

    gw_core_init(&c);
    for (unsigned int i = 0; i < GW_MAX_SEATS + 8u; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        eui[7] = (uint8_t)i;
        eui[6] = (uint8_t)(i >> 8);
        if (!gw_core_join(&c, eui, GW_TIER_FAST, &g)) continue;
        admitted++;
        if (g.tier == GW_TIER_FAST) fast++;
    }

    /* Every seat filled -- nobody refused for lack of airtime. */
    CHECK(admitted == GW_MAX_SEATS);
    CHECK(gw_core_seats_used(&c) == GW_MAX_SEATS);

    /* FAST is rationed to what the upgrade pool affords, and no further. */
    CHECK(fast == (unsigned int)GW_SCHED_MAX_FAST);
    CHECK(gw_core_upgrades_used(&c) <= GW_SCHED_UPGRADE_POOL);

    /* And the total never exceeds the airtime that physically exists. */
    CHECK(gw_core_cost_used(&c) <= GW_SCHED_CAPACITY);
}

/* The regression the capacity demo caught: 100 tags where the movers join
 * FIRST. Reserving only for current occupancy let those movers take the
 * airtime and refused 54 of the 100; reserving the floor for the whole seat
 * table admits all of them. */
static void test_movers_first_does_not_lock_out_later_tags(void)
{
    struct gw_core_ctx c;
    unsigned int admitted = 0;

    gw_core_init(&c);
    for (unsigned int i = 0; i < 100; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        eui[7] = (uint8_t)i;
        if (gw_core_join(&c, eui, (i < 10u) ? GW_TIER_FAST : GW_TIER_IDLE, &g))
            admitted++;
    }
    CHECK(admitted == 100);
    CHECK(gw_core_cost_used(&c) <= GW_SCHED_CAPACITY);
}

/* The floor arithmetic itself, so the derived figures cannot drift silently. */
static void test_capacity_split(void)
{
    CHECK(GW_SCHED_IDLE_FLOOR == GW_MAX_SEATS);
    CHECK(GW_SCHED_IDLE_FLOOR + GW_SCHED_UPGRADE_POOL == GW_SCHED_CAPACITY);
    CHECK(gw_core_tier_upgrade_cost(GW_TIER_IDLE) == 0u);
    CHECK(gw_core_tier_upgrade_cost(GW_TIER_SLOW) == 4u);
    CHECK(gw_core_tier_upgrade_cost(GW_TIER_FAST) == 24u);

    /* Worst case must fit: every seat at IDLE plus a full upgrade pool. */
    CHECK(GW_MAX_SEATS * 1u +
          GW_SCHED_MAX_FAST * gw_core_tier_upgrade_cost(GW_TIER_FAST)
          <= GW_SCHED_CAPACITY);
}

/* A keepalive must never evict the seat that sent it, even when the tier it
 * asks for cannot be granted. */
static void test_keepalive_upgrade_is_admission_controlled(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (unsigned int i = 0; i < GW_N_CFP; i++) {
        uint8_t e[UWB_FRAME_EUI_LEN];
        struct gw_grant gg;
        mk_eui(e, (uint8_t)i);
        CHECK(gw_core_join(&c, e, GW_TIER_FAST, &gg));
    }

    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g, ka;
    mk_eui(eui, 0xE0);
    CHECK(gw_core_join(&c, eui, GW_TIER_SLOW, &g));
    uint8_t got = g.tier;

    /* Ask to go FAST in a saturated cell. */
    gw_core_keepalive(&c, g.short_addr, GW_TIER_FAST, &ka);
    CHECK(ka.tier != GW_TIER_FAST);
    CHECK(ka.tier <= got);                                  /* not upgraded */
    CHECK(c.seats[g.seat_id].short_addr == g.short_addr);   /* still seated */
    CHECK(c.seats[g.seat_id].lease_remaining == GW_LEASE_SF);
}

/* The headline claim: 100 tags share 11 slots and every one is served. They all
 * join in the same superframe, so this window also covers the thundering-herd
 * drain, which takes ceil(100/11) = 10 superframes. */
static void test_hundred_tags_all_get_served(void)
{
    struct gw_core_ctx c;
    uint16_t addr[100];
    unsigned int seen[100];

    gw_core_init(&c);
    for (unsigned int i = 0; i < 100; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        eui[7] = (uint8_t)i;
        CHECK(gw_core_join(&c, eui, GW_TIER_IDLE, &g));
        addr[i] = g.short_addr;
        seen[i] = 0;
    }

    for (unsigned int t = 0; t < GW_SCHED_WINDOW_SF; t++) {
        uint16_t map[GW_N_CFP];

        gw_core_superframe_tick(&c);
        gw_core_build_slotmap(&c, map);
        for (unsigned int i = 0; i < GW_N_CFP; i++) {
            if (map[i] == UWB_FRAME_ADDR_BCAST) continue;
            for (unsigned int j = 0; j < 100; j++)
                if (addr[j] == map[i]) { seen[j]++; break; }
        }
    }

    for (unsigned int i = 0; i < 100; i++) CHECK(seen[i] >= 1);
}

/* The slot map must never name the same tag twice in one superframe -- the tag
 * would try to transmit in two slots. */
static void test_no_duplicate_in_one_superframe(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (unsigned int i = 0; i < 40; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        eui[7] = (uint8_t)i;
        CHECK(gw_core_join(&c, eui, GW_TIER_SLOW, &g));
    }

    for (unsigned int t = 0; t < 60; t++) {
        uint16_t map[GW_N_CFP];

        gw_core_superframe_tick(&c);
        gw_core_build_slotmap(&c, map);
        for (unsigned int i = 0; i < GW_N_CFP; i++) {
            if (map[i] == UWB_FRAME_ADDR_BCAST) continue;
            for (unsigned int j = i + 1; j < GW_N_CFP; j++)
                CHECK(map[j] != map[i]);
        }
    }
}

/* The frame counter wraps at 2^32. next_due is compared with signed-difference
 * arithmetic, so a seat straddling the wrap keeps being served; a plain
 * unsigned compare would emit one entirely empty slot map instead. */
static void test_schedule_survives_frame_counter_wrap(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x30);
    CHECK(gw_core_join(&c, eui, GW_TIER_FAST, &g));

    /* Park the counter just below the wrap; tick() increments first, so the
     * next tick lands exactly on 0. */
    c.frame_counter = 0xFFFFFFFFu;
    c.seats[g.seat_id].next_due = 0xFFFFFFFFu;

    CHECK(count_grants(&c, g.short_addr, 4) == 4);
    CHECK(c.frame_counter == 3u);
}

/* Only tiers 0..2 exist, and anything else must read as the CHEAPEST rather
 * than the fastest, so a corrupt or future-versioned request cannot be used to
 * claim airtime. */
static void test_tier_normalization(void)
{
    CHECK(gw_core_normalize_tier(GW_TIER_IDLE) == GW_TIER_IDLE);
    CHECK(gw_core_normalize_tier(GW_TIER_FAST) == GW_TIER_FAST);
    CHECK(gw_core_normalize_tier(3) == GW_TIER_IDLE);
    CHECK(gw_core_normalize_tier(0xFF) == GW_TIER_IDLE);

    CHECK(gw_core_tier_period(GW_TIER_FAST) == 1u);
    CHECK(gw_core_tier_period(GW_TIER_SLOW) == 5u);
    CHECK(gw_core_tier_period(GW_TIER_IDLE) == GW_SCHED_WINDOW_SF);

    CHECK(gw_core_tier_cost(GW_TIER_FAST) == GW_SCHED_WINDOW_SF);
    CHECK(gw_core_tier_cost(GW_TIER_SLOW) == 5u);
    CHECK(gw_core_tier_cost(GW_TIER_IDLE) == 1u);
    CHECK(GW_N_CFP * gw_core_tier_cost(GW_TIER_FAST) == GW_SCHED_CAPACITY);
}


/* The lease must sustain the slowest tier's period. A tag participates once per
 * tier period and so sleeps about that long, and it renews at half the lease,
 * so the cadence plus a margin for missed beacons has to fit inside that half.
 *
 * At the old GW_LEASE_SF of 50 this was FALSE for IDLE by a hair -- 25 + 4 is
 * not below 25 -- which put the slowest tier exactly on its own renewal
 * deadline. Pinned in both directions so the reason for 75 cannot be quietly
 * reverted, and so a future tier period that the lease cannot sustain fails
 * here rather than on a bench. */
static void test_lease_sustains_every_tier_period(void)
{
    for (unsigned int t = 0; t < GW_TIER_COUNT; t++) {
        uint32_t p = gw_core_tier_period((uint8_t)t);

        CHECK(p + GW_LEASE_MARGIN_SF < GW_LEASE_SF / 2u);
    }
    CHECK(GW_TIER_PERIOD_MAX == gw_core_tier_period(GW_TIER_IDLE));
    CHECK(!(GW_TIER_PERIOD_MAX + GW_LEASE_MARGIN_SF < 50u / 2u));

    /* And the tag's copy of the lease must agree -- the GRANT carries it. */
    CHECK(GW_LEASE_SF == 75u);
}

int main(void)
{
    test_init();
    test_join_fills_slots_in_order();
    test_rejoin_is_idempotent();
    test_twelfth_tag_is_admitted();
    test_hundred_tags_fit();
    test_seat_table_full();
    test_keepalive();
    test_lease_expiry();
    test_release();
    test_slotmap();
    test_addr_pool_skips_live_seats();
    test_find_eui_by_addr();
    test_find_eui_after_lease_expiry();
    test_find_eui_selectivity();
    test_tier_normalization();
    test_tier_cadence();
    test_capacity_split();
    test_lease_sustains_every_tier_period();
    test_presence_is_guaranteed_rate_is_not();
    test_movers_first_does_not_lock_out_later_tags();
    test_keepalive_upgrade_is_admission_controlled();
    test_hundred_tags_all_get_served();
    test_no_duplicate_in_one_superframe();
    test_schedule_survives_frame_counter_wrap();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

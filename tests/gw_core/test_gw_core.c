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
        CHECK(g.slot_index == i);
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
    CHECK(g3.slot_index == g1.slot_index);
    CHECK(g3.short_addr == g1.short_addr);
    CHECK(g3.tier == 2);          /* but the tier is updated */
}

static void test_network_full(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (int i = 0; i < GW_N_CFP; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        CHECK(gw_core_join(&c, eui, 1, &g));
    }

    uint8_t extra[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(extra, 0xF0);
    CHECK(!gw_core_join(&c, extra, 1, &g));    /* 13th is refused */
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
    CHECK(c.seats[g.slot_index].lease_remaining == GW_LEASE_SF - 10);

    gw_core_keepalive(&c, g.short_addr, 3);
    CHECK(c.seats[g.slot_index].lease_remaining == GW_LEASE_SF);
    CHECK(c.seats[g.slot_index].tier == 3);

    /* An unknown address must not disturb anything. */
    gw_core_keepalive(&c, 0xBEEF, 1);
    CHECK(c.seats[g.slot_index].tier == 3);
}

static void test_lease_expiry(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (int i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].short_addr != 0);    /* still held */

    gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].short_addr == 0);    /* reclaimed */
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
    CHECK(c.seats[g.slot_index].short_addr == 0);

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
    gw_core_build_slotmap(&c, map);

    CHECK(map[g.slot_index] == g.short_addr);
    for (int i = 0; i < GW_N_CFP; i++) {
        if (i != g.slot_index) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);
    }
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

    for (int i = 0; i < GW_LEASE_SF; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].short_addr == 0);      /* reclaimed */

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
    test_rejoin_is_idempotent();
    test_network_full();
    test_keepalive();
    test_lease_expiry();
    test_release();
    test_slotmap();
    test_addr_pool_skips_live_seats();
    test_find_eui_by_addr();
    test_find_eui_after_lease_expiry();
    test_find_eui_selectivity();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

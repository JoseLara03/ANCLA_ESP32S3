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

static void test_tag_id_is_the_low_32_bits_big_endian(void)
{
    /* Frozen by hand, because this value is a contract with the customer
     * platform: it keys its tag records on it. Big-endian read of eui[4..7],
     * the same byte order `apos enum` prints an EUI-64 in. */
    const uint8_t eui[UWB_FRAME_EUI_LEN] = {
        0x70, 0xB8, 0xF6, 0x12, 0x34, 0xAB, 0xCD, 0xEF
    };

    CHECK(gw_core_tag_id(eui) == 0x34ABCDEFu);   /* 883032047 */

    /* The high four bytes must NOT influence it: two tags sharing an OUI or a
     * FICR->DEVICEID[0] must still be told apart, and one whose low half
     * matches must NOT be. */
    const uint8_t same_low[UWB_FRAME_EUI_LEN] = {
        0x00, 0x00, 0x00, 0x00, 0x34, 0xAB, 0xCD, 0xEF
    };
    CHECK(gw_core_tag_id(same_low) == gw_core_tag_id(eui));

    /* Widest value survives the shifts without sign trouble. */
    const uint8_t all_ones[UWB_FRAME_EUI_LEN] = {
        0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF
    };
    CHECK(gw_core_tag_id(all_ones) == 0xFFFFFFFFu);

    CHECK(gw_core_tag_id(NULL) == 0);
}

static void test_eui_by_addr(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant ga, gb;
    const uint8_t *got;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);
    CHECK(gw_core_join(&c, eui_a, 1, &ga));
    CHECK(gw_core_join(&c, eui_b, 1, &gb));

    got = gw_core_eui_by_addr(&c, ga.short_addr);
    CHECK(got != NULL && memcmp(got, eui_a, UWB_FRAME_EUI_LEN) == 0);
    got = gw_core_eui_by_addr(&c, gb.short_addr);
    CHECK(got != NULL && memcmp(got, eui_b, UWB_FRAME_EUI_LEN) == 0);

    /* No seat holds these: the gateway must say so rather than return a
     * neighbouring seat's EUI, which would publish one tag's position under
     * another tag's identity. */
    CHECK(gw_core_eui_by_addr(&c, 0xBEEF) == NULL);
    CHECK(gw_core_eui_by_addr(&c, 0) == NULL);          /* 0 == free marker */

    gw_core_release(&c, ga.short_addr);
    CHECK(gw_core_eui_by_addr(&c, ga.short_addr) == NULL);
}

/* The regression test for the defect this whole mechanism exists to fix: a tag
 * that loses its seat and re-JOINs is issued a DIFFERENT short address, but must
 * keep the SAME platform identity. Publishing the short address as "Tid" made
 * the platform create a second record for the same physical tag and let the
 * first go stale. */
static void test_tag_id_survives_a_lease_expiry_and_rejoin(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant before, after;
    uint32_t id_before, id_after;

    gw_core_init(&c);
    mk_eui(eui, 0x10);

    CHECK(gw_core_join(&c, eui, 1, &before));
    id_before = gw_core_tag_id(gw_core_eui_by_addr(&c, before.short_addr));

    /* Silence for a full lease: no KEEPALIVE, so the seat is reclaimed. */
    for (unsigned i = 0; i < GW_LEASE_SF; i++) gw_core_superframe_tick(&c);
    CHECK(gw_core_eui_by_addr(&c, before.short_addr) == NULL);

    /* Same tag comes back. */
    CHECK(gw_core_join(&c, eui, 1, &after));
    id_after = gw_core_tag_id(gw_core_eui_by_addr(&c, after.short_addr));

    CHECK(after.short_addr != before.short_addr);   /* the lease DID change */
    CHECK(id_after == id_before);                   /* the identity did NOT */
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
    test_tag_id_is_the_low_32_bits_big_endian();
    test_eui_by_addr();
    test_tag_id_survives_a_lease_expiry_and_rejoin();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

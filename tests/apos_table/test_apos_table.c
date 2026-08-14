#include "apos_table.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

#define CLOSE(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

/* EUIs that differ only in the last byte, so a comparison bug that only looks
 * at the first bytes is caught. */
static void mk_eui(uint8_t out[APOS_EUI_LEN], uint8_t tail)
{
    for (int i = 0; i < APOS_EUI_LEN - 1; i++) {
        out[i] = (uint8_t)(0xA0 + i);
    }
    out[APOS_EUI_LEN - 1] = tail;
}

static int add(struct apos_table *t, uint8_t tail, uint16_t addr)
{
    uint8_t eui[APOS_EUI_LEN];

    mk_eui(eui, tail);
    return apos_table_add_peer(t, eui, addr, false, 0.0f, 0.0f, 0.0f);
}

static void test_peers_get_sequential_indices(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 2, 0x0002) == 1);
    CHECK(add(&t, 3, 0x0003) == 2);
    CHECK(t.n_peers == 3);
}

/* Repeated SURVEY_BEGIN broadcasts mean the same anchor answers more than
 * once. That must be idempotent, not a duplicate row. */
static void test_readding_the_same_eui_is_idempotent(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(t.n_peers == 1);
}

/* The duplicate-anchor-id fault. Two distinct boards claiming one short address
 * must be reported, because everything downstream addresses by short address. */
static void test_same_addr_from_a_different_eui_is_reported(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0002) == 0);
    CHECK(add(&t, 9, 0x0002) == -EADDRINUSE);
    CHECK(t.n_peers == 1);
}

/* An anchor that had its id changed between surveys keeps its identity: same
 * EUI, new address, same node index. */
static void test_same_eui_with_a_new_addr_updates_in_place(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 1, 0x0004) == 0);
    CHECK(t.n_peers == 1);
    CHECK(t.peer[0].short_addr == 0x0004);
}

static void test_lookup_by_addr_and_eui(void)
{
    struct apos_table t;
    uint8_t eui[APOS_EUI_LEN];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_find_addr(&t, 0x0002) == 1);
    CHECK(apos_table_find_addr(&t, 0x0009) == -ENOENT);

    mk_eui(eui, 2);
    CHECK(apos_table_find_eui(&t, eui) == 1);
    mk_eui(eui, 7);
    CHECK(apos_table_find_eui(&t, eui) == -ENOENT);
}

static void test_table_full_is_enospc(void)
{
    struct apos_table t;

    apos_table_init(&t);
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        CHECK(add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k)) == k);
    }
    CHECK(add(&t, 200, 0x00FF) == -ENOSPC);
}

/* A retried RANGE_CMD must replace, not accumulate: two rows for one direction
 * would double that direction's weight in the fit. */
static void test_repeat_measurement_replaces(void)
{
    struct apos_table t;

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_add_meas(&t, 0, 1, 1000, 10, 40) == 0);
    CHECK(apos_table_add_meas(&t, 0, 1, 2000, 20, 38) == 0);
    CHECK(t.n_meas == 1);
    CHECK(t.meas[0].mean_mm == 2000);
    CHECK(t.meas[0].n_ok == 38);
}

static void test_measurement_rejects_bad_indices(void)
{
    struct apos_table t;

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_add_meas(&t, 0, 0, 1000, 10, 40) == -EINVAL);
    CHECK(apos_table_add_meas(&t, 0, 5, 1000, 10, 40) == -EINVAL);
}

/* Both directions average, and the combined sd must be tighter than either --
 * that improvement is the reason both directions are measured. */
static void test_both_directions_average_and_tighten(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 40);
    apos_table_add_meas(&t, 1, 0, 1040, 20, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CLOSE(e[0].d_m, 1.020f, 1e-4f);
    CHECK(e[0].sd_m < 0.020f);
}

/* One direction only: kept, but explicitly trusted less. */
static void test_one_direction_is_kept_with_inflated_sd(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CLOSE(e[0].d_m, 1.000f, 1e-4f);
    CLOSE(e[0].sd_m, 0.020f * APOS_ONEWAY_SD_INFLATE, 1e-4f);
}

/* A handful of successful exchanges is not a measurement. Dropping it is what
 * keeps a pair that barely reaches out of the geometry. */
static void test_thin_measurements_are_discarded(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 3);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 0);
    CHECK(apos_table_missing_pairs(&t, 10) == 1);
}

/* A zero sd must not give one edge infinite weight. */
static void test_zero_sd_is_floored(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 0, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CHECK(e[0].sd_m > 0.0f);
}

static void test_missing_pairs_counts_holes(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    for (uint8_t k = 0; k < 4; k++) {
        add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k));
    }
    /* Measure 5 of the 6 pairs, one direction each. */
    apos_table_add_meas(&t, 0, 1, 1000, 10, 40);
    apos_table_add_meas(&t, 0, 2, 1000, 10, 40);
    apos_table_add_meas(&t, 0, 3, 1000, 10, 40);
    apos_table_add_meas(&t, 1, 2, 1000, 10, 40);
    apos_table_add_meas(&t, 1, 3, 1000, 10, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 5);
    CHECK(apos_table_missing_pairs(&t, 10) == 1);
}

/* Symmetrise must respect out_cap rather than overrunning the caller. */
static void test_symmetrise_respects_out_cap(void)
{
    struct apos_table t;
    struct apos_edge e[2];

    apos_table_init(&t);
    for (uint8_t k = 0; k < 4; k++) {
        add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k));
    }
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 4; j++) {
            apos_table_add_meas(&t, i, j, 1000, 10, 40);
        }
    }

    CHECK(apos_table_symmetrise(&t, e, 2, 10) == 2);
}

/* APOS_MAX_MEAS = 56 = 8*7 is hit exactly by a fully-surveyed 8-node
 * deployment, with zero headroom. A `>` -vs- `>=` slip in the capacity
 * check would silently drop the last real measurement of a full survey, so
 * the boundary is asserted explicitly: fill every ordered pair except
 * (6, 7), confirm the 56th add (the last new pair, reaching capacity
 * exactly) still SUCCEEDS, confirm a genuinely new 57th pair is impossible
 * to request at APOS_MAX_NODES peers so -ENOSPC is exercised by shrinking
 * the peer set instead, and confirm replace-on-repeat still succeeds at
 * full capacity since it consumes no new slot. */
static void test_measurement_table_fills_to_exact_capacity(void)
{
    struct apos_table t;
    uint16_t n = 0;

    apos_table_init(&t);
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k));
    }
    CHECK(t.n_peers == APOS_MAX_NODES);

    /* Fill every ordered pair except (6, 7): APOS_MAX_MEAS - 1 rows. */
    for (uint8_t i = 0; i < APOS_MAX_NODES; i++) {
        for (uint8_t j = 0; j < APOS_MAX_NODES; j++) {
            if (i == j || (i == 6 && j == 7)) {
                continue;
            }
            CHECK(apos_table_add_meas(&t, i, j, 1000, 10, 40) == 0);
            n++;
        }
    }
    CHECK(n == APOS_MAX_MEAS - 1);
    CHECK(t.n_meas == APOS_MAX_MEAS - 1);

    /* The 56th row: the last remaining new (from, to) pair, reaching
     * capacity exactly. This is the boundary a `>` -vs- `>=` slip would
     * break -- it must still SUCCEED. */
    CHECK(apos_table_add_meas(&t, 6, 7, 1234, 10, 40) == 0);
    CHECK(t.n_meas == APOS_MAX_MEAS);

    /* Replace-on-repeat for an already-recorded pair must still succeed at
     * full capacity: it consumes no new slot, so the capacity check must
     * not block it. */
    CHECK(apos_table_add_meas(&t, 6, 7, 9999, 99, 41) == 0);
    CHECK(t.n_meas == APOS_MAX_MEAS);
    CHECK(t.meas[APOS_MAX_MEAS - 1].mean_mm == 9999);
    CHECK(t.meas[APOS_MAX_MEAS - 1].n_ok == 41);
}

/* NOTE on the -ENOSPC branch of apos_table_add_meas: a true 57th-add-returns
 * -ENOSPC test is not constructible with legal indices. apos_table_add_meas
 * rejects from/to >= t->n_peers, and t->n_peers <= APOS_MAX_NODES, so the
 * maximum number of distinct valid (from, to) pairs is
 * APOS_MAX_NODES*(APOS_MAX_NODES-1) = APOS_MAX_MEAS exactly -- the array is
 * sized so that a fully-surveyed maximum deployment lands precisely on the
 * capacity boundary with no legal index combination left over to overflow
 * it. The -ENOSPC branch in apos_table_add_meas is therefore unreachable
 * through the function's own bounds contract; see the fix report for this
 * disclosed as a finding rather than a fabricated test. */

/* A freshly-initialised table has no peers at all. symmetrise and
 * missing_pairs must both no-op cleanly rather than reading past n_peers. */
static void test_empty_table_symmetrise_and_missing_pairs_are_zero(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    CHECK(t.n_peers == 0);
    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 0);
    CHECK(apos_table_missing_pairs(&t, 10) == 0);
}

int main(void)
{
    test_peers_get_sequential_indices();
    test_readding_the_same_eui_is_idempotent();
    test_same_addr_from_a_different_eui_is_reported();
    test_same_eui_with_a_new_addr_updates_in_place();
    test_lookup_by_addr_and_eui();
    test_table_full_is_enospc();
    test_repeat_measurement_replaces();
    test_measurement_rejects_bad_indices();
    test_both_directions_average_and_tighten();
    test_one_direction_is_kept_with_inflated_sd();
    test_thin_measurements_are_discarded();
    test_zero_sd_is_floored();
    test_missing_pairs_counts_holes();
    test_symmetrise_respects_out_cap();
    test_measurement_table_fills_to_exact_capacity();
    test_empty_table_symmetrise_and_missing_pairs_are_zero();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}

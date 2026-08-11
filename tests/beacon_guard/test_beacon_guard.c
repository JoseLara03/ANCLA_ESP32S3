#include "beacon_guard.h"
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Round numbers, not real hi32 values: the module is unit-agnostic. */
#define PERIOD     100000u
#define GUARD        1000u
#define OCCUPANCY    3000u

static void setup(struct beacon_guard *g, uint32_t first_beacon)
{
    beacon_guard_init(g, PERIOD, GUARD, OCCUPANCY);
    beacon_guard_beacon(g, first_beacon);
}

static void test_unlocked_allows_everything(void)
{
    struct beacon_guard g;
    beacon_guard_init(&g, PERIOD, GUARD, OCCUPANCY);

    /* No beacon seen yet: never suppress. Suppressing without a reference
     * would block transmits at arbitrary times. */
    CHECK(!beacon_guard_locked(&g));
    CHECK(beacon_guard_tx_allowed(&g, 12345u));
    CHECK(beacon_guard_tx_allowed(&g, 999999u));
}

static void test_window_edges(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);
    CHECK(beacon_guard_locked(&g));

    /* Next beacon predicted at 1000 + PERIOD = 101000.
     * Forbidden window is [101000 - GUARD, 101000 + OCCUPANCY + GUARD]
     *                   = [100000, 105000]. */
    CHECK(beacon_guard_tx_allowed(&g,  99999u));   /* just before */
    CHECK(!beacon_guard_tx_allowed(&g, 100000u));  /* leading edge */
    CHECK(!beacon_guard_tx_allowed(&g, 101000u));  /* beacon start */
    CHECK(!beacon_guard_tx_allowed(&g, 105000u));  /* trailing edge */
    CHECK(beacon_guard_tx_allowed(&g, 105001u));   /* just after */
}

static void test_rolls_forward_across_superframes(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* A TX three superframes out must be checked against the beacon in THAT
     * superframe, not the next one. Window 3 ahead: [300000, 305000]. */
    CHECK(beacon_guard_tx_allowed(&g, 299999u));
    CHECK(!beacon_guard_tx_allowed(&g, 301000u));
}

static void test_lock_drops_after_max_misses(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* Rolling past BEACON_GUARD_MAX_MISSES predicted beacons without seeing
     * one means the prediction is stale; stop suppressing rather than block
     * transmits against a guess. */
    uint32_t far = 1000u + PERIOD * (BEACON_GUARD_MAX_MISSES + 2u);
    CHECK(beacon_guard_tx_allowed(&g, far));
    CHECK(!beacon_guard_locked(&g));

    /* A fresh beacon re-acquires. */
    beacon_guard_beacon(&g, far + 500u);
    CHECK(beacon_guard_locked(&g));
}

static void test_beacon_resets_misses(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* Two missed superframes, then a beacon: still locked afterwards. */
    (void)beacon_guard_tx_allowed(&g, 1000u + PERIOD * 2u + OCCUPANCY + GUARD + 1u);
    CHECK(beacon_guard_locked(&g));
    beacon_guard_beacon(&g, 1000u + PERIOD * 3u);
    CHECK(beacon_guard_locked(&g));

    /* And the reference moved: window is now [.. + PERIOD .. ]. */
    CHECK(!beacon_guard_tx_allowed(&g, 1000u + PERIOD * 4u));
}

static void test_wrap_forward(void)
{
    struct beacon_guard g;
    /* Beacon shortly before the 32-bit wrap; the next one is past it. */
    setup(&g, 0xFFFFFF00u);

    /* 0xFFFFFF00 + 100000 wraps to 0x000185A0 (unsigned arithmetic is fine;
     * the COMPARISONS are what must be signed). Window [0x000181B8,
     * 0x00019388]. */
    uint32_t predicted = 0xFFFFFF00u + PERIOD;

    CHECK(!beacon_guard_tx_allowed(&g, predicted));
    CHECK(!beacon_guard_tx_allowed(&g, predicted - GUARD));
    CHECK(beacon_guard_tx_allowed(&g, predicted - GUARD - 1u));
    CHECK(beacon_guard_tx_allowed(&g, predicted + OCCUPANCY + GUARD + 1u));
}

static void test_wrap_tx_before_beacon_after(void)
{
    struct beacon_guard g;
    /* The case a naive unsigned compare gets wrong: the TX time is a huge
     * number just below the wrap, the predicted beacon a small one just
     * above it. Unsigned, tx > beacon and the roll-forward loop runs away;
     * signed, tx - beacon is a small negative and the TX is simply early. */
    setup(&g, 0xFFFF0000u - PERIOD);

    uint32_t predicted = 0xFFFF0000u;          /* next beacon, pre-wrap */
    CHECK(!beacon_guard_tx_allowed(&g, predicted + 100u));
    CHECK(beacon_guard_tx_allowed(&g, predicted - GUARD - 1u));
}

int main(void)
{
    test_unlocked_allows_everything();
    test_window_edges();
    test_rolls_forward_across_superframes();
    test_lock_drops_after_max_misses();
    test_beacon_resets_misses();
    test_wrap_forward();
    test_wrap_tx_before_beacon_after();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

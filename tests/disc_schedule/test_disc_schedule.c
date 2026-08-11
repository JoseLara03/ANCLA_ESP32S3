#include "disc_schedule.h"
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_base_and_stagger(void)
{
    /* Anchor 0 responds at the turnaround floor, not one slot in. */
    CHECK(disc_resp_delay_uus(0) == DISC_BASE_UUS);
    CHECK(disc_resp_delay_uus(0) == 6000u);

    /* Each subsequent anchor is exactly one slot later. */
    CHECK(disc_resp_delay_uus(1) == 6000u + 3500u);
    CHECK(disc_resp_delay_uus(2) == 6000u + 2u * 3500u);
    CHECK(disc_resp_delay_uus(3) == 6000u + 3u * 3500u);
}

static void test_slots_never_overlap(void)
{
    /* The whole point of the module: no two anchors share a transmit
     * instant, and the gap is always a full slot. */
    for (uint8_t i = 1; i < 4; i++) {
        CHECK(disc_resp_delay_uus(i) > disc_resp_delay_uus(i - 1));
        CHECK(disc_resp_delay_uus(i) - disc_resp_delay_uus(i - 1) == DISC_SLOT_UUS);
    }
}

int main(void)
{
    test_base_and_stagger();
    test_slots_never_overlap();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

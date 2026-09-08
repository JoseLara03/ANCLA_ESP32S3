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
    CHECK(disc_resp_delay_uus(0) == 2000u);

    /* Each subsequent anchor is exactly one slot later. */
    CHECK(disc_resp_delay_uus(1) == 2000u + 3500u);
    CHECK(disc_resp_delay_uus(2) == 2000u + 2u * 3500u);
    CHECK(disc_resp_delay_uus(3) == 2000u + 3u * 3500u);
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

static void test_group_match_partitions_every_anchor(void)
{
    /* network-scaling-v3: 32 anchors over 8 groups -- every id belongs to
     * exactly one group, and every group holds exactly 4 ids (32 / 8). */
    for (uint8_t n_groups = 1; n_groups <= 8; n_groups++) {
        for (uint16_t id = 0; id < 32; id++) {
            uint8_t hits = 0;

            for (uint8_t group = 0; group < n_groups; group++) {
                if (disc_group_match((uint8_t)id, group, n_groups)) {
                    hits++;
                }
            }
            CHECK(hits == 1);
        }
    }

    /* n_groups == 0 is a malformed frame (uwb_frame_parse_discovery() itself
     * rejects it with -EINVAL) -- stays safe rather than matching everything
     * or dividing by zero. */
    CHECK(disc_group_match(0, 0, 0) == false);
}

static void test_grouped_delay_bounded_at_32_anchors(void)
{
    /* n_groups = 8 for 32 anchors: every group holds at most 4 anchors
     * (rank 0..3), so the max delay is exactly the ungrouped 4-anchor case
     * -- the value TX_COMPLETE_TIMEOUT_MS (anchor_respond.c) is derived
     * from, unchanged by deployment size. */
    uint32_t max_delay = 0;

    for (uint16_t id = 0; id < 32; id++) {
        uint32_t delay_uus;
        bool ok = disc_resp_delay_uus_grouped((uint8_t)id, 8, &delay_uus);

        CHECK(ok);
        if (ok && delay_uus > max_delay) {
            max_delay = delay_uus;
        }
    }
    CHECK(max_delay == DISC_BASE_UUS + DISC_MAX_RANK * DISC_SLOT_UUS);
    CHECK(max_delay == 12500u);
}

static void test_grouped_delay_matches_ungrouped_at_n_groups_1(void)
{
    for (uint8_t id = 0; id < 4; id++) {
        uint32_t delay_uus;

        CHECK(disc_resp_delay_uus_grouped(id, 1, &delay_uus));
        CHECK(delay_uus == disc_resp_delay_uus(id));
    }
}

static void test_rank_over_max_refuses(void)
{
    /* n_groups undersized for the deployment: id 16 with n_groups = 2 has
     * rank = 8, far past DISC_MAX_RANK. Must refuse rather than schedule a
     * response past TX_COMPLETE_TIMEOUT_MS. */
    uint32_t delay_uus = 0xFFFFFFFFu;

    CHECK(disc_resp_delay_uus_grouped(16, 2, &delay_uus) == false);
    CHECK(delay_uus == 0xFFFFFFFFu); /* untouched on refusal */

    /* rank == DISC_MAX_RANK is still allowed; rank == DISC_MAX_RANK + 1 is
     * the first refusal, at n_groups = 1 (rank == anchor_id). */
    CHECK(disc_resp_delay_uus_grouped(3, 1, &delay_uus));
    CHECK(disc_resp_delay_uus_grouped(4, 1, &delay_uus) == false);
}

static void test_grouped_delay_n_groups_zero_refuses(void)
{
    uint32_t delay_uus;

    CHECK(disc_resp_delay_uus_grouped(0, 0, &delay_uus) == false);
}

int main(void)
{
    test_base_and_stagger();
    test_slots_never_overlap();
    test_group_match_partitions_every_anchor();
    test_grouped_delay_bounded_at_32_anchors();
    test_grouped_delay_matches_ungrouped_at_n_groups_1();
    test_rank_over_max_refuses();
    test_grouped_delay_n_groups_zero_refuses();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

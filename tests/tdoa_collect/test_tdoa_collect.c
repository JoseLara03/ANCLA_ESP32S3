#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "tdoa_collect.h"
#include "tdoa_solve.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static struct tdoa_meas fake_meas(float x, float y, int64_t t_dtu)
{
	struct tdoa_meas m = { .x = x, .y = y, .dz = 1.5f, .t_dtu = t_dtu };
	return m;
}

static struct tdoa_obs obs(uint16_t tag, uint8_t seq, uint8_t anchor,
			   float x, float y, int64_t t_dtu)
{
	struct tdoa_obs o = {
		.tag_addr  = tag,
		.blink_seq = seq,
		.anchor_id = anchor,
		.meas      = fake_meas(x, y, t_dtu),
	};
	return o;
}

/* Three observations of the same (tag, seq) are delivered together; a fourth
 * with a different seq must not be mixed in. */
static void test_groups_by_tag_and_seq(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	o = obs(0x0100, 7, 0, 0.0f, 0.0f, 1000);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0100, 7, 1, 10.0f, 0.0f, 1010);
	CHECK(tdoa_collect_add(&c, &o, 5));
	o = obs(0x0100, 7, 2, 10.0f, 10.0f, 1020);
	CHECK(tdoa_collect_add(&c, &o, 10));

	/* Not complete yet (POS_MAX_ANCHORS == 4): window still open. */
	CHECK(!tdoa_collect_take_ready(&c, 10, out, &n_out, &tag_out));

	/* A different blink_seq must open its own group, not join this one. */
	o = obs(0x0100, 8, 0, 0.0f, 0.0f, 2000);
	CHECK(tdoa_collect_add(&c, &o, 12));

	/* Complete the seq==7 group with its 4th anchor -- it releases
	 * immediately, and only 3 measurements belong to it. */
	o = obs(0x0100, 7, 3, 0.0f, 10.0f, 1030);
	CHECK(tdoa_collect_add(&c, &o, 15));
	CHECK(tdoa_collect_take_ready(&c, 15, out, &n_out, &tag_out));
	CHECK(n_out == 4);
	CHECK(tag_out == 0x0100);

	/* The seq==8 group is untouched and still open. */
	CHECK(!tdoa_collect_take_ready(&c, 15, out, &n_out, &tag_out));
}

/* Same blink_seq, different tag_addr: two independent groups. */
static void test_two_tags_do_not_mix(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	for (uint8_t a = 0; a < 4; a++) {
		o = obs(0x0100, 3, a, (float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, 0));
	}
	for (uint8_t a = 0; a < 4; a++) {
		o = obs(0x0101, 3, a, (float)a, 5.0f, 2000 + a);
		CHECK(tdoa_collect_add(&c, &o, 0));
	}

	bool got_100 = false, got_101 = false;

	for (int i = 0; i < 2; i++) {
		CHECK(tdoa_collect_take_ready(&c, 0, out, &n_out, &tag_out));
		CHECK(n_out == 4);
		if (tag_out == 0x0100) got_100 = true;
		if (tag_out == 0x0101) got_101 = true;
	}
	CHECK(got_100 && got_101);
	CHECK(!tdoa_collect_take_ready(&c, 0, out, &n_out, &tag_out));
}

/* Two observations never reach TDOA_MIN_ANCHORS and are not delivered while
 * the window is still open. */
static void test_below_minimum_is_not_ready(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	o = obs(0x0200, 1, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0200, 1, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 3));

	CHECK(!tdoa_collect_take_ready(&c, 3, out, &n_out, &tag_out));
	CHECK(!tdoa_collect_take_ready(&c, 100, out, &n_out, &tag_out));
}

/* An incomplete group vanishes, and frees its slot, once the window expires
 * with fewer than TDOA_MIN_ANCHORS gathered. */
static void test_window_expiry_drops_the_group(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	o = obs(0x0300, 2, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0300, 2, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 1));

	uint32_t past_deadline = 0 + TDOA_COLLECT_WINDOW_MS + 1;

	CHECK(!tdoa_collect_take_ready(&c, past_deadline, out, &n_out, &tag_out));

	/* Slot freed: a brand-new group at the same key must be accepted fresh,
	 * not silently merged with whatever the discarded one held. */
	o = obs(0x0300, 2, 0, 1.0f, 1.0f, 900);
	CHECK(tdoa_collect_add(&c, &o, past_deadline));
	o = obs(0x0300, 2, 1, 2.0f, 1.0f, 910);
	CHECK(tdoa_collect_add(&c, &o, past_deadline + 1));
	o = obs(0x0300, 2, 2, 3.0f, 1.0f, 920);
	CHECK(tdoa_collect_add(&c, &o, past_deadline + 1));

	/* Only 3 of 4 anchors reported: not complete, so it must wait out its own
	 * window before releasing, not before. */
	CHECK(!tdoa_collect_take_ready(&c, past_deadline + 1, out, &n_out, &tag_out));

	uint32_t second_deadline = past_deadline + TDOA_COLLECT_WINDOW_MS + 1;

	CHECK(tdoa_collect_take_ready(&c, second_deadline, out, &n_out, &tag_out));
	CHECK(n_out == 3);
}

/* tdoa_collect.c:109 branches on age_ms(...) >= TDOA_COLLECT_WINDOW_MS, so
 * the boundary the code actually tests is the EXACT edge, not one past it.
 * Below the minimum at exactly the window age must still discard; at or
 * above TDOA_MIN_ANCHORS at exactly the window age must still release. */
static void test_window_expiry_at_exact_boundary(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	/* Below minimum, exactly at the boundary: must discard. */
	o = obs(0x0D00, 5, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0D00, 5, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 0));
	CHECK(!tdoa_collect_take_ready(&c, TDOA_COLLECT_WINDOW_MS, out, &n_out,
				       &tag_out));

	/* At minimum, exactly at the boundary: must release. */
	o = obs(0x0D01, 6, 0, 0.0f, 0.0f, 200);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0D01, 6, 1, 10.0f, 0.0f, 210);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0D01, 6, 2, 10.0f, 10.0f, 220);
	CHECK(tdoa_collect_add(&c, &o, 0));
	CHECK(tdoa_collect_take_ready(&c, TDOA_COLLECT_WINDOW_MS, out, &n_out,
				      &tag_out));
	CHECK(n_out == 3);
	CHECK(tag_out == 0x0D01);
}

/* TDOA_COLLECT_SLOTS + 1 blinks open at once: the oldest is evicted so the
 * newest arrival always has somewhere to go. */
static void test_slot_exhaustion_drops_oldest(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	/* One observation each is enough to open a group -- fill every slot,
	 * each one one ms younger than the last so age ordering is unambiguous. */
	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		o = obs((uint16_t)(0x0400 + i), 0, 0, 0.0f, 0.0f, 1000);
		CHECK(tdoa_collect_add(&c, &o, i));
	}

	/* One more, later still: must evict tag 0x0400 (age 0, the oldest) and
	 * be accepted itself. */
	uint32_t now = TDOA_COLLECT_SLOTS;

	o = obs((uint16_t)(0x0400 + TDOA_COLLECT_SLOTS), 0, 0, 0.0f, 0.0f, 1000);
	CHECK(tdoa_collect_add(&c, &o, now));

	/* The evicted tag's group must be gone: adding a second anchor for it
	 * now opens a brand NEW group (n resets to 1) rather than joining
	 * whatever survived, and it can never reach TDOA_MIN_ANCHORS from just
	 * this one further observation. */
	o = obs(0x0400, 0, 1, 10.0f, 0.0f, 1010);
	CHECK(tdoa_collect_add(&c, &o, now + 1));
	CHECK(!tdoa_collect_take_ready(&c, now + 1, out, &n_out, &tag_out));

	/* The newest tag's group is intact and can still be completed. */
	for (uint8_t a = 1; a < 4; a++) {
		o = obs((uint16_t)(0x0400 + TDOA_COLLECT_SLOTS), 0, a,
			(float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, now + 2));
	}
	CHECK(tdoa_collect_take_ready(&c, now + 2, out, &n_out, &tag_out));
	CHECK(n_out == 4);
	CHECK(tag_out == (uint16_t)(0x0400 + TDOA_COLLECT_SLOTS));
}

/* Slot exhaustion must not destroy an already-releasable group. Fill one
 * slot to n == TDOA_MIN_ANCHORS (oldest, and therefore the group the OLD
 * "evict oldest" policy would have picked), fill the rest with barely-formed
 * n == 1 groups, then force a 17th distinct blink to arrive. The victim must
 * be one of the incomplete groups, never the releasable oldest one -- and
 * the releasable group must still be retrievable afterwards. */
static void test_slot_exhaustion_protects_a_complete_group(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	/* Slot 0: the oldest group, and already releasable (n == 3). */
	uint16_t complete_tag = 0x0800;

	for (uint8_t a = 0; a < TDOA_MIN_ANCHORS; a++) {
		o = obs(complete_tag, 0, a, (float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, 0));
	}

	/* Slots 1..15: younger, single-observation groups. */
	for (unsigned int i = 1; i < TDOA_COLLECT_SLOTS; i++) {
		o = obs((uint16_t)(0x0900 + i), 0, 0, 0.0f, 0.0f, 1000);
		CHECK(tdoa_collect_add(&c, &o, i));
	}

	/* The 17th distinct blink: every slot is full. The victim must be one of
	 * the n == 1 groups, not the releasable oldest one. */
	uint16_t newcomer = 0x0A00;
	uint32_t now = TDOA_COLLECT_SLOTS;

	CHECK(tdoa_collect_add(&c, &(struct tdoa_obs){
		.tag_addr = newcomer, .blink_seq = 0, .anchor_id = 0,
		.meas = fake_meas(0.0f, 0.0f, 1000),
	}, now));

	/* The complete group survived: at n == TDOA_MIN_ANCHORS (not
	 * POS_MAX_ANCHORS) it still needs its window to expire before
	 * take_ready() will hand it over -- protection means it was not
	 * evicted, not that it jumps the release queue. */
	uint32_t deadline = now + TDOA_COLLECT_WINDOW_MS + 1;

	CHECK(tdoa_collect_take_ready(&c, deadline, out, &n_out, &tag_out));
	CHECK(n_out == TDOA_MIN_ANCHORS);
	CHECK(tag_out == complete_tag);
}

/* When every slot already holds a releasable group, the new observation is
 * rejected outright rather than displacing any of them -- all
 * TDOA_COLLECT_SLOTS pending fixes are about to be drained anyway. */
static void test_slot_exhaustion_all_releasable_rejects_newcomer(void)
{
	struct tdoa_collect c;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		uint16_t tag = (uint16_t)(0x0B00 + i);

		for (uint8_t a = 0; a < TDOA_MIN_ANCHORS; a++) {
			struct tdoa_obs o = obs(tag, 0, a, (float)a, 0.0f, 1000 + a);

			CHECK(tdoa_collect_add(&c, &o, i));
		}
	}

	/* Every slot is full and releasable: rejected, not evicted-into. */
	struct tdoa_obs newcomer = obs(0x0C00, 0, 0, 0.0f, 0.0f, 1000);

	CHECK(!tdoa_collect_add(&c, &newcomer, TDOA_COLLECT_SLOTS));

	/* All 16 original groups are untouched and releasable, once their
	 * windows expire (n == TDOA_MIN_ANCHORS still waits on the window,
	 * unlike n == POS_MAX_ANCHORS). */
	uint32_t deadline = TDOA_COLLECT_SLOTS + TDOA_COLLECT_WINDOW_MS + 1;

	for (unsigned int i = 0; i < TDOA_COLLECT_SLOTS; i++) {
		CHECK(tdoa_collect_take_ready(&c, deadline, out, &n_out,
					      &tag_out));
		CHECK(n_out == TDOA_MIN_ANCHORS);
	}
	CHECK(!tdoa_collect_take_ready(&c, deadline, out, &n_out, &tag_out));
}

/* The same anchor_id reporting twice for one blink counts once. A duplicate
 * would otherwise feed the solver a range-difference equation against
 * itself and make its normal matrix singular. */
static void test_duplicate_anchor_is_ignored(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);

	o = obs(0x0500, 4, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0500, 4, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 1));

	/* Redelivery of anchor 0's observation: rejected, count unchanged. */
	o = obs(0x0500, 4, 0, 0.0f, 0.0f, 100);
	CHECK(!tdoa_collect_add(&c, &o, 2));

	o = obs(0x0500, 4, 2, 10.0f, 10.0f, 120);
	CHECK(tdoa_collect_add(&c, &o, 3));

	/* Still only 3 distinct anchors -- release on window expiry, not before,
	 * and confirm the count is 3, not 4. */
	uint32_t past_deadline = 0 + TDOA_COLLECT_WINDOW_MS + 1;

	CHECK(tdoa_collect_take_ready(&c, past_deadline, out, &n_out, &tag_out));
	CHECK(n_out == 3);
}

/* Same scenario as test_window_expiry_drops_the_group, but now_ms starts near
 * UINT32_MAX and the window crosses the wrap. Signed-difference arithmetic
 * must handle this exactly like any other interval. */
static void test_ms_clock_wrap(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;
	uint32_t t0 = UINT32_MAX - 5u;   /* wraps 6 ms later */

	tdoa_collect_init(&c);

	o = obs(0x0600, 9, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, t0));
	o = obs(0x0600, 9, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, t0 + 1));   /* wraps to 0 */

	uint32_t before = t0 + (TDOA_COLLECT_WINDOW_MS - 1u);   /* wrapped, still open */
	uint32_t after  = t0 + TDOA_COLLECT_WINDOW_MS + 1u;     /* wrapped, expired */

	CHECK(!tdoa_collect_take_ready(&c, before, out, &n_out, &tag_out));
	CHECK(!tdoa_collect_take_ready(&c, after, out, &n_out, &tag_out));

	/* Freed: a fresh group at the same key is accepted, proving the old one
	 * was actually discarded rather than corrupted by the wrap. */
	o = obs(0x0600, 9, 0, 1.0f, 1.0f, 900);
	CHECK(tdoa_collect_add(&c, &o, after));
	o = obs(0x0600, 9, 1, 2.0f, 1.0f, 910);
	CHECK(tdoa_collect_add(&c, &o, after));
	o = obs(0x0600, 9, 2, 3.0f, 1.0f, 920);
	CHECK(tdoa_collect_add(&c, &o, after));

	/* Only 3 of 4 anchors: must wait out its own window (again crossing a
	 * wrap boundary) before releasing. */
	uint32_t second_deadline = after + TDOA_COLLECT_WINDOW_MS + 1u;

	CHECK(!tdoa_collect_take_ready(&c, after, out, &n_out, &tag_out));
	CHECK(tdoa_collect_take_ready(&c, second_deadline, out, &n_out, &tag_out));
	CHECK(n_out == 3);
}

/* End-to-end: collector feeds tdoa_solve() directly, proving the two pieces
 * fit together rather than merely passing their own separate suites. Same
 * synthetic geometry as tests/tdoa_solve/test_tdoa_solve.c. */
static const float AX[4] = { 0.0f, 10.0f, 10.0f,  0.0f };
static const float AY[4] = { 0.0f,  0.0f, 10.0f, 10.0f };
#define DZ 1.5f

static void test_collector_feeds_solver(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;
	float tx = 5.0f, ty = 5.0f;

	tdoa_collect_init(&c);

	for (uint8_t a = 0; a < 4; a++) {
		float dx = tx - AX[a], dy = ty - AY[a];
		float r = sqrtf(dx * dx + dy * dy + DZ * DZ);
		int64_t t = 0x1234567890LL + (int64_t)llroundf(r / TDOA_M_PER_DTU);

		o = obs(0x0700, 1, a, AX[a], AY[a], t);
		o.meas.dz = DZ;
		CHECK(tdoa_collect_add(&c, &o, a));
	}
	CHECK(tdoa_collect_take_ready(&c, 3, out, &n_out, &tag_out));
	CHECK(n_out == 4);
	CHECK(tag_out == 0x0700);

	struct pos_result r;

	CHECK(tdoa_solve(out, n_out, NULL, &r));
	CHECK(r.valid);
	CHECK(fabsf(r.x - tx) < 0.05f);
	CHECK(fabsf(r.y - ty) < 0.05f);
	printf("  collector+solver: (%.1f,%.1f) -> (%.3f,%.3f)\n",
	       (double)tx, (double)ty, (double)r.x, (double)r.y);
}

/* tdoa_collect_set_expected(): a 3-anchor deployment must release as soon as
 * its 3rd anchor reports, not wait out the window the way a 4-anchor
 * deployment (today's hardcoded POS_MAX_ANCHORS) would. This is the actual
 * defect the setter exists to fix: without it, a live 3-anchor array never
 * hits the early-release branch at all. */
static void test_set_expected_releases_early(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 3);

	o = obs(0x0E00, 1, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0E00, 1, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 1));

	/* Only 2 of 3 expected: window is 150 ms, so this must NOT release yet. */
	CHECK(!tdoa_collect_take_ready(&c, 2, out, &n_out, &tag_out));

	o = obs(0x0E00, 1, 2, 10.0f, 10.0f, 120);
	CHECK(tdoa_collect_add(&c, &o, 2));

	/* 3rd of 3 expected: releases immediately, well inside the 150 ms
	 * window -- the whole point of the setter. */
	CHECK(tdoa_collect_take_ready(&c, 2, out, &n_out, &tag_out));
	CHECK(n_out == 3);
	CHECK(tag_out == 0x0E00);
}

/* tdoa_collect_set_expected(c, POS_MAX_ANCHORS) must reproduce exactly
 * today's behaviour -- a caller that always has 4 anchors and calls the
 * setter explicitly sees no change from one that never calls it at all. */
static void test_set_expected_pos_max_matches_default(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, POS_MAX_ANCHORS);

	for (uint8_t a = 0; a < 3; a++) {
		o = obs(0x0F00, 2, a, (float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, a));
	}
	/* 3 of 4: must still wait for the window, exactly like the unset-setter
	 * case in test_groups_by_tag_and_seq(). */
	CHECK(!tdoa_collect_take_ready(&c, 3, out, &n_out, &tag_out));

	o = obs(0x0F00, 2, 3, 3.0f, 0.0f, 1003);
	CHECK(tdoa_collect_add(&c, &o, 3));
	CHECK(tdoa_collect_take_ready(&c, 3, out, &n_out, &tag_out));
	CHECK(n_out == 4);
}

/* An out-of-range `n` is clamped, not rejected or left to corrupt state --
 * checked at both ends against the behaviour the clamp is supposed to
 * produce (TDOA_MIN_ANCHORS on the low side, POS_MAX_ANCHORS on the high
 * side). */
static void test_set_expected_clamps_out_of_range(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	/* Below TDOA_MIN_ANCHORS clamps up to it: 3 anchors releases early. */
	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 0);
	for (uint8_t a = 0; a < 3; a++) {
		o = obs(0x1000, 1, a, (float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, a));
	}
	CHECK(tdoa_collect_take_ready(&c, 2, out, &n_out, &tag_out));
	CHECK(n_out == 3);

	/* Above POS_MAX_ANCHORS clamps down to it: 3 of 4 still waits. */
	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 255);
	for (uint8_t a = 0; a < 3; a++) {
		o = obs(0x1001, 1, a, (float)a, 0.0f, 1000 + a);
		CHECK(tdoa_collect_add(&c, &o, a));
	}
	CHECK(!tdoa_collect_take_ready(&c, 2, out, &n_out, &tag_out));
}

/* Lowering `expected` must not touch the below-minimum discard path: a group
 * that never reaches TDOA_MIN_ANCHORS is still silently dropped once its
 * window expires, whatever `expected` is set to -- the setter changes the
 * EARLY-release threshold only, never the floor. */
static void test_set_expected_does_not_affect_minimum_discard(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 3);

	o = obs(0x1100, 1, 0, 0.0f, 0.0f, 100);
	CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x1100, 1, 1, 10.0f, 0.0f, 110);
	CHECK(tdoa_collect_add(&c, &o, 1));

	uint32_t past_deadline = TDOA_COLLECT_WINDOW_MS + 1;

	CHECK(!tdoa_collect_take_ready(&c, past_deadline, out, &n_out, &tag_out));
}

/*
 * meas[0] is the reference every downstream stage differences, linearises and
 * takes the filter's dt against, so which anchor lands there must depend on
 * the SET of anchors that reported and nothing else -- not on which MQTT
 * message happened to arrive first.
 *
 * Each observation carries its anchor's x as a marker, so the delivered order
 * is readable straight off the output. Every arrival permutation of the same
 * three anchors must produce the same meas[] order.
 */
static void test_reference_anchor_is_deterministic(void)
{
	/* All 6 orderings of anchors {1, 2, 3}. */
	static const uint8_t perm[6][3] = {
		{1, 2, 3}, {1, 3, 2}, {2, 1, 3},
		{2, 3, 1}, {3, 1, 2}, {3, 2, 1},
	};

	for (unsigned int p = 0; p < 6u; p++) {
		struct tdoa_collect c;
		struct tdoa_meas out[POS_MAX_ANCHORS];
		size_t n = 0;
		uint16_t tag = 0;

		tdoa_collect_init(&c);
		tdoa_collect_set_expected(&c, 3u);

		for (unsigned int k = 0; k < 3u; k++) {
			uint8_t a = perm[p][k];

			/* x = anchor id, so meas[i].x reveals the ordering. */
			CHECK(tdoa_collect_add(&c,
				&(struct tdoa_obs){
					.tag_addr = 0x0100, .blink_seq = 7,
					.anchor_id = a,
					.meas = fake_meas((float)a, 0.0f,
							  1000 + 10 * a),
				}, 1000u));
		}

		CHECK(tdoa_collect_take_ready(&c, 1000u, out, &n, &tag));
		CHECK(n == 3u);
		/* Ascending by anchor_id, whatever the arrival order was. */
		CHECK(out[0].x == 1.0f);
		CHECK(out[1].x == 2.0f);
		CHECK(out[2].x == 3.0f);
	}
	printf("  reference anchor: lowest id first for all 6 arrival orders\n");
}

/* The same, for a group that does NOT contain the lowest possible anchor id:
 * the reference is the lowest id PRESENT, which is the honest limit stated in
 * tdoa_collect.h -- deterministic given the set, not invariant across sets. */
static void test_reference_is_lowest_present_not_lowest_possible(void)
{
	struct tdoa_collect c;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n = 0;
	uint16_t tag = 0;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 3u);

	/* Anchor 0 never reports for this blink. */
	CHECK(tdoa_collect_add(&c, &(struct tdoa_obs){
		.tag_addr = 0x0100, .blink_seq = 9, .anchor_id = 3,
		.meas = fake_meas(3.0f, 0.0f, 1030) }, 500u));
	CHECK(tdoa_collect_add(&c, &(struct tdoa_obs){
		.tag_addr = 0x0100, .blink_seq = 9, .anchor_id = 1,
		.meas = fake_meas(1.0f, 0.0f, 1010) }, 500u));
	CHECK(tdoa_collect_add(&c, &(struct tdoa_obs){
		.tag_addr = 0x0100, .blink_seq = 9, .anchor_id = 2,
		.meas = fake_meas(2.0f, 0.0f, 1020) }, 500u));

	CHECK(tdoa_collect_take_ready(&c, 500u, out, &n, &tag));
	CHECK(n == 3u);
	CHECK(out[0].x == 1.0f);
}

/* ---- Release ORDER, not just release ------------------------------------
 *
 * Regression test for the defect measured on hardware 2026-09-03: several
 * groups releasable at once came out in TABLE order (slot allocation order,
 * i.e. unrelated to time), and tdoa_gw's filter derives its `dt` from the
 * difference between consecutive groups' reference timestamps -- so an
 * inverted release fed a constant-velocity filter a negative or
 * double-counted time step. The observed symptom was three groups drained in
 * one gateway cycle with dt of 0.400, 0.200 and 0.600 s for 200 ms of real
 * elapsed time, and FILTERED fixes noisier than the raw solve.
 *
 * The scenario has to put the OLDER group in a HIGHER slot, or table order
 * and time order agree by accident and the test proves nothing. find_free()
 * hands out the lowest unused slot, so: open A (slot 0) and B (slot 1),
 * release A to free slot 0, then open C -- which lands in slot 0 while the
 * older B still sits in slot 1. */
static void test_release_is_oldest_first(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 3);

	/* A: complete at t=0, takes slot 0. */
	o = obs(0x0100, 1, 0, 0.0f, 0.0f, 1000); CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0100, 1, 1, 10.0f, 0.0f, 1000); CHECK(tdoa_collect_add(&c, &o, 0));
	o = obs(0x0100, 1, 2, 10.0f, 10.0f, 1000); CHECK(tdoa_collect_add(&c, &o, 0));

	/* B opens at t=10 and takes slot 1 -- still incomplete. */
	o = obs(0x0100, 2, 0, 0.0f, 0.0f, 2000); CHECK(tdoa_collect_add(&c, &o, 10));

	/* Drain A, freeing slot 0. */
	CHECK(tdoa_collect_take_ready(&c, 10, out, &n_out, &tag_out));
	CHECK(out[0].t_dtu == 1000);

	/* C: complete at t=20, and it lands in the slot A vacated -- so the
	 * NEWER group is now at a LOWER index than the older B. */
	o = obs(0x0100, 3, 0, 0.0f, 0.0f, 3000); CHECK(tdoa_collect_add(&c, &o, 20));
	o = obs(0x0100, 3, 1, 10.0f, 0.0f, 3000); CHECK(tdoa_collect_add(&c, &o, 20));
	o = obs(0x0100, 3, 2, 10.0f, 10.0f, 3000); CHECK(tdoa_collect_add(&c, &o, 20));

	/* Complete B, which is older (first_ms 10 against C's 20). */
	o = obs(0x0100, 2, 1, 10.0f, 0.0f, 2000); CHECK(tdoa_collect_add(&c, &o, 20));
	o = obs(0x0100, 2, 2, 10.0f, 10.0f, 2000); CHECK(tdoa_collect_add(&c, &o, 20));

	/* Both releasable. Table order would hand back C (slot 0) first; the
	 * contract is oldest-first, so B must come out before C. This is the
	 * assertion that fails against the pre-2026-09-03 scan. */
	CHECK(tdoa_collect_take_ready(&c, 20, out, &n_out, &tag_out));
	CHECK(out[0].t_dtu == 2000);

	CHECK(tdoa_collect_take_ready(&c, 20, out, &n_out, &tag_out));
	CHECK(out[0].t_dtu == 3000);

	CHECK(!tdoa_collect_take_ready(&c, 20, out, &n_out, &tag_out));
}

/* The same property over a full drain of several groups: whatever slots they
 * occupy, consecutive take_ready() calls must hand them back in
 * non-decreasing first_ms order -- which is what makes the caller's dt
 * positive. Groups are opened in an order that scrambles slot index against
 * time by draining and refilling in between. */
static void test_full_drain_is_monotonic_in_time(void)
{
	struct tdoa_collect c;
	struct tdoa_obs o;
	struct tdoa_meas out[POS_MAX_ANCHORS];
	size_t n_out;
	uint16_t tag_out;
	int64_t prev = 0;
	unsigned int i;

	tdoa_collect_init(&c);
	tdoa_collect_set_expected(&c, 3);

	/* Open five complete groups whose first_ms DESCENDS with slot index:
	 * seq 1 in slot 0 at t=50, seq 2 in slot 1 at t=40, and so on. Table
	 * order is then exactly the reverse of time order. */
	for (i = 0; i < 5u; i++) {
		uint8_t seq = (uint8_t)(i + 1u);
		uint32_t at = 50u - (i * 10u);
		int64_t ts  = (int64_t)at * 1000;

		o = obs(0x0100, seq, 0, 0.0f, 0.0f, ts);
		CHECK(tdoa_collect_add(&c, &o, at));
		o = obs(0x0100, seq, 1, 10.0f, 0.0f, ts);
		CHECK(tdoa_collect_add(&c, &o, at));
		o = obs(0x0100, seq, 2, 10.0f, 10.0f, ts);
		CHECK(tdoa_collect_add(&c, &o, at));
	}

	for (i = 0; i < 5u; i++) {
		CHECK(tdoa_collect_take_ready(&c, 60, out, &n_out, &tag_out));
		CHECK(out[0].t_dtu > prev);
		prev = out[0].t_dtu;
	}
	CHECK(!tdoa_collect_take_ready(&c, 60, out, &n_out, &tag_out));
}

int main(void)
{
	test_groups_by_tag_and_seq();
	test_two_tags_do_not_mix();
	test_below_minimum_is_not_ready();
	test_window_expiry_drops_the_group();
	test_window_expiry_at_exact_boundary();
	test_slot_exhaustion_drops_oldest();
	test_slot_exhaustion_protects_a_complete_group();
	test_slot_exhaustion_all_releasable_rejects_newcomer();
	test_duplicate_anchor_is_ignored();
	test_ms_clock_wrap();
	test_collector_feeds_solver();
	test_set_expected_releases_early();
	test_set_expected_pos_max_matches_default();
	test_set_expected_clamps_out_of_range();
	test_set_expected_does_not_affect_minimum_discard();
	test_reference_anchor_is_deterministic();
	test_reference_is_lowest_present_not_lowest_possible();
	test_release_is_oldest_first();
	test_full_drain_is_monotonic_in_time();
	if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
	printf("tdoa_collect: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

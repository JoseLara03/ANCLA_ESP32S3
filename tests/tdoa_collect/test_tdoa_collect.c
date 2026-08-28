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

int main(void)
{
	test_groups_by_tag_and_seq();
	test_two_tags_do_not_mix();
	test_below_minimum_is_not_ready();
	test_window_expiry_drops_the_group();
	test_slot_exhaustion_drops_oldest();
	test_duplicate_anchor_is_ignored();
	test_ms_clock_wrap();
	test_collector_feeds_solver();
	if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
	printf("tdoa_collect: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

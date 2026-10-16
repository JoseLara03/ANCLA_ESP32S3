#include "sync_link.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Gateway (peer 0, addr 0) plus anchors at addr 1..4 with distinct EUIs. */
static void make_table(struct apos_table *t)
{
	apos_table_init(t);
	for (uint8_t i = 0; i < 5; i++) {
		uint8_t eui[APOS_EUI_LEN] = {0xA0, 0, 0, 0, 0, 0, 0, i};

		apos_table_add_peer(t, eui, i, false, 0, 0, 0);
	}
}

static void make_result(struct apos_result *r, const float xyz[5][3])
{
	memset(r, 0, sizeof(*r));
	r->n_nodes = 5;
	r->n_placed = 5;
	r->dim = APOS_GEOM_3D;
	for (int i = 0; i < 5; i++) {
		r->node[i].x = xyz[i][0];
		r->node[i].y = xyz[i][1];
		r->node[i].z = xyz[i][2];
		r->node[i].state = APOS_NODE_PLACED;
	}
}

static const float G[5][3] = {
	{0.0f, 0.0f, 0.0f},      /* gateway, root */
	{3.0f, 0.0f, 0.0f},      /* 3.000 m */
	{0.0f, 4.0f, 0.0f},      /* 4.000 m */
	{3.0f, 4.0f, 0.0f},      /* 5.000 m */
	{1.0f, 2.0f, 2.0f},      /* 3.000 m, off the plane: needs z */
};

static void test_known_distances_and_rounding(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];

	make_table(&t);
	make_result(&r, G);
	CHECK(sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX) == 4);

	/* 3000 mm / 4.690 = 639.66 -> 640 ; 4000 -> 852.88 -> 853 ;
	 * 5000 -> 1066.10 -> 1066 ; the 3-4-... off-plane node is exactly 3 m. */
	CHECK(l[0].geom_mm == 3000 && l[0].tau_dtu == 640);
	CHECK(l[1].geom_mm == 4000 && l[1].tau_dtu == 853);
	CHECK(l[2].geom_mm == 5000 && l[2].tau_dtu == 1066);
	CHECK(l[3].geom_mm == 3000 && l[3].tau_dtu == 640);
	for (int k = 0; k < 4; k++) {
		CHECK(l[k].valid && l[k].source == SYNC_LINK_SRC_SURVEY_STATIC);
		CHECK(!l[k].checked && !l[k].within_gate);
	}
}

static void test_height_enters_the_distance(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];
	float flat[5][3];

	memcpy(flat, G, sizeof(flat));
	flat[4][2] = 0.0f; /* same node, z dropped: 2.236 m instead of 3 m */
	make_table(&t);
	make_result(&r, flat);
	CHECK(sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX) == 4);
	CHECK(l[3].geom_mm == 2236);
	CHECK(l[3].tau_dtu != 640);
}

static void test_identity_is_eui_not_address(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];

	make_table(&t);
	make_result(&r, G);
	sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX);
	CHECK(l[1].child_eui[7] == 2 && l[1].child_addr == 2);
	CHECK(memcmp(l[1].parent_eui, t.peer[0].eui, APOS_EUI_LEN) == 0);
}

static void test_measured_range_is_checked_against_the_gate(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];

	make_table(&t);
	make_result(&r, G);
	/* gateway -> anchor 1: 40 mm over (inside the 150 mm gate) */
	apos_table_add_meas(&t, 0, 1, 3040, 12, 20);
	/* gateway -> anchor 2: 400 mm over (outside) */
	apos_table_add_meas(&t, 0, 2, 4400, 30, 20);
	/* anchor 3: too few samples to trust -> unchecked, not "passed" */
	apos_table_add_meas(&t, 0, 3, 5000, 10, 2);
	/* anchor 4: nothing at all */

	CHECK(sync_link_derive(&t, &r, 0, 10, l, SYNC_LINK_MAX) == 4);
	CHECK(l[0].checked && l[0].err_mm == 40 && l[0].within_gate);
	CHECK(l[1].checked && l[1].err_mm == 400 && !l[1].within_gate);
	CHECK(!l[2].checked && !l[2].within_gate);
	CHECK(!l[3].checked && !l[3].within_gate);
	CHECK(sync_link_count_good(l, 4) == 1);
	CHECK(SYNC_LINK_GATE_MM == 150);
}

static void test_unplaced_peer_has_no_tau(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];

	make_table(&t);
	make_result(&r, G);
	r.node[2].state = APOS_NODE_UNPLACED;
	CHECK(sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX) == 4);
	CHECK(!l[1].valid && l[1].tau_dtu == 0 && l[1].source == SYNC_LINK_SRC_NONE);
	CHECK(l[0].valid);
}

static void test_bad_arguments(void)
{
	struct apos_table t;
	struct apos_result r;
	struct sync_link l[SYNC_LINK_MAX];

	make_table(&t);
	make_result(&r, G);
	CHECK(sync_link_derive(NULL, &r, 0, 1, l, SYNC_LINK_MAX) < 0);
	CHECK(sync_link_derive(&t, &r, 9, 1, l, SYNC_LINK_MAX) < 0);
	r.node[0].state = APOS_NODE_UNPLACED; /* unplaced root */
	CHECK(sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX) < 0);
	r.node[0].state = APOS_NODE_PLACED;
	r.n_nodes = 4; /* result does not describe the table */
	CHECK(sync_link_derive(&t, &r, 0, 1, l, SYNC_LINK_MAX) < 0);
	r.n_nodes = 5;
	CHECK(sync_link_derive(&t, &r, 0, 1, l, 2) == 2); /* cap honoured */
	CHECK(sync_link_mm_to_dtu(-5) == 0);
}

/* tau is what feeds sync_model_set_tau(): it must be the ONE-WAY flight, so
 * doubling it (e.g. by also adding a TWR round trip) is a different number. */
static void test_tau_is_one_way_not_round_trip(void)
{
	CHECK(sync_link_mm_to_dtu(3000) == 640);
	CHECK(sync_link_mm_to_dtu(6000) == 1279); /* a round trip would be 2x */
	CHECK(sync_link_mm_to_dtu(6000) != 2 * sync_link_mm_to_dtu(3000) + 1);
}

int main(void)
{
	test_known_distances_and_rounding();
	test_height_enters_the_distance();
	test_identity_is_eui_not_address();
	test_measured_range_is_checked_against_the_gate();
	test_unplaced_peer_has_no_tau();
	test_bad_arguments();
	test_tau_is_one_way_not_round_trip();
	if (g_fail) {
		printf("%d CHECK(s) FAILED\n", g_fail);
		return 1;
	}
	printf("sync_link: ALL TESTS PASSED\n");
	return 0;
}

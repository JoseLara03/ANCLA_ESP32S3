/*
 * Host tests for apos_addrmap: the persisted EUI -> address map and the
 * assignment policy `apos assign` runs (spec 2026-10-21 D4/D6).
 *
 *   gcc -Wall -Wextra -Isrc -Itests/ccp_frame/shim -o tests/apos_addrmap/t.exe \
 *       tests/apos_addrmap/test_apos_addrmap.c src/apos_addrmap.c
 *
 * Mutation-checked when written: inverting the rule-3 tie-break, dropping
 * rule 2 (always re-assign), letting rule 4 hand out a reserved address, and
 * returning a partial plan on -ENOSPC each break at least one case below.
 */
#include "apos_addrmap.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

#define LO 0x0001u   /* UWB_ANCHOR_ADDR_BASE */
#define HI 0x0005u   /* BASE + UWB_MAX_ANCHORS (4) */
#define UNASSIGNED 0x00FFu

/* EUI with a chosen last byte; everything else fixed. Order of `k` is the
 * byte-wise EUI order. */
static void eui(uint8_t out[APOS_EUI_LEN], uint8_t k)
{
	const uint8_t base[APOS_EUI_LEN] = { 0x7C, 0xDF, 0xA1, 0xB2, 0xC3, 0x00, 0x00, 0x00 };

	memcpy(out, base, APOS_EUI_LEN);
	out[5] = k;
}

static void peer(struct apos_plan_peer *p, uint8_t k, uint16_t addr)
{
	eui(p->eui, k);
	p->addr = addr;
}

static void map_put(struct apos_addrmap *m, uint8_t k, uint16_t addr)
{
	uint8_t e[APOS_EUI_LEN];

	eui(e, k);
	CHECK(apos_addrmap_put(m, e, addr) == 0);
}

/* ---- codec ---- */

static void test_codec_round_trip(void)
{
	struct apos_addrmap m, back;
	uint8_t buf[APOS_ADDRMAP_MAX_LEN];

	for (uint8_t n = 0; n <= APOS_ADDRMAP_MAX; n++) {
		memset(&m, 0, sizeof(m));
		for (uint8_t k = 0; k < n; k++) {
			map_put(&m, (uint8_t)(k + 1u), (uint16_t)(0x0001u + k));
		}
		int len = apos_addrmap_encode(&m, buf, sizeof(buf));

		CHECK(len == (int)(APOS_ADDRMAP_HDR_LEN + n * APOS_ADDRMAP_ENTRY_LEN));
		memset(&back, 0xAA, sizeof(back));
		CHECK(apos_addrmap_decode(buf, (size_t)len, &back) == 0);
		CHECK(back.n == n);
		for (uint8_t k = 0; k < n; k++) {
			CHECK(memcmp(back.e[k].eui, m.e[k].eui, APOS_EUI_LEN) == 0);
			CHECK(back.e[k].addr == m.e[k].addr);
		}
	}

	/* Raw layout: version, n, EUI, addr little-endian. */
	memset(&m, 0, sizeof(m));
	map_put(&m, 9, 0x0102);
	CHECK(apos_addrmap_encode(&m, buf, sizeof(buf)) == 12);
	CHECK(buf[0] == APOS_ADDRMAP_VERSION && buf[1] == 1);
	CHECK(buf[2] == 0x7C && buf[7] == 9);
	CHECK(buf[10] == 0x02 && buf[11] == 0x01);
	CHECK(apos_addrmap_encode(&m, buf, 11) == -EMSGSIZE);
}

static void test_codec_rejects(void)
{
	struct apos_addrmap m, keep, out;
	uint8_t buf[APOS_ADDRMAP_MAX_LEN];
	int len;

	memset(&m, 0, sizeof(m));
	map_put(&m, 1, 0x0001);
	map_put(&m, 2, 0x0002);
	len = apos_addrmap_encode(&m, buf, sizeof(buf));
	memset(&keep, 0x5A, sizeof(keep));

	/* Every truncation and one trailing byte. */
	for (int l = 0; l < len; l++) {
		out = keep;
		CHECK(apos_addrmap_decode(buf, (size_t)l, &out) == -EBADMSG);
		CHECK(memcmp(&out, &keep, sizeof(out)) == 0);   /* untouched */
	}
	buf[len] = 0;
	CHECK(apos_addrmap_decode(buf, (size_t)len + 1u, &out) == -EBADMSG);

	/* Unknown version is NOT reinterpreted. */
	buf[0] = APOS_ADDRMAP_VERSION + 1u;
	out = keep;
	CHECK(apos_addrmap_decode(buf, (size_t)len, &out) == -ENOTSUP);
	CHECK(memcmp(&out, &keep, sizeof(out)) == 0);
	buf[0] = APOS_ADDRMAP_VERSION;

	/* n over the cap. */
	buf[1] = APOS_ADDRMAP_MAX + 1u;
	CHECK(apos_addrmap_decode(buf, (size_t)len, &out) == -EBADMSG);
	buf[1] = 2;

	/* Repeated address. */
	buf[2 + 10 + 8] = 0x01;
	CHECK(apos_addrmap_decode(buf, (size_t)len, &out) == -EBADMSG);
	buf[2 + 10 + 8] = 0x02;

	/* Repeated EUI. */
	buf[2 + 10 + 5] = 1;
	CHECK(apos_addrmap_decode(buf, (size_t)len, &out) == -EBADMSG);
	buf[2 + 10 + 5] = 2;

	/* All-zero EUI. */
	memset(&buf[2], 0, APOS_EUI_LEN);
	CHECK(apos_addrmap_decode(buf, (size_t)len, &out) == -EBADMSG);

	CHECK(apos_addrmap_decode(NULL, 2, &out) == -EINVAL);
}

static void test_put_find_forget(void)
{
	struct apos_addrmap m;
	uint8_t e1[APOS_EUI_LEN], e2[APOS_EUI_LEN], z[APOS_EUI_LEN] = {0};

	memset(&m, 0, sizeof(m));
	eui(e1, 1);
	eui(e2, 2);
	CHECK(apos_addrmap_put(&m, e1, 0x0001) == 0);
	CHECK(apos_addrmap_put(&m, e2, 0x0001) == -EADDRINUSE);
	CHECK(apos_addrmap_put(&m, e2, 0x0002) == 0);
	CHECK(apos_addrmap_put(&m, e1, 0x0003) == 0);           /* move */
	CHECK(m.n == 2 && apos_addrmap_find_addr(&m, 0x0001) < 0);
	CHECK(apos_addrmap_find_eui(&m, e1) == 0 && m.e[0].addr == 0x0003);
	CHECK(apos_addrmap_put(&m, z, 0x0004) == -EINVAL);
	CHECK(apos_addrmap_forget(&m, e1) == 0);
	CHECK(m.n == 1 && apos_addrmap_find_eui(&m, e2) == 0);
	CHECK(apos_addrmap_forget(&m, e1) == -ENOENT);

	memset(&m, 0, sizeof(m));
	for (uint8_t k = 0; k < APOS_ADDRMAP_MAX; k++) {
		map_put(&m, (uint8_t)(k + 1u), (uint16_t)(0x10u + k));
	}
	eui(e1, 0x40);
	CHECK(apos_addrmap_put(&m, e1, 0x0001) == -ENOSPC);
}

/* ---- policy ---- */

/* H3 in host form: a hand-numbered fleet with no map is adopted, untouched. */
static void test_plan_adopts_a_hand_numbered_fleet(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[4];
	struct apos_plan_out o[4];
	uint8_t na = 99;

	memset(&m, 0, sizeof(m));
	peer(&p[0], 4, 0x0001);
	peer(&p[1], 3, 0x0002);
	peer(&p[2], 2, 0x0003);
	peer(&p[3], 1, 0x0004);
	CHECK(apos_addr_plan(&m, p, 4, LO, HI, o, &na) == 0);
	CHECK(na == 0);
	for (int i = 0; i < 4; i++) {
		CHECK(o[i].action == APOS_PLAN_ADOPT);
		CHECK(o[i].addr == p[i].addr);
	}
}

static void test_plan_map_rules(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[2];
	struct apos_plan_out o[2];
	uint8_t na;

	memset(&m, 0, sizeof(m));
	map_put(&m, 5, 0x0002);
	map_put(&m, 6, 0x0003);
	peer(&p[0], 5, 0x0002);    /* already there */
	peer(&p[1], 6, 0x0001);    /* moved by hand: put back */
	CHECK(apos_addr_plan(&m, p, 2, LO, HI, o, &na) == 0);
	CHECK(o[0].action == APOS_PLAN_KEEP && o[0].addr == 0x0002);
	CHECK(o[1].action == APOS_PLAN_ASSIGN && o[1].addr == 0x0003);
	CHECK(na == 1);
}

static void test_plan_duplicate_with_a_map_entry(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[2];
	struct apos_plan_out o[2];

	/* The map holder wins even though the newcomer has the LOWER EUI. */
	memset(&m, 0, sizeof(m));
	map_put(&m, 9, 0x0002);
	peer(&p[0], 1, 0x0002);
	peer(&p[1], 9, 0x0002);
	CHECK(apos_addr_plan(&m, p, 2, LO, HI, o, NULL) == 0);
	CHECK(o[1].action == APOS_PLAN_KEEP && o[1].addr == 0x0002);
	CHECK(o[0].action == APOS_PLAN_ASSIGN && o[0].addr == 0x0001);
}

static void test_plan_duplicate_without_a_map_entry(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[2];
	struct apos_plan_out o[2];

	memset(&m, 0, sizeof(m));
	peer(&p[0], 8, 0x0003);
	peer(&p[1], 2, 0x0003);
	CHECK(apos_addr_plan(&m, p, 2, LO, HI, o, NULL) == 0);
	CHECK(o[1].action == APOS_PLAN_ADOPT && o[1].addr == 0x0003);   /* lower EUI */
	CHECK(o[0].action == APOS_PLAN_ASSIGN && o[0].addr == 0x0001);
}

static void test_plan_unassigned_get_lowest_free_in_eui_order(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[3];
	struct apos_plan_out o[3];
	uint8_t na;

	memset(&m, 0, sizeof(m));
	map_put(&m, 0x30, 0x0001);   /* reserved for a board that is OFF today */
	peer(&p[0], 7, UNASSIGNED);
	peer(&p[1], 3, UNASSIGNED);
	peer(&p[2], 5, UNASSIGNED);
	CHECK(apos_addr_plan(&m, p, 3, LO, HI, o, &na) == 0);
	CHECK(na == 3);
	CHECK(o[1].action == APOS_PLAN_ASSIGN && o[1].addr == 0x0002);  /* EUI 3 */
	CHECK(o[2].action == APOS_PLAN_ASSIGN && o[2].addr == 0x0003);  /* EUI 5 */
	CHECK(o[0].action == APOS_PLAN_ASSIGN && o[0].addr == 0x0004);  /* EUI 7 */
}

static void test_plan_reservation_is_never_reassigned(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[1];
	struct apos_plan_out o[1];

	/* A board claims, by hand, the address reserved for an absent board. */
	memset(&m, 0, sizeof(m));
	map_put(&m, 0x30, 0x0001);
	peer(&p[0], 4, 0x0001);
	CHECK(apos_addr_plan(&m, p, 1, LO, HI, o, NULL) == 0);
	CHECK(o[0].action == APOS_PLAN_ASSIGN && o[0].addr == 0x0002);
}

static void test_plan_no_space_is_all_or_nothing(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[5];
	struct apos_plan_out o[5];
	uint8_t na = 99;

	/* Five boards, four addresses. */
	memset(&m, 0, sizeof(m));
	for (uint8_t i = 0; i < 5; i++) {
		peer(&p[i], (uint8_t)(i + 1u), UNASSIGNED);
	}
	CHECK(apos_addr_plan(&m, p, 5, LO, HI, o, &na) == -ENOSPC);
	CHECK(o[4].action == APOS_PLAN_NOSPACE);   /* highest EUI is the one left */
	for (int i = 0; i < 4; i++) {
		CHECK(o[i].action == APOS_PLAN_ASSIGN);
	}

	/* Map full of reservations: even a board with a free-looking address
	 * cannot be adopted. */
	memset(&m, 0, sizeof(m));
	for (uint8_t k = 0; k < APOS_ADDRMAP_MAX; k++) {
		map_put(&m, (uint8_t)(0x40u + k), (uint16_t)(0x20u + k));
	}
	peer(&p[0], 1, 0x0001);
	CHECK(apos_addr_plan(&m, p, 1, LO, HI, o, NULL) == -ENOSPC);
	CHECK(o[0].action == APOS_PLAN_NOSPACE);
}

static void test_plan_is_order_independent(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[4], q[4];
	struct apos_plan_out o[4], r[4];
	static const int perm[4] = { 2, 0, 3, 1 };

	memset(&m, 0, sizeof(m));
	map_put(&m, 6, 0x0004);
	peer(&p[0], 9, 0x0002);        /* dup with EUI 2 */
	peer(&p[1], 2, 0x0002);
	peer(&p[2], 6, UNASSIGNED);    /* mapped */
	peer(&p[3], 4, UNASSIGNED);
	CHECK(apos_addr_plan(&m, p, 4, LO, HI, o, NULL) == 0);
	for (int i = 0; i < 4; i++) {
		q[i] = p[perm[i]];
	}
	CHECK(apos_addr_plan(&m, q, 4, LO, HI, r, NULL) == 0);
	for (int i = 0; i < 4; i++) {
		CHECK(r[i].action == o[perm[i]].action);
		CHECK(r[i].addr == o[perm[i]].addr);
	}
	/* And the outcome is the expected one. */
	CHECK(o[1].action == APOS_PLAN_ADOPT && o[1].addr == 0x0002);
	CHECK(o[2].action == APOS_PLAN_ASSIGN && o[2].addr == 0x0004);
	CHECK(o[3].action == APOS_PLAN_ASSIGN && o[3].addr == 0x0001);
	CHECK(o[0].action == APOS_PLAN_ASSIGN && o[0].addr == 0x0003);
}

static void test_plan_rejects_bad_input(void)
{
	struct apos_addrmap m;
	struct apos_plan_peer p[2];
	struct apos_plan_out o[2];

	memset(&m, 0, sizeof(m));
	peer(&p[0], 1, UNASSIGNED);
	peer(&p[1], 1, UNASSIGNED);   /* repeated EUI */
	CHECK(apos_addr_plan(&m, p, 2, LO, HI, o, NULL) == -EINVAL);
	memset(p[0].eui, 0, APOS_EUI_LEN);
	CHECK(apos_addr_plan(&m, p, 1, LO, HI, o, NULL) == -EINVAL);
	peer(&p[0], 1, UNASSIGNED);
	CHECK(apos_addr_plan(&m, p, 1, HI, LO, o, NULL) == -EINVAL);
	CHECK(apos_addr_plan(&m, p, APOS_ADDRMAP_MAX + 1u, LO, HI, o, NULL) == -EINVAL);
}

int main(void)
{
	test_codec_round_trip();
	test_codec_rejects();
	test_put_find_forget();
	test_plan_adopts_a_hand_numbered_fleet();
	test_plan_map_rules();
	test_plan_duplicate_with_a_map_entry();
	test_plan_duplicate_without_a_map_entry();
	test_plan_unassigned_get_lowest_free_in_eui_order();
	test_plan_reservation_is_never_reassigned();
	test_plan_no_space_is_all_or_nothing();
	test_plan_is_order_independent();
	test_plan_rejects_bad_input();
	if (g_fail) {
		printf("apos_addrmap: %d FAILED\n", g_fail);
		return 1;
	}
	printf("apos_addrmap: ALL TESTS PASSED\n");
	return 0;
}

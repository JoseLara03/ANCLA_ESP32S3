#include "apos_links.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static struct apos_links sample(void)
{
	struct apos_links l;

	memset(&l, 0, sizeof(l));
	l.generation = 0xA1B2C3D4u;
	l.n = 3;
	for (uint8_t k = 0; k < 3; k++) {
		memset(l.rec[k].child_eui, 0x10 + k, APOS_EUI_LEN);
		memset(l.rec[k].parent_eui, 0xEE, APOS_EUI_LEN);
		l.rec[k].tau_dtu = (k == 1) ? 0 : (k == 2 ? -37 : 640);
		l.rec[k].valid = true;
		l.rec[k].source = 1;
	}
	return l;
}

static void test_round_trip_keeps_zero_tau_valid(void)
{
	struct apos_links in = sample(), out;
	uint8_t buf[APOS_LINKS_MAX_LEN];
	int n = apos_links_encode(&in, buf, sizeof(buf));

	CHECK(n == (int)(APOS_LINKS_HDR_LEN + 3 * APOS_LINK_ENTRY_LEN));
	CHECK(buf[0] == APOS_LINKS_VERSION);
	/* generation is little-endian on the wire, not host-order memcpy */
	CHECK(buf[1] == 0xD4 && buf[2] == 0xC3 && buf[3] == 0xB2 && buf[4] == 0xA1);
	CHECK(apos_links_decode(buf, (size_t)n, &out) == 0);
	CHECK(out.generation == in.generation && out.n == 3);
	CHECK(out.rec[1].tau_dtu == 0 && out.rec[1].valid);   /* 0 != "absent" */
	CHECK(out.rec[2].tau_dtu == -37);
	CHECK(memcmp(out.rec[0].child_eui, in.rec[0].child_eui, APOS_EUI_LEN) == 0);
}

static void test_every_truncation_is_refused_and_leaves_out_untouched(void)
{
	struct apos_links in = sample(), out;
	uint8_t buf[APOS_LINKS_MAX_LEN + 1];
	int n = apos_links_encode(&in, buf, sizeof(buf));

	memset(&out, 0x5A, sizeof(out));
	for (int len = 0; len < n; len++) {
		CHECK(apos_links_decode(buf, (size_t)len, &out) != 0);
	}
	CHECK(apos_links_decode(buf, (size_t)n + 1u, &out) == -EBADMSG); /* trailing */
	CHECK(out.generation == 0x5A5A5A5Au);
}

static void test_unknown_version_is_rejected_not_reinterpreted(void)
{
	struct apos_links in = sample(), out;
	uint8_t buf[APOS_LINKS_MAX_LEN];
	int n = apos_links_encode(&in, buf, sizeof(buf));

	buf[0] = APOS_LINKS_VERSION + 1u;
	CHECK(apos_links_decode(buf, (size_t)n, &out) == -ENOTSUP);
	buf[0] = 0;
	CHECK(apos_links_decode(buf, (size_t)n, &out) == -ENOTSUP);
}

static void test_count_and_flag_corruption(void)
{
	struct apos_links in = sample(), out;
	uint8_t buf[APOS_LINKS_MAX_LEN];
	int n = apos_links_encode(&in, buf, sizeof(buf));

	buf[5] = APOS_LINKS_MAX + 1u; /* claims more than fits */
	CHECK(apos_links_decode(buf, (size_t)n, &out) == -EBADMSG);
	buf[5] = 3;
	buf[APOS_LINKS_HDR_LEN + 2 * APOS_EUI_LEN + 4] = 2; /* valid byte = 2 */
	CHECK(apos_links_decode(buf, (size_t)n, &out) == -EBADMSG);
}

static void test_encode_bounds(void)
{
	struct apos_links in = sample();
	uint8_t small[8];
	uint8_t buf[APOS_LINKS_MAX_LEN];

	CHECK(apos_links_encode(&in, small, sizeof(small)) == -EMSGSIZE);
	in.n = APOS_LINKS_MAX + 1u;
	CHECK(apos_links_encode(&in, buf, sizeof(buf)) == -EINVAL);
	CHECK(apos_links_encode(NULL, buf, sizeof(buf)) == -EINVAL);
	in.n = 0; /* an empty set is a legal record */
	CHECK(apos_links_encode(&in, buf, sizeof(buf)) == (int)APOS_LINKS_HDR_LEN);
	CHECK(apos_links_decode(buf, APOS_LINKS_HDR_LEN, &in) == 0 && in.n == 0);
}

static void test_find_is_by_eui(void)
{
	struct apos_links l = sample();
	uint8_t e[APOS_EUI_LEN];

	memset(e, 0x11, sizeof(e));
	CHECK(apos_links_find(&l, e) == &l.rec[1]);
	memset(e, 0x77, sizeof(e));
	CHECK(apos_links_find(&l, e) == NULL);
}

static void test_pending_round_trip_and_rejects(void)
{
	struct apos_pending p = {.generation = 9, .state = APOS_PENDING_APPLYING};
	struct apos_pending o;
	uint8_t b[APOS_PENDING_LEN];

	CHECK(apos_pending_encode(&p, b, sizeof(b)) == (int)APOS_PENDING_LEN);
	CHECK(apos_pending_decode(b, sizeof(b), &o) == 0);
	CHECK(o.generation == 9 && o.state == APOS_PENDING_APPLYING);
	CHECK(apos_pending_decode(b, sizeof(b) - 1u, &o) == -EBADMSG);
	b[0] = 9;
	CHECK(apos_pending_decode(b, sizeof(b), &o) == -ENOTSUP);
	b[0] = APOS_LINKS_VERSION;
	b[5] = 7;
	CHECK(apos_pending_decode(b, sizeof(b), &o) == -EBADMSG);
}

int main(void)
{
	test_round_trip_keeps_zero_tau_valid();
	test_every_truncation_is_refused_and_leaves_out_untouched();
	test_unknown_version_is_rejected_not_reinterpreted();
	test_count_and_flag_corruption();
	test_encode_bounds();
	test_find_is_by_eui();
	test_pending_round_trip_and_rejects();
	if (g_fail) {
		printf("%d CHECK(s) FAILED\n", g_fail);
		return 1;
	}
	printf("apos_links: ALL TESTS PASSED\n");
	return 0;
}

/*
 * Host tests for ccp_frame.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/ccp_frame/test_ccp_frame.exe \
 *       tests/ccp_frame/test_ccp_frame.c src/ccp_frame.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "ccp_frame.h"

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

/* The function code is a WIRE value allocated across two repositories, so it is
 * pinned here: changing it must be a deliberate edit to a test. 0xEF is also
 * the last free code in the range -- see the allocation table in ccp_frame.h. */
static void test_wire_constants(void)
{
	CHECK(CCP_FRAME_TYPE == 0xEF);
	CHECK(CCP_FRAME_LEN == 21);
	CHECK(CCP_OFF_SEQ == 10);
	CHECK(CCP_OFF_HOP == 11);
	CHECK(CCP_OFF_TX_DTU == 12);
	CHECK(CCP_OFF_ROOT_ID == 17);
	/* The 40-bit timestamp must not overlap root_id. */
	CHECK(CCP_OFF_TX_DTU + 5 == CCP_OFF_ROOT_ID);
	CHECK(CCP_OFF_ROOT_ID + 4 == CCP_FRAME_LEN);
}

static void test_roundtrip(void)
{
	uint8_t buf[CCP_FRAME_LEN];
	struct ccp_frame in = {
		.seq     = 0x5A,
		.hop     = 1,
		.tx_dtu  = 0xA1B2C3D4E5ULL,     /* all five bytes distinct */
		.root_id = 0x12345678u,
	};
	struct ccp_frame out;

	CHECK(ccp_frame_build(buf, sizeof(buf), 0x0003, 0x77, &in) ==
	      (int)CCP_FRAME_LEN);

	/* Header, byte for byte, matching what every other codec here emits. */
	CHECK(buf[0] == 0x41 && buf[1] == 0x88);
	CHECK(buf[2] == 0x77);
	CHECK(buf[3] == 0xCA && buf[4] == 0xDE);
	CHECK(buf[5] == 0xFF && buf[6] == 0xFF);      /* broadcast */
	CHECK(buf[7] == 0x03 && buf[8] == 0x00);      /* src, little-endian */
	CHECK(buf[9] == CCP_FRAME_TYPE);

	CHECK(ccp_frame_is_ccp(buf, sizeof(buf)));
	CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == 0);

	CHECK(out.seq == in.seq);
	CHECK(out.hop == in.hop);
	CHECK(out.tx_dtu == in.tx_dtu);
	CHECK(out.root_id == in.root_id);
	CHECK(out.src_addr == 0x0003);
}

/* The whole 40-bit range must survive, including the top byte -- a timestamp
 * silently losing its high bits would place a sync observation 17 seconds out
 * and still parse cleanly. */
static void test_full_40_bit_range(void)
{
	const uint64_t vals[] = {
		0, 1, 0xFFULL, 0x100ULL, 0xFFFFFFFFULL, 0x100000000ULL,
		0x7FFFFFFFFFULL, 0xFFFFFFFFFFULL,
	};

	for (unsigned int i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
		uint8_t buf[CCP_FRAME_LEN];
		struct ccp_frame in = { .seq = 1, .hop = 0,
					.tx_dtu = vals[i], .root_id = 0xDEADBEEF };
		struct ccp_frame out;

		CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);
		CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == 0);
		CHECK(out.tx_dtu == vals[i]);
		CHECK(out.root_id == 0xDEADBEEF);
	}
}

/* A timestamp wider than 40 bits is a caller bug -- most likely a raw counter
 * rather than a masked device timestamp. It must be REFUSED, not truncated:
 * truncation parses cleanly at the far end and yields a plausible but wrong
 * observation, which is the worst failure this frame has. */
static void test_oversized_timestamp_is_refused(void)
{
	uint8_t buf[CCP_FRAME_LEN];
	struct ccp_frame in = { .seq = 1, .hop = 0,
				.tx_dtu = 0x10000000000ULL, .root_id = 0 };

	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) == -EINVAL);

	in.tx_dtu = 0xFFFFFFFFFFFFFFFFULL;
	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) == -EINVAL);

	/* The boundary itself is legal. */
	in.tx_dtu = 0xFFFFFFFFFFULL;
	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);
}

static void test_build_argument_checks(void)
{
	uint8_t buf[CCP_FRAME_LEN];
	struct ccp_frame in = { .seq = 1, .hop = 0, .tx_dtu = 1, .root_id = 0 };

	CHECK(ccp_frame_build(NULL, sizeof(buf), 1, 0, &in) == -EINVAL);
	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, NULL) == -EINVAL);
	CHECK(ccp_frame_build(buf, CCP_FRAME_LEN - 1, 1, 0, &in) == -EMSGSIZE);
	CHECK(ccp_frame_build(buf, 0, 1, 0, &in) == -EMSGSIZE);
}

/* Hop policy. Adopting an unsynced or too-deep master is the one mistake that
 * corrupts the time base while leaving every downstream conversion looking
 * plausible, so the parser refuses it rather than trusting the caller. */
static void test_hop_policy(void)
{
	CHECK(ccp_hop_adoptable(CCP_HOP_ROOT));
	CHECK(ccp_hop_adoptable(CCP_HOP_MAX));
	CHECK(!ccp_hop_adoptable(CCP_HOP_MAX + 1));
	CHECK(!ccp_hop_adoptable(CCP_HOP_UNSYNCED));

	uint8_t buf[CCP_FRAME_LEN];
	struct ccp_frame in = { .seq = 1, .hop = 0, .tx_dtu = 42, .root_id = 7 };
	struct ccp_frame out;

	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);

	/* Rewrite the hop on the wire, as a real too-deep or unsynced master
	 * would send it. */
	buf[CCP_OFF_HOP] = CCP_HOP_MAX;
	CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == 0);

	buf[CCP_OFF_HOP] = CCP_HOP_MAX + 1;
	CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == -EPROTO);

	buf[CCP_OFF_HOP] = CCP_HOP_UNSYNCED;
	CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == -EPROTO);
}

/* is_ccp() must tolerate a length that still includes the FCS. This project has
 * been bitten by that before: dwt_getframelength() and cb_data->datalength both
 * count the 2 FCS bytes, and an `== CCP_FRAME_LEN` test would drop every real
 * frame from a caller that had not subtracted them. */
static void test_length_tolerates_the_fcs(void)
{
	uint8_t buf[CCP_FRAME_LEN + 2];
	struct ccp_frame in = { .seq = 3, .hop = 0, .tx_dtu = 9, .root_id = 1 };
	struct ccp_frame out;

	memset(buf, 0xA5, sizeof(buf));
	CHECK(ccp_frame_build(buf, sizeof(buf), 2, 0, &in) > 0);

	CHECK(ccp_frame_is_ccp(buf, CCP_FRAME_LEN));
	CHECK(ccp_frame_is_ccp(buf, CCP_FRAME_LEN + 2));   /* FCS included */
	CHECK(!ccp_frame_is_ccp(buf, CCP_FRAME_LEN - 1));
	CHECK(ccp_frame_parse(buf, CCP_FRAME_LEN + 2, &out) == 0);
	CHECK(out.tx_dtu == 9);
}

static void test_rejects_foreign_frames(void)
{
	uint8_t buf[CCP_FRAME_LEN];
	struct ccp_frame in = { .seq = 1, .hop = 0, .tx_dtu = 1, .root_id = 0 };
	struct ccp_frame out;

	CHECK(ccp_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);

	/* Every other function code in the allocation must be rejected -- this
	 * is what stopped 0xEB's double allocation from being harmless. */
	const uint8_t others[] = { 0xE0, 0xE4, 0xE5, 0xEA, 0xEB, 0xEC, 0xED,
				   0xEE };

	for (unsigned int i = 0; i < sizeof(others); i++) {
		buf[9] = others[i];
		CHECK(!ccp_frame_is_ccp(buf, sizeof(buf)));
		CHECK(ccp_frame_parse(buf, sizeof(buf), &out) == -EINVAL);
	}
	buf[9] = CCP_FRAME_TYPE;

	/* Wrong PAN and wrong frame control. */
	buf[3] = 0x00;
	CHECK(!ccp_frame_is_ccp(buf, sizeof(buf)));
	buf[3] = 0xCA;
	buf[0] = 0x00;
	CHECK(!ccp_frame_is_ccp(buf, sizeof(buf)));

	CHECK(!ccp_frame_is_ccp(NULL, CCP_FRAME_LEN));
}

/* Airtime, so the cost of running CCPs is on the record rather than discovered
 * later. At PLEN_1024 / 850 kbps a 21-byte payload is ~1.29 ms, which against a
 * 200 ms superframe is 0.65 % -- cheap, but it is one more frame competing with
 * the beacon and Phase 3's superframe layout has to budget for it. */
static void test_airtime_is_recorded(void)
{
	/* preamble 1042 us + SFD 8 + PHR 22 + (21+2) bytes at 9.41 us/byte. */
	const unsigned int airtime_us = 1042 + 8 + 22 + (21 + 2) * 941 / 100;

	CHECK(airtime_us > 1250 && airtime_us < 1330);
	printf("  CCP airtime at PLEN_1024/850k: ~%u us, %.2f %% of a 200 ms"
	       " superframe\n", airtime_us, (double)airtime_us / 2000.0);
}

int main(void)
{
	test_wire_constants();
	test_roundtrip();
	test_full_40_bit_range();
	test_oversized_timestamp_is_refused();
	test_build_argument_checks();
	test_hop_policy();
	test_length_tolerates_the_fcs();
	test_rejects_foreign_frames();
	test_airtime_is_recorded();

	if (failures) {
		printf("\n%d CHECK(s) FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("ccp_frame: ALL TESTS PASSED\n");
	return EXIT_SUCCESS;
}

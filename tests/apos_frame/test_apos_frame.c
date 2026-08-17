#include "apos_frame.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static uint8_t buf[64];

static const uint8_t eui_ref[APOS_EUI_LEN] = {
    0xDE, 0xCA, 0x01, 0x02, 0x03, 0x04, 0x05, 0x77
};

/* Every APOS frame must carry the same first five bytes as every other frame on
 * this network, or a peer's is_valid() check rejects it before the subtype is
 * ever looked at. */
static void test_header_matches_the_network_convention(void)
{
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0x1234, 30);

    CHECK(n == (int)APOS_LEN_SURVEY_BEGIN);
    CHECK(buf[0] == 0x41);
    CHECK(buf[1] == 0x88);
    CHECK(buf[3] == 0xCA);
    CHECK(buf[4] == 0xDE);
    CHECK(buf[9] == APOS_FRAME_TYPE);
    CHECK(buf[10] == APOS_SUB_SURVEY_BEGIN);
}

static void test_addresses_are_little_endian(void)
{
    int n = apos_frame_range_cmd_build(buf, sizeof(buf), 0x0000, 0x0203,
                                       0x1234, 0x0004, 40);

    CHECK(n == (int)APOS_LEN_RANGE_CMD);
    CHECK(buf[5] == 0x03);   /* dest lo */
    CHECK(buf[6] == 0x02);   /* dest hi */
    CHECK(buf[7] == 0x00);   /* src  lo */
    CHECK(buf[8] == 0x00);   /* src  hi */
    CHECK(apos_frame_dest(buf) == 0x0203);
    CHECK(apos_frame_src(buf) == 0x0000);
}

static void test_survey_begin_round_trip(void)
{
    uint16_t session = 0, window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0xBEEF, 45);

    CHECK(n == (int)APOS_LEN_SURVEY_BEGIN);
    CHECK(apos_frame_dest(buf) == APOS_ADDR_BCAST);
    CHECK(apos_frame_parse_survey_begin(buf, (size_t)n, &session, &window) == 0);
    CHECK(session == 0xBEEF);
    CHECK(window == 45);
}

static void test_enum_rsp_round_trip(void)
{
    uint16_t session = 0;
    uint8_t eui[APOS_EUI_LEN] = {0};
    bool pv = false;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int n = apos_frame_enum_rsp_build(buf, sizeof(buf), 0x0003, 0x0000,
                                      0xBEEF, eui_ref, true,
                                      1.25f, -2.5f, 3.75f);

    CHECK(n == (int)APOS_LEN_ENUM_RSP);
    CHECK(apos_frame_parse_enum_rsp(buf, (size_t)n, &session, eui, &pv,
                                    &x, &y, &z) == 0);
    CHECK(session == 0xBEEF);
    CHECK(memcmp(eui, eui_ref, APOS_EUI_LEN) == 0);
    CHECK(pv == true);
    /* IEEE-754 round-trips exactly for these values -- no tolerance needed,
     * and an exact check catches a byte-order bug a tolerance would hide. */
    CHECK(x == 1.25f);
    CHECK(y == -2.5f);
    CHECK(z == 3.75f);
}

static void test_range_cmd_round_trip(void)
{
    uint16_t session = 0, peer = 0;
    uint8_t nex = 0;
    int n = apos_frame_range_cmd_build(buf, sizeof(buf), 0x0000, 0x0001,
                                       0xBEEF, 0x0004, 40);

    CHECK(n == (int)APOS_LEN_RANGE_CMD);
    CHECK(apos_frame_parse_range_cmd(buf, (size_t)n, &session, &peer, &nex) == 0);
    CHECK(session == 0xBEEF);
    CHECK(peer == 0x0004);
    CHECK(nex == 40);
}

/* mean_mm is signed: the initiator reports a negative distance when antenna
 * delays over-correct at very short range, and clamping it at zero would hide
 * a real calibration fault. */
static void test_range_rsp_round_trip_including_negative_mean(void)
{
    uint16_t session = 0, peer = 0, sd = 0;
    int32_t mean = 0;
    uint8_t n_ok = 0;
    int n = apos_frame_range_rsp_build(buf, sizeof(buf), 0x0001, 0x0000,
                                       0xBEEF, 0x0004, -37, 21, 39);

    CHECK(n == (int)APOS_LEN_RANGE_RSP);
    CHECK(apos_frame_parse_range_rsp(buf, (size_t)n, &session, &peer, &mean,
                                     &sd, &n_ok) == 0);
    CHECK(session == 0xBEEF);
    CHECK(peer == 0x0004);
    CHECK(mean == -37);
    CHECK(sd == 21);
    CHECK(n_ok == 39);
}

static void test_setpos_round_trip(void)
{
    uint16_t session = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int n = apos_frame_setpos_build(buf, sizeof(buf), 0x0000, 0x0002,
                                    0xBEEF, 3.5f, 4.25f, -0.5f);

    CHECK(n == (int)APOS_LEN_SETPOS);
    CHECK(apos_frame_parse_setpos(buf, (size_t)n, &session, &x, &y, &z) == 0);
    CHECK(session == 0xBEEF);
    CHECK(x == 3.5f);
    CHECK(y == 4.25f);
    CHECK(z == -0.5f);
}

static void test_setpos_ack_round_trip(void)
{
    uint16_t session = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    bool ok = false;
    int n = apos_frame_setpos_ack_build(buf, sizeof(buf), 0x0002, 0x0000,
                                        0xBEEF, 3.5f, 4.25f, -0.5f, true);

    CHECK(n == (int)APOS_LEN_SETPOS_ACK);
    CHECK(apos_frame_parse_setpos_ack(buf, (size_t)n, &session, &x, &y, &z,
                                      &ok) == 0);
    CHECK(session == 0xBEEF);
    CHECK(x == 3.5f);
    CHECK(ok == true);
}

static void test_survey_end_round_trip(void)
{
    uint16_t session = 0;
    int n = apos_frame_survey_end_build(buf, sizeof(buf), 0x0000, 0xBEEF);

    CHECK(n == (int)APOS_LEN_SURVEY_END);
    CHECK(apos_frame_dest(buf) == APOS_ADDR_BCAST);
    CHECK(apos_frame_parse_survey_end(buf, (size_t)n, &session) == 0);
    CHECK(session == 0xBEEF);
}

static void test_is_apos_rejects_other_traffic(void)
{
    /* A VEWA response: right PAN, wrong type. */
    const uint8_t vewa[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
                            0xE1, 0x02};

    CHECK(!apos_frame_is_apos(vewa, sizeof(vewa)));

    /* Right type, unknown subtype. */
    apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);
    buf[10] = 0x7F;
    CHECK(!apos_frame_is_apos(buf, APOS_LEN_SURVEY_BEGIN));

    /* Too short to hold a subtype at all. */
    CHECK(!apos_frame_is_apos(buf, APOS_HDR_LEN - 1u));
    CHECK(!apos_frame_is_apos(NULL, APOS_LEN_SURVEY_BEGIN));
}

/* A truncated frame must be refused, not parsed from whatever follows in the
 * caller's buffer. */
static void test_truncated_frames_are_refused(void)
{
    uint16_t session = 0xAAAA, window = 0xAAAA;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    CHECK(apos_frame_parse_survey_begin(buf, (size_t)(n - 1), &session,
                                        &window) == -EINVAL);
    /* Untouched on failure. */
    CHECK(session == 0xAAAA);
    CHECK(window == 0xAAAA);
}

/* An FCS left on the end is the classic bug on this project: flen from
 * dwt_getframelength() includes it. An over-long frame must be refused so the
 * mistake shows up immediately instead of shifting every field. */
static void test_extra_trailing_bytes_are_refused(void)
{
    uint16_t session = 0;
    uint16_t window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    CHECK(apos_frame_parse_survey_begin(buf, (size_t)(n + 2), &session,
                                        &window) == -EINVAL);
}

static void test_parser_rejects_the_wrong_subtype(void)
{
    uint16_t session = 0, peer = 0;
    uint8_t nex = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    /* Right length would never match anyway, but the subtype check must be
     * what rejects it -- otherwise a length collision between two subtypes
     * would silently mis-parse. */
    CHECK(apos_frame_parse_range_cmd(buf, (size_t)n, &session, &peer,
                                     &nex) == -EINVAL);
}

static void test_builders_reject_a_short_buffer(void)
{
    CHECK(apos_frame_enum_rsp_build(buf, 4, 0x0003, 0x0000, 1, eui_ref,
                                    true, 0.0f, 0.0f, 0.0f) == -EMSGSIZE);
    CHECK(apos_frame_survey_begin_build(NULL, sizeof(buf), 0x0000, 1,
                                        1) == -EINVAL);
    CHECK(apos_frame_enum_rsp_build(buf, sizeof(buf), 0x0003, 0x0000, 1, NULL,
                                    true, 0.0f, 0.0f, 0.0f) == -EINVAL);
}

/* Raw-byte vectors, independent of any get_*() accessor: a byte swap confined
 * to put_u32/get_u32 (used by mean_mm and by every xyz field via
 * put_f32/get_f32) cancels out in a build-then-parse round trip and would
 * pass every test above this one silently. These assert literal buf[n]
 * values instead. */

/* mean_mm's four bytes are all distinct and asymmetric (0x01020304), so a
 * byte swap of any kind -- full reversal, or a 16-bit word swap -- changes
 * the value and cannot hide. sd_mm (0x0506) is checked the same way. */
static void test_range_rsp_positive_mean_raw_bytes(void)
{
    int n = apos_frame_range_rsp_build(buf, sizeof(buf), 0x0001, 0x0000,
                                       0xBEEF, 0x0004, 0x01020304, 0x0506, 7);

    CHECK(n == (int)APOS_LEN_RANGE_RSP);
    /* header */
    CHECK(buf[0] == 0x41);
    CHECK(buf[1] == 0x88);
    CHECK(buf[3] == 0xCA);
    CHECK(buf[4] == 0xDE);
    CHECK(buf[5] == 0x00);   /* dest lo (0x0000) */
    CHECK(buf[6] == 0x00);   /* dest hi */
    CHECK(buf[7] == 0x01);   /* src  lo (0x0001) */
    CHECK(buf[8] == 0x00);   /* src  hi */
    CHECK(buf[9] == APOS_FRAME_TYPE);
    CHECK(buf[10] == APOS_SUB_RANGE_RSP);
    /* payload: session(2) peer(2) mean_mm(4) sd_mm(2) n_ok(1) */
    CHECK(buf[11] == 0xEF); CHECK(buf[12] == 0xBE);           /* session 0xBEEF */
    CHECK(buf[13] == 0x04); CHECK(buf[14] == 0x00);           /* peer 0x0004 */
    CHECK(buf[15] == 0x04); CHECK(buf[16] == 0x03);           /* mean_mm LE */
    CHECK(buf[17] == 0x02); CHECK(buf[18] == 0x01);
    CHECK(buf[19] == 0x06); CHECK(buf[20] == 0x05);           /* sd_mm LE */
    CHECK(buf[21] == 7);                                      /* n_ok */
}

/* -37 pins the two's-complement bytes on the wire: 0xFFFFFFDB little-endian
 * is DB FF FF FF. */
static void test_range_rsp_negative_mean_raw_bytes(void)
{
    int n = apos_frame_range_rsp_build(buf, sizeof(buf), 0x0001, 0x0000,
                                       0xBEEF, 0x0004, -37, 21, 39);

    CHECK(n == (int)APOS_LEN_RANGE_RSP);
    CHECK(buf[15] == 0xDB);
    CHECK(buf[16] == 0xFF);
    CHECK(buf[17] == 0xFF);
    CHECK(buf[18] == 0xFF);
}

/* x = 1.0f is 00 00 80 3F, y = 0.0f is 00 00 00 00 -- the exact pattern
 * observed on this project's hardware over a sniffer (see CLAUDE.md's
 * "Ranging confirmed on the bench" note). z = -1.0f is 00 00 80 BF, pinning
 * the sign bit too. Asserts the full built frame byte-for-byte. */
static void test_setpos_raw_bytes(void)
{
    int n = apos_frame_setpos_build(buf, sizeof(buf), 0x0000, 0x0002,
                                    0x1234, 1.0f, 0.0f, -1.0f);
    static const uint8_t expect[] = {
        0x41, 0x88, 0x00, 0xCA, 0xDE,   /* fixed header */
        0x02, 0x00,                     /* dest 0x0002 */
        0x00, 0x00,                     /* src  0x0000 */
        APOS_FRAME_TYPE,
        APOS_SUB_SETPOS,
        0x34, 0x12,                     /* session 0x1234 */
        0x00, 0x00, 0x80, 0x3F,         /* x = 1.0f */
        0x00, 0x00, 0x00, 0x00,         /* y = 0.0f */
        0x00, 0x00, 0x80, 0xBF,         /* z = -1.0f */
    };

    CHECK(n == (int)APOS_LEN_SETPOS);
    CHECK(sizeof(expect) == APOS_LEN_SETPOS);
    CHECK(memcmp(buf, expect, sizeof(expect)) == 0);
}

/* EUI-64 bytes must land in the order given -- byte order is exactly the
 * kind of thing two independent implementations can silently disagree
 * about. */
static void test_enum_rsp_eui_raw_bytes(void)
{
    int n = apos_frame_enum_rsp_build(buf, sizeof(buf), 0x0003, 0x0000,
                                      0xBEEF, eui_ref, true,
                                      1.25f, -2.5f, 3.75f);

    CHECK(n == (int)APOS_LEN_ENUM_RSP);
    /* payload: session(2) eui(8) pos_valid(1) xyz(12) */
    CHECK(buf[13] == 0xDE);
    CHECK(buf[14] == 0xCA);
    CHECK(buf[15] == 0x01);
    CHECK(buf[16] == 0x02);
    CHECK(buf[17] == 0x03);
    CHECK(buf[18] == 0x04);
    CHECK(buf[19] == 0x05);
    CHECK(buf[20] == 0x77);
    CHECK(buf[21] == 0x01);   /* pos_valid = true */
}

static void test_seq_is_settable_without_disturbing_the_payload(void)
{
    uint16_t session = 0, window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0xBEEF, 45);

    apos_frame_set_seq(buf, 0x5A);
    CHECK(buf[2] == 0x5A);
    CHECK(apos_frame_parse_survey_begin(buf, (size_t)n, &session, &window) == 0);
    CHECK(session == 0xBEEF);
    CHECK(window == 45);
}

int main(void)
{
    test_header_matches_the_network_convention();
    test_addresses_are_little_endian();
    test_survey_begin_round_trip();
    test_enum_rsp_round_trip();
    test_range_cmd_round_trip();
    test_range_rsp_round_trip_including_negative_mean();
    test_setpos_round_trip();
    test_setpos_ack_round_trip();
    test_survey_end_round_trip();
    test_range_rsp_positive_mean_raw_bytes();
    test_range_rsp_negative_mean_raw_bytes();
    test_setpos_raw_bytes();
    test_enum_rsp_eui_raw_bytes();
    test_is_apos_rejects_other_traffic();
    test_truncated_frames_are_refused();
    test_extra_trailing_bytes_are_refused();
    test_parser_rejects_the_wrong_subtype();
    test_builders_reject_a_short_buffer();
    test_seq_is_settable_without_disturbing_the_payload();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}

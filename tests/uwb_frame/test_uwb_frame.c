#include "uwb_frame_802_15_4z.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_scaffold(void)
{
    /* Constants are wired correctly. */
    CHECK(UWB_FRAME_TYPE_DISC == 0xE2);
    CHECK(UWB_FRAME_TYPE_MPOL == 0xE3);
    CHECK(UWB_FRAME_TYPE_RESP == 0xE4);
    CHECK(UWB_FRAME_LEN_DISC == 14);
    CHECK(UWB_FRAME_LEN_RESP == 20);
}

static void test_utilities(void)
{
    /* Hand-built frame: dest=0xBEEF (EF BE), src=0x1234 (34 12), seq=7. */
    uint8_t f[UWB_FRAME_HDR_LEN] = {
        0x41, 0x88, 0x07, 0xCA, 0xDE, 0xEF, 0xBE, 0x34, 0x12, UWB_FRAME_TYPE_DISC
    };
    CHECK(uwb_frame_get_seq_num(f)  == 0x07);
    CHECK(uwb_frame_get_dest_addr(f) == 0xBEEF);
    CHECK(uwb_frame_get_src_addr(f)  == 0x1234);

    uwb_frame_set_seq_num(f, 0x2A);
    CHECK(uwb_frame_get_seq_num(f) == 0x2A);
    CHECK(f[2] == 0x2A);  /* set is a plain setter, no auto-increment */
}

static void test_discovery(void)
{
    uint8_t buf[32];
    int n = uwb_frame_discovery_build(buf, sizeof(buf), 0x1234, 0xAABBCCDD);
    CHECK(n == UWB_FRAME_LEN_DISC);

    /* Exact byte layout. */
    uint8_t expect[UWB_FRAME_LEN_DISC] = {
        0x41, 0x88, 0x00, 0xCA, 0xDE,   /* FC, seq, PANID */
        0xFF, 0xFF,                     /* dest = broadcast */
        0x34, 0x12,                     /* src 0x1234 LE */
        UWB_FRAME_TYPE_DISC,            /* type 0xE2 */
        0xDD, 0xCC, 0xBB, 0xAA          /* tx_ts 0xAABBCCDD LE */
    };
    CHECK(memcmp(buf, expect, UWB_FRAME_LEN_DISC) == 0);

    /* Validators. */
    CHECK(uwb_frame_is_valid(buf, n));
    CHECK(uwb_frame_is_discovery(buf, n));
    CHECK(!uwb_frame_is_response(buf, n));   /* type byte differs */

    /* Buffer too small. */
    CHECK(uwb_frame_discovery_build(buf, 5, 0x1234, 0) == -EMSGSIZE);
    /* Null buffer. */
    CHECK(uwb_frame_discovery_build(NULL, sizeof(buf), 0x1234, 0) == -EINVAL);
}

static void test_response(void)
{
    uint8_t buf[32];
    /* src(anchor)=0x4583, dest(tag)=0x1234, ts=0x11223344,
     * cir_power=-5000, cir_quality=200. */
    int n = uwb_frame_response_build(buf, sizeof(buf), 0x4583, 0x1234,
                                     0x11223344, -5000, 200);
    CHECK(n == UWB_FRAME_LEN_RESP);

    uint8_t expect[UWB_FRAME_LEN_RESP] = {
        0x41, 0x88, 0x00, 0xCA, 0xDE,
        0x34, 0x12,                     /* dest 0x1234 (tag) LE */
        0x83, 0x45,                     /* src 0x4583 (anchor) LE */
        UWB_FRAME_TYPE_RESP,            /* 0xE4 */
        0x44, 0x33, 0x22, 0x11,         /* tx_ts LE */
        0x78, 0xEC, 0xFF, 0xFF,         /* cir_power -5000 (0xFFFFEC78) LE */
        0xC8, 0x00                      /* cir_quality 200 LE u16 */
    };
    CHECK(memcmp(buf, expect, UWB_FRAME_LEN_RESP) == 0);
    CHECK(uwb_frame_is_response(buf, n));
    CHECK(!uwb_frame_is_discovery(buf, n));

    /* Round-trip parse. */
    uint16_t src; int32_t cirp; uint16_t cirq;
    CHECK(uwb_frame_parse_discovery_response(buf, n, &src, &cirp, &cirq) == 0);
    CHECK(src == 0x4583);
    CHECK(cirp == -5000);
    CHECK(cirq == 200);

    /* Parser rejects a non-response frame. */
    uint8_t disc[32];
    int dn = uwb_frame_discovery_build(disc, sizeof(disc), 0x1234, 0);
    CHECK(uwb_frame_parse_discovery_response(disc, dn, &src, &cirp, &cirq) == -EBADMSG);

    /* Parser rejects null out-params. */
    CHECK(uwb_frame_parse_discovery_response(buf, n, NULL, &cirp, &cirq) == -EINVAL);

    /* Builder buffer too small. */
    CHECK(uwb_frame_response_build(buf, 10, 0x4583, 0x1234, 0, 0, 0) == -EMSGSIZE);
}

static void test_multipoll(void)
{
    struct uwb_anchor_slot slots[4] = {
        { 0x4583, 1000 }, { 0xA381, 1500 }, { 0x7F21, 2000 }, { 0x2048, 2500 }
    };
    uint8_t buf[40];

    /* 4-anchor case: exact layout, length 31. */
    int n = uwb_frame_multipoll_build(buf, sizeof(buf), 0x1234, slots, 4, 0x09ABCDEF);
    CHECK(n == 31);
    CHECK(n == UWB_FRAME_LEN_MPOL(4));
    uint8_t expect[31] = {
        0x41, 0x88, 0x00, 0xCA, 0xDE,
        0xFF, 0xFF,                 /* dest broadcast */
        0x34, 0x12,                 /* src 0x1234 */
        UWB_FRAME_TYPE_MPOL,        /* 0xE3 */
        0x04,                       /* num_anchors */
        0x83, 0x45, 0xE8, 0x03,     /* slot0: 0x4583, 1000 (0x03E8) */
        0x81, 0xA3, 0xDC, 0x05,     /* slot1: 0xA381, 1500 (0x05DC) */
        0x21, 0x7F, 0xD0, 0x07,     /* slot2: 0x7F21, 2000 (0x07D0) */
        0x48, 0x20, 0xC4, 0x09,     /* slot3: 0x2048, 2500 (0x09C4) */
        0xEF, 0xCD, 0xAB, 0x09      /* tx_ts 0x09ABCDEF LE, at offset 27 */
    };
    CHECK(memcmp(buf, expect, 31) == 0);
    CHECK(uwb_frame_is_multipoll(buf, n));

    /* Round-trip for every anchor count 1..4 (verifies packed tx_ts offset). */
    for (uint8_t k = 1; k <= 4; k++) {
        uint8_t b[40];
        int m = uwb_frame_multipoll_build(b, sizeof(b), 0x1234, slots, k, 0x09ABCDEF);
        CHECK(m == 15 + 4 * k);

        struct uwb_anchor_slot out[4];
        uint8_t num = 0; uint32_t ts = 0;
        CHECK(uwb_frame_parse_multipoll(b, m, out, &num, &ts) == 0);
        CHECK(num == k);
        CHECK(ts == 0x09ABCDEF);
        for (uint8_t i = 0; i < k; i++) {
            CHECK(out[i].addr == slots[i].addr);
            CHECK(out[i].delay_us == slots[i].delay_us);
        }
    }

    /* Illegal anchor counts. */
    CHECK(uwb_frame_multipoll_build(buf, sizeof(buf), 0x1234, slots, 0, 0) == -EINVAL);
    CHECK(uwb_frame_multipoll_build(buf, sizeof(buf), 0x1234, slots, 5, 0) == -EINVAL);
    /* Buffer too small for 4 slots. */
    CHECK(uwb_frame_multipoll_build(buf, 20, 0x1234, slots, 4, 0) == -EMSGSIZE);
    /* Null slots. */
    CHECK(uwb_frame_multipoll_build(buf, sizeof(buf), 0x1234, NULL, 4, 0) == -EINVAL);
}

static void test_corruption(void)
{
    uint8_t buf[32];
    int n = uwb_frame_response_build(buf, sizeof(buf), 0x4583, 0x1234, 0, 0, 0);
    CHECK(n == UWB_FRAME_LEN_RESP);

    /* Wrong frame-control. */
    uint8_t b1[32]; memcpy(b1, buf, n); b1[0] = 0x00;
    CHECK(!uwb_frame_is_valid(b1, n));

    /* Wrong PANID. */
    uint8_t b2[32]; memcpy(b2, buf, n); b2[3] = 0xBE;
    CHECK(!uwb_frame_is_valid(b2, n));

    /* Truncated below header length. */
    CHECK(!uwb_frame_is_valid(buf, 9));

    /* Oversized beyond max frame length. */
    CHECK(!uwb_frame_is_valid(buf, UWB_FRAME_MAX_LEN + 1));

    /* Right shape, wrong type for the specific predicate. */
    CHECK(!uwb_frame_is_multipoll(buf, n));   /* it's a RESP */
    CHECK(!uwb_frame_is_discovery(buf, n));

    /* Response too short for its own fields is rejected by is_response. */
    CHECK(!uwb_frame_is_response(buf, UWB_FRAME_LEN_RESP - 1));
}

static void test_new_constants(void)
{
    CHECK(UWB_FRAME_TYPE_BEACON    == 0xE5);
    CHECK(UWB_FRAME_TYPE_JOIN      == 0xE6);
    CHECK(UWB_FRAME_TYPE_GRANT     == 0xE7);
    CHECK(UWB_FRAME_TYPE_KEEPALIVE == 0xE8);
    CHECK(UWB_FRAME_TYPE_RELEASE   == 0xE9);
    CHECK(UWB_ADDR_GATEWAY == 0x0000);
    CHECK(UWB_ADDR_UNASSOC == 0xFFFE);
    CHECK(UWB_FRAME_N_CFP  == 11);
    CHECK(UWB_FRAME_LEN_BEACON == 15 + 2 * 11);  /* 37 */
    CHECK(UWB_FRAME_LEN_JOIN   == 19);
    CHECK(UWB_FRAME_LEN_GRANT  == 24);
    CHECK(UWB_FRAME_LEN_KEEPALIVE == 12);
    CHECK(UWB_FRAME_LEN_RELEASE   == 10);
}

static void test_beacon(void)
{
    uint16_t map[UWB_FRAME_N_CFP] = {0};
    for (int i = 0; i < UWB_FRAME_N_CFP; i++) map[i] = 0xFFFF;  /* all idle */
    map[3] = 0x1234;  /* our slot */

    uint8_t buf[64];
    int n = uwb_frame_beacon_build(buf, sizeof(buf), 0xAABBCCDD, map, UWB_FRAME_N_CFP);
    CHECK(n == UWB_FRAME_LEN_BEACON);
    CHECK(buf[5] == 0xFF && buf[6] == 0xFF);       /* dest broadcast */
    CHECK(buf[7] == 0x00 && buf[8] == 0x00);       /* src gateway */
    CHECK(buf[9] == UWB_FRAME_TYPE_BEACON);
    CHECK(buf[10] == UWB_PROTO_VER);
    CHECK(buf[11] == 0xDD && buf[14] == 0xAA);     /* counter LE */
    CHECK(uwb_frame_is_beacon(buf, n));

    uint8_t ver, ns; uint32_t fc; uint16_t out[UWB_FRAME_N_CFP];
    CHECK(uwb_frame_parse_beacon(buf, n, &ver, &fc, out, &ns) == 0);
    CHECK(ver == UWB_PROTO_VER && fc == 0xAABBCCDD && ns == UWB_FRAME_N_CFP);
    CHECK(out[3] == 0x1234 && out[0] == 0xFFFF);
    CHECK(uwb_frame_beacon_find_addr(out, ns, 0x1234) == 3);
    CHECK(uwb_frame_beacon_find_addr(out, ns, 0x9999) == -1);

    CHECK(uwb_frame_beacon_build(buf, 5, 0, map, UWB_FRAME_N_CFP) == -EMSGSIZE);
}

static void test_join_grant(void)
{
    const uint8_t eui[8] = {1,2,3,4,5,6,7,8};
    uint8_t buf[32];

    int n = uwb_frame_join_build(buf, sizeof(buf), eui, 2 /*FAST*/);
    CHECK(n == UWB_FRAME_LEN_JOIN);
    CHECK(buf[5] == 0x00 && buf[6] == 0x00);       /* dest gateway */
    CHECK(buf[7] == 0xFE && buf[8] == 0xFF);       /* src unassoc LE */
    CHECK(buf[9] == UWB_FRAME_TYPE_JOIN);
    CHECK(uwb_frame_is_join(buf, n));
    uint8_t e2[8], t; CHECK(uwb_frame_parse_join(buf, n, e2, &t) == 0);
    CHECK(memcmp(e2, eui, 8) == 0 && t == 2);

    n = uwb_frame_grant_build(buf, sizeof(buf), eui, 0x0007, 3, 1, 50);
    CHECK(n == UWB_FRAME_LEN_GRANT);
    CHECK(buf[5] == 0xFF && buf[6] == 0xFF);       /* dest broadcast (EUI-matched) */
    CHECK(buf[9] == UWB_FRAME_TYPE_GRANT);
    CHECK(uwb_frame_is_grant(buf, n));
    uint8_t e3[8]; uint16_t sa, ls; uint8_t si, rt;
    CHECK(uwb_frame_parse_grant(buf, n, e3, &sa, &si, &rt, &ls) == 0);
    CHECK(memcmp(e3, eui, 8) == 0 && sa == 0x0007 && si == 3 && rt == 1 && ls == 50);
}

static void test_keepalive_release(void)
{
    uint8_t buf[16];
    int n = uwb_frame_keepalive_build(buf, sizeof(buf), 0x0007, 2, 3);
    CHECK(n == UWB_FRAME_LEN_KEEPALIVE);
    CHECK(buf[5] == 0x00 && buf[6] == 0x00);       /* dest gateway */
    CHECK(buf[7] == 0x07 && buf[8] == 0x00);       /* src 0x0007 LE */
    CHECK(buf[9] == UWB_FRAME_TYPE_KEEPALIVE);
    CHECK(uwb_frame_is_keepalive(buf, n));
    uint16_t sa; uint8_t rt, si;
    CHECK(uwb_frame_parse_keepalive(buf, n, &sa, &rt, &si) == 0);
    CHECK(sa == 0x0007 && rt == 2 && si == 3);

    n = uwb_frame_release_build(buf, sizeof(buf), 0x0007);
    CHECK(n == UWB_FRAME_LEN_RELEASE);
    CHECK(buf[9] == UWB_FRAME_TYPE_RELEASE);
    CHECK(uwb_frame_is_release(buf, n));
}

static void test_pos(void)
{
    uint8_t buf[UWB_FRAME_MAX_LEN];

    CHECK(UWB_FRAME_TYPE_POS == 0xEA);
    CHECK(UWB_FRAME_LEN_POS == 24);
    CHECK(UWB_FRAME_POS_SOC_UNKNOWN == 0xFF);

    /* -3.25, 7.5 and 0.125 are exactly representable in binary32, so these
     * round-trip comparisons can use == without a tolerance. */
    int n = uwb_frame_pos_build(buf, sizeof(buf), 0x0102,
                                -3.25f, 7.5f, 0.125f, 4, 87);
    CHECK(n == UWB_FRAME_LEN_POS);
    CHECK(uwb_frame_is_pos(buf, (size_t)n));
    CHECK(uwb_frame_get_dest_addr(buf) == UWB_ADDR_GATEWAY);
    CHECK(uwb_frame_get_src_addr(buf) == 0x0102);
    CHECK(buf[9] == UWB_FRAME_TYPE_POS);

    uint16_t sa = 0;
    float x = 0.0f, y = 0.0f, res = 0.0f;
    uint8_t na = 0, soc = 0;

    CHECK(uwb_frame_parse_pos(buf, (size_t)n, &sa, &x, &y, &res, &na, &soc) == 0);
    CHECK(sa == 0x0102);
    CHECK(x == -3.25f);
    CHECK(y == 7.5f);
    CHECK(res == 0.125f);
    CHECK(na == 4);
    CHECK(soc == 87);

    /* Every out-param is optional. */
    CHECK(uwb_frame_parse_pos(buf, (size_t)n, NULL, NULL, NULL, NULL, NULL, NULL) == 0);

    /* The unknown-battery sentinel survives the round trip unchanged, and is
     * not confused with a real 0 % reading. */
    n = uwb_frame_pos_build(buf, sizeof(buf), 0x0103, 0.0f, 0.0f, 0.0f, 3,
                            UWB_FRAME_POS_SOC_UNKNOWN);
    CHECK(n == UWB_FRAME_LEN_POS);
    CHECK(uwb_frame_parse_pos(buf, (size_t)n, NULL, NULL, NULL, NULL, &na, &soc) == 0);
    CHECK(na == 3);
    CHECK(soc == UWB_FRAME_POS_SOC_UNKNOWN);

    /* Wrong length is rejected by both the predicate and the parser. */
    CHECK(!uwb_frame_is_pos(buf, UWB_FRAME_LEN_POS - 1));
    CHECK(uwb_frame_parse_pos(buf, UWB_FRAME_LEN_POS - 1,
                              &sa, &x, &y, &res, &na, &soc) == -EINVAL);

    /* GRANT is also 24 bytes, so length alone cannot separate the two — only
     * the type byte does. This asserts that the predicates stay disjoint on a
     * frame whose length matches both. */
    buf[9] = UWB_FRAME_TYPE_GRANT;
    CHECK(!uwb_frame_is_pos(buf, UWB_FRAME_LEN_POS));
    CHECK(uwb_frame_is_grant(buf, UWB_FRAME_LEN_GRANT));
    buf[9] = UWB_FRAME_TYPE_POS;
    CHECK(uwb_frame_is_pos(buf, UWB_FRAME_LEN_POS));
    CHECK(!uwb_frame_is_grant(buf, UWB_FRAME_LEN_GRANT));

    /* A short buffer is refused, not overrun. */
    uint8_t small[UWB_FRAME_LEN_POS - 1];
    CHECK(uwb_frame_pos_build(small, sizeof(small), 0x0102,
                              1.0f, 2.0f, 0.1f, 4, 50) == -EMSGSIZE);
}

static void test_alert(void)
{
    /* 0xEE since 2026-08-25. Was 0xEB, which collided exactly with the anchor
     * survey's APOS_FRAME_TYPE -- same code, same 34-byte length, same
     * discriminating offset. Pinned here because a function code is a wire
     * value: changing it must be a deliberate edit to a test, not a silent
     * consequence of editing a header. */
    CHECK(UWB_FRAME_TYPE_ALERT == 0xEE);
    CHECK(UWB_FRAME_LEN_ALERT == 34);
    CHECK(UWB_FRAME_LEN_ALERT <= UWB_FRAME_MAX_LEN);
    CHECK(UWB_ALERT_STATE_HELP == 0x01);
    CHECK(UWB_ALERT_STATE_CANCEL == 0x00);
    CHECK(UWB_ALERT_HOP_UNKNOWN == 0xFF);

    struct uwb_alert a;
    memset(&a, 0, sizeof(a));
    a.state       = UWB_ALERT_STATE_HELP;
    a.epoch       = 7;
    a.repeat_seq  = 3;
    a.sender_hop  = UWB_ALERT_HOP_UNKNOWN;
    a.ttl         = UWB_ALERT_TTL_INIT;
    memcpy(a.orig_eui, (uint8_t[8]){1,2,3,4,5,6,7,8}, 8);
    a.orig_addr   = 0x1234;
    a.batt_soc    = 87;
    a.last_x      = -1.5f;
    a.last_y      = 2.25f;

    uint8_t buf[UWB_FRAME_MAX_LEN];
    int n = uwb_frame_alert_build(buf, sizeof(buf), 0x0102, &a);
    CHECK(n == UWB_FRAME_LEN_ALERT);
    CHECK(uwb_frame_is_alert(buf, (size_t)n));
    CHECK(uwb_frame_get_dest_addr(buf) == UWB_ADDR_GATEWAY);
    CHECK(uwb_frame_get_src_addr(buf) == 0x0102);
    CHECK(buf[9] == UWB_FRAME_TYPE_ALERT);

    /* Round-trip: build -> parse returns every field bit-identical. Compare
     * the whole struct via memcmp, not field-by-field ==, because last_x/
     * last_y may be NaN below and NaN never compares equal to itself. */
    struct uwb_alert b;
    memset(&b, 0, sizeof(b));
    CHECK(uwb_frame_parse_alert(buf, (size_t)n, &b) == 0);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0);

    /* NaN coordinates round-trip too (raw-byte compare, not ==). */
    struct uwb_alert nanA;
    memset(&nanA, 0, sizeof(nanA));
    nanA.state      = UWB_ALERT_STATE_CANCEL;
    nanA.epoch      = 200;
    nanA.sender_hop = 2;
    nanA.ttl        = 3;
    nanA.last_x     = NAN;
    nanA.last_y     = NAN;
    uint8_t nbuf[UWB_FRAME_MAX_LEN];
    int nn = uwb_frame_alert_build(nbuf, sizeof(nbuf), 0x0007, &nanA);
    CHECK(nn == UWB_FRAME_LEN_ALERT);
    struct uwb_alert nanB;
    memset(&nanB, 0, sizeof(nanB));
    CHECK(uwb_frame_parse_alert(nbuf, (size_t)nn, &nanB) == 0);
    CHECK(memcmp(&nanA, &nanB, sizeof(nanA)) == 0);

    /* uwb_frame_is_alert rejects: wrong length, wrong type, bad PAN. */
    CHECK(!uwb_frame_is_alert(buf, UWB_FRAME_LEN_ALERT - 1));  /* 33 */
    CHECK(!uwb_frame_is_alert(buf, UWB_FRAME_LEN_ALERT + 1));  /* 35 */
    {
        uint8_t bt[UWB_FRAME_LEN_ALERT]; memcpy(bt, buf, UWB_FRAME_LEN_ALERT);
        bt[9] = UWB_FRAME_TYPE_POS;
        CHECK(!uwb_frame_is_alert(bt, UWB_FRAME_LEN_ALERT));
    }
    {
        uint8_t bp[UWB_FRAME_LEN_ALERT]; memcpy(bp, buf, UWB_FRAME_LEN_ALERT);
        bp[3] = 0xBE;   /* corrupt PAN */
        CHECK(!uwb_frame_is_alert(bp, UWB_FRAME_LEN_ALERT));
    }

    /* uwb_frame_parse_alert rejects a frame with any reserved bit set. */
    {
        uint8_t br[UWB_FRAME_LEN_ALERT]; memcpy(br, buf, UWB_FRAME_LEN_ALERT);
        br[10] |= 0x02;   /* state offset (byte 10); reserved bit1 */
        struct uwb_alert out;
        CHECK(uwb_frame_parse_alert(br, UWB_FRAME_LEN_ALERT, &out) == -EINVAL);
    }

    /* uwb_frame_alert_build errors. */
    CHECK(uwb_frame_alert_build(buf, UWB_FRAME_LEN_ALERT - 1, 0x0102, &a) == -EMSGSIZE);
    CHECK(uwb_frame_alert_build(NULL, sizeof(buf), 0x0102, &a) == -EINVAL);
    CHECK(uwb_frame_alert_build(buf, sizeof(buf), 0x0102, NULL) == -EINVAL);

    /* Same length as GRANT/POS is not enough to be confused -- only the type
     * byte distinguishes them. */
    CHECK(!uwb_frame_is_grant(buf, UWB_FRAME_LEN_GRANT));
    CHECK(!uwb_frame_is_pos(buf, UWB_FRAME_LEN_POS));
}

int main(void)
{
    test_scaffold();
    test_utilities();
    test_discovery();
    test_response();
    test_multipoll();
    test_corruption();
    test_new_constants();
    test_beacon();
    test_join_grant();
    test_keepalive_release();
    test_pos();
    test_alert();
    if (g_fail) { printf("%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}

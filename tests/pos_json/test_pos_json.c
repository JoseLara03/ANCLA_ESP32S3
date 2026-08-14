#include "../../src/pos_json.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_fix_exact_contract(void)
{
    /* The byte-for-byte contract with the downstream consumer. */
    struct pos_fix f = { .src_addr = 0x1234, .x = 1.23f, .y = 4.56f,
                         .residual_m = 0.12f, .n_anchors = 3, .batt_soc = 87 };
    char buf[POS_JSON_MAX_LEN];
    int n = pos_json_fix(buf, sizeof(buf), &f);

    CHECK(n > 0);
    CHECK(strcmp(buf, "{\"Tid\":4660,\"x\":1.23,\"y\":4.56,\"z\":0}") == 0);
    CHECK(n == (int)strlen(buf));
}

static void test_fix_drops_diagnostics(void)
{
    /* residual, n_anchors and batt_soc must NOT reach the payload -- they stay
     * on the console log line. zoneName is also gone: the consumer looks the
     * zone up via the anchors topic instead. Changing this breaks the
     * consumer contract. */
    struct pos_fix f = { .src_addr = 0x0001, .x = 0.0f, .y = 0.0f,
                         .residual_m = 9.99f, .n_anchors = 4, .batt_soc = 42 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "residual") == NULL);
    CHECK(strstr(buf, "anchors")  == NULL);
    CHECK(strstr(buf, "battery")  == NULL);
    CHECK(strstr(buf, "zoneName") == NULL);
    CHECK(strstr(buf, "42")       == NULL);
}

static void test_tid_is_plain_decimal_not_hex(void)
{
    /* Tid is fix->src_addr as a bare decimal NUMBER, not a hex string:
     * 0x00AB must be 171, not "00AB" and not quoted at all. */
    struct pos_fix f = { .src_addr = 0x00AB, .x = 0.0f, .y = 0.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":171,") != NULL);
    CHECK(strstr(buf, "\"Tid\":\"") == NULL);

    f.src_addr = 0xBEEF;
    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":48879,") != NULL);
}

static void test_z_is_an_integer_literal(void)
{
    /* The solver is 2D. The consumer expects 0, not 0.00. */
    struct pos_fix f = { .src_addr = 0x0001, .x = 1.0f, .y = 2.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"z\":0}") != NULL);
    CHECK(strstr(buf, "\"z\":0.00") == NULL);
}

static void test_negative_and_large_coordinates(void)
{
    struct pos_fix f = { .src_addr = 0x0002, .x = -12.345f, .y = 1234.5f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    /* %.2f rounds half-away-from-zero here: -12.345 -> -12.35 or -12.34 is
     * platform-dependent in the last digit, so assert the stable prefix. */
    CHECK(strstr(buf, "\"x\":-12.3") != NULL);
    CHECK(strstr(buf, "\"y\":1234.50") != NULL);
}

static void test_fix_truncation_is_reported(void)
{
    /* A partial JSON document must never be published. */
    struct pos_fix f = { .src_addr = 0x1234, .x = 1.23f, .y = 4.56f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char small[8];

    CHECK(pos_json_fix(small, sizeof(small), &f) == -1);
}

static void test_anchors_stub(void)
{
    char buf[POS_JSON_MAX_LEN];
    int n = pos_json_anchors(buf, sizeof(buf), NULL);

    CHECK(n > 0);
    CHECK(n == (int)strlen(buf));
    CHECK(strstr(buf, "\"name\":\"852541\"") != NULL);
    CHECK(strstr(buf, "\"anchors\":[") != NULL);

    /* Exactly one axis/reference anchor, carrying the only real
     * (building-level) lat/long -- the other three are local-only. */
    CHECK(strstr(buf, "\"name\":\"ANC-LOBBY-001\",\"isAxis\":true,"
                      "\"isReferenceAxis\":true,"
                      "\"latitude\":21.01604164655441,"
                      "\"longitude\":-89.6521292940793") != NULL);
    CHECK(strstr(buf, "\"name\":\"ANC-LOBBY-002\",\"isAxis\":false,"
                      "\"isReferenceAxis\":false,"
                      "\"latitude\":0,\"longitude\":0") != NULL);
    CHECK(strstr(buf, "\"name\":\"ANC-LOBBY-003\"") != NULL);
    CHECK(strstr(buf, "\"name\":\"ANC-LOBBY-004\"") != NULL);

    /* Four anchors at the corners of a 2 m x 2 m square. */
    CHECK(strstr(buf, "\"x\":0.0,\"y\":0.0,\"z\":0.0") != NULL);
    CHECK(strstr(buf, "\"x\":2.0,\"y\":0.0,\"z\":0.0") != NULL);
    CHECK(strstr(buf, "\"x\":2.0,\"y\":2.0,\"z\":0.0") != NULL);
    CHECK(strstr(buf, "\"x\":0.0,\"y\":2.0,\"z\":0.0") != NULL);

    CHECK(buf[n - 1] == '}');
}

static void test_anchors_fits_in_max_len(void)
{
    /* POS_JSON_MAX_LEN must be large enough for the bigger of the two
     * documents, or the uplink buffer is undersized. */
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_anchors(buf, sizeof(buf), NULL) > 0);
}

static void test_anchors_truncation_is_reported(void)
{
    char small[32];

    CHECK(pos_json_anchors(small, sizeof(small), NULL) == -1);
}

/* A gateway that was never surveyed must still publish a valid document, so the
 * stub is a fallback and not dead code. */
static void test_anchors_falls_back_to_the_stub(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = false;

    CHECK(pos_json_anchors(buf, sizeof(buf), NULL) > 0);
    CHECK(strstr(buf, "ANC-LOBBY-001") != NULL);

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
    CHECK(strstr(buf, "ANC-LOBBY-001") != NULL);
}

static void test_anchors_emits_the_surveyed_geometry(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 3;
    s.node[0].short_addr = 0x0001;
    s.node[0].x = 0.0f;  s.node[0].y = 0.0f;  s.node[0].z = 0.0f;
    s.node[1].short_addr = 0x0002;
    s.node[1].x = 3.25f; s.node[1].y = 0.0f;  s.node[1].z = 0.1f;
    s.node[2].short_addr = 0x0004;
    s.node[2].x = 1.5f;  s.node[2].y = 4.75f; s.node[2].z = 2.0f;
    s.ref_lat = 21.016042;
    s.ref_lon = -89.652129;
    s.ref_valid = true;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);

    /* Named from the short address, so the platform's anchor names track the
     * boards rather than a hand-kept list. */
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-001") != NULL);
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-002") != NULL);
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-004") != NULL);
    /* The stub must be entirely gone -- a document with both would draw twice. */
    CHECK(strstr(buf, "ANC-LOBBY-001") == NULL);
    /* Surveyed coordinates, including a real z. */
    CHECK(strstr(buf, "\"x\":3.25") != NULL);
    CHECK(strstr(buf, "\"y\":4.75") != NULL);
    CHECK(strstr(buf, "\"z\":2.00") != NULL);
    /* Exactly one axis/reference anchor, and it is node[0]. */
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
    CHECK(strstr(buf, "21.016042") != NULL);
}

/* Without `apos ref` there is no lat/long to publish. The document must still be
 * valid -- zeroes, exactly as the stub does for its non-reference anchors -- so
 * the platform draws the geometry even before the site is geo-referenced. */
static void test_anchors_without_a_geo_reference(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 1;
    s.node[0].short_addr = 0x0001;
    s.ref_valid = false;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
    CHECK(strstr(buf, "\"latitude\":0") != NULL);
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
}

/* POS_JSON_MAX_LEN must still hold the largest real document, which is now a
 * full APOS_MAX_NODES survey rather than the four-anchor stub. */
static void test_full_survey_fits_the_buffer(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = APOS_MAX_NODES;
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        s.node[k].short_addr = (uint16_t)(0x0001 + k);
        /* Widest plausible values, so the check is on the real worst case. */
        s.node[k].x = -123.456f;
        s.node[k].y = -123.456f;
        s.node[k].z = -123.456f;
    }
    s.ref_lat = -89.123456;
    s.ref_lon = -179.123456;
    s.ref_valid = true;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
}

/* A survey too large for the buffer must be REFUSED, not truncated: half a JSON
 * document published retained would poison the topic until the next connect. */
static void test_anchors_refuses_a_short_buffer(void)
{
    char small[64];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = APOS_MAX_NODES;
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        s.node[k].short_addr = (uint16_t)(0x0001 + k);
    }

    CHECK(pos_json_anchors(small, sizeof(small), &s) == -1);
}

int main(void)
{
    test_fix_exact_contract();
    test_fix_drops_diagnostics();
    test_tid_is_plain_decimal_not_hex();
    test_z_is_an_integer_literal();
    test_negative_and_large_coordinates();
    test_fix_truncation_is_reported();
    test_anchors_stub();
    test_anchors_fits_in_max_len();
    test_anchors_truncation_is_reported();
    test_anchors_falls_back_to_the_stub();
    test_anchors_emits_the_surveyed_geometry();
    test_anchors_without_a_geo_reference();
    test_full_survey_fits_the_buffer();
    test_anchors_refuses_a_short_buffer();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

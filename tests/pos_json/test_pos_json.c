#include "../../src/pos_json.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_fix_exact_contract(void)
{
    /* The byte-for-byte contract with the downstream consumer. src_addr and
     * tag_id are deliberately different: if the code ever regresses back to
     * reading src_addr, this test must fail rather than pass by coincidence. */
    struct pos_fix f = { .src_addr = 0x9999, .tag_id = 4660, .x = 1.23f, .y = 4.56f,
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
    /* Tid is fix->tag_id as a bare decimal NUMBER, not a hex string:
     * 0x00AB must be 171, not "00AB" and not quoted at all. src_addr is
     * held at a fixed, distinguishable value throughout so a regression
     * back to reading src_addr would be caught rather than pass by
     * coincidence. */
    struct pos_fix f = { .src_addr = 0x0001, .tag_id = 0x00AB, .x = 0.0f, .y = 0.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":171,") != NULL);
    CHECK(strstr(buf, "\"Tid\":\"") == NULL);

    f.tag_id = 0xBEEF;
    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":48879,") != NULL);
}

static void test_tid_is_tag_id_not_src_addr(void)
{
    /* If this ever regresses back to reading src_addr, this is the test
     * that catches it: src_addr and tag_id are deliberately different
     * values here, and only one of them may appear as Tid. */
    struct pos_fix f = { .src_addr = 0x0001, .tag_id = 999888777,
                         .x = 0.0f, .y = 0.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":999888777") != NULL);
    CHECK(strstr(buf, "\"Tid\":1,") == NULL);   /* src_addr's decimal value must NOT appear */
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

    int n = pos_json_anchors(buf, sizeof(buf), &s);

    CHECK(n > 0);

    /* Named from the short address, so the platform's anchor names track the
     * boards rather than a hand-kept list. */
    /* The ANC-LOBBY- prefix is deliberately shared with the stub: renaming
     * anchors at the first `apos apply` would orphan the platform's existing
     * records. Only the coordinates change. See pos_json.c. */
    CHECK(strstr(buf, "ANC-LOBBY-001") != NULL);
    CHECK(strstr(buf, "ANC-LOBBY-002") != NULL);
    CHECK(strstr(buf, "ANC-LOBBY-004") != NULL);
    /* The stub's fourth anchor must be gone -- the survey has three nodes and
     * a document carrying both geometries would draw twice. */
    CHECK(strstr(buf, "ANC-LOBBY-003") == NULL);
    /* Surveyed coordinates, including a real z. */
    CHECK(strstr(buf, "\"x\":3.25") != NULL);
    CHECK(strstr(buf, "\"y\":4.75") != NULL);
    CHECK(strstr(buf, "\"z\":2.00") != NULL);
    /* Exactly one axis/reference anchor, and it is node[0]. */
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
    CHECK(strstr(buf, "21.016042") != NULL);

    /* Same shape of pin the stub test already applies: the return value is
     * the real strlen (no embedded NUL / no unaccounted bytes), and the
     * document actually closes. Neither of these is implied by the substring
     * checks above -- a missing comma or a doubled '}' would still pass all
     * of them. */
    CHECK(n == (int)strlen(buf));
    CHECK(buf[n - 1] == '}');

    /* Full-string comparison, so the exact byte layout (comma placement,
     * field order, %.8f/%.2f widths) is frozen the same way the stub's is. */
    CHECK(strcmp(buf,
        "{\"name\":\"852541\",\"anchors\":["
        "{\"name\":\"ANC-LOBBY-001\",\"isAxis\":true,\"isReferenceAxis\":true,"
        "\"latitude\":21.01604200,\"longitude\":-89.65212900,"
        "\"x\":0.00,\"y\":0.00,\"z\":0.00},"
        "{\"name\":\"ANC-LOBBY-002\",\"isAxis\":false,\"isReferenceAxis\":false,"
        "\"latitude\":0.00000000,\"longitude\":0.00000000,"
        "\"x\":3.25,\"y\":0.00,\"z\":0.10},"
        "{\"name\":\"ANC-LOBBY-004\",\"isAxis\":false,\"isReferenceAxis\":false,"
        "\"latitude\":0.00000000,\"longitude\":0.00000000,"
        "\"x\":1.50,\"y\":4.75,\"z\":2.00}"
        "]}") == 0);
}

/* Node counts APOS_MIN_NODES_3D-1 (1, 3, 8) are exercised elsewhere; fill in the
 * untested middle of the range (4..8) so the comma logic isn't only proven at
 * the edges. */
static void test_anchors_well_formed_across_node_counts(void)
{
    for (uint8_t count = 4; count <= APOS_MAX_NODES; count++) {
        char buf[POS_JSON_MAX_LEN];
        struct apos_survey s;

        memset(&s, 0, sizeof(s));
        s.valid = true;
        s.n_nodes = count;
        for (uint8_t k = 0; k < count; k++) {
            s.node[k].short_addr = (uint16_t)(0x0001 + k);
            s.node[k].x = (float)k * 1.5f;
            s.node[k].y = (float)k * -2.5f;
            s.node[k].z = 0.0f;
        }
        s.ref_lat = 21.0;
        s.ref_lon = -89.0;
        s.ref_valid = true;

        int n = pos_json_anchors(buf, sizeof(buf), &s);

        CHECK(n > 0);
        CHECK(n == (int)strlen(buf));
        CHECK(buf[n - 1] == '}');
        CHECK(strstr(buf, "\"anchors\":[") != NULL);
        /* No malformed comma runs: neither back-to-back commas nor a comma
         * immediately before the closing bracket. */
        CHECK(strstr(buf, ",,") == NULL);
        CHECK(strstr(buf, ",]") == NULL);
    }
}

/* A non-finite coordinate must never reach the wire: %f on a NaN/Inf prints a
 * bare, unquoted token that is not valid JSON, and the platform's parser would
 * reject the WHOLE retained document -- worse than the stub it replaced. This
 * is not hypothetical: at APOS_MIN_NODES_3D the accept criterion is isostatic
 * (6 edges == 3N-6 free parameters), so a degenerate/near-collinear solve can
 * be accepted with a non-finite coordinate already in it. */
static void test_anchors_refuses_a_nonfinite_coordinate(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 2;
    s.node[0].short_addr = 0x0001;
    s.node[1].short_addr = 0x0002;
    s.node[1].x = (float)NAN;
    s.ref_valid = false;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) == -1);

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 1;
    s.node[0].short_addr = 0x0001;
    s.node[0].z = (float)INFINITY;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) == -1);
}

/* Same failure mode for the geographic reference: a non-finite ref_lat/lon
 * must also refuse the whole document, not just the node coordinates. */
static void test_anchors_refuses_a_nonfinite_reference(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 1;
    s.node[0].short_addr = 0x0001;
    s.ref_valid = true;
    s.ref_lat = (double)NAN;
    s.ref_lon = -89.0;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) == -1);

    s.ref_lat = 21.0;
    s.ref_lon = -(double)INFINITY;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) == -1);
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
    test_tid_is_tag_id_not_src_addr();
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
    test_anchors_well_formed_across_node_counts();
    test_anchors_refuses_a_nonfinite_coordinate();
    test_anchors_refuses_a_nonfinite_reference();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

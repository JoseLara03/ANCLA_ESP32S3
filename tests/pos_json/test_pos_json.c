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
    int n = pos_json_anchors(buf, sizeof(buf));

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

    CHECK(pos_json_anchors(buf, sizeof(buf)) > 0);
}

static void test_anchors_truncation_is_reported(void)
{
    char small[32];

    CHECK(pos_json_anchors(small, sizeof(small)) == -1);
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
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

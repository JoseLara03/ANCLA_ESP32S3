#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pos_json.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* El tópico se compone de POS_JSON_ZONE_NAME, nunca se escribe literal: la
 * misma regla que los tópicos de posición y de anclas, para que un tópico no
 * pueda contradecir el payload publicado en él. */
static void test_topic_is_zone_scoped(void)
{
    CHECK(strcmp(POS_JSON_TOPIC_BLINK,
                 "uwb/anchor/blink/" POS_JSON_ZONE_NAME) == 0);
    CHECK(strstr(POS_JSON_TOPIC_BLINK, POS_JSON_ZONE_NAME) != NULL);
}

static void test_roundtrip(void)
{
    char buf[POS_JSON_BLINK_MAX_LEN];
    struct pos_blink_obs in = {
        .anchor_id = 2, .blink_seq = 0x5A, .batt_soc = 77,
        .tag_addr = 0x0101, .quality = 1234, .t_dtu = 123456789012LL,
    };
    struct pos_blink_obs out;
    int n = pos_json_blink(buf, sizeof(buf), &in);

    CHECK(n > 0);
    CHECK((size_t)n == strlen(buf));
    CHECK(pos_json_blink_parse(buf, strlen(buf), &out) == 0);
    CHECK(out.anchor_id == in.anchor_id);
    CHECK(out.blink_seq == in.blink_seq);
    CHECK(out.batt_soc == in.batt_soc);
    CHECK(out.tag_addr == in.tag_addr);
    CHECK(out.quality == in.quality);
    CHECK(out.t_dtu == in.t_dtu);
}

/* LA lección del Tid a int32, fijada como contrato: ts se emite como STRING,
 * y el valor máximo de 40 bits sobrevive el viaje de ida y vuelta. Un consumidor
 * que lo lea como número JSON de 32 bits lo truncaría en silencio; con comillas
 * no puede hacerlo por accidente. */
static void test_ts_is_a_quoted_string_and_survives_40_bits(void)
{
    char buf[POS_JSON_BLINK_MAX_LEN];
    struct pos_blink_obs in = {
        .anchor_id = 3, .blink_seq = 255, .batt_soc = 100,
        .tag_addr = 0xFFFD, .quality = 0xFFFF,
        .t_dtu = 1099511627775LL,   /* 2^40 - 1 */
    };
    struct pos_blink_obs out;

    CHECK(pos_json_blink(buf, sizeof(buf), &in) > 0);
    CHECK(strstr(buf, "\"ts\":\"1099511627775\"") != NULL);
    CHECK(strstr(buf, "2147483647") == NULL);
    CHECK(pos_json_blink_parse(buf, strlen(buf), &out) == 0);
    CHECK(out.t_dtu == 1099511627775LL);
    printf("  observation payload: %s (%u bytes)\n", buf,
           (unsigned int)strlen(buf));
}

/* El peor caso debe caber en POS_JSON_BLINK_MAX_LEN con el NUL, y un buffer
 * corto debe devolver -1 en vez de publicar JSON truncado: la misma regla que
 * pos_json_fix(). */
static void test_worst_case_fits_and_short_buffer_refuses(void)
{
    char buf[POS_JSON_BLINK_MAX_LEN];
    char tiny[16];
    struct pos_blink_obs in = {
        .anchor_id = 255, .blink_seq = 255, .batt_soc = 255,
        .tag_addr = 0xFFFF, .quality = 0xFFFF, .t_dtu = 1099511627775LL,
    };
    int n = pos_json_blink(buf, sizeof(buf), &in);

    CHECK(n > 0 && (size_t)n < POS_JSON_BLINK_MAX_LEN);
    CHECK(pos_json_blink(tiny, sizeof(tiny), &in) == -1);
}

static void test_parse_rejects_garbage(void)
{
    struct pos_blink_obs out;
    const char *bad[] = {
        "",
        "{}",
        "not json at all",
        "{\"a\":1,\"t\":2,\"s\":3,\"q\":4,\"b\":5}",          /* falta ts */
        "{\"a\":1,\"t\":2,\"s\":3,\"ts\":\"7\",\"b\":5}",      /* falta q */
        "{\"a\":1,\"t\":70000,\"s\":3,\"ts\":\"7\",\"q\":4,\"b\":5}", /* tag fuera de rango */
        "{\"a\":300,\"t\":2,\"s\":3,\"ts\":\"7\",\"q\":4,\"b\":5}",   /* ancla fuera de rango */
        "{\"a\":1,\"t\":2,\"s\":3,\"ts\":\"-1\",\"q\":4,\"b\":5}",    /* ts negativo */
    };

    for (unsigned int i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        CHECK(pos_json_blink_parse(bad[i], strlen(bad[i]), &out) == -1);
    }
    CHECK(pos_json_blink_parse(NULL, 4, &out) == -1);
    CHECK(pos_json_blink_parse("{}", 2, NULL) == -1);
}

/* El payload puede llegar sin NUL desde MQTT y de un publicador más nuevo con
 * campos extra. Ni una cosa ni la otra debe romper el parseo. */
static void test_parse_handles_unterminated_and_extra_fields(void)
{
    const char json[] = "{\"a\":1,\"t\":258,\"s\":9,\"ts\":\"42\",\"q\":7,"
                        "\"b\":55,\"future\":\"x\"}JUNK";
    struct pos_blink_obs out;

    CHECK(pos_json_blink_parse(json, strlen(json) - 4u, &out) == 0);
    CHECK(out.tag_addr == 258 && out.blink_seq == 9);
    CHECK(out.t_dtu == 42 && out.quality == 7 && out.batt_soc == 55);
    /* Un payload que no cabe en el buffer interno se rechaza, no se trunca. */
    CHECK(pos_json_blink_parse(json, POS_JSON_BLINK_MAX_LEN, &out) == -1);
}

int main(void)
{
    test_topic_is_zone_scoped();
    test_roundtrip();
    test_ts_is_a_quoted_string_and_survives_40_bits();
    test_worst_case_fits_and_short_buffer_refuses();
    test_parse_rejects_garbage();
    test_parse_handles_unterminated_and_extra_fields();
    if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
    printf("pos_json_blink: ALL TESTS PASSED\n");
    return EXIT_SUCCESS;
}

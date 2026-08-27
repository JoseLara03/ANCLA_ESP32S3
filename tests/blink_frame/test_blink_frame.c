/*
 * Host tests for blink_frame.
 *
 * Build:
 *   gcc -Wall -Wextra -Isrc -o tests/blink_frame/test_blink_frame.exe \
 *       tests/blink_frame/test_blink_frame.c src/blink_frame.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "blink_frame.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* El código de función es un valor de AIRE asignado entre dos repos. Fijado
 * aquí para que cambiarlo sea una edición deliberada a un test -- que 0xEB se
 * repartiera dos veces fue exactamente por no tener esto. */
static void test_wire_constants(void)
{
    CHECK(BLINK_FRAME_TYPE == 0xF0);
    CHECK(BLINK_FRAME_LEN == 14);
}

static void test_roundtrip(void)
{
    uint8_t buf[BLINK_FRAME_LEN];
    struct blink_frame in = { .seq = 0x5A, .batt_soc = 77, .flags = 0x01 };
    struct blink_frame out;

    CHECK(blink_frame_build(buf, sizeof(buf), 0x0101, 0x22, &in) ==
          (int)BLINK_FRAME_LEN);
    CHECK(buf[0] == 0x41 && buf[1] == 0x88);
    CHECK(buf[2] == 0x22);
    CHECK(buf[3] == 0xCA && buf[4] == 0xDE);
    CHECK(buf[5] == 0xFF && buf[6] == 0xFF);
    CHECK(buf[7] == 0x01 && buf[8] == 0x01);
    CHECK(buf[9] == BLINK_FRAME_TYPE);
    CHECK(buf[13] == 0);

    CHECK(blink_frame_is_blink(buf, sizeof(buf)));
    CHECK(blink_frame_parse(buf, sizeof(buf), &out) == 0);
    CHECK(out.src_addr == 0x0101);
    CHECK(out.seq == 0x5A);
    CHECK(out.batt_soc == 77);
    CHECK(out.flags == 0x01);
}

/* Igual que ccp_frame: dwt_getframelength() INCLUYE el FCS, así que un test
 * `len ==` tiraría todo frame real de un llamador que no lo restó. */
static void test_length_tolerates_the_fcs(void)
{
    uint8_t buf[BLINK_FRAME_LEN + 2];
    struct blink_frame in = { .seq = 1, .batt_soc = 50, .flags = 0 };

    CHECK(blink_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);
    CHECK(blink_frame_is_blink(buf, BLINK_FRAME_LEN));
    CHECK(blink_frame_is_blink(buf, BLINK_FRAME_LEN + 2));
    CHECK(!blink_frame_is_blink(buf, BLINK_FRAME_LEN - 1));
}

/* Los bits reservados deben ser cero. Aceptarlos hoy hace imposible usarlos
 * mañana sin romper compatibilidad. */
static void test_reserved_bits_rejected(void)
{
    uint8_t buf[BLINK_FRAME_LEN];
    struct blink_frame in = { .seq = 1, .batt_soc = 50, .flags = 0 };
    struct blink_frame out;

    CHECK(blink_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);
    buf[12] = 0x02;
    CHECK(blink_frame_parse(buf, sizeof(buf), &out) == -EPROTO);
    buf[12] = 0x00;
    buf[13] = 0x01;
    CHECK(blink_frame_parse(buf, sizeof(buf), &out) == -EPROTO);
}

static void test_rejects_other_codes(void)
{
    uint8_t buf[BLINK_FRAME_LEN];
    struct blink_frame in = { .seq = 1, .batt_soc = 0, .flags = 0 };
    struct blink_frame out;
    const uint8_t others[] = { 0xE0, 0xE5, 0xEA, 0xEB, 0xEE, 0xEF };

    CHECK(blink_frame_build(buf, sizeof(buf), 1, 0, &in) > 0);
    for (unsigned int i = 0; i < sizeof(others); i++) {
        buf[9] = others[i];
        CHECK(!blink_frame_is_blink(buf, sizeof(buf)));
        CHECK(blink_frame_parse(buf, sizeof(buf), &out) == -EINVAL);
    }
}

static void test_arg_checks(void)
{
    uint8_t buf[BLINK_FRAME_LEN];
    struct blink_frame in = { .seq = 1, .batt_soc = 0, .flags = 0 };

    CHECK(blink_frame_build(NULL, sizeof(buf), 1, 0, &in) == -EINVAL);
    CHECK(blink_frame_build(buf, sizeof(buf), 1, 0, NULL) == -EINVAL);
    CHECK(blink_frame_build(buf, BLINK_FRAME_LEN - 1, 1, 0, &in) == -EMSGSIZE);
    CHECK(!blink_frame_is_blink(NULL, BLINK_FRAME_LEN));
}

/* El airtime es el presupuesto entero de la Fase 3: 100 tags a 5 Hz son 500
 * blinks/s. Registrado para que el costo esté a la vista. */
static void test_airtime(void)
{
    const unsigned int us = 1042 + 8 + 22 + (BLINK_FRAME_LEN + 2) * 941 / 100;

    CHECK(us > 1200 && us < 1250);
    printf("  BLINK airtime: ~%u us; 500/s = %.1f %% de ocupacion\n",
           us, (double)us * 500.0 / 10000.0);
}

int main(void)
{
    test_wire_constants();
    test_roundtrip();
    test_length_tolerates_the_fcs();
    test_reserved_bits_rejected();
    test_rejects_other_codes();
    test_arg_checks();
    test_airtime();
    if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
    printf("blink_frame: ALL TESTS PASSED\n");
    return EXIT_SUCCESS;
}

#include <stdio.h>
#include <stdlib.h>
#include "tdoa_dtu.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void test_rebase_makes_the_reference_zero(void)
{
    struct tdoa_meas m[3] = {
        { .x = 0.0f, .y = 0.0f, .dz = 0.0f, .t_dtu = 1000000LL },
        { .x = 1.0f, .y = 0.0f, .dz = 0.0f, .t_dtu = 1000213LL },
        { .x = 0.0f, .y = 1.0f, .dz = 0.0f, .t_dtu =  999787LL },
    };

    tdoa_dtu_rebase(m, 3);
    CHECK(m[0].t_dtu == 0);
    CHECK(m[1].t_dtu == 213);
    CHECK(m[2].t_dtu == -213);
    /* La geometría no se toca. */
    CHECK(m[1].x == 1.0f && m[2].y == 1.0f);
}

/* EL punto de este módulo. El contador de 40 bits da la vuelta cada ~17.2 s;
 * dos anclas del mismo blink pueden caer en lados distintos de la vuelta, y la
 * diferencia CRUDA sería de 2^40 DTU (~5.16 millones de km de diferencia de
 * camino). */
static void test_rebase_survives_the_40_bit_wrap(void)
{
    const int64_t top = TDOA_DTU_MODULO - 100LL;   /* justo antes de la vuelta */
    struct tdoa_meas m[3] = {
        { .t_dtu = top },          /* referencia, antes de la vuelta   */
        { .t_dtu = 50LL },         /* ya volvió: +150 DTU en realidad  */
        { .t_dtu = top - 150LL },  /* antes de la vuelta: -150 DTU     */
    };

    tdoa_dtu_rebase(m, 3);
    CHECK(m[0].t_dtu == 0);
    CHECK(m[1].t_dtu == 150);
    CHECK(m[2].t_dtu == -150);
}

/* El caso inverso: la referencia es la que ya volvió. */
static void test_rebase_survives_the_wrap_with_wrapped_reference(void)
{
    struct tdoa_meas m[2] = {
        { .t_dtu = 20LL },
        { .t_dtu = TDOA_DTU_MODULO - 80LL },
    };

    tdoa_dtu_rebase(m, 2);
    CHECK(m[0].t_dtu == 0);
    CHECK(m[1].t_dtu == -100);
}

static void test_rebase_arg_checks(void)
{
    struct tdoa_meas m[1] = { { .t_dtu = 42LL } };

    tdoa_dtu_rebase(NULL, 3);      /* no debe caerse */
    tdoa_dtu_rebase(m, 0);         /* tampoco */
    CHECK(m[0].t_dtu == 42);
    tdoa_dtu_rebase(m, 1);
    CHECK(m[0].t_dtu == 0);
}

/* La cota física: 32768 DTU son 153.7 m de diferencia de camino, por encima de
 * cualquier celda que este MAC cubra. Lo que pasa el filtro es plausible; lo que
 * no, es sincronía rota o una vuelta mal corregida, no una medición. */
static void test_plausible_bounds_the_spread(void)
{
    struct tdoa_meas ok[3] = {
        { .t_dtu = 0LL }, { .t_dtu = 2000LL }, { .t_dtu = -1500LL },
    };
    struct tdoa_meas bad[3] = {
        { .t_dtu = 0LL }, { .t_dtu = TDOA_DTU_MAX_SPREAD + 1LL }, { .t_dtu = 0LL },
    };
    struct tdoa_meas bad_neg[3] = {
        { .t_dtu = 0LL }, { .t_dtu = 0LL }, { .t_dtu = -(TDOA_DTU_MAX_SPREAD + 1LL) },
    };
    struct tdoa_meas edge[2] = {
        { .t_dtu = 0LL }, { .t_dtu = TDOA_DTU_MAX_SPREAD },
    };

    CHECK(tdoa_dtu_plausible(ok, 3));
    CHECK(!tdoa_dtu_plausible(bad, 3));
    CHECK(!tdoa_dtu_plausible(bad_neg, 3));
    CHECK(tdoa_dtu_plausible(edge, 2));   /* la cota es inclusiva */
    CHECK(!tdoa_dtu_plausible(NULL, 3));
    CHECK(!tdoa_dtu_plausible(ok, 0));
    printf("  TDOA_DTU_MAX_SPREAD = %lld DTU = %.1f m de diferencia de camino\n",
           (long long)TDOA_DTU_MAX_SPREAD,
           (double)TDOA_DTU_MAX_SPREAD * 4.69175e-3);
}

/* Un grupo sin corregir la vuelta debe ser rechazado por el filtro: es la
 * defensa en profundidad que hace que un rebase olvidado se note. */
static void test_unrebased_wrap_is_rejected(void)
{
    struct tdoa_meas m[2] = {
        { .t_dtu = TDOA_DTU_MODULO - 100LL },
        { .t_dtu = 50LL },
    };

    CHECK(!tdoa_dtu_plausible(m, 2));
    tdoa_dtu_rebase(m, 2);
    CHECK(tdoa_dtu_plausible(m, 2));
}

int main(void)
{
    test_rebase_makes_the_reference_zero();
    test_rebase_survives_the_40_bit_wrap();
    test_rebase_survives_the_wrap_with_wrapped_reference();
    test_rebase_arg_checks();
    test_plausible_bounds_the_spread();
    test_unrebased_wrap_is_rejected();
    if (failures) { printf("\n%d CHECK(s) FAILED\n", failures); return EXIT_FAILURE; }
    printf("tdoa_dtu: ALL TESTS PASSED\n");
    return EXIT_SUCCESS;
}

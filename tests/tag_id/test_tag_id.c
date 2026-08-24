#include "tag_id.h"
#include <stdio.h>
#include <stdint.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

/* The published FNV-1a 32-bit vectors, each masked to 31 bits. Written as
 * `<published value> & 0x7FFFFFFF` rather than as the already-reduced number
 * so these still check the HASH against its reference vectors, and not merely
 * the masking step layered on top. */
static void test_known_fnv1a_vectors(void)
{
    CHECK(tag_id_from_eui((const uint8_t *)"", 0) == (2166136261u & 0x7FFFFFFFu));
    CHECK(tag_id_from_eui((const uint8_t *)"a", 1) == (3826002220u & 0x7FFFFFFFu));
    CHECK(tag_id_from_eui((const uint8_t *)"foobar", 6) == (3214735720u & 0x7FFFFFFFu));
}

static void test_eui_vectors(void)
{
    uint8_t eui_zero[8] = {0,0,0,0,0,0,0,0};
    uint8_t eui_pattern[8] = {0xDE,0xAD,0xBE,0xEF,0x00,0x11,0x22,0x33};

    CHECK(tag_id_from_eui(eui_zero, 8) == (2615243109u & 0x7FFFFFFFu));
    CHECK(tag_id_from_eui(eui_pattern, 8) == (4251971287u & 0x7FFFFFFFu));
}

static void test_deterministic_and_distinguishes_inputs(void)
{
    uint8_t eui_a[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    uint8_t eui_b[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09};   /* differs in last byte only */

    uint32_t a1 = tag_id_from_eui(eui_a, 8);
    uint32_t a2 = tag_id_from_eui(eui_a, 8);
    uint32_t b  = tag_id_from_eui(eui_b, 8);

    CHECK(a1 == a2);     /* same input, same output, every time */
    CHECK(a1 != b);      /* a one-byte difference must not collide here */
}

/* The platform stores Tid in a SIGNED 32-bit column. A hash with bit 31 set
 * arrives there as a negative number and the record is dropped -- observed on
 * the bench 2026-08-24 with a tag whose Tid was 2728562623 (0xA2A28FBF):
 * the gateway logged and published its fixes correctly, and the tag simply
 * never appeared, while two tags at 693116308 and 2082962887 did. The
 * consumer is third-party and cannot be changed, so the constraint has to
 * hold on this side. */
static void test_tag_id_always_fits_in_positive_int32(void)
{
    /* Chosen because plain FNV-1a hashes these with bit 31 set: without the
     * range constraint they come out as 2397001886, 2413779505 and
     * 2363446648, all above INT32_MAX. */
    uint8_t eui_hi_a[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x00,0x00};
    uint8_t eui_hi_b[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x00,0x01};
    uint8_t eui_hi_c[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x00,0x02};
    uint8_t eui_zero[8] = {0,0,0,0,0,0,0,0};
    uint8_t eui_pattern[8] = {0xDE,0xAD,0xBE,0xEF,0x00,0x11,0x22,0x33};

    CHECK(tag_id_from_eui(eui_hi_a, 8) <= (uint32_t)INT32_MAX);
    CHECK(tag_id_from_eui(eui_hi_b, 8) <= (uint32_t)INT32_MAX);
    CHECK(tag_id_from_eui(eui_hi_c, 8) <= (uint32_t)INT32_MAX);
    CHECK(tag_id_from_eui(eui_zero, 8) <= (uint32_t)INT32_MAX);
    CHECK(tag_id_from_eui(eui_pattern, 8) <= (uint32_t)INT32_MAX);

    /* Still distinct: clearing one bit must not collapse them onto each
     * other, or the range fix would trade an invisible tag for two tags
     * sharing a record. */
    CHECK(tag_id_from_eui(eui_hi_a, 8) != tag_id_from_eui(eui_hi_b, 8));
    CHECK(tag_id_from_eui(eui_hi_b, 8) != tag_id_from_eui(eui_hi_c, 8));
}

int main(void)
{
    test_known_fnv1a_vectors();
    test_eui_vectors();
    test_deterministic_and_distinguishes_inputs();
    test_tag_id_always_fits_in_positive_int32();

    if (g_fail == 0) {
        printf("OK\n");
        return 0;
    }
    printf("%d failure(s)\n", g_fail);
    return 1;
}

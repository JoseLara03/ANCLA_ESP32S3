#include "tag_id.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_known_fnv1a_vectors(void)
{
    CHECK(tag_id_from_eui((const uint8_t *)"", 0) == 2166136261);
    CHECK(tag_id_from_eui((const uint8_t *)"a", 1) == 3826002220);
    CHECK(tag_id_from_eui((const uint8_t *)"foobar", 6) == 3214735720);
}

static void test_eui_vectors(void)
{
    uint8_t eui_zero[8] = {0,0,0,0,0,0,0,0};
    uint8_t eui_pattern[8] = {0xDE,0xAD,0xBE,0xEF,0x00,0x11,0x22,0x33};

    CHECK(tag_id_from_eui(eui_zero, 8) == 2615243109);
    CHECK(tag_id_from_eui(eui_pattern, 8) == 4251971287);
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

int main(void)
{
    test_known_fnv1a_vectors();
    test_eui_vectors();
    test_deterministic_and_distinguishes_inputs();

    if (g_fail == 0) {
        printf("OK\n");
        return 0;
    }
    printf("%d failure(s)\n", g_fail);
    return 1;
}

#include "../../src/uwb_config.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_defaults(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    CHECK(c.mode == UWB_MODE_SLAVE);
    CHECK(c.anchor_id == 0);
    CHECK(c.ant_delay_tx == UWB_ANT_DELAY_DEFAULT);
    CHECK(c.ant_delay_rx == UWB_ANT_DELAY_DEFAULT);
    CHECK(c.x == 0.0f && c.y == 0.0f && c.z == 0.0f);
    CHECK(!c.position_valid);
}

static void test_set_id(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        CHECK(c.anchor_id == id);
    }

    /* Out of range is rejected and leaves the previous value intact. */
    CHECK(uwb_config_set_id(&c, 2));
    CHECK(!uwb_config_set_id(&c, UWB_MAX_ANCHORS));
    CHECK(c.anchor_id == 2);
    CHECK(!uwb_config_set_id(&c, 255));
    CHECK(c.anchor_id == 2);
}

static void test_set_mode(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    CHECK(uwb_config_set_mode(&c, UWB_MODE_GATEWAY));
    CHECK(c.mode == UWB_MODE_GATEWAY);
    CHECK(uwb_config_set_mode(&c, UWB_MODE_SLAVE));
    CHECK(c.mode == UWB_MODE_SLAVE);

    CHECK(uwb_config_set_mode(&c, UWB_MODE_GATEWAY));
    CHECK(!uwb_config_set_mode(&c, 2));
    CHECK(c.mode == UWB_MODE_GATEWAY);
}

static void test_mode_names(void)
{
    uint8_t m = 0xFF;

    CHECK(uwb_config_mode_from_name("slave", &m));
    CHECK(m == UWB_MODE_SLAVE);
    CHECK(uwb_config_mode_from_name("gateway", &m));
    CHECK(m == UWB_MODE_GATEWAY);

    m = UWB_MODE_GATEWAY;
    CHECK(!uwb_config_mode_from_name("master", &m));
    CHECK(!uwb_config_mode_from_name("", &m));
    CHECK(m == UWB_MODE_GATEWAY);   /* unchanged on rejection */

    CHECK(strcmp(uwb_config_mode_name(UWB_MODE_SLAVE), "slave") == 0);
    CHECK(strcmp(uwb_config_mode_name(UWB_MODE_GATEWAY), "gateway") == 0);
    CHECK(strcmp(uwb_config_mode_name(200), "unknown") == 0);
}

static void test_set_ant(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    CHECK(uwb_config_set_ant(&c, 0, 0));
    CHECK(c.ant_delay_tx == 0 && c.ant_delay_rx == 0);
    CHECK(uwb_config_set_ant(&c, 65535, 65535));
    CHECK(c.ant_delay_tx == 65535 && c.ant_delay_rx == 65535);

    /* Above uint16 range: rejected, both fields unchanged. */
    CHECK(uwb_config_set_ant(&c, 16385, 16400));
    CHECK(!uwb_config_set_ant(&c, 65536, 100));
    CHECK(c.ant_delay_tx == 16385 && c.ant_delay_rx == 16400);
    CHECK(!uwb_config_set_ant(&c, 100, 65536));
    CHECK(c.ant_delay_tx == 16385 && c.ant_delay_rx == 16400);
}

static void test_set_pos(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    CHECK(!c.position_valid);

    uwb_config_set_pos(&c, 1.5f, -2.25f, 0.75f);
    CHECK(c.x == 1.5f);
    CHECK(c.y == -2.25f);
    CHECK(c.z == 0.75f);
    CHECK(c.position_valid);

    /* The origin is a legitimate position — setting it must still mark valid. */
    uwb_config_set_pos(&c, 0.0f, 0.0f, 0.0f);
    CHECK(c.position_valid);
}

/* B14: FR-2 (persisted, editable, DTU, signed), FR-3 (source priority),
 * FR-11 (LEGACY forces 0 regardless of the stored survey value). */
static void test_sync_tau_defaults(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    CHECK(c.sync_tau_dtu == 0);
    CHECK(c.sync_tau_mode == SYNC_TAU_MODE_SURVEY);
    CHECK(uwb_config_sync_tau_effective(&c) == 0); /* 0 either way when unset */
}

static void test_sync_tau_set(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    uwb_config_set_sync_tau(&c, 12345);
    CHECK(c.sync_tau_dtu == 12345);
    uwb_config_set_sync_tau(&c, -54321);
    CHECK(c.sync_tau_dtu == -54321);
    uwb_config_set_sync_tau(&c, 0);
    CHECK(c.sync_tau_dtu == 0);
}

static void test_sync_tau_mode(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_LEGACY));
    CHECK(c.sync_tau_mode == SYNC_TAU_MODE_LEGACY);
    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_AUTO));
    CHECK(c.sync_tau_mode == SYNC_TAU_MODE_AUTO);

    CHECK(!uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_COUNT));
    CHECK(c.sync_tau_mode == SYNC_TAU_MODE_AUTO); /* unchanged on rejection */
}

static void test_sync_tau_mode_names(void)
{
    uint8_t m = 0xFF;

    CHECK(uwb_config_sync_tau_mode_from_name("legacy", &m));
    CHECK(m == SYNC_TAU_MODE_LEGACY);
    CHECK(uwb_config_sync_tau_mode_from_name("survey", &m));
    CHECK(m == SYNC_TAU_MODE_SURVEY);
    CHECK(uwb_config_sync_tau_mode_from_name("twr", &m));
    CHECK(m == SYNC_TAU_MODE_TWR);
    CHECK(uwb_config_sync_tau_mode_from_name("auto", &m));
    CHECK(m == SYNC_TAU_MODE_AUTO);

    m = SYNC_TAU_MODE_SURVEY;
    CHECK(!uwb_config_sync_tau_mode_from_name("bogus", &m));
    CHECK(m == SYNC_TAU_MODE_SURVEY); /* unchanged on rejection */

    CHECK(strcmp(uwb_config_sync_tau_mode_name(SYNC_TAU_MODE_LEGACY), "legacy") == 0);
    CHECK(strcmp(uwb_config_sync_tau_mode_name(SYNC_TAU_MODE_SURVEY), "survey") == 0);
    CHECK(strcmp(uwb_config_sync_tau_mode_name(SYNC_TAU_MODE_TWR), "twr") == 0);
    CHECK(strcmp(uwb_config_sync_tau_mode_name(SYNC_TAU_MODE_AUTO), "auto") == 0);
    CHECK(strcmp(uwb_config_sync_tau_mode_name(200), "unknown") == 0);
}

/* T7 (source priority), at the level this repo can host-test: no on-air TWR
 * source exists yet, so TWR/AUTO must resolve to the same value SURVEY does
 * (FR-9's degrade-with-no-source path) -- and LEGACY must force 0 regardless
 * of what tau_survey_dtu holds, which is the whole of FR-11's rollback
 * guarantee. */
static void test_sync_tau_effective_priority(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    uwb_config_set_sync_tau(&c, 9000);

    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_LEGACY));
    CHECK(uwb_config_sync_tau_effective(&c) == 0);

    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_SURVEY));
    CHECK(uwb_config_sync_tau_effective(&c) == 9000);

    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_TWR));
    CHECK(uwb_config_sync_tau_effective(&c) == 9000);

    CHECK(uwb_config_set_sync_tau_mode(&c, SYNC_TAU_MODE_AUTO));
    CHECK(uwb_config_sync_tau_effective(&c) == 9000);
}

static void test_singleton_starts_at_defaults(void)
{
    uwb_config_t *c = uwb_config_get();
    CHECK(c != NULL);
    CHECK(c->mode == UWB_MODE_SLAVE);
    CHECK(c->anchor_id == 0);
    CHECK(c->ant_delay_tx == UWB_ANT_DELAY_DEFAULT);
    CHECK(uwb_config_get() == c);   /* same instance every call */
}

static void test_short_addr(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    /* Default id 0 must not map to the gateway's reserved 0x0000. */
    CHECK(c.anchor_id == 0);
    CHECK(uwb_config_short_addr(&c) == 0x0001);

    /* The mapping is base + id across the whole valid range. */
    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        CHECK(uwb_config_short_addr(&c) == (uint16_t)(UWB_ANCHOR_ADDR_BASE + id));
    }

    /* No valid id can produce a reserved address. */
    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        uint16_t a = uwb_config_short_addr(&c);
        CHECK(a != UWB_ADDR_GATEWAY_RESERVED);
        CHECK(a != 0xFFFE);
        CHECK(a != 0xFFFF);
    }

    /* The low byte is what the tag echoes in the WAVE poll's byte 10. */
    CHECK(uwb_config_set_id(&c, 3));
    CHECK((uint8_t)(uwb_config_short_addr(&c) & 0xFFu) == 4);

    /* GATEWAY mode always reports the reserved address, regardless of
     * anchor_id -- a fresh/never-touched gateway defaults to id 0, which
     * must NOT alias onto a genuine anchor id 0's address (this collided
     * two boards' MQTT client ids on the same broker before this fix). */
    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        CHECK(uwb_config_set_mode(&c, UWB_MODE_GATEWAY));
        CHECK(uwb_config_short_addr(&c) == UWB_ADDR_GATEWAY_RESERVED);
    }
}

int main(void)
{
    test_defaults();
    test_set_id();
    test_short_addr();
    test_set_mode();
    test_mode_names();
    test_set_ant();
    test_set_pos();
    test_sync_tau_defaults();
    test_sync_tau_set();
    test_sync_tau_mode();
    test_sync_tau_mode_names();
    test_sync_tau_effective_priority();
    test_singleton_starts_at_defaults();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

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

static void test_singleton_starts_at_defaults(void)
{
    uwb_config_t *c = uwb_config_get();
    CHECK(c != NULL);
    CHECK(c->mode == UWB_MODE_SLAVE);
    CHECK(c->anchor_id == 0);
    CHECK(c->ant_delay_tx == UWB_ANT_DELAY_DEFAULT);
    CHECK(uwb_config_get() == c);   /* same instance every call */
}

int main(void)
{
    test_defaults();
    test_set_id();
    test_set_mode();
    test_mode_names();
    test_set_ant();
    test_set_pos();
    test_singleton_starts_at_defaults();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

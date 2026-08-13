#include "../../src/net_config.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void fill(char *buf, size_t n, char ch)
{
    memset(buf, ch, n);
    buf[n] = '\0';
}

static void test_defaults(void)
{
    net_config_t c;
    net_config_set_defaults(&c);
    CHECK(c.ssid[0] == '\0');
    CHECK(c.psk[0] == '\0');
    CHECK(c.broker[0] == '\0');
    CHECK(c.port == NET_MQTT_PORT_DEFAULT);
    CHECK(c.mqtt_user[0] == '\0');
    CHECK(c.mqtt_pass[0] == '\0');
    /* Defaults alone are not enough to attempt an uplink. */
    CHECK(!net_config_is_provisioned(&c));
}

static void test_ssid_bounds(void)
{
    net_config_t c;
    char buf[NET_SSID_MAX + 8];
    net_config_set_defaults(&c);

    CHECK(net_config_set_ssid(&c, "MyNetwork"));
    CHECK(strcmp(c.ssid, "MyNetwork") == 0);

    fill(buf, NET_SSID_MAX, 'a');
    CHECK(net_config_set_ssid(&c, buf));
    CHECK(strlen(c.ssid) == NET_SSID_MAX);

    /* Too long and empty are both rejected, leaving the previous value. */
    fill(buf, NET_SSID_MAX + 1, 'b');
    CHECK(!net_config_set_ssid(&c, buf));
    CHECK(strlen(c.ssid) == NET_SSID_MAX);
    CHECK(c.ssid[0] == 'a');

    CHECK(!net_config_set_ssid(&c, ""));
    CHECK(c.ssid[0] == 'a');
}

static void test_psk_bounds(void)
{
    net_config_t c;
    char buf[NET_PSK_MAX + 8];
    net_config_set_defaults(&c);

    /* WPA2 passphrase is 8..63 characters. */
    CHECK(net_config_set_psk(&c, "hunter22"));
    CHECK(strcmp(c.psk, "hunter22") == 0);

    CHECK(!net_config_set_psk(&c, "short7c"));
    CHECK(strcmp(c.psk, "hunter22") == 0);

    fill(buf, NET_PSK_MAX, 'x');
    CHECK(net_config_set_psk(&c, buf));
    CHECK(strlen(c.psk) == NET_PSK_MAX);

    fill(buf, NET_PSK_MAX + 1, 'y');
    CHECK(!net_config_set_psk(&c, buf));
    CHECK(strlen(c.psk) == NET_PSK_MAX);
    CHECK(c.psk[0] == 'x');
}

static void test_broker_and_port(void)
{
    net_config_t c;
    char buf[NET_BROKER_MAX + 8];
    net_config_set_defaults(&c);

    CHECK(net_config_set_broker(&c, "10.0.0.5", 1883));
    CHECK(strcmp(c.broker, "10.0.0.5") == 0);
    CHECK(c.port == 1883);

    CHECK(net_config_set_broker(&c, "broker.example.com", 65535));
    CHECK(c.port == 65535);

    /* Port 0 and anything above 65535 are rejected, atomically with the host. */
    CHECK(!net_config_set_broker(&c, "other.example.com", 0));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);
    CHECK(c.port == 65535);

    CHECK(!net_config_set_broker(&c, "other.example.com", 65536));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);

    CHECK(!net_config_set_broker(&c, "", 1883));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);

    fill(buf, NET_BROKER_MAX + 1, 'h');
    CHECK(!net_config_set_broker(&c, buf, 1883));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);
}

static void test_mqtt_credentials_may_be_empty(void)
{
    net_config_t c;
    char buf[NET_MQTT_PASS_MAX + 8];
    net_config_set_defaults(&c);

    /* An anonymous broker is legitimate, so empty is accepted here -- unlike
     * the SSID, where empty means "unprovisioned". */
    CHECK(net_config_set_user(&c, ""));
    CHECK(c.mqtt_user[0] == '\0');
    CHECK(net_config_set_mqtt_pass(&c, ""));
    CHECK(c.mqtt_pass[0] == '\0');

    CHECK(net_config_set_user(&c, "gateway1"));
    CHECK(strcmp(c.mqtt_user, "gateway1") == 0);
    CHECK(net_config_set_mqtt_pass(&c, "s3cret"));
    CHECK(strcmp(c.mqtt_pass, "s3cret") == 0);

    fill(buf, NET_MQTT_USER_MAX + 1, 'u');
    CHECK(!net_config_set_user(&c, buf));
    CHECK(strcmp(c.mqtt_user, "gateway1") == 0);

    fill(buf, NET_MQTT_PASS_MAX + 1, 'p');
    CHECK(!net_config_set_mqtt_pass(&c, buf));
    CHECK(strcmp(c.mqtt_pass, "s3cret") == 0);
}

static void test_is_provisioned(void)
{
    net_config_t c;
    net_config_set_defaults(&c);

    CHECK(!net_config_is_provisioned(&c));
    CHECK(net_config_set_ssid(&c, "MyNetwork"));
    /* SSID alone is not enough -- there is nowhere to publish. */
    CHECK(!net_config_is_provisioned(&c));
    CHECK(net_config_set_broker(&c, "10.0.0.5", 1883));
    CHECK(net_config_is_provisioned(&c));
}

static void test_singleton_starts_at_defaults(void)
{
    net_config_t *c;

    net_config_init();
    c = net_config_get();
    CHECK(c != NULL);
    CHECK(c->ssid[0] == '\0');
    CHECK(c->port == NET_MQTT_PORT_DEFAULT);
}

int main(void)
{
    test_defaults();
    test_ssid_bounds();
    test_psk_bounds();
    test_broker_and_port();
    test_mqtt_credentials_may_be_empty();
    test_is_provisioned();
    test_singleton_starts_at_defaults();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `net` command tree. Same contract as `anchor`: every setter validates,
 * then persists immediately, and changes take effect on the next boot.
 *
 * Secrets are never echoed. `net show` prints <set>/<unset> for the WiFi PSK
 * and the MQTT password, because the console output is exactly what ends up
 * pasted into a bug report.
 */

#include "net_config.h"
#include "net_store.h"
#include "net_uplink.h"

#include <zephyr/net/net_ip.h>
#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

/* strtoul() returns 0 on non-numeric input without reporting it, and the range
 * check alone would not catch a typo, so endptr does. Same helper shape as
 * anchor_shell.c's parse_ul(). */
static bool parse_ul(const char *arg, unsigned long *out)
{
	char *endptr;
	unsigned long v = strtoul(arg, &endptr, 0);

	if (endptr == arg || *endptr != '\0') {
		return false;
	}
	*out = v;
	return true;
}

static const char *secret_state(const char *s)
{
	return s[0] != '\0' ? "<set>" : "<unset>";
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	const net_config_t *c = net_config_get();
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!net_uplink_get_ip(ip, sizeof(ip))) {
		strcpy(ip, "none");
	}

	shell_print(sh,
		    "{\"ssid\":\"%s\",\"psk\":\"%s\",\"broker\":\"%s\",\"port\":%u,"
		    "\"user\":\"%s\",\"mqttpass\":\"%s\",\"provisioned\":%u,"
		    "\"state\":\"%s\",\"ip\":\"%s\"}",
		    c->ssid, secret_state(c->psk), c->broker, c->port,
		    c->mqtt_user, secret_state(c->mqtt_pass),
		    net_config_is_provisioned(c) ? 1u : 0u,
		    net_uplink_state_str(), ip);
	return 0;
}

static int cmd_ssid(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	if (!net_config_set_ssid(net_config_get(), argv[1])) {
		shell_error(sh, "error: ssid must be 1..%d characters", NET_SSID_MAX);
		return -EINVAL;
	}

	ret = net_store_save_ssid();
	if (ret) {
		shell_error(sh, "error: ssid applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: ssid set (saved) — reboot to apply");
	return 0;
}

static int cmd_pass(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	/* This is the WiFi PSK. The MQTT password is `net mqttpass`. */
	if (!net_config_set_psk(net_config_get(), argv[1])) {
		shell_error(sh, "error: WiFi passphrase must be %d..%d characters",
			    NET_PSK_MIN, NET_PSK_MAX);
		return -EINVAL;
	}

	ret = net_store_save_psk();
	if (ret) {
		shell_error(sh, "error: WiFi passphrase applied in RAM but NOT "
				"persisted (errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: WiFi passphrase set (saved) — reboot to apply");
	return 0;
}

static int cmd_broker(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long port = NET_MQTT_PORT_DEFAULT;
	int ret;

	if (argc == 3 && !parse_ul(argv[2], &port)) {
		shell_error(sh, "error: port must be a number in 1..65535");
		return -EINVAL;
	}

	if (!net_config_set_broker(net_config_get(), argv[1], (uint32_t)port)) {
		shell_error(sh, "error: broker must be 1..%d characters and "
				"port in 1..65535", NET_BROKER_MAX);
		return -EINVAL;
	}

	ret = net_store_save_broker();
	if (ret) {
		shell_error(sh, "error: broker applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: broker=%s:%lu (saved) — reboot to apply",
		    net_config_get()->broker, port);
	return 0;
}

static int cmd_user(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	if (!net_config_set_user(net_config_get(), argv[1])) {
		shell_error(sh, "error: user must be at most %d characters",
			    NET_MQTT_USER_MAX);
		return -EINVAL;
	}

	ret = net_store_save_user();
	if (ret) {
		shell_error(sh, "error: user applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: user set (saved) — reboot to apply");
	return 0;
}

static int cmd_mqttpass(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	/* This is the MQTT password. The WiFi passphrase is `net pass`. */
	if (!net_config_set_mqtt_pass(net_config_get(), argv[1])) {
		shell_error(sh, "error: MQTT password must be at most %d characters",
			    NET_MQTT_PASS_MAX);
		return -EINVAL;
	}

	ret = net_store_save_mqtt_pass();
	if (ret) {
		shell_error(sh, "error: MQTT password applied in RAM but NOT "
				"persisted (errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: MQTT password set (saved) — reboot to apply");
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	net_config_set_defaults(net_config_get());

	ret = net_store_save_all();
	if (ret) {
		shell_error(sh, "error: defaults applied in RAM but NOT fully "
				"persisted (errno %d)", ret);
		return ret;
	}

	shell_print(sh, "ok: network defaults restored (saved) — reboot to apply");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_net,
	SHELL_CMD_ARG(show,     NULL, "Print the network configuration as JSON",
		      cmd_show,     1, 0),
	SHELL_CMD_ARG(ssid,     NULL, "ssid <ssid> — set the WiFi SSID",
		      cmd_ssid,     2, 0),
	SHELL_CMD_ARG(pass,     NULL, "pass <psk> — set the WiFi passphrase",
		      cmd_pass,     2, 0),
	SHELL_CMD_ARG(broker,   NULL, "broker <host> [port] — set the MQTT broker "
				      "(port defaults to 1883)",
		      cmd_broker,   2, 1),
	SHELL_CMD_ARG(user,     NULL, "user <username> — set the MQTT username",
		      cmd_user,     2, 0),
	SHELL_CMD_ARG(mqttpass, NULL, "mqttpass <password> — set the MQTT password",
		      cmd_mqttpass, 2, 0),
	SHELL_CMD_ARG(reset,    NULL, "Restore the network defaults and persist them",
		      cmd_reset,    1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(net, &sub_net, "Gateway network and MQTT configuration", NULL);

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "node_cmd.h"

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include "ccp_slave.h"
#include "cmd_key.h"
#include "cmd_proto.h"
#include "pos_json.h"
#include "uwb_config.h"
#include "uwb_store.h"

LOG_MODULE_REGISTER(node_cmd, LOG_LEVEL_INF);

/* One command per second, accepted or not: the HMAC makes forging one infeasible, this just keeps
 * a flood from costing the uplink thread its time. */
#define MIN_GAP_MS 1000

/* Same bound the shell enforces implicitly via int32; a flight time beyond ~100 us (30 km) is a
 * typo, not a deployment. */
#define TAU_ABS_MAX_DTU 6400000

static struct cmd_challenge chal;
static bool    is_gw;
static char    my_node[17];   /* EUI16 or "gw", see node_cmd_init() */
static int64_t last_ms = INT64_MIN / 2;

static void rotate_with_fresh_random(void)
{
	cmd_challenge_rotate(&chal, sys_rand32_get(), sys_rand32_get());
}

void node_cmd_init(bool is_gateway, const char *node)
{
	is_gw = is_gateway;
	snprintf(my_node, sizeof(my_node), "%s", (node != NULL) ? node : "");
	cmd_challenge_init(&chal);
	rotate_with_fresh_random();
}

const char *node_cmd_challenge(void)
{
	return chal.have_cur ? chal.cur : "";
}

void node_cmd_rotate(void)
{
	rotate_with_fresh_random();
}

static struct node_cmd_result reply(char *ack, size_t cap, const char *name, bool ok,
				    const char *err, bool has_v, int32_t v)
{
	struct node_cmd_result r = { .ack_len = cmd_ack_json(ack, cap, name, ok, err, has_v, v) };

	return r;
}

/* The operations. Each returns NULL on success or the error string for the ack. */

static const char *do_tau_set(const struct cmd_msg *m, int32_t *value)
{
	uwb_config_t *cfg = uwb_config_get();

	if (!m->has_a || m->a > TAU_ABS_MAX_DTU || m->a < -TAU_ABS_MAX_DTU) {
		return "bad_arg";
	}
	uwb_config_set_sync_tau(cfg, m->a);
	if (uwb_store_save_sync_tau() != 0) {
		return "not_persisted";
	}
	ccp_slave_set_tau(uwb_config_sync_tau_effective(cfg));

	/* A hand-set tau is no longer one a survey installed: the link record must stop vouching for
	 * it, exactly as `sync tau set` does. */
	if (cfg->sync_link_valid) {
		uwb_config_clear_sync_link(cfg);
		(void)uwb_store_save_sync_link();
	}
	*value = ccp_slave_get_tau();
	return NULL;
}

static const char *do_tau_mode(const struct cmd_msg *m, int32_t *value)
{
	uwb_config_t *cfg = uwb_config_get();
	uint8_t mode;

	if (!m->has_a || (m->a != 0 && m->a != 1)) {
		return "bad_arg";
	}
	mode = (m->a == 0) ? SYNC_TAU_MODE_LEGACY : SYNC_TAU_MODE_SURVEY;
	if (!uwb_config_set_sync_tau_mode(cfg, mode)) {
		return "bad_arg";
	}
	if (uwb_store_save_sync_tau_mode() != 0) {
		return "not_persisted";
	}
	ccp_slave_set_tau(uwb_config_sync_tau_effective(cfg));
	*value = ccp_slave_get_tau();
	return NULL;
}

static const char *do_ant_set(const struct cmd_msg *m, int32_t *value)
{
	uwb_config_t *cfg = uwb_config_get();

	if (!m->has_a || !m->has_b || m->a < 0 || m->a > 65535 || m->b < 0 || m->b > 65535) {
		return "bad_arg";
	}
	if (!uwb_config_set_ant(cfg, (uint32_t)m->a, (uint32_t)m->b)) {
		return "bad_arg";
	}
	if (uwb_store_save_ant() != 0) {
		return "not_persisted";
	}
	*value = m->a;                  /* applies on the next boot, like `anchor ant` */
	return NULL;
}

struct node_cmd_result node_cmd_handle(const char *payload, size_t len, char *ack, size_t ack_cap)
{
	struct cmd_msg m;
	int64_t now = k_uptime_get();
	struct node_cmd_result none = { .ack_len = -1 };

	if (now - last_ms < MIN_GAP_MS) {
		return reply(ack, ack_cap, "?", false, "rate_limited", false, 0);
	}
	last_ms = now;

	int pr = cmd_parse(payload, len, &m);

	if (pr == CMD_E_FORMAT) {
		return reply(ack, ack_cap, "?", false, "bad_format", false, 0);
	}
	if (pr == CMD_E_UNKNOWN) {
		return reply(ack, ack_cap, m.name, false, "unknown_cmd", false, 0);
	}

	if (!cmd_key_is_set()) {
		return reply(ack, ack_cap, m.name, false, "disabled", false, 0);   /* fail closed */
	}
	size_t klen = 0;
	const uint8_t *key = cmd_key_get(&klen);

	if (!cmd_challenge_accepts(&chal, m.ch)) {
		return reply(ack, ack_cap, m.name, false, "stale_challenge", false, 0);
	}
	/* Signed against the EUI, not the anchor id: two boards sharing an id
	 * must not both accept one signed command. */
	if (!cmd_verify(&m, key, klen, POS_JSON_ZONE_NAME, my_node)) {
		return reply(ack, ack_cap, m.name, false, "bad_sig", false, 0);
	}

	/* Authentic and fresh. Burn the challenge BEFORE acting, so no path -- including a command
	 * that reboots -- can leave one signature valid twice. */
	cmd_challenge_burn(&chal);
	rotate_with_fresh_random();

	if (is_gw && m.id != CMD_STATUS && m.id != CMD_REBOOT) {
		return reply(ack, ack_cap, m.name, false, "not_supported", false, 0);
	}

	const char *err = NULL;
	int32_t value = 0;
	bool has_value = false;
	struct node_cmd_result r = none;

	switch (m.id) {
	case CMD_STATUS:
		r.status_now = true;
		break;
	case CMD_SYNC_RESET:
		ccp_slave_residual_reset();
		break;
	case CMD_TAU_SET:
		err = do_tau_set(&m, &value);
		has_value = (err == NULL);
		break;
	case CMD_TAU_MODE:
		err = do_tau_mode(&m, &value);
		has_value = (err == NULL);
		break;
	case CMD_ANT_SET:
		err = do_ant_set(&m, &value);
		has_value = (err == NULL);
		break;
	case CMD_REBOOT:
		r.reboot = true;
		break;
	default:
		err = "unknown_cmd";
		break;
	}

	LOG_WRN("remote command %s -> %s", m.name, err ? err : "ok");

	struct node_cmd_result out = reply(ack, ack_cap, m.name, err == NULL, err, has_value, value);

	out.status_now = r.status_now || (err == NULL && m.id != CMD_STATUS && m.id != CMD_REBOOT);
	out.reboot = r.reboot && err == NULL;
	return out;
}

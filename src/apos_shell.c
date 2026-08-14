/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `apos` command tree. Gateway-only in practice -- a SLAVE has no survey to
 * orchestrate -- but registered unconditionally, because refusing at the shell
 * with a clear message beats a command that silently does not exist on the board
 * the operator happens to be plugged into.
 *
 * No command transmits. Each sets state and returns; the gateway loop does the
 * radio work and logs the outcome. Two threads on the DW3220's SPI bus at once
 * would corrupt both -- the same rule cal_shell.c follows.
 */

#include "apos_gw.h"
#include "apos_store.h"
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

/* Same reasoning as anchor_shell.c's parse_ul: strtoul() reports a non-numeric
 * argument as 0, and 0 must be rejected here rather than accepted as an
 * address. Accepts 0x-prefixed input, which is how `apos enum` prints them. */
static bool parse_addr(const char *arg, uint16_t *out)
{
	char *endptr;
	unsigned long v = strtoul(arg, &endptr, 0);

	if (endptr == arg || *endptr != '\0' || v == 0u || v > 0xFFFFu) {
		return false;
	}
	*out = (uint16_t)v;
	return true;
}

static int require_gateway(const struct shell *sh)
{
	if (uwb_config_get()->mode != UWB_MODE_GATEWAY) {
		shell_error(sh, "error: `apos` runs on the GATEWAY — this board "
				"is a %s. Set `anchor mode gateway` and reboot.",
			    uwb_config_mode_name(uwb_config_get()->mode));
		return -ENOTSUP;
	}
	return 0;
}

static int cmd_enum(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	rc = apos_gw_start_enum();
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: enumeration refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "enumerating — peers are logged as they answer, then "
			"run `apos show`");
	return 0;
}

/* origin=<addr> xaxis=<addr> plane=<addr> up=<addr>, in any order. Named rather
 * than positional because four bare hex addresses in a row is exactly the kind
 * of argument list that gets silently transposed, and a transposed gauge
 * produces a plausible-looking but wrong coordinate frame. */
static int cmd_gauge(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const keys[4] = {"origin=", "xaxis=", "plane=",
					    "up="};
	uint16_t val[4] = {0, 0, 0, 0};

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	for (size_t a = 1; a < argc; a++) {
		bool matched = false;

		for (int k = 0; k < 4; k++) {
			size_t klen = strlen(keys[k]);

			if (strncmp(argv[a], keys[k], klen) != 0) {
				continue;
			}
			if (!parse_addr(argv[a] + klen, &val[k])) {
				shell_error(sh, "error: bad address in \"%s\"",
					    argv[a]);
				return -EINVAL;
			}
			matched = true;
			break;
		}
		if (!matched) {
			shell_error(sh, "error: unexpected argument \"%s\" — "
					"expected origin=/xaxis=/plane=/up=",
				    argv[a]);
			return -EINVAL;
		}
	}

	for (int k = 0; k < 4; k++) {
		if (val[k] == 0u) {
			shell_error(sh, "error: missing %s<addr>", keys[k]);
			return -EINVAL;
		}
	}

	rc = apos_gw_set_gauge(val[0], val[1], val[2], val[3]);
	if (rc == -EINVAL) {
		shell_error(sh, "error: the four addresses must be distinct and "
				"none may be 0x0000 (the gateway)");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: gauge refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "{\"apos_gauge\":{\"origin\":\"0x%04X\","
			"\"xaxis\":\"0x%04X\",\"plane\":\"0x%04X\","
			"\"up\":\"0x%04X\"}}",
		    val[0], val[1], val[2], val[3]);
	return 0;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct apos_table *t = apos_gw_table();
	const struct apos_survey *s = apos_store_get();
	struct apos_gw_status st;

	apos_gw_get_status(&st);

	shell_print(sh, "{\"phase\":%u,\"session\":%u,\"gauge_set\":%u,"
			"\"have_result\":%u}",
		    st.phase, st.session, apos_gw_gauge_set() ? 1u : 0u,
		    st.have_result ? 1u : 0u);

	shell_print(sh, "enumerated (%u):", t->n_peers);
	for (uint8_t k = 0; k < t->n_peers; k++) {
		shell_print(sh, "  {\"idx\":%u,\"addr\":\"0x%04X\","
				"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\","
				"\"pos_valid\":%u}",
			    k, t->peer[k].short_addr,
			    t->peer[k].eui[0], t->peer[k].eui[1],
			    t->peer[k].eui[2], t->peer[k].eui[3],
			    t->peer[k].eui[4], t->peer[k].eui[5],
			    t->peer[k].eui[6], t->peer[k].eui[7],
			    t->peer[k].pos_valid ? 1u : 0u);
	}

	shell_print(sh, "stored survey: %s", s->valid ? "yes" : "none");
	for (uint8_t k = 0; k < s->n_nodes; k++) {
		shell_print(sh, "  {\"addr\":\"0x%04X\",\"x\":%.3f,\"y\":%.3f,"
				"\"z\":%.3f}",
			    s->node[k].short_addr, (double)s->node[k].x,
			    (double)s->node[k].y, (double)s->node[k].z);
	}
	if (s->ref_valid) {
		shell_print(sh, "  {\"ref_lat\":%.6f,\"ref_lon\":%.6f}",
			    s->ref_lat, s->ref_lon);
	} else {
		shell_print(sh, "  {\"ref\":\"unset — run `apos ref <lat> <lon>` "
				"or the platform cannot place the map\"}");
	}
	return 0;
}

static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	rc = apos_gw_start_run();
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc == -EINVAL) {
		shell_error(sh, "error: set the frame first — `apos gauge "
				"origin=<addr> xaxis=<addr> plane=<addr> "
				"up=<addr>` (run `apos enum` to list addresses)");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: run refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "running — this takes a few seconds per anchor pair and "
			"reports as JSON when it finishes. NOTHING is persisted; "
			"run `apos apply` afterwards to commit.");
	return 0;
}

static int cmd_zoff(const struct shell *sh, size_t argc, char **argv)
{
	char *endptr;
	float v;

	ARG_UNUSED(argc);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	/* strtof, not strtod: anchor_shell.c's coordinate parser already uses
	 * it, and everything downstream (apos_gw_set_zoff, apos_geom_zoff) is
	 * float — parsing to double only to narrow it again buys nothing. */
	v = strtof(argv[1], &endptr);
	if (endptr == argv[1] || *endptr != '\0') {
		shell_error(sh, "error: \"%s\" is not a number of metres",
			    argv[1]);
		return -EINVAL;
	}

	apos_gw_set_zoff(v);
	shell_print(sh, "{\"apos_zoff_m\":%.3f} — takes effect on the next "
			"`apos run`", (double)v);
	return 0;
}

/* The unverified-mesh caveat, repeated at the point of commit.
 *
 * apos_gw_start_apply() also logs it, but the log and the console are not the
 * same audience: an operator typing `apos apply` gets this back synchronously,
 * under their own command, whereas the LOG_WRN lands in whatever the monitor
 * happens to be scrolling. Apply is the point of no return, so it is said in
 * both places -- and it is printed whether or not `force` was given, because
 * `force` overrides ACCEPTANCE, not physics. */
static void warn_unverified(const struct shell *sh)
{
	if (!apos_gw_result_unverified()) {
		return;
	}

	shell_warn(sh, "WARNING: this survey is UNVERIFIED. The mesh has %d "
		       "spare edge(s) (usable edges minus 3N-6), so the fit "
		       "reproduced the ranges exactly and rms/worst came back "
		       "at ~0 however bad the ranging was.",
		   apos_gw_result_redundancy());
	shell_warn(sh, "A PASS here means only that nothing contradicted the "
		       "ranges — NOT that they are correct. These coordinates "
		       "are being written to every anchor's NVS now. Check the "
		       "solved node-to-node distances against a tape measure, "
		       "or add a fifth anchor so the mesh has a spare edge.");
}

static int cmd_apply(const struct shell *sh, size_t argc, char **argv)
{
	bool force = (argc > 1) && (strcmp(argv[1], "force") == 0);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}
	if (argc > 1 && !force) {
		shell_error(sh, "error: the only argument is `force`");
		return -EINVAL;
	}

	rc = apos_gw_start_apply(force);
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc == -ENODATA) {
		shell_error(sh, "error: no result to apply — run `apos run` first");
		return rc;
	}
	if (rc == -EPERM) {
		const struct apos_result *r = apos_gw_result();

		shell_error(sh, "error: the last run FAILED acceptance "
				"(rms=%d mm, worst=%d mm on pair [%u,%u], "
				"placed=%u/%u, ambiguous=%u). Fix the geometry "
				"and re-run, or `apos apply force` to commit it "
				"anyway.",
			    (int)(r->rms_m * 1000.0f),
			    (int)(r->worst_edge_m * 1000.0f),
			    r->worst_i, r->worst_j, r->n_placed, r->n_nodes,
			    r->n_ambiguous);
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: apply refused (errno %d)", rc);
		return rc;
	}

	warn_unverified(sh);

	shell_print(sh, "applying — each anchor is pushed its coordinates and "
			"must acknowledge; watch for apos_apply_done");
	return 0;
}

static int cmd_ref(const struct shell *sh, size_t argc, char **argv)
{
	char *e1, *e2;
	double lat, lon;

	ARG_UNUSED(argc);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	lat = strtod(argv[1], &e1);
	lon = strtod(argv[2], &e2);
	if (e1 == argv[1] || *e1 != '\0' || e2 == argv[2] || *e2 != '\0') {
		shell_error(sh, "error: lat and lon must be decimal degrees");
		return -EINVAL;
	}
	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
		shell_error(sh, "error: lat must be -90..90 and lon -180..180");
		return -EINVAL;
	}

	rc = apos_store_set_ref(lat, lon);
	if (rc) {
		shell_error(sh, "error: reference NOT persisted (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "{\"apos_ref\":{\"lat\":%.6f,\"lon\":%.6f}} — this is "
			"the origin anchor's real-world position; the platform "
			"places the whole survey against it",
		    lat, lon);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_apos,
	SHELL_CMD_ARG(enum,  NULL,
		      "enum — discover anchors over the air and print their "
		      "EUI-64 and short address",
		      cmd_enum, 1, 0),
	SHELL_CMD_ARG(gauge, NULL,
		      "gauge origin=<addr> xaxis=<addr> plane=<addr> up=<addr> "
		      "— pin the coordinate frame",
		      cmd_gauge, 5, 0),
	SHELL_CMD_ARG(run,   NULL,
		      "run — range every anchor pair, solve, and REPORT ONLY "
		      "(persists nothing)",
		      cmd_run, 1, 0),
	SHELL_CMD_ARG(apply, NULL,
		      "apply [force] — push the last result to every anchor, "
		      "persist it, and close the survey",
		      cmd_apply, 1, 1),
	SHELL_CMD_ARG(ref,   NULL,
		      "ref <lat> <lon> — the origin anchor's real-world "
		      "position, for the platform map",
		      cmd_ref, 3, 0),
	SHELL_CMD_ARG(zoff,  NULL,
		      "zoff <metres> — shift z so z=0 is the floor rather than "
		      "the plane through the gauge anchors",
		      cmd_zoff, 2, 0),
	SHELL_CMD_ARG(show,  NULL,
		      "show — current phase, enumerated anchors and the stored "
		      "survey, as JSON",
		      cmd_show, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(apos, &sub_apos, "Anchor auto-positioning", NULL);

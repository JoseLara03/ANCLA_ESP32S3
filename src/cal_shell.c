/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `cal` command tree. Calibration image only.
 *
 * Both commands submit to cal_run()'s loop and block; the radio work never
 * happens on the shell thread, because two threads on the DW3220's SPI bus at
 * once would corrupt both.
 */

#include "cal_run.h"
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

/* Same reasoning as anchor_shell.c's parse_ul: strtoul() reports a non-numeric
 * argument as 0, and 0 is a legal distance. */
static bool parse_l(const char *arg, long *out)
{
	char *endptr;
	long v;

	v = strtol(arg, &endptr, 0);
	if (endptr == arg || *endptr != '\0') {
		return false;
	}

	*out = v;
	return true;
}

static void print_result(const struct shell *sh, const struct cal_result *r)
{
	shell_print(sh,
		    "{\"attempted\":%u,\"valid\":%u,\"kept\":%u,"
		    "\"mean_mm\":%d,\"ref_mm\":%d,\"error_mm\":%d}",
		    r->attempted, r->valid, (unsigned int)r->kept,
		    r->mean_mm, r->ref_mm, r->error_mm);
}

static void print_failure(const struct shell *sh, int status)
{
	switch (status) {
	case -ENODATA:
		shell_error(sh,
			    "error: too few valid responses — check the peer is "
			    "powered, addressed correctly, and on the same PHY");
		break;
	case -EBUSY:
		shell_error(sh, "error: a calibration batch is already running");
		break;
	case -ETIMEDOUT:
		shell_error(sh, "error: the ranging loop did not answer");
		break;
	default:
		shell_error(sh, "error: calibration failed (errno %d)", status);
		break;
	}
}

static int cmd_peer(const struct shell *sh, size_t argc, char **argv)
{
	struct cal_request req = {0};
	struct cal_result res = {0};
	long id, mm;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_l(argv[1], &id) || id < 0 || id >= UWB_MAX_ANCHORS) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}
	if (!parse_l(argv[2], &mm) || mm <= 0) {
		shell_error(sh, "error: distance must be a positive integer in mm");
		return -EINVAL;
	}

	/* Console ids are 0-based; the wire id is UWB_ANCHOR_ADDR_BASE +
	 * anchor_id, and the responder filters on its low byte. */
	req.peer_wire_id = (uint8_t)((UWB_ANCHOR_ADDR_BASE + id) & 0xFFu);
	req.ref_mm = (int32_t)mm;
	req.persist = false;

	ret = cal_run_execute(&req, &res);
	if (ret) {
		print_failure(sh, ret);
		return ret;
	}

	print_result(sh, &res);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cal,
	SHELL_CMD_ARG(peer, NULL,
		      "peer <id> <mm> — range anchor <id> at a known distance; "
		      "reports only, never persists",
		      cmd_peer, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cal, &sub_cal, "Antenna-delay calibration", NULL);

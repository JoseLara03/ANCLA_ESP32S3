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
#include "cal_math.h"   /* CAL_MAX_SAMPLES, for the -ENODATA gate */
#include "uwb_config.h"
#include "uwb_store.h"

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

static void print_failure(const struct shell *sh, const struct cal_result *r,
			  int status)
{
	switch (status) {
	case -ENODATA:
		/* The counts are the one thing that IS valid here, and on a
		 * RANGE test they are the measurement -- `valid` out of a
		 * fixed 128 attempts is the exchange success rate, and the
		 * region below this gate is exactly the region a range test
		 * cares about (see docs/range-test.md). Printing only the
		 * error message threw them away, and pointed at three causes
		 * that are all wrong when the real one is distance. */
		shell_print(sh,
			    "{\"attempted\":%u,\"valid\":%u,\"ref_mm\":%d}",
			    r->attempted, r->valid, r->ref_mm);
		shell_error(sh,
			    "error: only %u of %u exchanges succeeded (under "
			    "the %u needed for a calibration) -- no distance "
			    "reported. If the peer is powered, addressed "
			    "correctly and on the same PHY, this is the link: "
			    "the count above is still a usable range "
			    "measurement",
			    r->valid, r->attempted, CAL_MAX_SAMPLES / 4U);
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
		print_failure(sh, &res, ret);
		return ret;
	}

	print_result(sh, &res);
	return 0;
}

static int cmd_ref(const struct shell *sh, size_t argc, char **argv)
{
	struct cal_request req = {0};
	struct cal_result res = {0};
	uwb_config_t *cfg = uwb_config_get();
	long mm;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_l(argv[1], &mm) || mm <= 0) {
		shell_error(sh, "error: distance must be a positive integer in mm");
		return -EINVAL;
	}

	req.peer_wire_id = CAL_PEER_REFERENCE;
	req.ref_mm = (int32_t)mm;
	req.persist = true;

	ret = cal_run_execute(&req, &res);
	if (ret == -ERANGE) {
		shell_error(sh,
			    "error: solved ant_tx out of range (would be clamped "
			    "to %u) from error=%d mm — NOT applied. Check the "
			    "reference distance and that the peer is the "
			    "reference node.",
			    res.new_tx, res.error_mm);
		return ret;
	}
	if (ret) {
		print_failure(sh, &res, ret);
		return ret;
	}

	print_result(sh, &res);

	/* cal_run() has already applied the new delay to the radio and to its
	 * own live config. This updates the shared config singleton and writes
	 * NVS, so the value survives a reboot. */
	if (!uwb_config_set_ant(cfg, res.new_tx, cfg->ant_delay_rx)) {
		shell_error(sh, "error: ant_tx=%u rejected by the config layer",
			    res.new_tx);
		return -EINVAL;
	}

	ret = uwb_store_save_ant();
	if (ret) {
		shell_error(sh,
			    "error: ant_tx %u -> %u applied to the radio but NOT "
			    "persisted (errno %d) — will be lost on reboot",
			    res.old_tx, res.new_tx, ret);
		return ret;
	}

	shell_print(sh,
		    "ok: ant_tx %u -> %u (applied and saved) — run `cal ref %ld` "
		    "again to read the residual",
		    res.old_tx, res.new_tx, mm);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cal,
	SHELL_CMD_ARG(ref,  NULL,
		      "ref <mm> — calibrate against the reference node at a "
		      "known distance; applies and persists ant_tx",
		      cmd_ref, 2, 0),
	SHELL_CMD_ARG(peer, NULL,
		      "peer <id> <mm> — range anchor <id> at a known distance; "
		      "reports only, never persists",
		      cmd_peer, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cal, &sub_cal, "Antenna-delay calibration", NULL);

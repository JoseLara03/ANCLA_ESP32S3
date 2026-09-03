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

/* sqrt of a per-mille probability, in per-mille: sqrt(p/1000)*1000 =
 * sqrt(p*1000). Integer Newton, same reason cal_solve.c's is integer -- a
 * reported number must not differ between a host and the target. */
static uint32_t isqrt_permille(uint32_t p_permille)
{
	uint32_t v = p_permille * 1000u;
	uint32_t r, prev;

	if (v == 0u) return 0u;
	r = v;
	prev = 0u;
	while (r != prev) {
		prev = r;
		r = (r + v / r) / 2u;
	}
	while ((r + 1u) * (r + 1u) <= v) r++;
	while (r * r > v) r--;
	return r;
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

/* Success rate is a CLIFF; a link already failing shows up in the SPREAD and
 * in the received level long before the count moves. That is why this reports
 * sd/min/max and dBm and `cal peer` does not.
 *
 * p_oneway = sqrt(p_exchange) assumes the two directions are equally reliable.
 * That holds for anchor <-> anchor -- identical hardware at both ends and a
 * reciprocal channel -- and would NOT hold against a tag, which has no PA. It
 * is what lets this project the DS-TWR exchange rate WITHOUT implementing
 * DS-TWR: any TWR variant's exchange rate is p_oneway^(frames), so 2 frames
 * for SS-TWR and 3 for DS-TWR. See docs/range-test.md 2. */
static int cmd_link(const struct shell *sh, size_t argc, char **argv)
{
	struct cal_request req = {0};
	struct cal_result res = {0};
	long id, n = 0;
	int ret;

	if (!parse_l(argv[1], &id) || id < 0 || id >= UWB_MAX_ANCHORS) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}
	if (argc > 2 && (!parse_l(argv[2], &n) || n < 1 || n > 512)) {
		shell_error(sh, "error: attempts must be 1..512");
		return -EINVAL;
	}

	req.peer_wire_id = (uint8_t)((UWB_ANCHOR_ADDR_BASE + id) & 0xFFu);
	req.link = true;
	req.attempts = (uint32_t)n;
	req.persist = false;

	ret = cal_run_execute(&req, &res);
	if (ret) {
		print_failure(sh, &res, ret);
		return ret;
	}

	/* Rates in per-mille, integer: this is a report, and an integer cannot
	 * pick up a different last digit between two builds. */
	uint32_t p_exch = res.attempted ?
		(uint32_t)((uint64_t)res.valid * 1000u / res.attempted) : 0u;
	uint32_t p_one = (uint32_t)(isqrt_permille(p_exch));
	/* p_oneway^3 in one division, not two: chaining /1000 truncates twice
	 * and drifts. p_one <= 1000 so the cube is at most 1e9. */
	uint32_t p_ds  = (uint32_t)((uint64_t)p_one * p_one * p_one / 1000000u);

	shell_print(sh,
		    "{\"link\":{\"peer\":%d,\"attempted\":%u,\"valid\":%u,"
		    "\"p_exch_permille\":%u,\"p_oneway_permille\":%u,"
		    "\"p_dstwr_projected_permille\":%u,"
		    "\"stats_over\":%u,\"mean_mm\":%d,\"sd_mm\":%d,"
		    "\"min_mm\":%d,\"max_mm\":%d,"
		    "\"rx_dbm_x10\":{\"mean\":%d,\"min\":%d,\"max\":%d,\"n\":%u},"
		    "\"fail\":{\"tx_start\":%u,\"tx_done\":%u,\"rx_to_err\":%u,"
		    "\"len\":%u,\"hdr\":%u,\"layout\":%u}}}",
		    (int)id, res.attempted, res.valid,
		    p_exch, p_one, p_ds,
		    (unsigned int)((res.valid < CAL_MAX_SAMPLES) ?
				   res.valid : CAL_MAX_SAMPLES),
		    res.mean_mm, res.sd_mm, res.min_mm, res.max_mm,
		    res.rx_level_mean_x10, res.rx_level_min_x10,
		    res.rx_level_max_x10, res.rx_level_n,
		    res.f_tx_start, res.f_tx_done, res.f_rx_to_err,
		    res.f_len, res.f_hdr, res.f_layout);

	/* The one threshold an operator would otherwise have to look up.
	 * apos_node.h's batch deadline plus APOS_MIN_N_OK put the survey's
	 * floor at p_exch >= 0.363; below it the real survey runs out of its
	 * 700 ms deadline before collecting enough successes, whatever the
	 * link is doing. */
	if (p_exch < 363u) {
		shell_warn(sh,
			   "exchange rate %u.%u %% is below the survey's floor "
			   "of 36.3 %% -- `apos run` would report this pair as "
			   "unusable at this distance",
			   p_exch / 10u, p_exch % 10u);
	}
	if (res.rx_level_n == 0u && res.valid > 0u) {
		shell_warn(sh,
			   "no RX level: the CIA never reported done. Not a "
			   "weak signal -- check dwt_configciadiag() ran");
	}

	return 0;
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
	SHELL_CMD_ARG(link, NULL,
		      "link <id> [attempts] - range/link quality against a peer. "
		      "No reference distance: reports exchange success rate, "
		      "distance spread and RX level in dBm. Never persists.",
		      cmd_link, 2, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cal, &sub_cal, "Antenna-delay calibration", NULL);

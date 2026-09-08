/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `gw` command tree: gateway-side load-testing utilities. Today this is
 * just `gw seed <n>`, which fills the live seat table with fabricated tags so
 * the beacon and its slot maps can be exercised at full occupancy with no RF
 * from phantom devices. Gateway-only, same reasoning and refusal style as
 * apos_shell.c: a SLAVE has nothing for this command to act on, and refusing
 * at the shell with a clear message beats a command that silently does not
 * exist on the board the operator happens to be plugged into.
 *
 * Like apos_shell.c, this command never touches the seat table directly: it
 * only records a request that the gateway's own K_PRIO_COOP(0) loop notices
 * and processes on its own thread (uwb_gateway_request_seed(), implemented in
 * uwb_gateway.c next to the table it protects). Two threads touching
 * gw_core_ctx at once would corrupt it exactly the way two threads on the
 * DW3000's SPI bus at once would corrupt that.
 */

#include "gw_core.h"
#include "uwb_config.h"
#include "uwb_modes.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

static int require_gateway(const struct shell *sh)
{
	if (uwb_config_get()->mode != UWB_MODE_GATEWAY) {
		shell_error(sh, "error: `gw` runs on the GATEWAY — this board "
				"is a %s. Set `anchor mode gateway` and reboot.",
			    uwb_config_mode_name(uwb_config_get()->mode));
		return -ENOTSUP;
	}
	return 0;
}

/* `gw seed <n>` intentionally does NOT read the live seat table before
 * accepting a request: this command runs on the SHELL thread and the table
 * belongs to the GATEWAY loop's own thread (see uwb_gateway.c), so there is
 * no safe way to peek at current occupancy from here without the same race
 * this whole design exists to avoid. Instead `gw seed` is documented, and
 * warned about, as a load-test aid for a FRESHLY BOOTED, otherwise-empty
 * gateway -- not a way to top up a live deployment with fake tags alongside
 * real ones. The gateway loop still checks for real: if the table was not in
 * fact empty, gw_core_join() starts failing partway through the fill and
 * do_seed_fill() logs exactly how many synthetic seats actually landed. */
static int cmd_seed(const struct shell *sh, size_t argc, char **argv)
{
	char *endptr;
	long n;

	ARG_UNUSED(argc);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	n = strtol(argv[1], &endptr, 0);
	if (endptr == argv[1] || *endptr != '\0' || n <= 0) {
		shell_error(sh, "error: \"%s\" is not a positive integer",
			    argv[1]);
		return -EINVAL;
	}
	if (!gw_seed_valid_count((uint32_t)n)) {
		shell_error(sh, "error: n must be 1..%u (GW_MAX_SEATS — the "
				"table's total capacity across every phase)",
			    (unsigned)GW_MAX_SEATS);
		return -EINVAL;
	}

	/* Loud at the point of invocation -- this is the warning an operator
	 * typing the command sees synchronously, under their own prompt. */
	shell_warn(sh, "WARNING: this fills the seat table with %ld FAKE tag(s) "
			"for load testing. It assumes a freshly booted, "
			"otherwise-EMPTY gateway — running it against a live "
			"deployment mixes phantom occupancy in with real tags. "
			"This must never be mistaken for a real deployment "
			"state.", n);

	rc = uwb_gateway_request_seed((uint32_t)n);
	if (rc == -EBUSY) {
		shell_error(sh, "error: an earlier `gw seed` request has not "
				"been picked up by the gateway loop yet — "
				"try again shortly");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: seed request refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "requested — the gateway loop performs the fill on its "
			"own thread and logs the result (look for "
			"\"SYNTHETIC SEED\" in the console/monitor)");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gw,
	SHELL_CMD_ARG(seed, NULL,
		      "seed <n> — fill n FAKE tag seats for load testing "
		      "(gateway-only; assumes a freshly booted, empty table)",
		      cmd_seed, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gw, &sub_gw, "Gateway load-test utilities", NULL);

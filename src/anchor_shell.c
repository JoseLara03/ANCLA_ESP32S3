/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `anchor` command tree, over the native USB-JTAG console.
 *
 * Every setter validates, then persists immediately — there is no separate
 * `save` command, so there is no "I set it but forgot to save" failure. The
 * running mode is not disturbed: changes take effect on the next boot, which
 * is what lets the UWB thread treat the config as read-only and skip locking.
 */

#include "uwb_config.h"
#include "uwb_store.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

/* strtoul()/strtof() return 0 on non-numeric input without reporting the
 * failure, and 0 is a legal value for every field this file parses — so a
 * typo has to be caught here via endptr, not left to the range check below.
 * Rejects if nothing was consumed or if trailing garbage remains. */
static bool parse_ul(const char *arg, unsigned long *out)
{
	char *endptr;
	unsigned long v;

	v = strtoul(arg, &endptr, 0);
	if (endptr == arg || *endptr != '\0') {
		return false;
	}

	*out = v;
	return true;
}

static bool parse_flt(const char *arg, float *out)
{
	char *endptr;
	float v;

	v = strtof(arg, &endptr);
	if (endptr == arg || *endptr != '\0') {
		return false;
	}

	*out = v;
	return true;
}

static void print_config(const struct shell *sh)
{
	const uwb_config_t *cfg = uwb_config_get();

	shell_print(sh,
		    "{\"mode\":\"%s\",\"id\":%u,\"short_addr\":\"0x%04X\","
		    "\"cell_mode\":\"%s\","
		    "\"ant_tx\":%u,\"ant_rx\":%u,"
		    "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		    uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		    uwb_config_short_addr(cfg),
		    uwb_config_cell_mode_name(cfg->cell_mode),
		    cfg->ant_delay_tx, cfg->ant_delay_rx,
		    (double)cfg->x, (double)cfg->y, (double)cfg->z,
		    cfg->position_valid ? 1u : 0u);
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_config(sh);
	return 0;
}

static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	unsigned long v;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_ul(argv[1], &v) || v > UINT8_MAX || !uwb_config_set_id(cfg, (uint8_t)v)) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}

	ret = uwb_store_save_id();
	if (ret) {
		shell_error(sh,
			    "error: anchor_id=%u applied in RAM but NOT persisted "
			    "(errno %d) — will be lost on reboot",
			    cfg->anchor_id, ret);
		return ret;
	}

	shell_print(sh, "ok: anchor_id=%u (saved) — reboot to apply", cfg->anchor_id);
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	uint8_t mode;
	int ret;

	ARG_UNUSED(argc);

	if (!uwb_config_mode_from_name(argv[1], &mode)) {
		shell_error(sh, "error: mode must be slave or gateway");
		return -EINVAL;
	}

	uwb_config_set_mode(cfg, mode);
	ret = uwb_store_save_mode();
	if (ret) {
		shell_error(sh,
			    "error: mode=%s applied in RAM but NOT persisted "
			    "(errno %d) — will be lost on reboot",
			    uwb_config_mode_name(cfg->mode), ret);
		return ret;
	}

	shell_print(sh, "ok: mode=%s (saved) — reboot to apply",
		    uwb_config_mode_name(cfg->mode));
	return 0;
}

static int cmd_cell(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	uint8_t cell_mode;
	int ret;

	ARG_UNUSED(argc);

	if (!uwb_config_cell_mode_from_name(argv[1], &cell_mode)) {
		shell_error(sh, "error: cell must be twr or blink");
		return -EINVAL;
	}

	uwb_config_set_cell_mode(cfg, cell_mode);
	ret = uwb_store_save_cell_mode();
	if (ret) {
		shell_error(sh,
			    "error: cell=%s applied in RAM but NOT persisted "
			    "(errno %d) — will be lost on reboot",
			    uwb_config_cell_mode_name(cfg->cell_mode), ret);
		return ret;
	}

	shell_print(sh, "ok: cell=%s (saved) — reboot to apply, GATEWAY only",
		    uwb_config_cell_mode_name(cfg->cell_mode));
	return 0;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	float x, y, z;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_flt(argv[1], &x) || !parse_flt(argv[2], &y) || !parse_flt(argv[3], &z)) {
		shell_error(sh, "error: pos requires three numbers (x y z)");
		return -EINVAL;
	}

	uwb_config_set_pos(cfg, x, y, z);
	ret = uwb_store_save_pos();
	if (ret) {
		shell_error(sh,
			    "error: pos=(%.2f, %.2f, %.2f) applied in RAM but NOT "
			    "persisted (errno %d) — will be lost on reboot",
			    (double)cfg->x, (double)cfg->y, (double)cfg->z, ret);
		return ret;
	}

	shell_print(sh, "ok: pos=(%.2f, %.2f, %.2f) (saved) — reboot to apply",
		    (double)cfg->x, (double)cfg->y, (double)cfg->z);
	return 0;
}

static int cmd_ant(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	unsigned long tx, rx;
	int ret;

	ARG_UNUSED(argc);

	/* Say WHICH of the two things is wrong. Both arguments are bare
	 * positional numbers, and the natural mistake is to type the field
	 * names -- `anchor ant tx 16356` -- which used to be answered with a
	 * range complaint about a value the operator never supplied, sending
	 * them to check arithmetic that was already correct. Note both delays
	 * must be given every time, even when only one is being changed. */
	if (!parse_ul(argv[1], &tx) || !parse_ul(argv[2], &rx)) {
		shell_error(sh,
			    "error: expected two numbers, got \"%s\" and \"%s\"."
			    " Usage: anchor ant <tx> <rx> — both values, no "
			    "field names (e.g. anchor ant 16356 16385)",
			    argv[1], argv[2]);
		return -EINVAL;
	}

	if (!uwb_config_set_ant(cfg, (uint32_t)tx, (uint32_t)rx)) {
		shell_error(sh, "error: antenna delays must be 0..65535 "
				"(got tx=%lu rx=%lu)", tx, rx);
		return -EINVAL;
	}

	ret = uwb_store_save_ant();
	if (ret) {
		shell_error(sh,
			    "error: ant_tx=%u ant_rx=%u applied in RAM but NOT "
			    "persisted (errno %d) — will be lost on reboot",
			    cfg->ant_delay_tx, cfg->ant_delay_rx, ret);
		return ret;
	}

	shell_print(sh, "ok: ant_tx=%u ant_rx=%u (saved) — reboot to apply",
		    cfg->ant_delay_tx, cfg->ant_delay_rx);
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	int ret, r;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uwb_config_set_defaults(cfg);

	/* All five saves are always attempted, even if an earlier one fails;
	 * report the first failure. */
	ret = uwb_store_save_mode();
	r = uwb_store_save_id();
	ret = ret ? ret : r;
	r = uwb_store_save_cell_mode();
	ret = ret ? ret : r;
	r = uwb_store_save_ant();
	ret = ret ? ret : r;
	r = uwb_store_save_pos();
	ret = ret ? ret : r;

	if (ret) {
		shell_error(sh,
			    "error: defaults applied in RAM but NOT fully persisted "
			    "(errno %d) — some fields will revert on reboot", ret);
		print_config(sh);
		return ret;
	}

	shell_print(sh, "ok: defaults restored (saved) — reboot to apply");
	print_config(sh);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_anchor,
	SHELL_CMD_ARG(show,  NULL, "Print the active configuration as JSON",
		      cmd_show,  1, 0),
	SHELL_CMD_ARG(id,    NULL, "id <0..3> — set the ranging id",
		      cmd_id,    2, 0),
	SHELL_CMD_ARG(mode,  NULL, "mode <slave|gateway> — set the boot mode",
		      cmd_mode,  2, 0),
	SHELL_CMD_ARG(cell,  NULL, "cell <twr|blink> — set the GATEWAY's CFP mode",
		      cmd_cell,  2, 0),
	SHELL_CMD_ARG(pos,   NULL, "pos <x> <y> <z> — set the coordinates in metres",
		      cmd_pos,   4, 0),
	SHELL_CMD_ARG(ant,   NULL, "ant <tx> <rx> — set the antenna delays",
		      cmd_ant,   3, 0),
	SHELL_CMD_ARG(reset, NULL, "Restore the defaults and persist them",
		      cmd_reset, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(anchor, &sub_anchor, "Anchor identity and configuration", NULL);

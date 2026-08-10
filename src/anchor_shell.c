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

static void print_config(const struct shell *sh)
{
	const uwb_config_t *cfg = uwb_config_get();

	shell_print(sh,
		    "{\"mode\":\"%s\",\"id\":%u,\"ant_tx\":%u,\"ant_rx\":%u,"
		    "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		    uwb_config_mode_name(cfg->mode), cfg->anchor_id,
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

	ARG_UNUSED(argc);

	v = strtoul(argv[1], NULL, 0);
	if (v > UINT8_MAX || !uwb_config_set_id(cfg, (uint8_t)v)) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}

	uwb_store_save_id();
	shell_print(sh, "ok: anchor_id=%u (saved) — reboot to apply", cfg->anchor_id);
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	uint8_t mode;

	ARG_UNUSED(argc);

	if (!uwb_config_mode_from_name(argv[1], &mode)) {
		shell_error(sh, "error: mode must be slave or gateway");
		return -EINVAL;
	}

	uwb_config_set_mode(cfg, mode);
	uwb_store_save_mode();
	shell_print(sh, "ok: mode=%s (saved) — reboot to apply",
		    uwb_config_mode_name(cfg->mode));
	return 0;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);

	uwb_config_set_pos(cfg, strtof(argv[1], NULL), strtof(argv[2], NULL),
			   strtof(argv[3], NULL));
	uwb_store_save_pos();
	shell_print(sh, "ok: pos=(%.2f, %.2f, %.2f) (saved) — reboot to apply",
		    (double)cfg->x, (double)cfg->y, (double)cfg->z);
	return 0;
}

static int cmd_ant(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);

	if (!uwb_config_set_ant(cfg, (uint32_t)strtoul(argv[1], NULL, 0),
				(uint32_t)strtoul(argv[2], NULL, 0))) {
		shell_error(sh, "error: antenna delays must be 0..65535");
		return -EINVAL;
	}

	uwb_store_save_ant();
	shell_print(sh, "ok: ant_tx=%u ant_rx=%u (saved) — reboot to apply",
		    cfg->ant_delay_tx, cfg->ant_delay_rx);
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uwb_config_set_defaults(cfg);
	uwb_store_save_mode();
	uwb_store_save_id();
	uwb_store_save_ant();
	uwb_store_save_pos();

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
	SHELL_CMD_ARG(pos,   NULL, "pos <x> <y> <z> — set the coordinates in metres",
		      cmd_pos,   4, 0),
	SHELL_CMD_ARG(ant,   NULL, "ant <tx> <rx> — set the antenna delays",
		      cmd_ant,   3, 0),
	SHELL_CMD_ARG(reset, NULL, "Restore the defaults and persist them",
		      cmd_reset, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(anchor, &sub_anchor, "Anchor identity and configuration", NULL);

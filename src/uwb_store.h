/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Persistence for the anchor config, via Zephyr settings over NVS on the
 * board's storage_partition.
 */

#ifndef UWB_STORE_H
#define UWB_STORE_H

/* Initialise the settings subsystem and load every stored field into the
 * active config. Returns 0 on success, or a negative errno — on failure the
 * active config keeps its defaults and the caller should carry on booting. */
int uwb_store_init(void);

/* Persist one field of the active config. Each logs its own failure and
 * returns 0 on success or the negative errno from settings_save_one() — the
 * caller (the shell) uses this to tell the operator that a value took effect
 * in RAM but was NOT persisted, so it will be lost on reboot.
 *
 * uwb_store_save_ant() writes two keys (tx, then rx). Both writes are always
 * attempted; if either fails, the first failure's errno is returned. */
int uwb_store_save_mode(void);
int uwb_store_save_id(void);
int uwb_store_save_cell_mode(void);
int uwb_store_save_ant(void);
int uwb_store_save_pos(void);

#endif /* UWB_STORE_H */

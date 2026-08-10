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

/* Persist one field of the active config. Each logs its own failure; there is
 * nothing useful for a caller to do about a failed NVS write. */
void uwb_store_save_mode(void);
void uwb_store_save_id(void);
void uwb_store_save_ant(void);
void uwb_store_save_pos(void);

#endif /* UWB_STORE_H */

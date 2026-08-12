/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Network config persisted in NVS under the "net/" settings subtree, one key
 * per field, mirroring uwb_store.c's per-field style for the same reason: a
 * struct blob would force a version byte and a wipe on every layout change.
 *
 * There is no init function. The handler is loaded by the settings_load()
 * that uwb_store_init() already performs -- but net_config_init() must have
 * run before that, or the handler writes into an uninitialised singleton.
 */

#ifndef NET_STORE_H
#define NET_STORE_H

/* Persist one field of the active network config. Each logs its own failure
 * and returns 0 or a negative errno, so the shell can tell the operator that
 * a value took effect in RAM but will not survive a reboot. */
int net_store_save_ssid(void);
int net_store_save_psk(void);
int net_store_save_broker(void);
int net_store_save_user(void);
int net_store_save_mqtt_pass(void);

/* Persist every field. Used by `net reset`; returns the first failure's
 * errno, having still attempted all of the writes. */
int net_store_save_all(void);

#endif /* NET_STORE_H */

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The two anchor mode entry points. Each takes over the main thread and does
 * not return. Both are stubs until specs C (SLAVE) and D (GATEWAY) land.
 */

#ifndef UWB_MODES_H
#define UWB_MODES_H

#include "uwb_config.h"

#include <stdint.h>

void uwb_slave_run(const uwb_config_t *cfg);
void uwb_gateway_run(const uwb_config_t *cfg);

/* Request that the running GATEWAY loop fill `n` synthetic seats with
 * fabricated EUIs (`gw seed <n>`, src/gw_shell.c) -- a load-testing aid, not
 * a real deployment state. Runs on the SHELL thread and only ever touches a
 * single pending-request word; the seat table itself belongs to the
 * gateway's own K_PRIO_COOP(0) loop (src/uwb_gateway.c) and is never reached
 * from here. Returns 0 if the request is accepted (the loop performs the
 * fill on its own thread, possibly several superframes later), -EINVAL if n
 * is not in 1 .. GW_MAX_SEATS (gw_seed_valid_count(), src/gw_core.h), or
 * -EBUSY if an earlier request has not yet been picked up by the loop. */
int uwb_gateway_request_seed(uint32_t n);

#endif /* UWB_MODES_H */

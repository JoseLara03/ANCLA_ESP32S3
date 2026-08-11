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

void uwb_slave_run(const uwb_config_t *cfg);
void uwb_gateway_run(const uwb_config_t *cfg);

#endif /* UWB_MODES_H */

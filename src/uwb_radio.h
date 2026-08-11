/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW3220 bring-up, shared by every anchor mode.
 */

#ifndef UWB_RADIO_H
#define UWB_RADIO_H

#include "uwb_config.h"

/* Reset, probe, initialise and configure the DW3220 with the fixed PHY and
 * the antenna delays from cfg, then arm the IRQ line.
 *
 * Returns 0 on success, or a negative errno. On failure the transceiver is
 * left in whatever state it reached; the caller must not attempt any UWB
 * traffic, but the rest of the system (notably the shell) stays alive.
 */
int uwb_radio_init(const uwb_config_t *cfg);

#endif /* UWB_RADIO_H */

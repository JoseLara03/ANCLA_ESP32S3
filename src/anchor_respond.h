/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The two ranging responders. Each is offered every received frame and ignores
 * what is not addressed to it.
 *
 * IMPORTANT: len is the PAYLOAD length. The radio reports payload + FCS, and
 * the caller must subtract FCS_LEN before calling either function -- the frame
 * module derives field counts from the length and an extra 2 bytes silently
 * corrupts the result.
 */

#ifndef ANCHOR_RESPOND_H
#define ANCHOR_RESPOND_H

#include <stdint.h>

#include "uwb_config.h"

/* If buf is a legacy WAVE/0xE0 poll addressed to this anchor, schedule the
 * delayed VEWA/0xE1 response carrying our id, both timestamps and (x, y).
 * Otherwise no-op. */
void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq);

/* If buf is a DISCOVERY/0xE2 broadcast, schedule an id-staggered
 * RANGE-RESPONSE/0xE4 carrying our short address and CIR metrics. Otherwise
 * no-op.
 *
 * cir_power / cir_quality are captured by the caller from the RX callback --
 * see uwb_slave.c. They are 0 when CIA had not finished for that frame. */
void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality);

#endif /* ANCHOR_RESPOND_H */

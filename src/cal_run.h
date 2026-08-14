/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAL mode: a WAVE responder that can be told, from the console, to become a
 * temporary SS-TWR initiator for one batch of exchanges.
 *
 * Only built when CONFIG_ANCLA_CAL_MODE is set. It exists as a separate image
 * rather than a third runtime mode because a deployed anchor must never
 * transmit an unsolicited poll.
 */

#ifndef CAL_RUN_H
#define CAL_RUN_H

#include "uwb_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Peer id meaning "the external reference node". No ANCLA anchor can match it:
 * wire ids are UWB_ANCHOR_ADDR_BASE + anchor_id, i.e. 0x01..0x04. The stock
 * Qorvo responder on the DWM3001CDK ignores the id byte entirely, so it answers
 * a poll addressed this way while every ANCLA on air stays silent. */
#define CAL_PEER_REFERENCE 0xFFu

struct cal_request {
	uint8_t peer_wire_id; /* CAL_PEER_REFERENCE, or 0x01 + console id */
	int32_t ref_mm;       /* true antenna-to-antenna distance */
	bool    persist;      /* true: solve, apply hot and save. false: report only */
};

struct cal_result {
	uint32_t attempted;   /* exchanges started */
	uint32_t valid;       /* exchanges that produced a distance */
	size_t   kept;        /* samples surviving outlier rejection */
	int32_t  mean_mm;
	int32_t  ref_mm;
	int32_t  error_mm;    /* mean_mm - ref_mm */
	uint16_t old_tx;      /* only meaningful when persist was true */
	uint16_t new_tx;
	int      status;      /* 0, or a negative errno */
};

/* Run one batch. Called from the shell thread; blocks until cal_run()'s loop
 * has finished the batch, then fills *out. Returns out->status, or -EBUSY if a
 * batch is already running, or -ETIMEDOUT if the loop never answered. */
int cal_run_execute(const struct cal_request *req, struct cal_result *out);

/* Takes over the main thread and does not return. */
void cal_run(const uwb_config_t *cfg);

#endif /* CAL_RUN_H */

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
	int32_t ref_mm;       /* true antenna-to-antenna distance; ignored when link */
	bool    persist;      /* true: solve, apply hot and save. false: report only */
	/* LINK mode: measure the RADIO, not the calibration. No reference
	 * distance is needed or used, the -ENODATA floor does not apply (a low
	 * success rate IS the measurement at range), nothing is ever solved or
	 * persisted, and the extra statistics below are filled in. See
	 * docs/range-test.md. */
	bool    link;
	uint32_t attempts;    /* link mode only; 0 means CAL_MAX_SAMPLES */
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

	/* ---- link mode only (req->link); zero otherwise ------------------
	 *
	 * `sd_mm` is the one number `cal peer` never had, and the one that
	 * degrades SMOOTHLY: a success rate is a cliff, so a link already
	 * failing is visible in the spread before it is visible in the count.
	 *
	 * The RX level figures are in dBm x10 (-812 is -81.2 dBm) and are the
	 * measurement that can settle whether this board's post-rework TX
	 * power matches the prediction the whole range budget rests on --
	 * something nothing has ever checked (CLAUDE.md: a link budget taken
	 * before the QM14070 rework is not valid). `rx_level_n` counts the
	 * exchanges that produced one at all; the CIA can legitimately not
	 * have finished on this polled path, which is UNKNOWN, not weak.
	 *
	 * The mean is taken in the LOG domain, which is not the mean received
	 * power. At a fixed distance the spread is small and the difference is
	 * negligible; min and max are reported alongside precisely so a wide
	 * spread is visible rather than hidden inside an average. */
	int32_t  sd_mm;
	int32_t  min_mm;
	int32_t  max_mm;
	int32_t  rx_level_mean_x10;
	int32_t  rx_level_min_x10;
	int32_t  rx_level_max_x10;
	uint32_t rx_level_n;

	/* Why the exchange failed, from ss_initiator's own breakdown. At
	 * range `rx_timeout_or_err` should dominate -- the response never
	 * arrived. A batch dominated by `header_mismatch` or `layout_unknown`
	 * instead is interference or a peer on a different build, which needs
	 * the opposite response and otherwise looks identical from the count. */
	uint32_t f_tx_start;
	uint32_t f_tx_done;
	uint32_t f_rx_to_err;
	uint32_t f_len;
	uint32_t f_hdr;
	uint32_t f_layout;
};

/* Run one batch. Called from the shell thread; blocks until cal_run()'s loop
 * has finished the batch, then fills *out. Returns out->status, or -EBUSY if a
 * batch is already running, or -ETIMEDOUT if the loop never answered. */
int cal_run_execute(const struct cal_request *req, struct cal_result *out);

/* Takes over the main thread and does not return. */
void cal_run(const uwb_config_t *cfg);

#endif /* CAL_RUN_H */

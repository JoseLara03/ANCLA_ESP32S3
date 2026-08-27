/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ccp_slave.h"

#include "ccp_frame.h"
#include "uwb_debug.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ccp_slave, ANCLA_LOG_LEVEL);

/* static, not automatic. 72 bytes would fit anywhere, but CLAUDE.md records a
 * 2588-byte automatic silently overflowing the 4096-byte main stack, and the
 * rule that came out of it does not take exceptions for size. */
static struct sync_model model;

static uint32_t cur_root;
static uint8_t  last_seq;
static bool     have_seq;
static uint32_t n_rx;
static uint32_t n_gap;
static uint32_t n_reject;

void ccp_slave_init(void)
{
	sync_model_init(&model);
	cur_root = 0u;
	last_seq = 0u;
	have_seq = false;
	n_rx = 0u;
	n_gap = 0u;
	n_reject = 0u;
}

bool ccp_slave_on_rx(const uint8_t *buf, uint16_t plen, uint64_t rx_ts)
{
	struct ccp_frame f;

	if (!ccp_frame_is_ccp(buf, plen)) {
		return false;
	}

	/* ccp_frame_parse() also rejects a hop this node must not adopt, so the
	 * CCP_HOP_MAX check cannot be forgotten here. */
	if (ccp_frame_parse(buf, plen, &f) != 0) {
		n_reject++;
		return true;
	}

	if (!have_seq || f.root_id != cur_root) {
		/* A different root is a different time base, and the baseline
		 * built against the old one describes nothing. Start over
		 * rather than mixing two clocks into one rate estimate. */
		if (have_seq) {
			LOG_WRN("{\"ccp_slave\":{\"root_changed\":%u,"
				"\"was\":%u}}", f.root_id, cur_root);
		}
		sync_model_init(&model);
		cur_root = f.root_id;
		have_seq = true;
		last_seq = f.seq;
		sync_model_observe(&model, f.tx_dtu, rx_ts & SYNC_DTU_MASK);
		n_rx++;
		return true;
	}

	uint8_t gap = (uint8_t)(f.seq - last_seq); /* wraps at 256, as ccp_seq does */

	last_seq = f.seq;

	if (gap == 0u) {
		/* Same sequence number twice. Not a miss and not an
		 * observation: folding a duplicate in would count one interval
		 * as two and bias the rate. */
		n_reject++;
		return true;
	}

	if (gap > (uint8_t)(SYNC_MISS_MAX + 1u)) {
		/* Already coasted past the model's own limit, so its estimate
		 * is invalid whatever we do. A fresh baseline beats feeding it
		 * hundreds of misses one call at a time on the SLAVE loop. */
		n_gap += (uint32_t)gap - 1u;
		sync_model_init(&model);
	} else {
		for (uint8_t k = 1u; k < gap; k++) {
			sync_model_miss(&model);
			n_gap++;
		}
	}

	sync_model_observe(&model, f.tx_dtu, rx_ts & SYNC_DTU_MASK);
	n_rx++;
	return true;
}

struct sync_model *ccp_slave_model(void)
{
	return &model;
}

void ccp_slave_stats(uint32_t *rx, uint32_t *gap, uint32_t *reject,
		     uint32_t *root)
{
	if (rx) {
		*rx = n_rx;
	}
	if (gap) {
		*gap = n_gap;
	}
	if (reject) {
		*reject = n_reject;
	}
	if (root) {
		*root = cur_root;
	}
}

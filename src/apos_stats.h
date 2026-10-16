/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Batch statistics for one commanded survey pair: the median-absolute-deviation
 * filter and population sd that apos_node.c used to keep private. Extracted so
 * the gateway ranging an anchor ITSELF (gateway -> anchor, no RANGE_CMD frame)
 * reduces its exchanges with exactly the same rule an anchor-initiated batch
 * uses; two different filters would make a gateway edge and an anchor edge of
 * the same quality carry different weights in the fit.
 *
 * Pure C, host-tested in tests/apos_stats/.
 */

#ifndef APOS_STATS_H
#define APOS_STATS_H

#include <stdint.h>

/* Most samples one batch may hold. apos_node.h's APOS_MAX_EXCHANGES is this
 * value; the scratch copy inside apos_batch_stats() is sized from it. */
#define APOS_STATS_MAX 64u

/* Samples further than this from the batch median are discarded as a
 * reflection or a half-decoded frame; honest ranging noise here is tens of mm. */
#define APOS_STATS_REJECT_MM 300

/* Mean and population standard deviation (mm) of the samples within
 * APOS_STATS_REJECT_MM of the median. Returns the count kept. n > APOS_STATS_MAX
 * is clamped. With nothing kept, *mean_out is the median and *sd_out is 0.
 * Divides by `kept`, not kept - 1: this feeds a weight, not a hypothesis test. */
uint8_t apos_batch_stats(const int32_t *s, uint8_t n, int32_t *mean_out,
			 uint16_t *sd_out);

#endif /* APOS_STATS_H */

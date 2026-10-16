/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_stats.h. Moved unchanged from apos_node.c's batch_stats().
 */

#include "apos_stats.h"

#include <string.h>

uint8_t apos_batch_stats(const int32_t *s, uint8_t n, int32_t *mean_out,
			 uint16_t *sd_out)
{
	if (n > APOS_STATS_MAX) {
		n = (uint8_t)APOS_STATS_MAX;
	}
	if (n == 0) {
		*mean_out = 0;
		*sd_out = 0;
		return 0;
	}

	/* Median via insertion sort into a scratch copy: n <= 64, so O(n^2) is
	 * irrelevant and a real sort would be more code. Static, not on the
	 * stack: this runs on the SLAVE loop (4096 B main stack) and on the
	 * gateway's. Neither is reentrant with itself. */
	static int32_t tmp[APOS_STATS_MAX];

	memcpy(tmp, s, (size_t)n * sizeof(tmp[0]));
	for (uint8_t i = 1; i < n; i++) {
		int32_t v = tmp[i];
		/* int16_t, not int8_t: removes a silent dependency on the cap. */
		int16_t j = (int16_t)(i - 1);

		while (j >= 0 && tmp[j] > v) {
			tmp[j + 1] = tmp[j];
			j--;
		}
		tmp[j + 1] = v;
	}

	int32_t median = tmp[n / 2];
	int64_t sum = 0;
	uint8_t kept = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= APOS_STATS_REJECT_MM) {
			sum += s[i];
			kept++;
		}
	}
	if (kept == 0) {
		*mean_out = median;
		*sd_out = 0;
		return 0;
	}

	int32_t mean = (int32_t)(sum / kept);
	int64_t var = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= APOS_STATS_REJECT_MM) {
			int64_t d = (int64_t)s[i] - mean;

			var += d * d;
		}
	}

	uint32_t sd = 0;

	if (kept > 1) {
		int64_t v = var / kept;

		/* Integer sqrt: no float needed. */
		while ((int64_t)(sd + 1) * (sd + 1) <= v) {
			sd++;
		}
	}

	*mean_out = mean;
	*sd_out = (sd > UINT16_MAX) ? UINT16_MAX : (uint16_t)sd;
	return kept;
}

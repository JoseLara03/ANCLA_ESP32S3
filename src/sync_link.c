/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See sync_link.h.
 */

#include "sync_link.h"

#include <errno.h>
#include <math.h>
#include <string.h>

int sync_link_derive(const struct apos_table *t, const struct apos_result *r,
		     uint8_t root_idx, uint8_t min_n_ok, struct sync_link *out,
		     uint8_t cap)
{
	if (!t || !r || !out || root_idx >= t->n_peers ||
	    r->n_nodes != t->n_peers ||
	    r->node[root_idx].state == APOS_NODE_UNPLACED) {
		return -EINVAL;
	}

	uint8_t w = 0;

	for (uint8_t k = 0; k < t->n_peers && w < cap; k++) {
		if (k == root_idx) {
			continue;
		}

		struct sync_link *l = &out[w++];

		memset(l, 0, sizeof(*l));
		memcpy(l->child_eui, t->peer[k].eui, APOS_EUI_LEN);
		memcpy(l->parent_eui, t->peer[root_idx].eui, APOS_EUI_LEN);
		l->child_addr = t->peer[k].short_addr;

		if (r->node[k].state == APOS_NODE_UNPLACED) {
			continue; /* valid stays false: nothing to compute from */
		}

		float dx = r->node[k].x - r->node[root_idx].x;
		float dy = r->node[k].y - r->node[root_idx].y;
		float dz = r->node[k].z - r->node[root_idx].z;
		float d = sqrtf(dx * dx + dy * dy + dz * dz);

		if (!isfinite(d)) {
			continue;
		}

		l->geom_mm = (int32_t)lroundf(d * 1000.0f);
		l->tau_dtu = sync_link_mm_to_dtu(l->geom_mm);
		l->source = SYNC_LINK_SRC_SURVEY_STATIC;
		l->valid = true;

		int32_t mean = 0;
		uint16_t sd = 0;

		if (apos_table_get_meas(t, root_idx, k, min_n_ok, &mean, &sd)) {
			l->checked = true;
			l->meas_mm = mean;
			l->sd_mm = sd;
			l->err_mm = mean - l->geom_mm;

			int32_t a = l->err_mm < 0 ? -l->err_mm : l->err_mm;

			l->within_gate = a <= SYNC_LINK_GATE_MM;
		}
	}
	return w;
}

uint8_t sync_link_count_good(const struct sync_link *l, uint8_t n)
{
	uint8_t good = 0;

	if (!l) {
		return 0;
	}
	for (uint8_t k = 0; k < n; k++) {
		if (l[k].valid && l[k].checked && l[k].within_gate) {
			good++;
		}
	}
	return good;
}

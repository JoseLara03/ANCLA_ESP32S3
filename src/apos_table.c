/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_table.h.
 */

#include "apos_table.h"

#include <errno.h>
#include <math.h>
#include <string.h>

void apos_table_init(struct apos_table *t)
{
	if (!t) {
		return;
	}
	memset(t, 0, sizeof(*t));
}

int apos_table_find_eui(const struct apos_table *t,
			const uint8_t eui[APOS_EUI_LEN])
{
	if (!t || !eui) {
		return -EINVAL;
	}
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (memcmp(t->peer[k].eui, eui, APOS_EUI_LEN) == 0) {
			return k;
		}
	}
	return -ENOENT;
}

int apos_table_find_addr(const struct apos_table *t, uint16_t short_addr)
{
	if (!t) {
		return -EINVAL;
	}
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (t->peer[k].short_addr == short_addr) {
			return k;
		}
	}
	return -ENOENT;
}

int apos_table_add_peer(struct apos_table *t, const uint8_t eui[APOS_EUI_LEN],
			uint16_t short_addr, bool pos_valid,
			float x, float y, float z)
{
	if (!t || !eui) {
		return -EINVAL;
	}

	int existing = apos_table_find_eui(t, eui);

	/* A different board already claims this address. Checked before the
	 * insert so the table is never left holding two rows for one address --
	 * every later step addresses peers by short address. */
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (t->peer[k].short_addr == short_addr &&
		    (existing < 0 || (int)k != existing)) {
			return -EADDRINUSE;
		}
	}

	uint8_t slot;

	if (existing >= 0) {
		slot = (uint8_t)existing;
	} else {
		if (t->n_peers >= APOS_MAX_NODES) {
			return -ENOSPC;
		}
		slot = t->n_peers++;
		memcpy(t->peer[slot].eui, eui, APOS_EUI_LEN);
	}

	t->peer[slot].short_addr = short_addr;
	t->peer[slot].pos_valid = pos_valid;
	t->peer[slot].x = x;
	t->peer[slot].y = y;
	t->peer[slot].z = z;

	return slot;
}

int apos_table_add_meas(struct apos_table *t, uint8_t from, uint8_t to,
			int32_t mean_mm, uint16_t sd_mm, uint8_t n_ok)
{
	if (!t || from == to || from >= t->n_peers || to >= t->n_peers) {
		return -EINVAL;
	}

	for (uint16_t k = 0; k < t->n_meas; k++) {
		if (t->meas[k].from == from && t->meas[k].to == to) {
			t->meas[k].mean_mm = mean_mm;
			t->meas[k].sd_mm = sd_mm;
			t->meas[k].n_ok = n_ok;
			return 0;
		}
	}

	if (t->n_meas >= APOS_MAX_MEAS) {
		return -ENOSPC;
	}

	t->meas[t->n_meas].from = from;
	t->meas[t->n_meas].to = to;
	t->meas[t->n_meas].mean_mm = mean_mm;
	t->meas[t->n_meas].sd_mm = sd_mm;
	t->meas[t->n_meas].n_ok = n_ok;
	t->n_meas++;
	return 0;
}

/* The usable measurement for the ordered pair (a -> b), or NULL. */
static const struct apos_dir_meas *find_meas(const struct apos_table *t,
					     uint8_t a, uint8_t b,
					     uint8_t min_n_ok)
{
	for (uint16_t k = 0; k < t->n_meas; k++) {
		/* mean_mm <= 0 is rejected at the gateway before it ever gets
		 * here (apos_gw.c's on_range_rsp), so this is belt and braces --
		 * but it is cheap, and a non-positive distance reaching
		 * symmetrise() would be squared inside trilaterate3() and
		 * silently produce plausible-looking garbage geometry. Treated
		 * as "no usable measurement", i.e. a reported hole. */
		if (t->meas[k].from == a && t->meas[k].to == b &&
		    t->meas[k].n_ok >= min_n_ok && t->meas[k].mean_mm > 0) {
			return &t->meas[k];
		}
	}
	return NULL;
}

static float sd_m_of(const struct apos_dir_meas *m)
{
	uint16_t sd = (m->sd_mm >= APOS_SD_FLOOR_MM) ? m->sd_mm
						     : (uint16_t)APOS_SD_FLOOR_MM;

	return (float)sd / 1000.0f;
}

uint16_t apos_table_symmetrise(const struct apos_table *t,
			       struct apos_edge *out, uint16_t out_cap,
			       uint8_t min_n_ok)
{
	uint16_t n = 0;

	if (!t || !out) {
		return 0;
	}

	for (uint8_t i = 0; i < t->n_peers && n < out_cap; i++) {
		for (uint8_t j = (uint8_t)(i + 1);
		     j < t->n_peers && n < out_cap; j++) {
			const struct apos_dir_meas *f =
				find_meas(t, i, j, min_n_ok);
			const struct apos_dir_meas *r =
				find_meas(t, j, i, min_n_ok);

			if (!f && !r) {
				continue; /* a hole; the fit works around it */
			}

			out[n].i = i;
			out[n].j = j;

			if (f && r) {
				/* Inverse-variance weighted, which reduces to
				 * the plain mean when the two sds match -- the
				 * usual case, since both directions run the
				 * same number of exchanges. */
				float sf = sd_m_of(f), sr = sd_m_of(r);
				float wf = 1.0f / (sf * sf);
				float wr = 1.0f / (sr * sr);
				float df = (float)f->mean_mm / 1000.0f;
				float dr = (float)r->mean_mm / 1000.0f;

				out[n].d_m = (wf * df + wr * dr) / (wf + wr);
				out[n].sd_m = 1.0f / sqrtf(wf + wr);
			} else {
				const struct apos_dir_meas *m = f ? f : r;

				out[n].d_m = (float)m->mean_mm / 1000.0f;
				out[n].sd_m = sd_m_of(m) *
					      APOS_ONEWAY_SD_INFLATE;
			}
			n++;
		}
	}
	return n;
}

void apos_table_quality(const struct apos_table *t, uint8_t min_n_ok,
			int32_t *max_recip_mm, uint16_t *max_sd_mm)
{
	int32_t worst_recip = -1;
	uint16_t worst_sd = 0;

	if (!t) {
		goto out;
	}

	for (uint8_t i = 0; i < t->n_peers; i++) {
		for (uint8_t j = (uint8_t)(i + 1); j < t->n_peers; j++) {
			const struct apos_dir_meas *f = find_meas(t, i, j,
								  min_n_ok);
			const struct apos_dir_meas *r = find_meas(t, j, i,
								  min_n_ok);

			if (f && f->sd_mm > worst_sd) {
				worst_sd = f->sd_mm;
			}
			if (r && r->sd_mm > worst_sd) {
				worst_sd = r->sd_mm;
			}
			if (!f || !r) {
				/* A one-way edge has no reciprocal to disagree
				 * with. Excluded rather than counted as zero,
				 * which would flatter the reported maximum. */
				continue;
			}

			int32_t d = f->mean_mm - r->mean_mm;

			if (d < 0) {
				d = -d;
			}
			if (d > worst_recip) {
				worst_recip = d;
			}
		}
	}

out:
	if (max_recip_mm) {
		*max_recip_mm = worst_recip;
	}
	if (max_sd_mm) {
		*max_sd_mm = worst_sd;
	}
}

uint16_t apos_table_missing_pairs(const struct apos_table *t, uint8_t min_n_ok)
{
	uint16_t missing = 0;

	if (!t) {
		return 0;
	}
	for (uint8_t i = 0; i < t->n_peers; i++) {
		for (uint8_t j = (uint8_t)(i + 1); j < t->n_peers; j++) {
			if (!find_meas(t, i, j, min_n_ok) &&
			    !find_meas(t, j, i, min_n_ok)) {
				missing++;
			}
		}
	}
	return missing;
}

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See tau_filter.h.
 */

#include "tau_filter.h"

void tau_filter_init(struct tau_filter *f)
{
	f->have_estimate = false;
	f->value_dtu = 0;
	f->jump_positive = false;
	f->jump_run = 0u;
	f->jump_sum_dtu = 0;
	f->n_accepted = 0u;
	f->n_rejected_quality = 0u;
	f->n_jumps = 0u;
}

enum tau_filter_outcome tau_filter_step(struct tau_filter *f,
					int32_t raw_tau_dtu, uint8_t quality,
					int32_t *out_dtu)
{
	if (quality < TAU_FILTER_MIN_QUALITY) {
		/* Not folded into n_accepted, and deliberately does NOT touch
		 * jump_run either way: an unusable sample is not a
		 * "measurement" in FR-8's "sostenido por >= 5 mediciones
		 * consecutivas" sense at all -- it is simply absent, neither
		 * confirming nor breaking a run in progress. */
		f->n_rejected_quality++;
		return TAU_FILTER_REJECTED_QUALITY;
	}

	f->n_accepted++;

	if (!f->have_estimate) {
		/* First-ever accepted sample: seed directly. Waiting for a
		 * full EMA warm-up (or a 5-sample jump run) before applying
		 * anything would leave this anchor with no TWR tau at all for
		 * several round-robin cycles after every boot, for no
		 * benefit -- there is nothing yet to smooth AWAY from. */
		f->value_dtu = raw_tau_dtu;
		f->have_estimate = true;
		f->jump_run = 0u;
		if (out_dtu) {
			*out_dtu = f->value_dtu;
		}
		return TAU_FILTER_SMOOTHED;
	}

	int32_t delta = raw_tau_dtu - f->value_dtu;
	int32_t mag = (delta < 0) ? -delta : delta;

	if (mag > (int32_t)TAU_FILTER_JUMP_THRESHOLD_DTU) {
		bool positive = delta > 0;

		if (f->jump_run > 0u && f->jump_positive == positive) {
			f->jump_run++;
			f->jump_sum_dtu += raw_tau_dtu;
		} else {
			/* No run in progress, or this sample broke it by
			 * landing on the OTHER side of the applied value --
			 * per FR-8's "sostenido" wording a jump run must be
			 * consecutive AND consistent in direction. Either way
			 * this sample is itself the possible START of a new
			 * run, not discarded. */
			f->jump_run = 1u;
			f->jump_positive = positive;
			f->jump_sum_dtu = raw_tau_dtu;
		}

		if (f->jump_run >= TAU_FILTER_JUMP_SUSTAIN_COUNT) {
			/* Controlled transition: snap to the MEAN of the
			 * confirming run rather than just this last sample,
			 * so one further noisy exchange at the moment of
			 * confirmation cannot set where the filter lands. */
			f->value_dtu = (int32_t)(f->jump_sum_dtu /
						 (int64_t)f->jump_run);
			f->jump_run = 0u;
			f->jump_sum_dtu = 0;
			f->n_jumps++;
			if (out_dtu) {
				*out_dtu = f->value_dtu;
			}
			return TAU_FILTER_JUMPED;
		}

		/* Not sustained long enough yet. A single (or short run of)
		 * outlier(s) must not perturb the slow EMA at all -- that is
		 * the whole point of separating jump detection from ordinary
		 * smoothing -- so the applied value is reported UNCHANGED. */
		if (out_dtu) {
			*out_dtu = f->value_dtu;
		}
		return TAU_FILTER_SMOOTHED;
	}

	/* Within tolerance of the currently applied value: breaks any
	 * in-progress jump run (consecutive, per FR-8) and folds into the
	 * slow EMA. */
	f->jump_run = 0u;
	f->value_dtu += (int32_t)(((int64_t)delta * TAU_FILTER_EMA_NUM) /
				  TAU_FILTER_EMA_DEN);
	if (out_dtu) {
		*out_dtu = f->value_dtu;
	}
	return TAU_FILTER_SMOOTHED;
}

/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOT BUILT INTO THE FIRMWARE since 2026-10-12: the automatic TWR round-robin
 * this filter smoothed was retired (the flight-time compensation is now a
 * static per-link value installed by `apos apply`, src/sync_link.c). The
 * source and tests/tau_filter/ are kept, unreferenced by CMakeLists.txt, as the
 * record of the Fase B design; a future dynamic source could revive them.
 *
 * B14 Fase B, Task 4 (docs/superpowers/plans/2026-09-22-b14-flight-compensation.md
 * FR-8): the anchor-side filter that turns a stream of raw, unaveraged,
 * per-superframe TWR tau measurements (src/gw_selftau.c's auto round-robin,
 * piggybacked on CCP v2 -- src/ccp_frame.h) into a value safe to feed
 * sync_model_set_tau(). Three stages, in FR-8's own order:
 *
 *   1. NLOS/quality gate -- reject a measurement whose exchange diagnostics
 *      say the sample is not trustworthy, before it ever touches the
 *      estimate.
 *   2. Slow EMA (alpha ~= 0.05) -- ordinary noise around a stable tau is
 *      smoothed, not chased.
 *   3. Jump detection -- a SUSTAINED (>=5 consecutive accepted samples, all
 *      on the same side) deviation of more than ~2 ns from the currently
 *      applied value is a genuine relocation (the gateway or the anchor
 *      moved), not noise, and is adopted immediately as a "controlled
 *      transition" -- snapped to the mean of the confirming run, rather
 *      than left to crawl there over ~1/alpha (~20) EMA cycles.
 *
 * Deliberately pure C with no Zephyr dependency, same as sync_model.h and
 * every other estimator in this project: the arithmetic is host-tested to
 * exhaustion here, and what src/ccp_slave.c adds on top is only the parts a
 * host test cannot reach -- which CCP v2 frame this is, whether it is
 * addressed to this anchor, and the current sync_tau_mode gate.
 *
 * ---- Why quality gating is deliberately NARROW right now -----------------
 *
 * gw_selftau.c's twr_quality is a documented PLACEHOLDER encoding of the
 * SS-TWR response's RSSI (roughly dBm + 100, clamped), not a calibrated
 * confidence score -- CLAUDE.md already records that whether a DW3000
 * quality figure carries real information at fixed PLEN is UNMEASURED on
 * this hardware (the ipatovAccumCount entry). Inventing a specific RSSI
 * floor here, with no site data behind it, would be exactly the kind of
 * unvalidated threshold this project's own culture repeatedly warns
 * against shipping. So TAU_FILTER_MIN_QUALITY only rejects the one case
 * that is unambiguous by construction: GW_SELFTAU_AUTO_QUALITY_NONE,
 * meaning the exchange succeeded but the CIA diagnostics were not even
 * read. A real RSSI-floor gate is future work, pending a measurement
 * campaign like the one CLAUDE.md's rx-antenna-delay entry describes.
 */

#ifndef TAU_FILTER_H
#define TAU_FILTER_H

#include "sync_model.h"   /* SYNC_DTU_PER_NS */

#include <stdbool.h>
#include <stdint.h>

/* Integer fixed-point EMA gain, alpha = TAU_FILTER_EMA_NUM / TAU_FILTER_EMA_DEN
 * = 0.05, matching FR-8 exactly (not a power-of-two shift like
 * SYNC_PHASE_EMA_SHIFT -- 0.05 is not one, and this project's own sweep of
 * that constant already found shift-based gains within reach of 0.05 buy
 * negligible extra smoothing, see sync_model.h, so there is no reason to
 * approximate here instead of computing it exactly). Truncation toward
 * zero on the integer division makes each step slightly SMALLER in
 * magnitude than a true 5% correction, never biased: at steady state the
 * average correction is zero regardless, so this only costs a few extra
 * samples of convergence time, never a wrong limit. */
#define TAU_FILTER_EMA_NUM   5
#define TAU_FILTER_EMA_DEN   100

/* FR-8 jump threshold, 2 ns, expressed in DTU via the project's own named
 * conversion (sync_model.h's SYNC_DTU_PER_NS) rather than a second
 * independently-derived constant -- same "named derivation, not a second
 * copy" discipline CLAUDE.md documents for MAC_SFD_TIMEOUT_SYM and
 * SS_INITIATOR_MM_PER_DTU_X1000. */
#define TAU_FILTER_JUMP_THRESHOLD_DTU   (2u * SYNC_DTU_PER_NS)

/* Consecutive accepted (post-quality-gate) samples, all deviating from the
 * currently applied value in the SAME direction by more than the threshold
 * above, before the deviation is adopted as a genuine relocation rather
 * than treated as noise. FR-8's own number. */
#define TAU_FILTER_JUMP_SUSTAIN_COUNT   5u

/* See this header's top-of-file comment for why this is deliberately the
 * narrowest defensible gate rather than a tuned RSSI floor. */
#define TAU_FILTER_MIN_QUALITY  1u

struct tau_filter {
	bool     have_estimate;  /* value_dtu holds a real filtered estimate */
	int32_t  value_dtu;      /* current filtered value -- both the EMA's
				   * output and the value a jump snaps to */
	bool     jump_positive;  /* direction of the in-progress jump run */
	uint8_t  jump_run;       /* consecutive same-direction over-threshold
				   * samples seen so far */
	int64_t  jump_sum_dtu;   /* sum of those samples' raw values, for the
				   * mean a completed jump snaps to */
	uint32_t n_accepted;         /* samples that passed the quality gate */
	uint32_t n_rejected_quality; /* samples the quality gate refused */
	uint32_t n_jumps;            /* completed controlled transitions */
};

enum tau_filter_outcome {
	/* The quality gate refused this sample. *out_dtu (if requested) is
	 * NOT written -- the caller must not apply anything from this call. */
	TAU_FILTER_REJECTED_QUALITY,
	/* Accepted. Either folded into the slow EMA (ordinary case) or held
	 * as a not-yet-sustained outlier that left the applied value
	 * unchanged (mid-jump-run, not yet confirmed) -- both report the
	 * CURRENT applied value in *out_dtu, which is what a caller should
	 * keep using either way. */
	TAU_FILTER_SMOOTHED,
	/* Accepted, AND this sample completed a sustained same-direction run
	 * past the jump threshold: the filter re-seeded to the mean of the
	 * confirming run, and *out_dtu carries that NEW value. A caller
	 * should log this as a "controlled transition" event (FR-8). */
	TAU_FILTER_JUMPED,
};

void tau_filter_init(struct tau_filter *f);

/* Feed one raw, address-matched TWR tau measurement (DTU, same sign/units
 * as sync_model_set_tau()) and its exchange quality byte (CCP v2's
 * twr_quality, gw_selftau.c's encoding). This module knows nothing about
 * wire addresses, CCP frames or sync_tau_mode -- see ccp_slave.c for that
 * layer. `out_dtu` may be NULL. */
enum tau_filter_outcome tau_filter_step(struct tau_filter *f,
					int32_t raw_tau_dtu, uint8_t quality,
					int32_t *out_dtu);

#endif /* TAU_FILTER_H */

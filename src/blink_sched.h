/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Slot count and slot assignment for a cell running in BLINK mode (Task 4B).
 * See docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md sections
 * 1.1, 1.2 and 1.4 -- the design is decided there; this is the implementation
 * of it, not a re-derivation.
 *
 * Pure C, no radio dependency, host-tested in tests/blink_sched/.
 */

#ifndef BLINK_SCHED_H
#define BLINK_SCHED_H

#include <stdint.h>
#include <stdbool.h>

#include "mac_budget.h"

/* Guard after each BLINK slot, in UUS. MEASURED -- Task 2 of
 * docs/superpowers/plans/2026-08-30-blink-slotted-mac.md, 2026-09-01, three
 * tags at near-tail seat ids (120/121/122, forced there via
 * CONFIG_ANCLA_DEBUG_FORCE_HIGH_SEAT so the known ms-rounding risk at the top
 * of the seat table -- see gw_core.h's Task 5 note -- was actually exercised,
 * not just the low seat ids a small bench fleet lands on by default).
 * tools/blink_jitter.py against COM7_2026_09_01.12.43.19.533.txt, >=730
 * samples per tag, gave a max spread of 39.2 ns across all three -- the
 * tag's own TX-arm jitter (nRF52833, this port's first-ever
 * DWT_START_TX_DELAYED), four orders of magnitude below the two previously
 * KNOWN terms this guard has to cover:
 *
 *   gateway TX-arm jitter (ESP32-S3, measured on hardware separately): 64 us
 *   tag crystal drift over one 200 ms superframe at 40 ppm:              8 us
 *   tag's own TX-arm jitter (measured above, negligible):            0.04 us
 *                                                                  ---------
 *   sum of known/measured terms:                                    ~72 us
 *
 * BLINK_SLOT_GUARD_UUS = 100 -- rounded up from that ~72 us sum for margin,
 * not the raw figure, and well short of the old 200 us provisional
 * mid-point value (docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md
 * section 1.2), which was picked before any tag-side jitter had been
 * measured at all. Do NOT move this without first updating that section of
 * the design doc: the number here must always be the SAME one the doc says
 * the code carries. */
#define BLINK_SLOT_GUARD_UUS  100u

/* One BLINK's airtime plus its slot guard, the `slot_ns` mac_cell_max_slots()
 * is called with in BLINK mode. Exposed mainly for tests; callers normally
 * want blink_sched_n_slots() instead of re-deriving this. */
uint32_t blink_sched_slot_ns(void);

/* How many BLINK slots fit in `cell`'s usable CFP span. Thin wrapper over
 * mac_cell_max_slots(cell, blink_sched_slot_ns()) -- see design section 1.4.
 * Does not itself impose any relationship to GW_MAX_SEATS; the caller decides
 * what a seat_id >= the returned count means (see blink_sched_seat_admissible
 * below). */
uint16_t blink_sched_n_slots(const struct mac_cell *cell);

/* Which BLINK slot `seat_id` transmits in. The identity: valid, and
 * collision-free by construction, only when the caller has already checked
 * blink_sched_seat_admissible(seat_id, n_slots) -- see design section 1.1 for
 * why the identity is injective over the whole seat space whenever
 * n_slots >= GW_MAX_SEATS, and why the overflow band (n_slots < GW_MAX_SEATS)
 * is handled by refusing admission rather than by remapping the slot. */
uint8_t blink_sched_slot_index(uint8_t seat_id);

/* Overflow-band admission policy (design section 1.1, v1): a seat is
 * admissible in a BLINK cell iff its id falls inside the slot count the cell
 * actually has this build/geometry. `n_slots` is a parameter rather than a
 * constant because it depends on BLINK_SLOT_GUARD_UUS and the caller's
 * mac_cell (synthetic in a host test, the real superframe geometry on
 * target) -- this function must not fix that value itself. */
bool blink_sched_seat_admissible(uint8_t seat_id, uint16_t n_slots);

#endif /* BLINK_SCHED_H */

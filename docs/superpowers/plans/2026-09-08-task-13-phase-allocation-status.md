# Task 13 — Phase allocation by tier: status and gaps

**Date:** 2026-09-08
**Plan:** `docs/superpowers/plans/2026-09-07-network-scaling-anchor.md`, Task 13
**Commits:** `a63b17e` (implementation) + `bff1207` (fix round 1)
**Status:** Landed and pushed to `origin/docs/network-scaling-v3`. **Not yet safe to exercise on real
tag firmware** — see §3.

---

## 1. What this task implements

The gateway's tag-facing seat table (`src/gw_core.{c,h}`) now grants a tag a *set* of phases instead of
exactly one, sized by its requested rate tier:

| Tier | Ceiling | Spread | Rate at 200 ms/superframe |
|---|---|---|---|
| IDLE | 1 phase | — | 0.31 Hz (1 fix / 3.2 s) |
| SLOW | 2 phases | half-cycle apart | 0.625 Hz |
| FAST | 4 phases (was 8, see §2) | quarter-cycle apart | 1.25 Hz |

Phases are spread **evenly** around the 16-superframe cycle (`base, base+C/n, base+2C/n, ...`), not
consecutively — this is what turns "4 phases" into a steady 1.25 Hz instead of four fixes in 0.8 s
followed by 2.4 s of silence.

**Non-preemptive by construction:** the allocator only ever claims a cell that is free or already the
requesting tag's own. No tag's allocation can be reduced to satisfy someone else's request — this is a
structural property (`candidate_ok()` in `src/gw_core.c`), independently verified by the reviewer with a
240,000-operation randomized fuzz test (60 trials × 4,000 join/keepalive/release/tick operations),
finding zero violations.

**A tag holds the same CFP slot number across every phase it owns.** This was a deliberate design
choice (not mandated by the plan) made so a single 16-bit `phase_mask` plus one slot number fully
describes a grant — which is what a later, still-parked task ("GRANT carries the phase mask") needs to
be a clean, additive wire change rather than a redesign.

`gw_core_release()` now frees every cell a tag holds, not just the first one found — a necessary fix
once one tag could occupy multiple cells at once. `gw_core_keepalive()` now returns
`enum gw_keepalive_result` (`UNKNOWN`/`SAME`/`REPHASED`) plus an optional grant, so a phase-set change is
observable without a log line firing on every single KEEPALIVE at 100-tag scale.

34 host tests (`tests/gw_core/`, up 16→34 from Task 12), all passing, clean under `-Wall -Wextra
-Wpedantic -Wshadow -Wconversion` and under the real Xtensa cross-compiler.

---

## 2. What the review found, and what the fix round corrected

The allocator algorithm itself was reviewed and found correct — no Critical findings, and the reviewer
independently re-derived every correctness proof rather than trusting the implementer's report. Two
things needed fixing:

**A factually inverted claim, now corrected (`bff1207`).** The original implementation concluded that
granting a tag multiple phases needs **no wire change** — that a tag simply discovers its own slot by
scanning every beacon, so publishing it in *n* phases is automatically sufficient to make it range *n*
times per cycle. **This was backwards.** The actual tag firmware
(`tag_testting/src/uwb_net.c:282-284`, `:320-322`) treats *any* beacon that doesn't list the tag's
address as an immediately lost seat and drops back to `SCAN` — with no tolerance for that specific case.
Since the gateway publishes only one phase's row per beacon and a FAST-tier tag listens to every beacon,
a tag holding, say, 4 of 16 phases would see itself absent from the other 12 and drop its seat almost
immediately. The two comment blocks that asserted the opposite (`src/gw_core.h`'s file header,
`src/uwb_gateway.c`'s `send_grant()`) have been rewritten to state the correct dependency (see §3).

**`GW_PHASES_MAX_FAST` lowered from 8 to 4.** The reviewer's capacity arithmetic: today's real table is
`GW_CYCLE_C * GW_N_CFP` = 176 cells (not yet 224 — see the "224 vs 176" note in the main plan). Twenty
simultaneous FAST tags at 8 phases each plus 80 stationary tags at 1 phase each already overcommits even
the eventual 224-cell table, and exhaustion doesn't gracefully degrade the mover that caused it — it
refuses an unrelated stationary tag's JOIN outright (zero fixes), which is strictly worse than a smaller
allocation would have been. 4 phases (1.25 Hz) is also the more literal reading of the brief's own "FAST
4 (8 when the budget allows)" — 4 as the default, 8 as an unimplemented stretch goal. The allocator
itself stays generically capable of 8 (and any power-of-two rung up to 16); a test-only seam
(`gw_core_keepalive_max_for_test()`) keeps that code path covered so raising the ceiling later, once real
occupancy data justifies it, doesn't land on untested code.

Both fixes were verified: host suite rebuilt and passing (`tests/gw_core/test_gw_core.exe` → PASSED),
clean under `gcc -Wall -Wextra`.

---

## 3. What is still missing — read this before trusting a multi-phase grant on air

**Multi-phase grants are not yet safe to exercise against real tag firmware.** Two things are genuine
functional prerequisites, both still outstanding:

1. **Task 14 ("GRANT carries the phase mask")** — currently parked, because it requires a coordinated
   change to the shared `uwb_frame_802_15_4z.c` codec that must land in the tag repo first. Without it,
   a tag is never told which phases it owns.
2. **A tag-side firmware change** deriving `listen_skip` and the `in_map` check from that mask, so a tag
   only checks the beacons of the phases it actually owns — instead of reading absence from *any* beacon
   as lease loss. This does not exist in `tag_testting` today and is not part of this plan's ANCLA-side
   work; it belongs to the tag repo's own implementation plan.

Until both land, granting a tag more than one phase will cause it to drop to `SCAN` on real hardware,
not range faster. This is now stated plainly in `src/gw_core.h`'s file comment and
`src/uwb_gateway.c`'s `send_grant()` comment — read those before anyone is tempted to flash this and test
multi-phase ranging early.

**This is not a defect in what shipped.** The gateway-side seat bookkeeping (Tasks 12+13) is correct,
tested, self-contained, and safe to have landed — it simply isn't the *complete* feature by itself. The
plan's own framing already anticipated this: "a phase is only air-testable when both sides have finished
it."

**Six Minor findings from the Task 13 review are deferred, not fixed**, per the SDD process (Minor
findings are logged for the final whole-branch review rather than entering the per-task fix loop):

1. `gw_core_keepalive()`'s fallback path can return `SAME` without filling `out` — unreachable today
   (proven the n=1 rung always succeeds for a seated tag), but a latent trap for a future refactor that
   breaks that proof.
2. No build-time assert ties `GW_PHASES_MAX_FAST` to being a power-of-two divisor of `GW_CYCLE_C` — a
   bad future value would silently degrade rather than fail loudly.
3. `tag_footprint()` is computed twice on the re-JOIN path (~176 redundant cell reads, bounded and cheap,
   but avoidable).
4. `gw_phase_count()`'s comment is slightly imprecise (says "bounded at `GW_CYCLE_C` iterations"; it's
   actually bounded by the 16-bit width, equal to `GW_CYCLE_C` only via the adjacent assert).
5. A few coverage gaps: no test asserts a seated FAST tag *keeps* its 4 phases as the table fills around
   it (implied by an existing test, never asserted directly); no test asserts two FAST tags coexisting at
   full ceiling in different slots; one stability test checks only the base cell's lease refresh where a
   sibling test checks all cells.

**Not yet done at all:**

- **A scoped re-review of the `bff1207` fix round.** The original review's two Important findings were
  fixed and the host suite re-verified, but the SDD process's re-review step (an independent pass
  confirming both findings are genuinely addressed and the fix introduced no new breakage) has not yet
  been dispatched — this was interrupted by a session usage-limit reset. This is the next step before
  Task 13 can be marked fully complete in the plan's ledger.
- **No hardware has run any of this.** Every claim above is host-test-verified and cross-compiler-clean,
  never bench-tested. The plan's own hardware gates (an actual 100+ tag load test, `gw seed 224` staying
  on time for 10 minutes) remain untouched, consistent with the rest of this session's work — see the
  main ledger (`.superpowers/sdd/2026-09-07-network-scaling-anchor/progress.md`) for the full list of
  parked hardware- and cross-repo-gated tasks.

---

## 4. Where this fits

This is one task inside a larger plan (`docs/superpowers/plans/2026-09-07-network-scaling-anchor.md`)
targeting 100 tags over 32 anchors. Of that plan's 21 tasks, this session executed the subset that is
self-contained within `ANCLA_ESP32S3` and needs no tag-repo or hardware changes: Tasks 3, 7, 12, 13 (this
one), with 15/16/17/21 still to go. Eight tasks (4, 5, 6, 8, 9, 10, 14, 20) are parked pending
coordinated work in `tag_testting`; five more (1, 2, 11, 18, 19) are parked pending bench access. The
full ledger has the complete task-by-task record, every ruling made, and every deferred finding.

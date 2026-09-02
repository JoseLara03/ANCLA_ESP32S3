# TDoA Accuracy/Smoothing Filter — Session Summary (2026-09-02)

**Branch:** `feat/tdoa-accuracy-filter` (from `feat/rtls-scale-tdoa`)
**Plan/spec:** `docs/superpowers/plans/2026-09-02-tdoa-accuracy-filter.md`,
`docs/superpowers/specs/2026-09-02-tdoa-accuracy-filter-design.md`

## Objective

Reduce noise and improve temporal coherence in the TDoA position fixes the
gateway publishes, without touching the ~45 cm accuracy target (which
remains bounded by array GDOP and an uncalibrated RX antenna delay — neither
in scope here). Four implementation tasks plus one hardware-verification
task.

## What was implemented (Tasks 1–4)

1. **`SYNC_PHASE_EMA_SHIFT` sweep.** Measured {3,4,5,6} × 7 jitter levels ×
   12 seeds. Best-case improvement was 8.8%, against a 20% bar for changing
   the constant. **Result: negative — left unchanged at 3.** Recorded with
   the real numbers next to the constant so it isn't re-proposed without new
   evidence.
2. **Early group release fixed.** `tdoa_collect_set_expected()` replaces a
   hardcoded 4-anchor threshold with the actual surveyed anchor count. On a
   deployment with fewer than 4 anchors, this was previously costing every
   fix an extra ~200 ms of unnecessary latency.
3. **Per-tag smoothing filter.** A constant-velocity EKF (`pos_ekf`) now
   fuses TDoA range differences directly, rather than filtering the
   already-solved position. On synthetic noisy data it tracked ~2× more
   accurately than the raw per-fix solve.
4. **Filter wired into the gateway**, with `dt` computed from hardware
   timestamps rather than the gateway's own loop clock, the existing
   10 m mirror-branch jump gate preserved, and new counters exposed on
   `blink stats` for bench visibility.

All four were host-test-verified and confirmed building cleanly for both
firmware images before any hardware was involved.

**One design-premise correction:** the original design assumed the tag's
copy of `pos_ekf` was dead code, and instructed moving it into this repo
outright. That assumption was wrong — the tag still uses it on its
TWR-ranging path. Ownership follows this project's existing precedent for
this situation (`pos_solver.c`/`pos_residual.c`: shared copy, tag retains
ownership) instead.

## Hardware verification (Task 5) — three rounds, same live gateway

Task 5 did not run as a clean, single controlled test. It found the
production gateway already live on the bench, flashed the new build to it
directly, and iterated in place across three rounds as issues surfaced.

**Round 1 — first contact.** Boot clean, real 4-anchor survey, WiFi/MQTT
connected, a real tag joined, sync healthy. Checking `blink stats` surfaced
the first defect: **`tdoa_gw_ekf_stats()` had been declared and documented
as wired into the console output, but was never actually implemented or
called** — a real gap between what was reported done and what shipped, that
neither the host test suite nor code review had caught, because nothing
called the function to make its absence visible. Fixed and reflash-verified
in place.

**Round 2 — a second, more serious defect.** With the counters now visible,
one tag's published trace showed the same `(x, y)` to two decimal places
across three timestamps several minutes apart — not something a live filter
re-solving real data can produce. Root cause: once a tag's filter has ever
been seeded, the code path that decides whether to publish did not
distinguish "the filter's state changed this cycle" from "nothing happened,
republish whatever's there." When both the primary filtering path and its
solve-based fallback failed in the same cycle (an invalid time delta plus a
rejected fallback solve), the gateway published the filter's stale,
unchanged prior position as if it were a fresh measurement — indefinitely,
with no signal to a downstream consumer that the data was stale. Confirmed
independently by `blink stats`' own counters: total published fixes
exceeded the sum of its three producing paths by exactly the number of
stale republishes observed in the log. Fixed by tracking whether the filter
state actually advanced each cycle and suppressing publication when it did
not; the suppressed count is now its own counter (`no_update`) for future
visibility.

**Round 3 — post-fix, post-cold-reboot.** Following a `kernel reboot cold`,
the gateway came back up cleanly with two real tags active (one stationary,
one carried). The fix held: published-fix count matched the sum of its
producing paths exactly, with zero stale republishes. The walking tag's
trace was smooth and continuous — consecutive 200 ms fixes moved by no more
than ~15 cm, consistent with a person walking, with no metre-scale jumps.
The stationary tag showed no repeat of the earlier catastrophic failure
(a 3.5 m → 9.1 m lock that had repeated verbatim), but its four samples
over the capture window still spanned roughly 0.8 m — real residual noise,
above the ~45 cm target, and too small a sample to treat as a settled
number.

## Net result

- Two genuine hardware-only defects found and fixed, neither reachable by
  host tests or code review — both directly caused by the act of exercising
  the code on real, uncontrolled multi-tag traffic rather than a single
  synthetic scenario.
- The trace-continuity acceptance criterion (no jump discontinuities on a
  moving tag) is met on real data.
- The stationary-tag dispersion criterion is not yet closed: no pre-session
  baseline was captured, and the post-fix sample size for a motionless tag
  is too small to report a number with confidence.
- Absolute accuracy (~45 cm target) was never in scope for this work and
  remains gated on the uncalibrated RX antenna delay and array GDOP, per
  existing project documentation.

## What remains open

A longer, undisturbed stationary-tag capture to get a real dispersion
figure, and — separately, unrelated to this session's scope — the RX
antenna delay calibration campaign that gates absolute accuracy.

## Commits

- `9243229` — Tasks 1–4 implementation and first hardware smoke test.
- `79adc93` — Fix: stop republishing stale EKF state as a live fix.
- `dcfd577` — Documentation of the first bug and second smoke-test round.
- (this commit) — Documentation of the third round (post-fix, post-reboot)
  and this summary.

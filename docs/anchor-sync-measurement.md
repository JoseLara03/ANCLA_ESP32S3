# Anchor clock sync — the Phase 2 measurement

Date: 2026-08-25

## 0. What this measures, and why it is the gate

The TDoA migration needs every anchor that hears a tag's blink to timestamp it
on a **common clock to sub-nanosecond accuracy**. Nothing else in this firmware
needs that, and the MAC protocol contract §1 assumed it was unachievable
wirelessly at these distances. The whole migration turns on whether that
assumption is wrong.

`src/sync_model.{c,h}` answers the arithmetic half in simulation, and the answer
is yes with margin — 0.125 ns worst case, against a target of 1 ns, with a
deterministic floor of **zero**: the integer arithmetic is exact, so every DTU of
error is noise rather than rounding. Those numbers and the tables below the
header's "What the simulation actually measures" heading are reproducible with
`tests/sync_model/`.

What simulation cannot supply is the one input it is most sensitive to: the
**per-observation timestamp jitter** of a real DW3220 receiving a real frame from
a real neighbour. Sweeping it puts the crossover here:

| jitter per observation | resulting sync error | verdict |
|---|---|---|
| 0.09 ns | 0.02 ns | pass |
| **0.50 ns** | **0.55 ns** | **pass, marginal** |
| **1.00 ns** | **1.23 ns** | **fail** |
| 5.00 ns | 3.63 ns | fail |
| 10.0 ns | 8.52 ns | fail |

**So the gate is one number.** Measure the jitter. Under ~0.5 ns, Phase 3
proceeds. Over ~1 ns it does not, and the remedies — a faster CCP rate, or
averaging harder — both cost airtime that the capacity budget in the design
spec §3.2 would have to be re-run for.

## 0.1 This gate does NOT depend on antenna calibration

Worth stating up front because it decouples two work streams that look
sequential. Antenna delay is a fixed **bias**; this measurement is of **noise**.
A board with a wildly wrong `ant_tx` produces timestamps offset by a constant,
and a constant offset is absorbed entirely by the model's phase term without
affecting the residual spread at all.

So this can be measured on uncalibrated boards, in parallel with the
antenna-delay campaign in `docs/antenna-delay-calibration.md`, and neither
blocks the other.

## 0.2 How the measurement works: the model measures its own input

There is no reference instrument, no known distance, and no external clock in
this procedure. That is deliberate and it falls out of how the model works.

Every CCP is **predicted before it is folded in**, and the residual is
`(master's declared transmit time) − (predicted time)`. The master's value is
exact — it is a number the master put in the frame — so the residual is a direct
measurement of local timestamp noise.

One correction, because it is the kind that misleads in the confident direction:
the residual is **not** equal to the jitter. The prediction consumes *two* noisy
local timestamps — the new observation's, and the phase reference's — so the
residual differences two independent samples and its RMS sits at
`sqrt(2) × jitter` as a floor, measured at about **1.55 ×** once the phase filter
and rate term are included.

An operator reading 64 DTU of RMS and calling it 1 ns of jitter would be looking
at roughly 0.65 ns and rejecting hardware that passes. So:

> **Read `jitter_est`, not `rms`.** `sync_model_jitter_est_dtu()` applies the
> conversion. It is the value the table in §0 is indexed by.

`tests/sync_model/test_residual_rms_measures_the_jitter` pins the relation across
five noise amplitudes, so the factor cannot drift unnoticed.

## 1. Prerequisites

- **Two ANCLA anchors.** Antenna calibration not required (§0.1). Both must be
  reworked boards — a link budget measured before the PA rework is not valid,
  see `CLAUDE.md`.
- **Clear line of sight**, a few metres apart. Distance barely matters for this
  measurement: it is noise, and a constant flight time is a constant. What does
  matter is a strong, clean, single-path signal, because NLOS and multipath are
  precisely what would inflate the jitter.
- **Clear of metal**, and **all tags powered off**, for the same reason as the
  calibration procedure: interference corrupts individual samples.
- The PHY is the frozen one (channel 5, PLEN_1024, PAC32, code 9, 850 kbps,
  SFD 4z, STS off). Both boards run it by construction; nothing to configure.

## 2. What is implemented, and what is not

**Implemented and host-tested:**

| Piece | Where |
|---|---|
| The sync arithmetic, error budget, residual statistics | `src/sync_model.{c,h}`, `tests/sync_model/` |
| The CCP wire format | `src/ccp_frame.{c,h}`, `tests/ccp_frame/` |
| Where the CCP sits in the superframe | `src/ccp_sched.h`, `tests/ccp_sched/` |

**Implemented and building clean; hardware verification PENDING:**

| Piece | Where |
|---|---|
| Master role (gateway, hop 0) | `src/ccp_master.{c,h}` |
| Slave role, gap detection, model ownership | `src/ccp_slave.{c,h}` |
| `sync stats` / `sync reset` | `src/sync_shell.c` |

**No board has run this path yet.** The three pieces above compile and link
into their images, and their host-testable logic is covered by the suites
above them — but nothing in this second table has been observed transmitting,
receiving, or reporting a verdict on real hardware. Every number in §0 and §4
is a simulation result or a host-test result, not a measurement. §3 below is
the procedure that closes that gap, and until it has been run and its output
recorded in §4.1, this document describes a gate that is executable but not
yet passed, failed, or even attempted.

The CCP goes in the **post-beacon guard window**, where slaves already may not
transmit, so it costs **no** airtime from the CAP or the CFP —
`CCP_OFFSET_UUS` is `BEACON_OCCUPANCY_UUS` and two `BUILD_ASSERT`s in
`src/ccp_sched.h` prove the preamble does not overlap the beacon and the frame
is off the air before the guard closes. Its airtime is **1.289 ms, 0.645 % of a
200 ms superframe** — pinned in `tests/ccp_frame/test_airtime_is_recorded` and
again in `tests/ccp_sched/`.

**Role selection is deliberately not runtime-configurable.** The master is the
gateway, because putting an unsolicited transmit path in a production SLAVE
image is the collision hazard `apos_node.c`'s two gates exist to prevent. To
repeat the measurement with the roles swapped (§3), swap the **boards**:
`anchor mode gateway` on the other one, then `kernel reboot cold`.

## 3. Procedure

1. Flash **production** on both boards. One is `anchor mode gateway` (the CCP
   master, hop 0), the other `anchor mode slave` (the receiver).
2. On the gateway, confirm `{"ccp_master":{"root_id":N,...}}` at boot and note
   `root_id`.
3. On the slave, `sync stats`. Confirm `rx` climbing by ~5 per second, `root`
   equal to the gateway's `root_id`, and `valid:1` within a couple of seconds.
   `gaps` and `rejected` should both stay at or near 0.
4. `sync reset`, then leave it alone for **at least 90 seconds** — the verdict
   is withheld below 400 observations on purpose, and at ~5 per second that is
   ~80 s.
5. `sync stats`. **Read `verdict` and `jitter_est_ps`.** Do **not** read
   `rms_dtu` as the jitter — §0.2 explains why that misleads in the confident
   direction: the residual RMS runs about 1.55× the real jitter because the
   prediction consumes two noisy timestamps, so 64 DTU of RMS read as 1 ns is
   actually about 0.65 ns, and an operator making that substitution would
   reject hardware that passes.

Repeat with the boards swapped. A large asymmetry between the two directions
points at one board, not at the technology — the same logic as swapping
`anchor id` to separate a firmware fault from a board fault.

## 4. Reading the result

| `jitter_est` | | Verdict |
|---|---|---|
| < 32 DTU | < 0.5 ns | **Phase 3 proceeds.** |
| 32–64 DTU | 0.5–1.0 ns | **Marginal.** Sub-ns is reachable but with no margin. Re-measure at a different distance and orientation before deciding; if it holds, proceed only with the remedies in §5 costed first. |
| > 64 DTU | > 1 ns | **Phase 3 does not proceed as designed.** See §5. |

Cross-checks worth taking at the same time, because they cost nothing extra:

- **`drift_ppb`** should sit within roughly ±40 ppm (±40000 ppb) — that is the
  two crystals' combined tolerance. A much larger figure means one board is
  running something other than the crystal it should be.
- **`drift_ppb` against `dwt_readclockoffset()`**, which measures the same
  quantity by an entirely independent route (carrier frequency offset, ~14.9 ppb
  per LSB, already used in `src/ss_initiator.c`). They should agree. If they do
  not, one of the two paths is wrong and the sync model is the more likely
  suspect — CFO has been in production for months.
- **`max` versus `rms`.** A max more than about 5× the RMS says the distribution
  has a tail rather than being Gaussian noise, which usually means multipath or
  an intermittent obstruction, not clock noise. Fix the setup and re-measure
  before believing the number.

### 4.1 Recording the result

_Not yet run._ Nothing below has been filled in.

**Direction 1** — master: `___` (board), slave: `___` (board) — date: `___`

```
_not yet run_
```

**Direction 2** — master: `___` (board), slave: `___` (board) — date: `___`

```
_not yet run_
```

Until both blocks above are filled in with real `sync stats` output, the
Phase 2 gate is **unmeasured** and the A7 row in
`docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md` stays `parcial`.

## 5. If the number is too high

In rough order of cost:

1. **Fix the setup first.** NLOS, multipath, metal, and a marginal link all
   inflate timestamp jitter and all look identical to a hardware limit in this
   statistic. A `max`/`rms` ratio above ~5 is the tell (§4).
2. **Shorten the CCP interval.** Halving it halves the extrapolation distance
   and so the rate term. Costs airtime linearly, and §3.2's budget must be
   re-run — at 0.64 % per CCP there is room, but it is not free.
3. **Average the phase harder.** Raising `SYNC_PHASE_EMA_SHIFT` cuts reference
   noise as `sqrt(2^shift)` at the cost of tracking a genuine step more slowly.
   Free in airtime, and the cheapest real remedy.
4. **Accept worse than sub-ns and re-derive the accuracy target.** TDoA position
   error scales with sync error times the speed of light: 1 ns is 30 cm of range
   error, so a 2 ns sync would put the system at the very edge of the 10–30 cm
   requirement rather than inside it. This is a product conversation, not a
   firmware one.
5. **Wired sync.** What Sewio and Ciholas use the cable for. Needs a PCB
   revision and is out of scope for this hardware.

If none of 1–3 gets under ~1 ns, the honest outcome is that Phase 3 does not
proceed and the product stays on TWR with the Phase 1 scheduler — which is 9×
short of the 100-tag target. That is the decision this measurement exists to
inform, and it is better made on a number than on an assumption in either
direction.

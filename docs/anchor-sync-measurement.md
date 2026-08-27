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
transmit, so it costs **no** airtime from the CAP or the CFP. It is sent with
`DWT_START_TX_IMMEDIATE` right after the beacon's own TXFRS confirms, not on a
scheduled offset — an earlier design scheduled it with a DELAYED TX at a fixed
offset (`CCP_OFFSET_UUS`) after the beacon's RMARKER, and it failed 100% on
the bench: the arm sequence could not reliably finish before the CCP's own
PREAMBLE needed to start, a whole SHR (1050194 ns at PLEN_1024) earlier than
the RMARKER the old design scheduled against — see CLAUDE.md's hard-won fact
on this for the full diagnosis. The immediate-TX design removes the deadline
entirely rather than re-tuning it: there is nothing left to arm against, so
there is nothing for arm jitter to be late for. Since a transmitter cannot
know its own TX timestamp before transmitting, each CCP announces the
PREVIOUS CCP's measured timestamp instead of its own — `src/ccp_master.c`
reads the actual TX timestamp back once TXFRS confirms and carries it in the
following frame; `src/ccp_slave.c` pairs a CCP's announcement with the local
RX timestamp of the frame it describes. `src/ccp_sched.h` keeps one
`BUILD_ASSERT` — `CCP_SCHED_MAX_ARM_NS` (348300 ns) is positive, i.e. the
post-beacon guard window is wide enough to host the CCP's whole frame at all
after the beacon's own airtime — and reports the rest (where the CCP actually
lands, and the implied arm cost) as a runtime measurement instead, since
whether any one arm sequence is fast enough is no longer something a header
can know at compile time. Its airtime is still **1.289 ms, 0.645 % of a
200 ms superframe** — pinned in `tests/ccp_frame/test_airtime_is_recorded` and
again in `tests/ccp_sched/` — unchanged by any of this: the frame did not
change size, only when it transmits and what it carries.

**Role selection is deliberately not runtime-configurable.** The master is the
gateway, because putting an unsolicited transmit path in a production SLAVE
image is the collision hazard `apos_node.c`'s two gates exist to prevent. To
repeat the measurement with the roles swapped (§3), swap the **boards**:
`anchor mode gateway` on the other one, then `kernel reboot cold`.

## 3. Procedure

0. **Sniffer check, before touching either console.** Everything below assumes
   the CCP is actually reaching the air, and a sniffer settles that
   independently of anything either board self-reports. Capture a few seconds
   with a DWM3001CDK sniffer and confirm: a 21-byte `0xEF` frame follows
   **each** `0xE5` beacon, `src = 0x0000` (the gateway's reserved address), the
   byte at offset 10 (`ccp_seq`) incrementing with no gaps across consecutive
   CCPs, and the byte at offset 11 (`hop`) equal to `0x00` (`CCP_HOP_ROOT` —
   there is exactly one root in this Phase 2 setup). Note in passing: `gw_seq`,
   the ordinary 802.15.4 sequence field, now advances **twice** per superframe
   on the gateway — once building the beacon, once building the CCP that
   follows it — so an operator checking the BEACON's own sequence continuity
   on the same sniffer trace will correctly see it jump by 2 each superframe,
   not 1. That is expected, not a dropped frame.
1. Flash **production** on both boards. One is `anchor mode gateway` (the CCP
   master, hop 0), the other `anchor mode slave` (the receiver).
2. On the gateway, confirm `{"ccp_master":{"root_id":N,...}}` at boot and note
   `root_id`.
3. **On the gateway console, before reading anything on the slave**: confirm
   there is no `"beacon started but TXFRS never completed"` line anywhere in
   the log, then run `sync master` (or wait for its own rate-limited
   `{"ccp_master":{"sent":...,"dropped":...,"offset_ns_min":...,
   "offset_ns_max":...,"arm_cost_ns_last":...}}` `LOG_WRN` summary — it only
   prints when the drop count has actually moved). Under the immediate-TX
   design there is no scheduled deadline left to miss, so a dropped CCP here
   means something different than it used to: either `dwt_starttx()` itself
   failed (check for a `DW_SYS_STATE_TXERR`-class fault — CLAUDE.md's hard-won
   fact on the arm-deadline trap covers why a completed TX, unlike a completed
   RX, can leave the radio needing an explicit `dwt_forcetrxoff()` before the
   next transmit), or TXFRS never arrived within `CCP_MASTER_TX_TIMEOUT_MS`
   (8 ms) of a successful `dwt_starttx()`. **If `dropped` is climbing —
   worst case, `sent` stays near 0 and the slave never sees a single CCP —
   suspect the radio state left over from the beacon's own TX, or a genuine
   TXFRS timeout, not a scheduling budget: there is no schedule left to be
   too tight.** Also watch for a `{"ccp_master":{"cap_overlap":1,...}}`
   `LOG_WRN`: that means a CCP's frame end reached the earliest legitimate
   slave CAP preamble (`arm_cost_ns_last` approaching or exceeding
   `CCP_SCHED_MAX_ARM_NS`, 348300 ns) — the CCP is still being sent and
   confirmed, but it is colliding with real ranging traffic, which is a
   different problem from a drop and needs the arm sequence itself
   shortened, not a timeout raised.
4. On the slave, `sync stats`. Confirm `rx` climbing by ~5 per second, `root`
   equal to the gateway's `root_id`, and `valid:1` within a couple of seconds.
   `gaps` and `rejected` should both stay at or near 0. Under the
   deferred-timestamp design an observation lags its frame by one superframe,
   so also check `paired` is climbing roughly in step with `rx` (allowing for
   one `no_announce` — the master's first CCP after its own boot never carries
   an announcement) — `rx` climbing while `paired`/`count` stay flat points at
   the pairing logic or a `root`/sequence discontinuity, not at the RF link.
5. `sync reset`, then leave it alone for **at least 90 seconds** — the verdict
   is withheld below 400 observations on purpose, and at ~5 per second that is
   ~80 s.
6. `sync stats`. **Read `verdict` and `jitter_est_ps`.** Do **not** read
   `rms_dtu` as the jitter — §0.2 explains why that misleads in the confident
   direction: the residual RMS runs about 1.55× the real jitter because the
   prediction consumes two noisy timestamps, so 64 DTU of RMS read as 1 ns is
   actually about 0.65 ns, and an operator making that substitution would
   reject hardware that passes.
7. Once the number itself looks right, run the whole procedure **one more
   time** with the debug image (`west build ... -- -DEXTRA_CONF_FILE=debug.conf`,
   which turns on `CONFIG_THREAD_ANALYZER`) and read the gateway's `main`
   thread peak stack usage. Compare it against the 1748/4096-byte figure
   CLAUDE.md already recorded for the anchor survey's `do_solve()` path — this
   loop now does one more immediate TX and one more bounded TXFRS wait per
   superframe than that measurement covered, and nothing has confirmed yet
   that the extra call depth does not move the peak.

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

**Direction 1** — master: gateway, slave: anchor — date: 2026-08-26 — power:
**USB-C on both boards** — `sync reset` run before each reading (see §3 step 5;
skipping it inflates the number, see below).

**30 cm:**

```
jitter_est: 50 DTU = 782 ps        verdict: marginal
count (paired observations): 436
gaps: 0    rejected: 0
max/rms ratio: 3.47 (expected ~3.1 for a clean Gaussian at n=436 — no tail)
```

**3 m:**

```
jitter_est: 92-98 DTU = 1.44-1.53 ns   verdict: fail
counts: 221 and 357 (two runs)
gaps: 0    rejected: 0
drift_ppb: 7.2-7.9 ppm (well inside the ±40 ppm two-crystal budget)
one outlier: max 1968 DTU = 30.8 ns, 15.1x rms, seen once in 895 samples
  and not repeated in the following 357 — a rare real event, not the
  distribution; correctly fired the tail warning in §4.
```

`gaps:0` and `rejected:0` across ~2000 receptions total confirm the link and
the deferred-timestamp pairing both work correctly; the numbers above are a
genuine jitter measurement, not an artifact of dropped or misordered frames.

**`sync reset` is worth 3.7x.** An un-reset reading at 30 cm gave 188 DTU,
because the residual sum still contained observations taken while the rate
estimate was converging (`SYNC_BASELINE_USEFUL` is 10). Skipping §3 step 5
would have an operator conclude the hardware fails by a wide margin when it
does not — this is why that step exists and is stated plainly here rather
than left implicit.

**Decomposition.** 30 cm to 3 m is 10x the distance and 20 dB of path loss; a
purely SNR-limited (link-budget-limited) jitter would rise roughly 10x. It
rose **1.9x**. Fitting `sigma^2 = floor^2 + (k*d)^2` to the two points:

- floor ~= 49 DTU = 772 ps
- link term ~= 8 DTU at 30 cm, ~= 81 DTU at 3 m
- so at 30 cm, **~97% of the variance is floor, not link**

This is a **two-point fit** — two points cannot discriminate between models,
so treat this as a working hypothesis, not a proven decomposition. But the
conclusion it supports is hard to avoid either way: the limit here is not
link budget, and the floor alone (49 DTU) already exceeds the 32 DTU pass
threshold, so even a perfect link at any distance would not have passed as
designed.

**Next lever, not yet run.** The residual measures prediction error over one
CCP interval (200 ms). If the floor is crystal wander over that interval,
lengthening the interval should raise it proportionally and shortening it
should lower it (§5 remedy 2). The experiment is to send the CCP every 2
superframes instead of every 1 and re-measure at 30 cm — deliberately
lengthening, not shortening: shortening would need a second TX per
superframe, and CLAUDE.md's hard-won fact on the battery/CCP finding records
that the supply on this hardware already cannot sustain the one it has. If
the floor does not move with the interval, it is per-observation timestamp
noise and a hardware limit, not wander. **This experiment has not been run.**

Also unconfirmed: whether both boards used for this measurement had the
QM14070 PA rework CLAUDE.md records (the ~25 dB TX deficit fix). An
unreworked board would inflate the link term measured above.

**Product decision.** The 1 ns gate threshold targeted 10-30 cm TDoA accuracy
(1 ns ~= 30 cm of range error). The gate **failed** against that target — do
not read the numbers above as a pass. At the measured ~1.5 ns, range error is
roughly **45 cm**. The target has been consciously re-derived rather than the
hardware found adequate: **~45 cm is accepted for now, and Phase 3 proceeds
at that accuracy**, with the two levers above (CCP-interval experiment; link
term via TX power/antennas/anchor spacing) identified as the path to improve
it later. This is §5 remedy 4 ("Accept worse than sub-ns and re-derive the
accuracy target ... a product conversation, not a firmware one") being
exercised deliberately, not the pass path in the §4 table above.

**Direction 2** — master/slave roles swapped — **not yet run.**

```
_not yet run_
```

A large asymmetry between the two directions would point at one board rather
than the technology (§3, final paragraph); until direction 2 runs, that
cross-check is still outstanding.

The A7 row in `docs/superpowers/specs/2026-08-25-rtls-scale-tdoa-design.md`
is now `sí` — the gate has been measured — but measured is not the same as
passed at the original target; see that spec's "Gate de Fase 2" line for the
outcome recorded there.

## 5. If the number is too high

### 5.0 If there is no number at all

Before any of the remedies below: `rx:0` on the slave, or a `verdict` stuck at
`"no-lock"` no matter how long it runs, is a **different** failure from a high
jitter number, and the remedies below do not apply to it. First, check what
the gateway is powered from — a LiPo cannot sustain the CCP's second PA-driven
TX per superframe and presents exactly as `sent:0` with `dropped` climbing,
indistinguishable at the console from the firmware faults below (see
CLAUDE.md's hard-won fact on the battery/CCP finding). Only once USB-C power
is confirmed should the **transmit** side be checked next, not the link: `sync master` on the gateway (or its
own periodic `LOG_WRN` summary). If `dropped` is climbing while `sent` stays
low or zero, the gateway is not getting CCPs onto the air at all — under the
immediate-TX design (§2, §3 step 3) that means either `dwt_starttx()` itself
is failing (check for the `DW_SYS_STATE_TXERR` radio-state trap CLAUDE.md's
hard-won fact describes, left over from the beacon's own TX) or TXFRS is
timing out — and no amount of moving boards, checking line of sight, or
reading `max`/`rms` on the slave will fix a CCP that never transmitted. If
instead `sent` is climbing normally but the slave's `rx` stays flat, that
points at the RF link as expected; but if `rx` climbs while `paired`/`count`
stay flat, check `no_announce` and a possible `root`/sequence discontinuity
before suspecting the link (§3 step 4) — that is the deferred-timestamp
design's own failure mode, not an RF one either. Only once `sync master`
shows `sent` climbing and `dropped` flat, AND `sync stats` shows `paired`
tracking `rx`, does a stuck `verdict` become a genuine jitter question rather
than a transmit or pairing one.

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

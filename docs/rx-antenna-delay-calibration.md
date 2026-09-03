# RX antenna-delay calibration (the TDoA half)

**Status: procedure written and the fitting tool verified against synthetic
data with a known injected bias. NOT yet run on hardware.**

## 0. Read this first: this is not `cal ref`, and `cal ref` cannot do it

`cal ref` and the `cal peer` cross-check
(`docs/antenna-delay-calibration.md`) calibrate the **sum**
`ant_tx + ant_rx`. That is all SS-TWR can observe, and it is all TWR ranging
needs. Those procedures constrain the TX/RX **split** not at all.

TDoA is the opposite case. Every anchor only ever **receives**, there is no
round trip, and nothing cancels: the observable carries the **difference in
RX-delay error between anchors**, at **4.69 mm per DTU** with no averaging to
remove it. So a fleet that passed `cal ref` and `cal peer` can still be
metres wrong in TDoA — and no TWR measurement of any kind will show it.

Concretely, on this deployment as of 2026-09-03: every board has
`ant_rx = 16385`, the untouched factory default, while the whole `cal ref`
correction went into TX (`ant_tx` around 16356). The 2026-08-28 laser
campaign measured roughly **207 units of per-board spread** in the quantity
TWR *can* see; nothing has ever measured the RX half. The bias attributed to
it — a reported position that moved **3.11 m** when a fourth anchor was added
while the tag never left its spot — is the reason this document exists.

## 1. What the measurement actually is

The DW3220 **subtracts** the configured `ant_rx` from every RX timestamp, so
anchor *k* reports

```
tau_k = T_emit + r_k / c - delta_k + noise
```

`T_emit` — and with it the transmitting tag's own TX delay — is common to
every anchor that hears one blink, so it cancels in a range difference:

```
(tau_k - tau_0) * 4.69mm  -  (r_k - r_0)  =  -(delta_k - delta_0) * 4.69mm
```

The left side is computable from a raw observation capture plus the surveyed
geometry plus a tape-measured tag position. The right side is a
**constant** — it does not depend on where the tag is.

Two consequences, and both matter:

- The fit is **separable per anchor**: a plain mean, no matrix, no solver.
- It is **falsifiable**. Estimate `b_k` independently at several tag
  positions and a genuine RX-delay offset must come out **identical** at
  every one. If the estimates drift with position, the residual is geometry
  or tag height, not RX delay, and applying the fit would add a bias rather
  than remove one. `tools/rx_cal.py` prints that spread and **refuses to
  print an apply command** when it is too large.

Only **differences** are observable. That is not a limitation: a common RX
offset shifts every anchor's timestamp equally and is invisible to TDoA. The
lowest-id anchor is pinned as the gauge (matching `tdoa_collect`'s own choice
of reference) and its delays are left alone.

## 2. Why this needs no new firmware

The raw input is already on the broker. Every anchor publishes one JSON
document per stamped blink on `uwb/anchor/blink/<zone>`:

```json
{"a":1,"t":256,"s":37,"ts":"842135697411","q":103,"b":80,"f":2,"e":33}
```

`a` = anchor id, `t` = tag short address, `s` = blink sequence, `ts` = the
40-bit RX timestamp **already converted into the master's time base** by
`sync_model_to_master()`. That is exactly the observable above.

Applying a correction needs no new firmware either — `anchor ant <tx> <rx>`
already exists and `uwb_radio.c` applies both at boot.

## 3. Prerequisites

- The survey applied and **tape-validated** (`apos show` against a tape on
  all three edges). This procedure cannot separate a survey error from an RX
  delay at a single tag position; it separates them only through the
  consistency check in §1, and only if the geometry it is given is right.
- `cal ref` already done, so the **sum** is calibrated. This procedure moves
  the split and holds the sum, so it neither needs nor disturbs that.
- MQTT reachable, and `mosquitto_sub` on the capture machine.
- A tape measure, and at least **four** marked positions inside the anchor
  hull. Spread them: a fit taken at four nearby spots cannot tell a real
  offset from a local geometry error.
- `apos tagz` is irrelevant to the capture (the gateway's own solve does not
  enter this at all) but the **same number** must be passed to the tool as
  `--tag-z`, and it has to be measured. It is negative when tags sit below
  ceiling anchors.

## 4. Capture

Record the anchors' live delays first — the apply step needs them:

```
anchor show          # on each anchor: note ant_tx and ant_rx
```

Then, one capture file per tag position. Put the tag on its mark, leave it
still, and capture roughly two minutes (about 250 blinks at 2 Hz):

```bash
mosquitto_sub -h <broker> -t "uwb/anchor/blink/#" -v > spot1.txt
```

Repeat for each mark. Nothing else needs to change between spots — do not
reboot anything, do not re-survey, and do not move an anchor. An anchor that
moves mid-campaign invalidates every capture taken before it.

The noise budget: per-timestamp sync jitter is about 33 DTU
(`sync stats`' `jitter_est_dtu`), so one range difference carries about
47 DTU. Averaging 250 blinks brings that to roughly 3 DTU, i.e. **~14 mm** of
resolution on `b_k`. Verified on synthetic data with a known injected bias:
+150 DTU recovered as +147.9, −90 as −92.9.

## 5. Fit

```bash
python tools/rx_cal.py \
  --anchors-from gateway_console.txt \
  --tag-z -1.60 \
  --current 1:16356,16385 --current 2:16360,16385 \
  --spot "0.80,1.00:spot1.txt" \
  --spot "1.60,2.20:spot2.txt" \
  --spot "0.20,2.60:spot3.txt" \
  --spot "1.90,0.60:spot4.txt"
```

`--anchors-from` reads the geometry out of a gateway log's own `apos_peer`
lines, which is preferred over typing coordinates: it cannot disagree with
the survey the gateway was actually solving against — the exact class of
mistake the anchor-auto-positioning branch exists to remove. `--anchor
ID:X,Y,Z` is the manual fallback.

Read the **CONSISTENCY** block, not the per-spot block. A per-spot number
looks convincing on its own and means nothing on its own.

- **`consistent` on every anchor** — the offsets are real. The tool prints
  the `anchor ant` command to run.
- **`INCONSISTENT` on any anchor** — stop. Check, in this order: the
  tape-measured spot coordinates, the survey geometry against a tape, then
  `--tag-z`. A wrong tag height biases every spot by a *different* amount,
  because `sqrt(rho^2 + dz^2)` is nonlinear, so it shows up here as exactly
  this kind of spread. Demonstrated: feeding the tool `--tag-z 0.0` for data
  generated at −1.6 m turns a 5–8 DTU spread into 136–162 DTU.

## 6. Apply — move the split, hold the sum

```
anchor ant <tx - b>  <rx + b>
kernel reboot cold
```

on each non-reference anchor, with the numbers the tool printed.

**Holding `ant_tx + ant_rx` constant is the whole point.** SS-TWR observes
only that sum — CLAUDE.md's antenna-delay derivation shows both TX and RX are
half a tick of range per unit on that path — so TWR ranging, the `apos`
survey and the existing `cal ref` calibration all come out **unchanged**.
Only the TDoA-visible split moves. This is what makes the campaign safe to
run on an already-calibrated fleet.

Note the sensitivity asymmetry, because it is easy to get wrong: one unit of
`ant_rx` is a **full** 4.69 mm in TDoA (a one-way timestamp) but only
**2.35 mm** in SS-TWR (half of each unit lands in the round trip). The tool
works in TDoA units throughout.

The reference anchor is deliberately not touched.

## 7. Acceptance

Not the fit report. **Re-capture and re-fit**: after the reboot, the refitted
`b_k` must collapse toward zero on every anchor. A residual under ~10 DTU
(~47 mm) is at the noise floor of a 250-blink capture and is as good as this
method resolves.

Then, and only then, the thing the campaign is actually for: a stationary tag
at a tape-measured mark, reported by the gateway within the accuracy target,
and the reported position **not** moving when a fourth anchor joins or leaves
the solve. That last check is the one that would have caught the 3.11 m
discrepancy in the first place.

## 8. What this does NOT do

- It does not calibrate the **sum**. `cal ref` still owns that.
- It does not give **absolute** RX delays, only differences. Absolute values
  are unobservable from range differences and unnecessary for TDoA.
- It does not improve **geometry**. On the array measured 2026-09-03,
  amplification (position error per unit of range-difference bias) is 1.20
  median and 1.82 at the 90th percentile — so ~14 mm of residual RX bias is
  ~17–25 mm of position error, and the remaining accuracy budget is spent on
  sync jitter and GDOP, not on this.
- It says nothing about TWR accuracy, which the split does not affect at all.

# Range test: what this hardware can actually reach

**Status: procedure written, derived from the project's own recorded
measurements and constants. NOT yet run. Every number below marked
*predicted* is a prediction and the test is what settles it.**

## 0. Two corrections before anything is measured

### This board has a PA and NO LNA

`uwb_radio.c` sets `DWT_LNA_ENABLE | DWT_PA_ENABLE`, but there is no LNA on
the board — the bit is cosmetic (CLAUDE.md, "Driving the external PA"). The
link is therefore **asymmetric**: transmit is amplified, receive is not.

This matters differently for each path:

| path | TX amplified | RX amplified | used by |
|---|---|---|---|
| tag -> anchor | no (tag has no PA) | no | TDoA blink |
| gateway -> anchor | yes | no | beacon, CCP, `RANGE_CMD` |
| anchor <-> anchor | yes | no | the `apos` survey (SS-TWR) |

Every path has an unamplified receiver. That is the common factor and it is
where the range limit lives.

### UWB transmit is capped by regulation, so range is a RECEIVER problem

UWB is limited to **-41.3 dBm/MHz EIRP**. A PA in a UWB anchor exists to
overcome front-end loss (filter, switch, traces) and *reach* that cap, not to
exceed it. Both ends land near the same EIRP.

The project's own two numbers, normalised to one distance:

```
tag,    MEASURED:   -74.5 dBm @ 3 m   ->   -58.9 dBm @ 0.5 m
anchor, PREDICTED:  -57.0 dBm @ 0.5 m
                                            difference: 1.9 dB
```

One is a measurement and one is a prediction, and **no post-rework anchor
measurement exists** — CLAUDE.md is explicit that a link budget taken before
the QM14070 rework is not valid. So this is a hypothesis, and confirming or
killing it is the first thing the test below does.

If it holds, the consequence is that putting a PA on both ends of the
anchor-to-anchor link buys ~2 dB, not the 12 dB that separates 30 m from
100 m, and no firmware change closes that gap.

## 1. What the budget predicts

Sensitivity floor: **-93 dBm** at 850 kbps / PLEN_1024, the figure
`uwb_radio.c` itself uses (frames landing at -90..-92 dBm cost ~40 % of
DISCOVERY responses).

```
tag -> anchor:     -74.5 dBm @ 3 m,  18.5 dB margin  ->  ~25 m free space
anchor -> anchor:  -57.0 dBm @ 0.5 m, 36 dB margin   ->  ~32 m free space  (predicted)
```

Both land near 30 m, which is what was observed in the field. **30 m is not a
defect; it is what the numbers say.**

For 100 m the received power must be 30.5 dB below the 3 m reference, i.e.
**-105 dBm — 12 dB under the floor**, and that is with zero fade margin.

Where 12 dB could come from, and what each costs:

| lever | gain | cost |
|---|---|---|
| a real LNA | +10..15 dB | hardware |
| higher-gain RX antenna | +3..6 dB | hardware; unrestricted (the EIRP cap is on transmit) |
| 850 kbps -> 110 kbps | ~+5 dB | airtime; PHY contract change in BOTH repos |
| PLEN 1024 -> 4096 | ~+6 dB | +3.15 ms per frame — ruinous for the capacity the TDoA migration exists to buy |
| more TX power | ~0 dB | already at the regulatory cap |

**100 m at 850 kbps / PLEN 1024 / no LNA is not reachable by firmware.**

## 2. The survey has a ceiling of its own, below the link budget

This one comes out of the constants, not the RF. From `apos_node.h`:

```
APOS_RANGE_BATCH_DEADLINE_MS = 700    (sized against beacon_guard's lock budget)
successful exchange  ~5 ms
failed exchange      ~37 ms   (TX_DONE 10 + RX_DONE 25 + 2 ms inter-exchange)
APOS_MIN_N_OK        = 10
```

With exchange success probability `p`, the batch fits `N = 700 / (37 - 32p)`
attempts and collects `pN` successes. Requiring 10:

```
700p / (37 - 32p) >= 10   ->   p >= 0.363
```

An SS-TWR exchange needs BOTH directions, so `p ~= p_oneway^2`:

| | exchange success needed | **one-way success needed** |
|---|---|---|
| SS-TWR (2 frames) | >= 0.363 | **>= 0.60** |
| DS-TWR (3 frames) | >= 0.363 | **>= 0.71** |

So the survey needs a **60 % one-way packet success rate**, not the ~50 %
that a bare sensitivity figure implies. Its useful range is shorter than the
link's.

**DS-TWR makes this worse, not better.** Three frames must survive instead of
two, so it demands a stronger link for the same survey outcome. DS-TWR buys
accuracy (it cancels clock offset by construction), never range. Do not plan
a longer-range deployment around it.

One instrument artefact worth knowing before reading `n_ok`: in the failure
case each attempt costs 37 ms, so near the edge a batch reaches only ~19
attempts rather than 64. `n_ok` therefore saturates exactly where resolution
is most wanted — a pair at `n_ok` 9 and one at `n_ok` 2 are much further
apart in link quality than the numbers suggest.

## 3. What does NOT break at 100 m

Stated because these are what one assumes fails first, and none of them does:

- **Time of flight at 100 m is 333 ns**, against a 2000 uus (2052 us)
  responder turnaround — 0.016 %. The initiator's RX window
  (`RESP_TIMEOUT_UUS` 4000 uus) has ~2 ms of slack. SS-TWR timing is not the
  limit.
- `dwt_setpreambledetecttimeout(0)` — disabled, so nothing truncates a
  distant reception.
- `TDOA_DTU_MAX_SPREAD` is 153.7 m of path difference; anchors 100 m apart
  can differ by at most 100 m. Fits, with 1.5x margin — **a larger array does
  not.**

Two topology constraints that DO appear at that scale: the gateway must reach
every anchor to command `RANGE_CMD`, and for TDoA every anchor must hear the
gateway's CCP. Both are gateway-TX-to-anchor-RX, the same unamplified
receiver. At 100 m spacing the CCP is the likely binding constraint on TDoA,
ahead of the blink itself.

## 4. Pin the confounders FIRST

Nothing measured before this is worth recording.

- **TX power is not the same on every board.**
  `UWB_PHY_TXCONFIG_INITIALIZER` changed from `0xffffffff` to `0xfafafafa`
  (fine gain 63 -> 58, ~-1.25 dB) on an untested hypothesis that the DW3220
  overdrives the QM14070 into compression.
  `docs/discovery-silent-anchor-debug.md` says it plainly: *"boards flashed
  before this commit are still at `0xffffffff`"*. Three anchors flashed at
  different times are transmitting at different powers, which on its own
  explains "only anchors 2 and 3 reached 30 m".
  **Reflash all boards from one build.**
- **`dwt_setfinegraintxseq(0)` has never had a controlled before/after.**
  The same range test settles both if it is run as an A/B on TX power.
- **Record the power source.** CLAUDE.md: on a 3.6 V LiPo the gateway emits
  beacons and no CCPs at all, while USB-C works. A range number taken on
  battery is not comparable to one taken on USB-C.
- **Record antenna height at both ends and keep it constant.** At 6.5 GHz
  (lambda 4.6 cm) the two-ray breakpoint is `4*h1*h2/lambda`: 7.8 m at 0.3 m
  height, 86 m at 1.0 m, 195 m at 1.5 m. Past the breakpoint path loss goes
  as d^4 instead of d^2. Two runs at different heights are not comparable.

## 5. Procedure

### 5a. The SS-TWR link (what the survey needs) -- use `cal link`

`cal link <peer id> [attempts]` measures the RADIO. It takes **no reference
distance**, never persists anything, and does not apply the 25 % floor that
`cal peer` does -- at range a low success rate is not a failed measurement, it
IS the measurement.

```
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf"
west flash -d build_cal
```

on BOTH boards, from the same tree so their TX power agrees (see 4). Then on
one of them, at each distance:

```
cal link 2          # 128 attempts, the default
cal link 2 512      # a bigger denominator, for the far end of the sweep
```

```json
{"link":{"peer":2,"attempted":128,"valid":96,
 "p_exch_permille":750,"p_oneway_permille":866,
 "p_dstwr_projected_permille":649,
 "stats_over":96,"mean_mm":30124,"sd_mm":47,"min_mm":29980,"max_mm":30310,
 "rx_dbm_x10":{"mean":-812,"min":-874,"max":-771,"n":94},
 "fail":{"tx_start":0,"tx_done":0,"rx_to_err":32,"len":0,"hdr":0,"layout":0}}}
```

Four things it reports that a success count alone does not:

- **`p_dstwr_projected_permille`** -- the DS-TWR exchange rate this link would
  give, **without DS-TWR being implemented**. Any TWR variant's exchange rate
  is `p_oneway^frames`: 2 for SS-TWR, 3 for DS-TWR. `p_oneway = sqrt(p_exch)`
  assumes the two directions are equally reliable, which holds for
  anchor-to-anchor (identical hardware, reciprocal channel) and would NOT hold
  against a tag, which has no PA. At the survey's own 36.3 % floor, DS-TWR
  projects to **21.8 %** -- below the same floor. That is section 2's
  conclusion, measured rather than assumed.
- **`sd_mm`** -- success rate is a CLIFF; the spread widens well before the
  count moves. This is the early-warning number and `cal peer` never had it.
- **`rx_dbm_x10`** -- the actual received level. It degrades smoothly where
  the success rate does not, so it says how far from the -93 dBm floor the
  link is rather than only whether it is over it. **This is also the
  measurement that settles section 0's open hypothesis**: read it at a known
  short distance and compare against the -57 dBm at 0.5 m prediction. Nothing
  has ever checked that on a post-rework board. `n` counts the exchanges that
  produced a level at all -- the CIA can legitimately not have finished on
  this polled path, which is UNKNOWN, not weak.
- **`fail`** -- at range `rx_to_err` should dominate (the response never
  arrived). A batch dominated by `hdr` or `layout` instead is interference or
  a peer on a different build, and needs the opposite response.

`stats_over` is how many samples the distance statistics used: `attempts` may
exceed the 128-sample store, in which case `valid` keeps counting past it and
the statistics cover the first 128.

Sweep 10, 20, 30, 40, 50, 75, 100 m. Acceptance for "the survey works at
distance d" is `p_exch_permille >= 363` (section 2); the command warns below it.

#### Cross-check with `apos run`

`cal link` measures the radio; `apos run` measures the thing that has to work.
They are not interchangeable, and the differences run both ways:

| | `apos run` | `cal link` |
|---|---|---|
| attempts per batch | 64 while good, **~19 near the edge** (the 700 ms deadline, section 2) | **128 fixed (up to 512), no deadline** |
| needs a gateway | yes, and it must reach BOTH anchors | no -- **two boards** |
| beacon_guard / superframe | yes | not in that image |
| RX level, spread, failure breakdown | no | yes |
| runs the code that ships | **yes** | no |

So sweep with `cal link`, then confirm the best usable distance with
`apos run` on the production image. **The cal image draws far less current**
(`cal.conf` sets `CONFIG_WIFI=n` / `CONFIG_NETWORKING=n`) and this project has
twice measured supply sag taking out a transmission -- so a `cal link` range
is an **upper bound** for production, and the gap between the two is itself
worth recording.

### 5a-bis. `cal peer` (needs a reference distance)


The calibration image's `cal peer <id> <mm>` is the better instrument here,
and not only because it is convenient:

| | `apos run` | `cal peer` |
|---|---|---|
| attempts per batch | 64 while the link is good, **~19 near the edge** (the 700 ms deadline, see §2) | **128, fixed, no deadline** |
| needs a gateway | yes, and it must reach BOTH anchors | no -- **two boards** |
| beacon_guard / superframe | yes | not in that image |
| spread reported | `sd_mm` | **no sd** -- only `kept` vs `valid` |

The fixed denominator is the point. `apos`'s `n_ok` saturates exactly where
resolution is wanted, because a failing exchange costs ~37 ms and the batch is
cut off by time rather than by count. `valid / 128` does not.

```
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_cal -- "-DEXTRA_CONF_FILE=cal.conf"
west flash -d build_cal
```

on BOTH boards -- from the same tree, so their TX power agrees (§4). Then, on
one of them, at each distance:

```
cal peer <peer anchor id> <tape distance in mm>
```

`cal peer` **never persists anything** -- it reports and returns -- so it is
safe to run repeatedly at every distance. (`cal ref` does persist; do not
confuse them during a range test.)

Read `valid` out of `attempted` (128). That is the exchange success rate.
`error_mm` is the ranging accuracy at that distance, for free.

**Below 32 of 128 (25 %) `cal peer` refuses to report a distance** -- that
gate exists for its actual job, calibration, where a mean over a handful of
lucky frames would look like a result. It still prints `attempted` and
`valid`, which in a range test is the measurement, so the region below the
gate is not lost. (It did not print them before 2026-09-03; the message named
three causes that are all wrong when the real one is distance.)

Sweep 10, 20, 30, 40, 50, 75, 100 m and record `valid`, `kept`, `error_mm`
and the tape truth at each.

Acceptance for "the survey works at distance d": from §2, `valid/128 >= 0.363`
-- i.e. **at least 47 of 128**. Below that the real survey's batch runs out of
its 700 ms deadline before collecting `APOS_MIN_N_OK` successes, whatever the
link is doing.

**Two things this instrument does NOT tell you**, both worth writing down
before the numbers are trusted:

- **No spread.** `cal_result` carries no `sd_mm`, so degradation short of
  outright failure is only visible as `kept` falling away from `valid`
  (samples lost to outlier rejection). `apos run` reports `sd_mm` properly;
  cross-check the final distance there.
- **The cal image draws far less current than production.** `cal.conf` sets
  `CONFIG_WIFI=n` / `CONFIG_NETWORKING=n`, and this project has twice
  measured supply sag taking out a transmission (the wedged-DW3220 entry and
  the battery-gateway CCP entry in CLAUDE.md). So a range measured on the cal
  image is an **upper bound** for production, and the gap between them is
  itself worth measuring: repeat the best distance with `apos run` on the
  production image and compare.

### 5b. The TDoA receive path (tag -> anchor, the unamplified direction)

Reading `blink stats` on three consoles in a field is impractical. The broker
sees every anchor at once and the `a` field says which one heard each blink:

```bash
mosquitto_sub -h HOST -t "uwb/anchor/blink/#" -v > range_30m.txt
python tools/blink_coverage.py range_30m.txt
```

That reports, per anchor, the fraction of blinks it contributed to.

**One thing MQTT cannot see**: an anchor that hears a blink but has lost CCP
sync DISCARDS it and publishes nothing (`blink_rx.c`). So an MQTT capture
measures `stamped`, not `rx`. If coverage collapses at distance, read
`blink stats` on that anchor's console once — `no_sync == rx` means the CCP
link died, not the blink link, and those need opposite fixes.

### 5c. Recording

For every run: distance (tape), antenna height both ends, power source, the
TX power value each board is flashed with, and whether line of sight was
clear. A run missing any of these cannot be compared to another one.

## 6. The <1 m failures are a different problem

An anchor that does not answer `apos enum` at under 1 m is not a range
problem — there is ~60 dB of spare margin there. Triage in this order,
cheapest first:

1. **Put it on USB-C.** If it answers on USB and not on battery, it is the
   supply, and this project has already measured that failure twice (the
   wedged-DW3220 entry and the battery-gateway CCP entry in CLAUDE.md). Two
   minutes, and it rules out the expensive explanation.
2. **`anchor show` on each board.** Confirm `id` and `mode`: a board in
   `gateway` mode does not answer `apos enum`. Note that a physical label is
   not the `anchor id` — recent captures show peers at id 1, 2 and 3 with
   **id 0 absent from every one of them**.
3. **Swap `anchor id` between a working board and a suspect one.** CLAUDE.md's
   own recipe: if the symptom follows the *id* it is firmware or config; if it
   follows the *board* it is hardware. Two console commands, no rebuild.
4. Only then, reflow-inspect the PA.

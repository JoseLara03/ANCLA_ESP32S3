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

### 5a. The one-way link (what sets everything else)

The cleanest instrument is the **survey's own** `apos_pair` report, because
it gives success rate and distance together:

```
apos gauge origin=<id> xaxis=<id> plane=<id>
apos run
```

then read, per pair:

```json
{"apos_pair":{"i":3,"of":6,"from":"0x0004","to":"0x0003","mean_mm":2750,"sd_mm":39,"n_ok":35}}
```

`n_ok` out of 64 is the exchange success count; `mean_mm` and `sd_mm` are the
distance and its spread. Walk one anchor out in steps (10, 20, 30, 40, 50,
75, 100 m), running `apos run` at each, and record `n_ok`, `mean_mm`, `sd_mm`
plus the tape-measured truth.

The number to plot is `n_ok / attempts`, and remember from §2 that
`attempts` is 64 only while the link is good — near the edge it is ~19.

Acceptance for "the survey works at distance d": `n_ok >= APOS_MIN_N_OK`
(10) on every pair, and `sd_mm` still in the tens rather than the hundreds.

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

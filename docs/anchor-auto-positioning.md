# Anchor auto-positioning — operator procedure

Date: 2026-08-14

Measures every anchor pair over the air, solves the inter-anchor geometry in 3D,
and writes the solved coordinates into each anchor's NVS and into the gateway's
retained anchors payload — so the platform map and the tag's solver agree by
construction instead of by hand-typed `anchor pos` commands.

**Status of this procedure at time of writing:** implemented, built, and code
reviewed (branch `feat/anchor-auto-positioning`). It has **not been run against
real hardware** — not one survey has been executed on a board. This document is
the runnable procedure for whoever does that next. Do not read the presence of
this document as evidence that a deployment is surveyed; check `apos show` for
`stored survey: yes` instead.

Two numbers in particular have never been timed on hardware and are the first
things to measure on a bench session:

- `APOS_GW_SOLVE_BUDGET_UUS` (150000, `src/apos_gw.h`) — the clear-air the
  Levenberg-Marquardt solve must see before it is allowed to start on the
  cooperative gateway loop.
- The survey-persist gate, which deliberately **reuses the same constant**
  (`src/apos_gw.c`, `step_apply()`), because the NVS write is an XIP flash
  operation that stalls all execution regardless of thread priority.

Both exist to keep the beacon on time. If a bench run ever shows a late beacon
at the instant `{"apos_solve":...}` or `{"apos_store":...}` is logged, measure
those two operations before changing anything else.

---

## 0. Read this before you trust a result

**On the real deployment the acceptance check cannot fail for the right
reasons, and `"accepted":1` does not mean the geometry is correct.**

The gauge (§3) pins 6 of the solved system's degrees of freedom, leaving
`3N - 6` free parameters for `N` placed nodes. A **four**-node array with a full
mesh has exactly **6 edges against exactly 6 free parameters** — an isostatic
system with no spare equation. Levenberg-Marquardt then re-embeds *whatever*
distances it was handed, exactly, and `rms_m` / `worst_edge_m` come back at
**identically zero however bad the ranging was**. Between N = 5 and N = 7 there
is enough redundancy for `rms_mm` to read nonzero, but not always enough for
`worst_pair` to name the pair actually at fault: least-squares masking can shift
two good nodes just enough to move the disagreement onto a merely-correlated
edge. Both were reproduced in `tests/apos_geom/test_apos_geom.c`.

**The deployment is four ranging slaves plus a gateway — exactly the degenerate
case.** The gateway is not surveyed (§2), so N = 4.

The firmware says so itself rather than hiding it:

- the solve report carries `"spare_edges"` and `"rms_meaningful"` fields;
- a two-line `LOG_WRN` pair fires at solve time and again at apply time;
- `apos apply` prints a `WARNING: this survey is UNVERIFIED` block back to your
  own console, whether or not you passed `force` — `force` overrides
  acceptance, not physics.

So: on a 4-anchor array, `"accepted":1` means only **"nothing contradicted the
ranges"**. The checks that actually validate the geometry are

1. a **tape measure** against the solved node-to-node distances printed by
   `{"apos_node":...}`, and
2. the two **ranging-quality** numbers the solve reports alongside `rms_mm`:
   `max_reciprocal_mm` and `max_sd_mm` (also printed by `apos show`).

`max_reciprocal_mm` is the largest `|d(A→B) − d(B→A)|` over the pairs measured
in both directions — a number the fit never sees, because
`apos_table_symmetrise()` averages the two directions away. `max_sd_mm` is the
largest per-pair spread within a batch. Both describe the **ranging**: a
`max_reciprocal_mm` in the tens of millimetres is a healthy array, and one in the
hundreds means a bad pair however clean `rms_mm` looks.

**Be precise about what they do:** they make the ranging *observable*. They do
**not** make the geometry over-determined. A rigid four-node framework is
isostatic whatever its edges measure, so `rms_mm` stays vacuous and the tape
measure stays the check on the geometry itself.

There is no fifth-anchor option. `UWB_MAX_ANCHORS` is 4, `anchor id` is bounded
`0..3`, and an anchor refuses a `RANGE_CMD` naming a peer at or beyond
`UWB_ANCHOR_ADDR_BASE + UWB_MAX_ANCHORS` (`src/apos_node.c`) — so
`APOS_MAX_NODES` = 8 is headroom in the data structures, not a configuration.
Raising the anchor count is **engineering work, never an operator action**: it
touches `UWB_MAX_ANCHORS`, `disc_schedule`'s response stagger,
`anchor_respond.c`'s `TX_COMPLETE_TIMEOUT_MS` (derived from that stagger's worst
case), and the tag project's `UWB_FRAME_MAX_ANCHORS` on the far side of a frozen
wire format. Every supported deployment is the degenerate case.

## 1. Prerequisites

### 1.1 Antenna-delay calibration, on every anchor, first

**Run `docs/antenna-delay-calibration.md` on every board before surveying.**

Every unit still ships the factory seed `ant_tx = ant_rx = 16385`
(`UWB_ANT_DELAY_DEFAULT`), which has never been trimmed on this hardware. An
uncalibrated pair's range is offset by a constant that the survey has no way to
separate from real geometry, so a survey run first produces a **confidently
wrong** set of coordinates, accepted without complaint (see §0 — on a 4-node
mesh it cannot complain). The acceptance thresholds in §5 are derived from the
calibration cross-check's per-pair figure of 30 mm; before calibration they mean
nothing at all.

Check `anchor show` on each board for `ant_tx != 16385` before you start.

### 1.2 Boards

- **Four ranging anchors**, `anchor mode slave`, ids `0..3` (short addresses
  `0x0001`–`0x0004`). Four distinct nodes is the minimum a 3D gauge can pin
  (`APOS_MIN_NODES`), and it is what `apos run` refuses below.
- **One gateway**, `anchor mode gateway`, with its own `anchor pos` set. Five
  boards in total.
- The production image on all five — `west build -b ancla_esp32s3/esp32s3/procpu`
  with no `EXTRA_CONF_FILE`. The survey is production-image functionality; the
  calibration image (`cal.conf`) is a different procedure and must not be on air
  at the same time.

## 2. The gateway is not a survey node

The gateway is MAC-only for ranging: it holds the reserved short address
`0x0000`, consumes no `anchor_id`, and **never answers a ranging poll**. Nothing
can therefore measure a distance to it, and it is not part of the solved
geometry. Its coordinate stays hand-entered with `anchor pos`, and an anchor
that answers enumeration claiming `0x0000` is rejected outright:

```
{"apos_error":"an anchor answered enumeration with the gateway's reserved address 0x0000 — ignored"}
```

A 1 + 4 deployment surveys **four** nodes.

## 3. Physical setup and the gauge

Mount every anchor in its final position first. Ranging fixes the array's
*shape* but not its placement — the solution is free up to translation, rotation
and reflection — so four operator designations pin the frame:

| designation | meaning |
|---|---|
| `origin` | becomes `(0, 0, 0)` |
| `xaxis`  | becomes `(d, 0, 0)`, `d > 0` — defines +x |
| `plane`  | becomes `(x, y, 0)`, `y > 0` — defines the +y side |
| `up`     | forced to `z > 0` — resolves the reflection |

Pick the four against a **site sketch**, not against whichever board is nearest.
The gauge is entered as short addresses, not indices, precisely so it survives a
re-enumeration and an `anchor id` change; a transposed gauge produces a
plausible-looking but wrong coordinate frame that nothing downstream can detect.

**Put at least one anchor at a clearly different height.** A coplanar array has
no information about z, and the solver reports this as a small `planarity_mm`
with the warning

```
{"apos_warn":"array is near-coplanar (… mm) — x and y are good but the solved z values are not survey-quality"}
```

x and y remain usable in that case; z does not.

`apos zoff <metres>` adds a constant to every placed node's z after the solve,
which is how you move `z = 0` off the plane through the three gauge anchors and
onto the floor. It is applied **at solve time**, so changing it needs a re-run —
it never silently rewrites a result you have already read.

## 4. Walkthrough

Everything below is typed on the **GATEWAY's** console (`uwb:~$`). Every `apos`
subcommand except `show` refuses on a slave with

```
error: `apos` runs on the GATEWAY — this board is a SLAVE. Set `anchor mode gateway` and reboot.
```

No `apos` command transmits anything itself: each sets state and returns, and the
gateway loop does the radio work and logs the outcome. That is why the results
below appear in the monitor rather than under your prompt.

### 4.1 Enumerate

```
apos enum
```

`SURVEY_BEGIN` is broadcast `APOS_GW_ENUM_ROUNDS` (3) times, 400 ms apart, and
each anchor replies in an EUI-64-hashed stagger slot. Expect one line per anchor
and then the summary:

```
{"apos":"enumerating","session":41337}
{"apos_peer":{"idx":0,"addr":"0x0001","eui":"A1B2C3D4E5F60708","pos_valid":0,"x":0.000,"y":0.000,"z":0.000}}
…
{"apos_enum_done":{"peers":4}}
```

`pos_valid:0` on a fresh deployment is expected and is not a fault.

### 4.2 Pin the frame

```
apos gauge origin=0x0001 xaxis=0x0002 plane=0x0003 up=0x0004
```

Named arguments, in any order; all four are required and must be distinct and
non-zero. Confirmed with `{"apos_gauge":{…}}`. The addresses need not be
enumerated yet — setting the gauge from a site sketch before powering the array
is legitimate.

### 4.3 Optionally move z = 0 to the floor

```
apos zoff 2.4
```

Prints `{"apos_zoff_m":2.400} — takes effect on the next apos run`.

### 4.4 Run

```
apos run
```

Re-enumerates (always — a board may have been rebooted, re-addressed or
swapped), then commands **every ordered pair** to range: `N*(N-1)` = 12
commands for four anchors, each a batch of `APOS_GW_N_EXCHANGES` (40) SS-TWR
exchanges. Both directions of each pair are measured and averaged, which is what
cancels residual antenna-delay asymmetry.

**`apos run` persists nothing.** It reports and stops.

```
{"apos":"run","session":41338}
{"apos":"ranging","ordered_pairs":12}
{"apos_edges":{"usable":6,"missing_pairs":0}}
{"apos_solve":{"nodes":4,"placed":4,"ambiguous":0,"rms_mm":0,"worst_mm":0,"worst_pair":[0,0],"planarity_mm":312,"iters":4,"spare_edges":0,"rms_meaningful":0,"accepted":1}}
{"apos_node":{"idx":0,"addr":"0x0001","x":0.000,"y":0.000,"z":0.000,"state":1,"resid_mm":0}}
…
{"apos_warn":"rms_mm and worst_mm are NOT a quality check on this array: …"}
{"apos_warn":"\"accepted\":1 above therefore means only that nothing contradicted the ranges …"}
```

Note the `rms_mm:0` / `spare_edges:0` / `rms_meaningful:0` combination in that
example. That is the normal, expected output of a healthy four-anchor run — and
it is exactly the output §0 is about. **Check the `apos_node` coordinates
against a tape measure now**, and read `max_reciprocal_mm` / `max_sd_mm`
(§0), before applying. Taking your time here is safe: `apos apply` re-opens the
anchors' survey windows before it pushes anything (§4.5).

### 4.5 Apply

```
apos apply
```

Apply first re-broadcasts one `SURVEY_BEGIN` on the **same session**, then
settles for `APOS_GW_APPLY_SETTLE_MS` (400 ms) before the first `SETPOS`. That
re-opens every anchor's survey window: a window is refreshed only by an
in-session `SETPOS` or `RANGE_CMD` addressed to that anchor, the last
`RANGE_CMD` an anchor sees can be near the *start* of the ranging phase, and
`APOS_NODE_REFRESH_S` is 60 s — far less than the time §4.4 tells you to spend
with a tape measure. Without the re-broadcast every `SETPOS` in the array would
be refused. The staggered `ENUM_RSP` replies the re-broadcast provokes are
discarded by the gateway (its enumeration handler is phase-guarded).

Pushes each node's coordinates to its anchor as a `SETPOS`, waits for a
`SETPOS_ACK` from each (up to `APOS_GW_APPLY_RETRIES` + 1 attempts), persists the
survey on the gateway, and closes the survey windows with `SURVEY_END`. Anchors
apply the new position **immediately**, without a reboot.

If the last run failed acceptance, `apos apply` refuses:

```
error: the last run FAILED acceptance (rms=… mm, worst=… mm on pair [i,j], placed=n/N, ambiguous=k). Fix the geometry and re-run, or `apos apply force` to commit it anyway.
```

`apos apply force` is the only argument accepted; anything else is
``error: the only argument is `force` ``. `force` overrides the **thresholds**
only — the unverified-mesh warning still prints.

The line to read at the end:

```
{"apos_apply_done":{"ok":4,"failed":0,"skipped":0,"nodes":4,"persisted":1}}
```

All five fields matter. See §6 for what a nonzero `failed`/`skipped` or a
`"persisted":0` means.

### 4.6 Geographic reference

```
apos ref 21.016042 -89.652129
```

Decimal degrees, lat in `-90..90` and lon in `-180..180`. This is the **origin
anchor's** real-world position; the platform places the entire local frame
against it, and every other anchor is published as local-only metres relative to
it. It is persisted independently of the survey and survives a re-survey — the
building does not move when you re-measure the geometry.

`apos ref` **refuses while a survey is running** (`apos_gw_busy()`):

```
error: a survey is running — wait for it to finish, then set the reference
```

That is deliberate: it writes flash from the shell thread, and on this
XIP-from-flash part a write or erase stalls all execution.

### 4.7 Verify and reboot

```
apos show
kernel reboot cold
```

`apos show` prints the current phase and session, the enumerated peers, the
stored survey, and either the stored `ref_lat`/`ref_lon` or

```
{"ref":"unset — run `apos ref <lat> <lon>` or the platform cannot place the map"}
```

`apos show` is the one `apos` subcommand that works on any board.

## 5. Reading the report

From `{"apos_solve":...}`:

| field | meaning |
|---|---|
| `nodes` / `placed` | enumerated nodes, and how many the solve could place. A node with fewer than three edges to placed nodes stays unplaced. |
| `ambiguous` | placed, but its reflection was a guess — it needs a fourth measured edge. |
| `rms_mm` | RMS residual over usable edges. **Meaningless unless `rms_meaningful` is 1 — see §0.** |
| `worst_mm`, `worst_pair` | largest single-edge residual and the node-index pair that produced it. Same caveat, plus masking (§0). |
| `planarity_mm` | RMS distance of placed nodes from their own best-fit plane. **Small is bad**: it means near-coplanar and untrustworthy z. |
| `iters` | LM iterations used, capped at `APOS_LM_MAX_ITER` (200). A clean mesh converges in under ten. |
| `spare_edges` | usable edges minus `3*placed - 6`. `<= 0` is the unverified regime. |
| `rms_meaningful` | 0 exactly when `spare_edges <= 0`. |
| `accepted` | whether every threshold below passed. |

From `{"apos_node":...}`: `state` is 0 unplaced / 1 placed / 2 ambiguous, and
`resid_mm` is that node's own RMS over its edges.

From `{"apos_edges":...}`: `usable` edges after symmetrisation and
`missing_pairs`, the unordered pairs with no usable measurement in either
direction. A hole is reported, not fatal — the fit works around it.

**Acceptance** (`src/apos_gw.h`), all of which must hold for `"accepted":1`:

- every node placed (`placed == nodes`);
- `ambiguous == 0`;
- `rms_mm < APOS_ACCEPT_RMS_MM` (50);
- `worst_edge <= APOS_ACCEPT_WORST_FACTOR (3.0) * rms + 50 mm`.

`APOS_ACCEPT_PLANARITY_MM` (100) does **not** block acceptance — x and y are
still good when the array is flat — but it is warned about.

These three constants are the numbers most likely to need adjusting after the
first real bench session. They are thresholds on an already-reported result, so
changing one never changes what was measured — only whether `apos apply` will
proceed without `force`.

## 6. Troubleshooting

### A discovered anchor that never answers a ranging poll

**Read this first — it looks exactly like an RF fault and is not one.**

An anchor with no position now **refuses** to answer a WAVE ranging poll
(`anchor_respond.c`: `WAVE poll refused — no surveyed position`). This replaced
the old behaviour of answering with a silent `(0, 0)`, which is strictly worse.

But `anchor_respond_discovery()` is deliberately **not** gated. An unpositioned
anchor therefore still answers DISCOVERY, still enters the tag's anchor list —
and then goes silent on the poll. The bench symptom is *"the tag discovers three
anchors and ranges none of them"*, or a sniffer capture showing `0xE2`/`0xE4`
DISCOVERY traffic with no matching `0xE0`/`0xE1`.

The fix is not the radio. Either survey the board, or give it an `anchor pos`.
A freshly `anchor reset` board is silent to tags by design until one of those
happens.

The gate has one exception, which is what lets a **cold** deployment bootstrap:
while a gateway-opened survey window is open (`apos_node_window_open()`), an
unpositioned anchor does answer polls — otherwise no anchor could ever range its
peers to get a position in the first place.

### "duplicate short address 0x000N — two boards share an `anchor id`"

Two boards are configured with the same `anchor id`. Everything downstream
addresses peers by short address, so the run is aborted rather than ranging one
board and pushing coordinates to the other. Fix with `anchor id` on one of them
and re-run.

### `gauge address 0x000N did not answer enumeration`

That board did not reply — powered off, out of range, or on a different
`anchor id` than the site sketch says. Run `apos enum` alone and compare the
addresses listed against the gauge you typed.

### `the gauge anchors are not mutually ranged`

The seed needs the three gauge-plane edges plus at least three edges from `up`.
Move the gauge anchors into line of sight of each other and re-run, or pick a
different four.

### `{"apos_hole":{"from":…,"to":…,"why":"no RANGE_RSP"}}`

That ordered pair could not complete after `APOS_GW_RANGE_RETRIES` + 1 attempts.
The fit works around a hole, but every hole removes a constraint from an already
constraint-starved mesh (§0). Move an anchor into line of sight, or accept it
knowingly.

### `{"apos_thin":{"from":…,"to":…,"n_ok":k}}`

The batch completed but produced fewer than `APOS_MIN_N_OK` (10) good exchanges,
so the measurement was discarded as too thin to trust. Reported rather than
suppressed: "the pair cannot range" and "the command never arrived" must not
collapse into the same silence. Usually a marginal link.

### `… node(s) reflection-ambiguous`

A node was placed from exactly three edges, so its mirror image fits equally
well and the solver guessed. It needs a **fourth** measured edge to a placed
node, or a repositioned anchor. Ambiguity blocks acceptance.

### `array is near-coplanar (… mm)`

All the anchors are at effectively the same height. x and y are fine; the solved
z values are not survey-quality. Raise or lower one anchor and re-run.

### `0x000N never acknowledged SETPOS after 4 attempts — it is still on its OLD coordinates`

That anchor kept its previous position while its peers moved to new ones — a
silently inconsistent deployment. Also reported as a nonzero `failed` in
`apos_apply_done`. Re-run `apos apply`: each apply re-opens the anchors' survey
windows before its first `SETPOS` (§4.5), so a second attempt starts from the
same clean state as the first, however long you spent between them.

If **every** anchor fails this way, and the anchors' own consoles show
`SETPOS for session … ignored — current is 0`, the windows had lapsed and were
not re-opened — which would mean the re-broadcast in §4.5 did not go out. Look
for the `apos` `TX failed` warnings on the gateway in the same window.

### `0x000N applied but could not persist — it will revert on reboot`

The anchor took the coordinates into RAM but its NVS write failed, so it will
silently revert on its next reboot. Counted as a failure. Re-run `apos apply`.

### `{"apos_skip":{"addr":…,"reason":"unplaced"}}`

The solve could not place that node, so there was nothing to push. Pushing
`(0, 0, 0)` is exactly the failure this feature exists to remove, so it is
skipped and counted.

### `"persisted":0` in `apos_apply_done`

**The anchors are correct and the gateway is not.** The coordinates landed in
every anchor's NVS, but the gateway failed to save its own copy of the survey.
After a gateway reboot `apos_store_get()->valid` is false, `pos_json_anchors()`
falls back to the four-anchor `ANC-LOBBY-001..004` stub, and **the platform map
will disagree with the solver** while every ranging result stays correct — a
split that is very hard to diagnose from the broker. Re-run `apos apply`.

The same condition is also logged on its own line at persist time and again as
an explicit `apos_error` in the summary.

### The survey appears stuck

`{"apos_error":"no step handler for phase N — survey is stuck; kernel reboot cold to clear"}`
is the only genuinely wedged state, and it self-clears the phase. Otherwise a
survey simply takes a while: steps are skipped whenever less than
`APOS_GW_STEP_BUDGET_UUS` remains before the beacon must be armed, which costs
latency and never correctness.

`error: a survey is already running` from `apos enum` / `apos run` / `apos apply`
means `apos_gw_busy()` is true. Wait for the current phase to report.

## 7. Acceptance gates for a first hardware session

None of these have been run. Tick them in order.

- [ ] Every anchor calibrated per `docs/antenna-delay-calibration.md`
      (`anchor show` reports `ant_tx != 16385` on each).
- [ ] `apos enum` on the gateway lists exactly four peers with four distinct
      EUI-64s and four distinct short addresses `0x0001`–`0x0004`.
- [ ] `apos run` completes with `missing_pairs:0`, `placed == nodes`,
      `ambiguous:0`, and a `planarity_mm` comfortably above 100.
- [ ] The `apos_node` coordinates match a **tape measure** to within ~50 mm on
      every node-to-node distance. This, not `rms_mm`, is the real check on a
      four-anchor array (§0).
- [ ] The beacon stays on time throughout: no
      `"beacon started but TXFRS never completed"` in the monitor during the
      ranging phase, during the solve, or during the persist.
- [ ] `apos apply` reports `ok:4, failed:0, skipped:0, persisted:1`.
- [ ] Each anchor's `anchor show` reports its solved coordinates immediately,
      with no reboot.
- [ ] Coordinates survive `kernel reboot cold` on every anchor and on the
      gateway (`apos show` still says `stored survey: yes`).
- [ ] `apos ref` set, and the retained `uwb/anchor/setup/852541` payload carries
      the surveyed COORDINATES. The anchor **names** stay `ANC-LOBBY-00N`
      deliberately — the platform may key its records on them, so a survey
      changes coordinates only (`src/pos_json.c`).
- [ ] A tag ranges all four anchors and its `0xEA` `residual` is under ~0.1 m,
      with `(x, y)` stable between consecutive fixes with nothing moving.

## 8. What this procedure does not fix

- **It does not validate itself on a four-anchor array.** §0. A tape measure or
  the ranging-quality numbers plus a tape measure do (§0). There is no fifth
  anchor to add.
- **It does not calibrate antenna delay**, and cannot detect an uncalibrated
  array — a uniform delay error looks like a slightly larger room.
- **It does not survey the gateway.** §2.
- **It does not run itself.** `apos_gw_trigger_from_mqtt()` exists and accepts
  `{"cmd":"run"}` / `{"cmd":"apply"}` on `uwb/anchor/survey/<zone>`, but
  `net_uplink.c` subscribes to nothing, so nothing calls it yet. Setting the
  gauge and `apply force` are deliberately console-only and will stay so.

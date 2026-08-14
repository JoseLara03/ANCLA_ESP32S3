# Anchor auto-positioning — design

Date: 2026-08-14
Status: design approved, not implemented

## 1. Problem

Anchor coordinates are hand-entered with `anchor pos <x> <y> <z>` and are
silently wrong when unset. `position_valid` gates only the GATEWAY's beaconing,
so a SLAVE with no position answers ranging polls reporting `(0, 0)`. Three
anchors all claiming the origin yield a confidently meaningless fix with no
error reported anywhere. This has already cost a bench session: after an
`anchor id` swap the coordinates did not follow the ids, leaving two live
anchors on the same baseline and the apex coordinate stranded on a third board.

The anchors payload published retained to `uwb/anchor/setup/<zone>` is also
still a stub — `ANC-LOBBY-001..004` at the corners of a 2 m square, only `-001`
carrying a real lat/long (`pos_json.c`). The platform therefore draws the stub
geometry while the tag solves against whatever was hand-entered. Map and
solver disagree by construction.

This design replaces hand entry with a measured survey: anchors range each
other, the gateway solves the geometry, and the result is pushed back to every
anchor and published as the real anchors payload.

### 1.1 What the prior implementation did, and why it is not ported

`fw-cre/firmeware_creator/Src/owner/owner_auto_position.c` achieved
auto-positioning on the nRF5 platform with a sequential bootstrap: an ordered
list of anchor ids (`cfg->coord_anchor_ids[]`) is broadcast, each anchor finds
its own id in that list, index `k` selects a time slot `k * 6000 ms`, and in
that slot the anchor ranges whichever peers already have positions and
trilaterates itself. Index 0 becomes the origin, index 1 takes `(d, 0)`, index
2 solves a 2-circle intersection, and index 3+ least-squares.

Three properties make it the wrong starting point here:

- **Every board must be pre-loaded with the same ordered deployment list.** Role
  comes from a board's index in that list, so the list is per-board
  configuration that must agree across the whole deployment. This is precisely
  the class of state that desynchronised on the `anchor id` swap.
- **Errors compound down the chain.** Anchor 3's position depends on 1 and 2
  being already correct, and a failure at step `k` invalidates every later step.
- **It requires a full mesh.** Each anchor must range enough *already-positioned*
  peers in its own slot. A pair that cannot hear each other stalls the chain
  rather than being fitted around.

The DS-TWR-between-anchors approach recorded in `CLAUDE.md` was also
reconsidered and rejected for now — see §4.

## 2. Decisions

| Question | Decision |
|---|---|
| Orchestration | Gateway orchestrates and solves; anchors are dumb and get told their coordinates |
| Identity | EUI-64, discovered over the air; `anchor_id` is never assumed |
| Frame gauge | Operator supplies it at trigger time, picked from an enumeration table |
| Ranging | SS-TWR, both directions, heavily averaged — the already-calibrated path |
| Scale | Arbitrary N, global sparse least-squares fit; unmeasured pairs are holes, not errors |
| Dimensionality | Full 3D solve |
| 3D scope | Survey is 3D; the tag stays 2D — VEWA is not changed |
| Commit model | `apos run` reports only; `apos apply` persists |

### 2.1 Gateway orchestrates and solves

The gateway enumerates present anchors over the air, commands each one to range
a specific peer, collects the full directed range matrix, solves the whole
geometry centrally in one pass, then pushes coordinates back to each anchor to
persist.

Anchors never decide their own role and never need a deployment list. The
gateway holds the authoritative table, keyed by EUI-64, and it already has the
WiFi/MQTT path needed to publish the anchors payload. Binding coordinates to
EUI rather than to `anchor_id` is what makes an `anchor id` swap harmless.

### 2.2 Anchors are identified by EUI-64

Enumeration reports each anchor's EUI-64 alongside the short address it is
currently using. Nothing in the survey assumes ids `0..3`, contiguity, or any
particular id-to-position mapping. The short address is used only as the
on-air destination for that run; the EUI is what persists.

## 3. Coordinate frame

Inter-anchor ranges determine the array's *shape* but not its placement: the
solution is free up to translation, rotation and reflection. In 3D that is 6
degrees of freedom plus handedness, pinned by four operator designations:

| Designation | Constraint | Fixes |
|---|---|---|
| `origin=<addr>` | `(0, 0, 0)` | translation (3 DOF) |
| `xaxis=<addr>` | `(d, 0, 0)`, `d > 0` | 2 of 3 rotations |
| `plane=<addr>` | `(x, y, 0)`, `y > 0` | the last rotation and the +y side |
| `up=<addr>` | `z > 0` | the reflection |

The three anchors `origin`, `xaxis` and `plane` *define* the `z = 0` plane.
`apos zoff <metres>` shifts every solved z afterwards, so the operator can move
`z = 0` from "the plane through the three gauge anchors" to "the floor" with one
tape measurement.

`up` doubles as the coplanarity canary. If the array is near-planar, `up`'s
solved z comes out near zero and the run is reported as z-degenerate rather
than emitting confident nonsense heights.

The gauge lives on the gateway for the duration of a run. Re-running with a
different gauge is one console command and requires no reflashing and no
per-board state.

### 3.1 Why the gauge is operator input and not a persisted per-anchor role

A `anchor role <origin|xaxis|plane|up>` flag persisted in each board's NVS would
also survive an id change. It was rejected because it is four more pieces of
per-board configuration that must agree across the deployment — the same shape
of state whose disagreement caused the failure in §1. Operator input at trigger
time cannot desynchronise, because it does not persist anywhere to desynchronise
from.

A fully automatic canonical gauge (lowest EUI as origin, farthest peer as +x)
was rejected because the resulting frame's orientation and handedness relative
to the building are arbitrary: the platform map could come out rotated or
mirrored, and the reference anchor's lat/long would be meaningless.

## 4. Ranging method

**SS-TWR, both directions of every pair, averaged over many exchanges.**

The poll is the existing WAVE `0xE0` frame that `cal_initiator.c` sends and
`anchor_respond_wave_poll()` already answers in the production image. This is
the exact exchange the antenna delays were calibrated through, it already
applies the mandatory `dwt_readclockoffset()` correction, and it is
bench-proven. No new radio protocol is introduced, so bring-up risk is close to
nil.

Both directions are measured because averaging A→B with B→A cancels
antenna-delay asymmetry that a single direction bakes into the geometry. Anchors
are static, so averaging is free; the per-direction standard deviation is
retained and becomes the edge weight and the run's quality signal.

DS-TWR between anchors — the approach `CLAUDE.md` proposed evaluating — is
deferred. Its advantage is cancelling clock offset by construction rather than
by correction, but the correction is already applied and is already mandatory on
this network. Against that it costs a new initiator, a new responder path
compiled into the production image, roughly four new frame types, and a fresh
hardware bring-up cycle on a stack that has already spent debug cycles on
delayed-TX timing budgets — all before any geometry could be tested. If measured
pairwise repeatability turns out insufficient, `apos_table`'s edge interface is
the seam to swap it in behind.

## 5. Module layout

Pure C, no Zephyr dependency, host-tested — the same discipline as `gw_core`,
`beacon_guard` and `cal_solve`:

- **`src/apos_frame.{c,h}`** — codec for the survey frames. Deliberately *not*
  added to `uwb_frame_802_15_4z.c`, which must stay byte-identical to the tag's
  copy.
- **`src/apos_geom.{c,h}`** — the sparse 3D solver (§7).
- **`src/apos_table.{c,h}`** — discovered-anchor table, directed-edge
  accumulator, symmetrisation, acceptance thresholds.

Zephyr-side:

- **`src/apos_gw.{c,h}`** — gateway orchestration, stepped from
  `uwb_gateway_run()`'s existing inner loop so the beacon cadence is never
  blocked.
- **`src/apos_node.{c,h}`** — anchor side: answer enumeration, execute one range
  command, persist a pushed position.
- **`src/ss_initiator.{c,h}`** — `cal_initiator.c` promoted out of the cal-only
  file set and shared by both images. It is the exchange the antenna delays were
  calibrated through; two copies would drift.
- **`src/apos_shell.c`** — the `apos` command tree, gateway only.
- **`src/apos_store.{c,h}`** — the gateway's persistent EUI→coords table under a
  new `apos/` settings subtree.

### 5.1 How `apos_gw` hooks into the gateway loop

The gateway loop runs at `K_PRIO_COOP(0)` and the beacon is the network's time
base, so the survey must never take that loop away from beaconing. `apos_gw` is
therefore a step function, not a thread and not a blocking routine, called from
exactly two places in `uwb_gateway_run()`'s existing inner loop:

- **`dispatch()`** gains an `apos_frame_is_apos()` branch that hands received
  `ENUM_RSP` / `RANGE_RSP` / `SETPOS_ACK` frames to `apos_gw_on_rx()`.
- **The idle path** — immediately after a bounded `k_sem_take()` on the RX window
  — calls `apos_gw_step()`, which may emit at most **one** APOS frame and then
  returns. It emits nothing at all unless a survey is active.

One frame per RX window bounds the survey's intrusion on the loop to a single
transmission, and the existing `BEACON_ARM_MARGIN_UUS` break-out is untouched:
the survey simply gets no step in the window where the beacon is being armed.
Survey timeouts are driven off `k_uptime_get()` deltas observed across steps, so
a step that is skipped costs latency and never correctness.

The shell commands do not transmit. `apos run` sets state and returns
immediately; the gateway loop does the work and the result is logged as JSON when
it completes. This keeps the console responsive and avoids the shell thread ever
touching the radio.

## 6. Wire protocol

One frame type, `UWB_FRAME_TYPE_APOS = 0xEB`, with a subtype byte at index 10.
It uses the **addressed** 10-byte header the module frames already use —
`0x41 0x88 seq 0xCA 0xDE dest_lo dest_hi src_lo src_hi type` — rather than the
`WAVE`/`VEWA` literal-ident form, because the survey needs real src/dest
addressing.

| Subtype | Direction | Payload |
|---|---|---|
| `0x01 SURVEY_BEGIN` | gw → broadcast | session, window_s |
| `0x02 ENUM_RSP` | anchor → gw | eui[8], pos_valid, current x/y/z |
| `0x03 RANGE_CMD` | gw → anchor | session, peer_addr, n_exchanges |
| `0x04 RANGE_RSP` | anchor → gw | session, peer_addr, mean_mm, sd_mm, n_ok |
| `0x05 SETPOS` | gw → anchor | session, x, y, z |
| `0x06 SETPOS_ACK` | anchor → gw | session, echoed x/y/z, ok |
| `0x07 SURVEY_END` | gw → broadcast | session |

`SURVEY_BEGIN` is both the enumeration request and the thing that opens the
survey window (§6.3).

`session` is a `uint16_t` randomised by the gateway at the start of each run.
Every subtype carries it and every receiver drops a frame whose session does not
match the one it is currently participating in, so a late reply from an abandoned
run cannot contaminate a new one.

### 6.1 Staggering the enumeration replies

`SURVEY_BEGIN` is a broadcast, so every anchor would answer at once. The stagger
cannot be derived from `anchor_id` — the whole point of this design is not to
assume anything about ids — so each anchor delays its `ENUM_RSP` by
`(hash(own EUI-64) mod APOS_ENUM_SLOTS) * APOS_ENUM_SLOT_MS`. EUI-64 is unique by
construction, so the stagger is collision-free without any configuration and is
stable across reboots.

The gateway repeats `SURVEY_BEGIN` a few times and unions the replies, since a
stagger collision between two EUIs hashing to the same slot is possible and
costs only a retry.

A useful side effect: because the stagger is EUI-derived and not
address-derived, **two anchors misconfigured with the same `anchor id` both
reply** and the gateway sees one short address claimed by two distinct EUIs. That
is reported as a hard error and aborts the run — it is a fault that today
produces silently wrong ranging with no indication anywhere.

### 6.2 One peer per RANGE_CMD

Every frame stays under roughly 24 bytes, so the existing `RX_BUF_LEN` of 64 in
`uwb_slave.c` and `uwb_gateway.c` is untouched and there is no chunking logic.

The real reason is timing, not size. An anchor acting as initiator has all
interrupts disabled (`ss_initiator_enter()`) and therefore cannot see the
beacon, so its `beacon_guard` prediction goes stale for the whole batch.
Capping a command at one peer keeps each initiator batch to roughly 250 ms,
after which the anchor returns to being a plain responder and re-observes the
beacon before the next command arrives. Batching many peers into one command
would leave `beacon_guard` stale for seconds.

`n_exchanges` is a field in `RANGE_CMD`, not a constant on the anchor, so the
gateway owns the tradeoff. Default 40: at roughly 5 ms per SS-TWR exchange that
is the ~200 ms batch the paragraph above is sized around, and 40 samples make the
reported sd a usable quality signal rather than noise.

Cost is one extra frame per ~250 ms of ranging, which is negligible: N=4 is 12
directed pairs (~3 s total), N=8 is 56 (~14 s). This is a commissioning
operation run once.

#### 6.2.1 The gateway is not a survey node

The gateway is MAC-only for ranging: it never answers a ranging poll, so no
anchor can measure a distance to it and it cannot appear in the range matrix. Its
own coordinate stays hand-entered with `anchor pos`, which is harmless — the
tag's fix is solved purely from the anchors it ranges, and the gateway's position
is used only to satisfy its own refusal to beacon unpositioned.

So a deployment of one gateway plus four slaves surveys **four** nodes, not five.

### 6.3 Two independent gates on anchor TX

`CLAUDE.md` warns that a board transmitting unsolicited polls would collide with
tag ranging traffic. The production initiator is therefore gated on **both**
conditions holding:

1. A survey window is open on this board. `SURVEY_BEGIN` opens it for
   `window_s`; any subsequent APOS frame refreshes it; `SURVEY_END` closes it
   early; it auto-closes on expiry.
2. A `RANGE_CMD` for the *current* session arrived with src `0x0000`.

Every poll additionally passes through `beacon_guard_tx_allowed()`, exactly as
the two existing responders do.

### 6.4 Serialisation

The gateway commands one anchor at a time and waits for its `RANGE_RSP` before
issuing the next command. That single constraint removes every collision
question: exactly one initiator exists at any instant, and every other anchor is
in its normal responder state.

### 6.5 The `position_valid` hazard

The obvious fix for §1's silent `(0, 0)` — refuse to answer a ranging poll when
`!position_valid` — breaks the survey, because during commissioning every anchor
is unpositioned yet must answer its peers' polls. The survey window resolves it:

```
respond to a WAVE poll  iff  position_valid || apos_window_open()
```

A deployed anchor that was never surveyed then goes silent instead of lying,
matching the gateway's existing refusal to beacon unpositioned, and the survey
still works from a cold, fully unpositioned deployment.

Note the residual: while the window is open, unpositioned anchors do answer with
`x = y = 0`. That is today's behaviour, the window is bounded, and it is
operator-triggered at commissioning time when no tag is expected.

## 7. The solver

```c
struct apos_edge { uint8_t i, j; float d_m, sd_m; };

int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges,
                    uint8_t n_nodes, const struct apos_gauge *g,
                    struct apos_result *out);
```

Input is a flat edge list rather than a matrix, which is what makes holes free.

**Symmetrise.** A→B and B→A collapse into one undirected edge, inverse-variance
weighted. The weight also carries how many of the two directions succeeded.

**Seed, closed form.** `origin` → `(0,0,0)`; `xaxis` → `(d,0,0)`; `plane` →
2-circle intersection in `z = 0` with `y > 0`. `up` is placed next, from its
edges to those three, taking the `z > 0` branch — that is what its designation
means and it is the only node whose reflection is resolved by operator input.

Each remaining node is then trilaterated from its already-placed neighbours:

- **4 or more** placed neighbours, not all coplanar → unique solution.
- **exactly 3** placed neighbours → two solutions mirrored about the plane
  through those three. Pick the branch consistent with any further edge the node
  has; if it has none, pick the branch on the same side of that plane as the
  centroid of the already-placed nodes, and **mark the node reflection-ambiguous
  in the report**. An ambiguous node is placed (so the fit can proceed) but its
  flag is part of acceptance: a run with any ambiguous node needs either another
  measured edge or a repositioned anchor.
- **fewer than 3** → left unplaced and named in the report. That is a site
  problem the operator needs told, not a solver failure.

Placement order is by descending count of placed neighbours, re-evaluated after
each placement, so the best-determined nodes go first and later nodes inherit a
better-conditioned seed.

**Refine.** Levenberg–Marquardt over the free parameters — `3N − 6` of them,
since the gauge pins 3 + 2 + 1 — minimising `Σ w·(‖pᵢ−pⱼ‖ − d)²`. LM rather
than plain Gauss-Newton because near-coplanar geometry is exactly where the
normal equations go ill-conditioned; damping makes it report rather than
diverge.

**Diagnostics.** RMS edge residual; worst single edge and which pair it was;
per-node residual; and a **planarity metric** — the RMS distance of placed nodes
from their own best-fit plane. The planarity figure is what tells the operator
whether the solved z values mean anything: a ceiling-mounted array will show a
small figure and the report says so plainly.

**Acceptance.** Every node placed; RMS residual under threshold (starting at
50 mm, given the antenna-delay cross-check accepts 30 mm per pair); worst edge
under roughly 3× RMS; z flagged untrustworthy when planarity falls below a few
times the range noise.

### 7.1 Survey capacity is independent of the ranging MAC

```c
#define APOS_MAX_NODES 8   /* survey capacity — NOT UWB_MAX_ANCHORS */
```

The survey has no reason to inherit the tag-facing MAC's limit of 4. Keeping
them separate makes the real ceiling explicit rather than hidden: growing the
deployment past 4 ranging anchors requires re-deriving `disc_schedule.h`'s
stagger and `anchor_respond.c`'s `TX_COMPLETE_TIMEOUT_MS` (currently 18 ms,
sized for `disc_resp_delay_uus(3) = 12500` uus; anchor id 5 would need 19.5 ms
and would silently lose every DISCOVERY response), and raising the tag's
`UWB_FRAME_MAX_ANCHORS`. Those are separate work; this design does not do them,
but it does not add to them either.

## 8. Operator flow

Report first, commit second — the discipline `cal ref` / `cal peer` already
established:

```
apos enum                                   discover; print addr + EUI + pos_valid
apos gauge origin=0x0001 xaxis=0x0002 plane=0x0003 up=0x0004
apos zoff <m>                               optional: move z=0 to the floor
apos run                                    range everything, solve, REPORT ONLY
apos apply                                  push SETPOS to every anchor, persist, publish
apos ref <lat> <lon>                        geo reference for the origin anchor
apos show                                   stored survey table as JSON
```

`apos run` never writes anything, on any board. Nothing persists until
`apos apply`, and `apply` refuses a run that failed acceptance unless explicitly
forced.

## 9. Persistence

**Anchor.** `SETPOS` → `uwb_config_set_pos()` + `uwb_store` save, then
`SETPOS_ACK`. `uwb_config_t` already carries `float x, y, z`, so the config
struct is unchanged. The gateway retries an anchor that does not ACK and reports
any that never do — a half-applied survey is the one outcome worth being loud
about.

**Gateway.** `apos_store` persists the table under a new `apos/` settings
subtree, keyed by **EUI-64**. This is what fixes §1's bench failure:
coordinates are bound to boards, so an `anchor id` swap can no longer strand a
coordinate on the wrong box.

## 10. MQTT

### 10.1 The anchors payload stops being a stub

`pos_json_anchors()` becomes driven by the survey table, falling back to the
existing stub only when no survey exists. Names derive from short address
(`ANC-<zone>-%03u`), and the origin anchor carries the operator-supplied
lat/long from `apos ref` — which is why that command exists: without it the
platform has nothing to place the local frame against. Map and solver then agree
by construction.

The position payload (`{"Tid":…,"x":…,"y":…,"z":0}`) is **not** touched. It is a
fixed contract with the downstream consumer and the tag remains 2D.

### 10.2 Trigger stub

`net_uplink.c` has no subscribe path — only `MQTT_EVT_CONNACK`,
`MQTT_EVT_DISCONNECT` and `MQTT_EVT_PUBACK`. The trigger is therefore a stub,
labelled as one:

```c
/* Unwired: net_uplink.c does not subscribe to anything yet. */
int apos_gw_trigger_from_mqtt(const char *payload, size_t len);
```

It parses and validates a trigger document and returns. A reserved topic
constant sits next to the existing two in `pos_json.h`, composed from
`POS_JSON_ZONE_NAME` so a topic can never disagree with its payload. Wiring a
subscribe path is later, separate work.

## 11. Why the tag is not changed

The VEWA response the tag reads carries only x and y
(`anchor_respond.c`: `POS_ANCHOR_X_IDX 19`, `POS_ANCHOR_Y_IDX 23`). There is no
z field on the wire, so the tag cannot consume a surveyed z without a
coordinated wire-format change across two firmware projects plus the fixed
`pos_json` consumer contract.

The survey is 3D anyway because z is immediately useful without any of that: for
the coplanarity diagnostic, for the platform map, and for correcting slant
ranges inside the survey itself. Extending VEWA and the tag's solver later
becomes a clean follow-up with no rework, since the anchors will already know
their z.

## 12. Testing

Host tests, plain gcc, no Zephyr — the pattern the other pure-C modules follow:

```
tests/apos_frame/   round-trip all 7 subtypes; truncation; wrong type; bad session
tests/apos_geom/    synthetic layouts (square, L-shape, near-coplanar,
                    sparse-with-holes); noise injection; gauge correctness;
                    unplaceable-node detection; planarity metric; LM convergence
tests/apos_table/   edge accumulation; symmetrisation; dedup; acceptance thresholds
```

The solver can be developed to completion with zero hardware, which matters
given how long the antenna-delay hardware gate has taken.

## 13. Hardware gates

A new `docs/anchor-auto-positioning.md`, in the style of
`docs/antenna-delay-calibration.md`, lists these in the order a human hits them:

1. `apos enum` finds every anchor present, with correct EUI and short address.
2. One commanded pair ranges, returning a plausible mean and a small sd, and the
   commanded anchor returns to responder duty afterwards.
3. A full `apos run` over 3 anchors at tape-measured spacing meets the
   acceptance thresholds of §7.
4. `apos apply` reaches every anchor, and the coordinates survive
   `kernel reboot cold` on each one.
5. The payoff: a tag's `0xEA` residual against surveyed coordinates is no worse
   than against carefully hand-entered ones, and `(x, y)` is stable between
   consecutive fixes.
6. `uwb/anchor/setup/<zone>` carries the surveyed geometry, not the stub, and
   the platform draws the deployment.

Gate 5 depends on the antenna-delay calibration having been completed first —
an uncalibrated array cannot distinguish a survey error from a delay error.

## 14. Implementation phasing

The subsystem is large enough that it should not land as one change. The natural
decomposition, each phase independently verifiable:

1. **`apos_geom` + `apos_table`, host-tested only.** No radio, no Zephyr, no
   frames. Ends with the solver passing synthetic 3D layouts including
   near-coplanar and sparse cases. This is the largest and riskiest piece of
   logic and it needs no hardware, so it goes first.
2. **`apos_frame`, host-tested.** Codec round-trips for all seven subtypes.
3. **`ss_initiator` extraction.** Promote `cal_initiator.{c,h}` to the shared
   file set with no behaviour change; the calibration image must still pass its
   own procedure afterwards.
4. **`apos_node`** — survey window, `ENUM_RSP`, one commanded range batch,
   `SETPOS` persistence — plus the §6.5 `position_valid` gate. Verifiable with a
   sniffer and one hand-crafted command.
5. **`apos_gw` + `apos_shell` + `apos_store`.** The orchestration, the console
   tree, and persistence. Ends at hardware gate 4 of §13.
6. **`pos_json_anchors()` rework + the MQTT trigger stub.** Ends at gate 6.

Phases 1 and 2 can proceed in parallel with nothing else; 3 gates 4; 4 gates 5;
6 depends only on 5.

## 15. Out of scope

- DS-TWR between anchors (§4). Deferred behind `apos_table`'s edge interface.
- A z field in the VEWA response and a 3D tag solver (§11).
- Wiring an MQTT subscribe path (§10.2).
- Raising the deployment past 4 ranging anchors (§7.1).
- Re-surveying automatically on a schedule. The survey is operator-triggered.

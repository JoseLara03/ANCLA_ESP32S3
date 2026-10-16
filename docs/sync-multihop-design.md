# Static sync links and the multi-hop contract

Written 2026-10-12 with the implementation of
`docs/superpowers/plans/2026-10-12-b14-static-survey-multihop.md`. This is the
**as-built** description plus the contract a future multi-hop sync tree must
respect. Operator procedure is in `docs/static-survey-install.md`.

**Status: implemented, host-tested and build-verified (production and
calibration images). NOT run on hardware** — every gate in
`docs/static-survey-install.md` §6 is `NO EJECUTADO`.

## 1. What changed, in one paragraph

The gateway (CCP root, short address `0x0000`) is now a **node of its own 3D
survey**. `apos run` ranges it against every anchor, solves XYZ for five nodes
with a gauge made of **four anchors only**, and derives for each anchor one
static flight-time compensation `tau_dtu = round(distance_3d / 4.690 mm)`.
`apos apply` installs coordinates **and** that link on every anchor, and only
after every anchor acknowledged does the gateway persist the result and resume
publishing fixes. The automatic TWR round-robin is **gone**; `sync tau measure`
survives as a manual diagnostic that applies nothing. The CCP, the rate/phase
estimate and the B14 feed-forward are unchanged.

## 2. Decisions this rests on (operator-confirmed 2026-10-12)

| ID | Decision | Consequence in the code |
|---|---|---|
| D1 | Heights are solved automatically, 3D survey. | `apos gauge` needs all four of `origin xaxis plane up`; no manual-Z mode, no 2D fallback inside the install flow. |
| D2 | Gauge is anchors only. | `apos_gw_set_gauge()` refuses `0x0000` in any slot. The gateway is placed as the 5th node. |
| D3 | Survey and manual measurement only; no automatic TWR. | `gw_selftau` auto path, its CCP v2 payload, the anchor-side filter and the 60 s freshness degrade are removed. `sync tau auto on` is refused. |

The historical **2D survey** (`apos gauge ... ` without `up=`) is kept for
compatibility and behaves exactly as before: anchors only, no gateway node, no
links, no hold. It cannot produce the static install.

## 3. Deliberate deviation from the plan: one-way gateway edges

The plan asks to measure gateway↔anchor in **both** directions. This build
measures **gateway → anchor only**. Reasons:

* The reverse direction needs the gateway to answer an anchor's poll with a
  delayed TX from the loop that owns the beacon: a new production responder
  path with its own beacon-collision hazard, on the one board whose timing
  everything else derives from.
* SS-TWR antenna-delay error is **exactly reciprocal** (swap initiator and
  responder: identical expression), so a second direction cancels none of it.
  It would only cancel residual clock-offset error.
* The fit's redundancy is unchanged: 5 nodes, 10 edges (6 anchor–anchor
  averaged both ways, 4 gateway edges one-way) against 3·5−6 = 9 free
  parameters — one spare edge, so `rms_meaningful` is true.

A gateway edge therefore carries `apos_table`'s one-way sd inflation
(`APOS_ONEWAY_SD_INFLATE`). If a later campaign shows the clock-offset term
matters on these edges, the responder path is the engineering work to schedule.
**The gateway must itself have passed `cal ref`**: its TX delay is common to all
four of its edges and would otherwise scale into every tau.

## 4. Data model

### 4.1 Identity

Everything persistent is keyed by **EUI-64**, never by short address or array
index. A node record is `{eui, short_addr, x, y, z}`; the role is derived from
`short_addr == 0x0000` (gateway). The survey blob (`apos/survey`) is **unchanged
byte for byte** — a board upgraded in place keeps its survey.

### 4.2 Links (`apos/links`, versioned codec `src/apos_links.c`)

Little-endian, fixed offsets, no struct padding:

```
[0] version (1)   [1..4] generation u32   [5] n (<= 8)
n x 22 bytes: child EUI(8) parent EUI(8) tau_dtu i32 valid u8 source u8
```

* `valid` is explicit: `tau_dtu == 0` is a legal calibrated value and must never
  be read as "absent".
* An unknown version, a length that disagrees with `n`, trailing bytes or a
  non-0/1 `valid` is **rejected, not reinterpreted, and not erased**; links read
  as absent (`needs_survey`) and the console says why. A rollback of this
  firmware therefore does not destroy the install.
* `source`: `1` = `survey_static`. In this delivery every `parent` is the
  gateway and the tree is one level deep; the field exists so a later parent
  needs no format change.

### 4.3 Apply-in-progress marker (`apos/pending`)

`{version, generation, state}`. Persisted **before the first anchor is
touched**, cleared only after every anchor acknowledged and the gateway's own
survey and links are on flash. An unreadable marker is treated as *interrupted*:
"cannot tell" must read as "it was".

### 4.4 Anchor side (`anchor/sync_link`, 13 bytes)

`valid(1) generation(4) parent EUI(8)`, beside the existing `anchor/sync_tau`.
`uwb_config_t.sync_link_valid` is the only thing that says a survey installed
the tau; `needs_survey` is what an anchor reports without it. A stored
`sync_tau_mode` of `twr`/`auto` (retired) is migrated to `survey` in RAM on load,
keeping the stored tau, with a warning.

## 5. Wire: APOS `SETTAU` / `SETTAU_ACK`

Subtypes `0x08` / `0x09` of type `0xEB`, same 10-byte addressed header.

```
SETTAU      (30 B)  session u16, version u8 (=1), generation u32,
                    tau_dtu i32, parent EUI[8]
SETTAU_ACK  (22 B)  session u16, generation u32, tau_dtu i32, status u8
status: 0 ok | 1 applied in RAM but NVS write failed | 2 unsupported version
```

* Exact-length parse, like every other subtype.
* **Older anchor firmware drops the subtype** (`apos_frame_is_apos()` is false
  for an unknown subtype), never ACKs, and the gateway reports that anchor by
  address as *incompatible firmware or out of range, still on its OLD link*. An
  incompatible anchor is an explicit error, not a silent half-apply.
* The ACK reports what the anchor **holds**. The gateway accepts it only if
  status is 0 **and** generation **and** tau echo what was sent; anything else is
  a failure and is not retried (a retry would restate the same command to a
  board that answered clearly).
* Idempotent on the anchor: the same `(generation, tau, parent)` twice leaves
  the same state, because the gateway retries on a lost ACK and cannot tell
  "never arrived" from "arrived, ACK lost".
* Applied **live** (`ccp_slave_set_tau()`), persisted in the order **tau →
  mode → link record**. A power cut part-way leaves the old generation recorded
  against a possibly-new tau, which the gateway reads as a mismatch and retries;
  the reverse order would let a new generation vouch for a tau that never landed.

## 6. The apply sequence and what it guarantees

```
MARK     persist marker, hold fixes             (flash write, gated on 150 ms of clear air)
REOPEN   SURVEY_BEGIN so every window is open
SETPOS   one per anchor, each ACKed            (gateway is never commanded)
SETTAU   one per anchor, each ACKed + verified
COMMIT   all ACKed?  yes: save survey, save links, clear marker, release hold
                     no : keep PREVIOUS survey+links, keep marker, KEEP the hold
END      SURVEY_END, summary line
```

* **No atomicity is claimed.** Anchors that took the new generation are not
  rolled back. Recovery is `apos apply` again: it restates everything and uses a
  **new, monotonic generation** (an earlier failed attempt already stamped some
  anchors with the old one, so it is never reused).
* **Fixes are suspended** (`apos_gw_publish_hold()`) from MARK until a complete
  commit, across reboots (marker on flash). `tdoa_gw` drains and counts
  observations (`held` on `blink stats`) instead of solving them, and discards
  groups collected before the hold. Nothing is queued behind the hold — that would
  replay stale observations the moment it lifts. Tags still in classic TWR mode
  publish their own `0xEA` fixes, which involve no tau and are not held.
* If the marker cannot be written, the apply is abandoned **before touching any
  anchor** — the one failure that costs the operator nothing.
* If every anchor ACKed but the gateway cannot write its survey/links/marker
  clear, the hold stays and the previous survey stays active.
* A flash write on the gateway loop stalls all execution while the sector
  erases; the three-write COMMIT is therefore gated on the same 150 ms budget the
  solve uses. This is not a hard bound — see CLAUDE.md's flash-stall fact.

## 7. Gauge conditioning

"Non-zero volume" is not a test: ranging noise alone makes a coplanar gauge
reconstruct with an apparent height of about `sqrt(2·L·δ)` (L gauge size, δ edge
error) — for L = 2 m, δ = 30 mm that is ~0.35 m, so a flat install can read as a
tall one. `apos_geom_gauge_conditioning()` therefore requires

```
collinearity >= 0.10            (|xaxis × plane| / |xaxis|²)
up_height    >= 2.0 · sqrt(L · max(measured sd, 30 mm))
```

`2.0` was chosen with `tests/apos_gauge_quality/` against the real solver: 0 of
240 noisy coplanar gauges (30 mm edge noise, gateway 1.4 m off the plane)
accepted, 2 of 387 gauges with `up` 0.8 m high refused. Both failures are
**refusals that `apos apply force` cannot override**: the frame's z and mirror
side would be noise, and a gateway off the plane does not rescue it because the
gateway is excluded from the gauge by decision D2.

Mount `origin`, `xaxis` and `plane` in one (preferably horizontal) plane and `up`
clearly off it. **+Z points toward `up`.** Solved z is *relative to that plane*,
not a measured height above the floor: a tilted plane gives a tilted frame and
`apos zoff` (a translation) cannot fix a tilt. `apos tagz` is in the same frame.
The TDoA solver stays 2D with a configured tag height; solving the survey in 3D
does not change that.

## 8. Static tau is not clock validity

`tau` removes the master→anchor flight bias. It says nothing about whether the
anchor's clock model is **fresh**. `sync_model_to_master()` keeps extrapolating a
converged model for as long as no CCP arrives to count a gap, so a silent link
used to keep producing "valid" conversions indefinitely.

`blink_rx` now refuses to convert unless the last accepted CCP is at most
`SYNC_CLOCK_MAX_AGE_MS` old — the existing holdover policy (`SYNC_MISS_MAX` = 25
superframes) expressed in time: **5000 ms**. Uptime, not the DW3220 clock
(which wraps every 17.2 s and could not tell 12 s of silence from 5). Refusals
count in `blink stats` `stale` (also in `no_sync`, so `no_sync == rx` still reads
as "CCP link down"). The old 60 s TWR freshness window is not a clock-validity
figure and is not used. The static link itself **does not expire by time**; it is
invalidated by changing the equipment, the parent, the geometry or the
installation configuration (a hand `sync tau set` clears the link record).

## 9. Contract for a future multi-hop sync tree (NOT implemented)

None of this is built. It is the contract, so the data above does not have to
change when it is.

1. A sync tree has **one identified root, explicit parents and no cycles**. It is
   distinct from the observation transport network and from the survey graph.
2. Each child compensates **only its own parent→child flight**. The parent
   announces its **real** TX instant expressed in root time. Never add the
   ancestors' flight again.
3. Keep announcing **confirmed** TX timestamps (the deferred-timestamp scheme);
   do not replace them by planned times without proving equivalence.
4. A relay needs the validity of its upstream model **and** of its own TX
   reference. The RX-delay cancellation shown for a receiving anchor does not by
   itself prove a relay's TX accuracy — calibrate and validate that separately.
5. A new parent requires a calibrated link and a rebuilt model **before** any
   observation is accepted. Static geometry is not permission to keep using a
   stale clock (§8).
6. Never mix observations from different roots or incompatible time epochs. The
   future protocol must carry the identifiers; reserving fields now does not make
   the current wire guarantee it.
7. A global survey needs rigidity and shared references. A chain of parent–child
   distances does not determine coordinates; cells cannot choose independent
   gauges and publish them as one frame.
8. Upstream noise/covariance and CCP TX/RX windows need their own budget.
   Removing periodic TWR does not by itself prevent relay collisions.

## 10. Limits and things deliberately left alone

* One-way gateway edges (§3).
* No rollback of anchors after a partial apply; recovery is re-apply (§6).
* A node-count ceiling: the survey holds 8 nodes, so 8 answering anchors leave no
  room for the gateway and the run says so. (More than 4 *ranging* anchors is a
  separate, larger piece of work — see CLAUDE.md.)
* The stored survey blob has no schema version (left alone on purpose); only the
  new records are versioned.
* `tau_filter` source and tests remain in the tree, unbuilt.
* Out of scope: local server/broker, queue/solver optimisation, more tags, PHY or
  superframe changes, TWR→DS-TWR, RX-split calibration, BLINK NLOS, the gateway as
  a TDoA receiver, executing multi-hop.

# Static survey install — operator procedure and acceptance record

Companion to `docs/sync-multihop-design.md` (what and why) and
`docs/anchor-auto-positioning.md` (the survey tool this extends). This file is
the runnable procedure, the T0 baseline record and the hardware acceptance
checklist.

**Nothing here has been run on a board.** Every row of §6 is `NO EJECUTADO`.
Host tests and builds (§7) say the code is internally consistent, not that it
works.

## 0. T0 — baseline and disposition (2026-10-12)

| Item | Record |
|---|---|
| Base | `fix/b14-flight-compensation` @ `da3bfab` (docs: Gate B item 1 — 3 of 4 anchors pass concordance, `0x0004` open) |
| Branch | `codex/b14-static-survey`, cut from the base above. The old `feat/rtls-scale-tdoa` checkout is not the base. |
| Stash `stash@{0}` (`codex-preserve-before-tdoa-review-2026-10-12`) | **Reviewed, not applied, not dropped.** Touches `blink_shell.c`, `gw_core.{c,h}`, `tdoa_gw.{c,h}`, `tdoa_solve.c`, `tests/gw_core` (+350/−175): a TDoA solver / jump-gate / gw_core change set independent of this work. It overlaps three files this branch edits (`blink_shell.c`, `tdoa_gw.{c,h}`), so applying it later is a manual merge, not a `stash pop`. |
| Other stashes | `stash@{1}` (phy-256-6m8), `stash@{2}` (tdoa-accuracy-filter, TWR rollback) — unrelated, untouched. |
| Fleet state before | Four anchors with `cal ref` done; tau installed **by hand** with `sync tau set` / `sync tau mode survey|twr|auto`; gateway running the TWR auto round-robin (default ON); retained anchors payload with surveyed coordinates. **Record each board's `anchor show` and `sync tau show` and the output of `apos show` before flashing** — that is the rollback data (§5). |
| Position baseline | Not re-measured by this change. Use the Gate A numbers in CLAUDE.md (0.293 m / 0.208 m at two laser-verified points) only as history; take a fresh capture in the cell being installed. |

### Physical layout and the height convention (record this before installing)

* `origin`, `xaxis`, `plane`: three anchors **in one plane**, not collinear
  (ratio |plane.y|/xaxis.x ≥ 0.10, i.e. not a thin sliver).
* `up`: the fourth anchor **clearly off that plane**. The check is
  `up_height ≥ 2·sqrt(L · max(sd, 30 mm))` with L the RMS gauge edge — about
  **0.65 m for a 3×3 m gauge**, more for a larger one. Four anchors at one
  height **cannot** pass, and the gateway being off the plane does not help
  (decision D2).
* **+Z points toward `up`.** For +Z to be vertical-up, mount the three plane
  anchors on one horizontal plane and put `up` **above** it. Ceiling anchors at
  one height plus a lower `up` give a frame whose +Z points down: legal, but then
  tags below the plane have **positive** z and `apos tagz` must be positive.
  Whatever you choose, write it here for the site:

  | | |
  |---|---|
  | Site / zone | |
  | origin / xaxis / plane / up (anchor ids) | |
  | Plane horizontal? (spirit level / laser) | |
  | +Z points | up / down |
  | `apos zoff` to put z = 0 on the floor | |
  | `apos tagz` for the tag plane | (unmeasured until someone measures it) |

## 1. Prerequisites

1. All of `docs/anchor-auto-positioning.md` §1, and **the gateway itself has
   passed `cal ref`** — its TX delay is common to all four gateway edges and
   scales into every tau.
2. Gateway and all four anchors on this firmware. **Deploy order:
   gateway → anchors**, in one maintenance window: an old anchor drops `SETTAU`
   and the apply reports it as incompatible, and an old gateway never sends it.
3. Anchors mounted in their final positions; the gateway in its final position
   (moving the gateway afterwards invalidates every tau — §5).
4. `anchor mode gateway` on the gateway, `kernel reboot cold` after.

## 2. Procedure

```
apos enum                                       # ids and EUIs; all four must answer
apos gauge origin=0 xaxis=1 plane=2 up=3        # four ANCHORS, never the gateway
apos zoff <m>        # optional: move z = 0 (a translation; cannot fix a tilt)
apos run
apos show
```

`apos run` takes a few seconds per anchor pair plus 30 gateway exchanges per
anchor (up to 8 per step, bounded by the time left before the beacon, so a few
superframes each). It reports as JSON and **persists nothing**. Read, in this order:

1. `{"apos_gauge":{...}}` — `coplanar:0`, `collinear:0`. If either is 1 the
   candidate is **refused and `apos apply force` will not install it**; re-mount.
2. `{"apos_link":{...}}` — one per anchor. `checked:1` and `within_gate:1`
   (`|err_mm| ≤ 150`, the 0.5 ns Gate B heritage). `meas_mm` is the gateway's own
   range to that anchor, `geom_mm` the solved distance. This is evidence for the
   geometry, **not** an independent check of it: both come from the same
   ranging. Confirm the node-to-node distances **and the gateway distances**
   against a tape or laser before applying.
3. `apos_solve` — `spare_edges:1`, `rms_meaningful:1` on a full five-node mesh. A
   missing edge makes it `0`; the unverified-mesh warning then applies.
4. `max_reciprocal_mm` / `max_sd_mm` as before.

```
apos apply
```

Fixes are **suspended from this point** until the apply completes. Watch for
`apos_apply_done`: `failed:0`, `persisted:1`, `tau_ok:4`, `hold:0`.

```
apos ref <lat> <lon>     # the ORIGIN anchor's real-world position, as before
kernel reboot cold       # on the gateway, then check `apos show`: generation,
                         # links "installed", hold "none"
```

On each anchor: `sync tau show` → `"link":"installed"`, the same `generation`,
`"parent"` = the gateway's EUI (`apos show` prints it), `tau_source`
`"static-survey"`.

## 3. Recovery

| Symptom | Meaning | Do |
|---|---|---|
| `0x000N never acknowledged SETTAU … incompatible firmware … OLD link` | Anchor is out of range, off, or on firmware that predates `SETTAU` | Power/flash it, **re-run `apos apply`** (idempotent) |
| `blink stats` `"hold":"apply-incomplete"`, `held` climbing | Some anchor did not take it, or the gateway could not persist | Fix the cause, `apos apply` again. Fixes stay suspended until one completes. |
| `"hold":"interrupted-apply"` after a reboot | The gateway died or lost power between the marker and the commit; anchors may hold a mix of generations | `apos run` then `apos apply`. RAM state is gone, so it cannot resume — it restarts. |
| `apos apply` → `REFUSED — the four gauge anchors do not span a 3D frame` | Coplanar or collinear gauge | Re-mount (§0). `force` does not help. |
| `apos apply` → `a sync link could not be derived …` | An anchor the solve left unplaced, or a gateway pair never measured | Read `apos_hole` / `apos_thin`, fix line of sight, re-run |
| `"links":"needs_survey"` in `apos show`, `"link":"needs_survey"` on an anchor | No survey installed this link (an upgrade in place, or a hand `sync tau set`). The stored tau is **not** a calibrated value; `0` means unset | Run the procedure |
| `"links":"unreadable"` | A stored links record has an unknown version or is corrupt; **it was not erased** | Re-run the procedure (a rollback to the matching firmware also reads it) |
| `blink stats` `stale` climbing, `no_sync == rx` | No CCP for > 5 s: the CCP link is down. BLINKs are being refused, correctly | `sync stats`, check the gateway is on USB-C (CLAUDE.md, battery/CCP fact) |

## 4. Console surface added or changed

```
apos gauge origin= xaxis= plane= up=     all four required for the static install;
                                          omitting up= is the historical 2D survey
                                          (no gateway node, no links, no hold)
apos show                                 + gateway EUI/in_survey, hold, generation,
                                            links installed|needs_survey|unreadable,
                                            gauge quality, candidate links
apos apply [force]                        force overrides thresholds, never a
                                          degenerate gauge and never a missing link
blink stats                               + stale, held, hold (reason)
sync tau show                             link, generation, parent, clock_fresh,
                                          clock_age_ms; tau_source static-survey|
                                          unsurveyed|legacy
sync tau mode <legacy|survey>             twr/auto are refused (retired)
sync tau set <dtu>                        still works; clears the link record
sync tau auto [off]                       prints "retired"; `on` is refused
sync tau measure <id> [n] / result        manual diagnostic only; applies nothing
```

## 5. Rollback and what moving equipment means

* **Binary rollback** to the B14 build restores its behaviour **including the
  TWR auto round-robin ON by default**, and it reads `anchor/sync_tau*` as it did.
  The new keys (`anchor/sync_link`, `apos/links`, `apos/pending`) are ignored by
  the old build and are not erased by it. Restore firmware **and** the exported
  configuration from the same generation (§0 records what to export).
* **Do not reuse geometry measured after a physical move together with an older
  tau.** A moved gateway or anchor needs `apos run` + `apos apply` again; nothing
  tracks it automatically any more. To isolate a gateway move from an anchor
  move, keep the anchors fixed and move only the gateway.
* The new firmware **never re-enables the periodic TWR** even if restored data
  says `twr`/`auto`; that value is migrated to `survey` with a warning.

## 6. Hardware acceptance (plan §7) — all NO EJECUTADO

Record PASS / FAIL / NO EJECUTADO with the log file name and the configuration.

| # | Gate | Result | Evidence |
|---|---|---|---|
| 1 | Fresh install: gateway + 4 anchors, no prior positions, 3D survey with a 4-anchor non-coplanar gauge; edges and XYZ vs independent measurement. Also: 4 coplanar anchors + gateway off the plane **are refused** and not applied | NO EJECUTADO | |
| 2 | Identities: change boot order and one board's `anchor id`; coordinates and tau follow the EUI. Replacing a board requires re-validation | NO EJECUTADO | |
| 3 | Persistence: reboot all five; same generation, source and tau. 10 min idle: no autonomous TWR, no degradation | NO EJECUTADO | |
| 4 | Timing: beacon/CCP during survey and during operation; zero missed deadlines attributable to the new path; report duration and load. **In particular the local-ranging bursts (`apos_local_range`) and the three-write COMMIT against `BEACON_ARM_MARGIN_UUS`** | NO EJECUTADO | |
| 5 | Positioning: fixed geometry, two FAST tags; before/after position, fix rate, interval p50/p95/max, rejections, per-anchor sync. Same network and broker. Do not claim the old 0.35 m without measuring in the new cell | NO EJECUTADO | |
| 6 | CCP loss while BLINKs keep arriving: timely refusal (`stale`), counter, clean recovery | NO EJECUTADO | |
| 7 | Failure during apply: an absent anchor and a reboot mid-apply; recovery, no mixed generations, previous configuration kept | NO EJECUTADO | |
| 8 | Deliberate physical change: manual recalibration restores geometry/tau; no automatic tracking expected | NO EJECUTADO | |
| 9 | Manual diagnostic: `sync tau measure` does not change the active link | NO EJECUTADO | |

Gate B heritage still applies to each link: tau within 0.5 ns (~32 DTU, ~150 mm)
of an **independent** reference (laser). The in-run `within_gate` flag is not
that reference.

## 7. What was verified without hardware (2026-10-12)

Host suites (all pass): `apos_geom`, `apos_gauge_quality` (Monte-Carlo gauge
conditioning), `apos_table`, `apos_frame` (SETTAU/ACK raw bytes, lengths),
`apos_links` (round-trip, every truncation, unknown version, corruption),
`apos_stats`, `sync_link` (known distances, heights, rounding, gate, unplaced,
one-way vs round-trip), `sync_model` (clock-age boundaries incl. uptime wrap),
`uwb_config` (retired modes, link record), `pos_json` (gateway omitted, comma
placement, single reference anchor), `tau_filter` (still passes, unbuilt), plus
the pre-existing suites.

`tests/apos_gw/` runs the **real `apos_gw.c`** against a fake radio, fake
anchors, fake NVS with injectable write failures and a fake clock: ordering
(marker on flash before the first SETPOS; hold held through SETPOS/SETTAU;
released only at the end), lost / duplicated / stale / wrong-generation /
wrong-status ACKs, an anchor that never answers (previous survey kept, marker
set, hold kept; re-apply uses generation 2), reboot with a pending or unreadable
marker, marker write failure (abandoned before touching anything), links write
failure, coplanar gauge (`-EDOM` even with `force`), unverified link (`-EPERM`),
no room for the gateway, the 2D flow unchanged, gateway refused as a gauge
reference, enumeration order not changing identities. It was **mutation-checked**:
12 deliberate defects in `apos_gw.c` (no hold, commit despite failure, each stage
guard, gateway commanded, ACK content unchecked, force overriding the gauge
refusal, marker failure ignored, generation reuse, anchor→gateway `RANGE_CMD`,
boot ignoring the marker, acceptance ignoring gauge/links) are each caught.

Both images build clean (production `dram0_0_seg` 273264 B, calibration 79208 B;
`net_uplink.c`, `blink_shell.c` and `tdoa_gw.c` compile into the calibration
image with networking off). **Stack and RAM headroom on the 4096-byte main stack
are not measured** — `CONFIG_THREAD_ANALYZER` (`debug.conf`) is the only thing
that answers it, and the new COMMIT path (survey + links + marker writes) and the
solve are the two deep chains to watch.

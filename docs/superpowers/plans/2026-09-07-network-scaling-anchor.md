# Network scaling (protocol v3) — ANCHOR / GATEWAY implementation plan

> **For agentic workers:** steps use checkbox (`- [ ]`) syntax for tracking. Implement
> task-by-task, in order; each task ends in a state that builds and tests clean.

> **Status ledger note (2026-09-08):** the SDD progress ledger this plan's checkboxes
> were meant to track
> (`.superpowers/sdd/2026-09-07-network-scaling-anchor/progress.md`) does not exist in
> this repo — lost or never committed. Checkboxes below were reconstructed from `git log`
> and this session's own work, not from that ledger. **`git log` is the source of truth**
> for what has actually landed; if a checkbox here ever disagrees with the log, trust the
> log and fix the checkbox.

**Design doc:** `../../../tag_testting/spec/2026-09-07-network-scaling-design.md` — read
it first. This plan implements the anchor/gateway half and does not restate the
rationale.
**Tag half:** `../../../tag_testting/plan/2026-09-07-network-scaling-tag.md`. The two
plans are phase-locked: a phase is only air-testable when both sides have finished it.

**Goal:** one PAN serving 100 tags over 32 anchors with 100 m links. Today: 4 anchors
hard-capped in five places, ~7 tag seats, and a transmit chain measuring ~25 dB below
free-space expectation.

**Tech stack:** C, Zephyr 4.4.x, ESP32-S3 + DW3220. Host unit tests with plain gcc, no
Zephyr, per the existing pattern.

## Global constraints

- `$env:ZEPHYR_BASE` is required for every build (see `CLAUDE.md`).
- **`src/uwb_frame_802_15_4z.{c,h}` is a byte-for-byte copy of the tag's.** Never edit it
  here: the tag repo's task lands the change, this repo copies the file and re-runs
  `tests/uwb_frame/` with the tag's own suite. Same rule for `src/cal_math.{c,h}`.
- **The GATEWAY loop runs at `K_PRIO_COOP(0)`.** Every new busy-wait must be bounded, and
  `apos_gw_step()`'s "at most one frame per call, never block" rule extends to every new
  step machine added here.
- **Any flash write stalls all execution** (XIP). No new NVS write may happen on the
  gateway loop outside the existing budget gate.
- **`TX_COMPLETE_TIMEOUT_MS` is derived from the worst-case scheduled delay**, and there
  are three unrelated constants with that name (`anchor_respond.c` 18, `apos_gw.c` 8,
  `apos_node.c` 8). Each task that changes a stagger must re-derive the one it affects
  and say which.
- **A board that answers before it is positioned is worse than a silent one.** Keep
  `anchor_respond_wave_poll()`'s `position_valid` refusal on every new response path.
- `proto_ver` -> 3 happens once, in Task 21, on both repos in the same commit.

---

## Phase 0 — measurements that gate everything (no code)

### Task 1: Root-cause the ~25 dB transmit deficit

- [ ] Re-run the sniffer measurement recorded in `CLAUDE.md` (beacon at 0.5 m read
      -84 dBm against ~-57 dBm predicted) with a controlled before/after on
      `dwt_setfinegraintxseq(0)` + `dwt_setlnapamode()`, same positions, firmware toggled.
      That A/B was never run and its contribution is unknown.
- [ ] Check the obvious hardware suspects in order: PA enable line actually asserted
      across the frame (scope on EXTTXE), antenna feed/matching, and supply sag at TX
      (the "one frame per boot then silent" board was a supply symptom).
- [ ] Compare against a DWM3001CDK transmitting the same frame at the same distance —
      that is the reference for "what a correct board looks like".
- [ ] **No range work starts until this closes.** Design §2.3: at -25 dB the 100 m budget
      closes at ~6 m, which is exactly the observed bench behaviour.

### Task 2: Measure the turnaround floor and the RX sensitivity

- [ ] With the SPI bus at 26.67 MHz, measure RX-to-`dwt_starttx()` on the WAVE path and
      find the real floor for `POLL_RX_TO_RESP_TX_DLY_UUS` (currently 2000, sized for a
      2 MHz bus; `disc_schedule.h` records ~180 us of RX-side work at the fast rate).
      Do not change it yet — Task 15 does, in lockstep with the tag.
- [ ] Measure anchor RX sensitivity as built (no LNA on this board) with a stepped
      attenuator, at PLEN-1024 / 850 kbps. This is the receiving end of the weak link and
      the datasheet number is not the board's number.

---

## Phase 1 — 32 anchors

### Task 3: Lift the deployment cap — **done** (`2dfe282`, `9fc3685`)

- [x] `src/uwb_config.h`: `UWB_MAX_ANCHORS` 4 -> 32. Add a comment naming the **wire cap
      of 254** (`UWB_ANCHOR_ADDR_BASE + id` must stay below `0x0100` so the poll's
      one-byte id stays unique) and that 32 is the supported number.
- [x] `src/anchor_shell.c`: `anchor id <0..31>`, and the help text with it.
- [x] `tests/uwb_config/`: accept 31, reject 32; the short-address mapping stays
      `0x0001 + id`.
- [x] Verify `apos_node.c`'s `RANGE_CMD` peer range check follows `UWB_MAX_ANCHORS`
      rather than a literal.

### Task 4: Grouped DISCOVERY responder — **done** (2026-09-08, this session)

- [x] Copy the tag's updated `uwb_frame_802_15_4z.{c,h}` (tag Task 3) and run both
      repos' `tests/uwb_frame/`.
- [x] `src/disc_schedule.{c,h}`: add
      `bool disc_group_match(uint8_t anchor_id, uint8_t group, uint8_t n_groups)` and
      `uint32_t disc_resp_delay_uus_grouped(uint8_t anchor_id, uint8_t n_groups)` using
      `rank = anchor_id / n_groups`, `delay = DISC_BASE_UUS + rank * DISC_SLOT_UUS`.
- [x] Keep `disc_resp_delay_uus()` for `n_groups == 1` compatibility, or delete it and
      update every caller — do not leave two schedules that can disagree. (Kept, and
      `tests/disc_schedule/` asserts the two agree at `n_groups = 1`.)
- [x] **Bound the rank.** With `n_groups` under-sized for the deployment, `rank` exceeds
      3 and the response lands past `TX_COMPLETE_TIMEOUT_MS`, silently losing every
      DISCOVERY response for high ids — the exact failure the 10 ms timeout caused
      before. Refuse to answer (and `LOG_WRN`) when `rank > DISC_MAX_RANK` (3) rather
      than transmitting late.
- [x] `src/anchor_respond.c`: parse `group`/`n_groups`, answer only on a match, use the
      grouped delay. Re-derive `TX_COMPLETE_TIMEOUT_MS` in its comment from
      `DISC_BASE_UUS + DISC_MAX_RANK * DISC_SLOT_UUS` = 12500 uus — the value 18 stays,
      but the derivation must name the rank, not the id.
- [x] `tests/disc_schedule/`: every (id, n_groups) pair for 32 anchors matches exactly
      one group; max delay over all ids at `n_groups = 8` is 12500 uus; the `rank > 3`
      refusal fires.
- Verified: `tests/disc_schedule/` PASSED; production image (`west build`) links clean,
  zero warnings, with this responder compiled in. Not yet run on hardware.

### Task 5: ANNOUNCE (`0xEC`) emitter — **done** (2026-09-08, this session)

- [x] Copy the tag's frame module again (tag Task 4); run both suites.
- [x] `src/uwb_slave.c`: after receiving a beacon, if `beacon.announce_id == anchor_id`,
      transmit one `0xEC` at a fixed delay inside the announce window, carrying the short
      address, `(x, y, z)` and the CIR quality of that beacon. **Deviation from the literal
      text:** `announce_id` is not read off the beacon — see Task 6's note; it is computed
      locally by both sides from `frame_counter`, which the beacon already carries.
- [x] Gate it on `position_valid` exactly like the WAVE responder — an unpositioned board
      must not advertise coordinates it does not have.
- [x] Route it through `beacon_guard` like every other delayed TX.
- [x] `z` comes from `anchor pos <x> <y> <z>`, which already exists and is already
      surveyed by `apos`. **Deviation:** ships `cfg->z` as configured rather than
      NaN-when-unset — this codebase has no state distinguishing "no height known" from
      "height is 0" (`position_valid` gates `(x, y, z)` atomically, and a 2D survey's
      `z = 0` is a real floor-relative value, not a placeholder). Documented in
      `anchor_respond.c`; adding that distinction is future work if it turns out to
      matter, not done here.
- [ ] **Step 4 (user, hardware):** sniffer shows exactly one ANNOUNCE per superframe,
      from the anchor whose id matches, inside the window and never overlapping the CAP.

### Task 6: Gateway schedules the announce — **done, cheaper than speced** (2026-09-08)

- [x] `src/uwb_gateway.c`: beacon gains `announce_id = frame_counter % A` with
      **`A = 37`**, prime and coprime with the cycle `C = 16`. **Deviation:** implemented
      as a value BOTH sides derive locally from `frame_counter` (`ANNOUNCE_CYCLE_A` in
      `uwb_mac.h`), not a new byte on the beacon wire. The gateway already transmits
      `frame_counter` every beacon and every anchor already parses it back out, so the
      byte the design doc proposed is redundant — this costs nothing in airtime, avoids
      growing `UWB_FRAME_LEN_BEACON` a second time before the Task 10 slot-count bump,
      and needs no `uwb_gateway.c` change beyond nothing (the constant lives in
      `uwb_mac.h`, consumed by `anchor_respond_announce()` in `src/anchor_respond.c`, not
      by the gateway at all — the gateway does not need to know whose turn it is).
- [x] Put the coprimality requirement in a comment *and* in a host test: with `A = 32` a
      tag awake only on `frame_counter % 16 == p` observes just two announce ids ever.
      This is the kind of aliasing that looks like an RF fault for a week.
- [x] `A` lives next to `C` in one header so the two cannot drift into a common factor.
      (`ANNOUNCE_CYCLE_A` in `uwb_mac.h`, `GW_CYCLE_C` in `gw_core.h`, cross-referenced in
      both files' comments — not literally one header, since `GW_CYCLE_C` is
      gateway-internal seat-table state no slave has reason to include, but the two
      cannot drift silently: `tests/gw_core/` checks the pair directly.)
- [x] `tests/gw_core/`: for every phase `p` in `0..15`, the set of announce ids observed
      by a tag sampling that phase covers all `0..A-1`.
- Verified: `tests/gw_core/` PASSED (includes a negative control proving the test itself
  can fail: `A == C` collapses to one observed id). Production image builds clean.

### Task 7: Survey scales to 32 (sparse mesh) — **done** (prior session, per `CLAUDE.md`
    §3 and `git log`)

- [x] `src/apos_geom.h`: `APOS_MAX_NODES` 8 -> 32. Note the memory: `APOS_MAX_EDGES`
      becomes 496 and `APOS_MAX_MEAS` 992 — size the structs deliberately, do not let
      them land on the gateway loop's stack.
- [x] **Candidate pairs only.** `apos_table` records which peers heard each other during
      `apos enum`; `apos_gw` ranges only those ordered pairs. A full mesh at 32 nodes is
      992 exchanges at one frame per superframe — hours.
- [x] **Rigidity check before trusting a solve.** A sparse framework can be flexible or
      reflection-ambiguous with many edges. Add a connectivity + degree-of-freedom test
      (`2N-3` in 2D, `3N-6` in 3D against the edge count, plus a per-node minimum degree)
      and report it in the solve JSON next to `rms_mm`.
- [x] `rms_mm` becomes a real acceptance signal at 32 sparse-but-redundant nodes — but
      **only when the rigidity check passes**. Keep `apos_gw_result_unverified()` and make
      it read the new check rather than the hard-coded 4-node case.
- [x] `apos enum` stagger must fit 32 responders: reuse the EUI-hash stagger with enough
      slots, or run enumeration in rounds like grouped discovery. Whichever, bound the
      window and say what bounds it. (8-round stagger, `APOS_GW_ENUM_ROUNDS`.)
- [x] `pos_json_anchors()` at 32 nodes is ~4.8 kB — check it against the MQTT buffer and
      chunk or truncate deliberately rather than overflowing.
- [x] `tests/apos_geom/`, `tests/apos_table/`: a sparse 12-node mesh solves; a
      disconnected mesh is rejected; a flexible mesh is flagged, not accepted; the
      candidate-pair filter never drops an edge that was measurable.
- [ ] **Step 4 (user, hardware):** an 8-anchor survey completes and the solved
      node-to-node distances match a tape measure.
- Verified this session: `tests/apos_geom/`, `tests/apos_table/` PASSED. Not run on
  hardware — see `CLAUDE.md`'s "None of this ... has been run on any board."

---

## Phase 2 — multi-poll responder (the slot lever)

### Task 8: MPOL_RESP (`0xED`) — **done** (2026-09-08, this session)

- [x] Copy the tag's frame module (tag Task 7); run both suites.
- [x] Confirm the byte layout against the tag's parser with a shared vector — this frame
      carries timestamps, so a silent field-offset disagreement becomes a wrong distance,
      not a dropped frame. (`tests/uwb_frame/` is the tag's own suite, copied verbatim —
      shares the tag's vectors by construction.)

### Task 9: Answer `0xE3` MULTI-POLL — **done** (2026-09-08, this session)

- [x] `src/anchor_respond.c`: new `anchor_respond_multipoll()`. Parse the slot list,
      find our own short address, and schedule one delayed `0xED` at
      `poll_rx_ts + delay_us` — the delay the tag assigned, not one we compute.
- [x] Ignore a poll that does not name us. Log it at DBG the way `wave_other` does; that
      line is the first fork when diagnosing a silent anchor.
- [x] `beacon_guard` applies per response exactly as for WAVE.
- [x] `TX_COMPLETE_TIMEOUT_MS` on this path must cover the **largest** `delay_us` the tag
      can assign (`MPOL_BASE_UUS + 3 * MPOL_SLOT_UUS` ~ 7.1 ms) plus airtime. State the
      derivation in the comment. Do not reuse the DISCOVERY constant by accident. (Comment
      states the exact numbers: `2200 + 3*1700 = 7300` uus, confirmed to stay inside the
      existing 18 ms bound rather than needing a new one.)
- [x] Keep the WAVE single-poll responder compiled and working — the calibration image
      and the bench depend on it. (Untouched.)
- [ ] **Step 4 (user, hardware):** sniffer shows one `0xE3` followed by up to four `0xED`
      at the assigned offsets, no overlap, and a tag solving from them.
- Verified: production image (`west build`) links clean, zero warnings, with both
  responders and the WAVE path all compiled in together. Not run on hardware.

### Task 10: Slot constants follow the measurement

- [ ] After the tag measures the real slot occupancy (tag Task 8 Step 4): update
      `UWB_FRAME_N_CFP` 11 -> 14 and `UWB_FRAME_MAX_LEN` 37 -> 44 via the frame-module
      copy, then fix `uwb_slave.c`'s `BUILD_ASSERT` that ties them together.
- [ ] Re-check that a 44-byte beacon still fits every `RX_BUF_LEN` (64) and that
      `dwt_getframelength()`'s FCS handling is still `flen - FCS_LEN` everywhere — an
      unsubtracted FCS turns a 14-slot beacon into 15 and reads the FCS as a seat.

### Task 11: Tighten the turnaround (optional, measurement-gated)

- [ ] Only if Task 2 found real headroom: lower `POLL_RX_TO_RESP_TX_DLY_UUS` toward
      ~1200 uus, in lockstep with the tag's `POLL_TX_TO_RESP_RX_DLY_UUS` **and** the
      DWM3001CDK reference node's `POLL_RX_TO_RESP_TX_DLY_UUS` (which must match, or
      calibration stops working — `docs/antenna-delay-calibration.md` §1.1).
- [ ] Re-run antenna-delay calibration afterwards: the turnaround is inside the number
      `cal ref` solves for.
- [ ] Abort this task rather than run it close to the deadline; the near-miss margin is
      what produced the unbounded-TXFRS console freeze.

---

## Phase 3 — gateway: 224 seats, phases, and lease from POS

### Task 12: Seat table gains phases — **done** (prior session, `git log`)

- [x] `src/gw_core.h`: `GW_CYCLE_C` = 16; `seats[GW_CYCLE_C][GW_N_CFP]`. At 14 slots that
      is 224 seats; check the struct size against available RAM and keep it out of the
      gateway loop's stack.
- [x] `gw_core_build_slotmap(c, phase, out)` publishes one phase's row.
- [x] `gw_core_superframe_tick()` ages leases once per superframe for **all** phases, not
      only the current one. A tag that owns one phase must not age 16x slower than one
      that owns four.
- [x] `tests/gw_core/`: 224 distinct tags each get a seat; the 225th is refused cleanly;
      every phase's map is consistent across a full cycle; a lease expires after exactly
      `GW_LEASE_SF` superframes regardless of the tag's phase count.

### Task 13: Phase allocation by tier — **done** (prior session, `git log`; see also
    `docs/superpowers/plans/2026-09-08-task-13-phase-allocation-status.md`)

- [x] `gw_core_join()` grants one phase by default; `gw_core_keepalive()` /
      the POS path re-grants `n_phases` from the tag's requested tier: IDLE 1, SLOW 2,
      FAST 4 (8 when the budget allows).
- [x] Allocation spreads a mover's phases **evenly** around the cycle (`phase, phase+C/n,
      ...`), not consecutively — four consecutive phases give four fixes in 0.8 s and then
      2.4 s of nothing, which is worse for the EKF than an even 1.25 Hz.
- [x] Under contention, a FAST tag never takes the *last* phase of a stationary tag: an
      idle tag with zero phases has silently lost its seat.
- [x] `tests/gw_core/`: even spread for 2, 4 and 8 phases; contention never strands a tag
      at zero phases; a tier drop returns phases to the pool.
- Landed, but **not yet safe to exercise against real tag firmware for grants smaller
  than the full cycle** — see `src/gw_core.h`'s file comment and the status doc above
  (now corrected, see Task 14's note below). Verified: `tests/gw_core/` PASSED.

### Task 14: GRANT carries the phase mask — **done** (2026-09-08, this session)

- [x] Frame-module copy from the tag (tag Task 11): GRANT 24 -> 26 with a 16-bit
      `phase_mask`. Run both suites. (Already present in the frame module from the Task 4
      copy; this task is specifically about the gateway actually SENDING it.)
- [x] `send_grant()` fills it from the allocator. `g->phase_mask` now goes out on the wire
      instead of being deliberately omitted — see `uwb_gateway.c`'s updated comment on
      `send_grant()` for what safety property this does and does not complete (the tag
      parses and derives `listen_skip`/`in_map` from it already; multi-phase grants are
      still not safe to exercise on air per `gw_core.h`'s file comment, unchanged by this
      task).
- [x] A re-tier that changes only the mask is published in the next GRANT, not by
      forcing a re-JOIN. (Unaffected by this task — already true of `send_grant()`'s
      existing call sites.)
- Verified: production image (`west build`) links clean, zero warnings. Not run on
  hardware.

### Task 15: POS renews the lease; JOIN backoff — **partial -> now closed in code**
    (POS-renews-lease landed prior session; JOIN backoff done 2026-09-08, this session,
    on the **tag** side — `tag_testting/src/uwb_net.c`, not this repo)

- [x] `uwb_gateway.c`'s POS dispatch already decodes `0xEA` and looks up the seat by
      short address — refresh `lease_remaining` there. Keep the existing rule that POS is
      **not** gated on seat state: a straggler after expiry is still published, it simply
      does not resurrect a seat.
- [x] `tests/gw_core/`: `gw_core_pos_seen(addr)` refreshes exactly the matching seat, does
      nothing for an unknown address, and never creates a seat.
- [x] JOIN backoff. **Landed differently than speced, and in the other repo.** Tracing the
      actual collision mechanism (`tag_testting/src/uwb_net.c`'s `UWB_ST_JOINING`: "retry
      each beacon until GRANT", no jitter) showed a gateway-side signal (a GRANT field
      telling a REFUSED tag to back off) cannot fix the described failure: on a bulk
      restart, most colliding JOINs never reach the gateway at all, so there is nothing
      for the gateway to respond to. The fix is tag-side: `uwb_net_join_backoff(eui)`
      (FNV-1a over the EUI, mod `UWB_NET_JOIN_BACKOFF_MAX` = 64) delays a joining
      episode's FIRST JOIN by 0..63 superframes before `UWB_ST_JOINING` ever transmits,
      spreading a bulk rejoin's arrivals across ~12.8 s instead of contending the same 4
      Aloha mini-slots every superframe. `tests/uwb_net/test_join_backoff` (tag repo)
      covers range, per-EUI determinism, non-collision across differing EUIs, and the
      countdown mechanic itself. No ANCLA-side change was needed or made for this bullet.

### Task 16: Short addresses are reused by EUI — **done** (`git log`, "reuse short
    address by EUI")

- [x] `alloc_short_addr()` keeps an EUI -> address map (64 entries, LRU) so a rejoining
      tag gets its previous address back.
- [x] This also removes the `Tid` phantom-device case documented in `CLAUDE.md`. Keep the
      EUI-hash `Tid` regardless — it is the stable identity and does not depend on this.
- [x] `tests/gw_core/`: rejoin after lease expiry returns the same address; the map
      evicts oldest-first; a full map still grants a fresh address rather than failing.

### Task 17: `gw seed <n>` — synthetic full occupancy — **done** (`4cb4d61`, `25a326f`)

- [x] Console command filling `n` synthetic seats with fabricated EUIs, so the real
      beacon publishes real maps at full 224-seat occupancy with no RF from phantom tags.
      Gateway-only, refuses in SLAVE mode, and prints a loud warning — this must never be
      mistaken for a real deployment state.
- [ ] **Step 4 (user, hardware):** at `gw seed 224`, the beacon stays on time for 10
      minutes (no `"beacon started but TXFRS never completed"`), maps are consistent
      across a whole cycle on a sniffer, and the console stays responsive. This is the
      test that actually stands behind the "100 tags" claim.

---

## Phase 4 — range and closeout

### Task 18: 100 m link validation

- [ ] Only after Task 1 closes. With the corrected transmit chain plus whatever antenna
      and RX-LNA change it implies, measure the tag -> anchor link (the weak direction) at
      25, 50, 75 and 100 m, LOS, and record RSSI and range success at each.
- [ ] Repeat with the tag worn on a body — body loss is 3-6 dB and it is the deployed
      case, not a pessimistic one.
- [ ] If the link does not close at 100 m, the escalation order is: anchor RX LNA,
      anchor antenna gain, mounting height/LOS, and only then reopening the PHY decision
      in design §8 with the measurement attached.

### Task 19: Beacon coverage check

- [ ] Confirm the gateway's beacon reaches every anchor and every tag position across
      the 100 m site. If it does not, the site needs the deferred beacon-extension work
      (design §10) — flag it, do not paper over it with a second unsynchronised gateway.

### Task 20: `proto_ver` -> 3 — **code done**, reflash/confirm pending (hardware)

- [x] Bump `UWB_PROTO_VER` in the frame module (tag repo lands it, this repo copies) and
      reflash the **whole** fleet in one session: gateway, every anchor, every tag. The
      frame module already carries `UWB_PROTO_VER 3` from the tag repo's earlier work
      (its Task 12); this session's Task 4 copy brought it into ANCLA. Every frame this
      repo builds (`write_hdr()`) is stamped with it automatically — no ANCLA code reads
      or gates on `proto_ver` itself, only logs it (`uwb_slave.c`); the tag is the side
      that refuses a mismatched beacon. **Reflashing the fleet is the hardware half of
      this bullet and has not happened.**
- [ ] Confirm a v2 node is cleanly deaf rather than half-working: a v2 tag must stay in
      SCAN against a v3 beacon.

### Task 21: Documentation

- [ ] Update this repo's `CLAUDE.md`: the 32-anchor cap and the 254 wire cap, grouped
      discovery and its two derived timeouts, ANNOUNCE and the `gcd(A, C) = 1`
      requirement, the phase seat table, POS-based lease renewal, EUI address reuse, and
      `gw seed`.
- [ ] Retire the "There is no fifth-anchor option to suggest to an operator" hard-won
      fact — it is superseded here — and replace it with what the new limit is and what
      derives from it.
- [ ] Fold the Task 1, 2, 9 and 18 measurements back into the design's §5 and §11.

# POS frame (`0xEA`) — tag position reports over UWB

**Date:** 2026-08-11
**Status:** design approved, not yet implemented
**Sub-project:** E1 of E1+E2 (see "Scope split" below)
**Branch:** `feat/pos-frame` off `master` (ANCLA_ESP32S3) and off
**`feat/npm1304-battery`** (tag) — see §6.0, *not* the tag's `master`

---

## 1. Motivation

The tag already computes its own position: `pos_solve()` in `src/pos_solver.c`
produces `(x, y)` from up to four ranges, and `uwb_net_runner.c:690` hands the
result to `position_publish()`. That function today formats `P:x,y` to the BLE
NUS console and nothing else. Its own comment anticipates this work:

> A future UWB-to-master sender replaces only this function body.

Nothing carries the fix off the tag over UWB, so the gateway — which sees every
frame in the superframe — has no idea where any tag is. This spec defines the
frame that closes that gap.

## 2. Scope split

The original request covers two independent subsystems. They are separated
because their failure modes are unrelated and mixing them makes the first bench
run unreadable:

- **E1 (this spec)** — the `0xEA` POS frame end-to-end: tag TX, gateway RX,
  decoded and logged as JSON on the gateway console.
- **E2 (separate spec)** — WiFi + MQTT uplink on the gateway, publishing to
  `testtopic/1/position` and a stubbed `testtopic/1/anchors`. E2 replaces the
  back end of E1's sink and touches nothing else.

E1 must range cleanly on hardware before E2 starts.

### Carried forward to E2

One finding from E1's exploration belongs in E2's spec and is recorded here so
it is not rediscovered:

**"WiFi on its own isolated core" is not achievable.** Zephyr's ESP32 WiFi
driver declares `depends on !SMP` (`drivers/wifi/esp32/Kconfig.esp32:8`), with
the help text *"Only supported in single core mode because the network stack is
not aware of SMP stuff."* So `CONFIG_SMP` + `k_thread_cpu_pin()` will not build
with WiFi enabled, and the AMP alternative (appcpu as a separate Zephyr image)
cannot host the network stack either — the WiFi blobs are bound to procpu.

The requirement's *intent* is nonetheless already met. Unlike fw-cre's
busy-polling gateway, the Zephyr gateway loop blocks in
`k_sem_take(&rx_sem, K_MSEC(400))` (`src/uwb_gateway.c:305`) and yields for most
of every superframe. A lower-priority network thread runs in that slack without
ever preempting a delayed-TX arm. E2 achieves isolation by thread priority plus
a decoupling queue, not by core affinity.

## 3. Wire format

`0xEA` is the next free function code after `0xE9` RELEASE. Added to
`src/uwb_frame_802_15_4z.{c,h}`, which **must remain byte-identical between the
tag and anchor repos** — as must the shared host test file.

```
#define UWB_FRAME_TYPE_POS  0xEA
#define UWB_FRAME_LEN_POS   24
```

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–9 | header | — | dest `0x0000` (gateway), src = tag short addr, type `0xEA` |
| 10–13 | `x` | float32 LE | metres |
| 14–17 | `y` | float32 LE | metres |
| 18–21 | `residual_m` | float32 LE | RMS range residual, metres |
| 22 | `n_anchors` | uint8 | 3 or 4 |
| 23 | `batt_soc` | uint8 | 0–100 percent; `0xFF` = unknown/charging |

24 bytes is well under `UWB_FRAME_MAX_LEN` (39), so no buffer constant changes.

**Design notes:**

- Tag identity and sequence number come free in the 802.15.4z header; they are
  not duplicated in the payload.
- float32 throughout matches how `0xE4` RANGE-RESPONSE already carries
  coordinates. An int16-centimetres residual would save two bytes — about 19 µs
  of airtime — which does not justify introducing a unit conversion.
- **No ACK.** An acknowledgement is another ~1.3 ms frame in a budget that is
  already over (§5), and at the 5 Hz fast-tier rate a lost fix is superseded
  200 ms later. POS is fire-and-forget.
- Adding a new frame type does **not** by itself bump `proto_ver`: contract §7
  requires a bump for changed timing constants, changed `N_CFP`/`N_CAP`, or a
  changed *existing* layout. See §5 — the hardware gate's outcome may force a
  bump for a different reason.

## 4. Placement in the superframe

The tag transmits POS in the **tail of its own CFP slot**, immediately after its
fourth ranging exchange completes and the solve returns. Immediate TX, not
delayed — the tag is inside a slot no other tag may use.

This is contention-free by construction and requires no new MAC state. The CAP
alternative was rejected because POS would then contend with JOIN/KEEPALIVE from
other tags, losing fixes exactly under the load where they matter most.
Piggybacking on KEEPALIVE (`0xE8`) was rejected because it couples the position
rate to the lease-renewal cadence and overloads a MAC frame with application
data.

The gateway needs no change to hear it: it already arms RX continuously between
beacons, re-arming with a timeout right up to the beacon margin.

## 5. Slot budget — BLOCKING HARDWARE GATE

**No implementation of §4 may be finalised before this gate is resolved.**

The contract's superframe budget (§2.1) is:

```
T_beacon + g + N_CAP·t_minislot + g + N_CFP·(T_slot + g)
  = 1.5 + 0.5 + 4·1.5 + 0.5 + 12·15.5  ≈ 194.5 ms  ≤ 200 ms
```

That leaves **5.5 ms of slack across the entire superframe**. A POS frame costs
roughly 1.3 ms of airtime — at PLEN-1024 the ~1.04 ms preamble dominates and the
24-byte payload adds ~226 µs at 850 kbps. Adding that to every CFP slot costs
`12 × 1.3 ≈ 15.6 ms`, overrunning 200 ms by about 10 ms.

There is a second, larger uncertainty. `T_slot ≈ 15 ms` assumes four exchanges
at ~3.75 ms each. With current constants — `POLL_RX_TO_RESP_TX_DLY_UUS = 2000`
(~2.05 ms) plus two ~1.25 ms frames — a single exchange looks closer to
**~4.5 ms**, implying a ~18 ms sweep that is already over budget before POS
exists. This is arithmetic from the anchor's constants, **not** a measurement of
the tag's sweep, which may overlap operations not visible from the anchor side.

### Gate: measure the real `T_slot`

Measure the actual wall-clock duration of one tag's four-anchor sweep, from
first poll TX to the last response RX, on hardware. One tag, one superframe; a
scope on the TX line or DW3000 timestamp logging both suffice.

Record the measured value in this spec before choosing among:

| Option | Effect | Cost |
|---|---|---|
| `N_CFP` 12 → 11 | frees 15.5 ms, spends ~15.6 ms on POS | one fewer simultaneous mover; **bumps `proto_ver`** (contract §7); ripples through `UWB_FRAME_LEN_BEACON`, `UWB_FRAME_MAX_LEN`, the beacon slot-count identity check in `src/uwb_slave.c`, and the shared codec in both repos |
| Shrink `T_slot` to fit | keeps 12 seats | only viable if the measurement shows the sweep is materially under 15 ms |
| Adopt MULTI-POLL `0xE3` | `T_slot` → ~6–7 ms, ample room, ~2× seats | explicitly gated on hardware validation of multi-poll timing; turns E1 into a much larger change |
| Lower POS rate than ranging | preserves 12 seats and all timing | gateway must schedule who reports when; MQTT updates slower than the ranging rate |

### Decision (2026-08-11)

**`N_CFP` 12 → 11.** No hardware `T_slot` measurement was taken — the project
owner has hardware for 5 tags at present, so 11 concurrent CFP seats is not a
practical constraint today, and re-measuring/re-tuning `T_slot` is deferred to
a future optimization pass (see `CLAUDE.md` Phase 4 — Optimization). This is a
conservative capacity decision, not a measured one: it trades one simultaneous
mover for headroom without needing to know the real sweep duration.

Per the table above, this bumps `UWB_PROTO_VER` from `1` to `2` (contract §7)
and ripples through `UWB_FRAME_LEN_BEACON`, `UWB_FRAME_MAX_LEN`, the beacon
slot-count identity check in `src/uwb_slave.c`, and the shared codec in both
repos — all owned by Task 2 of the implementation plan.

Before this system is pushed past 5 concurrent tags, or before `N_CFP` is
reconsidered, the hardware gate above (measured `T_slot`, min/mean/max over
≥20 sweeps) still needs to be run for real.

## 6. Tag-side changes

### 6.0 Base branch and radio ownership

**E1 branches from the tag's `feat/npm1304-battery`, not `master`.** That branch
is 27 commits ahead and `master` is fully contained in it, so this is a
fast-forward relationship rather than a divergence — but two of those commits
are prerequisites:

- `4bf03d6 feat(batt): migrate battery monitoring from bq274xx to nPM1304` —
  the SoC source §6.2 depends on. The `batt_read_soc()` API described there is
  the nPM1304 version and exists only on this branch.
- `13eac0c feat(uwb): add uwb_radio_owner` and `d3f4b90 feat(runner): yield the
  radio once per superframe` — an explicit DW3000 handover protocol.

**POS needs no radio claim.** The transmission happens inside the runner's own
ranging sweep, where the runner already owns the DW3000; `uwb_radio_owner` is
for *other* threads (calibration) asking the runner to step aside. Adding a
`uwb_radio_request()` around the POS TX would be wrong — the runner would be
claiming from itself.

Two constraints follow from that module's contract
(`src/uwb_radio_owner.h`):

1. The POS TX must complete **before** the runner reaches its once-per-superframe
   yield point, so the sweep and its position report stay one indivisible unit.
2. POS TX uses only `dwt_writetxdata` / `dwt_writetxfctrl` / `dwt_starttx`, none
   of which are on the PHY-state restore list, so it disturbs nothing a claimant
   would need rebuilt. This must remain true: the header warns that touching
   `dwt_setcallbacks`, frame filtering, or PAN/short address without restoring
   them "silently breaks runner RX with no error anywhere — the tag simply stops
   hearing beacons."

### 6.1 `pos_solver` — add a residual

`struct pos_result` gains `float residual_m`. After `p` is solved, for each of
the `n` measurements compute the predicted range `|p − (x_i, y_i)|`, subtract the
measured `range_m`, and take the RMS over `n`. Four square roots; the
`pos_meas` array is already in scope. Set to `0.0f` when `valid == false`.

The existing host test for `pos_solve()` is extended: an exactly-consistent
geometry must yield a residual at zero within float tolerance, and a
deliberately inconsistent one must yield a clearly non-zero residual.

### 6.2 `batt` — cached SoC

POS must **not** call `batt_read_soc()` inline: it is an ADC read plus a curve
lookup, and the slot tail is timing-critical. POS reads a cached byte instead.

`batt_monitor_start()` already runs a periodic timer, but it samples *current*,
not state of charge — SoC is read on demand today. So this work adds the cached
SoC: either extend that timer to also refresh a cached percentage, or add a
low-rate sampler beside it. The cache is written from a non-timing-critical
context and read by POS as a plain byte. A refresh interval on the order of
seconds is ample; battery percentage does not move at 5 Hz.

`batt_read_soc()` returns `-EBUSY` while the charger is connected, because
terminal voltage then says nothing about charge. That case, and any other error,
maps to the `0xFF` unknown sentinel rather than a fabricated percentage.

### 6.3 `position_publish()` — the TX hook

`src/uwb_ss_initiator.c:426`. Signature extends to carry `residual_m` and
`n_anchors` alongside `x`/`y`; the caller at `uwb_net_runner.c:690` already has
both in scope.

The BLE `P:x,y` line **stays**. The function's comment anticipates a UWB sender
"replacing only this function body", but during bring-up the console line is the
only independent check that the tag solved what the gateway received. Keeping
both costs one `twr_log()` call.

### 6.4 Prerequisite

The tag working tree currently carries an uncommitted change to
`src/uwb_ss_initiator.c` that removes the oversized-frame guard added by HEAD
commit `0b93e28`, allowing `do_one_range()` to fall through to a `memcmp`
against a stale `rx_buf`. E1 edits this file; the tree must be resolved (either
direction) before work starts.

## 7. Gateway-side changes

- `uwb_frame_is_pos()` and `uwb_frame_parse_pos()` in the shared codec.
- A `POS` branch in the gateway's `dispatch()`.
- New `src/pos_sink.{c,h}`: receives a decoded fix and, in E1, emits one JSON
  line per fix on the console — deliberately the same shape E2 will publish, so
  E2 swaps the sink's back end and touches nothing else.
- The gateway remains **MAC-only**: it does not range, and POS handling adds no
  TX.

**A POS from a tag holding no current lease is logged, not dropped.** Telemetry
should not be gated on lease state, and a silently discarded fix is close to
undebuggable from the broker's side.

## 8. Error handling

| Condition | Behaviour |
|---|---|
| Malformed / short POS frame | drop, `LOG_WRN` — matches the existing beacon path |
| `batt_read_soc()` error or `-EBUSY` | `batt_soc = 0xFF` |
| `pos_solve()` returns false | no POS frame sent at all |
| POS from unknown short addr | logged normally (§7) |
| POS would overrun the slot | governed by §5; not a runtime check |

## 9. Testing

**Host tests** (plain gcc, `CHECK` macro, non-zero exit on failure):

- Codec round-trip for `0xEA`: build → parse → field-for-field equality,
  including the `0xFF` battery sentinel and both `n_anchors` values. The test
  file is shared byte-identically between repos.
- Rejection: truncated frames, wrong function code, wrong length.
- `pos_solve()` residual, per §6.1.

**On-target:**

1. `T_slot` measurement (§5 gate) — blocking, precedes the rest.
2. One tag ranging in a granted slot; gateway console shows one JSON line per
   superframe with plausible `x`/`y` and `n_anchors == 4`.
3. Tag walked between known points; logged coordinates track the movement.
4. Battery field: reads 0–100 on battery, `0xFF` with the charger attached.
5. Two tags in different slots; both fixes appear, correctly attributed by the
   header's source address.

## 10. Out of scope

- WiFi, MQTT, JSON transport (E2).
- Anchor position distribution — the `testtopic/1/anchors` payload is stubbed in
  E2, and how anchors learn their own coordinates is a later design.
- 3D positioning: the solver is 2D, so POS carries no `z`.
- MULTI-POLL adoption, unless §5's gate selects it.

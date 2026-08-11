# GATEWAY mode: SPI clock, TDMA beacon and CAP seats — design

**Date:** 2026-08-11
**Status:** approved, not yet implemented
**Scope:** sub-project **D** of the nRF5 → Zephyr anchor port, the last one

## Problem

After spec C the anchor answers ranging polls in both wire formats, bench-confirmed
over the air. `uwb_gateway.c` is still the spec A stub: nothing emits the beacon that
defines the network's time base, and nothing grants a tag a seat. Without a gateway the
tags range opportunistically and nothing bounds how many can do so at once.

This spec makes GATEWAY mode real, and makes the slaves aware of the beacon it emits.

## Deployment

Five boards. The gateway holds the contract's reserved `0x0000` and consumes no
`anchor_id`; the four slaves take ids 0..3 and short addresses `0x0001`–`0x0004`.

```
0x0000  GATEWAY   MAC only — beacon, JOIN/GRANT/KEEPALIVE/RELEASE
0x0001  SLAVE id 0   ranging
0x0002  SLAVE id 1   ranging
0x0003  SLAVE id 2   ranging
0x0004  SLAVE id 3   ranging
```

No addressing change is needed: `UWB_MAX_ANCHORS` stays 4 and
`disc_resp_delay_uus()` already staggers exactly four anchors.

Four ranging anchors rather than three is deliberate. The tag's solver accepts three
(`pos_solver.c:10`: `if (n < 3 || n > POS_MAX_ANCHORS)`), but three gives two equations
for two unknowns — an exact solve with no redundancy and no averaging of range error.
The fourth makes the system overdetermined and lets the least-squares stage suppress
noise and outliers.

`anchor_id` is unused on a node in GATEWAY mode. It is left in the config rather than
special-cased, so a board can be switched between modes without reconfiguration.

## Decisions

Recorded with their reasons, because several depart from the nRF5 source.

| Decision | Rationale |
|---|---|
| Gateway is **MAC-only** — it does not answer ranging polls | Departs from `owner_gateway.c`'s `dispatch()` fall-through. Simplifies the beacon arm margin (below) and keeps the MAC loop free of a 16.5 ms discovery stagger. Costs one board, not one anchor. |
| The **position gate is kept** as `owner_gateway.c:161-167` has it | A MAC-only gateway never transmits its coordinates, so the gate protects nothing functional. Kept anyway: the default mode is SLAVE, so reaching GATEWAY already requires a deliberate console session, and refusing to start an unpositioned gateway keeps the operator honest about a value that becomes load-bearing the moment the gateway ever ranges. |
| Slaves get **TX suppression only**, RX stays continuous | See "What beacon awareness buys" below. |
| The SPI clock fix is **task 1**, with a hard gate | See "The SPI clock" below. |

## The SPI clock

### Finding: the radio has been running at 2 MHz, not 8

`dw3000_spi_init()` (`modules/dw3000-decadriver/platform/dw3000_spi.c`) builds two
configs and selects the slow one:

```c
spi_cfgs[0].frequency = 2000000;
spi_cfgs[1].frequency = DT_PROP(DW_INST, spi_max_frequency);   /* 8 MHz, from the DTS */
spi_cfg = &spi_cfgs[0];                                        /* slow */
```

Nothing ever switches. `dw3000_spi_speed_fast()` exists and is wired into the driver's
vtable as `.setfastrate` (`platform/deca_port.c:37`), but its only call sites are inside
a `static int32_t init()` wrapper this build never reaches: `dwt_initialise()` dispatches
through `dwt_ops->initialize`, and that is `.initialize = ull_initialise`
(`dw3000/dw3000_device.c:9371`), not the wrapper. `uwb_radio.c` does not call it either,
because the spec A design table recorded `port_set_dw_ic_spi_fastrate()` as *"no
equivalent — dropped; SPI speed is fixed at 8 MHz in the DTS."* That was wrong twice
over: the equivalent exists, and the DTS value was never in effect.

The boot log has been printing `DW3000 SPI (max 8MHz)` throughout. It reports
`spi_cfgs[1].frequency` — the config that is never selected.

The bench measurement corroborates it exactly. `read_cir()`'s
`dwt_readdiagnostics()` under `DW_CIA_DIAG_LOG_ALL` performs two ~108-byte bursts:

| | |
|---|---|
| transfer | 216 B = 1728 bits |
| at 2 MHz | **864 µs** |
| measured | **~944 µs** |

80 µs of overhead on a transfer that is otherwise entirely clock-bound. This also
retires the earlier conclusion recorded in `CLAUDE.md` — the cost was never the
`ioctl`-style dispatch or the bit-banged CS, and dropping to 2400 µs failed for the same
reason logging removal did not help: 1728 bits at 2 MHz is 864 µs no matter what else
changes.

### Target rate: the DW3000 maximum, which on this SoC is 26.67 MHz

Take the highest rate the part allows. Two ceilings apply, and they interact.

**The DW3000 ceiling is ~38 MHz** (Qorvo DW3000 family datasheet; the figure is not
reproduced anywhere in the vendored module, so confirm it against the datasheet on hand
before flashing). What *is* in the module is the lower bound's counterpart — the part
must be clocked slowly until it leaves INIT_RC:

```
deca_device_api.h:2381  "Once the device is in IDLE_RC SPI rate can be increased to
                         more than 7 MHz."
deca_device_api.h:2601  "SPI rate must be <= 7MHz before a call to this function as the
                         device will use FOSC/4 as part of internal reset"
```

**The ESP32-S3 ceiling is the divisor ladder.** SPI2 is clocked at APB/N from 80 MHz, so
the only reachable rates are 80/N: … 20, **26.67**, 40, 80. Nothing exists between 26.67
and 40. There is no 32 MHz as on the nRF52 — the module README's
`spi-max-frequency = <32000000>` is the nRF52840 SPIM3 limit, not a DW3000 figure, and
does not transfer.

Those two together give the answer: 40 MHz overruns the part, so **26.67 MHz — 80/3 — is
the maximum allowed rate on this hardware.** The conclusion is insensitive to the exact
datasheet number: whether the DW3000 ceiling is 36 or 38 MHz, the next step up the
ESP32-S3 ladder is 40, which exceeds both.

Two practical points on setting it:

- **Requesting `26000000` yields 20 MHz.** The HAL selects the highest achievable rate
  not exceeding the request, and 80/3 = 26.666… MHz exceeds 26. The DTS value must sit
  just above the divisor — `spi-max-frequency = <26670000>` — to land on 26.67 rather
  than silently dropping a rung. Verify against the boot log rather than assuming.
- **The pins are the IO_MUX set.** GPIO 11/12/13 are exactly ESP32-S3 SPI2's
  FSPID / FSPICLK / FSPIQ, so there is no GPIO-matrix routing penalty or input-delay
  compensation problem at this rate. CS being a bit-banged GPIO does not constrain the
  clock.

If 26.67 MHz proves unreliable on the PCB — a signal-integrity question no amount of
datasheet reading settles — the fallback is one rung down at 20 MHz, which still clears
the task 1 gate with room. Task 1 therefore validates the rate by reading `DEV_ID` and
re-confirming ranging, not by the boot log alone.

The staging still matters more than the number. The fix is a `dw3000_spi_speed_fast()`
call placed after `dwt_initialise()` and `dwt_checkidlerc()` and before
`dwt_configure()` — the position `owner_ss_twr_responder.c:75` uses — not merely a
larger DTS value, which on its own would only change a config that is never selected.
`dw3000_spi.h` is already on the include path `uwb_radio.c` uses for `dw3000_hw.h`, so
no change to `modules/` is required.

### The gate

The contract's superframe budget (`spec/2026-06-17-uwb-mac-protocol-contract.md` §2.1):

```
T_beacon + g + N_CAP·t_minislot + g + N_CFP·(T_slot + g)
   1.5   + 0.5 +    4·1.5      + 0.5 +   12·15.5        ≈ 194.5 ms ≤ 200 ms ✓
```

`T_slot ≈ 15 ms` is defined as *"one single-poll ×4 sweep"* — the tag ranging four
anchors inside its slot. Each range contains one anchor turnaround, so `T_slot` is a
direct function of `POLL_RX_TO_RESP_TX_DLY_UUS`:

Turnaround is quoted in UUS, the unit the constant is expressed in; 1 UUS = 1.0256 µs.

| turnaround | per range | `T_slot` (×4) | `N_CFP`·(T_slot+g) | superframe |
|---|---|---|---|---|
| 2000 UUS (≈2.05 ms) | ~3.4 ms | ~15 ms | 186 ms | ~194.5 ms ✓ |
| **2500 UUS (≈2.56 ms)** | ~3.9 ms | ~15.4 ms | 191 ms | **~199.3 ms ✓** |
| 6000 UUS (≈6.15 ms, today) | ~7.5 ms | ~30 ms | 366 ms | **~375 ms ✗** |

At today's 6000 UUS the contract's 12-slot superframe is not merely slow, it is
arithmetically unsatisfiable — only six slots fit in 200 ms, halving network capacity.

**Task 1's gate: `POLL_RX_TO_RESP_TX_DLY_UUS` and `DISC_BASE_UUS` back under 2500 UUS.**
2500 is not arbitrary; it is where the budget above reaches 199.3 ms and stops fitting.
If the measurement will not go under it, work stops and `N_CFP` is renegotiated with the
tag side before any slot map is designed against a number that cannot be honoured.

Task 1 ends by removing the temporary `k_cycle_get_32()` profiling block from
`uwb_slave.c`.

A likely side effect: `read_cir()` at 26.67 MHz costs ~65 µs of clock rather than 864,
which retires the open "skip the CIR read" decision recorded in `CLAUDE.md` — a ~145 µs
diagnostic read is not worth giving up the tag's anchor-ranking signal for.

## Module structure

| File | Origin |
|---|---|
| `src/uwb_radio.c`, board DTS | modified — the SPI fix |
| `src/uwb_dwtime.{c,h}` | extended — `uwb_get_tx_timestamp_u64()` |
| `src/gw_core.{c,h}` | ported from `fw-cre`, unchanged logic |
| `src/uwb_gateway.c` | rewritten from the spec A stub |
| `src/beacon_guard.{c,h}` | new, pure C |
| `src/uwb_slave.c` | extended — TX suppression |

### `gw_core.{c,h}` — seat management

Ported as-is. It is already pure C with no radio dependency: a 12-entry seat table keyed
by CFP slot index, `GW_LEASE_SF = 50` superframes (10 s at 200 ms), a monotonic tag
address pool from `GW_TAG_ADDR_BASE = 0x0100`, idempotent re-join by EUI, and lease aging
in `gw_core_superframe_tick()`. Nothing about it needs to change for this target, and it
is fully host-testable.

### `uwb_gateway.c` — the MAC loop

Interrupt-driven, with the same callback shape as `uwb_slave.c` and the DW3000 system
clock authoritative over the beacon cadence:

```
beacon_tx_ts = tx_beacon(immediate)
loop:
    next_beacon = beacon_tx_ts + T_SUPERFRAME_UUS            (hi32 units)
    while (int32_t)(next_beacon - now32) > BEACON_ARM_MARGIN:
        dwt_setrxtimeout(gap - margin)        /* RXFTO -> cbRxTo -> semaphore */
        k_sem_take(&rx_sem, K_MSEC(bounded))
        dispatch:  JOIN      -> gw_core_join()      -> GRANT
                   KEEPALIVE -> gw_core_keepalive()
                   RELEASE   -> gw_core_release()
    gw_core_superframe_tick()
    beacon_tx_ts = tx_beacon(delayed, next_beacon) ?: re-base on now32
```

`T_SUPERFRAME_UUS = 195000` (200 ms; 1 UUS = 1.0256 µs), matching the contract and the
nRF5 source.

The RX timeout is computed to expire `BEACON_ARM_MARGIN_UUS` *before* the beacon, so the
delayed-TX setup window is guaranteed. Without that subtraction the timeout fires at
exactly the beacon time and the delayed TX always fails — a trap the nRF5 source
documents at `owner_gateway.c:189-191` and worth preserving.

MAC-only pays off here. With no `anchor_respond` in the gateway, worst-case service
latency is one GRANT — `RX_TO_TX_DLY_UUS` (2000) plus ~1.3 ms airtime plus a bounded
TXFRS wait — so `BEACON_ARM_MARGIN_UUS` can be ~5000 rather than the 8000 the nRF5
gateway needed to cover a 16.5 ms discovery stagger.

The semaphore wait is bounded rather than `K_FOREVER`. `DWT_INT_RX` includes `RXFTO` so
a timeout will normally arrive, but the unbounded-TXFRS hang already cost one bench
session; a MAC loop that can wedge takes the whole network down rather than one range.

Config is **snapshotted by value at entry**, not held by pointer. This is the aliasing
defect found in the spec C final review, and it matters more here: the gateway loop runs
indefinitely, so a console `anchor ant` would otherwise perturb a running network.

### `beacon_guard.{c,h}` — slave-side collision avoidance

#### What beacon awareness buys

Not what the phrase "gate RX on the beacon" suggests. An anchor is a responder; tags
range during CFP slots, which cover ~150 of the 200 ms superframe. CAP is tag→gateway
only, so the anchor may skip it, but that plus the beacon and guards is about 8 ms —
**windowed RX would save roughly 4% duty cycle and nothing else**, while introducing
drift tracking accurate enough that a window edge never clips a tag poll. A clipped poll
is indistinguishable from a turnaround-budget failure, which is the hardest class of bug
to attribute on this stack.

The transmit side is where the value is. A response is scheduled at a fixed delay from
the poll that triggered it, and nothing currently checks where that delay lands. Two
cases are real, and they are not uniform across traffic:

- **Discovery responses, the dominant risk.** DISCOVERY is broadcast by tags that have
  not yet joined, so it is *not* confined to a CFP slot and can arrive at any point in
  the superframe — including immediately before a beacon. `disc_resp_delay_uus(3)` is
  16.5 ms at current budgets, so the response lands far from the poll with no relation
  to the superframe structure at all. Every unjoined tag in range generates these.
- **The tail of the last CFP slot.** Inside the CFP the structure normally protects us:
  slots are disjoint from the beacon by construction, which is what the superframe is
  for. The exception is the end of the final slot — the contract's budget leaves ~5.5 ms
  of slack before the next beacon (194.5 of 200 ms), and a `POLL_RX_TO_RESP_TX_DLY_UUS`
  of 6000 UUS ≈ 6.15 ms exceeds it. A poll arriving in the last few milliseconds of slot
  12 produces a response inside the next beacon.

The second case largely closes on its own once task 1 brings the turnaround under
2500 UUS (~2.56 ms), which fits the 5.5 ms slack. The first does not close at any
turnaround, because the stagger is what carries it past the slot boundary.

Either way a hit corrupts a broadcast every node in the network depends on, which costs
far more than the one range that caused it. The gateway protects itself with
`BEACON_ARM_MARGIN_UUS`; nothing gives the slaves the mirror of it.

So: **suppress, do not window.** Before scheduling any delayed response, check whether
its TX would land inside the beacon guard window; if so, drop that one range.

#### Why not `beacon_track_core`

The tag has `beacon_track_core.{c,h}` (118 lines, explicitly Zephyr-free, host-tested at
`tests/beacon_track/`), and it was the obvious port. It is the wrong tool here. Its EMA
period estimate and ACQUIRING/TRACKING machinery exist to narrow an RX window — the thing
this design rejects — and it works in milliseconds, so using it would mean converting
between the kernel clock and DW3000 system time on every check.

The anchor needs one prediction in DW3000 system time and an integer comparison. Because
the guard re-anchors on every received beacon, drift accumulates for a single superframe:
±20 ppm crystals give ~8 µs of relative error per 200 ms, against a 500 µs guard. There
is nothing for an EMA to earn. `beacon_guard` is ~60 lines instead of 118.

#### The wrap

`dwt_readsystimestamphi32()` counts in units of 256 DTU ≈ 4.006 ns, so a 32-bit value
wraps every **~17.2 s**. Every comparison must be signed-difference arithmetic —
`(int32_t)(a - b)` — which is correct for intervals well under ~8.6 s and wrong for a
naive unsigned compare. This is the specific thing a first implementation gets wrong, and
it gets a host test.

#### Lock loss

Beacons will be missed, most often because the anchor was transmitting. The guard
extrapolates the prediction across **4 consecutive missed superframes** (~800 ms, by
which point accumulated drift is ~32 µs against a 500 µs guard — still far inside it),
then drops lock, logs once, and **stops suppressing**. Suppressing against a stale prediction would
block legitimate transmits at arbitrary times — worse than the collisions suppression
exists to prevent.

## Failure behavior

Consistent with specs A and C: the shell survives everything.

- Beacon misses its delayed slot → `dwt_forcetrxoff()`, log, and re-base the cadence on
  the current system time rather than compounding the error into the next superframe.
- GRANT TX fails or misses its slot → dropped. The tag retries through CAP backoff; the
  seat is already allocated, and a repeat JOIN from the same EUI is idempotent.
- All 12 seats occupied → `gw_core_join()` returns false, no GRANT is sent, the tag
  retries. Logged at warning level, since a persistently full network is an operational
  fact worth seeing.
- Semaphore wait times out → log and re-arm RX; the loop re-derives the time to the next
  beacon from the system clock, so a lost event costs no cadence.
- GATEWAY with `position_valid == 0` → error, no beacons, shell alive.
- Beacon lock lost on a slave → suppression disabled, logged once, ranging continues.

## Verification

### Host tests

`tests/gw_core/` — seat allocation fills slots in order; re-join with the same EUI is
idempotent and returns the same slot and address; `keepalive` refreshes a lease and an
unknown address is ignored; `release` frees a seat; `superframe_tick` expires a lease
after exactly `GW_LEASE_SF` ticks; a 13th JOIN is refused; `build_slotmap` marks free
seats `0xFFFF`; the address pool skips a value already held by a live seat.

`tests/beacon_guard/` — a TX inside the window is refused and one outside is allowed;
both edges of the guard are tested; **the 17.2 s hi32 wrap is exercised across the
boundary in both directions**; extrapolation across missed beacons stays accurate within
the guard; lock drops after the bounded miss count and suppression stops.

### On target, in order

1. `west build` clean; the boot log reports the SPI rate actually selected — 26.67 MHz,
   not 20, which is what a `26000000` DTS value would silently give. `DEV_ID` still reads
   `0xDECA0312` at that rate, proving the bus is sound before anything downstream is
   trusted.
2. **Task 1 gate:** measured turnaround under 2500 µs with the profiling block, then
   `DISC_BASE_UUS` and `POLL_RX_TO_RESP_TX_DLY_UUS` lowered accordingly and ranging
   re-confirmed against the sniffer. Work stops here if the gate is not met.
3. Gateway beacons at 200 ms; a SLAVE logs them with an incrementing `frame_counter`.
4. The tag JOINs and receives a GRANT carrying a slot index and short address; the
   gateway's next beacon shows that address in the slot map.
5. A tag ranges four anchors inside its granted slot and the sniffer shows four
   `0xE1`/`0xE4` responses with distinct source addresses.
6. A KEEPALIVE refreshes the lease; withholding one for `GW_LEASE_SF` superframes frees
   the seat and the slot map returns to `0xFFFF`.
7. **Task 6:** a slave's discovery response scheduled across a beacon is dropped rather
   than transmitted — visible as a suppression log line with no corresponding frame in
   the sniffer capture, and no corrupted beacon.

## Out of scope

- Windowed RX on slaves, and any slave-side power management. Explicitly rejected above.
- Gateway participation in ranging. If it is ever reinstated, it needs `beacon_guard`
  against its own beacon exactly as the slaves do.
- Backhaul. The contract lists it as a gateway responsibility; nothing in this project
  consumes it yet.
- Exponential backoff on the tag's CAP retries — tag-side behaviour, already specified in
  the contract.
- Multi-gateway / beacon-coverage extension (contract §Topology). Single gateway only.
- Auto-position, trilateration, and the STS-SDC responder — never part of this port.

# SLAVE mode: 802.15.4z frames + SS-TWR responder — design

**Date:** 2026-08-11
**Status:** approved, not yet implemented
**Scope:** sub-projects **B and C** of the nRF5 → Zephyr anchor port, merged

## Why B and C are one spec

The [spec A design](2026-08-10-anchor-config-console-design.md) split the port into
four: A (config/console), B (`uwb_frame_802_15_4z` port), C (SLAVE responder),
D (GATEWAY). B was scoped as a port. It is not one.

`fw-cre/firmeware_creator/Src/uwb_frame_802_15_4z.c` and
`tag_testting/src/uwb_frame_802_15_4z.c` are byte-identical except for eight lines:

```
5,12d4
< /* Define POSIX errno values not available in nRF5 SDK */
< #ifndef EMSGSIZE
< #define EMSGSIZE 90
< #endif
< #ifndef EBADMSG
< #define EBADMSG 77
< #endif
```

The headers are identical, all 114 lines. The tag deleted that shim when it moved to
Zephyr; this project would delete it for the same reason. And
`tag_testting/tests/uwb_frame/test_uwb_frame.c` — 286 lines, ten test functions,
asserting exact byte layouts for every frame type including the 0xE5–0xE9 TDMA set —
compiles against the header alone and ports unchanged.

So B is a three-file copy and a `CMakeLists.txt` line. It does not carry enough
decisions to justify its own spec → plan → review cycle, and C cannot be verified
without it. They are built together.

**The tag is the source of truth, not fw-cre.** The tag is the on-air peer and is
already stripped of nRF5-isms. Copying from it makes the anchor's file byte-identical
to the peer's, so future drift between the two is a one-line diff.

## Problem

After spec A the anchor boots, loads its identity from NVS, brings the DW3220 up and
dispatches on mode — into a stub. It has never received or transmitted a frame.

This spec makes SLAVE mode real: the anchor answers the tag's ranging polls in both
wire formats, and observes the gateway beacon without yet acting on it.

## Inherited decisions

From the spec A decomposition, unchanged:

- The anchor speaks **both** wire formats, as the nRF5 anchor does: the legacy
  hand-packed `WAVE`/0xE0 → `VEWA`/0xE1 pair and the addressed 802.15.4z
  `DISCOVERY`/0xE2 → `RESP`/0xE4 pair. Each responder ignores frames not for it.
- SLAVE **observes** the beacon (records `frame_counter` and `rx_ts`, logs them) but
  does not gate its RX windows on it.

## Findings that shape the design

Four things the sources get wrong or leave out for this target. Each is a decision
recorded here so the implementation does not re-derive it.

### 1. The Qorvo `shared_functions` helpers are not vendored

`owner_ss_twr_responder.c` and `anchor_respond.c` use `get_rx_timestamp_u64()`,
`resp_msg_set_ts()`, `waitforsysstatus()` and `UUS_TO_DWT_TIME`. These live in Qorvo's
*examples* tree (`examples/shared_data/shared_functions.{c,h}`), which the vendored
br101 module does not carry — `modules/dw3000-decadriver/` has only the driver and the
platform layer. The tag hit the same gap and solved it by defining what it needed
locally. The anchor does the same, in `src/uwb_dwtime.{c,h}`.

### 2. `UUS_TO_DWT_TIME` differs between source and peer, and the source is wrong

| Project | Value |
|---|---|
| `fw-cre` (nRF5, the port source) | `63898` |
| `tag_testting` (the on-air peer) | `65536` |
| `ESP32S3UWB` (ranges against that tag on hardware) | `65536` |

The constant scales the delayed-TX turnaround both ends must agree on. A verbatim
transcription would take a 2.6%-low value into that calculation. **This project uses
65536.**

### 3. The contract reserves `0x0000`, and spec A's default collides with it

`spec/2026-06-17-uwb-mac-protocol-contract.md`, §Topology:

> **Gateway** … Short address `0x0000`.
> **Anchors**: slaves. … Fixed short addresses `0x0001 …`.

Spec A gave `anchor_id` the range 0..3, default 0, and `anchor_respond_discovery()`
uses it directly as the src short address:

```c
uwb_frame_response_build(out, sizeof(out),
                         cfg->anchor_id,   /* my short addr (v1 = anchor_id) */
```

A default anchor would therefore transmit `src = 0x0000`, claiming the gateway's
address — which spec D puts on the same air.

This cannot be patched on one side only. The tag derives the id it polls from what the
anchor sends back (`uwb_net_runner.c:269`: *"anchor ID is taken from the low byte of
src_addr in each response"*), then writes that byte at index 10 of the WAVE poll, which
`anchor_respond_wave_poll()` filters on. The DISCOVERY src and the WAVE filter byte
must be the same number.

**Decision:** the console id stays 0..3 with default 0, as originally specified. The
short address is derived:

```c
#define UWB_ANCHOR_ADDR_BASE 0x0001u
uint16_t uwb_config_short_addr(const uwb_config_t *cfg);  /* BASE + anchor_id */
```

| console id | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| short address | 0x0001 | 0x0002 | 0x0003 | 0x0004 |
| WAVE poll byte 10 | 1 | 2 | 3 | 4 |
| id the tag reports | 1 | 2 | 3 | 4 |

The cost is a one-off between console and air. It is contained by putting the mapping
in exactly one function and by having `anchor show` print both, so the relationship is
never inferred:

```
uwb:~$ anchor show
{"mode":"slave","id":0,"short_addr":"0x0001","ant_tx":16385,...}
```

`disc_resp_delay_uus()` is the exception: it stays keyed on the **0-based** id, or
anchor 0 would wait 5500 µs instead of 2000.

### 4. A verbatim `anchor_read_cir()` hangs under the callback API

The interrupt-driven design (below) means our code runs from `dwt_isr()`'s callback.
The driver clears status *before* invoking it:

```c
/* modules/dw3000-decadriver/dwt_uwb_driver/dw3000/dw3000_device.c */
dwt_write32bitreg(dw, SYS_STATUS_ID, cia_err | SYS_STATUS_ALL_RX_GOOD);  /* :4764 */
...
dw->callbacks.cbRxOk(&LOCAL_DATA(dw)->cbData);                           /* :4791 */
```

and `SYS_STATUS_ALL_RX_GOOD` includes `DWT_INT_CIADONE_BIT_MASK`
(`deca_device_api.h:359`). So `anchor_read_cir()`'s opening
`waitforsysstatus(&s, NULL, DWT_INT_CIADONE_BIT_MASK, 0)` waits on a bit the ISR just
cleared, which will not set again until the next frame. Transcribed as written, the
anchor hangs on the first frame it receives.

The driver snapshots the pre-clear status into `cbData.status`
(`dw3000_device.c:4602`), so the callback tests the snapshot instead of polling:

```c
if (cb_data->status & DWT_INT_CIADONE_BIT_MASK) {
        dwt_readdiagnostics(&diag);      /* CIA finished within this ISR pass */
        cir_power   = (int32_t)diag.ipatovPower;
        cir_quality = diag.ipatovAccumCount;
} else {
        cir_power = 0;                   /* not ready — report absent, never wait */
        cir_quality = 0;
}
```

Reporting zero is safe: the tag uses `cir_power`/`cir_quality` to rank anchors, not to
accept or reject a range, so a frame with absent metrics costs ranking quality for one
sweep and nothing else.

## Module structure

| File | Origin | Notes |
|---|---|---|
| `src/uwb_frame_802_15_4z.{c,h}` | copied from `tag_testting/src/` | unmodified |
| `tests/uwb_frame/test_uwb_frame.c` | copied from `tag_testting/tests/` | unmodified |
| `src/uwb_dwtime.{c,h}` | new | the four missing Qorvo helpers |
| `src/disc_schedule.{c,h}` | ported from `fw-cre` | unchanged, 3 lines |
| `src/anchor_respond.{c,h}` | ported from `fw-cre` | + finding 4 |
| `src/uwb_slave.c` | rewritten | replaces the spec A stub |
| `src/uwb_config.{c,h}` | extended | `uwb_config_short_addr()` |
| `src/anchor_shell.c` | extended | `short_addr` in `anchor show` |

### `uwb_frame_802_15_4z.{c,h}` — copied verbatim

Three defects exist in this file. **None are fixed here**, deliberately:

| Defect | Why it waits for spec D |
|---|---|
| `uwb_frame_beacon_build()` does not bound `n_slots`; `need = 15 + 2*n_slots` can build a frame longer than `UWB_FRAME_MAX_LEN` (39), which `uwb_frame_is_valid()` then rejects — the builder accepts what the validator refuses | only the gateway builds beacons |
| `join_build()` and `grant_build()` call `write_hdr()` before checking `eui != NULL`, so a null EUI returns `-EINVAL` with ten bytes already written into the caller's buffer | tag- and gateway-side builders; unused here |
| `parse_beacon()` returns `-EINVAL` where `parse_discovery_response()` and `parse_multipoll()` return `-EBADMSG` for the same class of failure | beacon-observe only needs non-zero |

Keeping three projects on one identical file is worth more than three guards on paths
this spec never executes. Fixing them means fixing them in the tag too, which is a
separate change to a separate repository.

### `uwb_dwtime.{c,h}` — the missing Qorvo helpers

`UUS_TO_DWT_TIME` (65536, per finding 2), `get_rx_timestamp_u64()`,
`resp_msg_set_ts()`, and a status-poll helper. Small and self-contained; it exists so
the port's dependency on an un-vendored Qorvo example file is visible in one place
rather than smeared across two responders.

### `anchor_respond.{c,h}` — the two responders

Ported as-is apart from finding 4 and the short-address change. Both entry points keep
their shape: given a received buffer, decide whether it is theirs, and if so schedule a
delayed response.

- `anchor_respond_wave_poll()` — matches the legacy header, filters byte 10 against our
  short address's low byte, and replies `VEWA`/0xE1 carrying **that same byte** at
  index 10, both timestamps, and `(x, y)` from config. The source writes
  `cfg->anchor_id` there; it must now write the short address's low byte instead, or
  the tag drops the response — it checks `rx_buf[POS_ANCHOR_ID_IDX] != aid` against the
  id it polled with (`uwb_ss_initiator.c:275`). Filter and reply must use one value.
  `z` is stored but stays off the wire, as in spec A.
- `anchor_respond_discovery()` — `uwb_frame_is_discovery()`, then
  `uwb_frame_response_build()` with our short address as src and the tag's as dest,
  carrying the CIR metrics, transmitted at `disc_resp_delay_uus(anchor_id)`.

### `uwb_slave.c` — the RX loop

Interrupt-driven, not polled. `uwb_radio_init()` already calls
`dw3000_hw_init_interrupt()`; this adds `dwt_setinterrupt()` for the RX events and
`dwt_setcallbacks()`. Each callback gives a semaphore; the main thread blocks on it.

**`dwt_setinterrupt()` must enable the RX events only — not `TXFRS`.** `tx_delayed()`
polls `waitforsysstatus(..., DWT_INT_TXFRS_BIT_MASK, 0)` after `dwt_starttx()`. With
TXFRS unmasked the IRQ line never asserts for transmit completion, so `dwt_isr()` never
runs and never clears the bit, and the poll works as written. Enabling TXFRS would make
the ISR clear it first and hang that poll — the same failure as finding 4, in the
transmit path.

```
main thread                          ISR context
-----------                          -----------
k_sem_take(&rx_sem, K_FOREVER)  <--  cbRxOk:  snapshot CIR per finding 4
  dwt_readrxdata()                            k_sem_give(&rx_sem)
  anchor_respond_wave_poll()       cbRxTo / cbRxErr: k_sem_give(&rx_sem)
  anchor_respond_discovery()
  beacon_observe()
  re-arm RX
```

The polled alternative was rejected for a concrete reason: Zephyr's main thread runs at
priority 0 and the shell thread at 14, so a non-yielding `while (1)
{ dwt_readsysstatuslo(); }` starves the shell — the console would die the moment SLAVE
mode started. The `uwb_console_poll_config()` escape hatch that loop was built around
was already removed in spec A, so nothing is lost by dropping the structure.

RX is armed continuously (`dwt_setrxtimeout(0)`, `dwt_setpreambledetecttimeout(0)`) and
re-armed after every event and after every response TX.

### Beacon observe

New code — there is no `anchor_*` beacon listener in `fw-cre` to port. Deliberately
minimal: `uwb_frame_is_beacon()` → `uwb_frame_parse_beacon()` → log `frame_counter`,
`rx_ts`, and whether `uwb_frame_beacon_find_addr()` finds our short address in the slot
map. It records and logs; nothing reads what it records. Gating RX windows on the
beacon needs a real gateway on air to develop against, which is spec D.

## Failure behavior

Consistent with spec A: the shell stays alive through every failure so a mis-set
antenna delay or id is recoverable over USB without a reflash.

- A response TX that misses its delayed slot (`dwt_starttx` returns error) —
  `dwt_forcetrxoff()`, log at warning level, re-arm RX. One lost range, not a stuck
  radio. This is the `TX_DLY LATE` path in the source.
- A frame longer than the RX buffer, or of length zero — dropped, RX re-armed.
- CIA not finished — CIR reported as zero, per finding 4.
- The frame matches no responder — ignored silently. Most received frames are other
  anchors' responses and other tags' polls; logging each would flood the console.

## Verification

### Host tests

```powershell
gcc -Wall -Wextra -o tests/uwb_frame/test_uwb_frame.exe `
    tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
./tests/uwb_frame/test_uwb_frame.exe       # ALL TESTS PASSED, exits 0
```

The suite is the tag's, unmodified — it passing against our copy is itself the evidence
the copy is faithful.

`tests/uwb_config/` gains cases for `uwb_config_short_addr()`: id 0 → 0x0001, id 3 →
0x0004, and that no valid id can produce `0x0000` or `0xFFFE`/`0xFFFF`.

`disc_schedule.c` is kept a separate module rather than folded into `anchor_respond.c`
precisely so it stays host-testable: it is the arithmetic that keeps four anchors from
transmitting on top of each other, and `anchor_respond.c` cannot be compiled on the
host because it pulls in `deca_device_api.h`.

### On target, in order

1. `west build -b ancla_esp32s3/esp32s3/procpu` — clean, no warnings.
2. One anchor at `id 0`, tag powered: the tag's DISCOVERY sweep finds it and adds it to
   its anchor pool as anchor 1.
3. The tag ranges it and prints a distance. This is the gate that matters — it is the
   only step that proves the turnaround timing, the antenna delays and
   `UUS_TO_DWT_TIME` are all right at once.
4. Anchor log shows a received beacon with a plausible `frame_counter`, if a gateway is
   on air. Skipped if not — no gateway exists until spec D.
5. A second anchor at `id 1`: both answer one DISCOVERY, and the tag pools two anchors.
   This is the only test that exercises `DISC_SLOT_UUS`, and the reason two boards are
   needed.

Step 5 failing while step 3 passes means the stagger is too tight — `DISC_SLOT_UUS` is
marked *TUNE ON HARDWARE* in the source and 3500 µs is an estimate, not a measurement.

## Out of scope

- GATEWAY: beacon TX, `gw_core` CAP seats, JOIN/GRANT/KEEPALIVE/RELEASE. Spec D.
- Gating SLAVE RX windows on beacon timing. Spec D, per the inherited decision.
- `0xE3` MULTI-POLL. The module carries the codec; the contract defers the mode, and
  the tag does not send it.
- The three frame-module defects above.
- Auto-position / trilateration, the STS-SDC responder, and the nRF5 `simple-rx` and
  `ss-twr-cal` modes — not part of this port.
- Antenna-delay calibration. `ant_delay_tx`/`ant_delay_rx` are set at the console and
  used as given; deriving them is not this spec's job.

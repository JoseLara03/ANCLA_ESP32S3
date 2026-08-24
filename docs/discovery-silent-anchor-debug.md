# Debugging a surveyed anchor that stops answering DISCOVERY

**Symptom.** With three anchors surveyed by `apos` and a gateway running, a tag
discovers all three, ranges them, and reports a plausible position. After a
power cycle at least one anchor no longer appears in the tag's anchor set. The
affected board is not always the same one.

**Status: root cause not yet identified.** This document is the evidence-gathering
step, not a fix. It describes the debug image, what each line means, and the
decision table that maps a capture onto one of the candidate causes. Read §4
before changing any constant — several candidates are indistinguishable without a
capture, and the logging as it stood could not separate them, which is why this
image exists.

## 0. Why a separate image

Every module on the ranging path registers its own log level explicitly, e.g.
`LOG_MODULE_REGISTER(anchor_respond, LOG_LEVEL_INF)`. A per-module level is a
**hard compile-time cap**: the `LOG_DBG` line that reports a beacon-guard
suppression is not filtered at runtime, it is not in the binary at all. Neither
raising `CONFIG_LOG_DEFAULT_LEVEL` nor `log enable dbg anchor_respond` at the
shell can bring it back — `CONFIG_LOG_RUNTIME_FILTERING` is already on in
production and can only ever narrow what a build compiled in.

`CONFIG_ANCLA_RANGING_DEBUG` (`debug.conf`) routes those registrations through
`ANCLA_LOG_LEVEL` (`src/uwb_debug.h`) and compiles in the extra diagnostics.

The debug image changes **no frame, no timing constant and no gate**, so a debug
board is an ordinary peer on air and can be mixed with production boards. It is
still not a deployment image: the per-frame console traffic and the heartbeat's
extra SPI read both cost time on the SLAVE loop.

## 1. Build and flash

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build_dbg -- "-DEXTRA_CONF_FILE=debug.conf"
west flash -d build_dbg
west espressif monitor -p COM5
```

Quote the `-D` argument. PowerShell splits an unquoted
`-DEXTRA_CONF_FILE=debug.conf` at the dot and the build fails looking for a file
named `debug`.

Flash the debug image to **all three anchors** and capture all three consoles at
once, each to its own file. The fault is a comparison between boards — a single
console cannot show that one anchor behaves differently from its neighbours.

Leave the gateway and the tag on the production image. Nothing in this
investigation is read from them.

## 2. What to capture

1. Bring up the gateway and let the beacon settle.
2. Start all three anchor captures.
3. Power-cycle all three anchors — the trigger, so the boot banners land inside
   the capture.
4. Bring up the tag and leave it for two minutes.
5. Repeat from step 3 until a run reproduces the fault. Keep a good run too: the
   working capture is the reference the broken one is read against.

## 3. The lines

### Boot banner — `src/main.c`

```json
{"mode":"slave","id":1,"short_addr":"0x0002","ant_tx":16385,"ant_rx":16385,
 "x":1.00,"y":0.00,"z":0.00,"pos_valid":1,"disc_delay_uus":5500}
```

`disc_delay_uus` is derived from `id`, not stored. **The three consoles must show
three different values here.** Two boards sharing an `id` answer the tag's
broadcast in the same stagger slot and collide on air, and neither board can
detect that locally — this line is the only place it becomes visible.

### Per-DISCOVERY verdict — `src/anchor_respond.c`

```json
{"disc":{"tag":"0x0100","id":1,"seq":37,"delay_uus":5500,"verdict":"sent",
  "to_beacon_uus":178432,"locked":1,"pwr":-92,"q":1024}}
```

- The line existing at all proves the frame was received **and** accepted as a
  DISCOVERY. Its absence means the frame never arrived, or was malformed.
- `verdict` separates the four causes that used to share one silence: `sent`,
  `suppressed` (beacon guard refused), `missed_slot` (`dwt_starttx()` rejected
  the scheduled time), `no_txfrs` (armed and never completed).
- `to_beacon_uus` is signed UUS from this response to the beacon the guard is
  predicting. The forbidden window is `guard + occupancy + guard` = 4500 UUS wide
  against a 3500 UUS slot pitch, so it can catch one or two adjacent ids but
  never all three.
- `seq` is consumed **before** the suppression check, so a gap in this board's
  sequence numbers on a sniffer capture lines up one-for-one against these lines.

### Per-second heartbeat — `src/uwb_slave.c`

```json
{"slave_hb":{"id":1,"addr":"0x0002","pos_valid":1,"rx_ok":31,"rx_err":2,
  "wave":18,"disc":5,"beacon":5,"apos":0,"other":1,
  "sys":"0x00000002","locked":1,"misses":0}}
```

Counts are **per second, not cumulative**. The line is emitted whether or not a
frame arrived, which is the point: an all-zero line separates "this board is
deaf" from "this board hears the tag and declines to answer" with no sniffer and
no tag-side visibility, and it keeps printing while the board is deaf. Per-frame
logging alone goes silent in exactly the failure it was added to catch, and that
silence is indistinguishable from a wedged thread or a closed console.

`sys` is `SYS_STATUS_LO`, the receiver's own account of itself.

`disc` counts the 0xE2 type byte, while a `{"disc":...}` verdict line also
requires the frame to pass `uwb_frame_is_discovery()`. **`disc` counting up with
no verdict lines means malformed DISCOVERY frames**, not refused ones.

### Other lines

- `{"wave_other":{"polled_id":2,"our_id":1}}` — a WAVE poll addressed to a
  different anchor. Proof that this board's receiver works and the tag is polling
  somebody.
- `{"rx_other":{"type":"0x..","plen":..,"src":"0x...."}}` — a frame no responder
  claimed.
- `WAVE poll refused — no surveyed position` — already in production, rate
  limited to one line per 10 s. This gates **WAVE** only;
  `anchor_respond_discovery()` is deliberately not gated on position, so an
  unpositioned anchor still answers DISCOVERY and then never answers a poll.

## 4. Decision table

Read the broken anchor's capture against the two working ones.

| Observation | Cause it points at | Next step |
|---|---|---|
| Two boards print the same `disc_delay_uus` | Duplicate `anchor id` — both answer in the same stagger slot and collide. The survey pushes coordinates by EUI-64, so ids are never reconciled by it | `anchor id` on one board, then `kernel reboot cold` |
| Heartbeat prints, all counters 0, `sys` unchanging | Wedged DW3220. Already a known failure on this hardware ("transmits one frame per boot then goes silent"); prime suspect is the supply sagging under the PA at first TX | Swap the board. Symptom follows the board = hardware; follows the `id` = timing or config |
| No heartbeat at all | Main thread blocked, or the board is not running | Check the console is attached. A bounded wait was the fix for a previous freeze of exactly this shape |
| `beacon` counts up, `disc` stays 0 | The anchor hears the gateway but not the tag. One-way link budget, not firmware | §5 |
| `disc` counts up, no `{"disc":...}` lines | Malformed DISCOVERY reaching this board (`uwb_frame_is_discovery()` rejects it) | Compare `plen` against `UWB_FRAME_LEN_DISC`; confirm the FCS is being subtracted |
| `verdict:"suppressed"` on one id repeatedly while the others pass | Beacon-guard suppression. The tag's DISCOVERY is beacon-synchronised, so `to_beacon_uus` should read ~170000+; a small value means the guard's prediction is stale | Read `misses` and `locked` on the same line |
| `verdict:"missed_slot"` | `DISC_BASE_UUS`/`DISC_SLOT_UUS` too tight for this board's RX-to-TX path | Confirm the SPI bus actually came up at fast rate — the boot log's rate line is not evidence |
| `verdict:"no_txfrs"` | Armed and never transmitted: the `dwt_starttx()` HPDWARN race | Compare `TX_COMPLETE_TIMEOUT_MS` (18 ms) against `disc_resp_delay_uus(id)` |
| `verdict:"sent"` every round on all three, tag still sees two | **Not an anchor fault.** Either the response never reaches the tag, or the tag discards it | §5, then the tag's `anchor_pool` / `selected[]` scoring in `uwb_net_runner.c` |

## 5. If the anchor says `sent` and the tag disagrees

Two known leads, both about the link, neither yet resolved:

- **The ANCLA boards transmit ~25 dB below free-space expectation.** Measured
  with a sniffer 0.5 m from the gateway: beacons at −84 dBm where ~−57 dBm is
  predicted, while the tag at ~3 m matched physics. That ceiling is why early
  range tests died at 2–3 m. `dwt_setfinegraintxseq(0)` before
  `dwt_setlnapamode()` was applied, but a controlled before/after was never run.
  A `verdict:"sent"` the tag never hears is exactly this shape, and it would hit
  the **farthest** anchor first — which fits "not always the same board".

- **`src/uwb_phy.h` now carries a reduced TX power**,
  `UWB_PHY_TXCONFIG_INITIALIZER` at `0xfafafafa` where it was `0xffffffff`. Per
  byte the coarse gain is unchanged and the fine gain drops 63 → 58, roughly
  1.25 dB. It is committed as of this branch, but **it has never been measured**.
  Lowering drive on a link already 25 dB under budget and failing at 2–3 m looks
  backwards, and it is only defensible under one hypothesis: that the DW3220
  overdrives the QM14070 PA into compression, so backing off 1.25 dB raises net
  radiated power instead of lowering it. That hypothesis is untested — the same
  controlled before/after that was never run for `dwt_setfinegraintxseq(0)`
  would settle both at once. If a range test gets *worse* after this branch,
  this line is the first thing to revert.
  **Establish which value is on each board before reading any capture** — three
  anchors flashed at different times may not agree, and boards flashed before
  this commit are still at `0xffffffff`.

`pwr` and `q` in the `{"disc":...}` line are the anchor's Ipatov CIR metrics for
the **tag's** transmission, so they measure the tag→anchor direction only. The
anchor→tag direction is not observable from these consoles at all; that needs the
DWM3001CDK sniffer.

## 6. Returning to production

```powershell
west build -b ancla_esp32s3/esp32s3/procpu --pristine -d build
west flash -d build
```

The production image is behaviourally unaffected by this work: with the flag off,
`ANCLA_LOG_LEVEL` resolves to `LOG_LEVEL_INF` — exactly what each module
hard-coded before — and every added diagnostic compiles out. Verified by
`strings build/zephyr/zephyr.elf | grep slave_hb` returning nothing.

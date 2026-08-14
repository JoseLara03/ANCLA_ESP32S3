# Antenna-delay calibration — operator procedure

Date: 2026-08-14

Trims each anchor's `ant_delay_tx` against a Qorvo-reference node, then
cross-checks anchor-to-anchor. See `CLAUDE.md` ("URGENT next work" §1) for why
this matters: every board still runs the factory seed `16385/16385`, and until
it is trimmed no coordinate this system emits means anything.

**Status of this procedure at time of writing:** implemented, built, and code
reviewed (branch `cal/antenna-delay`). It has **not yet been run against real
hardware** — no board has actually been calibrated with it. This document is
the runnable procedure for whoever does that next. Do not read the presence of
this document as evidence that a board is calibrated; check `anchor show` for
`ant_tx != 16385` instead.

## 1. Prerequisites

### 1.1 The DWM3001CDK reference node

The calibration procedure needs one external device whose antenna delay is
already known good, to compare every ANCLA anchor against. This is a
DWM3001CDK running Qorvo's stock `ss_twr_responder` example, modified as
follows:

- **PHY matched to `src/uwb_phy.h`**: channel 5, PLEN_1024, PAC32, code 9,
  850 kbps, SFD_IEEE_4Z, STS and PDoA off. If the DWM's PHY does not match
  exactly, the exchange will not decode at all (see Troubleshooting,
  `-ENODATA`).
- **Antenna delay loaded from OTP**, not the example's hardcoded `16385`. The
  whole point of using this board as a reference is that its delay is
  factory-known; leaving the example's default defeats that.
- **`POLL_RX_TO_RESP_TX_DLY_UUS` raised from 450 to 2000.** At PLEN_1024 the
  preamble alone takes ~1.05 ms, so the example's stock 450 uus turnaround is
  physically impossible — the response would have to leave before the poll's
  preamble finished. ANCLA's own responder (`anchor_respond.c`) already turns
  around at 2000 uus; matching it here is what lets `cal_initiator.c` use a
  single RX window for both peer types.

This board is prepared once and reused for every anchor calibrated afterward.

### 1.2 Hardware needed for the full procedure

- The three ANCLA anchors to be calibrated, each able to take the `cal`
  firmware image.
- The prepared DWM3001CDK reference node (§1.1).
- A tape measure (or equivalently precise fixed reference — see §3 on why
  precision here matters).
- Optionally, a second ANCLA anchor already running the `cal` image, for the
  `cal peer` cross-check in §5 (three anchors calibrated in sequence can all
  cross-check each other; no extra hardware is required beyond the three
  anchors themselves).

## 2. Build and flash the calibration image

The calibration image is a **separate build**, not a runtime mode: it is
selected by an `EXTRA_CONF_FILE` overlay (`cal.conf`) on top of the normal
`prj.conf`, gated by `CONFIG_ANCLA_CAL_MODE` in the project `Kconfig`.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
west flash
west espressif monitor -p COM5
```

`--pristine` is worth doing explicitly the first time, since switching between
the production build and the cal build changes `Kconfig` symbols that a
non-pristine build will not always pick up cleanly.

On boot you should see the ordinary boot banner, then a JSON status line:

```
{"status":"listening","mode":"cal","id":<n>,"short_addr":"0x000n","ant_tx":16385,"ant_rx":16385}
```

and a live `uwb:~$` prompt. In this state the board is an ordinary WAVE
responder — it answers `0xE0` polls exactly as the production image would —
until you run a `cal` command, at which point it becomes a temporary SS-TWR
*initiator* for the duration of that one batch, then reverts to responder duty
automatically.

**Do not build the cal image for a deployed anchor.** `cal.conf` also disables
WiFi/MQTT, so a board running it will not join the gateway's beacon protocol
or publish anything. It exists only for the calibration bench session; reflash
the production image (`west build --pristine`, no `EXTRA_CONF_FILE`) before
returning a board to service. The calibrated `ant_delay_tx` value survives
that reflash — it lives in NVS under `anchor/`, which reflashing the
application does not erase.

## 3. Physical setup

The calibration solves for antenna delay from a measured round-trip time and a
*declared* true distance. **Any error in the declared distance enters the
result 1:1** — a 10 mm tape error is a 10 mm calibration error, with no
averaging or filtering able to remove it, because it is a bias on the ground
truth, not noise on the samples. Measure carefully and record what you
measured.

The distance to declare is **antenna phase-centre to antenna phase-centre**,
not enclosure edge to enclosure edge, not mounting-hole to mounting-hole. For
the ANCLA board and the DWM3001CDK, use each board's documented antenna
location; when in doubt, measure to the centre of the printed antenna element
itself.

Setup checklist, all of which the Task 4 hardware gate calls out explicitly:

- Reference distance **≥ 2 m**, tape-measured, antenna phase-centre to
  antenna phase-centre.
- Clear line of sight between the two antennas — no furniture, bodies, or
  equipment in the direct path.
- Equal height between the two antennas.
- Matched antenna orientation (both boards in the same physical attitude —
  e.g. both flat, both vertical — not tilted relative to each other).
- Clear of metal near either antenna (large metal surfaces distort the near
  field and the reported delay).
- **All tags powered off.** A tag transmitting DISCOVERY/WAVE polls during a
  calibration batch can corrupt individual samples (though `cal_filtered_mean`
  will reject a handful of gross outliers — see Task 1's outlier rejection —
  it cannot recover a batch dominated by interference).

## 4. Per-anchor calibration: `cal ref`

With the DWM3001CDK powered and running as described in §1.1, and the anchor
under test flashed with the cal image and positioned per §3:

```
cal ref <mm>
```

where `<mm>` is the tape-measured phase-centre-to-phase-centre distance in
millimetres (e.g. `cal ref 2000` for 2.000 m).

This takes 128 SS-TWR exchanges with the reference node
(`CAL_PEER_REFERENCE = 0xFF`, which the DWM's stock responder answers because
it does not filter on peer id — only ANCLA's own responder does), rejects
outliers, solves a new `ant_delay_tx`, applies it to the radio immediately,
and — because `cal ref` always persists — writes it to NVS.

Expected sequence:

```
cal ref 2000
```
— first run reports a large `error_mm` (this is the measurement of the
uncalibrated factory-default offset) and `ok: ant_tx 16385 -> <new>`.

```
cal ref 2000
```
— second run, same distance, no reboot in between: `|error_mm|` should now be
**under 15 mm**. The delay was applied hot by the first run, so this second run
is reading the residual after correction, not a fresh uncalibrated exchange.

Verify persistence:

- `anchor show` reports the new `ant_tx`; `ant_rx` is unchanged at `16385`
  (only TX is ever trimmed — see CLAUDE.md's "Only the sum `ant_tx + ant_rx`
  is observable" note for why RX must stay pinned).
- `kernel reboot cold`, then `anchor show` again: the new `ant_tx` must still
  be there. If it reverts to `16385`, the `uwb_store_save_ant()` write failed
  — treat this as a calibration failure and re-run `cal ref`.

Repeat this whole section for each of the three anchors in turn (one anchor at
a time against the DWM3001CDK; the others can be powered off or elsewhere —
they play no part in this step).

## 5. Cross-check: `cal peer`

`cal ref` calibrates each anchor independently against the reference node.
`cal peer` is **independent evidence** that the calibration generalizes: it
ranges two already-calibrated ANCLA anchors against each other, on pairs that
were never used to calibrate anything.

With all three anchors calibrated (§4 done for each), place two of them a
tape-measured distance apart (again, phase-centre to phase-centre, same
physical-setup discipline as §3) and, from one anchor's console:

```
cal peer <id> <mm>
```

`<id>` is the other anchor's 0-based console id (`anchor id`, not the wire
address); `<mm>` is the tape-measured distance. `cal peer` never persists —
it only reports.

**Acceptance: `|error_mm| < 30` on all three anchor pairs** (0↔1, 0↔2, 1↔2).
Run it in both directions where convenient (e.g. `cal peer 1 <mm>` from anchor
0 and `cal peer 0 <mm>` from anchor 1) as a sanity check that both boards'
responder duty survived their own calibration batches.

If `cal ref` passed (§4's `< 15 mm` residual) on every board individually but
this cross-check fails by a similar-sized error on *every* pair, suspect that
the DWM3001CDK's OTP delay was not actually loaded (i.e. it is still running
the example's hardcoded `16385`) — that failure mode is invisible in §4
(a single board self-consistently "calibrates" against a wrong-but-fixed
reference) and only shows up here, in the anchor-to-anchor comparison.

## 6. Troubleshooting

| Error | Where it comes from | Likely physical cause |
|---|---|---|
| `error: too few valid responses` (`-ENODATA`) | `run_batch()` in `src/cal_run.c`: fewer than a quarter of the 128 attempted exchanges produced a decodable response | Peer is powered off or out of range; peer's PHY does not match `src/uwb_phy.h` (wrong channel/PRF/preamble/SFD); wrong peer id (`cal peer <id>` pointed at an anchor id that isn't actually there, or the DWM3001CDK isn't answering `0xFF`); no line of sight |
| `error: solved ant_tx out of range ... NOT applied` (`-ERANGE`) | `cal_solve_tx_delay()` in `src/cal_solve.c`, surfaced by `cmd_ref()` in `src/cal_shell.c` | The declared reference distance is wrong (wrong tape measurement, wrong units — the command takes millimetres, not metres); the peer answering is not actually the reference node (e.g. another ANCLA anchor answered instead of the DWM); a multipath reflection is present and biasing the whole batch mean, not just a few samples. **Do not raise `CAL_TX_DLY_MIN`/`CAL_TX_DLY_MAX`, or the `< CAL_MAX_SAMPLES/4` floor, to make this pass** — they are diagnostics that a bad measurement is being taken, not conservative defaults to relax |
| `error: the ranging loop did not answer` (`-ETIMEDOUT`) | `cal_run_execute()` in `src/cal_run.c`: the responder/initiator loop never completed a batch within 30 s | The cal image's main loop is stuck or was never reached (check for the `{"status":"listening",...}` boot line); extremely unlikely in normal operation since a fully-failing 128-sample batch is bounded at ~6 s of timeouts — a full 30 s stall points at a firmware or radio fault, not a bad measurement, and is worth a `kernel reboot cold` before re-running |
| `error: a calibration batch is already running` (`-EBUSY`) | `cal_run_execute()` rejecting a second command | A previous `cal ref`/`cal peer` is still in flight (up to ~6 s for a fully failing batch); just wait and retry |

## 7. What this procedure does not fix

`cal ref` and `cal peer` only ever touch `ant_delay_tx` on the board they run
on. If, after all three anchors pass the `< 30 mm` cross-check in §5, the
production system's `0xEA` position residual (see `pos_sink`'s JSON log line)
is still large, the remaining error is **anchor geometry**, not antenna
delay — i.e. the `anchor pos` coordinates hand-entered for the deployment are
wrong or the anchors' true positions were not what was declared. That is
CLAUDE.md's URGENT item 2 (auto-positioning), and is explicitly out of scope
for this calibration procedure.

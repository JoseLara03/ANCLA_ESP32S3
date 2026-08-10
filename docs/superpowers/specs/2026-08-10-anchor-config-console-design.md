# Anchor config, console and mode selection — design

**Date:** 2026-08-10
**Status:** approved, not yet implemented
**Scope:** sub-project A of the nRF5 → Zephyr anchor port (see *Decomposition* below)

## Problem

`ANCLA_ESP32S3` today contains a single DW3220 bring-up smoke test (`src/main.c`).
The anchor firmware it needs to become already exists for nRF5 SDK in
`fw-cre/firmeware_creator`, as `owner_ss_twr_responder.c` (SLAVE) and
`owner_gateway.c` (GATEWAY) plus their dependencies. Porting those requires, first,
the thing both of them read on their very first line: a persistent per-anchor
configuration, and a way for an operator to set it over the serial console.

The deployment is four anchors. Each needs a distinct ranging id (default 0) and a
mode — SLAVE or GATEWAY — chosen at the console rather than at compile time, so a
single firmware image flashes to every unit.

## Decomposition

The full port is four specs, built in order. This document covers **A** only.

| | Spec | Depends on | Hardware check that closes it |
|---|---|---|---|
| **A** | Config, NVS, shell, mode dispatch, shared radio bring-up | — | `anchor id 2` → cold reboot → still 2 |
| **B** | `uwb_frame_802_15_4z` port + host tests | — | host tests pass off-target |
| **C** | SLAVE: SS-TWR responder (WAVE + DISCOVERY) + beacon observe | A, B | a tag ranges to the anchor |
| **D** | GATEWAY: beacon TX + `gw_core` CAP seats | A, B, C | slave logs beacons; tag JOINs, gets a GRANT |

A and B are independent; B may be done at any point before C.

Decisions already fixed for C and D, recorded here because they shape A:

- GATEWAY carries the **full CAP seat protocol** — BEACON with a 12-slot CFP map,
  plus JOIN→GRANT / KEEPALIVE / RELEASE and `gw_core.c` lease aging. Not a bare
  periodic sync broadcast.
- SLAVE in its first iteration **observes** the beacon (records `rx_ts` and
  `frame_counter`, logs them) but does not yet gate its RX windows on it.
- The anchor speaks **both** wire formats, as the nRF5 anchor does: the legacy
  hand-packed `WAVE`/0xE0 → `VEWA`/0xE1 pair and the addressed 802.15.4z
  `DISCOVERY`/0xE2 → `RESP`/0xE4 pair. Each responder ignores frames not for it.

## Module structure

Five modules in `src/`, replacing the current smoke-test `main.c`, plus one
constants header (`uwb_phy.h`, the fixed PHY contract).

### `uwb_config.{c,h}` — config data and validation

Pure C, no Zephyr headers, so the validation is host-testable.

This is deliberately **not** the nRF5 `uwb_config_t`. That struct carries eleven PHY
fields (`channel`, `preamble_len`, `pac_size`, `preamble_code`, `sfd_type`,
`data_rate`, `sts_mode`, `sts_length`, …) because the nRF5 firmware doubled as a PHY
bench rig. This anchor's PHY is fixed by the network contract — channel 5, PLEN-1024,
PAC32, code 9, 850 kbps, SFD_IEEE_4Z, STS off — so those become compile-time
constants in `uwb_phy.h`, not runtime settings. Likewise dropped: `role`,
`num_anchors`, `coord_anchor_count`/`coord_anchor_ids`, which belong to the
initiator and auto-position paths that are out of scope.

What remains configurable:

| Field | Range | Default | Used by |
|---|---|---|---|
| `mode` | `SLAVE` \| `GATEWAY` | `SLAVE` | boot dispatch |
| `anchor_id` | 0..3 | **0** | C: poll filter, response stagger. D: beacon src |
| `ant_delay_tx` | uint16 | 16385 | radio bring-up |
| `ant_delay_rx` | uint16 | 16385 | radio bring-up |
| `x`, `y`, `z` | float | 0.0 | C: self-reported coords in the VEWA response |
| `position_valid` | bool | false | D: GATEWAY refuses to beacon without coordinates |

`UWB_MAX_ANCHORS` is 4, matching the deployment and the nRF5 constant.

Interface: a defaults initializer, a per-field validator returning success/failure
without mutating on failure, and an accessor for the single active instance. The
instance is written only during boot (settings load) and by shell commands; the UWB
loop treats it as read-only.

### `uwb_store.{c,h}` — persistence

Zephyr `settings` over NVS, bound to the board's existing `storage_partition`
(192 K at 0x7b0000, already present in `partitions_0x0_amp_8M.dtsi` — **no board DTS
change is required**).

Per-field keys under one subtree, rather than a single struct blob:

```
anchor/mode      u8
anchor/id        u8
anchor/ant_tx    u16
anchor/ant_rx    u16
anchor/pos       struct { float x, y, z; u8 valid; }
```

A blob for the whole struct is fewer lines today, but specs C and D will add fields,
and a blob forces a version byte plus a wipe-to-defaults on every layout change.
Per-field keys silently ignore keys they don't recognize and leave unknown fields at
their defaults, which is what we want across A→D.

Two operations: load-all at boot, and save-one after a shell write.

### `anchor_shell.c` — console commands

Root command `anchor`, namespaced alongside Zephyr's built-in `kernel`, `device` and
`flash`.

```
anchor show                    JSON dump of the active config
anchor id <0..3>               set ranging id
anchor mode <slave|gateway>    set boot mode
anchor pos <x> <y> <z>         set coordinates; implies position_valid = 1
anchor ant <tx> <rx>           set antenna delays
anchor reset                   restore defaults and persist
```

`show` emits JSON because the nRF5 firmware's status output was machine-parsed
(`{"status":"listening","mode":"ss-twr","anchor_id":0}`); keeping that shape means
existing tooling still reads it.

```
uwb:~$ anchor id 2
ok: anchor_id=2 (saved) — reboot to apply
uwb:~$ anchor show
{"mode":"slave","id":2,"ant_tx":16385,"ant_rx":16385,"x":0.00,"y":0.00,"z":0.00,"pos_valid":0}
```

Every setter validates, then persists immediately, then reports. There is no separate
`save` command: auto-persist is fewer keystrokes and removes the "I set it but forgot
to save" failure mode. Reverting means setting the value back, or `anchor reset`.

The console is the **native USB-JTAG serial** peripheral, not UART0. The board's
`chosen` node already selects it (`zephyr,console` and `zephyr,shell-uart` both
`&usb_serial`), and `serial_esp32_usb.c` implements the full interrupt-driven UART
API (`fifo_read`, `irq_rx_ready`, `irq_callback_set`) with `SERIAL_ESP32_USB`
selecting `SERIAL_SUPPORT_INTERRUPT` — so the shell backend works over it with no
fallback needed.

### `uwb_radio.{c,h}` — shared DW3000 bring-up

`owner_ss_twr_responder.c` and `owner_gateway.c` each open with the same ~25-line
prologue, duplicated verbatim. It is factored out once here:

reset → `dwt_probe()` → `dwt_initialise()` → `dwt_configure()` with the fixed PHY →
`dwt_configciadiag(DW_CIA_DIAG_LOG_ALL)` → `dwt_configuretxrf()` → antenna delays from
config → `dwt_setlnapamode()`.

Two ordering constraints carry over and must not be reshuffled by the refactor:

- **`dwt_initialise()` must be the first register access after `dwt_probe()`.** It is
  what assigns `dw->priv`; anything before it — including `dwt_checkidlerc()` or
  `dwt_configure()` — faults with `EXCCAUSE 28 / VADDR 0x24`. Already documented in
  `docs/dw3000-zephyr-port.md`; restated here because this refactor is exactly where
  it would get broken.
- **`dwt_configciadiag()` must come after `dwt_configure()`**, or every diagnostic
  register reads zero — which spec C needs for the CIR power/quality carried in the
  DISCOVERY response.

Platform mapping from nRF5 to the vendored br101 module:

| nRF5 | Zephyr / br101 |
|---|---|
| `reset_DWIC()` | `dw3000_hw_reset()` |
| `port_EnableEXT_IRQ()` / `port_DisableEXT_IRQ()` | `dw3000_hw_init_interrupt()` / `dw3000_hw_interrupt_disable()` |
| `Sleep(2)` | `k_msleep(2)` |
| `port_set_dw_ic_spi_slowrate()` / `_fastrate()` | no equivalent — dropped; SPI speed is fixed at 8 MHz in the DTS |
| `test_run_info()` | `LOG_INF()` |

### `main.c` — boot and dispatch

Settings init → config load → radio bring-up → dispatch on `mode` to
`uwb_slave_run()` or `uwb_gateway_run()`.

In spec A both entry points are stubs that log which mode was entered and sleep.
They are the seam specs C and D fill in.

The **main thread is the UWB thread** — no additional thread is created. The shell
already runs on its own thread, and because config is read-only after boot, the
ranging hot path needs no locking. This is the resolution of the threading question:
the nRF5 code polled the console every loop iteration
(`uwb_console_poll_config()`) and dropped to a config REPL on demand; that escape
hatch is unnecessary once the shell is preemptible, so identity and mode changes are
persisted immediately and take effect on reboot.

### `prj.conf` additions

`SHELL`, `FLASH`, `FLASH_MAP`, `NVS`, `SETTINGS`, `SETTINGS_NVS`,
`CBPRINTF_FP_SUPPORT` (for `%f` in `anchor pos` / `anchor show`), `REBOOT` (for
`kernel reboot cold`), and a `MAIN_STACK_SIZE` bump, since the main thread now runs
the mode loop.

## Failure behavior

- `settings_load()` fails → log a warning, run on defaults. Boot is never blocked.
- A stored value outside its range → that one field falls back to its default and is
  logged; the other stored fields are kept.
- Radio bring-up fails → log the error and stop entering the mode loop, but **leave
  the shell alive**, so a mis-set antenna delay or a wiring fault is recoverable over
  USB without a reflash.

The last point is a deliberate departure from the nRF5 code, which spins forever in
`while (1) {}` on `INIT FAILED` / `CONFIG FAILED` and requires a reflash to recover.

## Verification

### Host tests — `tests/uwb_config/test_uwb_config.c`

Plain-gcc `main()` with a `CHECK` macro, returning non-zero on failure — the
pattern both sibling projects already use (`tests/anchor_id/test_anchor_id.c` in
`ESP32S3UWB`, `tests/uwb_frame/test_uwb_frame.c` in the tag). No twister and no
`native_sim`: `uwb_config.c` has no Zephyr dependencies, so a single `gcc` command
compiles and runs it.

Cases:

- defaults are the values this document claims, in particular `id == 0` and
  `mode == SLAVE`
- `id` accepts 0..3 and rejects 4 and above, leaving the previous value intact
- `mode` parses `slave` and `gateway`, rejects anything else
- `pos` sets `position_valid`

### On-target, in order

1. `west build -b ancla_esp32s3/esp32s3/procpu` — clean, no warnings
2. Shell prompt appears over USB-JTAG; `anchor show` reports `"mode":"slave","id":0`
3. `anchor id 2` → `kernel reboot cold` → `anchor show` reports `"id":2`
4. `anchor mode gateway` → reboot → the log shows the GATEWAY stub entered, not SLAVE
5. Radio bring-up after config load logs the DEV_ID and a successful `dwt_configure()`
   — i.e. the existing smoke test still passes, now through `uwb_radio.c`

Criterion 5 is the one that matters most: it proves the refactor of the bring-up
prologue preserved the `dwt_probe()` → `dwt_initialise()` ordering.

## Out of scope

Named explicitly so they are not smuggled in:

- Any UWB frame TX or RX. Spec A brings the radio up and stops.
- `uwb_frame_802_15_4z` (spec B), the responders (spec C), `gw_core` and the beacon
  (spec D).
- Auto-position / trilateration (`owner_auto_position*.c`, `position_solver*.c`) and
  the STS-SDC responder — not part of this port.
- Runtime PHY configuration. The PHY is a fixed contract constant.
- The nRF5 `simple-rx` sniffer and `ss-twr-cal` modes.

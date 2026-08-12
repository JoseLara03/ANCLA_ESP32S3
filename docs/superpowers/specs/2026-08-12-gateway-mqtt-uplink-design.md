# Gateway WiFi + MQTT uplink

**Date:** 2026-08-12
**Status:** design approved, not yet implemented
**Sub-project:** E2 of E1+E2 (see the E1 spec, `2026-08-11-pos-frame-design.md`)
**Branch:** `feat/mqtt-uplink` off `feat/pos-frame` (E1 is not on `master`)

---

## 1. Motivation

E1 landed the `0xEA` POS frame end to end: tags transmit their own solved
position, and the gateway decodes it in `dispatch()` and hands it to
`pos_sink_publish()`, which logs one JSON line to the console. That is the whole
sink today — the data reaches a USB console and stops there.

E2 completes the gateway's job. The fixes leave the board over WiFi as MQTT
messages, so something other than a terminal can consume them.

E1's sink was written for exactly this: its header already says *"E2 replaces the
body with an MQTT publish to `testtopic/1/position` and changes nothing else."*
That holds — `src/uwb_gateway.c` is not edited by this spec.

## 2. Scope

**In scope:** WiFi association, MQTT connection, publishing decoded fixes to
`testtopic/1/position`, a stubbed retained publish to `testtopic/1/anchors`,
NVS-backed network configuration with a `net` console tree, and the thread
priority work that keeps all of it away from the beacon.

**Out of scope:** TLS (see §8), SNTP/wall-clock time, anchor position survey
(the anchors topic stays stubbed), MQTT subscribe / downlink commands, and any
change to the tag firmware. The tag is untouched by E2.

## 3. Threading model

### 3.1 "Isolated core" — correcting the E1 carry-forward

The original request asked for the WiFi work to run on its own isolated core.
That is not achievable, for the reason E1 recorded: Zephyr's ESP32 WiFi driver
declares `depends on !SMP` (`drivers/wifi/esp32/Kconfig.esp32:8`), so
`CONFIG_SMP` + `k_thread_cpu_pin()` will not build with WiFi enabled, and the
AMP alternative cannot host the network stack because the WiFi blobs bind to
procpu. The build already has `CONFIG_SMP` off and `CONFIG_MP_MAX_NUM_CPUS=1`,
and the board already sets `&wifi { status = "okay"; }`, so nothing has to be
restructured to make WiFi buildable — but there is genuinely one core.

**E1's carry-forward was too optimistic about what follows from that, and this
section supersedes it.** E1 stated that "a lower-priority network thread runs in
that slack without ever preempting a delayed-TX arm." That is true of *our own*
uplink thread and false of Zephyr's networking threads:

- WiFi blob tasks are created by passing the ESP-IDF priority value straight
  into `k_thread_create()` (`hal/espressif/components/esp_wifi/esp32s3/esp_adapter.c`,
  `task_create_wrapper`), bounded by `CONFIG_ESP32_WIFI_MAX_THREAD_PRIORITY`
  (default 7). Positive Zephyr priorities are **preemptible** and numerically
  *lower* priority than the main thread at 0. These are harmless.
- Zephyr's own traffic-class RX/TX threads are not. The `NET_TC_THREAD_TYPE`
  choice has no explicit default, so it resolves to its first entry,
  `NET_TC_THREAD_COOPERATIVE` (`subsys/net/ip/Kconfig`), at
  `NET_TC_RX_THREAD_BASE_PRIO`/`NET_TC_TX_THREAD_BASE_PRIO` = 0 — i.e.
  `K_PRIO_COOP(0)` = −16. A cooperative thread cannot be preempted. An RX burst
  would run to completion and can overrun `BEACON_ARM_MARGIN_UUS` (5 ms).

Isolation is therefore achieved by **priority**, and it has to be stated
explicitly rather than left to defaults.

### 3.2 The priority map

| Thread | Priority | Role |
|---|---|---|
| UWB gateway loop (main) | `K_PRIO_COOP(0)` = −16 | beacon, GRANT, `dispatch()` |
| Zephyr net TC RX/TX | preemptible (forced) | see `CONFIG_NET_TC_THREAD_PREEMPTIVE` below |
| WiFi blob tasks | 0…7, preemptible | created by the ESP-IDF OS adapter |
| `net_uplink` | 10, preemptible | WiFi assoc, MQTT, publish, reconnect |

Two changes implement this:

1. **`CONFIG_NET_TC_THREAD_PREEMPTIVE=y`.** Without it the net TC threads sit at
   the *same* cooperative priority as the promoted UWB loop, and whichever runs
   first runs to completion. This single Kconfig line is the difference between
   the beacon being protected and the beacon being usually fine.
2. **`k_thread_priority_set(k_current_get(), K_PRIO_COOP(0))` in `main()`**,
   immediately before `uwb_gateway_run()`.

The loop already yields voluntarily at `k_sem_take(&rx_sem, K_MSEC(400))`
(`src/uwb_gateway.c:317`), which is where every other thread gets its time. When
the loop *is* runnable — arming the beacon, polling TXFRS — nothing else runs.

`net_uplink` sits at 10, deliberately *below* the WiFi driver threads, so the
driver drains its packet queues before we hand it more work.

### 3.3 Consequences to respect

**Promotion happens in GATEWAY mode only.** SLAVE mode keeps the main thread at
priority 0. Slaves run no network thread, so the change would buy nothing and
would alter timing on a path that is already bench-confirmed. Less regression
surface.

**A cooperative main thread makes every unbounded spin fatal.** CLAUDE.md
records that an unbounded TXFRS wait once froze the main thread and killed the
shell with it. At priority 0 that was already fatal; at `K_PRIO_COOP(0)` any
future unbounded spin is unrecoverable by construction, because no lower-priority
thread — including the shell — can ever run again. `uwb_wait_for_sysstatus_lo()`'s
bounded wait is therefore a **hard requirement** of this design, not a nicety.
Any new busy-wait added to the gateway path must carry a timeout.

**`net_uplink` is a second thread, so lazy config initialisation is unsafe.**
CLAUDE.md's hard-won fact about `uwb_config_get()` applies directly: its lazy
initialiser is unsynchronised and is safe today only because a human cannot type
a shell command inside the ~26 ms before `main()` claims it. A second thread
breaks that. `net_config` must therefore be initialised **explicitly** from
`main()` before `net_uplink` is started, and must not rely on a lazy
first-caller-wins initialiser.

**`net_uplink` starts in GATEWAY mode only.** A slave has nothing to publish;
starting WiFi there would spend ~50 KB of heap and add radio activity for no
benefit.

## 4. Modules

Following the established pure-C-core / Zephyr-shell split, so the parts worth
testing are testable on the host:

- **`src/pos_json.{c,h}`** — pure C. Formats a `struct pos_fix` into a
  caller-supplied buffer via `snprintf`, and emits the stub anchor map. No
  Zephyr, no sockets. `pos_sink.h` is already `stdint`-only, so it can be
  included directly. Host-tested.
- **`src/net_config.{c,h}`** — pure C. SSID, PSK, broker host, port, MQTT
  user/pass, with validating setters returning `bool` and leaving the struct
  untouched on rejection — the `uwb_config_set_*` contract. Host-tested.
- **`src/net_store.{c,h}`** — Zephyr settings/NVS under a `net/` subtree, its own
  handler, separate from `anchor/`. Mirrors `uwb_store.c` per-field-key style.
- **`src/net_shell.c`** — the `net` console tree, registered alongside `anchor`.
- **`src/net_uplink.{c,h}`** — the thread. Owns the `k_msgq`, the WiFi/MQTT state
  machine, the poll loop and the backoff ladder.
- **`src/pos_sink.c`** — rewritten as the queue producer. **`pos_sink.h` does not
  change**, so `src/uwb_gateway.c` needs no edit.

## 5. Data flow

```
DW3000 IRQ → rx_sem → gateway loop → dispatch() → pos_sink_publish()
                                                        │ k_msgq_put(K_NO_WAIT)
                                                        ▼
                                              [ k_msgq, depth 16 ]
                                                        │
                                net_uplink drains → pos_json_fix() → mqtt_publish()
```

`pos_sink_publish()` keeps its existing `n_anchors` range check, rejecting a
corrupt frame before it occupies a queue slot, then does a single
`k_msgq_put(..., K_NO_WAIT)` and returns. It never touches a socket.

**The existing `LOG_INF` JSON line stays**, unchanged, ahead of the enqueue. It
is the only place `residual_m` and `batt_soc` remain visible (§6.1 drops them
from the published payload), and it is what makes the gateway debuggable over
USB with no broker present. Enqueueing replaces nothing; it is added after.

**Queue depth 16** ≈ 1.5 superframes at 11 CFP seats — enough to absorb a publish
stalling behind one TCP retransmit, small enough that a real outage discards
rather than accumulates.

**Drop-oldest on full:** `k_msgq_get()` to discard one, then retry the put. A
stale position is worthless in an RTLS, and unbounded buffering would exhaust the
same heap WiFi needs.

**Poll loop:** `net_uplink` waits on the MQTT socket with `zsock_poll()` at a
50 ms timeout, calling `mqtt_input()` when readable and `mqtt_live()` each
iteration (which handles PINGREQ keepalive), then drains the queue
non-blocking. Zephyr cannot wait on a socket and a `k_msgq` in one call without
an eventfd, and a short poll timeout is the simpler mechanism: it costs ≤50 ms of
added publish latency against a 200 ms superframe.

## 6. Payload contracts

### 6.1 Position — `testtopic/1/position`

QoS 0, **not** retained. The topic is flat, so retaining would only preserve
whichever tag published last, which is actively misleading. QoS 0 matches the
UWB link underneath, which is already lossy; a fix needing a retransmit is stale
by the time it lands.

```json
{"tagId":"1234","x":1.23,"y":4.56,"z":0,"zoneName":"Zona"}
```

This matches an existing Python consumer byte for byte, and the field set is
deliberately **exactly** those five keys:

| Field | Format | Notes |
|---|---|---|
| `tagId` | `"%04X"` of `src_addr` | **Bare uppercase hex, zero-padded to 4, no `0x` prefix.** `0x00AB` → `"00AB"` |
| `x`, `y` | `%.2f` | mirrors the Python `round(..., 2)` |
| `z` | integer literal `0` | not `0.00` — the solver is 2D |
| `zoneName` | compile-time `#define` | `"Zona"`; YAGNI until a second zone exists |

`tagId` has no `0x` prefix by explicit choice, to stay compatible with the
existing consumer. It is therefore ambiguous to a human reading the broker —
`"1234"` is hex, not decimal 1234. The format string must carry a comment saying
so; this is the kind of thing that silently becomes an off-by-4660 bug.

`residual_m`, `n_anchors` and `batt_soc` still arrive in every `0xEA` frame and
are still decoded and logged on the console. They are simply not published. If a
consumer later wants them, adding keys is backward-compatible.

### 6.2 Anchors stub — `testtopic/1/anchors`

QoS 1, **retained**, published once per successful MQTT connect. Retained is
correct here and wrong for position: this is slow-changing state that a late
subscriber needs.

```json
{"name":"Zona","anchors":[
 {"name":"A0","isAxis":true,"isReferenceAxis":true,"latitude":0.0,"longitude":0.0,"x":0.0,"y":0.0,"z":0.0},
 {"name":"A1","isAxis":true,"isReferenceAxis":false,"latitude":0.0,"longitude":0.0,"x":0.0,"y":0.0,"z":0.0},
 {"name":"A2","isAxis":false,"isReferenceAxis":false,"latitude":0.0,"longitude":0.0,"x":0.0,"y":0.0,"z":0.0}]}
```

Placeholder data in the real schema, so the downstream consumer can be built and
tested now. The zone name is the same `"Zona"` define as `zoneName` above — both
topics must agree on the zone identifier.

Replacing this with surveyed positions later changes only where the numbers come
from, not the topic, the schema or the publish timing.

### 6.3 Client identity

MQTT client id is `uwb-gw-%04X` of the gateway's short address (`0x0000` per the
MAC contract, so `uwb-gw-0000` in practice). Distinct from `tagId`'s bare-hex
format because a client id is not consumed by the Python parser and readability
wins there.

## 7. Configuration and console

NVS keys under a `net/` subtree: `net/ssid`, `net/psk`, `net/broker`,
`net/port`, `net/user`, `net/mqttpass`.

```
net ssid <ssid>              WiFi SSID (1..32 chars)
net pass <psk>               WPA2 passphrase (8..63 chars)
net broker <host> [port]     broker address; port optional, defaults to 1883
net user <username>          MQTT username (may be empty)
net mqttpass <password>      MQTT password (may be empty)
net show                     configuration and live state as JSON
net reset                    restore defaults and persist
```

```json
{"ssid":"MyNetwork","psk":"<set>","broker":"10.0.0.5","port":1883,
 "user":"gateway1","mqttpass":"<set>","state":"connected","ip":"10.0.0.42"}
```

There are two passwords here and they must never be confused. `net pass` sets
the **WiFi** PSK and reads back as `"psk"`; `net mqttpass` sets the **MQTT**
password and reads back as `"mqttpass"`. The NVS key, the command and the JSON
key agree in each case — an earlier draft had `net pass` writing `net/psk` while
`net show` printed `"pass"` for the unrelated MQTT secret, which is exactly the
sort of thing that gets one credential pasted into the other's field.

`net show` prints `<set>`/`<unset>` for both secrets and never the values. The
console is the one place they would otherwise leak, and it is also what gets
pasted into a bug report.

Settings persist immediately and apply on reboot, matching the `anchor` tree's
documented semantics. `net show` additionally reports live association and MQTT
state plus the DHCP-assigned address, so provisioning is verifiable from the
console without a broker-side check.

Broker address accepts a hostname or an IPv4 literal, resolved through
`getaddrinfo()` — hence `CONFIG_DNS_RESOLVER=y`. Supporting only literals would
be smaller, but a broker that moves is then a reflash.

## 8. Security

MQTT username/password over plain TCP. The credentials cross the network in the
clear: this is access control, not confidentiality.

**Note for whoever adds TLS later:** it is cheaper than it looks. `WIFI_ESP32`
already does `select MBEDTLS` and `select PSA_CRYPTO` for WPA2 supplicant
crypto, so mbedTLS is linked whether or not MQTT uses it. The incremental cost of
`CONFIG_MQTT_LIB_TLS` is the TLS heap arena, a CA certificate to provision and
rotate, and — because certificate validity is time-checked — SNTP before the
first connect. An earlier draft of this design claimed plain TCP avoids mbedTLS
entirely; that was wrong and is corrected here.

## 9. Error handling

The governing rule: **the network never degrades UWB.** A gateway with no WiFi
configured, no AP in range, or a dead broker must beacon and grant seats exactly
as it does today.

| Condition | Behaviour |
|---|---|
| No SSID configured | `net_uplink` logs once and sleeps. Never retries, never touches the radio |
| WiFi association fails or drops | Exponential backoff 1 s → 32 s, capped. State visible in `net show` |
| MQTT connect fails | Same backoff ladder, tracked independently of the WiFi one |
| Queue full | Drop oldest, increment a counter, rate-limited `LOG_WRN` at most once per 10 s |
| Publish returns an error | Drop that fix, mark the connection dead, fall into the reconnect path |
| `n_anchors` out of range | Already rejected in `pos_sink_publish()`, before a queue slot is taken |
| `pos_json_fix()` truncation | Treated as a formatting bug: drop the fix and `LOG_ERR`. Never publish a truncated JSON document |

The rate limit on the queue-full warning matters more than it looks. A sustained
outage produces roughly 55 drops per second, and an unthrottled warning would
flood the very console being used to diagnose it.

## 10. Build impact

New Kconfig, all in `prj.conf`:

```
CONFIG_WIFI=y
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_DHCPV4=y
CONFIG_NET_SOCKETS=y
CONFIG_DNS_RESOLVER=y
CONFIG_MQTT_LIB=y
CONFIG_NET_TC_THREAD_PREEMPTIVE=y   # see §3.2 — not optional
```

`CONFIG_WIFI_ESP32` enables itself from the devicetree node. It pulls in
`CONFIG_HEAP_MEM_POOL_ADD_SIZE_ESP_WIFI=51200`, so the system heap grows from
its current ~4 KB by ~50 KB plus network buffers. `net_uplink` gets a 4 KB stack;
the MQTT client gets modest fixed RX/TX buffers plus one payload buffer sized for
the anchors stub, which is the larger of the two documents.

Flash and RAM growth are expected to be substantial and should be **measured and
recorded**, not estimated — the ESP32-S3-WROOM-1-N8R2 has room, but this is the
first time the network stack has been linked into this image.

## 11. Testing

### Host tests

Plain gcc, `CHECK` macro, non-zero exit on failure, matching `tests/uwb_frame/`:

- **`tests/pos_json/`** — exact byte-for-byte output against the §6.1 contract
  string; `z` emitted as an integer; `tagId` zero-padding for a low address
  (`0x00AB` → `"00AB"`); negative and large coordinates; `snprintf` truncation
  reported rather than silently emitting a partial document; the anchors stub
  matching §6.2.
- **`tests/net_config/`** — SSID and PSK length bounds, port range, empty
  username/password accepted, and rejection leaving the struct completely
  untouched.

### On-target

1. `net` settings persist across a reboot; `net show` never prints a secret.
2. `mosquitto_sub -t 'testtopic/1/#'` receives position messages matching §6.1
   exactly, and the retained anchors message arrives on a **late** subscribe.
3. **Beacon integrity under network load.** With WiFi associated and traffic
   flowing, the gateway logs no `grant missed its slot` and no
   `TXFRS never completed` warnings, and slaves hold their CFP seats without
   re-JOINing. This is what justifies §3.2 — if it fails, the threading model is
   wrong, not the tuning.
4. Pull the AP: the gateway keeps beaconing, slaves keep their seats, the drop
   counter climbs. Restore it: publishing resumes with no reboot.
5. Record the flash and RAM deltas from the build output (§10).

Test 3 is the acceptance gate for this sub-project. The rest can be fixed
incrementally; a beacon that stutters under network load means the design is
wrong.

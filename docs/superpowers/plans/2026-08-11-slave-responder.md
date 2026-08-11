# SLAVE Responder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make SLAVE mode real — the anchor answers the tag's ranging polls in both wire formats and observes the gateway beacon.

**Architecture:** Copy the 802.15.4z frame codec verbatim from the tag (it is the on-air peer and already Zephyr-clean), add the Qorvo timestamp helpers br101 did not vendor, port the two responders from the nRF5 anchor, and replace the spec A `uwb_slave_run()` stub with an interrupt-driven RX loop. Callbacks give a semaphore; the main thread does all SPI.

**Tech Stack:** Zephyr 4.4.x on ESP32-S3, `west`, Qorvo `dwt_uwb_driver` 08.02.02 via the vendored br101 module, plain-gcc host tests.

**Spec:** [`docs/superpowers/specs/2026-08-11-slave-responder-design.md`](../specs/2026-08-11-slave-responder-design.md)

## Global Constraints

- **Do not modify `boards/` or `modules/`.** The board definition and the vendored driver are out of scope; the module carries deliberate local deltas.
- **Do not edit the tag project** at `../../../tag_testting`. It is read-only context. Files are copied *from* it, never back to it.
- **`UUS_TO_DWT_TIME` is `65536`**, not the `63898` in the fw-cre source. See spec finding 2.
- **Every length passed to the frame module is `flen - FCS_LEN`.** `dwt_getframelength()` / `cb_data->datalength` include the 2-byte FCS. See spec finding 5.
- **Never wait on `DWT_INT_CIADONE_BIT_MASK`** after an RX event. The ISR clears it before the callback runs; waiting hangs. Test the `cb_data->status` snapshot instead. See spec finding 4.
- **`dwt_setinterrupt()` enables `DWT_INT_RX` only.** Enabling `TXFRS` would hang `tx_delayed()`'s completion poll.
- **`uwb_frame_802_15_4z.{c,h}` and `tests/uwb_frame/test_uwb_frame.c` are copied byte-for-byte** from the tag and must stay that way. Do not reformat, do not fix the three known defects (spec §"copied verbatim").
- **Build:** `$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"` must be set; the project lives outside the west workspace.
- Source style: tabs for indentation in Zephyr-facing files, `LOG_INF`/`LOG_WRN`/`LOG_ERR` for output, Apache-2.0 header comment as in existing `src/` files. The copied frame module keeps the tag's 4-space style — it is not ours to restyle.

---

### Task 1: Copy the 802.15.4z frame module and its test suite

**Files:**
- Create: `src/uwb_frame_802_15_4z.c` (copy)
- Create: `src/uwb_frame_802_15_4z.h` (copy)
- Create: `tests/uwb_frame/test_uwb_frame.c` (copy)
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: the whole frame API. Later tasks use `uwb_frame_is_discovery(const uint8_t *buf, size_t len)`, `uwb_frame_get_src_addr(const uint8_t *buf)`, `uwb_frame_response_build(uint8_t *buf, size_t buf_len, uint16_t src_addr, uint16_t dest_addr, uint32_t tx_ts, int32_t cir_power, uint16_t cir_quality)`, `uwb_frame_set_seq_num(uint8_t *buf, uint8_t seq)`, `uwb_frame_is_beacon(const uint8_t *buf, size_t len)`, `uwb_frame_parse_beacon(const uint8_t *buf, size_t len, uint8_t *proto_ver, uint32_t *frame_counter, uint16_t *slot_map_out, uint8_t *n_slots)`, `uwb_frame_beacon_find_addr(const uint16_t *slot_map, uint8_t n_slots, uint16_t addr)`. Constants: `UWB_FRAME_LEN_RESP` (20), `UWB_FRAME_MAX_LEN` (39), `UWB_FRAME_N_CFP` (12), `UWB_ADDR_GATEWAY` (0x0000).

- [ ] **Step 1: Copy the three files**

```powershell
$tag = "C:\Users\JoseAntonioLaraPerez\Documents\tag_testting"
Copy-Item "$tag\src\uwb_frame_802_15_4z.c" src\
Copy-Item "$tag\src\uwb_frame_802_15_4z.h" src\
New-Item -ItemType Directory -Force tests\uwb_frame | Out-Null
Copy-Item "$tag\tests\uwb_frame\test_uwb_frame.c" tests\uwb_frame\
```

- [ ] **Step 2: Verify the copies are byte-identical to the source**

```powershell
$tag = "C:\Users\JoseAntonioLaraPerez\Documents\tag_testting"
foreach ($f in @("uwb_frame_802_15_4z.c","uwb_frame_802_15_4z.h")) {
    $a = (Get-FileHash "$tag\src\$f").Hash
    $b = (Get-FileHash "src\$f").Hash
    if ($a -ne $b) { Write-Error "MISMATCH: $f" } else { Write-Host "OK: $f" }
}
$a = (Get-FileHash "$tag\tests\uwb_frame\test_uwb_frame.c").Hash
$b = (Get-FileHash "tests\uwb_frame\test_uwb_frame.c").Hash
if ($a -ne $b) { Write-Error "MISMATCH: test_uwb_frame.c" } else { Write-Host "OK: test_uwb_frame.c" }
```

Expected: `OK:` for all three. If a hash differs, the copy was altered — redo step 1.
Do not "fix" a mismatch by editing our copy to taste; the whole point is that it stays identical.

- [ ] **Step 3: Run the host test suite**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_frame\test_uwb_frame.exe `
    tests\uwb_frame\test_uwb_frame.c src\uwb_frame_802_15_4z.c
.\tests\uwb_frame\test_uwb_frame.exe
```

Expected: `ALL TESTS PASSED`, exit code 0, and no compiler warnings. The suite passing unmodified against our copy is the evidence the copy is faithful.

- [ ] **Step 4: Add the module to the firmware build**

In `CMakeLists.txt`, add to the existing `target_sources(app PRIVATE ...)` list, keeping it alphabetical:

```cmake
	src/uwb_frame_802_15_4z.c
```

The list becomes `anchor_shell.c, main.c, uwb_config.c, uwb_frame_802_15_4z.c, uwb_gateway.c, uwb_radio.c, uwb_slave.c, uwb_store.c`.

- [ ] **Step 5: Verify the firmware still builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings. Nothing calls the module yet, so this only proves it compiles under the Zephyr toolchain.

- [ ] **Step 6: Ignore the test binary**

Add to `.gitignore` if not already covered:

```
tests/**/*.exe
```

- [ ] **Step 7: Commit**

```bash
git add src/uwb_frame_802_15_4z.c src/uwb_frame_802_15_4z.h \
        tests/uwb_frame/test_uwb_frame.c CMakeLists.txt .gitignore
git commit -m "feat(frame): copy the 802.15.4z frame module and test suite from the tag

Byte-identical to tag_testting/src/uwb_frame_802_15_4z.{c,h}, so future
drift between anchor and peer is a one-line diff. The tag's 286-line
test suite ports unmodified and passes against our copy, which is the
evidence the copy is faithful.

Three known defects are left in place deliberately (unbounded n_slots in
beacon_build, partial write on null EUI in join/grant_build, -EINVAL vs
-EBADMSG in parse_beacon) -- all sit in builders this spec never calls.
See the design doc."
```

---

### Task 2: Derive the contract-compliant short address

**Files:**
- Modify: `src/uwb_config.h`
- Modify: `src/uwb_config.c`
- Modify: `src/anchor_shell.c:54-65` (`print_config`)
- Modify: `src/main.c:24-32` (`log_config`)
- Test: `tests/uwb_config/test_uwb_config.c`

**Interfaces:**
- Consumes: `uwb_config_t`, `UWB_MAX_ANCHORS` from Task 0 (already in the tree).
- Produces: `uint16_t uwb_config_short_addr(const uwb_config_t *c)` and `#define UWB_ANCHOR_ADDR_BASE 0x0001u`. Tasks 4 and 5 use the return value as the DISCOVERY src address and its low byte as the WAVE id.

Background: `spec/2026-06-17-uwb-mac-protocol-contract.md` reserves `0x0000` for the gateway and gives anchors `0x0001…`. The console id stays 0..3 with default 0; the wire address is derived. See spec finding 3.

- [ ] **Step 1: Write the failing test**

Add to `tests/uwb_config/test_uwb_config.c`, and add `test_short_addr();` to `main()` after `test_set_id();`:

```c
static void test_short_addr(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    /* Default id 0 must not map to the gateway's reserved 0x0000. */
    CHECK(c.anchor_id == 0);
    CHECK(uwb_config_short_addr(&c) == 0x0001);

    /* The mapping is base + id across the whole valid range. */
    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        CHECK(uwb_config_short_addr(&c) == (uint16_t)(UWB_ANCHOR_ADDR_BASE + id));
    }

    /* No valid id can produce a reserved address. */
    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        uint16_t a = uwb_config_short_addr(&c);
        CHECK(a != UWB_ADDR_GATEWAY_RESERVED);
        CHECK(a != 0xFFFE);
        CHECK(a != 0xFFFF);
    }

    /* The low byte is what the tag echoes in the WAVE poll's byte 10. */
    CHECK(uwb_config_set_id(&c, 3));
    CHECK((uint8_t)(uwb_config_short_addr(&c) & 0xFFu) == 4);
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_config\test_uwb_config.exe `
    tests\uwb_config\test_uwb_config.c src\uwb_config.c
```

Expected: FAIL to compile — `uwb_config_short_addr` and `UWB_ANCHOR_ADDR_BASE` undeclared.

- [ ] **Step 3: Add the declaration**

In `src/uwb_config.h`, after the `UWB_ANT_DELAY_DEFAULT` define:

```c
/* Anchor short addresses start at 1: the MAC contract reserves 0x0000 for the
 * gateway (spec/2026-06-17-uwb-mac-protocol-contract.md, Topology). The console
 * id stays 0-based; the wire address is derived. */
#define UWB_ANCHOR_ADDR_BASE 0x0001u

/* Reserved by the contract; declared here so the host test can assert no valid
 * id ever collides with it. */
#define UWB_ADDR_GATEWAY_RESERVED 0x0000u
```

and after `uwb_config_mode_name()`:

```c
/* This anchor's 16-bit short address: UWB_ANCHOR_ADDR_BASE + anchor_id.
 * The single place the console id maps to the on-air address. Its low byte is
 * also the id the tag echoes in the legacy WAVE poll. */
uint16_t uwb_config_short_addr(const uwb_config_t *c);
```

- [ ] **Step 4: Implement it**

In `src/uwb_config.c`, at the end:

```c
uint16_t uwb_config_short_addr(const uwb_config_t *c)
{
	return (uint16_t)(UWB_ANCHOR_ADDR_BASE + c->anchor_id);
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_config\test_uwb_config.exe `
    tests\uwb_config\test_uwb_config.c src\uwb_config.c
.\tests\uwb_config\test_uwb_config.exe
```

Expected: `PASSED`, exit code 0.

- [ ] **Step 6: Surface both numbers in the console and the boot banner**

The operator sees a 0-based id while the tag reports 1-based. Printing both means the relationship is never inferred.

In `src/anchor_shell.c`, replace the body of `print_config()`:

```c
static void print_config(const struct shell *sh)
{
	const uwb_config_t *cfg = uwb_config_get();

	shell_print(sh,
		    "{\"mode\":\"%s\",\"id\":%u,\"short_addr\":\"0x%04X\","
		    "\"ant_tx\":%u,\"ant_rx\":%u,"
		    "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		    uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		    uwb_config_short_addr(cfg),
		    cfg->ant_delay_tx, cfg->ant_delay_rx,
		    (double)cfg->x, (double)cfg->y, (double)cfg->z,
		    cfg->position_valid ? 1u : 0u);
}
```

In `src/main.c`, replace the body of `log_config()`:

```c
static void log_config(const uwb_config_t *cfg)
{
	LOG_INF("{\"mode\":\"%s\",\"id\":%u,\"short_addr\":\"0x%04X\","
		"\"ant_tx\":%u,\"ant_rx\":%u,"
		"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		uwb_config_short_addr(cfg),
		cfg->ant_delay_tx, cfg->ant_delay_rx,
		(double)cfg->x, (double)cfg->y, (double)cfg->z,
		cfg->position_valid ? 1u : 0u);
}
```

- [ ] **Step 7: Verify the firmware builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings.

- [ ] **Step 8: Commit**

```bash
git add src/uwb_config.h src/uwb_config.c src/anchor_shell.c src/main.c \
        tests/uwb_config/test_uwb_config.c
git commit -m "feat(config): derive the contract-compliant anchor short address

The MAC contract reserves 0x0000 for the gateway and gives anchors
0x0001+, but spec A's default id is 0 and the responder used it directly
as the src short address -- so a default anchor would have claimed the
gateway's address, which spec D puts on the same air.

The console id stays 0..3 with default 0 as specified; the wire address
is UWB_ANCHOR_ADDR_BASE + id, in one function. anchor show and the boot
banner now print both, since the tag reports anchors 1..4 while the
console says 0..3."
```

---

### Task 3: The Qorvo timestamp helpers br101 did not vendor

**Files:**
- Create: `src/uwb_dwtime.h`
- Create: `src/uwb_dwtime.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `deca_device_api.h` from the vendored module.
- Produces: `UUS_TO_DWT_TIME` (65536), `FCS_LEN` (2), `uint64_t uwb_get_rx_timestamp_u64(void)`, `void uwb_resp_msg_set_ts(uint8_t *ts_field, uint64_t ts)`, `void uwb_wait_for_sysstatus_lo(uint32_t lo_mask)`. Tasks 4 and 5 use all of them.

Background: `owner_ss_twr_responder.c` and `anchor_respond.c` use these from Qorvo's `examples/shared_data/shared_functions.c`, which br101 did not vendor — `modules/dw3000-decadriver/` carries only the driver and platform layer. See spec finding 1.

- [ ] **Step 1: Create the header**

`src/uwb_dwtime.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The handful of helpers the Qorvo examples provide in
 * examples/shared_data/shared_functions.c, which the vendored br101 module does
 * not carry -- it ships only the driver and the platform layer. Kept in one
 * place so that dependency is visible rather than smeared across the
 * responders.
 */

#ifndef UWB_DWTIME_H
#define UWB_DWTIME_H

#include <stdint.h>

/* DWT time units per UWB microsecond.
 *
 * fw-cre's shared_defines.h says 63898; this project uses 65536, which is what
 * both peers on this network use -- tag_testting/src/uwb_ss_initiator.c and
 * ESP32S3UWB/src/ranging.c, the latter having ranged against that tag on
 * hardware. The constant scales the delayed-TX turnaround both ends must agree
 * on, so the source's value is not merely a different convention here. */
#define UUS_TO_DWT_TIME 65536UL

/* Frame check sequence appended by the radio. dwt_getframelength() and
 * cb_data->datalength both report payload + FCS. */
#define FCS_LEN 2u

/* Width of a timestamp field in the legacy VEWA response. */
#define RESP_MSG_TS_LEN 4u

/* The 40-bit RX timestamp of the last received frame, as a 64-bit value.
 * Valid until the next reception. */
uint64_t uwb_get_rx_timestamp_u64(void);

/* Write the low 4 bytes of ts into a response timestamp field, least
 * significant byte first. */
void uwb_resp_msg_set_ts(uint8_t *ts_field, uint64_t ts);

/* Spin until every bit in lo_mask is set in the low system status register.
 *
 * Only safe for events the ISR does not clear first. Never call this with
 * DWT_INT_CIADONE_BIT_MASK after an RX event: dwt_isr() clears
 * SYS_STATUS_ALL_RX_GOOD -- which includes CIADONE -- before invoking the RX
 * callback, so the bit will not set again until the next frame and this spins
 * forever. The same applies to TXFRS if TX interrupts are ever enabled. */
void uwb_wait_for_sysstatus_lo(uint32_t lo_mask);

#endif /* UWB_DWTIME_H */
```

- [ ] **Step 2: Create the implementation**

`src/uwb_dwtime.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uwb_dwtime.h"

#include <deca_device_api.h>

uint64_t uwb_get_rx_timestamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;

	dwt_readrxtimestamp(ts_tab);
	for (int8_t i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

void uwb_resp_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
	for (uint8_t i = 0; i < RESP_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t)(ts >> (i * 8));
	}
}

void uwb_wait_for_sysstatus_lo(uint32_t lo_mask)
{
	while (!(dwt_readsysstatuslo() & lo_mask)) {
	}
}
```

- [ ] **Step 3: Add to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, alphabetically after `src/uwb_config.c`:

```cmake
	src/uwb_dwtime.c
```

- [ ] **Step 4: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings. `dwt_readrxtimestamp` takes a `uint8_t *`; if the compiler objects to the signature, check `deca_device_api.h` rather than casting.

There is no host test here: every function touches the radio over SPI, so there is nothing to exercise off-target. The constants are asserted indirectly by Task 6's on-target ranging.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_dwtime.h src/uwb_dwtime.c CMakeLists.txt
git commit -m "feat(dwtime): add the Qorvo timestamp helpers br101 did not vendor

get_rx_timestamp_u64, resp_msg_set_ts and a status-poll helper live in
Qorvo's examples/shared_data tree, which the vendored module does not
carry. The tag hit the same gap and solved it the same way.

UUS_TO_DWT_TIME is 65536, not fw-cre's 63898: it scales the delayed-TX
turnaround both ends must agree on, and both peers on this network use
65536.

The status-poll helper carries an explicit warning against CIADONE --
dwt_isr() clears it before the RX callback runs, so waiting on it hangs."
```

---

### Task 4: The response stagger

**Files:**
- Create: `src/disc_schedule.h`
- Create: `src/disc_schedule.c`
- Create: `tests/disc_schedule/test_disc_schedule.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: `uint32_t disc_resp_delay_uus(uint8_t anchor_id)`, `DISC_BASE_UUS` (2000), `DISC_SLOT_UUS` (3500). Task 5 calls it.

This is kept a separate module rather than folded into `anchor_respond.c` so it stays host-testable: it is the arithmetic that keeps four anchors from transmitting on top of each other, and `anchor_respond.c` cannot compile on the host because it pulls in `deca_device_api.h`.

Note it takes the **0-based** `anchor_id`, not the short address from Task 2. Keying it on the address would make anchor 0 wait 5500 µs instead of 2000.

- [ ] **Step 1: Write the failing test**

`tests/disc_schedule/test_disc_schedule.c`:

```c
#include "disc_schedule.h"
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_base_and_stagger(void)
{
    /* Anchor 0 responds at the turnaround floor, not one slot in. */
    CHECK(disc_resp_delay_uus(0) == DISC_BASE_UUS);
    CHECK(disc_resp_delay_uus(0) == 2000u);

    /* Each subsequent anchor is exactly one slot later. */
    CHECK(disc_resp_delay_uus(1) == 2000u + 3500u);
    CHECK(disc_resp_delay_uus(2) == 2000u + 2u * 3500u);
    CHECK(disc_resp_delay_uus(3) == 2000u + 3u * 3500u);
}

static void test_slots_never_overlap(void)
{
    /* The whole point of the module: no two anchors share a transmit
     * instant, and the gap is always a full slot. */
    for (uint8_t i = 1; i < 4; i++) {
        CHECK(disc_resp_delay_uus(i) > disc_resp_delay_uus(i - 1));
        CHECK(disc_resp_delay_uus(i) - disc_resp_delay_uus(i - 1) == DISC_SLOT_UUS);
    }
}

int main(void)
{
    test_base_and_stagger();
    test_slots_never_overlap();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe `
    tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
```

Expected: FAIL — `src\disc_schedule.c` and `disc_schedule.h` do not exist.

- [ ] **Step 3: Create the header**

`src/disc_schedule.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Discovery responder turnaround floor and per-anchor stagger. This is what
 * keeps four anchors from answering one broadcast DISCOVERY on top of each
 * other, which is why it is its own host-testable module.
 */

#ifndef DISC_SCHEDULE_H
#define DISC_SCHEDULE_H

#include <stdint.h>

/* Minimum turnaround from DISCOVERY RX to the first anchor's response TX. */
#define DISC_BASE_UUS 2000u

/* Gap between consecutive anchors' responses.
 * TUNE ON HARDWARE: must exceed one 0xE4 reply's air time. 3500 (3.5 ms >
 * 1.3 ms frame airtime + SPI and settle margin) is an estimate carried over
 * from the nRF5 anchor, not a measurement on this board. */
#define DISC_SLOT_UUS 3500u

/* Delay in UWB microseconds from DISCOVERY RX to this anchor's response TX.
 * Takes the 0-based console id, NOT the short address -- keying it on the
 * address would make anchor 0 wait a full slot for no reason. */
uint32_t disc_resp_delay_uus(uint8_t anchor_id);

#endif /* DISC_SCHEDULE_H */
```

- [ ] **Step 4: Create the implementation**

`src/disc_schedule.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "disc_schedule.h"

uint32_t disc_resp_delay_uus(uint8_t anchor_id)
{
	return DISC_BASE_UUS + (uint32_t)anchor_id * DISC_SLOT_UUS;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe `
    tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
.\tests\disc_schedule\test_disc_schedule.exe
```

Expected: `PASSED`, exit code 0.

- [ ] **Step 6: Add to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, alphabetically first among the `src/` entries after `src/anchor_shell.c`:

```cmake
	src/disc_schedule.c
```

- [ ] **Step 7: Commit**

```bash
git add src/disc_schedule.h src/disc_schedule.c \
        tests/disc_schedule/test_disc_schedule.c CMakeLists.txt
git commit -m "feat(disc): per-anchor discovery response stagger

Ported unchanged from the nRF5 anchor. Kept as its own module rather
than folded into anchor_respond.c so it stays host-testable --
anchor_respond.c pulls in deca_device_api.h and cannot compile on the
host, and this is the arithmetic that keeps four anchors off each
other's transmissions.

Keyed on the 0-based console id, not the short address."
```

---

### Task 5: The two responders

**Files:**
- Create: `src/anchor_respond.h`
- Create: `src/anchor_respond.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `uwb_config_short_addr()` (Task 2); `UUS_TO_DWT_TIME`, `FCS_LEN`, `RESP_MSG_TS_LEN`, `uwb_resp_msg_set_ts()`, `uwb_wait_for_sysstatus_lo()` (Task 3); `disc_resp_delay_uus()` (Task 4); `uwb_frame_is_discovery()`, `uwb_frame_get_src_addr()`, `uwb_frame_response_build()`, `uwb_frame_set_seq_num()`, `UWB_FRAME_LEN_RESP` (Task 1).
- Produces: `void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts, const uwb_config_t *cfg, uint8_t *seq)` and `void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts, const uwb_config_t *cfg, uint8_t *seq, int32_t cir_power, uint16_t cir_quality)`. Task 6 calls both. **`len` is the payload length — the caller has already subtracted `FCS_LEN`.**

Ported from `fw-cre/firmeware_creator/Src/anchor_respond.c` with three changes: the short address replaces the raw `anchor_id` on the wire, `anchor_read_cir()` is gone (Task 6 captures CIR from the callback snapshot instead), and `test_run_info` becomes `LOG_*`.

- [ ] **Step 1: Create the header**

`src/anchor_respond.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The two ranging responders. Each is offered every received frame and ignores
 * what is not addressed to it.
 *
 * IMPORTANT: len is the PAYLOAD length. The radio reports payload + FCS, and
 * the caller must subtract FCS_LEN before calling either function -- the frame
 * module derives field counts from the length and an extra 2 bytes silently
 * corrupts the result.
 */

#ifndef ANCHOR_RESPOND_H
#define ANCHOR_RESPOND_H

#include <stdint.h>

#include "uwb_config.h"

/* If buf is a legacy WAVE/0xE0 poll addressed to this anchor, schedule the
 * delayed VEWA/0xE1 response carrying our id, both timestamps and (x, y).
 * Otherwise no-op. */
void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq);

/* If buf is a DISCOVERY/0xE2 broadcast, schedule an id-staggered
 * RANGE-RESPONSE/0xE4 carrying our short address and CIR metrics. Otherwise
 * no-op.
 *
 * cir_power / cir_quality are captured by the caller from the RX callback --
 * see uwb_slave.c. They are 0 when CIA had not finished for that frame. */
void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality);

#endif /* ANCHOR_RESPOND_H */
```

- [ ] **Step 2: Create the implementation**

`src/anchor_respond.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ported from the nRF5 anchor (fw-cre Src/anchor_respond.c). Both responders
 * keep their original shape; what changed is the short address on the wire
 * (the MAC contract reserves 0x0000 for the gateway) and the removal of
 * anchor_read_cir() -- under the callback API the CIA status bit is already
 * cleared by the time we run, so uwb_slave.c captures CIR instead.
 */

#include "anchor_respond.h"

#include "disc_schedule.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(anchor_respond, LOG_LEVEL_INF);

/* Legacy responder turnaround, unchanged from the nRF5 anchor and matching the
 * working ESP32S3UWB responder. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000u

/* ---- Legacy WAVE/0xE0 -> VEWA/0xE1 ---- */
static const uint8_t rx_poll_ref[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0
};

#define POS_ANCHOR_ID_IDX  10
#define POS_POLL_RX_TS_IDX 11
#define POS_RESP_TX_TS_IDX 15
#define POS_ANCHOR_X_IDX   19
#define POS_ANCHOR_Y_IDX   23

static uint8_t tx_resp_msg[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
	0,          /* anchor id  @10 */
	0, 0, 0, 0, /* poll_rx_ts @11..14 */
	0, 0, 0, 0, /* resp_tx_ts @15..18 */
	0, 0, 0, 0, /* x          @19..22 */
	0, 0, 0, 0, /* y          @23..26 */
	0, 0        /* FCS        @27..28 */
};

static void tx_delayed(const uint8_t *buf, uint16_t payload_len, uint32_t tx_time,
		       int ranging)
{
	dwt_setdelayedtrxtime(tx_time);
	dwt_writetxdata(payload_len, (uint8_t *)(uintptr_t)buf, 0);
	dwt_writetxfctrl((uint16_t)(payload_len + (ranging ? 0u : FCS_LEN)), 0, ranging);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("delayed TX missed its slot — response dropped");
		return;
	}

	/* Safe to poll: TXFRS is not in the enabled interrupt mask (uwb_slave.c
	 * enables DWT_INT_RX only), so dwt_isr() never runs for it and never
	 * clears the bit ahead of us. */
	uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK);
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq)
{
	/* The id byte the tag polls with is the low byte of our short address,
	 * because that is what it read out of our DISCOVERY response
	 * (tag uwb_net_runner.c: "anchor ID is taken from the low byte of
	 * src_addr"). Filter and reply must use the same value or the tag
	 * discards the response. */
	const uint8_t wire_id = (uint8_t)(uwb_config_short_addr(cfg) & 0xFFu);

	if (len < POS_ANCHOR_ID_IDX + 1) {
		return;
	}
	/* Byte 2 is the tag's per-poll sequence number — skip it. */
	if (memcmp(buf, rx_poll_ref, 2) != 0) {
		return;
	}
	if (memcmp(buf + 3, rx_poll_ref + 3, 7) != 0) {
		return;
	}
	if (buf[POS_ANCHOR_ID_IDX] != wire_id) {
		return;
	}

	uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
		((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
	uint64_t resp_tx_ts =
		(((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + cfg->ant_delay_tx;

	tx_resp_msg[2] = (*seq)++;
	tx_resp_msg[POS_ANCHOR_ID_IDX] = wire_id;
	uwb_resp_msg_set_ts(&tx_resp_msg[POS_POLL_RX_TS_IDX], poll_rx_ts);
	uwb_resp_msg_set_ts(&tx_resp_msg[POS_RESP_TX_TS_IDX], resp_tx_ts);
	memcpy(&tx_resp_msg[POS_ANCHOR_X_IDX], &cfg->x, sizeof(float));
	memcpy(&tx_resp_msg[POS_ANCHOR_Y_IDX], &cfg->y, sizeof(float));

	/* tx_resp_msg already carries the two FCS placeholder bytes, hence
	 * ranging=1 and no extra FCS_LEN in writetxfctrl. */
	tx_delayed(tx_resp_msg, sizeof(tx_resp_msg), resp_tx_time, 1);
}

void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality)
{
	if (!uwb_frame_is_discovery(buf, len)) {
		return;
	}

	uint16_t tag_addr = uwb_frame_get_src_addr(buf);
	uint8_t out[UWB_FRAME_LEN_RESP];

	int n = uwb_frame_response_build(out, sizeof(out),
					 uwb_config_short_addr(cfg), tag_addr,
					 0u, /* tx_ts unused for anchor selection */
					 cir_power, cir_quality);
	if (n < 0) {
		LOG_WRN("response build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(out, (*seq)++);

	LOG_INF("DISC from 0x%04X -> resp src 0x%04X pwr=%d q=%u", tag_addr,
		uwb_config_short_addr(cfg), cir_power, cir_quality);

	uint32_t delay_uus = disc_resp_delay_uus(cfg->anchor_id);
	uint32_t resp_tx_time = (uint32_t)((disc_rx_ts +
		((uint64_t)delay_uus * UUS_TO_DWT_TIME)) >> 8);

	/* Module frames carry no FCS placeholder: ranging=0 adds FCS_LEN. */
	tx_delayed(out, (uint16_t)n, resp_tx_time, 0);
}
```

- [ ] **Step 3: Add to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, alphabetically before `src/anchor_shell.c`:

```cmake
	src/anchor_respond.c
```

- [ ] **Step 4: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings. Nothing calls these yet.

- [ ] **Step 5: Commit**

```bash
git add src/anchor_respond.h src/anchor_respond.c CMakeLists.txt
git commit -m "feat(respond): port the WAVE and DISCOVERY responders

Both keep the shape they had in the nRF5 anchor. Three changes:

- the short address goes on the wire instead of the raw anchor_id, and
  the WAVE filter and the WAVE reply now use one value -- the tag checks
  the id it gets back against the id it polled with, so they cannot
  differ
- anchor_read_cir() is gone: under the callback API the ISR clears
  CIADONE before we run, so waiting on it would hang. uwb_slave.c
  captures CIR from the callback status snapshot instead
- test_run_info -> LOG_*

The TXFRS completion poll in tx_delayed() is only safe while TX
interrupts stay masked; the comment says so at the call site."
```

---

### Task 6: The interrupt-driven SLAVE loop

**Files:**
- Modify: `src/uwb_slave.c` (replaces the whole stub)

**Interfaces:**
- Consumes: `anchor_respond_wave_poll()`, `anchor_respond_discovery()` (Task 5); `uwb_get_rx_timestamp_u64()`, `FCS_LEN` (Task 3); `uwb_config_short_addr()` (Task 2); `uwb_frame_is_beacon()`, `uwb_frame_parse_beacon()`, `uwb_frame_beacon_find_addr()`, `UWB_FRAME_N_CFP` (Task 1).
- Produces: nothing — `uwb_slave_run()` is already declared in `src/uwb_modes.h` and called from `src/main.c`.

- [ ] **Step 1: Replace `src/uwb_slave.c` entirely**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SLAVE mode: an SS-TWR responder that also observes the gateway beacon.
 *
 * Interrupt-driven rather than polled. br101 runs dwt_isr() from the system
 * workqueue (platform/dw3000_hw.c), so the callbacks below are in thread
 * context -- but they still do the minimum and hand off, because the main
 * thread is ours to block and the workqueue is shared. A polling loop was
 * rejected outright: Zephyr's main thread is priority 0 and the shell thread
 * is 14, so a loop that never yields would starve the console.
 */

#include "uwb_modes.h"

#include "anchor_respond.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(uwb_slave, LOG_LEVEL_INF);

/* Longest frame the contract defines is a 39-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Written by the RX callbacks, read by the main thread. Safe without locking:
 * RX is not re-armed until the main thread has finished with the previous
 * event, so only one writer can be live at a time. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t frame_seq_nb;

static void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
	rx_status = cb_data->status;
	rx_len = cb_data->datalength;
	k_sem_give(&rx_sem);
}

static void cb_rx_fail(const dwt_cb_data_t *cb_data)
{
	rx_status = cb_data->status;
	rx_len = 0;
	k_sem_give(&rx_sem);
}

static void rx_arm(void)
{
	dwt_setpreambledetecttimeout(0);
	dwt_setrxtimeout(0);
	dwt_setrxaftertxdelay(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* Read the Ipatov CIR metrics, but only if CIA had finished for this frame.
 *
 * dwt_isr() clears SYS_STATUS_ALL_RX_GOOD -- which includes CIADONE -- before
 * invoking cb_rx_ok, so the register cannot be polled here; the pre-clear
 * snapshot in cb_data->status is the only evidence available. Reporting zero is
 * safe: the tag ranks anchors by these metrics, it does not accept or reject a
 * range on them. */
static void read_cir(uint32_t status, int32_t *cir_power, uint16_t *cir_quality)
{
	if (!(status & DWT_INT_CIADONE_BIT_MASK)) {
		*cir_power = 0;
		*cir_quality = 0;
		return;
	}

	dwt_rxdiag_t diag;

	memset(&diag, 0, sizeof(diag));
	dwt_readdiagnostics(&diag);
	/* Ipatov (preamble) metrics — STS is off in this PHY. */
	*cir_power = (int32_t)diag.ipatovPower;
	*cir_quality = diag.ipatovAccumCount;
}

static void observe_beacon(const uint8_t *buf, uint16_t len, const uwb_config_t *cfg)
{
	if (!uwb_frame_is_beacon(buf, len)) {
		return;
	}

	uint8_t proto_ver = 0;
	uint32_t frame_counter = 0;
	uint16_t slot_map[UWB_FRAME_N_CFP];
	uint8_t n_slots = 0;

	int rc = uwb_frame_parse_beacon(buf, len, &proto_ver, &frame_counter,
					slot_map, &n_slots);
	if (rc) {
		LOG_WRN("malformed beacon (%d)", rc);
		return;
	}
	/* slot_map cannot have overflowed above: parse_beacon derives n_slots as
	 * (len - 15) / 2, and uwb_frame_is_valid() already capped len at
	 * UWB_FRAME_MAX_LEN (39) == 15 + 2 * UWB_FRAME_N_CFP, so n_slots <= 12.
	 * This check exists to catch that identity being broken by a future
	 * change to either constant, not to protect the write. */
	if (n_slots > UWB_FRAME_N_CFP) {
		LOG_ERR("beacon parsed %u slots, buffer holds %u — "
			"UWB_FRAME_MAX_LEN and UWB_FRAME_N_CFP disagree",
			n_slots, UWB_FRAME_N_CFP);
		return;
	}

	int slot = uwb_frame_beacon_find_addr(slot_map, n_slots,
					      uwb_config_short_addr(cfg));

	/* Recorded and logged only. Gating RX windows on beacon timing needs a
	 * real gateway to develop against — that is spec D. */
	LOG_INF("BEACON ver=%u counter=%u slots=%u our_slot=%d", proto_ver,
		frame_counter, n_slots, slot);
}

void uwb_slave_run(const uwb_config_t *cfg)
{
	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* DWT_INT_RX is the RX-only mask. TXFRS must stay out of it: tx_delayed()
	 * polls for transmit completion, and an ISR that cleared the bit first
	 * would hang that poll. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	LOG_INF("{\"status\":\"listening\",\"mode\":\"slave\",\"id\":%u,"
		"\"short_addr\":\"0x%04X\"}",
		cfg->anchor_id, uwb_config_short_addr(cfg));

	rx_arm();

	while (1) {
		k_sem_take(&rx_sem, K_FOREVER);

		uint32_t status = rx_status;
		uint16_t flen = rx_len;

		/* flen counts the FCS. Anything that cannot hold one is not a
		 * frame we can reason about. */
		if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
			rx_arm();
			continue;
		}

		int32_t cir_power = 0;
		uint16_t cir_quality = 0;

		read_cir(status, &cir_power, &cir_quality);

		dwt_readrxdata(rx_buf, flen, 0);
		uint64_t rx_ts = uwb_get_rx_timestamp_u64();

		/* Payload length: every consumer below derives field counts from
		 * it, and two extra bytes corrupt them silently. */
		uint16_t plen = (uint16_t)(flen - FCS_LEN);

		/* Offered to each in turn; each ignores what is not its own. */
		anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg, &frame_seq_nb);
		anchor_respond_discovery(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
					 cir_power, cir_quality);
		observe_beacon(rx_buf, plen, cfg);

		rx_arm();
	}
}
```

- [ ] **Step 2: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings.

If `dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT)` fails to compile, check the third parameter's type in `deca_device_api.h:2438` — it is `dwt_INT_options_e`, and `DWT_ENABLE_INT` is one of its enumerators.

- [ ] **Step 3: Re-run every host test**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_frame\test_uwb_frame.exe `
    tests\uwb_frame\test_uwb_frame.c src\uwb_frame_802_15_4z.c
.\tests\uwb_frame\test_uwb_frame.exe
gcc -Wall -Wextra -o tests\uwb_config\test_uwb_config.exe `
    tests\uwb_config\test_uwb_config.c src\uwb_config.c
.\tests\uwb_config\test_uwb_config.exe
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe `
    tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
.\tests\disc_schedule\test_disc_schedule.exe
```

Expected: `ALL TESTS PASSED`, `PASSED`, `PASSED`.

- [ ] **Step 4: Commit**

```bash
git add src/uwb_slave.c
git commit -m "feat(slave): interrupt-driven SS-TWR responder and beacon observer

Replaces the spec A stub. RX callbacks capture the pre-clear status
snapshot and the frame length, then give a semaphore; the main thread
does every SPI access. Polling was rejected because Zephyr's main thread
outranks the shell thread, so a non-yielding loop would kill the console
the moment SLAVE mode started.

Two constraints are load-bearing and commented at their sites: CIR is
read from the cb_data->status snapshot rather than by polling CIADONE
(the ISR clears it first), and the interrupt mask is DWT_INT_RX so
tx_delayed()'s TXFRS poll still works.

Frame lengths are payload-only -- the radio reports payload + FCS, and
uwb_frame_parse_beacon() derives its slot count from the length, so an
unsubtracted FCS would parse a 12-slot beacon as 13."
```

---

### Task 7: Documentation

**Files:**
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: nothing.
- Produces: nothing.

- [ ] **Step 1: Add the new modules to the Layout section**

In `CLAUDE.md`, in the `## Layout` list, after the `src/uwb_radio.{c,h}` bullet:

```markdown
- `src/uwb_frame_802_15_4z.{c,h}` — 802.15.4z frame codec, **copied byte-for-byte**
  from `tag_testting/src/`. Keep it identical: the tag is the on-air peer, and
  divergence is a wire-format bug waiting to happen. Host-tested in
  `tests/uwb_frame/` with the tag's own suite.
- `src/uwb_dwtime.{c,h}` — the Qorvo `shared_functions` helpers the vendored
  module does not carry, plus `UUS_TO_DWT_TIME` and `FCS_LEN`.
- `src/disc_schedule.{c,h}` — per-anchor discovery response stagger. Host-tested
  in `tests/disc_schedule/`.
- `src/anchor_respond.{c,h}` — the WAVE/0xE1 and DISCOVERY/0xE4 responders.
```

and replace the `src/uwb_slave.c` / `src/uwb_gateway.c` bullet with:

```markdown
- `src/uwb_slave.c` — SLAVE mode: interrupt-driven SS-TWR responder plus beacon
  observe. `src/uwb_gateway.c` is still a stub until spec D.
```

- [ ] **Step 2: Add the hard-won facts**

In `CLAUDE.md`, append to `## Hard-won facts (do not re-derive)`:

```markdown
- **`dwt_getframelength()` and `cb_data->datalength` include the 2-byte FCS.**
  Confirmed on hardware in the sibling project (`ESP32S3UWB@9e1087b`: flen=13 for
  an 11-byte payload). Always pass `flen - FCS_LEN` to the frame module. The
  `is_*` validators tolerate the extra bytes because they test `len >=`, but
  `uwb_frame_parse_beacon()` derives its slot count from the length — an
  unsubtracted FCS turns a 12-slot beacon into 13, with the FCS read as the
  thirteenth slot. It fails silently and the output looks plausible.
- **Never poll `DWT_INT_CIADONE_BIT_MASK` after an RX event.** `dwt_isr()` clears
  `SYS_STATUS_ALL_RX_GOOD` — which includes CIADONE — *before* it calls
  `cbRxOk` (`dw3000_device.c:4764` then `:4791`). Waiting on it hangs until the
  next frame. Test `cb_data->status` instead: it is the pre-clear snapshot
  (`:4602`). The nRF5 `anchor_read_cir()` polls it and cannot be transcribed.
- **Keep `TXFRS` out of the enabled interrupt mask.** `dwt_setinterrupt(DWT_INT_RX,
  0, DWT_ENABLE_INT)` is deliberate: `anchor_respond.c`'s `tx_delayed()` polls for
  transmit completion, and enabling the TX interrupt would let the ISR clear
  TXFRS first and hang that poll — the same failure as CIADONE, on the TX side.
- **br101 runs `dwt_isr()` from the system workqueue, not the GPIO handler**
  (`platform/dw3000_hw.c:71-82`). DW3000 callbacks are therefore in thread
  context and may do SPI. That is what makes reading CIA diagnostics inside
  `cbRxOk` legal at all.
- **`UUS_TO_DWT_TIME` is 65536 here.** The nRF5 source says 63898. Both peers on
  this network — the tag and `ESP32S3UWB` — use 65536, and it scales the
  delayed-TX turnaround the two ends must agree on.
- **Anchor short address is `0x0001 + anchor_id`.** The MAC contract reserves
  `0x0000` for the gateway, so the console's 0-based id cannot go on the wire
  directly. The tag reports anchors as 1..4 while `anchor show` says 0..3; both
  are printed by `anchor show` and the boot banner.
```

- [ ] **Step 3: Add the new host tests to the Host tests section**

Replace the `## Host tests` code block in `CLAUDE.md` with:

````markdown
```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe          # PASSED, exits 0

gcc -Wall -Wextra -o tests/uwb_frame/test_uwb_frame.exe tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
./tests/uwb_frame/test_uwb_frame.exe            # ALL TESTS PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/disc_schedule/test_disc_schedule.exe tests/disc_schedule/test_disc_schedule.c src/disc_schedule.c
./tests/disc_schedule/test_disc_schedule.exe    # PASSED, exits 0
```
````

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record the SLAVE responder modules and five hard-won facts

The FCS-in-frame-length and CIADONE-cleared-before-callback facts each
cost real investigation and each fails in a way that looks like
something else -- silently plausible beacon output, and a hang that
looks like a radio fault."
```

---

## On-target verification

Runs after Task 7, in this order. Not a task: it needs hardware and a human.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

1. **Boot banner** reports `"id":0,"short_addr":"0x0001"` and the log shows
   `{"status":"listening","mode":"slave",...}` after the radio comes up.
2. **`anchor show`** over the USB-JTAG console reports the same, and the console
   stays responsive — proof the responder is not starving the shell thread.
3. **DISCOVERY.** With the tag powered, the anchor logs
   `DISC from 0x…  -> resp src 0x0001` and the tag adds anchor **1** to its pool.
4. **Ranging.** The tag prints a distance for anchor 1. This is the gate that
   matters — it is the only step that proves the turnaround timing, the antenna
   delays and `UUS_TO_DWT_TIME` are all right simultaneously.
5. **Stagger.** Flash a second board, set `anchor id 1` (short address 0x0002),
   reboot. Both answer one DISCOVERY and the tag pools two anchors. This is the
   only test that exercises `DISC_SLOT_UUS`.
6. **Beacon.** Skipped — no gateway exists until spec D. The path is exercised by
   Task 1's host suite (`test_beacon`), not on air.

If step 5 fails while step 4 passes, the stagger is too tight: `DISC_SLOT_UUS` is
marked *TUNE ON HARDWARE* and 3500 µs is an estimate carried from the nRF5
anchor, not a measurement on this board. Raise it and re-test.

If step 4 fails while step 3 passes, the DISCOVERY path works and the legacy WAVE
path does not — look first at the id byte, since the filter and the reply must
carry the same value (Task 5, step 2), and at `UUS_TO_DWT_TIME`.

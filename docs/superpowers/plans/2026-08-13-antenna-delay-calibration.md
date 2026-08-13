# Antenna-Delay Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the ANCLA anchor a calibration-only firmware image that trims its
`ant_delay_tx` against an OTP-calibrated DWM3001CDK, and cross-checks the result
anchor-to-anchor on pairs never used to calibrate anything.

**Architecture:** A separate build (`CONFIG_ANCLA_CAL_MODE`) whose main loop is an
ordinary WAVE responder that can be told, from the console, to become a temporary
SS-TWR *initiator*. It polls a chosen peer 128 times, rejects outliers, solves a
new TX antenna delay, applies it hot and persists it. The existing WAVE/`0xE0` →
VEWA/`0xE1` responder is the peer on the other end and is **not modified** — the
calibration therefore exercises the identical code path the tag uses.

**Tech Stack:** Zephyr 4.4.x, ESP32-S3, Qorvo DW3220 via the vendored br101
decadriver module, Zephyr shell + settings/NVS, host gcc for the pure-C tests.

**Spec:** `docs/superpowers/specs/2026-08-13-antenna-delay-calibration-design.md`

## Global Constraints

- Branch is `cal/antenna-delay`, already created off `master`.
- `$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"` is
  **required** for every build — this project lives outside the west workspace.
- Production behaviour must not change. `src/anchor_respond.c`, `src/uwb_slave.c`
  and `src/uwb_gateway.c` are **not edited by this plan**. The only production
  file touched is `src/main.c`, and only to add one `#ifdef` branch.
- Only `ant_delay_tx` is ever written. `ant_delay_rx` stays at
  `UWB_ANT_DELAY_DEFAULT` (16385). Only the sum is observable, so this is
  equivalent, not merely conventional — see spec §3.1.
- One combined-delay unit is 2.34 mm of reported distance, negative-going.
  `CAL_MM_PER_UNIT_X1000 = 2340`.
- `ant_delay_tx` is clamped to `[14000, 19000]`. A clamp hit is a **reported
  failure**, never silently accepted.
- Batch size is 128 samples — `CAL_MAX_SAMPLES` in the copied `cal_math.h`, which
  `cal_filtered_mean()` will reject anything above.
- Anchor console ids are 0-based; wire ids are `0x0001 + anchor_id`, so the id
  byte on the wire is `console_id + 1`. Never put a console id on the wire.
- `dwt_getframelength()` and `cb_data->datalength` **include** the 2-byte FCS.
  Always subtract `FCS_LEN` before reasoning about payload.
- Every busy-wait must be bounded. Copyright header on every new `.c`/`.h`:
  `Copyright (c) 2026 Innovaforce` / `SPDX-License-Identifier: Apache-2.0`.
- Tabs for indentation in `src/*.c`, 4 spaces in `tests/*.c` — matching each
  directory's existing files.

## External dependency — not built by this plan

The DWM3001CDK must run Qorvo's `ss_twr_responder` example with:

- PHY matched to `src/uwb_phy.h`: channel 5, PLEN_1024, PAC32, code 9, 850 kbps,
  SFD_IEEE_4Z, STS and PDoA off.
- Antenna delay loaded **from OTP**, not the example's hardcoded `16385`.
- `POLL_RX_TO_RESP_TX_DLY_UUS` raised from 450 to **2000**. At PLEN_1024 the
  preamble alone is ~1.05 ms, so 450 µus is physically impossible.

Tasks 1–3 can be completed and verified without it. Task 4's hardware check
needs it.

---

### Task 1: Pure-C solver and its host test

**Files:**
- Create: `src/cal_math.h` (verbatim copy of `../../tag_testting/src/cal_math.h`)
- Create: `src/cal_math.c` (verbatim copy of `../../tag_testting/src/cal_math.c`)
- Create: `src/cal_solve.h`
- Create: `src/cal_solve.c`
- Test: `tests/cal_solve/test_cal_solve.c`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly)` (from the copied `cal_math.h`)
  - `bool cal_filtered_mean(const int32_t *samples, size_t n, int32_t *out_mean, size_t *out_kept)` (from the copied `cal_math.h`)
  - `int cal_math_selftest(void)` (from the copied `cal_math.h`)
  - `#define CAL_MAX_SAMPLES 128u` (from the copied `cal_math.h`)
  - `int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm, uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx)`
  - `#define CAL_TX_DLY_MIN 14000u`, `#define CAL_TX_DLY_MAX 19000u`

- [ ] **Step 1: Copy the tag's solver verbatim**

Two file copies, no edits of any kind. From the repo root:

```powershell
Copy-Item "..\..\..\tag_testting\src\cal_math.h" "src\cal_math.h"
Copy-Item "..\..\..\tag_testting\src\cal_math.c" "src\cal_math.c"
```

If those relative paths do not resolve, the absolute source is
`C:\Users\JoseAntonioLaraPerez\Documents\tag_testting\src\`.

Verify the copy is byte-identical before continuing:

```powershell
Compare-Object (Get-Content "src\cal_math.c") (Get-Content "C:\Users\JoseAntonioLaraPerez\Documents\tag_testting\src\cal_math.c")
```

Expected: no output (files identical). Do **not** "fix" anything in these files —
the `cal_record` struct, `cal_crc32`, `cal_record_finalize`, `cal_record_valid`
and `cal_split_dly` are unused here on purpose, so that this stays a copy the tag
and the anchor share rather than a fork that drifts (spec §5.1).

- [ ] **Step 2: Write the failing test**

Create `tests/cal_solve/test_cal_solve.c`. Note 4-space indentation and the
`CHECK` macro, matching `tests/beacon_guard/test_beacon_guard.c`.

```c
#include "cal_solve.h"
#include "cal_math.h"

#include <errno.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* The tag's own vectors travel with the copied file. If these fail, the copy is
 * wrong -- not this project's arithmetic. */
static void test_tag_selftest_vectors_pass(void)
{
    CHECK(cal_math_selftest() == 0);
}

/* 234 mm too far at 2.34 mm/unit is +100 units of combined delay, and with
 * cur_rx pinned the whole +100 lands on tx. */
static void test_measuring_too_far_increases_tx(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2234, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16485);
}

/* Symmetric: measuring short pulls tx down by the same 100 units. */
static void test_measuring_too_short_decreases_tx(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(1766, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16285);
}

/* A perfect measurement must not move the delay at all. */
static void test_zero_error_is_a_fixed_point(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(2000, 2000, 16385, 16385, &tx) == 0);
    CHECK(tx == 16385);
}

/* cur_rx is held fixed, so an already-asymmetric pair keeps its rx and moves
 * only tx -- the correction is computed on the SUM but applied to tx alone. */
static void test_rx_is_held_fixed_and_correction_lands_on_tx(void)
{
    uint16_t tx = 0;

    /* Combined 16000 + 16385 = 32385, +100 units -> 32485, minus rx -> 16100. */
    CHECK(cal_solve_tx_delay(2234, 2000, 16000, 16385, &tx) == 0);
    CHECK(tx == 16100);
}

/* Out-of-range results are a reported failure, not a silent clamp: a 10 m error
 * means the setup is wrong (wrong peer, wrong tape, reflection), and quietly
 * pinning the delay at the rail would hide that. */
static void test_clamp_high_is_reported(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(12000, 2000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx == CAL_TX_DLY_MAX);
}

static void test_clamp_low_is_reported(void)
{
    uint16_t tx = 0;

    CHECK(cal_solve_tx_delay(0, 10000, 16385, 16385, &tx) == -ERANGE);
    CHECK(tx == CAL_TX_DLY_MIN);
}

/* The batch statistics the shell feeds cal_solve_tx_delay: a gross outlier from
 * a reflection or a half-decoded frame must not drag the mean. */
static void test_filtered_mean_rejects_a_gross_outlier(void)
{
    int32_t s[] = {2001, 1999, 2000, 2002, 1998, 9000};
    int32_t mean = 0;
    size_t kept = 0;

    CHECK(cal_filtered_mean(s, 6, &mean, &kept));
    CHECK(kept == 5);
    CHECK(mean >= 1998 && mean <= 2002);
}

int main(void)
{
    test_tag_selftest_vectors_pass();
    test_measuring_too_far_increases_tx();
    test_measuring_too_short_decreases_tx();
    test_zero_error_is_a_fixed_point();
    test_rx_is_held_fixed_and_correction_lands_on_tx();
    test_clamp_high_is_reported();
    test_clamp_low_is_reported();
    test_filtered_mean_rejects_a_gross_outlier();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}
```

- [ ] **Step 3: Run the test to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/cal_solve/test_cal_solve.exe tests/cal_solve/test_cal_solve.c src/cal_solve.c src/cal_math.c
```

Expected: FAIL — `cal_solve.h: No such file or directory`, and
`src/cal_solve.c` does not exist.

- [ ] **Step 4: Write `src/cal_solve.h`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The one piece of arithmetic this project adds on top of the tag's solver
 * (cal_math.c, copied verbatim): convert a solved COMBINED antenna delay back
 * into a TX-only value, holding ant_delay_rx fixed.
 *
 * Splitting is legitimate because only ant_tx + ant_rx is observable. The
 * responder derives its delayed TX time from poll_rx_ts, so raising ant_rx
 * makes it physically transmit earlier by exactly as much as raising ant_tx
 * makes it report a longer turnaround -- both move the initiator's result by
 * half a tick per unit. See the design spec, section 3.1; note that CLAUDE.md's
 * "RX_ANT_DLY cancels in RTD_resp" reaches the right conclusion by the wrong
 * route, and the wrong route makes ant_delay_rx look like a free parameter.
 *
 * Pure C with no Zephyr and no driver dependency, so it is host-testable like
 * uwb_config.c and beacon_guard.c.
 */

#ifndef CAL_SOLVE_H
#define CAL_SOLVE_H

#include <stdint.h>

/* Accepted ant_delay_tx range, about +/- 5.9 m of correction around the 16385
 * factory seed. Matches the sibling ESP-IDF project's DLY_MIN/DLY_MAX
 * (ESP32S3UWB/src/anchor_cal.c). A result outside it means the measurement was
 * wrong, not that the board needs that much trim. */
#define CAL_TX_DLY_MIN 14000u
#define CAL_TX_DLY_MAX 19000u

/* Solve a new ant_delay_tx from a batch mean, holding cur_rx fixed.
 *
 *   measured_mm  mean of the accepted samples
 *   ref_mm       true antenna-to-antenna distance
 *   cur_tx/cur_rx  delays currently programmed into the radio
 *   out_tx       receives the new value -- written even when the result is
 *                out of range, so the caller can report what was rejected
 *
 * Returns 0 on success, or -ERANGE if the result had to be clamped. The caller
 * must treat -ERANGE as a failed calibration and NOT persist the value. */
int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm,
		       uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx);

#endif /* CAL_SOLVE_H */
```

- [ ] **Step 5: Write `src/cal_solve.c`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cal_solve.h"

#include "cal_math.h"

#include <errno.h>

int cal_solve_tx_delay(int32_t measured_mm, int32_t ref_mm,
		       uint16_t cur_tx, uint16_t cur_rx, uint16_t *out_tx)
{
	/* The solve is on the combined delay -- that is the only observable
	 * quantity -- and the whole correction is then applied to tx, because
	 * cur_rx is pinned at UWB_ANT_DELAY_DEFAULT by the calling procedure. */
	uint32_t cur_total = (uint32_t)cur_tx + (uint32_t)cur_rx;
	uint16_t new_total = cal_solve_step(measured_mm, ref_mm,
					    (uint16_t)cur_total);

	int32_t new_tx = (int32_t)new_total - (int32_t)cur_rx;

	if (new_tx < (int32_t)CAL_TX_DLY_MIN) {
		*out_tx = (uint16_t)CAL_TX_DLY_MIN;
		return -ERANGE;
	}
	if (new_tx > (int32_t)CAL_TX_DLY_MAX) {
		*out_tx = (uint16_t)CAL_TX_DLY_MAX;
		return -ERANGE;
	}

	*out_tx = (uint16_t)new_tx;
	return 0;
}
```

- [ ] **Step 6: Run the test to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/cal_solve/test_cal_solve.exe tests/cal_solve/test_cal_solve.c src/cal_solve.c src/cal_math.c
./tests/cal_solve/test_cal_solve.exe
```

Expected: `PASSED`, exit code 0, and **no compiler warnings**. If
`test_tag_selftest_vectors_pass` is the only failure, the copy in Step 1 is
wrong — re-copy rather than editing the file.

- [ ] **Step 7: Commit**

```bash
git add src/cal_math.h src/cal_math.c src/cal_solve.h src/cal_solve.c tests/cal_solve/test_cal_solve.c
git commit -m "feat(cal): pure-C antenna-delay solver, host-tested"
```

---

### Task 2: The calibration build and its responder loop

Deliverable: a separate firmware image that boots, keeps the console, and
behaves as an ordinary WAVE responder — with no initiator yet. This is the
riskiest structural step, so it gets its own hardware gate before any radio
behaviour is added.

**Files:**
- Create: `Kconfig`
- Create: `cal.conf`
- Create: `src/cal_run.h`
- Create: `src/cal_run.c`
- Modify: `CMakeLists.txt:10-30`
- Modify: `src/main.c:58-76`

**Interfaces:**
- Consumes: `uwb_config_t` / `uwb_config_short_addr()` (`src/uwb_config.h`);
  `anchor_respond_wave_poll()` (`src/anchor_respond.h`);
  `uwb_get_rx_timestamp_u64()`, `FCS_LEN` (`src/uwb_dwtime.h`).
- Produces:
  - `void cal_run(const uwb_config_t *cfg)` — takes the main thread, never returns
  - `#define CAL_PEER_REFERENCE 0xFFu`
  - `struct cal_request { uint8_t peer_wire_id; int32_t ref_mm; bool persist; }`
  - `struct cal_result { uint32_t attempted, valid; size_t kept; int32_t mean_mm, ref_mm, error_mm; uint16_t old_tx, new_tx; int status; }`
  - `int cal_run_execute(const struct cal_request *req, struct cal_result *out)`
    — called from the shell thread, blocks until the batch finishes

- [ ] **Step 1: Create the project `Kconfig`**

The project has none today. Zephyr requires the application `Kconfig` to source
`Kconfig.zephyr` **last**, after the application's own symbols.

```kconfig
# Copyright (c) 2026 Innovaforce
# SPDX-License-Identifier: Apache-2.0

menu "ANCLA anchor"

config ANCLA_CAL_MODE
	bool "Antenna-delay calibration firmware"
	help
	  Build the calibration image instead of the production anchor.

	  main() hands the main thread to cal_run() rather than dispatching on
	  the configured mode, giving the board a `cal` console tree that lets
	  it act as a temporary SS-TWR initiator. Production images must never
	  enable this: a deployed anchor transmitting unsolicited polls would
	  collide with tag ranging traffic.

endmenu

source "Kconfig.zephyr"
```

- [ ] **Step 2: Create `cal.conf`**

An overlay applied on top of `prj.conf`, not a replacement — `EXTRA_CONF_FILE`
merges. It only needs to add the cal symbol and `CONFIG_POLL`, and switch off
the networking stack the cal image has no use for.

```
# Copyright (c) 2026 Innovaforce
# SPDX-License-Identifier: Apache-2.0
#
# Calibration image. Applied on top of prj.conf:
#   west build -b ancla_esp32s3/esp32s3/procpu -- -DEXTRA_CONF_FILE=cal.conf

CONFIG_ANCLA_CAL_MODE=y

# cal_run()'s loop waits on the RX semaphore and the shell request semaphore at
# the same time (k_poll), so a `cal` command is serviced between frames rather
# than only after one arrives.
CONFIG_POLL=y

# No gateway, no uplink: nothing in the cal image publishes anything. Turning
# these off also removes the WiFi blob's threads from the picture entirely, so
# nothing preempts a ranging exchange.
CONFIG_WIFI=n
CONFIG_NETWORKING=n
CONFIG_MQTT_LIB=n
CONFIG_MQTT_LIB_TLS=n
CONFIG_NET_SOCKETS=n
CONFIG_NET_SOCKETS_SOCKOPT_TLS=n
CONFIG_NET_TCP=n
CONFIG_NET_IPV4=n
CONFIG_NET_DHCPV4=n
CONFIG_DNS_RESOLVER=n
CONFIG_NET_LOG=n
```

- [ ] **Step 3: Write `src/cal_run.h`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAL mode: a WAVE responder that can be told, from the console, to become a
 * temporary SS-TWR initiator for one batch of exchanges.
 *
 * Only built when CONFIG_ANCLA_CAL_MODE is set. It exists as a separate image
 * rather than a third runtime mode because a deployed anchor must never
 * transmit an unsolicited poll.
 */

#ifndef CAL_RUN_H
#define CAL_RUN_H

#include "uwb_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Peer id meaning "the external reference node". No ANCLA anchor can match it:
 * wire ids are UWB_ANCHOR_ADDR_BASE + anchor_id, i.e. 0x01..0x04. The stock
 * Qorvo responder on the DWM3001CDK ignores the id byte entirely, so it answers
 * a poll addressed this way while every ANCLA on air stays silent. */
#define CAL_PEER_REFERENCE 0xFFu

struct cal_request {
	uint8_t peer_wire_id; /* CAL_PEER_REFERENCE, or 0x01 + console id */
	int32_t ref_mm;       /* true antenna-to-antenna distance */
	bool    persist;      /* true: solve, apply hot and save. false: report only */
};

struct cal_result {
	uint32_t attempted;   /* exchanges started */
	uint32_t valid;       /* exchanges that produced a distance */
	size_t   kept;        /* samples surviving outlier rejection */
	int32_t  mean_mm;
	int32_t  ref_mm;
	int32_t  error_mm;    /* mean_mm - ref_mm */
	uint16_t old_tx;      /* only meaningful when persist was true */
	uint16_t new_tx;
	int      status;      /* 0, or a negative errno */
};

/* Run one batch. Called from the shell thread; blocks until cal_run()'s loop
 * has finished the batch, then fills *out. Returns out->status, or -EBUSY if a
 * batch is already running, or -ETIMEDOUT if the loop never answered. */
int cal_run_execute(const struct cal_request *req, struct cal_result *out);

/* Takes over the main thread and does not return. */
void cal_run(const uwb_config_t *cfg);

#endif /* CAL_RUN_H */
```

- [ ] **Step 4: Write `src/cal_run.c` — responder duty only**

The initiator is stubbed in this task and filled in by Task 3. The RX callbacks
and their `rx_pending` guard are lifted from `src/uwb_slave.c:61-89`; the guard
is load-bearing for the reason documented there, so do not simplify it away.

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_run.h. The RX path mirrors uwb_slave.c deliberately -- it is the
 * production responder, and the whole point of calibrating in this image is
 * that the exchange being measured is the one the tag actually uses.
 */

#include "cal_run.h"

#include "anchor_respond.h"
#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <errno.h>

LOG_MODULE_REGISTER(cal_run, LOG_LEVEL_INF);

#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);
static K_SEM_DEFINE(req_sem, 0, 1);
static K_SEM_DEFINE(done_sem, 0, 1);

/* Handed between the shell thread and the main loop, one batch at a time. The
 * semaphore pair is the handshake: the shell writes g_req before giving
 * req_sem and reads g_res only after taking done_sem, so the two never touch
 * either struct at once. g_busy rejects a second command while one is running
 * rather than letting it corrupt the first. */
static struct cal_request g_req;
static struct cal_result g_res;
static bool g_busy;

/* See uwb_slave.c:38-51 for why rx_pending is required. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t frame_seq_nb;

static void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = cb_data->datalength;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

static void cb_rx_fail(const dwt_cb_data_t *cb_data)
{
	if (rx_pending) {
		return;
	}
	rx_status = cb_data->status;
	rx_len = 0;
	rx_pending = true;
	k_sem_give(&rx_sem);
}

static void rx_arm(void)
{
	dwt_setpreambledetecttimeout(0);
	dwt_setrxtimeout(0);
	dwt_setrxaftertxdelay(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

int cal_run_execute(const struct cal_request *req, struct cal_result *out)
{
	if (g_busy) {
		return -EBUSY;
	}
	g_busy = true;
	g_req = *req;
	k_sem_give(&req_sem);

	/* Generous: a fully failing 128-sample batch is 128 * (10 + 25) ms of
	 * timeouts plus the inter-sample sleep, about 6 s. */
	if (k_sem_take(&done_sem, K_SECONDS(30)) != 0) {
		g_busy = false;
		return -ETIMEDOUT;
	}

	*out = g_res;
	g_busy = false;
	return g_res.status;
}

/* Filled in by Task 3. */
static void run_batch(const struct cal_request *req, struct cal_result *res,
		      uwb_config_t *cfg)
{
	ARG_UNUSED(req);
	ARG_UNUSED(cfg);

	res->status = -ENOSYS;
}

void cal_run(const uwb_config_t *cfg)
{
	/* Mutable, unlike uwb_slave_run()'s const snapshot: a `cal ref` applies
	 * its solved delay hot, and this copy and the radio's TX antenna delay
	 * register must move together. anchor_respond.c:141 adds
	 * cfg->ant_delay_tx in software to build the reported resp_tx_ts while
	 * the radio applies the register to the frame it actually sends; if the
	 * two disagree this board silently reports a turnaround it did not
	 * perform. cal_run owns this copy outright -- the shell thread never
	 * touches it. */
	static uwb_config_t cfg_live;

	cfg_live = *cfg;

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* RX only, same as the production responder: TXFRS must stay out of the
	 * mask or the responder's tx_delayed() completion poll would hang. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	LOG_INF("{\"status\":\"listening\",\"mode\":\"cal\",\"id\":%u,"
		"\"short_addr\":\"0x%04X\",\"ant_tx\":%u,\"ant_rx\":%u}",
		cfg_live.anchor_id, uwb_config_short_addr(&cfg_live),
		cfg_live.ant_delay_tx, cfg_live.ant_delay_rx);

	struct k_poll_event events[2] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&rx_sem, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&req_sem, 0),
	};

	rx_arm();

	while (1) {
		k_poll(events, 2, K_FOREVER);

		if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
			k_sem_take(&req_sem, K_NO_WAIT);

			struct cal_result res = {0};

			res.ref_mm = g_req.ref_mm;
			run_batch(&g_req, &res, &cfg_live);
			g_res = res;

			rx_arm();
			k_sem_give(&done_sem);
		}

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			k_sem_take(&rx_sem, K_NO_WAIT);

			uint16_t flen = rx_len;

			rx_pending = false;

			if (flen > FCS_LEN && flen <= RX_BUF_LEN) {
				dwt_readrxdata(rx_buf, flen, 0);

				uint64_t rx_ts = uwb_get_rx_timestamp_u64();
				uint16_t plen = (uint16_t)(flen - FCS_LEN);

				/* No beacon guard: there is no gateway on air
				 * during calibration, and
				 * beacon_guard_tx_allowed() returns true while
				 * unlocked in any case (beacon_guard.c:37). */
				anchor_respond_wave_poll(rx_buf, plen, rx_ts,
							 &cfg_live,
							 &frame_seq_nb, NULL);
			}

			rx_arm();
		}

		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;
	}
}
```

- [ ] **Step 5: Add the conditional sources to `CMakeLists.txt`**

Append after the existing `target_sources(app PRIVATE ...)` block, leaving that
block untouched:

```cmake
# Calibration image only (see cal.conf, docs/superpowers/specs/
# 2026-08-13-antenna-delay-calibration-design.md). Deliberately excluded from
# the production build: a deployed anchor must never initiate a poll.
target_sources_ifdef(CONFIG_ANCLA_CAL_MODE app PRIVATE
	src/cal_initiator.c
	src/cal_math.c
	src/cal_run.c
	src/cal_shell.c
	src/cal_solve.c
)
```

`src/cal_initiator.c` and `src/cal_shell.c` do not exist yet, so create both as
minimal placeholders in this task so the build links — Tasks 3 and 4 fill them:

`src/cal_initiator.c`:
```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
```

`src/cal_shell.c`:
```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
```

- [ ] **Step 6: Dispatch to `cal_run()` from `main.c`**

Add the include alongside the existing ones at `src/main.c:17-22`:

```c
#ifdef CONFIG_ANCLA_CAL_MODE
#include "cal_run.h"
#endif
```

Then replace the mode dispatch at `src/main.c:58-76` with:

```c
#ifdef CONFIG_ANCLA_CAL_MODE
	/* The calibration image ignores the configured mode entirely: it is
	 * neither a SLAVE nor a GATEWAY, and it must not beacon. */
	cal_run(cfg);
#else
	if (cfg->mode == UWB_MODE_GATEWAY) {
```

...leaving the existing GATEWAY body and `else { uwb_slave_run(cfg); }` exactly
as they are, and closing with `#endif` after the `else` block's closing brace.
Do not reindent or reword the GATEWAY comment block — it documents a
bench-confirmed constraint.

- [ ] **Step 7: Verify both images still build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
```

Expected: both succeed. The production build must **not** compile any `cal_*.c`
file — confirm by checking the build log or `build/CMakeFiles` for their absence.

- [ ] **Step 8: Hardware gate — the cal image is a working responder**

Flash the cal image to one anchor and a production image to nothing else yet.

```powershell
west flash
west espressif monitor -p COM5
```

Expected on the console:
- the boot banner, then `{"status":"listening","mode":"cal",...}`
- `uwb:~$` prompt alive; `anchor show` prints the config

Then, with a second board running the production SLAVE image and a tag (or the
DWM3001CDK) generating WAVE polls, confirm on the sniffer that the cal-image
board answers `0xE0` with `0xE1` exactly as the production image does. If it does
not, the fault is in this task's RX loop, not in anything later.

- [ ] **Step 9: Commit**

```bash
git add Kconfig cal.conf CMakeLists.txt src/main.c src/cal_run.h src/cal_run.c src/cal_initiator.c src/cal_shell.c
git commit -m "feat(cal): separate calibration image with a responder-only loop"
```

---

### Task 3: The SS-TWR initiator and `cal peer`

Deliverable: two anchors can range each other from the console. The numbers will
be *wrong* (nothing is calibrated yet) but must be stable and plausible — within
a couple of metres of the tape distance, not random.

**Files:**
- Create: `src/cal_initiator.h`
- Modify: `src/cal_initiator.c` (replace the Task 2 placeholder)
- Modify: `src/cal_run.c` (`run_batch()`)
- Modify: `src/cal_shell.c` (replace the Task 2 placeholder)

**Interfaces:**
- Consumes: `struct cal_request`, `struct cal_result`, `cal_run_execute()`,
  `CAL_PEER_REFERENCE` (Task 2); `cal_filtered_mean()`, `CAL_MAX_SAMPLES`
  (Task 1); `UWB_MAX_ANCHORS`, `UWB_ANCHOR_ADDR_BASE` (`src/uwb_config.h`).
- Produces:
  - `int32_t cal_initiator_range(uint8_t peer_wire_id)` — mm, or `INT32_MIN`
  - `void cal_initiator_enter(void)` / `void cal_initiator_leave(void)`

- [ ] **Step 1: Write `src/cal_initiator.h`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One SS-TWR exchange with this board as the initiator -- the role the tag
 * normally plays. Used only by the calibration image.
 *
 * Interrupts are disabled for the duration (cal_initiator_enter/_leave) and the
 * exchange polls SYS_STATUS instead. That inverts the production responder's
 * rule, and deliberately: CLAUDE.md's warning against polling TXFRS/CIADONE
 * applies because dwt_isr() clears those bits before the callback runs. With no
 * ISR enabled there is nothing to clear them first, which is also what Qorvo's
 * own ss_twr_initiator example does. Every poll here is bounded.
 */

#ifndef CAL_INITIATOR_H
#define CAL_INITIATOR_H

#include <stdint.h>

/* Put the radio into initiator mode. Must be paired with cal_initiator_leave();
 * between the two, this board does not answer anything. */
void cal_initiator_enter(void);

/* Restore the RX state the responder loop expects. */
void cal_initiator_leave(void);

/* One poll/response exchange with the peer whose wire id is peer_wire_id.
 * Returns the distance in mm (clock-offset corrected), or INT32_MIN if the
 * response never arrived, was malformed, or came from an unexpected layout. */
int32_t cal_initiator_range(uint8_t peer_wire_id);

#endif /* CAL_INITIATOR_H */
```

- [ ] **Step 2: Write `src/cal_initiator.c`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_initiator.h.
 */

#include "cal_initiator.h"

#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(cal_initiator, LOG_LEVEL_INF);

/* Speed of light in air as every peer on this network uses it: the tag
 * (uwb_ss_initiator.c) and the sibling ESP-IDF anchor (ranging.c) both use
 * 299702547.0. A different constant here would show up as a fixed scale error. */
#define SPEED_OF_LIGHT 299702547.0
#define DWT_TIME_UNITS_V (1.0 / 499.2e6 / 128.0)

/* RX opens this long after our poll finishes. The response preamble cannot
 * arrive sooner: the peer's RMARKER is 2000 uus after its own poll_rx_ts and
 * the PLEN_1024 preamble runs ~1.05 ms ahead of that. */
#define RX_AFTER_TX_UUS 300U

/* RX window. Both peer types turn around in 2000 uus (the DWM3001CDK's stock
 * 450 is raised to match -- at PLEN_1024, 450 is shorter than the preamble and
 * physically impossible), so one window covers both. */
#define RESP_TIMEOUT_UUS 4000U

#define TX_DONE_TIMEOUT_MS 10U
#define RX_DONE_TIMEOUT_MS 25U

#define ALL_MSG_COMMON_LEN 10U
#define ALL_MSG_SN_IDX 2U
#define POLL_PEER_ID_IDX 10U

/* Stock Qorvo ss_twr_responder response: 18 payload bytes. */
#define RESP_LEN_STOCK 18U
#define RESP_TS_IDX_STOCK_POLL_RX 10U
#define RESP_TS_IDX_STOCK_RESP_TX 14U

/* ANCLA VEWA response (anchor_respond.c:73): 27 payload bytes, both timestamp
 * fields pushed one byte later by the anchor-id byte at index 10. */
#define RESP_LEN_ANCLA 27U
#define RESP_TS_IDX_ANCLA_POLL_RX 11U
#define RESP_TS_IDX_ANCLA_RESP_TX 15U

#define RX_BUF_LEN 64

/* 11 bytes: the 10 stock Qorvo header bytes plus a peer-id byte. The stock
 * responder compares only the first ALL_MSG_COMMON_LEN and ignores the
 * eleventh; the ANCLA responder requires it and filters on it
 * (anchor_respond.c:124,134). One frame therefore addresses either peer, and
 * exactly one board ever answers. */
static uint8_t tx_poll[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0,
	0 /* peer id @10 */
};

static const uint8_t rx_resp_ref[] = {
	0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1
};

static uint8_t poll_seq;

void cal_initiator_enter(void)
{
	dwt_forcetrxoff();
	/* Every interrupt off: this path polls SYS_STATUS, and an enabled ISR
	 * would clear TXFRS/RXFCG before the poll could see them. */
	dwt_setinterrupt(0xFFFFFFFFU, 0xFFFFFFFFU, DWT_DISABLE_INT);
	dwt_setrxaftertxdelay(RX_AFTER_TX_UUS);
	dwt_setrxtimeout(RESP_TIMEOUT_UUS);
	dwt_setpreambledetecttimeout(0);
}

void cal_initiator_leave(void)
{
	dwt_forcetrxoff();
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);
}

/* Bounded wait for ANY bit in mask. uwb_wait_for_sysstatus_lo() waits for ALL
 * of them, which cannot express "RXFCG or RXTO or RXERR". */
static uint32_t wait_any_sysstatus_lo(uint32_t mask, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	for (;;) {
		uint32_t status = dwt_readsysstatuslo();

		if (status & mask) {
			return status & mask;
		}
		if (k_uptime_get() > deadline) {
			return 0;
		}
	}
}

static uint32_t ts_from(const uint8_t *p)
{
	uint32_t ts = 0;

	for (int i = 0; i < 4; i++) {
		ts |= (uint32_t)p[i] << (i * 8);
	}
	return ts;
}

int32_t cal_initiator_range(uint8_t peer_wire_id)
{
	tx_poll[ALL_MSG_SN_IDX] = poll_seq++;
	tx_poll[POLL_PEER_ID_IDX] = peer_wire_id;

	dwt_writetxdata(sizeof(tx_poll), tx_poll, 0);
	dwt_writetxfctrl(sizeof(tx_poll) + FCS_LEN, 0, 1);

	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) !=
	    DWT_SUCCESS) {
		return INT32_MIN;
	}

	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_DONE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		return INT32_MIN;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	uint32_t rx_mask = DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
			   SYS_STATUS_ALL_RX_ERR;
	uint32_t got = wait_any_sysstatus_lo(rx_mask, RX_DONE_TIMEOUT_MS);

	if (!(got & DWT_INT_RXFCG_BIT_MASK)) {
		dwt_forcetrxoff();
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		return INT32_MIN;
	}
	dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

	uint8_t rng = 0;
	uint16_t flen = dwt_getframelength(&rng);

	if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
		return INT32_MIN;
	}

	uint8_t buf[RX_BUF_LEN];

	dwt_readrxdata(buf, flen, 0);

	/* flen includes the FCS -- always subtract before deriving anything
	 * from the length, which is exactly what the layout choice below does. */
	uint16_t plen = (uint16_t)(flen - FCS_LEN);

	buf[ALL_MSG_SN_IDX] = 0;
	if (memcmp(buf, rx_resp_ref, ALL_MSG_COMMON_LEN) != 0) {
		return INT32_MIN;
	}

	uint16_t poll_rx_idx, resp_tx_idx;

	if (plen == RESP_LEN_ANCLA) {
		poll_rx_idx = RESP_TS_IDX_ANCLA_POLL_RX;
		resp_tx_idx = RESP_TS_IDX_ANCLA_RESP_TX;
	} else if (plen == RESP_LEN_STOCK) {
		poll_rx_idx = RESP_TS_IDX_STOCK_POLL_RX;
		resp_tx_idx = RESP_TS_IDX_STOCK_RESP_TX;
	} else {
		LOG_DBG("unknown response layout, plen=%u", plen);
		return INT32_MIN;
	}

	uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
	uint32_t resp_rx_ts = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
	uint32_t poll_rx_ts = ts_from(&buf[poll_rx_idx]);
	uint32_t resp_tx_ts = ts_from(&buf[resp_tx_idx]);

	/* Mandatory, not a refinement. SS-TWR range error from crystal offset
	 * is c/2 * ppm * T_reply; at this project's 2000 uus turnaround 5 ppm is
	 * ~1.5 m, which dwarfs the antenna-delay error being measured. Both
	 * existing peers correct for it identically -- tag uwb_ss_initiator.c
	 * and ESP32S3UWB ranging.c:301. */
	double clk_off = ((double)dwt_readclockoffset()) / (double)(1U << 26);
	int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
	int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
	double tof = ((rtd_init - rtd_resp * (1.0 - clk_off)) / 2.0) *
		     DWT_TIME_UNITS_V;

	return (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
}
```

- [ ] **Step 3: Implement `run_batch()` in `src/cal_run.c`**

Replace the Task 2 stub. Add `#include "cal_initiator.h"`, `#include
"cal_math.h"` and `#include "cal_solve.h"` to the includes.

```c
/* Static, not on the stack: CONFIG_MAIN_STACK_SIZE is 4096 and this is 512 B,
 * which is a large fraction of it. */
static int32_t samples[CAL_MAX_SAMPLES];

static void run_batch(const struct cal_request *req, struct cal_result *res,
		      uwb_config_t *cfg)
{
	uint32_t valid = 0;

	cal_initiator_enter();

	for (uint32_t i = 0; i < CAL_MAX_SAMPLES; i++) {
		int32_t mm = cal_initiator_range(req->peer_wire_id);

		if (mm != INT32_MIN) {
			samples[valid++] = mm;
		}

		/* Yields to the shell (priority 14) so the console stays
		 * responsive through the batch, and decorrelates consecutive
		 * exchanges slightly. */
		k_sleep(K_MSEC(2));
	}

	cal_initiator_leave();

	res->attempted = CAL_MAX_SAMPLES;
	res->valid = valid;

	/* Enough samples to mean anything. Below this the peer is not really
	 * answering -- wrong PHY, wrong peer id, or out of range -- and a mean
	 * over a handful of lucky frames would look like a calibration. */
	if (valid < CAL_MAX_SAMPLES / 4U) {
		res->status = -ENODATA;
		return;
	}

	if (!cal_filtered_mean(samples, valid, &res->mean_mm, &res->kept)) {
		res->status = -EIO;
		return;
	}

	res->error_mm = res->mean_mm - res->ref_mm;

	if (!req->persist) {
		res->status = 0;
		return;
	}

	res->old_tx = cfg->ant_delay_tx;

	int rc = cal_solve_tx_delay(res->mean_mm, res->ref_mm,
				    cfg->ant_delay_tx, cfg->ant_delay_rx,
				    &res->new_tx);

	if (rc) {
		res->status = rc;
		return;
	}

	/* Applied hot, and the snapshot and the register move together -- see
	 * the comment on cfg_live in cal_run(). Persistence is the shell's job:
	 * it owns the uwb_store call, this loop owns the radio. */
	cfg->ant_delay_tx = res->new_tx;
	dwt_settxantennadelay(res->new_tx);

	res->status = 0;
}
```

- [ ] **Step 4: Write `src/cal_shell.c` with `cal peer` only**

`cal ref` is added in Task 4.

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `cal` command tree. Calibration image only.
 *
 * Both commands submit to cal_run()'s loop and block; the radio work never
 * happens on the shell thread, because two threads on the DW3220's SPI bus at
 * once would corrupt both.
 */

#include "cal_run.h"
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

/* Same reasoning as anchor_shell.c's parse_ul: strtoul() reports a non-numeric
 * argument as 0, and 0 is a legal distance. */
static bool parse_l(const char *arg, long *out)
{
	char *endptr;
	long v;

	v = strtol(arg, &endptr, 0);
	if (endptr == arg || *endptr != '\0') {
		return false;
	}

	*out = v;
	return true;
}

static void print_result(const struct shell *sh, const struct cal_result *r)
{
	shell_print(sh,
		    "{\"attempted\":%u,\"valid\":%u,\"kept\":%u,"
		    "\"mean_mm\":%d,\"ref_mm\":%d,\"error_mm\":%d}",
		    r->attempted, r->valid, (unsigned int)r->kept,
		    r->mean_mm, r->ref_mm, r->error_mm);
}

static void print_failure(const struct shell *sh, int status)
{
	switch (status) {
	case -ENODATA:
		shell_error(sh,
			    "error: too few valid responses — check the peer is "
			    "powered, addressed correctly, and on the same PHY");
		break;
	case -EBUSY:
		shell_error(sh, "error: a calibration batch is already running");
		break;
	case -ETIMEDOUT:
		shell_error(sh, "error: the ranging loop did not answer");
		break;
	default:
		shell_error(sh, "error: calibration failed (errno %d)", status);
		break;
	}
}

static int cmd_peer(const struct shell *sh, size_t argc, char **argv)
{
	struct cal_request req = {0};
	struct cal_result res = {0};
	long id, mm;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_l(argv[1], &id) || id < 0 || id >= UWB_MAX_ANCHORS) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}
	if (!parse_l(argv[2], &mm) || mm <= 0) {
		shell_error(sh, "error: distance must be a positive integer in mm");
		return -EINVAL;
	}

	/* Console ids are 0-based; the wire id is UWB_ANCHOR_ADDR_BASE +
	 * anchor_id, and the responder filters on its low byte. */
	req.peer_wire_id = (uint8_t)((UWB_ANCHOR_ADDR_BASE + id) & 0xFFu);
	req.ref_mm = (int32_t)mm;
	req.persist = false;

	ret = cal_run_execute(&req, &res);
	if (ret) {
		print_failure(sh, ret);
		return ret;
	}

	print_result(sh, &res);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cal,
	SHELL_CMD_ARG(peer, NULL,
		      "peer <id> <mm> — range anchor <id> at a known distance; "
		      "reports only, never persists",
		      cmd_peer, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cal, &sub_cal, "Antenna-delay calibration", NULL);
```

- [ ] **Step 5: Build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
```

Expected: success, no warnings. If `dwt_readsysstatuslo`,
`SYS_STATUS_ALL_RX_TO` or `DWT_COMPAT_NONE` do not resolve, check the vendored
driver's `deca_device_api.h` for the exact spelling before inventing one — do
not guess an API that is not there.

- [ ] **Step 6: Hardware gate — two anchors range each other**

Flash the cal image to **two** anchors, give them different `anchor id`s, place
them a tape-measured 2 m apart with clear line of sight, and reboot both.

From anchor 0's console:

```
cal peer 1 2000
```

Expected:
- `valid` close to 128 (a handful of misses is normal; fewer than ~100 means
  something is wrong with the setup, not the code)
- `kept` within a few of `valid`
- `mean_mm` **stable across repeated runs** — repeat the command three times and
  the means should agree within a few tens of mm
- `error_mm` anywhere in roughly ±2000. It is *supposed* to be wrong: neither
  board is calibrated. Stability is the acceptance criterion here, not accuracy.

Then confirm the responder duty survived: `cal peer 1 2000` from anchor 1
addressed back at anchor 0 must work too, and neither board should stop
answering after a batch (that would mean `cal_initiator_leave()` is not
restoring RX).

- [ ] **Step 7: Commit**

```bash
git add src/cal_initiator.h src/cal_initiator.c src/cal_run.c src/cal_shell.c
git commit -m "feat(cal): SS-TWR initiator and the cal peer cross-check command"
```

---

### Task 4: `cal ref` — solve, apply hot, persist

Deliverable: an anchor calibrates itself against the DWM3001CDK.

**Files:**
- Modify: `src/cal_shell.c`

**Interfaces:**
- Consumes: everything from Tasks 1–3, plus `uwb_config_get()`,
  `uwb_config_set_ant()` (`src/uwb_config.h`) and `uwb_store_save_ant()`
  (`src/uwb_store.h`).
- Produces: no new interfaces.

- [ ] **Step 1: Add `cal ref` to `src/cal_shell.c`**

Add `#include "uwb_store.h"` to the includes, then this command before the
`SHELL_STATIC_SUBCMD_SET_CREATE` block:

```c
static int cmd_ref(const struct shell *sh, size_t argc, char **argv)
{
	struct cal_request req = {0};
	struct cal_result res = {0};
	uwb_config_t *cfg = uwb_config_get();
	long mm;
	int ret;

	ARG_UNUSED(argc);

	if (!parse_l(argv[1], &mm) || mm <= 0) {
		shell_error(sh, "error: distance must be a positive integer in mm");
		return -EINVAL;
	}

	req.peer_wire_id = CAL_PEER_REFERENCE;
	req.ref_mm = (int32_t)mm;
	req.persist = true;

	ret = cal_run_execute(&req, &res);
	if (ret == -ERANGE) {
		shell_error(sh,
			    "error: solved ant_tx out of range (would be clamped "
			    "to %u) from error=%d mm — NOT applied. Check the "
			    "reference distance and that the peer is the "
			    "reference node.",
			    res.new_tx, res.error_mm);
		return ret;
	}
	if (ret) {
		print_failure(sh, ret);
		return ret;
	}

	print_result(sh, &res);

	/* cal_run() has already applied the new delay to the radio and to its
	 * own live config. This updates the shared config singleton and writes
	 * NVS, so the value survives a reboot. */
	if (!uwb_config_set_ant(cfg, res.new_tx, cfg->ant_delay_rx)) {
		shell_error(sh, "error: ant_tx=%u rejected by the config layer",
			    res.new_tx);
		return -EINVAL;
	}

	ret = uwb_store_save_ant();
	if (ret) {
		shell_error(sh,
			    "error: ant_tx %u -> %u applied to the radio but NOT "
			    "persisted (errno %d) — will be lost on reboot",
			    res.old_tx, res.new_tx, ret);
		return ret;
	}

	shell_print(sh,
		    "ok: ant_tx %u -> %u (applied and saved) — run `cal ref %ld` "
		    "again to read the residual",
		    res.old_tx, res.new_tx, mm);
	return 0;
}
```

Then add it to the subcommand set, keeping `peer`:

```c
SHELL_STATIC_SUBCMD_SET_CREATE(sub_cal,
	SHELL_CMD_ARG(ref,  NULL,
		      "ref <mm> — calibrate against the reference node at a "
		      "known distance; applies and persists ant_tx",
		      cmd_ref, 2, 0),
	SHELL_CMD_ARG(peer, NULL,
		      "peer <id> <mm> — range anchor <id> at a known distance; "
		      "reports only, never persists",
		      cmd_peer, 3, 0),
	SHELL_SUBCMD_SET_END
);
```

- [ ] **Step 2: Build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu -- -DEXTRA_CONF_FILE=cal.conf
```

Expected: success, no warnings.

- [ ] **Step 3: Hardware gate — calibrate one anchor**

Requires the DWM3001CDK prepared per "External dependency" above. Place it a
tape-measured ≥ 2 m from the anchor, antenna phase centre to antenna phase
centre, clear line of sight, equal height, matched orientation, clear of metal.
All tags powered off.

```
cal ref 2000
cal ref 2000
```

Expected:
- first run reports a large `error_mm` (this is the uncalibrated offset) and
  `ok: ant_tx 16385 -> <new>`
- second run reports `|error_mm| < 15` — the delay was applied hot, so no reboot
  in between
- `anchor show` reports the new `ant_tx` and `ant_rx` still 16385
- after `kernel reboot cold`, `anchor show` still reports the new `ant_tx`

If the first run returns `-ERANGE`, do not adjust the clamp. Check the reference
distance, that the DWM is actually running the responder, and that its PHY and
turnaround match.

- [ ] **Step 4: Hardware gate — all three anchors, then the acceptance test**

Repeat Step 3 for the remaining two anchors. Then, with all three trimmed, run
the cross-check on each pair at tape-measured distances:

```
cal peer 1 <mm>
cal peer 2 <mm>
```
(and from anchor 1: `cal peer 2 <mm>`)

**Acceptance: `|error_mm| < 30` on all three pairs.** These pairs were never used
to calibrate anything, so this is independent evidence.

If Step 3 passed on every board but this fails by a similar amount on every
pair, suspect the DWM's OTP delay is not actually being loaded — that failure
mode shows up here and nowhere else.

- [ ] **Step 5: Commit**

```bash
git add src/cal_shell.c
git commit -m "feat(cal): cal ref solves, applies and persists the TX antenna delay"
```

---

### Task 5: End-to-end verification and documentation

**Files:**
- Create: `docs/antenna-delay-calibration.md`
- Modify: `CLAUDE.md`

- [ ] **Step 1: Hardware gate — the tag's residual collapses**

This is spec §7 criterion 4, and it is the only step that measures the thing the
whole branch exists for. It needs the production image, not the cal image.

Reflash all three calibrated anchors and the gateway with the **production**
build (`west build --pristine` with no `EXTRA_CONF_FILE`, then `west flash`).
The calibrated `ant_delay_tx` values survive: they live in NVS under `anchor/`,
which reflashing the application does not erase. Confirm with `anchor show` on
each board before proceeding — if any reads 16385, its calibration was lost and
Task 4 must be repeated for that board.

Set the three anchors' `anchor pos` coordinates to a hand-measured geometry,
reboot, power a tag, and watch the gateway console.

Expected in the `pos_sink` JSON line, which is the only place `residual` is
visible:
- `residual` under ~0.1 m, against the 0.48–2.0 m recorded in `CLAUDE.md`
- `(x, y)` stable between consecutive fixes with the tag stationary, against the
  ~0.7 m wander recorded there

Record the actual numbers — they go into `CLAUDE.md` in Step 4.

If the residual stays large *after* Task 4's cross-check passed at < 30 mm, the
remaining error is geometry, not antenna delay: that is URGENT item 2
(auto-positioning) and explicitly not this branch's to fix. Say so plainly in the
commit message rather than reopening the calibration.

- [ ] **Step 2: Write the operator procedure**

Create `docs/antenna-delay-calibration.md` covering, in runnable order: the
DWM3001CDK prerequisites from "External dependency" above; the cal build and
flash commands; the physical setup from Task 4 Step 3; the per-anchor `cal ref`
sequence; the `cal peer` acceptance test with its < 30 mm threshold; and a short
troubleshooting list mapping each error the shell can print (`-ENODATA`,
`-ERANGE`, `-ETIMEDOUT`) to its likely physical cause.

State explicitly that tape error enters the calibration 1:1, and that the
measurement is antenna phase centre to antenna phase centre.

- [ ] **Step 3: Correct the `RX_ANT_DLY` claim in `CLAUDE.md`**

In the "URGENT next work" section, replace the bullet beginning *"Only **TX**
needs trimming: `RX_ANT_DLY` cancels in `RTD_resp`..."* with this, keeping the
surrounding bullets untouched:

```markdown
- **Only the sum `ant_tx + ant_rx` is observable — trim TX and pin RX.**
  `RTD_resp` as a *number* is `D + ant_delay_tx` and independent of
  `RX_ANT_DLY`, but the delayed TX time is derived *from* `poll_rx_ts`
  (`anchor_respond.c:138`), which the hardware has already shifted by
  `RX_ANT_DLY`. Raising `ant_delay_rx` by a tick therefore makes the anchor
  physically transmit a tick earlier, moving the initiator's result by exactly
  as much as raising `ant_delay_tx` does by growing the reported turnaround.
  Both are half a tick of range per unit. So putting the whole correction into
  TX is *equivalent*, not merely conventional — and `ant_delay_rx` is **not** a
  free parameter: change it after calibration and you have decalibrated the
  board. An earlier version of this bullet said RX "drops out of the SS-TWR
  result", which is the right conclusion by the wrong route. Full derivation in
  `docs/superpowers/specs/2026-08-13-antenna-delay-calibration-design.md` §3.1.
```

- [ ] **Step 4: Record the new build, commands and test in `CLAUDE.md`**

- Under "Build & flash": the `-DEXTRA_CONF_FILE=cal.conf` invocation and what
  the cal image is for.
- Under "Console": the `cal ref` / `cal peer` commands, noting they exist only
  in the calibration image.
- Under "Layout": `src/cal_math.{c,h}` (copied verbatim from the tag, same rule
  as `uwb_frame_802_15_4z.c`), `src/cal_solve.{c,h}`, `src/cal_initiator.{c,h}`,
  `src/cal_run.{c,h}`, `src/cal_shell.c`, `Kconfig`, `cal.conf`.
- Under "Host tests": the `tests/cal_solve` gcc line from Task 1 Step 6.
- Under "Hard-won facts": that the DWM3001CDK's `POLL_RX_TO_RESP_TX_DLY_UUS`
  must be 2000 and not the example's 450, because at PLEN_1024 the preamble
  alone is ~1.05 ms.

Rewrite "URGENT next work" item 1 to record that calibration is done: the
procedure, the per-board `ant_delay_tx` values actually measured, the
cross-check residuals, and the `0xEA` `residual` figures from Step 1. Leave item
2 (auto-positioning) exactly as it is — it is still urgent and still unstarted.

- [ ] **Step 5: Commit**

```bash
git add docs/antenna-delay-calibration.md CLAUDE.md
git commit -m "docs: antenna-delay calibration procedure, and correct the RX_ANT_DLY claim"
```

---

## Notes for the executor

- **Do not tune the constants to make a bench test pass.** `CAL_TX_DLY_MIN/MAX`,
  the `valid < CAL_MAX_SAMPLES/4` floor and the 30 mm acceptance threshold are
  diagnostics. If one trips, the setup or the reference node is wrong; changing
  the number hides the fault rather than fixing it.
- **The responder is not yours to edit.** If a cal exchange fails, the fault is
  in `cal_initiator.c` or the setup. `anchor_respond.c` is bench-confirmed
  against a real tag and changing it invalidates the whole premise that this
  procedure calibrates the production path.
- Task 3 Step 6 and Task 4 Steps 3–4 need hardware and cannot be faked. If you
  cannot run them, stop and report which gates are unverified rather than
  marking the task done.

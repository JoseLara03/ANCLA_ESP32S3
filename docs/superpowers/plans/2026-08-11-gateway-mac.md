# GATEWAY MAC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise the SPI clock to the DW3000's usable maximum, then make GATEWAY mode emit the TDMA beacon and grant CAP seats, and make slaves suppress responses that would collide with that beacon.

**Architecture:** The SPI fix comes first because `T_slot` is a direct function of the response turnaround, and at today's value the contract's 12-slot superframe does not fit 200 ms. The gateway is MAC-only — it does not answer ranging polls — and drives its beacon cadence from the DW3000 system clock, interrupt-driven with the same callback shape as `uwb_slave.c`. Slaves gain a pure-C `beacon_guard` that refuses any delayed transmit landing inside the beacon window.

**Tech Stack:** Zephyr 4.4.x on ESP32-S3, `west`, Qorvo `dwt_uwb_driver` 08.02.02 via the vendored br101 module, plain-gcc host tests.

**Spec:** [`docs/superpowers/specs/2026-08-11-gateway-mac-design.md`](../specs/2026-08-11-gateway-mac-design.md)

## Global Constraints

- **Do not modify `modules/`.** `dw3000_spi.h` is already on the include path `uwb_radio.c` uses for `dw3000_hw.h`; no module change is needed for the SPI fix.
- **Do not modify `src/uwb_frame_802_15_4z.{c,h}` or `tests/uwb_frame/test_uwb_frame.c`.** They are byte-identical to `tag_testting/src/` and must stay that way. The three known defects in that file stay unfixed; Task 6 documents at the call sites (`tx_beacon()`, `send_grant()`) why they cannot fire here.
- **Do not edit the tag project** at `../../../tag_testting`. Read-only context.
- **`dw3000_spi_speed_fast()` goes after `dwt_initialise()` and `dwt_checkidlerc()`, before `dwt_configure()`.** The DW3000 must be clocked at ≤7 MHz until it leaves INIT_RC (`deca_device_api.h:2381`, `:2601`).
- **DTS rate is `26670000`, not `26000000`.** The HAL picks the highest rate not exceeding the request, and 80/3 = 26.666… MHz exceeds 26 — asking for 26 silently yields 20.
- **All times crossing into the radio are DW3000 hi32 units** (256 DTU ≈ 4.006 ns) and **wrap every ~17.2 s**. Every comparison is `(int32_t)(a - b)` signed-difference arithmetic. A naive unsigned compare is wrong.
- **Never poll `DWT_INT_CIADONE_BIT_MASK` after an RX event**, and **keep `TXFRS` out of the enabled interrupt mask** — `dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT)` only. Both are in `CLAUDE.md`'s hard-won facts.
- **Every length handed to the frame module is `flen - FCS_LEN`.**
- **Snapshot config by value at mode entry**, never hold the `uwb_config_get()` pointer. `uwb_slave.c:164` is the pattern.
- **Build:** `$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"` must be set.
- Source style: tabs, `LOG_*` for output, Apache-2.0 header comment as in existing `src/` files.

---

### Task 1: Switch the SPI bus to fast rate

**Files:**
- Modify: `src/uwb_radio.c:57-76`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts:91`

**Interfaces:**
- Consumes: `dw3000_spi_speed_fast()` from `modules/dw3000-decadriver/platform/dw3000_spi.h`.
- Produces: nothing in code. Produces the *measurement* Hardware Gate A depends on.

Background: `dw3000_spi_init()` sets `spi_cfg = &spi_cfgs[0]` (2 MHz) and nothing ever switches, so the radio has been running at 2 MHz while the boot log printed `DW3000 SPI (max 8MHz)` — that log line reports `spi_cfgs[1].frequency`, the config that is never selected. `dwt_initialise()` dispatches to `ull_initialise` (`dw3000_device.c:9371`), not the `init()` wrapper that would have called `setfastrate`. See spec §"The SPI clock".

- [ ] **Step 1: Raise the DTS rate**

In `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`, replace the `spi-max-frequency` line inside the `dw3000@0` node:

```dts
		/* 80 MHz APB / 3. The ESP32-S3 only produces 80/N, so the ladder
		 * is 20 / 26.67 / 40 with nothing between; 40 overruns the
		 * DW3000's ~38 MHz ceiling, making 26.67 the maximum usable rate
		 * on this hardware. Ask for 26670000, not 26000000: the HAL picks
		 * the highest rate NOT EXCEEDING the request, and 80/3 =
		 * 26.666... MHz exceeds 26, so asking for 26 silently gives 20.
		 * GPIO 11/12/13 are the SPI2 IO_MUX pins (FSPID/FSPICLK/FSPIQ),
		 * so there is no GPIO-matrix penalty at this rate. */
		spi-max-frequency = <26670000>;
```

- [ ] **Step 2: Call the fast-rate switch at the right point**

In `src/uwb_radio.c`, add the include alongside the existing `<dw3000_hw.h>`:

```c
#include <dw3000_spi.h>
```

Then insert the switch between the `dwt_checkidlerc()` block and the `LOG_INF("DW3220 device ID: ...")` line:

```c
	/* Up to fast rate now that the part is out of INIT_RC. Until this
	 * point the DW3000 requires <= 7 MHz (deca_device_api.h:2381, :2601),
	 * which is why dw3000_spi_init() starts on spi_cfgs[0] at 2 MHz.
	 *
	 * Nothing in the driver does this for us on this build: .setfastrate
	 * is wired into the vtable (platform/deca_port.c:37) but its only
	 * call sites are inside a static init() wrapper, and dwt_initialise()
	 * dispatches to ull_initialise instead (dw3000_device.c:9371). Without
	 * this call the whole session runs at 2 MHz -- which is what it did
	 * until now, while the module's own boot log printed "max 8MHz" from
	 * the never-selected spi_cfgs[1]. */
	dw3000_spi_speed_fast();
```

- [ ] **Step 3: Log the rate that is actually in effect**

Immediately after the `dw3000_spi_speed_fast()` call, still before the DEV_ID line:

```c
	LOG_INF("SPI at fast rate (%u Hz requested)",
		(unsigned)DT_PROP(DT_INST(0, decawave_dw3000), spi_max_frequency));
```

Add `#include <zephyr/devicetree.h>` if the build reports `DT_INST` or `DT_PROP` undeclared.

This prints the *requested* value. Confirming the rate the HAL actually selected is a Hardware Gate A step — the boot log has already misreported this once.

- [ ] **Step 4: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean, no warnings.

- [ ] **Step 5: Re-run every host test**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_config\test_uwb_config.exe tests\uwb_config\test_uwb_config.c src\uwb_config.c
.\tests\uwb_config\test_uwb_config.exe
gcc -Wall -Wextra -o tests\uwb_frame\test_uwb_frame.exe tests\uwb_frame\test_uwb_frame.c src\uwb_frame_802_15_4z.c
.\tests\uwb_frame\test_uwb_frame.exe
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
.\tests\disc_schedule\test_disc_schedule.exe
```

Expected: `PASSED`, `ALL TESTS PASSED`, `PASSED`. None of them touch SPI; this is a regression check, not evidence the fix works.

- [ ] **Step 6: Commit**

```bash
git add src/uwb_radio.c boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts
git commit -m "fix(radio): switch the SPI bus to fast rate after init

The radio has been running at 2 MHz, not the 8 MHz the boot log claimed.
dw3000_spi_init() selects spi_cfgs[0] and nothing ever calls
dw3000_spi_speed_fast(): .setfastrate is in the vtable, but its only
call sites sit in a static init() wrapper that dwt_initialise() does not
dispatch to -- it goes to ull_initialise. The log line printed
spi_cfgs[1].frequency, the config that is never selected.

read_cir()'s 216-byte transfer measured ~944 us against 864 us of pure
clock time at 2 MHz, so the RX-side overhead was never the ioctl
dispatch or the bit-banged CS.

DTS asks for 26670000: 80/3 = 26.67 MHz is the highest rung below the
DW3000's ~38 MHz ceiling, and asking for exactly 26000000 would drop
silently to 20."
```

---

## Hardware Gate A — measure, choose the rate, re-tune

**This is a human step. It needs a board, and Task 2 cannot start without its numbers.**
Work stops here if the gate is not met.

1. Flash and monitor:

   ```powershell
   west flash
   west espressif monitor -p COM5
   ```

2. **Confirm the bus is sound at speed.** `DEV_ID` must still read `0xDECA0312`, and `dwt_configure()` must succeed. If either fails, drop the DTS to `20000000` (80/4) and retry before concluding anything about the part.

3. **Read the profiling output.** The `prof us: cir=... readdata=... readts=...` line is already in `uwb_slave.c:233`. Record all three numbers with a tag ranging. At 2 MHz `cir` was ~944 µs; at 26.67 MHz expect roughly 1/13 of the clock-bound portion.

4. **Find the new turnaround floor.** Lower `DISC_BASE_UUS` (`src/disc_schedule.h`) and `POLL_RX_TO_RESP_TX_DLY_UUS` (`src/anchor_respond.c`) together and confirm ranging still works against the sniffer at each step. The old values are both `6000`.

5. **Apply the gate:**

   | Result | Action |
   |---|---|
   | Both constants hold under **2500 UUS** | Gate met. Proceed to Task 2 with the measured values. |
   | 2500 UUS not reachable | **Stop.** `N_CFP = 12` cannot be honoured — see the spec's budget table. Renegotiate `N_CFP` with the tag side before any slot map is designed. |

6. Record the chosen SPI rate and the three measured constants. Task 2 consumes them.

---

### Task 2: Apply the measured turnaround constants and remove the profiler

**Files:**
- Modify: `src/disc_schedule.h` (`DISC_BASE_UUS`)
- Modify: `src/anchor_respond.c` (`POLL_RX_TO_RESP_TX_DLY_UUS`, `TX_COMPLETE_TIMEOUT_MS`)
- Modify: `src/uwb_slave.c:199-206, 230-236` (delete the profiling block)

**Interfaces:**
- Consumes: the measured values from Hardware Gate A.
- Produces: `DISC_BASE_UUS` and `DISC_SLOT_UUS` at their tuned values — Task 4 uses them to reason about worst-case TX scheduling, and Task 6's suppression is sized against `disc_resp_delay_uus(3)`.

- [ ] **Step 1: Set the two turnaround constants to the measured values**

Replace the `DISC_BASE_UUS` define in `src/disc_schedule.h` and the `POLL_RX_TO_RESP_TX_DLY_UUS` define in `src/anchor_respond.c` with the value confirmed in Gate A step 4. Keep the existing explanatory comments and **append** the new bench result rather than deleting the history — the record of what failed at 2000 and 2400 is why nobody re-tries those.

Add to both comments:

```
 *   - <measured> confirmed working after the SPI bus was switched to fast
 *     rate (26.67 MHz). The earlier 2000/2400 failures were the 2 MHz bus,
 *     not the driver's per-call overhead: read_cir()'s 216-byte transfer
 *     alone was 864 uus of pure clock time at 2 MHz.
```

- [ ] **Step 2: Re-derive `TX_COMPLETE_TIMEOUT_MS`**

This bound must exceed the **worst-case scheduled delay**, which is `disc_resp_delay_uus(3)` = `DISC_BASE_UUS + 3 × DISC_SLOT_UUS`, plus frame airtime and margin. It is not a frame-airtime figure — `dwt_starttx()` returns as soon as the delayed TX is armed, and the transmission happens only when the scheduled delay elapses.

Worked example at `DISC_BASE_UUS = 2400`:

```
disc_resp_delay_uus(3) = 2400 + 3*3500 = 12900 uus = 13.2 ms
+ ~1.3 ms airtime + margin  ->  TX_COMPLETE_TIMEOUT_MS = 20
```

Set it to `ceil(disc_resp_delay_uus(3) × 1.0256 / 1000) + 5`, rounded up to a whole millisecond. Update the constant's comment with the new arithmetic. **Do not leave it larger than necessary and do not shrink it below the worked rule** — a bound shorter than the scheduled delay force-cancels every response for anchor ids ≥ 2, which is exactly the bug the current comment records.

- [ ] **Step 3: Delete the profiling block**

In `src/uwb_slave.c`, remove the `TEMP PROFILING` comment and `uint32_t t0 = k_cycle_get_32();` at lines 199-206, the `t1`/`t2`/`t3` assignments interleaved with the three SPI reads, and the whole trailing block from `/* TEMP PROFILING: see the checkpoint comment above. */` through the `LOG_INF("prof us: ...")` call. The three reads themselves stay:

```c
		int32_t cir_power = 0;
		uint16_t cir_quality = 0;

		read_cir(status, &cir_power, &cir_quality);
		dwt_readrxdata(rx_buf, flen, 0);
		uint64_t rx_ts = uwb_get_rx_timestamp_u64();
```

- [ ] **Step 4: Verify it builds and host tests pass**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
.\tests\disc_schedule\test_disc_schedule.exe
```

Expected: clean build with no unused-variable warnings (which would mean a profiling leftover), and `PASSED`.

The `disc_schedule` test asserts `disc_resp_delay_uus(0) == DISC_BASE_UUS` symbolically but also `== 2000u` literally. **Update that literal to the new value**, and the three stagger literals with it — the test is checking the arithmetic, not the constant's history.

- [ ] **Step 5: Commit**

```bash
git add src/disc_schedule.h src/anchor_respond.c src/uwb_slave.c \
        tests/disc_schedule/test_disc_schedule.c
git commit -m "perf(respond): tune the turnaround budgets for the fast SPI bus

Measured on hardware after the bus went from 2 MHz to 26.67 MHz.
TX_COMPLETE_TIMEOUT_MS re-derived from disc_resp_delay_uus(3), which is
what it has to cover -- dwt_starttx() returns when the delayed TX is
armed, not when it fires.

Removes the temporary cycle-counter profiling block; it did its job."
```

---

### Task 3: TX timestamp, UUS→hi32 conversion, and the shared MAC constants

**Files:**
- Modify: `src/uwb_dwtime.h`
- Modify: `src/uwb_dwtime.c`
- Create: `src/uwb_mac.h`

**Interfaces:**
- Consumes: `dwt_readtxtimestamp(uint8_t *timestamp)` (`deca_device_api.h:1915` — note it takes **one** argument, unlike `dwt_readrxtimestamp`, which takes two in this driver version).
- Produces: `uint64_t uwb_get_tx_timestamp_u64(void)`, `UUS_TO_HI32(uus)`, and `T_SUPERFRAME_UUS` / `BEACON_OCCUPANCY_UUS` / `BEACON_GUARD_UUS` in `uwb_mac.h`. Tasks 6 and 7 use all of them.

- [ ] **Step 1: Add the declaration and the macro**

In `src/uwb_dwtime.h`, after the `UUS_TO_DWT_TIME` define:

```c
/* Convert a UUS span to DW3000 hi32 system-time units (256 DTU ~= 4.006 ns),
 * the unit dwt_readsystimestamphi32() and dwt_setdelayedtrxtime() work in.
 *
 * hi32 is 32 bits and wraps every ~17.2 s (2^32 * 4.006 ns), so every
 * comparison between two hi32 values must be signed-difference arithmetic --
 * (int32_t)(a - b) -- which is correct for intervals well under ~8.6 s. A
 * plain unsigned compare is wrong across the wrap. */
#define UUS_TO_HI32(uus) ((uint32_t)(((uint64_t)(uus) * UUS_TO_DWT_TIME) >> 8))
```

and after `uwb_get_rx_timestamp_u64()`:

```c
/* The 40-bit TX timestamp of the last transmitted frame, as a 64-bit value.
 * Valid until the next transmission. The gateway uses it to anchor the next
 * superframe on the beacon it actually sent, rather than on when it meant to. */
uint64_t uwb_get_tx_timestamp_u64(void);
```

- [ ] **Step 2: Implement it**

In `src/uwb_dwtime.c`, after `uwb_get_rx_timestamp_u64()`:

```c
uint64_t uwb_get_tx_timestamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;

	dwt_readtxtimestamp(ts_tab);
	for (int8_t i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}
```

- [ ] **Step 3: Create the shared MAC constants header**

The gateway schedules the beacon and the slave predicts it. Both need the same
superframe period, and a copy in each file is a pair that drifts silently — the
slave would keep suppressing against a period the gateway no longer uses, and the
symptom would be intermittent beacon corruption that looks like an RF problem.
One definition, included by both.

`src/uwb_mac.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Superframe constants from the MAC contract
 * (tag_testting/spec/2026-06-17-uwb-mac-protocol-contract.md, section 2.1).
 *
 * Fixed by protocol version and identical on every node, which is why they live
 * in one header rather than beside the code that uses them: the gateway
 * schedules the beacon from these and the slaves predict it from the same
 * values. Two copies would drift, and a slave predicting against a stale period
 * fails as intermittent beacon corruption -- which looks like an RF fault, not a
 * constant.
 *
 * The PHY contract lives separately in uwb_phy.h; this is the MAC layer.
 */

#ifndef UWB_MAC_H
#define UWB_MAC_H

/* T_superframe = 200 ms. 1 UUS = 512/499.2 MHz = 1.0256 us, so 195000 UUS is
 * 200.0 ms. */
#define T_SUPERFRAME_UUS 195000u

/* T_beacon ~1.5 ms and T_guard 0.5 ms, rounded up in UUS. The guard errs wide
 * deliberately: suppressing one extra ranging response costs one range, while
 * corrupting one beacon costs every node in the network. */
#define BEACON_OCCUPANCY_UUS 1500u
#define BEACON_GUARD_UUS      500u

#endif /* UWB_MAC_H */
```

- [ ] **Step 4: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: clean build. If the compiler complains about `dwt_readtxtimestamp`'s arity, check `deca_device_api.h:1915` — this driver version takes one argument, and copying the two-argument shape from `uwb_get_rx_timestamp_u64()` above it is the easy mistake.

`uwb_mac.h` is header-only, so it needs no `CMakeLists.txt` entry and nothing includes it yet.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_dwtime.h src/uwb_dwtime.c src/uwb_mac.h
git commit -m "feat(dwtime): TX timestamp, UUS->hi32, and shared MAC constants

The gateway anchors each superframe on the beacon's actual TX timestamp
rather than its intended one, so a late beacon does not compound into
the next period.

UUS_TO_HI32 carries the wrap warning: hi32 wraps every ~17.2 s, so every
comparison has to be (int32_t)(a - b).

uwb_mac.h holds T_superframe and the beacon window in one place. The
gateway schedules the beacon from them and the slaves predict it from
the same values; two copies would drift, and a slave predicting against
a stale period fails as intermittent beacon corruption -- which reads as
an RF fault rather than a constant."
```

---

### Task 4: `beacon_guard` — beacon-collision TX suppression

**Files:**
- Create: `src/beacon_guard.h`
- Create: `src/beacon_guard.c`
- Create: `tests/beacon_guard/test_beacon_guard.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `UUS_TO_HI32` (Task 3) — used by callers, not by this module, which is unit-agnostic.
- Produces: `struct beacon_guard`, `beacon_guard_init(struct beacon_guard *g, uint32_t period, uint32_t guard, uint32_t occupancy)`, `beacon_guard_beacon(struct beacon_guard *g, uint32_t beacon_at)`, `bool beacon_guard_tx_allowed(struct beacon_guard *g, uint32_t tx_at)`, `bool beacon_guard_locked(const struct beacon_guard *g)`, `BEACON_GUARD_MAX_MISSES`. Tasks 5 and 6 use them.

Pure C, no Zephyr and no radio headers, so it is host-testable. All times are opaque 32-bit tick counts; the caller decides they are DW3000 hi32 units.

- [ ] **Step 1: Write the failing test**

`tests/beacon_guard/test_beacon_guard.c`:

```c
#include "beacon_guard.h"
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Round numbers, not real hi32 values: the module is unit-agnostic. */
#define PERIOD     100000u
#define GUARD        1000u
#define OCCUPANCY    3000u

static void setup(struct beacon_guard *g, uint32_t first_beacon)
{
    beacon_guard_init(g, PERIOD, GUARD, OCCUPANCY);
    beacon_guard_beacon(g, first_beacon);
}

static void test_unlocked_allows_everything(void)
{
    struct beacon_guard g;
    beacon_guard_init(&g, PERIOD, GUARD, OCCUPANCY);

    /* No beacon seen yet: never suppress. Suppressing without a reference
     * would block transmits at arbitrary times. */
    CHECK(!beacon_guard_locked(&g));
    CHECK(beacon_guard_tx_allowed(&g, 12345u));
    CHECK(beacon_guard_tx_allowed(&g, 999999u));
}

static void test_window_edges(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);
    CHECK(beacon_guard_locked(&g));

    /* Next beacon predicted at 1000 + PERIOD = 101000.
     * Forbidden window is [101000 - GUARD, 101000 + OCCUPANCY + GUARD]
     *                   = [100000, 105000]. */
    CHECK(beacon_guard_tx_allowed(&g,  99999u));   /* just before */
    CHECK(!beacon_guard_tx_allowed(&g, 100000u));  /* leading edge */
    CHECK(!beacon_guard_tx_allowed(&g, 101000u));  /* beacon start */
    CHECK(!beacon_guard_tx_allowed(&g, 105000u));  /* trailing edge */
    CHECK(beacon_guard_tx_allowed(&g, 105001u));   /* just after */
}

static void test_rolls_forward_across_superframes(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* A TX three superframes out must be checked against the beacon in THAT
     * superframe, not the next one. Window 3 ahead: [300000, 305000]. */
    CHECK(beacon_guard_tx_allowed(&g, 299999u));
    CHECK(!beacon_guard_tx_allowed(&g, 301000u));
}

static void test_lock_drops_after_max_misses(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* Rolling past BEACON_GUARD_MAX_MISSES predicted beacons without seeing
     * one means the prediction is stale; stop suppressing rather than block
     * transmits against a guess. */
    uint32_t far = 1000u + PERIOD * (BEACON_GUARD_MAX_MISSES + 2u);
    CHECK(beacon_guard_tx_allowed(&g, far));
    CHECK(!beacon_guard_locked(&g));

    /* A fresh beacon re-acquires. */
    beacon_guard_beacon(&g, far + 500u);
    CHECK(beacon_guard_locked(&g));
}

static void test_beacon_resets_misses(void)
{
    struct beacon_guard g;
    setup(&g, 1000u);

    /* Two missed superframes, then a beacon: still locked afterwards. */
    (void)beacon_guard_tx_allowed(&g, 1000u + PERIOD * 2u + OCCUPANCY + GUARD + 1u);
    CHECK(beacon_guard_locked(&g));
    beacon_guard_beacon(&g, 1000u + PERIOD * 3u);
    CHECK(beacon_guard_locked(&g));

    /* And the reference moved: window is now [.. + PERIOD .. ]. */
    CHECK(!beacon_guard_tx_allowed(&g, 1000u + PERIOD * 4u));
}

static void test_wrap_forward(void)
{
    struct beacon_guard g;
    /* Beacon shortly before the 32-bit wrap; the next one is past it. */
    setup(&g, 0xFFFFFF00u);

    /* 0xFFFFFF00 + 100000 wraps to 0x000185A0 (unsigned arithmetic is fine;
     * the COMPARISONS are what must be signed). Window [0x000181B8,
     * 0x00019388]. */
    uint32_t predicted = 0xFFFFFF00u + PERIOD;

    CHECK(!beacon_guard_tx_allowed(&g, predicted));
    CHECK(!beacon_guard_tx_allowed(&g, predicted - GUARD));
    CHECK(beacon_guard_tx_allowed(&g, predicted - GUARD - 1u));
    CHECK(beacon_guard_tx_allowed(&g, predicted + OCCUPANCY + GUARD + 1u));
}

static void test_wrap_tx_before_beacon_after(void)
{
    struct beacon_guard g;
    /* The case a naive unsigned compare gets wrong: the TX time is a huge
     * number just below the wrap, the predicted beacon a small one just
     * above it. Unsigned, tx > beacon and the roll-forward loop runs away;
     * signed, tx - beacon is a small negative and the TX is simply early. */
    setup(&g, 0xFFFF0000u - PERIOD);

    uint32_t predicted = 0xFFFF0000u;          /* next beacon, pre-wrap */
    CHECK(!beacon_guard_tx_allowed(&g, predicted + 100u));
    CHECK(beacon_guard_tx_allowed(&g, predicted - GUARD - 1u));
}

int main(void)
{
    test_unlocked_allows_everything();
    test_window_edges();
    test_rolls_forward_across_superframes();
    test_lock_drops_after_max_misses();
    test_beacon_resets_misses();
    test_wrap_forward();
    test_wrap_tx_before_beacon_after();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\beacon_guard\test_beacon_guard.exe `
    tests\beacon_guard\test_beacon_guard.c src\beacon_guard.c
```

Expected: FAIL — `src\beacon_guard.c` and `beacon_guard.h` do not exist.

- [ ] **Step 3: Create the header**

`src/beacon_guard.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Beacon-collision avoidance for the ranging responders.
 *
 * An anchor schedules its response at a fixed delay from the poll that
 * triggered it and, without this, never checks where that delay lands. Two
 * cases put a response on top of the gateway's beacon:
 *
 *   - DISCOVERY, the dominant one. It is broadcast by tags that have not
 *     joined yet, so it is not confined to a CFP slot and can arrive anywhere
 *     in the superframe -- and disc_resp_delay_uus(3) puts the response many
 *     milliseconds later with no relation to the frame structure at all.
 *   - The tail of the last CFP slot, where the superframe's ~5.5 ms of slack
 *     is shorter than the response turnaround.
 *
 * A hit corrupts a broadcast every node in the network depends on, which costs
 * far more than the single range that caused it. The gateway protects itself
 * with its beacon arm margin; this is the slaves' mirror of that.
 *
 * Pure C -- no Zephyr, no radio headers -- so it is host-testable. Times are
 * opaque 32-bit ticks; callers supply DW3000 hi32 units (256 DTU ~= 4.006 ns)
 * via UUS_TO_HI32 in uwb_dwtime.h.
 */

#ifndef BEACON_GUARD_H
#define BEACON_GUARD_H

#include <stdbool.h>
#include <stdint.h>

/* Consecutive predicted beacons the guard may roll past without seeing one
 * before it declares the prediction stale.
 *
 * 4 superframes is ~800 ms, over which two independent +/-20 ppm crystals
 * drift ~32 us relative -- still far inside a 500 us guard, so a prediction
 * this old is still trustworthy. Beyond it, suppressing against a guess would
 * block legitimate transmits at arbitrary times, which is worse than the
 * collisions suppression exists to prevent. */
#define BEACON_GUARD_MAX_MISSES 4

struct beacon_guard {
	uint32_t next_beacon; /* predicted start of the next beacon */
	uint32_t period;      /* superframe period */
	uint32_t guard;       /* pad on each side of the beacon */
	uint32_t occupancy;   /* beacon airtime */
	uint8_t  misses;      /* predicted beacons rolled past unseen */
	bool     locked;      /* false until the first beacon, and after a
			       * stale-prediction drop */
};

/* Configure and clear to the unlocked state. Suppression does nothing until
 * the first beacon_guard_beacon(). */
void beacon_guard_init(struct beacon_guard *g, uint32_t period, uint32_t guard,
		       uint32_t occupancy);

/* Record a received beacon at beacon_at, re-anchoring the prediction and
 * clearing the miss count. Re-anchoring every beacon is why no drift model is
 * needed: error accumulates for one superframe, not indefinitely. */
void beacon_guard_beacon(struct beacon_guard *g, uint32_t beacon_at);

/* True if a transmission at tx_at is clear of the beacon window.
 *
 * Mutating: rolls the prediction forward past tx_at, counting each predicted
 * beacon it passes as a miss, and drops the lock past BEACON_GUARD_MAX_MISSES.
 * Always true while unlocked. */
bool beacon_guard_tx_allowed(struct beacon_guard *g, uint32_t tx_at);

/* Whether the prediction is currently trusted. Diagnostics and tests. */
bool beacon_guard_locked(const struct beacon_guard *g);

#endif /* BEACON_GUARD_H */
```

- [ ] **Step 4: Create the implementation**

`src/beacon_guard.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "beacon_guard.h"

/* Signed-difference comparison. hi32 wraps every ~17.2 s, so "is a after b"
 * is only meaningful as the sign of the difference, never as a > b. Correct
 * for any interval under ~8.6 s; every span here is one superframe (200 ms). */
static inline bool after_or_eq(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) >= 0;
}

void beacon_guard_init(struct beacon_guard *g, uint32_t period, uint32_t guard,
		       uint32_t occupancy)
{
	g->next_beacon = 0;
	g->period = period;
	g->guard = guard;
	g->occupancy = occupancy;
	g->misses = 0;
	g->locked = false;
}

void beacon_guard_beacon(struct beacon_guard *g, uint32_t beacon_at)
{
	g->next_beacon = beacon_at + g->period;
	g->misses = 0;
	g->locked = true;
}

bool beacon_guard_tx_allowed(struct beacon_guard *g, uint32_t tx_at)
{
	if (!g->locked) {
		return true;
	}

	/* Advance to the superframe tx_at actually falls in. Each beacon we
	 * step over is one we predicted but never received. */
	while (after_or_eq(tx_at, g->next_beacon + g->occupancy + g->guard)) {
		if (g->misses >= BEACON_GUARD_MAX_MISSES) {
			g->locked = false;
			return true;
		}
		g->misses++;
		g->next_beacon += g->period;
	}

	/* Forbidden: [next_beacon - guard, next_beacon + occupancy + guard]. */
	if (after_or_eq(tx_at, g->next_beacon - g->guard)) {
		return false;
	}
	return true;
}

bool beacon_guard_locked(const struct beacon_guard *g)
{
	return g->locked;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\beacon_guard\test_beacon_guard.exe `
    tests\beacon_guard\test_beacon_guard.c src\beacon_guard.c
.\tests\beacon_guard\test_beacon_guard.exe
```

Expected: `PASSED`, exit code 0, no warnings.

- [ ] **Step 6: Add to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, alphabetically after `src/anchor_shell.c`:

```cmake
	src/beacon_guard.c
```

- [ ] **Step 7: Verify the firmware builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: clean build. Nothing calls it yet.

- [ ] **Step 8: Commit**

```bash
git add src/beacon_guard.h src/beacon_guard.c \
        tests/beacon_guard/test_beacon_guard.c CMakeLists.txt
git commit -m "feat(guard): beacon-collision TX suppression for the responders

A staggered DISCOVERY response can land on the gateway's beacon --
DISCOVERY comes from unjoined tags, so it is not confined to a CFP slot
and the response has no relation to the superframe at all. Corrupting a
beacon costs the whole network, not one range.

Pure C and host-tested, including both directions across the ~17.2 s
hi32 wrap: the case a naive unsigned compare gets wrong is a TX time
just below the wrap against a predicted beacon just above it.

Suppression is off until a beacon is seen and switches off again after
BEACON_GUARD_MAX_MISSES stale superframes -- blocking transmits against
a guess is worse than the collisions it would prevent."
```

---

### Task 5: `gw_core` — seat management

**Files:**
- Create: `src/gw_core.h`
- Create: `src/gw_core.c`
- Create: `tests/gw_core/test_gw_core.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `UWB_FRAME_N_CFP` (12), `UWB_FRAME_EUI_LEN` (8), `UWB_FRAME_ADDR_BCAST` (0xFFFF) from `uwb_frame_802_15_4z.h`.
- Produces: `struct gw_seat`, `struct gw_core_ctx`, `struct gw_grant`, `gw_core_init()`, `gw_core_join()`, `gw_core_keepalive()`, `gw_core_release()`, `gw_core_superframe_tick()`, `gw_core_build_slotmap()`, `GW_N_CFP`, `GW_LEASE_SF` (50), `GW_TAG_ADDR_BASE` (0x0100). Task 6 uses all of them.

Ported from `fw-cre/firmeware_creator/Src/gw_core.{c,h}` with no logic changes — it is already pure C with no radio dependency.

- [ ] **Step 1: Write the failing test**

`tests/gw_core/test_gw_core.c`:

```c
#include "gw_core.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void mk_eui(uint8_t out[UWB_FRAME_EUI_LEN], uint8_t tag)
{
    for (int i = 0; i < UWB_FRAME_EUI_LEN; i++) out[i] = (uint8_t)(tag + i);
}

static void test_init(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    CHECK(c.frame_counter == 0);
    CHECK(c.next_short_addr == GW_TAG_ADDR_BASE);
    for (int i = 0; i < GW_N_CFP; i++) CHECK(c.seats[i].short_addr == 0);
}

static void test_join_fills_slots_in_order(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (int i = 0; i < 3; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)(0x10 + i));
        CHECK(gw_core_join(&c, eui, 1, &g));
        CHECK(g.slot_index == i);
        CHECK(g.short_addr == (uint16_t)(GW_TAG_ADDR_BASE + i));
        CHECK(g.lease == GW_LEASE_SF);
        CHECK(g.tier == 1);
    }
}

static void test_rejoin_is_idempotent(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant g1, g2, g3;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);

    CHECK(gw_core_join(&c, eui_a, 1, &g1));
    CHECK(gw_core_join(&c, eui_b, 1, &g2));

    /* A repeat JOIN from a known EUI -- a tag that missed its GRANT and
     * retried -- must return the SAME seat, not consume a second one. */
    CHECK(gw_core_join(&c, eui_a, 2, &g3));
    CHECK(g3.slot_index == g1.slot_index);
    CHECK(g3.short_addr == g1.short_addr);
    CHECK(g3.tier == 2);          /* but the tier is updated */
}

static void test_network_full(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    for (int i = 0; i < GW_N_CFP; i++) {
        uint8_t eui[UWB_FRAME_EUI_LEN];
        struct gw_grant g;
        mk_eui(eui, (uint8_t)i);
        CHECK(gw_core_join(&c, eui, 1, &g));
    }

    uint8_t extra[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(extra, 0xF0);
    CHECK(!gw_core_join(&c, extra, 1, &g));    /* 13th is refused */
}

static void test_keepalive(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (int i = 0; i < 10; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].lease_remaining == GW_LEASE_SF - 10);

    gw_core_keepalive(&c, g.short_addr, 3);
    CHECK(c.seats[g.slot_index].lease_remaining == GW_LEASE_SF);
    CHECK(c.seats[g.slot_index].tier == 3);

    /* An unknown address must not disturb anything. */
    gw_core_keepalive(&c, 0xBEEF, 1);
    CHECK(c.seats[g.slot_index].tier == 3);
}

static void test_lease_expiry(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));

    for (int i = 0; i < GW_LEASE_SF - 1; i++) gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].short_addr != 0);    /* still held */

    gw_core_superframe_tick(&c);
    CHECK(c.seats[g.slot_index].short_addr == 0);    /* reclaimed */
    CHECK(c.frame_counter == GW_LEASE_SF);
}

static void test_release(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));
    gw_core_release(&c, g.short_addr);
    CHECK(c.seats[g.slot_index].short_addr == 0);

    /* Releasing an unknown address is a no-op, not a crash. */
    gw_core_release(&c, 0xBEEF);
}

static void test_slotmap(void)
{
    struct gw_core_ctx c;
    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    uint16_t map[GW_N_CFP];

    gw_core_init(&c);
    mk_eui(eui, 0x10);
    CHECK(gw_core_join(&c, eui, 1, &g));
    gw_core_build_slotmap(&c, map);

    CHECK(map[g.slot_index] == g.short_addr);
    for (int i = 0; i < GW_N_CFP; i++) {
        if (i != g.slot_index) CHECK(map[i] == UWB_FRAME_ADDR_BCAST);
    }
}

static void test_addr_pool_skips_live_seats(void)
{
    struct gw_core_ctx c;
    uint8_t eui_a[UWB_FRAME_EUI_LEN], eui_b[UWB_FRAME_EUI_LEN];
    struct gw_grant ga, gb, gc;

    gw_core_init(&c);
    mk_eui(eui_a, 0x10);
    mk_eui(eui_b, 0x20);
    CHECK(gw_core_join(&c, eui_a, 1, &ga));
    CHECK(gw_core_join(&c, eui_b, 1, &gb));

    /* Free the FIRST seat, then join a third tag: the pool has moved on, so
     * the new tag must not be handed an address a live seat still holds. */
    gw_core_release(&c, ga.short_addr);
    uint8_t eui_c[UWB_FRAME_EUI_LEN];
    mk_eui(eui_c, 0x30);
    CHECK(gw_core_join(&c, eui_c, 1, &gc));
    CHECK(gc.short_addr != gb.short_addr);
}

int main(void)
{
    test_init();
    test_join_fills_slots_in_order();
    test_rejoin_is_idempotent();
    test_network_full();
    test_keepalive();
    test_lease_expiry();
    test_release();
    test_slotmap();
    test_addr_pool_skips_live_seats();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\gw_core\test_gw_core.exe `
    tests\gw_core\test_gw_core.c src\gw_core.c
```

Expected: FAIL — `src\gw_core.c` and `gw_core.h` do not exist.

- [ ] **Step 3: Copy the two files from fw-cre**

```powershell
$fw = "C:\Users\JoseAntonioLaraPerez\Documents\fw-cre\firmeware_creator\Src"
Copy-Item "$fw\gw_core.h" src\
Copy-Item "$fw\gw_core.c" src\
```

The logic is correct as it stands and must not be rewritten. Add only an Apache-2.0 header comment matching the other `src/` files, noting the origin:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CFP seat management for the gateway: a 12-entry table keyed by slot index,
 * leases aged one superframe at a time, and a monotonic short-address pool for
 * joining tags. Ported unchanged from the nRF5 gateway (fw-cre Src/gw_core.c);
 * it was already pure C with no radio dependency, which is why it carries the
 * whole host-test burden for the MAC's state machine.
 */
```

- [ ] **Step 4: Run the test to verify it passes**

Run:

```powershell
gcc -Wall -Wextra -Isrc -o tests\gw_core\test_gw_core.exe `
    tests\gw_core\test_gw_core.c src\gw_core.c
.\tests\gw_core\test_gw_core.exe
```

Expected: `PASSED`, exit code 0.

If `test_addr_pool_skips_live_seats` fails, do **not** patch the test to match the behaviour — `alloc_short_addr()` explicitly skips addresses held by live seats, and a failure there means the copy is wrong.

- [ ] **Step 5: Add to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, alphabetically after `src/disc_schedule.c`:

```cmake
	src/gw_core.c
```

- [ ] **Step 6: Verify the firmware builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: clean build.

- [ ] **Step 7: Commit**

```bash
git add src/gw_core.h src/gw_core.c tests/gw_core/test_gw_core.c CMakeLists.txt
git commit -m "feat(gw): port the CFP seat manager

Unchanged from the nRF5 gateway -- already pure C with no radio
dependency. Host tests cover the whole state machine: slot allocation,
idempotent re-join by EUI (a tag that missed its GRANT and retried must
not consume a second seat), lease aging to the exact expiry tick, a
refused thirteenth join, and the address pool skipping a value a live
seat still holds."
```

---

### Task 6: The gateway MAC loop

**Files:**
- Modify: `src/uwb_gateway.c` (replaces the whole stub)

**Interfaces:**
- Consumes: `gw_core_*` (Task 5); `uwb_get_tx_timestamp_u64()`, `UUS_TO_HI32`, `UUS_TO_DWT_TIME`, `FCS_LEN`, `uwb_wait_for_sysstatus_lo()` (Task 3 and existing); `uwb_frame_beacon_build()`, `uwb_frame_is_join()`, `uwb_frame_parse_join()`, `uwb_frame_grant_build()`, `uwb_frame_is_keepalive()`, `uwb_frame_parse_keepalive()`, `uwb_frame_is_release()`, `uwb_frame_get_src_addr()`, `uwb_frame_set_seq_num()`, `UWB_FRAME_MAX_LEN`, `UWB_FRAME_LEN_BEACON`, `UWB_FRAME_LEN_GRANT`, `UWB_FRAME_EUI_LEN` (existing).
- Produces: nothing — `uwb_gateway_run()` is already declared in `src/uwb_modes.h` and called from `src/main.c`.

- [ ] **Step 1: Replace `src/uwb_gateway.c` entirely**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATEWAY mode: the TDMA beacon and the CAP seat protocol.
 *
 * MAC-only -- this node does NOT answer ranging polls, unlike the nRF5
 * gateway's dispatch() fall-through. That costs a board rather than an anchor
 * (the deployment is one gateway plus four slaves) and buys a much smaller
 * beacon arm margin: with no anchor_respond in the loop the worst-case service
 * latency is one GRANT, not a 16.5 ms discovery stagger.
 *
 * Interrupt-driven with the same callback shape as uwb_slave.c. The DW3000
 * system clock is authoritative over the beacon cadence -- the beacon IS the
 * network's time base, so it is scheduled against the radio's own clock and
 * never against the kernel's.
 */

#include "uwb_modes.h"

#include "gw_core.h"
#include "uwb_dwtime.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_mac.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

LOG_MODULE_REGISTER(uwb_gateway, LOG_LEVEL_INF);

/* T_SUPERFRAME_UUS comes from uwb_mac.h — the slaves predict the beacon from
 * the same definition, and a local copy here would drift against theirs. */

/* Delay from a received CAP frame to our GRANT TX. Same budget class as the
 * responders' turnaround; see disc_schedule.h for why this port needs more
 * than the nRF5 anchor's 2000. */
#define RX_TO_TX_DLY_UUS 2000u

/* Stop servicing and arm the beacon when this close to it. MAC-only keeps this
 * small: worst-case service latency is one GRANT (RX_TO_TX_DLY_UUS + ~1.3 ms
 * airtime + the bounded TXFRS wait), not the nRF5 gateway's 8000, which had to
 * cover a discovery stagger this node no longer performs. */
#define BEACON_ARM_MARGIN_UUS 5000u

/* Bound for the post-dwt_starttx() TXFRS wait. Covers the scheduled delay
 * itself plus airtime -- dwt_starttx() returns when the TX is armed, not when
 * it fires. The longest scheduled delay here is RX_TO_TX_DLY_UUS (~2.05 ms);
 * 10 ms is comfortable. See uwb_dwtime.h for why this must be bounded at all. */
#define TX_COMPLETE_TIMEOUT_MS 10

/* Longest frame the contract defines is a 39-byte beacon; +FCS, rounded up. */
#define RX_BUF_LEN 64

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Same rx_pending guard as uwb_slave.c: br101's IRQ-drain loop can call
 * dwt_isr() again before the main thread has consumed the previous event, and
 * the limit-1 semaphore would silently swallow the second give while the
 * second callback corrupted the still-unread values. */
static volatile uint32_t rx_status;
static volatile uint16_t rx_len;
static volatile bool rx_pending;

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
static uint8_t gw_seq;

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

/* Transmit the beacon. Returns its 40-bit TX timestamp, or 0 if it did not go
 * out. delayed=0 for the very first beacon, which has no predecessor to
 * schedule against. */
static uint64_t tx_beacon(struct gw_core_ctx *ctx, bool delayed, uint32_t tx_at)
{
	uint16_t slot_map[GW_N_CFP];

	gw_core_build_slotmap(ctx, slot_map);

	/* GW_N_CFP == UWB_FRAME_N_CFP == 12, so need = 15 + 24 = 39 =
	 * UWB_FRAME_MAX_LEN and beacon_buf is exactly large enough.
	 * uwb_frame_beacon_build() does not bound n_slots itself -- a known
	 * defect in the frame module, left unfixed because that file is kept
	 * byte-identical to the tag's. It cannot fire here: the argument is a
	 * compile-time constant equal to the maximum. */
	int n = uwb_frame_beacon_build(beacon_buf, sizeof(beacon_buf),
				       ctx->frame_counter, slot_map, GW_N_CFP);
	if (n < 0) {
		LOG_ERR("beacon build failed (%d)", n);
		return 0;
	}
	uwb_frame_set_seq_num(beacon_buf, gw_seq++);

	if (delayed) {
		dwt_setdelayedtrxtime(tx_at);
	}
	dwt_writetxdata((uint16_t)n, beacon_buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(delayed ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
	    != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("beacon missed its slot — re-basing cadence");
		return 0;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("beacon started but TXFRS never completed — forced off");
		return 0;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	return uwb_get_tx_timestamp_u64();
}

static void send_grant(const uint8_t eui[UWB_FRAME_EUI_LEN],
		       const struct gw_grant *g, uint64_t rx_ts)
{
	uint8_t buf[UWB_FRAME_LEN_GRANT];

	/* eui is non-NULL by construction: it comes from a successfully parsed
	 * JOIN. uwb_frame_grant_build() writes its header before checking the
	 * pointer -- another known frame-module defect left unfixed for
	 * byte-identity with the tag -- which cannot fire on this path. */
	int n = uwb_frame_grant_build(buf, sizeof(buf), eui, g->short_addr,
				      g->slot_index, g->tier, g->lease);
	if (n < 0) {
		LOG_WRN("grant build failed (%d)", n);
		return;
	}
	uwb_frame_set_seq_num(buf, gw_seq++);

	uint32_t tx_at = (uint32_t)((rx_ts +
		((uint64_t)RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

	dwt_setdelayedtrxtime(tx_at);
	dwt_writetxdata((uint16_t)n, buf, 0);
	dwt_writetxfctrl((uint16_t)(n + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("grant missed its slot — tag will retry via CAP");
		return;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("grant started but TXFRS never completed — forced off");
		return;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

static void dispatch(struct gw_core_ctx *ctx, const uint8_t *buf, uint16_t len,
		     uint64_t rx_ts)
{
	if (uwb_frame_is_join(buf, len)) {
		uint8_t eui[UWB_FRAME_EUI_LEN];
		uint8_t req_tier = 0;
		struct gw_grant g;

		if (uwb_frame_parse_join(buf, len, eui, &req_tier) != 0) {
			return;
		}
		if (!gw_core_join(ctx, eui, req_tier, &g)) {
			LOG_WRN("JOIN refused — all %u seats occupied", GW_N_CFP);
			return;
		}
		LOG_INF("GRANT addr=0x%04X slot=%u tier=%u lease=%u",
			g.short_addr, g.slot_index, g.tier, g.lease);
		send_grant(eui, &g, rx_ts);
	} else if (uwb_frame_is_keepalive(buf, len)) {
		uint16_t sa = 0;
		uint8_t rt = 0, si = 0;

		if (uwb_frame_parse_keepalive(buf, len, &sa, &rt, &si) == 0) {
			gw_core_keepalive(ctx, sa, rt);
		}
	} else if (uwb_frame_is_release(buf, len)) {
		uint16_t sa = uwb_frame_get_src_addr(buf);

		LOG_INF("RELEASE addr=0x%04X", sa);
		gw_core_release(ctx, sa);
	}
	/* Anything else is tag<->anchor ranging traffic. MAC-only: not ours,
	 * and logging every frame on a busy network would flood the console. */
}

void uwb_gateway_run(const uwb_config_t *cfg)
{
	/* Snapshot at entry — see uwb_slave.c for why. It matters more here:
	 * this loop runs indefinitely and drives every other node's timing. */
	uwb_config_t cfg_snapshot = *cfg;

	cfg = &cfg_snapshot;

	if (!cfg->position_valid) {
		LOG_ERR("{\"error\":\"gateway not positioned\"} — "
			"set `anchor pos <x> <y> <z>` and reboot");
		return;
	}

	static dwt_callbacks_s cbs;

	cbs.cbRxOk = cb_rx_ok;
	cbs.cbRxTo = cb_rx_fail;
	cbs.cbRxErr = cb_rx_fail;
	dwt_setcallbacks(&cbs);

	/* RX events only. TXFRS must stay masked: tx_beacon() and send_grant()
	 * poll for it, and an ISR that cleared it first would make both wait
	 * out their full timeout on every single transmission. */
	dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT);

	struct gw_core_ctx ctx;

	gw_core_init(&ctx);

	LOG_INF("{\"status\":\"gateway\",\"x\":%.2f,\"y\":%.2f,"
		"\"superframe_ms\":200,\"slots\":%u}",
		(double)cfg->x, (double)cfg->y, GW_N_CFP);

	uint64_t beacon_tx_ts = tx_beacon(&ctx, false, 0);

	if (beacon_tx_ts == 0) {
		LOG_ERR("first beacon failed to transmit — cannot start");
		return;
	}

	while (1) {
		uint32_t next_beacon = (uint32_t)((beacon_tx_ts +
			((uint64_t)T_SUPERFRAME_UUS * UUS_TO_DWT_TIME)) >> 8);

		for (;;) {
			uint32_t now = dwt_readsystimestamphi32();
			int32_t to_beacon = (int32_t)(next_beacon - now);

			if (to_beacon <= (int32_t)UUS_TO_HI32(BEACON_ARM_MARGIN_UUS)) {
				break;
			}

			/* Expire the RX window BEACON_ARM_MARGIN_UUS before the
			 * beacon, so the delayed-TX setup has a guaranteed
			 * window. Without the subtraction the timeout fires at
			 * exactly the beacon instant and the delayed TX fails
			 * every time. */
			uint32_t span_hi32 =
				(uint32_t)to_beacon - UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
			uint32_t rx_to_uus =
				(uint32_t)(((uint64_t)span_hi32 << 8) / UUS_TO_DWT_TIME);

			dwt_setpreambledetecttimeout(0);
			dwt_setrxtimeout(rx_to_uus);
			dwt_setrxaftertxdelay(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);

			/* Bounded, not K_FOREVER. DWT_INT_RX includes RXFTO so a
			 * timeout normally arrives, but a MAC loop that can wedge
			 * takes the whole network down, not one range. One
			 * superframe of slack past the window is ample. */
			if (k_sem_take(&rx_sem, K_MSEC(400)) != 0) {
				LOG_WRN("no RX event within the window — re-arming");
				dwt_forcetrxoff();
				continue;
			}

			/* rx_status is captured by the callbacks for symmetry with
			 * uwb_slave.c but is not read here: the gateway needs no
			 * CIR, since it does not answer ranging polls. */
			uint16_t flen = rx_len;

			rx_pending = false;

			if (flen <= FCS_LEN || flen > RX_BUF_LEN) {
				continue;
			}

			dwt_readrxdata(rx_buf, flen, 0);

			uint64_t rx_ts = uwb_get_rx_timestamp_u64();

			dispatch(&ctx, rx_buf, (uint16_t)(flen - FCS_LEN), rx_ts);
		}

		gw_core_superframe_tick(&ctx);

		uint64_t ts = tx_beacon(&ctx, true, next_beacon);

		/* On a miss, re-base on the current time rather than compounding
		 * the error into every following superframe. */
		beacon_tx_ts = ts ? ts
				  : (((uint64_t)dwt_readsystimestamphi32()) << 8);
	}
}
```

- [ ] **Step 2: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: clean build, no warnings.

`CONFIG_CBPRINTF_FP_SUPPORT` is already set in `prj.conf`, so the `%.2f` in the status banner works.

- [ ] **Step 3: Run every host test**

Run:

```powershell
gcc -Wall -Wextra -o tests\uwb_config\test_uwb_config.exe tests\uwb_config\test_uwb_config.c src\uwb_config.c
.\tests\uwb_config\test_uwb_config.exe
gcc -Wall -Wextra -o tests\uwb_frame\test_uwb_frame.exe tests\uwb_frame\test_uwb_frame.c src\uwb_frame_802_15_4z.c
.\tests\uwb_frame\test_uwb_frame.exe
gcc -Wall -Wextra -Isrc -o tests\disc_schedule\test_disc_schedule.exe tests\disc_schedule\test_disc_schedule.c src\disc_schedule.c
.\tests\disc_schedule\test_disc_schedule.exe
gcc -Wall -Wextra -Isrc -o tests\gw_core\test_gw_core.exe tests\gw_core\test_gw_core.c src\gw_core.c
.\tests\gw_core\test_gw_core.exe
gcc -Wall -Wextra -Isrc -o tests\beacon_guard\test_beacon_guard.exe tests\beacon_guard\test_beacon_guard.c src\beacon_guard.c
.\tests\beacon_guard\test_beacon_guard.exe
```

Expected: `PASSED`, `ALL TESTS PASSED`, `PASSED`, `PASSED`, `PASSED`.

- [ ] **Step 4: Commit**

```bash
git add src/uwb_gateway.c
git commit -m "feat(gateway): TDMA beacon and CAP seat protocol

Replaces the spec A stub. MAC-only: this node does not answer ranging
polls, which costs a board rather than an anchor and lets the beacon arm
margin drop from the nRF5 gateway's 8000 to 5000 UUS -- there is no
16.5 ms discovery stagger to cover.

The RX window is closed BEACON_ARM_MARGIN_UUS before the beacon so the
delayed-TX setup has a guaranteed window; without that subtraction the
timeout fires at exactly the beacon instant and the TX fails every time.

A missed beacon re-bases the cadence on the current system time instead
of compounding the error forward. Both TXFRS waits are bounded, and the
semaphore take is bounded too -- a wedged MAC loop takes the network
down, not one range."
```

---

### Task 7: Wire beacon suppression into the slave

**Files:**
- Modify: `src/anchor_respond.h`
- Modify: `src/anchor_respond.c`
- Modify: `src/uwb_slave.c`

**Interfaces:**
- Consumes: `struct beacon_guard`, `beacon_guard_init()`, `beacon_guard_beacon()`, `beacon_guard_tx_allowed()`, `beacon_guard_locked()` (Task 4); `UUS_TO_HI32` (Task 3).
- Produces: both responder functions gain a trailing `struct beacon_guard *bg` parameter. `NULL` disables suppression, so the API stays usable from a caller with no beacon reference.

- [ ] **Step 1: Add the parameter to the responder declarations**

In `src/anchor_respond.h`, add the include and extend both prototypes:

```c
#include "beacon_guard.h"
```

```c
/* bg may be NULL, which disables suppression. When non-NULL, a response whose
 * scheduled TX would land inside the beacon window is dropped rather than
 * transmitted -- one lost range instead of a corrupted broadcast. */
void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len, uint64_t poll_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      struct beacon_guard *bg);

void anchor_respond_discovery(const uint8_t *buf, uint16_t len, uint64_t disc_rx_ts,
			      const uwb_config_t *cfg, uint8_t *seq,
			      int32_t cir_power, uint16_t cir_quality,
			      struct beacon_guard *bg);
```

- [ ] **Step 2: Apply the check in both responders**

In `src/anchor_respond.c`, update both signatures to match, and insert the check immediately after `resp_tx_time` is computed and **before** any `tx_delayed()` call.

In `anchor_respond_wave_poll()`, after the `resp_tx_ts` assignment:

```c
	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("WAVE response suppressed — would land on the beacon");
		return;
	}
```

In `anchor_respond_discovery()`, after the `resp_tx_time` assignment:

```c
	if (bg && !beacon_guard_tx_allowed(bg, resp_tx_time)) {
		LOG_DBG("DISCOVERY response suppressed — would land on the beacon");
		return;
	}
```

Both use `LOG_DBG` for the same reason the existing per-frame logs do: this path's timing budget is tight, and the module registers at `LOG_LEVEL_INF` so the call compiles out. Bump the module level to see them.

- [ ] **Step 3: Instantiate and feed the guard in the slave**

In `src/uwb_slave.c`, add the includes:

```c
#include "beacon_guard.h"
#include "uwb_mac.h"
```

Add the instance near the other file-scope state. The superframe and beacon-window
constants come from `uwb_mac.h` — the same definitions the gateway schedules from,
so the prediction cannot drift against the transmitter:

```c
static struct beacon_guard bguard;
```

In `observe_beacon()`, re-anchor the prediction on every beacon actually received. Change its signature to take the RX timestamp, and add the call after the `n_slots` sanity check:

```c
static void observe_beacon(const uint8_t *buf, uint16_t len,
			   const uwb_config_t *cfg, uint64_t rx_ts)
{
	...
	/* Re-anchor on the beacon's own RX timestamp, in the same hi32 units
	 * every scheduled TX is expressed in — no clock conversion, so no
	 * second clock to drift against. */
	beacon_guard_beacon(&bguard, (uint32_t)(rx_ts >> 8));

	LOG_DBG("BEACON ver=%u counter=%u slots=%u our_slot=%d", proto_ver,
		frame_counter, n_slots, slot);
}
```

- [ ] **Step 4: Initialise the guard and pass it to the responders**

In `uwb_slave_run()`, initialise before `rx_arm()`:

```c
	beacon_guard_init(&bguard, UUS_TO_HI32(T_SUPERFRAME_UUS),
			  UUS_TO_HI32(BEACON_GUARD_UUS),
			  UUS_TO_HI32(BEACON_OCCUPANCY_UUS));
```

and update the three dispatch calls in the main loop:

```c
		anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
					 &bguard);
		anchor_respond_discovery(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
					 cir_power, cir_quality, &bguard);
		observe_beacon(rx_buf, plen, cfg, rx_ts);
```

**Order matters and is deliberate:** the responders run *before* `observe_beacon()`, so a beacon frame is never both the thing that re-anchors the prediction and the thing checked against it. A beacon is not a poll, so neither responder acts on it — but keeping the order explicit means a future responder that did would still see the previous superframe's reference.

- [ ] **Step 5: Verify it builds**

Run:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: clean build, no warnings. If the compiler reports too few arguments to either responder, a call site was missed — `src/uwb_slave.c` is the only caller.

- [ ] **Step 6: Run every host test**

Run the five-suite block from Task 6 Step 3. Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/anchor_respond.h src/anchor_respond.c src/uwb_slave.c
git commit -m "feat(slave): suppress responses that would land on the beacon

Both responders take a beacon_guard (NULL disables) and drop a response
whose scheduled TX falls inside the beacon window. The guard re-anchors
on each received beacon's own RX timestamp, in the same hi32 units the
TX time is already expressed in -- so there is no clock conversion and
no second clock to drift against.

Responders run before observe_beacon() so a beacon can never be both the
frame that re-anchors the prediction and the frame checked against it."
```

---

### Task 8: Documentation

**Files:**
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: nothing.
- Produces: nothing.

- [ ] **Step 1: Add the new modules to the Layout section**

In `CLAUDE.md`, after the `src/disc_schedule.{c,h}` bullet:

```markdown
- `src/gw_core.{c,h}` — CFP seat table, leases and the tag address pool. Pure
  C, ported unchanged from the nRF5 gateway, host-tested in `tests/gw_core/`.
- `src/beacon_guard.{c,h}` — predicts the next beacon and refuses any delayed
  TX that would land on it. Pure C, host-tested in `tests/beacon_guard/`.
```

and replace the `src/uwb_slave.c` bullet with:

```markdown
- `src/uwb_slave.c` — SLAVE mode: interrupt-driven SS-TWR responder, beacon
  observe, and beacon-collision TX suppression.
- `src/uwb_gateway.c` — GATEWAY mode: TDMA beacon plus the CAP seat protocol.
  MAC-only; it does not answer ranging polls.
```

- [ ] **Step 2: Correct the SPI record and replace the superseded facts**

The `POLL_RX_TO_RESP_TX_DLY_UUS` and `read_cir()` bullets in `## Hard-won facts` both attribute the RX-side overhead to driver dispatch and the bit-banged CS. That conclusion is now known to be wrong. **Replace both bullets** with:

```markdown
- **The SPI bus defaults to 2 MHz and nothing switches it — you must call
  `dw3000_spi_speed_fast()` yourself.** `dw3000_spi_init()` selects
  `spi_cfgs[0]` (2 MHz); `spi_cfgs[1]` carries the DTS rate and is never
  selected unless something calls the switch. `.setfastrate` is wired into the
  driver vtable (`platform/deca_port.c:37`) but its only call sites are in a
  `static init()` wrapper that `dwt_initialise()` does not dispatch to — it
  goes to `ull_initialise` (`dw3000_device.c:9371`). The module's boot log
  prints `spi_cfgs[1].frequency`, so it reports the DTS rate whether or not
  that rate is in use: **that log line is not evidence.** `uwb_radio.c` makes
  the call after `dwt_checkidlerc()`, which is the earliest legal point — the
  part requires ≤7 MHz until it leaves INIT_RC (`deca_device_api.h:2381`,
  `:2601`).
- **The RX-side overhead was the 2 MHz bus, not the driver.** An earlier
  entry here blamed the vendored driver's ioctl-style dispatch and bit-banged
  CS for ~1700 uus between RX and `dwt_starttx()`, and concluded a 2000 uus
  turnaround was unreachable on this stack. It was reachable; the bus was
  running at a quarter of the configured rate. `read_cir()`'s
  `DW_CIA_DIAG_LOG_ALL` read is two ~108-byte bursts = 1728 bits, which is
  864 us of pure clock time at 2 MHz against ~944 us measured — the transfer
  was essentially all of it. The corollary also stands: logging was never the
  dominant cost, so do not re-litigate that either.
- **Maximum usable SPI rate here is 26.67 MHz (80/3).** The ESP32-S3 only
  produces 80 MHz/N, so the ladder is 20 / 26.67 / 40 with nothing between,
  and 40 overruns the DW3000's ~38 MHz ceiling. Request `26670000` in the DTS,
  never `26000000`: the HAL picks the highest rate **not exceeding** the
  request, so asking for 26 silently yields 20. GPIO 11/12/13 are the SPI2
  IO_MUX pins (FSPID/FSPICLK/FSPIQ), so there is no GPIO-matrix penalty at
  this rate. The module README's `spi-max-frequency = <32000000>` is the
  nRF52840 SPIM3 limit and does not transfer.
```

- [ ] **Step 3: Add the hi32 wrap fact**

Append to `## Hard-won facts`:

```markdown
- **`dwt_readsystimestamphi32()` wraps every ~17.2 s.** hi32 counts 256 DTU
  ≈ 4.006 ns per tick, so 2³² ticks is 17.2 seconds. Every comparison between
  two hi32 values must be signed-difference arithmetic — `(int32_t)(a - b)` —
  which is correct for any interval under ~8.6 s. A plain unsigned compare is
  wrong across the wrap and the failure is rare, timing-dependent and looks
  like a radio fault. `beacon_guard.c` does this correctly and
  `tests/beacon_guard/` covers both directions across the boundary.
- **The gateway is MAC-only and holds two reserved values at once.** It uses
  short address `0x0000` per the contract and consumes no `anchor_id`, so the
  four ranging slaves take ids 0..3 (`0x0001`–`0x0004`). The deployment is
  therefore **five boards**, not four. Three ranging anchors would satisfy the
  tag's solver (`pos_solver.c:10` accepts n ≥ 3) but only as an exact solve
  with no averaging; the fourth makes it overdetermined.
```

- [ ] **Step 4: Update the Console section**

The `anchor show` output gained `short_addr` in the SLAVE work. Confirm the `## Console` block reflects it, and add the gateway note:

```markdown
anchor mode <slave|gateway>    boot mode (default slave). GATEWAY refuses to
                               beacon unless `anchor pos` has been set.
```

- [ ] **Step 5: Add the new host tests**

Append to the `## Host tests` block:

````markdown
```powershell
gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe tests/gw_core/test_gw_core.c src/gw_core.c
./tests/gw_core/test_gw_core.exe                # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/beacon_guard/test_beacon_guard.exe tests/beacon_guard/test_beacon_guard.c src/beacon_guard.c
./tests/beacon_guard/test_beacon_guard.exe      # PASSED, exits 0
```
````

- [ ] **Step 6: Update the System context section**

Replace the sentence beginning *"but the gateway/beacon side is still to come"* with:

```markdown
The gateway/beacon side is now implemented (`src/uwb_gateway.c`,
`src/gw_core.{c,h}`): a MAC-only gateway emits the 200 ms TDMA beacon and runs
the CAP seat protocol (JOIN→GRANT, KEEPALIVE, RELEASE), and slaves suppress
ranging responses that would collide with that beacon
(`src/beacon_guard.{c,h}`).
```

- [ ] **Step 7: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record the gateway modules and correct the SPI record

Two hard-won facts were wrong and are replaced rather than amended: the
RX-side overhead was the 2 MHz bus, not the vendored driver's dispatch
or its bit-banged CS, and a 2000 uus turnaround was never unreachable on
this stack. Leaving the old conclusion in place would send the next
reader looking for savings in the wrong layer.

Adds the hi32 17.2 s wrap, the five-board deployment, and the rule that
the module's SPI boot log reports the DTS rate whether or not it is in
use -- that line is not evidence."
```

---

## On-target verification

Runs after Task 8, with all five boards. Not a task: it needs hardware and a human.

Set up: one board `anchor mode gateway` + `anchor pos <x> <y> <z>`, four boards `anchor mode slave` with `anchor id 0`..`3`. Cold reboot each after configuring.

1. **Gateway starts.** Boot log shows `{"status":"gateway",...,"slots":12}`. An unpositioned gateway instead logs `gateway not positioned` and emits nothing — verify that path too, then set the position.
2. **Beacons are periodic.** A slave with its module at `LOG_LEVEL_DBG` logs `BEACON` lines with an incrementing `counter` roughly every 200 ms. Confirm the interval on the sniffer, not just the count.
3. **A tag JOINs.** Sniffer shows `0xE6` from `0xFFFE`, then `0xE7` from `0x0000` carrying a slot index and a short address from `0x0100`. The next beacon's slot map contains that address.
4. **Ranging inside the seat.** The tag ranges four anchors in its slot; the sniffer shows four `0xE1`/`0xE4` responses with distinct source addresses `0x0001`–`0x0004`.
5. **Lease aging.** Stop the tag's KEEPALIVEs. After `GW_LEASE_SF` (50) superframes ≈ 10 s the seat frees and the slot map returns to `0xFFFF`. A RELEASE frees it immediately instead.
6. **Suppression fires.** With a slave at `LOG_LEVEL_DBG`, `... response suppressed — would land on the beacon` appears, and the corresponding frame is absent from the sniffer capture with the beacon intact. If it never appears over several minutes of discovery traffic, check that `beacon_guard_locked()` is true — an unlocked guard allows everything and would look identical to "no collisions occurred."

Step 6's failure mode is the one to watch: silence is ambiguous between "working" and "never armed."

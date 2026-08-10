# Anchor Config, Console and Mode Selection — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the ESP32-S3 anchor a persistent, console-configurable identity — ranging id (default 0), SLAVE/GATEWAY mode, antenna delays and coordinates — and dispatch to a mode entry point at boot.

**Architecture:** Five modules in `src/`. `uwb_config` is pure C (config data + validation, host-tested). `uwb_store` persists it via Zephyr `settings` over NVS on the board's existing `storage_partition`. `anchor_shell` exposes an `anchor` shell command tree over the native USB-JTAG console. `uwb_radio` holds the DW3000 bring-up sequence that specs C and D will both call. `main.c` wires them together and dispatches on mode to stub entry points that specs C and D fill in. The main thread is the UWB thread; config is read-only after boot, so the ranging hot path needs no locking.

**Tech Stack:** Zephyr 4.4.x, ESP32-S3 (`ancla_esp32s3/esp32s3/procpu`), vendored br101 `dw3000-decadriver` module (Qorvo `dwt_uwb_driver` 08.02.02), Zephyr `settings` + `nvs` + `shell`, MinGW gcc 16.1 for host tests.

**Spec:** `docs/superpowers/specs/2026-08-10-anchor-config-console-design.md`

## Global Constraints

- **`$env:ZEPHYR_BASE` must be set** to `C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr` before any `west` command — this project lives outside the west workspace.
- **Build command:** `west build -b ancla_esp32s3/esp32s3/procpu`. Flash with `west flash`, monitor with `west espressif monitor -p COM5`.
- **`dwt_initialise()` must be the first register access after `dwt_probe()`.** It is what assigns `dw->priv`; calling `dwt_checkidlerc()`, `dwt_readdevid()` or `dwt_configure()` before it faults with `EXCCAUSE 28 / VADDR 0x24`.
- **`dwt_configciadiag()` must be called after `dwt_configure()`**, or every diagnostic register reads zero.
- **PHY is fixed, never runtime-configurable:** channel 5, `DWT_PLEN_1024`, `DWT_PAC32`, TX/RX preamble code 9, `DWT_BR_850K`, `DWT_SFD_IEEE_4Z` (`sfdType = 3`), `DWT_STS_MODE_OFF`, `DWT_PDOA_M0`, `sfdTO = 1025 + 8 - 32`.
- **Default anchor id is 0. Default mode is SLAVE.** Default antenna delays are 16385 TX and 16385 RX.
- **`UWB_MAX_ANCHORS` is 4.** Valid ids are 0..3.
- **Console is `usb_serial` (native USB-JTAG), not `uart0`.** The board's `chosen` node already selects it; do not change it.
- **Do not modify `boards/` or `modules/`.** `storage_partition` already exists (192 K at 0x7b0000); no DTS change is needed.
- Commit on `master`. This repo is local-only with no remote.
- Every file in `src/` starts with the existing header style: `/*\n * Copyright (c) 2026 Innovaforce\n *\n * SPDX-License-Identifier: Apache-2.0\n *\n * <one-paragraph description>\n */`

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `src/uwb_config.h` / `.c` | Config struct, defaults, per-field validation. Pure C, no Zephyr headers. | 1 |
| `tests/uwb_config/test_uwb_config.c` | Host test for the above. | 1 |
| `src/uwb_phy.h` | The fixed PHY contract as a `dwt_config_t` initializer plus antenna-delay default. | 2 |
| `src/uwb_radio.h` / `.c` | DW3000 bring-up: reset → probe → initialise → configure → ciadiag → txrf → antenna delays → LNA/PA. | 2 |
| `src/uwb_store.h` / `.c` | Zephyr `settings` handler over NVS; load-all at boot, save-one per field. | 3 |
| `src/uwb_modes.h` | `uwb_slave_run()` / `uwb_gateway_run()` declarations — the seam specs C and D fill. | 3 |
| `src/uwb_slave.c` | SLAVE entry point. Stub in this spec. | 3 |
| `src/uwb_gateway.c` | GATEWAY entry point. Stub in this spec. | 3 |
| `src/anchor_shell.c` | `anchor` shell command tree. | 4 |
| `src/main.c` | Boot: settings load → radio bring-up → dispatch on mode. | 2, 3 |
| `prj.conf` | Kconfig for flash/NVS/settings/shell. | 2, 3, 4 |
| `CMakeLists.txt` | Source list. | 1–4 |
| `CLAUDE.md` | Project guide — layout and console sections. | 4 |

---

### Task 1: `uwb_config` — config data and validation

Pure C with no Zephyr dependency, so it compiles and runs on the host with one `gcc` command. Everything else in this plan consumes it.

**Files:**
- Create: `src/uwb_config.h`
- Create: `src/uwb_config.c`
- Create: `tests/uwb_config/test_uwb_config.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: the `uwb_config_t` struct and `UWB_MODE_SLAVE` / `UWB_MODE_GATEWAY`, plus `uwb_config_get()`, `uwb_config_set_defaults()`, `uwb_config_set_mode()`, `uwb_config_set_id()`, `uwb_config_set_ant()`, `uwb_config_set_pos()`, `uwb_config_mode_from_name()`, `uwb_config_mode_name()`. Tasks 2, 3 and 4 all use these exact names.

- [ ] **Step 1: Write the failing test**

Create `tests/uwb_config/test_uwb_config.c`:

```c
#include "../../src/uwb_config.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_defaults(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    CHECK(c.mode == UWB_MODE_SLAVE);
    CHECK(c.anchor_id == 0);
    CHECK(c.ant_delay_tx == UWB_ANT_DELAY_DEFAULT);
    CHECK(c.ant_delay_rx == UWB_ANT_DELAY_DEFAULT);
    CHECK(c.x == 0.0f && c.y == 0.0f && c.z == 0.0f);
    CHECK(!c.position_valid);
}

static void test_set_id(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    for (uint8_t id = 0; id < UWB_MAX_ANCHORS; id++) {
        CHECK(uwb_config_set_id(&c, id));
        CHECK(c.anchor_id == id);
    }

    /* Out of range is rejected and leaves the previous value intact. */
    CHECK(uwb_config_set_id(&c, 2));
    CHECK(!uwb_config_set_id(&c, UWB_MAX_ANCHORS));
    CHECK(c.anchor_id == 2);
    CHECK(!uwb_config_set_id(&c, 255));
    CHECK(c.anchor_id == 2);
}

static void test_set_mode(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    CHECK(uwb_config_set_mode(&c, UWB_MODE_GATEWAY));
    CHECK(c.mode == UWB_MODE_GATEWAY);
    CHECK(uwb_config_set_mode(&c, UWB_MODE_SLAVE));
    CHECK(c.mode == UWB_MODE_SLAVE);

    CHECK(uwb_config_set_mode(&c, UWB_MODE_GATEWAY));
    CHECK(!uwb_config_set_mode(&c, 2));
    CHECK(c.mode == UWB_MODE_GATEWAY);
}

static void test_mode_names(void)
{
    uint8_t m = 0xFF;

    CHECK(uwb_config_mode_from_name("slave", &m));
    CHECK(m == UWB_MODE_SLAVE);
    CHECK(uwb_config_mode_from_name("gateway", &m));
    CHECK(m == UWB_MODE_GATEWAY);

    m = UWB_MODE_GATEWAY;
    CHECK(!uwb_config_mode_from_name("master", &m));
    CHECK(!uwb_config_mode_from_name("", &m));
    CHECK(m == UWB_MODE_GATEWAY);   /* unchanged on rejection */

    CHECK(strcmp(uwb_config_mode_name(UWB_MODE_SLAVE), "slave") == 0);
    CHECK(strcmp(uwb_config_mode_name(UWB_MODE_GATEWAY), "gateway") == 0);
    CHECK(strcmp(uwb_config_mode_name(200), "unknown") == 0);
}

static void test_set_ant(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);

    CHECK(uwb_config_set_ant(&c, 0, 0));
    CHECK(c.ant_delay_tx == 0 && c.ant_delay_rx == 0);
    CHECK(uwb_config_set_ant(&c, 65535, 65535));
    CHECK(c.ant_delay_tx == 65535 && c.ant_delay_rx == 65535);

    /* Above uint16 range: rejected, both fields unchanged. */
    CHECK(uwb_config_set_ant(&c, 16385, 16400));
    CHECK(!uwb_config_set_ant(&c, 65536, 100));
    CHECK(c.ant_delay_tx == 16385 && c.ant_delay_rx == 16400);
    CHECK(!uwb_config_set_ant(&c, 100, 65536));
    CHECK(c.ant_delay_tx == 16385 && c.ant_delay_rx == 16400);
}

static void test_set_pos(void)
{
    uwb_config_t c;
    uwb_config_set_defaults(&c);
    CHECK(!c.position_valid);

    uwb_config_set_pos(&c, 1.5f, -2.25f, 0.75f);
    CHECK(c.x == 1.5f);
    CHECK(c.y == -2.25f);
    CHECK(c.z == 0.75f);
    CHECK(c.position_valid);

    /* The origin is a legitimate position — setting it must still mark valid. */
    uwb_config_set_pos(&c, 0.0f, 0.0f, 0.0f);
    CHECK(c.position_valid);
}

static void test_singleton_starts_at_defaults(void)
{
    uwb_config_t *c = uwb_config_get();
    CHECK(c != NULL);
    CHECK(c->mode == UWB_MODE_SLAVE);
    CHECK(c->anchor_id == 0);
    CHECK(c->ant_delay_tx == UWB_ANT_DELAY_DEFAULT);
    CHECK(uwb_config_get() == c);   /* same instance every call */
}

int main(void)
{
    test_defaults();
    test_set_id();
    test_set_mode();
    test_mode_names();
    test_set_ant();
    test_set_pos();
    test_singleton_starts_at_defaults();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
```

Expected: FAIL — `fatal error: ../../src/uwb_config.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `src/uwb_config.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-anchor configuration: identity, mode, antenna calibration and position.
 * Pure C with no Zephyr dependency so the validation is host-testable; the
 * persistence layer lives in uwb_store.c and the console in anchor_shell.c.
 *
 * The PHY is deliberately absent: it is a fixed network contract (see
 * uwb_phy.h), not a runtime setting.
 */

#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Anchors in the deployment; valid ids are 0..UWB_MAX_ANCHORS-1. */
#define UWB_MAX_ANCHORS 4

/* Factory-reference seed for an uncalibrated unit, in DWT units. */
#define UWB_ANT_DELAY_DEFAULT 16385u

enum uwb_mode {
	UWB_MODE_SLAVE   = 0, /* SS-TWR responder; observes the gateway beacon */
	UWB_MODE_GATEWAY = 1, /* emits the TDMA beacon and grants CFP seats    */
	UWB_MODE_COUNT
};

typedef struct {
	uint8_t  mode;           /* enum uwb_mode */
	uint8_t  anchor_id;      /* 0..UWB_MAX_ANCHORS-1 */
	uint16_t ant_delay_tx;
	uint16_t ant_delay_rx;
	float    x;
	float    y;
	float    z;
	bool     position_valid; /* false until coordinates are set */
} uwb_config_t;

/* Overwrite *c with the documented defaults: SLAVE, id 0, 16385/16385,
 * origin, position invalid. */
void uwb_config_set_defaults(uwb_config_t *c);

/* The single active instance, at defaults until uwb_store_load() runs. */
uwb_config_t *uwb_config_get(void);

/* Validating setters. Each returns true and mutates on success, or returns
 * false and leaves *c completely untouched on a rejected value. */
bool uwb_config_set_mode(uwb_config_t *c, uint8_t mode);
bool uwb_config_set_id(uwb_config_t *c, uint8_t id);
bool uwb_config_set_ant(uwb_config_t *c, uint32_t tx, uint32_t rx);

/* Always succeeds; any coordinate triple is legitimate. Sets position_valid. */
void uwb_config_set_pos(uwb_config_t *c, float x, float y, float z);

/* "slave" / "gateway", matched exactly. Returns false and leaves *out
 * untouched on an unknown name. */
bool uwb_config_mode_from_name(const char *name, uint8_t *out);

/* "slave", "gateway", or "unknown" for an out-of-range value. Never NULL. */
const char *uwb_config_mode_name(uint8_t mode);

#endif /* UWB_CONFIG_H */
```

- [ ] **Step 4: Write the implementation**

Create `src/uwb_config.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Config defaults, validation and the single active instance.
 */

#include "uwb_config.h"

#include <string.h>

static const char *const mode_names[UWB_MODE_COUNT] = {
	[UWB_MODE_SLAVE]   = "slave",
	[UWB_MODE_GATEWAY] = "gateway",
};

void uwb_config_set_defaults(uwb_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->mode         = UWB_MODE_SLAVE;
	c->anchor_id    = 0;
	c->ant_delay_tx = UWB_ANT_DELAY_DEFAULT;
	c->ant_delay_rx = UWB_ANT_DELAY_DEFAULT;
	/* x/y/z zeroed above; position_valid stays false so a GATEWAY that has
	 * never been positioned refuses to beacon rather than beaconing from
	 * a bogus origin. */
}

uwb_config_t *uwb_config_get(void)
{
	static uwb_config_t cfg;
	static bool initialised;

	if (!initialised) {
		uwb_config_set_defaults(&cfg);
		initialised = true;
	}
	return &cfg;
}

bool uwb_config_set_mode(uwb_config_t *c, uint8_t mode)
{
	if (mode >= UWB_MODE_COUNT) {
		return false;
	}
	c->mode = mode;
	return true;
}

bool uwb_config_set_id(uwb_config_t *c, uint8_t id)
{
	if (id >= UWB_MAX_ANCHORS) {
		return false;
	}
	c->anchor_id = id;
	return true;
}

bool uwb_config_set_ant(uwb_config_t *c, uint32_t tx, uint32_t rx)
{
	/* Both are validated before either is written, so a rejected pair
	 * leaves the config exactly as it was. */
	if (tx > UINT16_MAX || rx > UINT16_MAX) {
		return false;
	}
	c->ant_delay_tx = (uint16_t)tx;
	c->ant_delay_rx = (uint16_t)rx;
	return true;
}

void uwb_config_set_pos(uwb_config_t *c, float x, float y, float z)
{
	c->x = x;
	c->y = y;
	c->z = z;
	c->position_valid = true;
}

bool uwb_config_mode_from_name(const char *name, uint8_t *out)
{
	for (uint8_t i = 0; i < UWB_MODE_COUNT; i++) {
		if (strcmp(name, mode_names[i]) == 0) {
			*out = i;
			return true;
		}
	}
	return false;
}

const char *uwb_config_mode_name(uint8_t mode)
{
	if (mode >= UWB_MODE_COUNT) {
		return "unknown";
	}
	return mode_names[mode];
}
```

- [ ] **Step 5: Run the test to verify it passes**

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe
```

Expected: `PASSED`, exit code 0, and no compiler warnings.

- [ ] **Step 6: Add the source to the build**

Modify `CMakeLists.txt` — replace the `target_sources` line with:

```cmake
target_sources(app PRIVATE
	src/main.c
	src/uwb_config.c
)
```

- [ ] **Step 7: Verify the firmware still builds**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: build succeeds. `uwb_config.c` is compiled but not yet called by anything.

- [ ] **Step 8: Ignore the test binary**

Modify `.gitignore`, appending:

```
tests/**/*.exe
```

- [ ] **Step 9: Commit**

```powershell
git add src/uwb_config.h src/uwb_config.c tests/uwb_config/test_uwb_config.c CMakeLists.txt .gitignore
git commit -m "feat(config): anchor config struct, defaults and validation"
```

---

### Task 2: `uwb_radio` — extract the DW3000 bring-up

The bring-up sequence currently sits inline in `main.c` as a smoke test, and is duplicated verbatim at the top of both nRF5 owner entry points. Factor it out once, so specs C and D both call it. `main.c` keeps working exactly as it does today — this task must not regress the smoke test.

**Files:**
- Create: `src/uwb_phy.h`
- Create: `src/uwb_radio.h`
- Create: `src/uwb_radio.c`
- Modify: `src/main.c` (whole file rewritten)
- Modify: `CMakeLists.txt`
- Modify: `prj.conf`

**Interfaces:**
- Consumes: `uwb_config_t`, `uwb_config_get()` from Task 1.
- Produces: `int uwb_radio_init(const uwb_config_t *cfg)` — returns 0 on success or a negative errno. Task 3's `main.c` calls it; specs C and D call it as their first action.

- [ ] **Step 1: Write the PHY contract header**

Create `src/uwb_phy.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The fixed PHY contract. Every node on this network — anchors and tags —
 * must match these values exactly, so they are compile-time constants rather
 * than settings. Changing anything here changes the wire and invalidates
 * every unit's antenna calibration.
 */

#ifndef UWB_PHY_H
#define UWB_PHY_H

#include <deca_device_api.h>

/* Channel 5, PLEN-1024, PAC32, code 9, 850 kbps, SFD_IEEE_4Z, STS off. */
#define UWB_PHY_CONFIG_INITIALIZER                                             \
	{                                                                      \
		5,                  /* channel */                              \
		DWT_PLEN_1024,      /* TX preamble length */                   \
		DWT_PAC32,          /* RX preamble acquisition chunk */        \
		9,                  /* TX preamble code */                     \
		9,                  /* RX preamble code */                     \
		3,                  /* SFD type: 4z 8-symbol */                \
		DWT_BR_850K,        /* data rate */                            \
		DWT_PHRMODE_STD,                                               \
		DWT_PHRRATE_STD,                                               \
		(1025 + 8 - 32),    /* SFD timeout: PLEN + 1 + SFD - PAC */    \
		DWT_STS_MODE_OFF,                                              \
		DWT_STS_LEN_64,     /* ignored while STS is off */             \
		DWT_PDOA_M0                                                    \
	}

/* PG delay and TX power for channel 5. */
#define UWB_PHY_TXCONFIG_INITIALIZER                                           \
	{                                                                      \
		0x34,       /* PG delay */                                     \
		0xffffffff, /* TX power */                                     \
		0x0         /* PG count */                                     \
	}

#endif /* UWB_PHY_H */
```

- [ ] **Step 2: Write the radio header**

Create `src/uwb_radio.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW3220 bring-up, shared by every anchor mode.
 */

#ifndef UWB_RADIO_H
#define UWB_RADIO_H

#include "uwb_config.h"

/* Reset, probe, initialise and configure the DW3220 with the fixed PHY and
 * the antenna delays from cfg, then arm the IRQ line.
 *
 * Returns 0 on success, or a negative errno. On failure the transceiver is
 * left in whatever state it reached; the caller must not attempt any UWB
 * traffic, but the rest of the system (notably the shell) stays alive.
 */
int uwb_radio_init(const uwb_config_t *cfg);

#endif /* UWB_RADIO_H */
```

- [ ] **Step 3: Write the implementation**

Create `src/uwb_radio.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW3220 bring-up: reset, probe, initialise, configure the fixed PHY, enable
 * CIA diagnostics, apply antenna calibration and arm the IRQ.
 */

#include "uwb_radio.h"
#include "uwb_phy.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>

LOG_MODULE_REGISTER(uwb_radio, LOG_LEVEL_INF);

static dwt_config_t   phy_cfg   = UWB_PHY_CONFIG_INITIALIZER;
static dwt_txconfig_t tx_cfg    = UWB_PHY_TXCONFIG_INITIALIZER;

int uwb_radio_init(const uwb_config_t *cfg)
{
	int ret;

	ret = dw3000_hw_init();
	if (ret) {
		LOG_ERR("dw3000_hw_init failed (%d)", ret);
		return ret;
	}

	dw3000_hw_reset();

	/* The DW3220 needs a moment after RSTn is released before it answers SPI. */
	k_msleep(2);

	/* dwt_probe() reads DEV_ID over raw SPI, bypassing the register-access
	 * layer, so success here already proves the bus, CS and reset polarity
	 * are good. */
	ret = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	if (ret) {
		LOG_ERR("dwt_probe failed (%d) — check SPI wiring, CS and reset polarity",
			ret);
		return ret;
	}

	/* dwt_initialise() MUST be the first register access after dwt_probe().
	 * It is what assigns the driver's local-data pointer (dw->priv), and
	 * every register access dereferences that pointer for the SPI-CRC mode.
	 * Calling anything else first — dwt_readdevid(), dwt_checkidlerc(),
	 * dwt_configure() — faults on a NULL dereference. The DW3220 is already
	 * in IDLE_RC by now: dw3000_hw_reset() plus the delay above cover the
	 * INIT_RC -> IDLE_RC time. */
	if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
		LOG_ERR("dwt_initialise failed");
		return -EIO;
	}

	if (!dwt_checkidlerc()) {
		LOG_ERR("device not in IDLE_RC after init");
		return -EIO;
	}

	LOG_INF("DW3220 device ID: 0x%08x", dwt_readdevid());

	if (dwt_configure(&phy_cfg)) {
		LOG_ERR("dwt_configure failed — PHY rejected");
		return -EIO;
	}

	/* Must follow dwt_configure(); without it every diagnostic register
	 * reads zero, and the DISCOVERY response carries CIR power/quality. */
	dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

	dwt_configuretxrf(&tx_cfg);
	dwt_settxantennadelay(cfg->ant_delay_tx);
	dwt_setrxantennadelay(cfg->ant_delay_rx);
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	ret = dw3000_hw_init_interrupt();
	if (ret) {
		LOG_ERR("dw3000_hw_init_interrupt failed (%d)", ret);
		return ret;
	}

	LOG_INF("radio ready (ant_tx=%u ant_rx=%u)", cfg->ant_delay_tx,
		cfg->ant_delay_rx);
	return 0;
}
```

- [ ] **Step 4: Rewrite `main.c` to call it**

Replace the entire contents of `src/main.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor boot: bring the DW3220 up with the active configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "uwb_config.h"
#include "uwb_radio.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	ret = uwb_radio_init(cfg);
	if (ret) {
		LOG_ERR("radio bring-up failed (%d)", ret);
		return ret;
	}

	LOG_INF("DW3220 ready, IRQ armed");
	return 0;
}
```

- [ ] **Step 5: Add the source and the log level**

Modify `CMakeLists.txt` — `target_sources` becomes:

```cmake
target_sources(app PRIVATE
	src/main.c
	src/uwb_config.c
	src/uwb_radio.c
)
```

Modify `prj.conf`, appending:

```
CONFIG_LOG_DEFAULT_LEVEL=3
```

- [ ] **Step 6: Build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: build succeeds with no warnings.

- [ ] **Step 7: Flash and verify on hardware**

```powershell
west flash
west espressif monitor -p COM5
```

Expected log lines, in this order:

```
<inf> uwb_radio: DW3220 device ID: 0xdeca0302
<inf> uwb_radio: radio ready (ant_tx=16385 ant_rx=16385)
<inf> main: DW3220 ready, IRQ armed
```

The device ID is whatever the part reports (`0xdeca0302` for DW3220); what matters is that it is not `0x00000000` or `0xffffffff`. If `dwt_probe failed` appears, the extraction reordered something — compare against the sequence in `git show HEAD~1:src/main.c`.

**This is the gate for the whole task.** `dwt_configure()` is new here (the old smoke test never called it), so a failure at that line means the PHY initializer is wrong, not that the extraction broke.

- [ ] **Step 8: Commit**

```powershell
git add src/uwb_phy.h src/uwb_radio.h src/uwb_radio.c src/main.c CMakeLists.txt prj.conf
git commit -m "refactor(radio): extract DW3220 bring-up and apply the fixed PHY"
```

---

### Task 3: `uwb_store` — NVS persistence and mode dispatch

**Files:**
- Create: `src/uwb_store.h`
- Create: `src/uwb_store.c`
- Create: `src/uwb_modes.h`
- Create: `src/uwb_slave.c`
- Create: `src/uwb_gateway.c`
- Modify: `src/main.c`
- Modify: `CMakeLists.txt`
- Modify: `prj.conf`

**Interfaces:**
- Consumes: `uwb_config_t`, `uwb_config_get()`, `uwb_config_set_*()`, `uwb_config_mode_name()` from Task 1; `uwb_radio_init()` from Task 2.
- Produces: `int uwb_store_init(void)`, `void uwb_store_save_mode(void)`, `void uwb_store_save_id(void)`, `void uwb_store_save_ant(void)`, `void uwb_store_save_pos(void)` — Task 4's shell calls the save functions. Also `void uwb_slave_run(const uwb_config_t *cfg)` and `void uwb_gateway_run(const uwb_config_t *cfg)`, which specs C and D replace.

- [ ] **Step 1: Enable flash, NVS and settings**

Modify `prj.conf`, appending:

```
# Persistent per-anchor configuration in NVS on storage_partition.
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y
CONFIG_SETTINGS=y
CONFIG_SETTINGS_NVS=y

# %f in the config banner and in `anchor show`.
CONFIG_CBPRINTF_FP_SUPPORT=y

# The main thread runs the mode loop.
CONFIG_MAIN_STACK_SIZE=4096
```

- [ ] **Step 2: Write the store header**

Create `src/uwb_store.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Persistence for the anchor config, via Zephyr settings over NVS on the
 * board's storage_partition.
 */

#ifndef UWB_STORE_H
#define UWB_STORE_H

/* Initialise the settings subsystem and load every stored field into the
 * active config. Returns 0 on success, or a negative errno — on failure the
 * active config keeps its defaults and the caller should carry on booting. */
int uwb_store_init(void);

/* Persist one field of the active config. Each logs its own failure; there is
 * nothing useful for a caller to do about a failed NVS write. */
void uwb_store_save_mode(void);
void uwb_store_save_id(void);
void uwb_store_save_ant(void);
void uwb_store_save_pos(void);

#endif /* UWB_STORE_H */
```

- [ ] **Step 3: Write the store implementation**

Create `src/uwb_store.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-field settings keys under the "anchor" subtree, rather than one struct
 * blob: specs C and D will add fields, and a blob would force a version byte
 * plus a wipe-to-defaults on every layout change. Unknown keys are ignored
 * and their fields keep their defaults.
 *
 * A stored value that fails validation falls back to its default and is
 * logged; the other stored fields are still applied.
 */

#include "uwb_store.h"
#include "uwb_config.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(uwb_store, LOG_LEVEL_INF);

#define KEY_MODE   "anchor/mode"
#define KEY_ID     "anchor/id"
#define KEY_ANT_TX "anchor/ant_tx"
#define KEY_ANT_RX "anchor/ant_rx"
#define KEY_POS    "anchor/pos"

/* Written as one record so x, y, z and the valid flag can never disagree. */
struct stored_pos {
	float   x;
	float   y;
	float   z;
	uint8_t valid;
};

static int read_val(settings_read_cb read_cb, void *cb_arg, void *dst, size_t len)
{
	ssize_t n = read_cb(cb_arg, dst, len);

	if (n != (ssize_t)len) {
		return -EINVAL;
	}
	return 0;
}

static int anchor_settings_set(const char *key, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(len);

	if (strcmp(key, "mode") == 0) {
		uint8_t v;

		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_mode(cfg, v)) {
			LOG_WRN("stored mode %u invalid — keeping %s", v,
				uwb_config_mode_name(cfg->mode));
		}
		return 0;
	}

	if (strcmp(key, "id") == 0) {
		uint8_t v;

		if (read_val(read_cb, cb_arg, &v, sizeof(v))) {
			return -EINVAL;
		}
		if (!uwb_config_set_id(cfg, v)) {
			LOG_WRN("stored id %u invalid — keeping %u", v, cfg->anchor_id);
		}
		return 0;
	}

	if (strcmp(key, "ant_tx") == 0) {
		return read_val(read_cb, cb_arg, &cfg->ant_delay_tx,
				sizeof(cfg->ant_delay_tx));
	}

	if (strcmp(key, "ant_rx") == 0) {
		return read_val(read_cb, cb_arg, &cfg->ant_delay_rx,
				sizeof(cfg->ant_delay_rx));
	}

	if (strcmp(key, "pos") == 0) {
		struct stored_pos p;

		if (read_val(read_cb, cb_arg, &p, sizeof(p))) {
			return -EINVAL;
		}
		if (p.valid) {
			uwb_config_set_pos(cfg, p.x, p.y, p.z);
		}
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(anchor, "anchor", NULL, anchor_settings_set,
			       NULL, NULL);

int uwb_store_init(void)
{
	int ret;

	ret = settings_subsys_init();
	if (ret) {
		LOG_WRN("settings_subsys_init failed (%d) — running on defaults", ret);
		return ret;
	}

	ret = settings_load();
	if (ret) {
		LOG_WRN("settings_load failed (%d) — running on defaults", ret);
		return ret;
	}

	return 0;
}

static void save_one(const char *key, const void *val, size_t len)
{
	int ret = settings_save_one(key, val, len);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", key, ret);
	}
}

void uwb_store_save_mode(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_MODE, &cfg->mode, sizeof(cfg->mode));
}

void uwb_store_save_id(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_ID, &cfg->anchor_id, sizeof(cfg->anchor_id));
}

void uwb_store_save_ant(void)
{
	const uwb_config_t *cfg = uwb_config_get();

	save_one(KEY_ANT_TX, &cfg->ant_delay_tx, sizeof(cfg->ant_delay_tx));
	save_one(KEY_ANT_RX, &cfg->ant_delay_rx, sizeof(cfg->ant_delay_rx));
}

void uwb_store_save_pos(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	struct stored_pos p = {
		.x     = cfg->x,
		.y     = cfg->y,
		.z     = cfg->z,
		.valid = cfg->position_valid ? 1u : 0u,
	};

	save_one(KEY_POS, &p, sizeof(p));
}
```

- [ ] **Step 4: Write the mode seam**

Create `src/uwb_modes.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The two anchor mode entry points. Each takes over the main thread and does
 * not return. Both are stubs until specs C (SLAVE) and D (GATEWAY) land.
 */

#ifndef UWB_MODES_H
#define UWB_MODES_H

#include "uwb_config.h"

void uwb_slave_run(const uwb_config_t *cfg);
void uwb_gateway_run(const uwb_config_t *cfg);

#endif /* UWB_MODES_H */
```

Create `src/uwb_slave.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SLAVE mode: SS-TWR responder that also observes the gateway beacon.
 * Stub — the responder lands in spec C.
 */

#include "uwb_modes.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uwb_slave, LOG_LEVEL_INF);

void uwb_slave_run(const uwb_config_t *cfg)
{
	LOG_INF("SLAVE mode, anchor_id=%u — responder not implemented yet",
		cfg->anchor_id);

	while (1) {
		k_sleep(K_FOREVER);
	}
}
```

Create `src/uwb_gateway.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATEWAY mode: TDMA beacon plus CAP seat granting.
 * Stub — the beacon and gw_core land in spec D.
 */

#include "uwb_modes.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uwb_gateway, LOG_LEVEL_INF);

void uwb_gateway_run(const uwb_config_t *cfg)
{
	if (!cfg->position_valid) {
		LOG_ERR("gateway not positioned — set `anchor pos <x> <y> <z>` first");
	}

	LOG_INF("GATEWAY mode, anchor_id=%u — beacon not implemented yet",
		cfg->anchor_id);

	while (1) {
		k_sleep(K_FOREVER);
	}
}
```

The `position_valid` check logs but does not return: in spec D it becomes a hard refusal to beacon, but there is no beacon to refuse yet, and returning here would drop out of `main()` and lose the mode banner that criterion 4 checks for.

- [ ] **Step 5: Wire boot together**

Replace the entire contents of `src/main.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor boot: load the persisted configuration, bring the DW3220 up with it,
 * and hand the main thread to the configured mode.
 *
 * A radio failure deliberately does not halt the system — the shell stays
 * alive so a mis-set antenna delay or a wiring fault is recoverable over USB
 * without a reflash.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "uwb_config.h"
#include "uwb_modes.h"
#include "uwb_radio.h"
#include "uwb_store.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void log_config(const uwb_config_t *cfg)
{
	LOG_INF("{\"mode\":\"%s\",\"id\":%u,\"ant_tx\":%u,\"ant_rx\":%u,"
		"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		cfg->ant_delay_tx, cfg->ant_delay_rx,
		(double)cfg->x, (double)cfg->y, (double)cfg->z,
		cfg->position_valid ? 1u : 0u);
}

int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	uwb_store_init();
	log_config(cfg);

	ret = uwb_radio_init(cfg);
	if (ret) {
		LOG_ERR("radio bring-up failed (%d) — shell stays up, not entering a mode",
			ret);
		return ret;
	}

	if (cfg->mode == UWB_MODE_GATEWAY) {
		uwb_gateway_run(cfg);
	} else {
		uwb_slave_run(cfg);
	}

	return 0;
}
```

- [ ] **Step 6: Add the sources**

Modify `CMakeLists.txt` — `target_sources` becomes:

```cmake
target_sources(app PRIVATE
	src/main.c
	src/uwb_config.c
	src/uwb_gateway.c
	src/uwb_radio.c
	src/uwb_slave.c
	src/uwb_store.c
)
```

- [ ] **Step 7: Build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: build succeeds with no warnings.

- [ ] **Step 8: Flash and verify on hardware**

```powershell
west flash
west espressif monitor -p COM5
```

Expected, in this order:

```
<inf> main: {"mode":"slave","id":0,"ant_tx":16385,"ant_rx":16385,"x":0.00,"y":0.00,"z":0.00,"pos_valid":0}
<inf> uwb_radio: DW3220 device ID: 0xdeca0302
<inf> uwb_radio: radio ready (ant_tx=16385 ant_rx=16385)
<inf> uwb_slave: SLAVE mode, anchor_id=0 — responder not implemented yet
```

Three things this proves: `settings_subsys_init()` found `storage_partition` and did not fault on an empty NVS; the floats print as `0.00` rather than the `%f`-unsupported placeholder, so `CBPRINTF_FP_SUPPORT` took effect; and the mode dispatch reaches SLAVE.

- [ ] **Step 9: Commit**

```powershell
git add src/uwb_store.h src/uwb_store.c src/uwb_modes.h src/uwb_slave.c src/uwb_gateway.c src/main.c CMakeLists.txt prj.conf
git commit -m "feat(store): persist anchor config in NVS and dispatch on mode at boot"
```

---

### Task 4: `anchor_shell` — the console command tree

**Files:**
- Create: `src/anchor_shell.c`
- Modify: `CMakeLists.txt`
- Modify: `prj.conf`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: everything from Tasks 1 and 3 — `uwb_config_get()`, `uwb_config_set_id()`, `uwb_config_set_mode()`, `uwb_config_set_ant()`, `uwb_config_set_pos()`, `uwb_config_set_defaults()`, `uwb_config_mode_from_name()`, `uwb_config_mode_name()`, `uwb_store_save_mode()`, `uwb_store_save_id()`, `uwb_store_save_ant()`, `uwb_store_save_pos()`.
- Produces: nothing consumed by later code. Specs C and D add their own subcommands to the `anchor` root defined here.

- [ ] **Step 1: Enable the shell**

Modify `prj.conf`, appending:

```
# `anchor` command tree over the native USB-JTAG console.
CONFIG_SHELL=y
CONFIG_SHELL_PROMPT_UART="uwb:~$ "
CONFIG_REBOOT=y
```

`CONFIG_UART_INTERRUPT_DRIVEN` is selected automatically by the serial shell backend, and `serial_esp32_usb.c` implements the interrupt API, so no extra option is needed for USB-JTAG.

- [ ] **Step 2: Write the shell commands**

Create `src/anchor_shell.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `anchor` command tree, over the native USB-JTAG console.
 *
 * Every setter validates, then persists immediately — there is no separate
 * `save` command, so there is no "I set it but forgot to save" failure. The
 * running mode is not disturbed: changes take effect on the next boot, which
 * is what lets the UWB thread treat the config as read-only and skip locking.
 */

#include "uwb_config.h"
#include "uwb_store.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

static void print_config(const struct shell *sh)
{
	const uwb_config_t *cfg = uwb_config_get();

	shell_print(sh,
		    "{\"mode\":\"%s\",\"id\":%u,\"ant_tx\":%u,\"ant_rx\":%u,"
		    "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"pos_valid\":%u}",
		    uwb_config_mode_name(cfg->mode), cfg->anchor_id,
		    cfg->ant_delay_tx, cfg->ant_delay_rx,
		    (double)cfg->x, (double)cfg->y, (double)cfg->z,
		    cfg->position_valid ? 1u : 0u);
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_config(sh);
	return 0;
}

static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	unsigned long v;

	ARG_UNUSED(argc);

	v = strtoul(argv[1], NULL, 0);
	if (v > UINT8_MAX || !uwb_config_set_id(cfg, (uint8_t)v)) {
		shell_error(sh, "error: id must be 0..%u", UWB_MAX_ANCHORS - 1);
		return -EINVAL;
	}

	uwb_store_save_id();
	shell_print(sh, "ok: anchor_id=%u (saved) — reboot to apply", cfg->anchor_id);
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();
	uint8_t mode;

	ARG_UNUSED(argc);

	if (!uwb_config_mode_from_name(argv[1], &mode)) {
		shell_error(sh, "error: mode must be slave or gateway");
		return -EINVAL;
	}

	uwb_config_set_mode(cfg, mode);
	uwb_store_save_mode();
	shell_print(sh, "ok: mode=%s (saved) — reboot to apply",
		    uwb_config_mode_name(cfg->mode));
	return 0;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);

	uwb_config_set_pos(cfg, strtof(argv[1], NULL), strtof(argv[2], NULL),
			   strtof(argv[3], NULL));
	uwb_store_save_pos();
	shell_print(sh, "ok: pos=(%.2f, %.2f, %.2f) (saved) — reboot to apply",
		    (double)cfg->x, (double)cfg->y, (double)cfg->z);
	return 0;
}

static int cmd_ant(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);

	if (!uwb_config_set_ant(cfg, (uint32_t)strtoul(argv[1], NULL, 0),
				(uint32_t)strtoul(argv[2], NULL, 0))) {
		shell_error(sh, "error: antenna delays must be 0..65535");
		return -EINVAL;
	}

	uwb_store_save_ant();
	shell_print(sh, "ok: ant_tx=%u ant_rx=%u (saved) — reboot to apply",
		    cfg->ant_delay_tx, cfg->ant_delay_rx);
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	uwb_config_t *cfg = uwb_config_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uwb_config_set_defaults(cfg);
	uwb_store_save_mode();
	uwb_store_save_id();
	uwb_store_save_ant();
	uwb_store_save_pos();

	shell_print(sh, "ok: defaults restored (saved) — reboot to apply");
	print_config(sh);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_anchor,
	SHELL_CMD_ARG(show,  NULL, "Print the active configuration as JSON",
		      cmd_show,  1, 0),
	SHELL_CMD_ARG(id,    NULL, "id <0..3> — set the ranging id",
		      cmd_id,    2, 0),
	SHELL_CMD_ARG(mode,  NULL, "mode <slave|gateway> — set the boot mode",
		      cmd_mode,  2, 0),
	SHELL_CMD_ARG(pos,   NULL, "pos <x> <y> <z> — set the coordinates in metres",
		      cmd_pos,   4, 0),
	SHELL_CMD_ARG(ant,   NULL, "ant <tx> <rx> — set the antenna delays",
		      cmd_ant,   3, 0),
	SHELL_CMD_ARG(reset, NULL, "Restore the defaults and persist them",
		      cmd_reset, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(anchor, &sub_anchor, "Anchor identity and configuration", NULL);
```

The `SHELL_CMD_ARG` argument counts (`1`, `2`, `4`, `3`, `1` — command name included) mean the shell rejects a wrong-arity invocation before the handler runs, which is why each handler can read `argv[1]` unconditionally.

- [ ] **Step 3: Add the source**

Modify `CMakeLists.txt` — `target_sources` becomes:

```cmake
target_sources(app PRIVATE
	src/anchor_shell.c
	src/main.c
	src/uwb_config.c
	src/uwb_gateway.c
	src/uwb_radio.c
	src/uwb_slave.c
	src/uwb_store.c
)
```

- [ ] **Step 4: Build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: build succeeds with no warnings.

- [ ] **Step 5: Flash and verify the console**

```powershell
west flash
west espressif monitor -p COM5
```

Press Enter. Expected: the `uwb:~$ ` prompt appears. Then:

```
uwb:~$ anchor show
{"mode":"slave","id":0,"ant_tx":16385,"ant_rx":16385,"x":0.00,"y":0.00,"z":0.00,"pos_valid":0}
```

- [ ] **Step 6: Verify validation rejects bad input**

```
uwb:~$ anchor id 4
error: id must be 0..3
uwb:~$ anchor mode master
error: mode must be slave or gateway
uwb:~$ anchor ant 70000 100
error: antenna delays must be 0..65535
uwb:~$ anchor show
```

Expected: the final `anchor show` is byte-identical to Step 5's — no rejected value leaked into the config.

- [ ] **Step 7: Verify persistence across a cold reboot**

```
uwb:~$ anchor id 2
ok: anchor_id=2 (saved) — reboot to apply
uwb:~$ anchor pos 1.5 -2.25 0.75
ok: pos=(1.50, -2.25, 0.75) (saved) — reboot to apply
uwb:~$ kernel reboot cold
```

After the board comes back, the boot banner must read:

```
<inf> main: {"mode":"slave","id":2,"ant_tx":16385,"ant_rx":16385,"x":1.50,"y":-2.25,"z":0.75,"pos_valid":1}
```

**This is the gate for the task** — it is the spec's headline criterion.

- [ ] **Step 8: Verify mode dispatch and restore the defaults**

```
uwb:~$ anchor mode gateway
ok: mode=gateway (saved) — reboot to apply
uwb:~$ kernel reboot cold
```

Expected after reboot: `<inf> uwb_gateway: GATEWAY mode, anchor_id=2 — beacon not implemented yet`, and **not** the `uwb_slave` line. Since `pos_valid` is 1 from Step 7, the `gateway not positioned` error must not appear.

Then restore:

```
uwb:~$ anchor reset
ok: defaults restored (saved) — reboot to apply
{"mode":"slave","id":0,"ant_tx":16385,"ant_rx":16385,"x":0.00,"y":0.00,"z":0.00,"pos_valid":0}
uwb:~$ kernel reboot cold
```

Expected after reboot: the banner shows the defaults again, proving `anchor reset` persisted rather than only clearing RAM.

- [ ] **Step 9: Update the project guide**

Modify `CLAUDE.md`. In the **Layout** section, replace the `src/main.c` bullet with:

```markdown
- `src/main.c` — boot: load config from NVS, bring the DW3220 up, dispatch on
  mode to `uwb_slave_run()` / `uwb_gateway_run()`.
- `src/uwb_config.{c,h}` — per-anchor config (mode, id, antenna delays,
  position). Pure C, host-tested in `tests/uwb_config/`.
- `src/uwb_store.{c,h}` — the above persisted in NVS on `storage_partition`,
  via Zephyr settings, one key per field under `anchor/`.
- `src/uwb_radio.{c,h}` — DW3220 bring-up shared by both modes.
- `src/uwb_phy.h` — the fixed PHY contract. Not runtime-configurable.
- `src/anchor_shell.c` — the `anchor` console command tree.
- `src/uwb_slave.c` / `src/uwb_gateway.c` — mode entry points; stubs until
  specs C and D.
```

Add a new section after **Build & flash** (the outer fence below is four
backticks so the inner three-backtick block survives the copy — write only the
inner content into `CLAUDE.md`):

````markdown
## Console

Native USB-JTAG (not UART0), prompt `uwb:~$ `.

```
anchor show                    active config as JSON
anchor id <0..3>               ranging id (default 0)
anchor mode <slave|gateway>    boot mode (default slave)
anchor pos <x> <y> <z>         coordinates in metres; sets pos_valid
anchor ant <tx> <rx>           antenna delays (default 16385/16385)
anchor reset                   restore defaults
kernel reboot cold             apply — every setter persists immediately,
                               but changes take effect only on reboot
```
````

In **Hard-won facts**, append:

```markdown
- **`dwt_configciadiag()` must be called after `dwt_configure()`**, or every
  diagnostic register reads zero — which breaks the CIR power/quality the
  DISCOVERY response carries.
```

Add a **Host tests** section (again, write only the inner content):

````markdown
## Host tests

Plain gcc, no Zephyr, following the sibling projects' pattern:

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe    # prints PASSED, exits 0
```
````

- [ ] **Step 10: Re-run the host test**

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe
```

Expected: `PASSED`, exit code 0. Task 1's contract must still hold after three tasks of consumers.

- [ ] **Step 11: Commit**

```powershell
git add src/anchor_shell.c CMakeLists.txt prj.conf CLAUDE.md
git commit -m "feat(shell): anchor config command tree over the USB-JTAG console"
```

---

## Done when

All four tasks are committed and the spec's five on-target criteria pass:

1. `west build -b ancla_esp32s3/esp32s3/procpu` — clean, no warnings (Task 4 Step 4)
2. Shell prompt over USB-JTAG; `anchor show` reports `"mode":"slave","id":0` (Task 4 Step 5)
3. `anchor id 2` → cold reboot → `"id":2` (Task 4 Step 7)
4. `anchor mode gateway` → cold reboot → the GATEWAY stub is entered, not SLAVE (Task 4 Step 8)
5. Radio bring-up logs the DEV_ID and a successful `dwt_configure()` (Task 2 Step 7, re-confirmed Task 3 Step 8)

Specs B, C and D then build on this. The seams they plug into are `uwb_slave_run()` / `uwb_gateway_run()` in `src/uwb_modes.h`, `uwb_radio_init()` for radio setup, and the `anchor` shell root for any new subcommands.

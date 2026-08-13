# Gateway WiFi + MQTT Uplink Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish decoded `0xEA` tag position fixes from the gateway to an MQTT broker over WiFi, without disturbing the UWB beacon.

**Architecture:** `pos_sink_publish()` becomes a non-blocking producer onto a bounded `k_msgq`; a dedicated low-priority `net_uplink` thread owns WiFi association, the MQTT connection and publishing. Isolation from the beacon is by thread priority, not by core. Payload formatting and configuration validation are pure C, host-tested; the Zephyr layers around them are thin.

**Tech Stack:** Zephyr 4.4.x, ESP32-S3 native WiFi (`WIFI_ESP32`), Zephyr `MQTT_LIB`, Zephyr `settings`/NVS, Zephyr `shell`.

**Spec:** `docs/superpowers/specs/2026-08-12-gateway-mqtt-uplink-design.md`
**Branch:** `feat/mqtt-uplink`, off `feat/pos-frame` (E1 is not on `master`)

## Global Constraints

- **Build:** `$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"` then `west build -b ancla_esp32s3/esp32s3/procpu`. `ZEPHYR_BASE` is required — this project lives outside the west workspace.
- **`src/uwb_gateway.c` is not modified by this plan.** `pos_sink.h` keeps its exact current signature.
- **`src/uwb_frame_802_15_4z.{c,h}` must stay byte-identical to the tag's copy.** Not touched here.
- **`modules/` must not be modified** — it carries three deliberate local deltas.
- **No `k_malloc` / dynamic allocation** in any new module. Fixed buffers only.
- **Host tests are plain gcc**, no Zephyr, `CHECK` macro, non-zero exit on failure. New pure-C modules must not include any Zephyr header.
- **Zone name is `"Zona"`**, defined once, used by both topics.
- **Topics:** `testtopic/1/position` and `testtopic/1/anchors`, exactly.
- **`tagId` is bare uppercase hex, 4 digits, zero-padded, no `0x` prefix.**
- **Any busy-wait added to the gateway path must be bounded** — after this plan the gateway loop runs cooperatively, so an unbounded spin is unrecoverable.
- Commit after every task. Local git only, no remote, never merge to `master`.

---

## File Structure

| File | Responsibility |
|---|---|
| `src/pos_json.{c,h}` | **New.** Pure C. Formats the position payload and the anchors stub. |
| `src/net_config.{c,h}` | **New.** Pure C. Network settings struct + validating setters. |
| `src/net_store.{c,h}` | **New.** `net/` settings subtree persistence. |
| `src/net_shell.c` | **New.** The `net` console command tree. |
| `src/net_uplink.{c,h}` | **New.** The uplink thread: WiFi, MQTT, the fix queue. |
| `src/pos_sink.c` | **Modified.** Keeps its log line, adds a non-blocking enqueue. |
| `src/main.c` | **Modified.** Explicit config init, priority promotion, thread start. |
| `prj.conf` | **Modified.** Networking, MQTT, and the traffic-class priority fix. |
| `CMakeLists.txt` | **Modified.** New sources. |
| `tests/pos_json/`, `tests/net_config/` | **New.** Host tests. |

---

## Task 1: Position and anchor JSON formatting

Pure C, no Zephyr, no sockets. This is where the wire contract with the existing Python consumer is pinned down.

**Files:**
- Create: `src/pos_json.h`, `src/pos_json.c`
- Create: `tests/pos_json/test_pos_json.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `struct pos_fix` from `src/pos_sink.h` (already `stdint`-only).
- Produces:
  - `#define POS_JSON_ZONE_NAME "Zona"`
  - `#define POS_JSON_MAX_LEN 512`
  - `int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix);`
  - `int pos_json_anchors(char *buf, size_t len);`
  - Both return bytes written excluding NUL, or `-1` on truncation.

- [ ] **Step 1: Write the failing test**

Create `tests/pos_json/test_pos_json.c`:

```c
#include "../../src/pos_json.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_fix_exact_contract(void)
{
    /* The byte-for-byte contract with the existing Python consumer. */
    struct pos_fix f = { .src_addr = 0x1234, .x = 1.23f, .y = 4.56f,
                         .residual_m = 0.12f, .n_anchors = 3, .batt_soc = 87 };
    char buf[POS_JSON_MAX_LEN];
    int n = pos_json_fix(buf, sizeof(buf), &f);

    CHECK(n > 0);
    CHECK(strcmp(buf,
        "{\"tagId\":\"1234\",\"x\":1.23,\"y\":4.56,\"z\":0,\"zoneName\":\"Zona\"}") == 0);
    CHECK(n == (int)strlen(buf));
}

static void test_fix_drops_diagnostics(void)
{
    /* residual, n_anchors and batt_soc must NOT reach the payload -- they stay
     * on the console log line. Changing this breaks the consumer contract. */
    struct pos_fix f = { .src_addr = 0x0001, .x = 0.0f, .y = 0.0f,
                         .residual_m = 9.99f, .n_anchors = 4, .batt_soc = 42 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "residual") == NULL);
    CHECK(strstr(buf, "anchors")  == NULL);
    CHECK(strstr(buf, "battery")  == NULL);
    CHECK(strstr(buf, "42")       == NULL);
}

static void test_tag_id_is_zero_padded_uppercase_hex(void)
{
    /* 0x00AB must be "00AB", not "AB" and not "171". */
    struct pos_fix f = { .src_addr = 0x00AB, .x = 0.0f, .y = 0.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"tagId\":\"00AB\"") != NULL);

    f.src_addr = 0xBEEF;
    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"tagId\":\"BEEF\"") != NULL);
}

static void test_z_is_an_integer_literal(void)
{
    /* The solver is 2D. The consumer expects 0, not 0.00. */
    struct pos_fix f = { .src_addr = 0x0001, .x = 1.0f, .y = 2.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"z\":0,") != NULL);
    CHECK(strstr(buf, "\"z\":0.00") == NULL);
}

static void test_negative_and_large_coordinates(void)
{
    struct pos_fix f = { .src_addr = 0x0002, .x = -12.345f, .y = 1234.5f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    /* %.2f rounds half-away-from-zero here: -12.345 -> -12.35 or -12.34 is
     * platform-dependent in the last digit, so assert the stable prefix. */
    CHECK(strstr(buf, "\"x\":-12.3") != NULL);
    CHECK(strstr(buf, "\"y\":1234.50") != NULL);
}

static void test_fix_truncation_is_reported(void)
{
    /* A partial JSON document must never be published. */
    struct pos_fix f = { .src_addr = 0x1234, .x = 1.23f, .y = 4.56f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char small[16];

    CHECK(pos_json_fix(small, sizeof(small), &f) == -1);
}

static void test_anchors_stub(void)
{
    char buf[POS_JSON_MAX_LEN];
    int n = pos_json_anchors(buf, sizeof(buf));

    CHECK(n > 0);
    CHECK(n == (int)strlen(buf));
    /* Same zone identifier as the position payload -- both topics must agree. */
    CHECK(strstr(buf, "\"name\":\"Zona\"") != NULL);
    CHECK(strstr(buf, "\"anchors\":[") != NULL);
    CHECK(strstr(buf, "\"isAxis\":true") != NULL);
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
    CHECK(strstr(buf, "\"latitude\":0.0") != NULL);
    CHECK(strstr(buf, "\"longitude\":0.0") != NULL);
    CHECK(buf[n - 1] == '}');
}

static void test_anchors_fits_in_max_len(void)
{
    /* POS_JSON_MAX_LEN must be large enough for the bigger of the two
     * documents, or the uplink buffer is undersized. */
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_anchors(buf, sizeof(buf)) > 0);
}

static void test_anchors_truncation_is_reported(void)
{
    char small[32];

    CHECK(pos_json_anchors(small, sizeof(small)) == -1);
}

int main(void)
{
    test_fix_exact_contract();
    test_fix_drops_diagnostics();
    test_tag_id_is_zero_padded_uppercase_hex();
    test_z_is_an_integer_literal();
    test_negative_and_large_coordinates();
    test_fix_truncation_is_reported();
    test_anchors_stub();
    test_anchors_fits_in_max_len();
    test_anchors_truncation_is_reported();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c
```

Expected: FAIL — `src/pos_json.c: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `src/pos_json.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MQTT payload formatting. Pure C with no Zephyr dependency so the wire
 * contract is host-testable; the transport lives in net_uplink.c.
 *
 * The position payload is a fixed contract with an existing consumer. Do not
 * add, remove or rename fields without changing that consumer too.
 */

#ifndef POS_JSON_H
#define POS_JSON_H

#include <stddef.h>

#include "pos_sink.h"

/* Zone identifier. Published as "zoneName" on the position topic and as "name"
 * on the anchor topic -- one define so the two can never disagree. */
#define POS_JSON_ZONE_NAME "Zona"

/* Buffer size that fits either document plus its NUL. The anchors stub is the
 * larger of the two; tests/pos_json/ asserts it still fits. */
#define POS_JSON_MAX_LEN 512

/* Format one fix as the position payload:
 *   {"tagId":"1234","x":1.23,"y":4.56,"z":0,"zoneName":"Zona"}
 *
 * Returns the number of bytes written excluding the NUL, or -1 if the buffer
 * was too small. On -1 the caller MUST drop the message: publishing a
 * truncated JSON document is worse than publishing nothing. */
int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix);

/* Format the stubbed zone/anchor map. Same return contract.
 * Placeholder coordinates until the anchor survey work lands. */
int pos_json_anchors(char *buf, size_t len);

#endif /* POS_JSON_H */
```

- [ ] **Step 4: Write the implementation**

Create `src/pos_json.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos_json.h"

#include <stdio.h>

int pos_json_fix(char *buf, size_t len, const struct pos_fix *fix)
{
	int n;

	/* tagId is BARE UPPERCASE HEX: four digits, zero-padded, no "0x" prefix.
	 * 0x00AB formats as "00AB". This matches an existing Python consumer, so
	 * it is not free to change. Note it reads as decimal to a human --
	 * "1234" is 0x1234 = 4660, not one thousand two hundred thirty-four.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and the
	 * consumer expects 0.
	 *
	 * residual_m, n_anchors and batt_soc are deliberately absent. They stay
	 * on pos_sink.c's console log line. */
	n = snprintf(buf, len,
		     "{\"tagId\":\"%04X\",\"x\":%.2f,\"y\":%.2f,\"z\":0,"
		     "\"zoneName\":\"" POS_JSON_ZONE_NAME "\"}",
		     (unsigned int)fix->src_addr,
		     (double)fix->x, (double)fix->y);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}

int pos_json_anchors(char *buf, size_t len)
{
	/* Stub: the real anchor positions need a survey step that does not exist
	 * yet. The schema is final; only the numbers are placeholders, so a
	 * downstream consumer written against this stays valid. */
	static const char doc[] =
		"{\"name\":\"" POS_JSON_ZONE_NAME "\",\"anchors\":["
		"{\"name\":\"A0\",\"isAxis\":true,\"isReferenceAxis\":true,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"A1\",\"isAxis\":true,\"isReferenceAxis\":false,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"A2\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0.0,\"longitude\":0.0,\"x\":0.0,\"y\":0.0,\"z\":0.0}"
		"]}";
	int n = snprintf(buf, len, "%s", doc);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c
./tests/pos_json/test_pos_json.exe
```

Expected: `PASSED`, exit 0. If `test_anchors_fits_in_max_len` fails, raise `POS_JSON_MAX_LEN` rather than shrinking the document.

- [ ] **Step 6: Add the source to the build**

In `CMakeLists.txt`, add to `target_sources(app PRIVATE ...)`, keeping the list alphabetical:

```cmake
	src/pos_json.c
```

- [ ] **Step 7: Verify the firmware still builds**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean. `pos_json.c` is compiled but not yet called.

- [ ] **Step 8: Commit**

```bash
git add src/pos_json.c src/pos_json.h tests/pos_json/test_pos_json.c CMakeLists.txt
git commit -m "feat(json): position and anchor payload formatting

Pure C, host-tested. Pins the byte-for-byte contract with the existing
Python consumer: bare uppercase hex tagId, integer z, and no diagnostic
fields."
```

---

## Task 2: Network configuration

Pure C validating config, mirroring `uwb_config.{c,h}`. **Unlike `uwb_config_get()`, this singleton has no lazy initialiser** — `net_uplink` is a second thread, and CLAUDE.md records that the lazy pattern is only safe because a human cannot type fast enough.

**Files:**
- Create: `src/net_config.h`, `src/net_config.c`
- Create: `tests/net_config/test_net_config.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `net_config_t` with fields `ssid`, `psk`, `broker`, `port`, `mqtt_user`, `mqtt_pass`
  - `void net_config_set_defaults(net_config_t *c);`
  - `void net_config_init(void);`
  - `net_config_t *net_config_get(void);`
  - `bool net_config_set_ssid(net_config_t *c, const char *ssid);`
  - `bool net_config_set_psk(net_config_t *c, const char *psk);`
  - `bool net_config_set_broker(net_config_t *c, const char *host, uint32_t port);`
  - `bool net_config_set_user(net_config_t *c, const char *user);`
  - `bool net_config_set_mqtt_pass(net_config_t *c, const char *pass);`
  - `bool net_config_is_provisioned(const net_config_t *c);`

- [ ] **Step 1: Write the failing test**

Create `tests/net_config/test_net_config.c`:

```c
#include "../../src/net_config.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void fill(char *buf, size_t n, char ch)
{
    memset(buf, ch, n);
    buf[n] = '\0';
}

static void test_defaults(void)
{
    net_config_t c;
    net_config_set_defaults(&c);
    CHECK(c.ssid[0] == '\0');
    CHECK(c.psk[0] == '\0');
    CHECK(c.broker[0] == '\0');
    CHECK(c.port == NET_MQTT_PORT_DEFAULT);
    CHECK(c.mqtt_user[0] == '\0');
    CHECK(c.mqtt_pass[0] == '\0');
    /* Defaults alone are not enough to attempt an uplink. */
    CHECK(!net_config_is_provisioned(&c));
}

static void test_ssid_bounds(void)
{
    net_config_t c;
    char buf[NET_SSID_MAX + 8];
    net_config_set_defaults(&c);

    CHECK(net_config_set_ssid(&c, "MyNetwork"));
    CHECK(strcmp(c.ssid, "MyNetwork") == 0);

    fill(buf, NET_SSID_MAX, 'a');
    CHECK(net_config_set_ssid(&c, buf));
    CHECK(strlen(c.ssid) == NET_SSID_MAX);

    /* Too long and empty are both rejected, leaving the previous value. */
    fill(buf, NET_SSID_MAX + 1, 'b');
    CHECK(!net_config_set_ssid(&c, buf));
    CHECK(strlen(c.ssid) == NET_SSID_MAX);
    CHECK(c.ssid[0] == 'a');

    CHECK(!net_config_set_ssid(&c, ""));
    CHECK(c.ssid[0] == 'a');
}

static void test_psk_bounds(void)
{
    net_config_t c;
    char buf[NET_PSK_MAX + 8];
    net_config_set_defaults(&c);

    /* WPA2 passphrase is 8..63 characters. */
    CHECK(net_config_set_psk(&c, "hunter22"));
    CHECK(strcmp(c.psk, "hunter22") == 0);

    CHECK(!net_config_set_psk(&c, "short7c"));
    CHECK(strcmp(c.psk, "hunter22") == 0);

    fill(buf, NET_PSK_MAX, 'x');
    CHECK(net_config_set_psk(&c, buf));
    CHECK(strlen(c.psk) == NET_PSK_MAX);

    fill(buf, NET_PSK_MAX + 1, 'y');
    CHECK(!net_config_set_psk(&c, buf));
    CHECK(strlen(c.psk) == NET_PSK_MAX);
    CHECK(c.psk[0] == 'x');
}

static void test_broker_and_port(void)
{
    net_config_t c;
    char buf[NET_BROKER_MAX + 8];
    net_config_set_defaults(&c);

    CHECK(net_config_set_broker(&c, "10.0.0.5", 1883));
    CHECK(strcmp(c.broker, "10.0.0.5") == 0);
    CHECK(c.port == 1883);

    CHECK(net_config_set_broker(&c, "broker.example.com", 65535));
    CHECK(c.port == 65535);

    /* Port 0 and anything above 65535 are rejected, atomically with the host. */
    CHECK(!net_config_set_broker(&c, "other.example.com", 0));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);
    CHECK(c.port == 65535);

    CHECK(!net_config_set_broker(&c, "other.example.com", 65536));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);

    CHECK(!net_config_set_broker(&c, "", 1883));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);

    fill(buf, NET_BROKER_MAX + 1, 'h');
    CHECK(!net_config_set_broker(&c, buf, 1883));
    CHECK(strcmp(c.broker, "broker.example.com") == 0);
}

static void test_mqtt_credentials_may_be_empty(void)
{
    net_config_t c;
    char buf[NET_MQTT_PASS_MAX + 8];
    net_config_set_defaults(&c);

    /* An anonymous broker is legitimate, so empty is accepted here -- unlike
     * the SSID, where empty means "unprovisioned". */
    CHECK(net_config_set_user(&c, ""));
    CHECK(c.mqtt_user[0] == '\0');
    CHECK(net_config_set_mqtt_pass(&c, ""));
    CHECK(c.mqtt_pass[0] == '\0');

    CHECK(net_config_set_user(&c, "gateway1"));
    CHECK(strcmp(c.mqtt_user, "gateway1") == 0);
    CHECK(net_config_set_mqtt_pass(&c, "s3cret"));
    CHECK(strcmp(c.mqtt_pass, "s3cret") == 0);

    fill(buf, NET_MQTT_USER_MAX + 1, 'u');
    CHECK(!net_config_set_user(&c, buf));
    CHECK(strcmp(c.mqtt_user, "gateway1") == 0);

    fill(buf, NET_MQTT_PASS_MAX + 1, 'p');
    CHECK(!net_config_set_mqtt_pass(&c, buf));
    CHECK(strcmp(c.mqtt_pass, "s3cret") == 0);
}

static void test_is_provisioned(void)
{
    net_config_t c;
    net_config_set_defaults(&c);

    CHECK(!net_config_is_provisioned(&c));
    CHECK(net_config_set_ssid(&c, "MyNetwork"));
    /* SSID alone is not enough -- there is nowhere to publish. */
    CHECK(!net_config_is_provisioned(&c));
    CHECK(net_config_set_broker(&c, "10.0.0.5", 1883));
    CHECK(net_config_is_provisioned(&c));
}

static void test_singleton_starts_at_defaults(void)
{
    net_config_t *c;

    net_config_init();
    c = net_config_get();
    CHECK(c != NULL);
    CHECK(c->ssid[0] == '\0');
    CHECK(c->port == NET_MQTT_PORT_DEFAULT);
}

int main(void)
{
    test_defaults();
    test_ssid_bounds();
    test_psk_bounds();
    test_broker_and_port();
    test_mqtt_credentials_may_be_empty();
    test_is_provisioned();
    test_singleton_starts_at_defaults();
    printf(g_fail ? "FAILED (%d)\n" : "PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/net_config/test_net_config.exe tests/net_config/test_net_config.c src/net_config.c
```

Expected: FAIL — `src/net_config.c: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `src/net_config.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gateway network configuration: WiFi credentials and MQTT broker details.
 * Pure C with no Zephyr dependency so the validation is host-testable; the
 * persistence lives in net_store.c and the console in net_shell.c.
 *
 * Two passwords live here and they must never be confused: `psk` is the WiFi
 * WPA2 passphrase, `mqtt_pass` is the MQTT password. The console commands
 * (`net pass` / `net mqttpass`), the NVS keys and the JSON keys all agree with
 * these field names.
 */

#ifndef NET_CONFIG_H
#define NET_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Maximum useful lengths, all excluding the NUL terminator. */
#define NET_SSID_MAX      32  /* 802.11 SSID                    */
#define NET_PSK_MAX       63  /* WPA2 passphrase; minimum is 8  */
#define NET_PSK_MIN        8
#define NET_BROKER_MAX    63  /* hostname or IPv4 literal       */
#define NET_MQTT_USER_MAX 32
#define NET_MQTT_PASS_MAX 63

#define NET_MQTT_PORT_DEFAULT 1883u

typedef struct {
	char     ssid[NET_SSID_MAX + 1];
	char     psk[NET_PSK_MAX + 1];
	char     broker[NET_BROKER_MAX + 1];
	uint16_t port;
	char     mqtt_user[NET_MQTT_USER_MAX + 1];
	char     mqtt_pass[NET_MQTT_PASS_MAX + 1];
} net_config_t;

/* Overwrite *c with the documented defaults: everything empty, port 1883. */
void net_config_set_defaults(net_config_t *c);

/* Initialise the singleton to defaults.
 *
 * MUST be called from main() before anything else can reach net_config_get().
 * Unlike uwb_config_get(), this has no lazy first-caller-wins initialiser:
 * net_uplink is a second thread, which breaks the assumption that makes the
 * lazy pattern safe over there (see CLAUDE.md). */
void net_config_init(void);

/* The single active instance. Undefined before net_config_init(). */
net_config_t *net_config_get(void);

/* Validating setters. Each returns true and mutates on success, or returns
 * false and leaves *c completely untouched on a rejected value. */
bool net_config_set_ssid(net_config_t *c, const char *ssid);
bool net_config_set_psk(net_config_t *c, const char *psk);
bool net_config_set_broker(net_config_t *c, const char *host, uint32_t port);
bool net_config_set_user(net_config_t *c, const char *user);
bool net_config_set_mqtt_pass(net_config_t *c, const char *pass);

/* True when there is enough configuration to attempt an uplink at all: an
 * SSID to join and a broker to publish to. MQTT credentials may legitimately
 * be empty (anonymous broker), so they are not required here. */
bool net_config_is_provisioned(const net_config_t *c);

#endif /* NET_CONFIG_H */
```

- [ ] **Step 4: Write the implementation**

Create `src/net_config.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_config.h"

#include <string.h>

static net_config_t g_cfg;

/* Copy `src` into a field of capacity `cap` (excluding NUL) if its length is
 * within [min_len, cap]. Returns false without touching `dst` otherwise, which
 * is what gives every setter its all-or-nothing contract. */
static bool set_str(char *dst, size_t cap, const char *src, size_t min_len)
{
	size_t n;

	if (src == NULL) {
		return false;
	}

	n = strlen(src);
	if (n < min_len || n > cap) {
		return false;
	}

	memcpy(dst, src, n);
	dst[n] = '\0';
	return true;
}

void net_config_set_defaults(net_config_t *c)
{
	memset(c, 0, sizeof(*c));
	c->port = NET_MQTT_PORT_DEFAULT;
}

void net_config_init(void)
{
	net_config_set_defaults(&g_cfg);
}

net_config_t *net_config_get(void)
{
	return &g_cfg;
}

bool net_config_set_ssid(net_config_t *c, const char *ssid)
{
	return set_str(c->ssid, NET_SSID_MAX, ssid, 1);
}

bool net_config_set_psk(net_config_t *c, const char *psk)
{
	return set_str(c->psk, NET_PSK_MAX, psk, NET_PSK_MIN);
}

bool net_config_set_broker(net_config_t *c, const char *host, uint32_t port)
{
	char scratch[NET_BROKER_MAX + 1];

	/* Validate both fields before writing either: a rejected port must not
	 * leave a half-applied broker behind. */
	if (port == 0u || port > 65535u) {
		return false;
	}
	if (!set_str(scratch, NET_BROKER_MAX, host, 1)) {
		return false;
	}

	memcpy(c->broker, scratch, sizeof(scratch));
	c->port = (uint16_t)port;
	return true;
}

bool net_config_set_user(net_config_t *c, const char *user)
{
	return set_str(c->mqtt_user, NET_MQTT_USER_MAX, user, 0);
}

bool net_config_set_mqtt_pass(net_config_t *c, const char *pass)
{
	return set_str(c->mqtt_pass, NET_MQTT_PASS_MAX, pass, 0);
}

bool net_config_is_provisioned(const net_config_t *c)
{
	return c->ssid[0] != '\0' && c->broker[0] != '\0' && c->port != 0u;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/net_config/test_net_config.exe tests/net_config/test_net_config.c src/net_config.c
./tests/net_config/test_net_config.exe
```

Expected: `PASSED`, exit 0.

- [ ] **Step 6: Add the source to the build**

In `CMakeLists.txt`, add `src/net_config.c` to `target_sources`, alphabetically.

- [ ] **Step 7: Verify the firmware still builds**

```powershell
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds clean.

- [ ] **Step 8: Commit**

```bash
git add src/net_config.c src/net_config.h tests/net_config/test_net_config.c CMakeLists.txt
git commit -m "feat(net): host-tested network configuration

Explicit init rather than uwb_config's lazy singleton -- net_uplink is a
second thread, which breaks the assumption that makes lazy init safe."
```

---

## Task 3: Persistence and the `net` console tree

Deliverable: `net` commands set values that survive a reboot, and `net show` never prints a secret.

**Files:**
- Create: `src/net_store.h`, `src/net_store.c`, `src/net_shell.c`
- Modify: `src/main.c`, `CMakeLists.txt`

**Interfaces:**
- Consumes: everything from `src/net_config.h` (Task 2).
- Produces:
  - `int net_store_save_ssid(void);`
  - `int net_store_save_psk(void);`
  - `int net_store_save_broker(void);` — writes both host and port
  - `int net_store_save_user(void);`
  - `int net_store_save_mqtt_pass(void);`
  - `int net_store_save_all(void);` — used by `net reset`
  - Registers a `SETTINGS_STATIC_HANDLER_DEFINE` on the `net` subtree, loaded by the existing `settings_load()` inside `uwb_store_init()`.

- [ ] **Step 1: Write the settings handler**

Create `src/net_store.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Network config persisted in NVS under the "net/" settings subtree, one key
 * per field, mirroring uwb_store.c's per-field style for the same reason: a
 * struct blob would force a version byte and a wipe on every layout change.
 *
 * There is no init function. The handler is loaded by the settings_load()
 * that uwb_store_init() already performs -- but net_config_init() must have
 * run before that, or the handler writes into an uninitialised singleton.
 */

#ifndef NET_STORE_H
#define NET_STORE_H

/* Persist one field of the active network config. Each logs its own failure
 * and returns 0 or a negative errno, so the shell can tell the operator that
 * a value took effect in RAM but will not survive a reboot. */
int net_store_save_ssid(void);
int net_store_save_psk(void);
int net_store_save_broker(void);
int net_store_save_user(void);
int net_store_save_mqtt_pass(void);

/* Persist every field. Used by `net reset`; returns the first failure's
 * errno, having still attempted all of the writes. */
int net_store_save_all(void);

#endif /* NET_STORE_H */
```

Create `src/net_store.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_store.h"
#include "net_config.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(net_store, LOG_LEVEL_INF);

#define KEY_SSID     "net/ssid"
#define KEY_PSK      "net/psk"
#define KEY_BROKER   "net/broker"
#define KEY_PORT     "net/port"
#define KEY_USER     "net/user"
#define KEY_MQTTPASS "net/mqttpass"

/* Read a stored NUL-terminated string into `dst` via the validating setter
 * `apply`. Rejects anything that does not fit the scratch buffer or that the
 * setter refuses, leaving the field at its default. */
static int load_str(settings_read_cb read_cb, void *cb_arg, size_t len,
		    const char *name,
		    bool (*apply)(net_config_t *, const char *))
{
	char scratch[NET_PSK_MAX + 2]; /* the longest field plus NUL */
	ssize_t n;

	if (len == 0u || len > sizeof(scratch)) {
		LOG_WRN("stored %s size %u invalid — keeping the default",
			name, (unsigned int)len);
		return -EINVAL;
	}

	n = read_cb(cb_arg, scratch, len);
	if (n != (ssize_t)len) {
		return -EINVAL;
	}

	scratch[len - 1] = '\0'; /* defend against a stored value with no NUL */

	if (!apply(net_config_get(), scratch)) {
		LOG_WRN("stored %s rejected by validation — keeping the default",
			name);
	}
	return 0;
}

static bool apply_broker_host(net_config_t *c, const char *host)
{
	/* The port is a separate key and may not have been loaded yet, so reuse
	 * whatever port the config currently holds (the 1883 default at worst).
	 * Settings keys arrive in an unspecified order, so neither key may
	 * depend on the other having been seen. */
	return net_config_set_broker(c, host, c->port);
}

static int net_settings_set(const char *key, size_t len,
			    settings_read_cb read_cb, void *cb_arg)
{
	net_config_t *cfg = net_config_get();

	if (strcmp(key, "ssid") == 0) {
		return load_str(read_cb, cb_arg, len, "ssid", net_config_set_ssid);
	}
	if (strcmp(key, "psk") == 0) {
		return load_str(read_cb, cb_arg, len, "psk", net_config_set_psk);
	}
	if (strcmp(key, "broker") == 0) {
		return load_str(read_cb, cb_arg, len, "broker", apply_broker_host);
	}
	if (strcmp(key, "user") == 0) {
		return load_str(read_cb, cb_arg, len, "user", net_config_set_user);
	}
	if (strcmp(key, "mqttpass") == 0) {
		return load_str(read_cb, cb_arg, len, "mqttpass",
				net_config_set_mqtt_pass);
	}

	if (strcmp(key, "port") == 0) {
		uint16_t v;

		if (len != sizeof(v)) {
			LOG_WRN("stored port size %u invalid — keeping %u",
				(unsigned int)len, cfg->port);
			return -EINVAL;
		}
		if (read_cb(cb_arg, &v, sizeof(v)) != (ssize_t)sizeof(v)) {
			return -EINVAL;
		}
		/* Validated inline rather than through net_config_set_broker():
		 * that setter requires a non-empty host, and settings keys
		 * arrive in an unspecified order, so the broker key may not have
		 * been seen yet. The port range is the whole rule here. */
		if (v == 0u) {
			LOG_WRN("stored port 0 invalid — keeping %u", cfg->port);
			return 0;
		}
		cfg->port = v;
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(net, "net", NULL, net_settings_set, NULL, NULL);

static int save_one(const char *key, const void *val, size_t len)
{
	int ret = settings_save_one(key, val, len);

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", key, ret);
	}
	return ret;
}

int net_store_save_ssid(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_SSID, c->ssid, strlen(c->ssid) + 1u);
}

int net_store_save_psk(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_PSK, c->psk, strlen(c->psk) + 1u);
}

int net_store_save_broker(void)
{
	const net_config_t *c = net_config_get();
	int first, ret;

	/* Two keys, both always attempted; report the first failure. */
	first = save_one(KEY_BROKER, c->broker, strlen(c->broker) + 1u);
	ret = save_one(KEY_PORT, &c->port, sizeof(c->port));

	return first ? first : ret;
}

int net_store_save_user(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_USER, c->mqtt_user, strlen(c->mqtt_user) + 1u);
}

int net_store_save_mqtt_pass(void)
{
	const net_config_t *c = net_config_get();

	return save_one(KEY_MQTTPASS, c->mqtt_pass, strlen(c->mqtt_pass) + 1u);
}

int net_store_save_all(void)
{
	int first = 0, ret;

	ret = net_store_save_ssid();
	if (ret && !first) { first = ret; }
	ret = net_store_save_psk();
	if (ret && !first) { first = ret; }
	ret = net_store_save_broker();
	if (ret && !first) { first = ret; }
	ret = net_store_save_user();
	if (ret && !first) { first = ret; }
	ret = net_store_save_mqtt_pass();
	if (ret && !first) { first = ret; }

	return first;
}
```

- [ ] **Step 2: Write the console tree**

Create `src/net_shell.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `net` command tree. Same contract as `anchor`: every setter validates,
 * then persists immediately, and changes take effect on the next boot.
 *
 * Secrets are never echoed. `net show` prints <set>/<unset> for the WiFi PSK
 * and the MQTT password, because the console output is exactly what ends up
 * pasted into a bug report.
 */

#include "net_config.h"
#include "net_store.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

/* strtoul() returns 0 on non-numeric input without reporting it, and the range
 * check alone would not catch a typo, so endptr does. Same helper shape as
 * anchor_shell.c's parse_ul(). */
static bool parse_ul(const char *arg, unsigned long *out)
{
	char *endptr;
	unsigned long v = strtoul(arg, &endptr, 0);

	if (endptr == arg || *endptr != '\0') {
		return false;
	}
	*out = v;
	return true;
}

static const char *secret_state(const char *s)
{
	return s[0] != '\0' ? "<set>" : "<unset>";
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	const net_config_t *c = net_config_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh,
		    "{\"ssid\":\"%s\",\"psk\":\"%s\",\"broker\":\"%s\",\"port\":%u,"
		    "\"user\":\"%s\",\"mqttpass\":\"%s\",\"provisioned\":%u}",
		    c->ssid, secret_state(c->psk), c->broker, c->port,
		    c->mqtt_user, secret_state(c->mqtt_pass),
		    net_config_is_provisioned(c) ? 1u : 0u);
	return 0;
}

static int cmd_ssid(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	if (!net_config_set_ssid(net_config_get(), argv[1])) {
		shell_error(sh, "error: ssid must be 1..%d characters", NET_SSID_MAX);
		return -EINVAL;
	}

	ret = net_store_save_ssid();
	if (ret) {
		shell_error(sh, "error: ssid applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: ssid set (saved) — reboot to apply");
	return 0;
}

static int cmd_pass(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	/* This is the WiFi PSK. The MQTT password is `net mqttpass`. */
	if (!net_config_set_psk(net_config_get(), argv[1])) {
		shell_error(sh, "error: WiFi passphrase must be %d..%d characters",
			    NET_PSK_MIN, NET_PSK_MAX);
		return -EINVAL;
	}

	ret = net_store_save_psk();
	if (ret) {
		shell_error(sh, "error: WiFi passphrase applied in RAM but NOT "
				"persisted (errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: WiFi passphrase set (saved) — reboot to apply");
	return 0;
}

static int cmd_broker(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long port = NET_MQTT_PORT_DEFAULT;
	int ret;

	if (argc == 3 && !parse_ul(argv[2], &port)) {
		shell_error(sh, "error: port must be a number in 1..65535");
		return -EINVAL;
	}

	if (!net_config_set_broker(net_config_get(), argv[1], (uint32_t)port)) {
		shell_error(sh, "error: broker must be 1..%d characters and "
				"port in 1..65535", NET_BROKER_MAX);
		return -EINVAL;
	}

	ret = net_store_save_broker();
	if (ret) {
		shell_error(sh, "error: broker applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: broker=%s:%lu (saved) — reboot to apply",
		    net_config_get()->broker, port);
	return 0;
}

static int cmd_user(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	if (!net_config_set_user(net_config_get(), argv[1])) {
		shell_error(sh, "error: user must be at most %d characters",
			    NET_MQTT_USER_MAX);
		return -EINVAL;
	}

	ret = net_store_save_user();
	if (ret) {
		shell_error(sh, "error: user applied in RAM but NOT persisted "
				"(errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: user set (saved) — reboot to apply");
	return 0;
}

static int cmd_mqttpass(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);

	/* This is the MQTT password. The WiFi passphrase is `net pass`. */
	if (!net_config_set_mqtt_pass(net_config_get(), argv[1])) {
		shell_error(sh, "error: MQTT password must be at most %d characters",
			    NET_MQTT_PASS_MAX);
		return -EINVAL;
	}

	ret = net_store_save_mqtt_pass();
	if (ret) {
		shell_error(sh, "error: MQTT password applied in RAM but NOT "
				"persisted (errno %d) — will be lost on reboot", ret);
		return ret;
	}

	shell_print(sh, "ok: MQTT password set (saved) — reboot to apply");
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	net_config_set_defaults(net_config_get());

	ret = net_store_save_all();
	if (ret) {
		shell_error(sh, "error: defaults applied in RAM but NOT fully "
				"persisted (errno %d)", ret);
		return ret;
	}

	shell_print(sh, "ok: network defaults restored (saved) — reboot to apply");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_net,
	SHELL_CMD_ARG(show,     NULL, "Print the network configuration as JSON",
		      cmd_show,     1, 0),
	SHELL_CMD_ARG(ssid,     NULL, "ssid <ssid> — set the WiFi SSID",
		      cmd_ssid,     2, 0),
	SHELL_CMD_ARG(pass,     NULL, "pass <psk> — set the WiFi passphrase",
		      cmd_pass,     2, 0),
	SHELL_CMD_ARG(broker,   NULL, "broker <host> [port] — set the MQTT broker "
				      "(port defaults to 1883)",
		      cmd_broker,   2, 1),
	SHELL_CMD_ARG(user,     NULL, "user <username> — set the MQTT username",
		      cmd_user,     2, 0),
	SHELL_CMD_ARG(mqttpass, NULL, "mqttpass <password> — set the MQTT password",
		      cmd_mqttpass, 2, 0),
	SHELL_CMD_ARG(reset,    NULL, "Restore the network defaults and persist them",
		      cmd_reset,    1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(net, &sub_net, "Gateway network and MQTT configuration", NULL);
```

- [ ] **Step 3: Initialise the config before settings load**

In `src/main.c`, add the include and the init call. The order matters: `uwb_store_init()` calls `settings_load()`, which invokes the `net` handler, which writes into the singleton.

```c
#include "net_config.h"
```

```c
int main(void)
{
	const uwb_config_t *cfg = uwb_config_get();
	int ret;

	/* Before uwb_store_init(): its settings_load() runs the "net" handler,
	 * which writes into this singleton. Explicit rather than lazy because
	 * net_uplink is a second thread (see net_config.h). */
	net_config_init();

	uwb_store_init();
	log_config(cfg);
```

- [ ] **Step 4: Add the sources to the build**

In `CMakeLists.txt`, add `src/net_shell.c` and `src/net_store.c` to `target_sources`, alphabetically.

- [ ] **Step 5: Build and flash**

```powershell
west build -b ancla_esp32s3/esp32s3/procpu
west flash
west espressif monitor -p COM5
```

- [ ] **Step 6: Verify persistence on target**

```
uwb:~$ net show
{"ssid":"","psk":"<unset>","broker":"","port":1883,"user":"","mqttpass":"<unset>","provisioned":0}
uwb:~$ net ssid MyNetwork
uwb:~$ net pass hunter22
uwb:~$ net broker 10.0.0.5
uwb:~$ net user gateway1
uwb:~$ net mqttpass s3cret
uwb:~$ net show
uwb:~$ kernel reboot cold
uwb:~$ net show
```

Expected after reboot: `ssid`, `broker`, `port` 1883, `user` all restored; `psk` and `mqttpass` read `<set>` and **never** show their values; `provisioned` is 1.

Also verify rejection: `net pass short` is refused and leaves the stored passphrase intact; `net broker myhost 0` is refused.

- [ ] **Step 7: Commit**

```bash
git add src/net_store.c src/net_store.h src/net_shell.c src/main.c CMakeLists.txt
git commit -m "feat(net): NVS persistence and the net console tree

Per-field keys under net/, mirroring uwb_store. net_config_init() runs
before settings_load() because the handler writes into that singleton.
Secrets read back as <set>/<unset>, never as values."
```

---

## Task 4: Enable networking, protect the beacon, associate to WiFi

The riskiest task: it links the network stack into the image for the first time. Deliverable: the gateway associates, gets a DHCP address, and **still beacons cleanly**.

**Files:**
- Create: `src/net_uplink.h`, `src/net_uplink.c`
- Modify: `prj.conf`, `src/main.c`, `src/net_shell.c`, `CMakeLists.txt`

**Interfaces:**
- Consumes: `net_config_get()`, `net_config_is_provisioned()` (Task 2).
- Produces:
  - `void net_uplink_start(void);`
  - `const char *net_uplink_state_str(void);` — `"unconfigured"`, `"wifi-connecting"`, `"wifi-connected"`, `"mqtt-connecting"`, `"connected"`
  - `bool net_uplink_get_ip(char *buf, size_t len);` — false when no address is assigned
  - `void net_uplink_submit(const struct pos_fix *fix);` — declared here, **implemented in Task 6**

- [ ] **Step 1: Add the networking Kconfig**

Append to `prj.conf`:

```
# WiFi + MQTT uplink (GATEWAY mode only; see src/net_uplink.c).
# CONFIG_WIFI_ESP32 enables itself from the board's &wifi node.
CONFIG_WIFI=y
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_DHCPV4=y
CONFIG_NET_SOCKETS=y
CONFIG_DNS_RESOLVER=y
CONFIG_MQTT_LIB=y

# NOT OPTIONAL. The NET_TC_THREAD_TYPE choice defaults to its first entry,
# NET_TC_THREAD_COOPERATIVE at base priority 0 -- i.e. K_PRIO_COOP(0), the
# same priority main() is promoted to below. A cooperative thread cannot be
# preempted, so an RX burst would run to completion and can overrun the
# gateway's BEACON_ARM_MARGIN_UUS (5 ms). Forcing the traffic-class threads
# preemptible is what keeps the beacon protected rather than merely lucky.
CONFIG_NET_TC_THREAD_PREEMPTIVE=y

CONFIG_NET_LOG=y
```

- [ ] **Step 2: Write the uplink header**

Create `src/net_uplink.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's WiFi + MQTT uplink thread.
 *
 * Isolation from the UWB beacon is by PRIORITY, not by core: the ESP32 WiFi
 * driver is `depends on !SMP`, so there is exactly one core. This thread runs
 * preemptible and below both the promoted gateway loop and the WiFi driver's
 * own threads, and the gateway hands work over through a bounded queue that
 * never blocks the caller.
 */

#ifndef NET_UPLINK_H
#define NET_UPLINK_H

#include <stdbool.h>
#include <stddef.h>

#include "pos_sink.h"

/* Start the uplink thread. GATEWAY mode only -- a slave has nothing to publish
 * and starting WiFi there would cost ~50 KB of heap for no benefit. Safe to
 * call when unprovisioned: the thread logs once and idles. */
void net_uplink_start(void);

/* Hand one decoded fix to the uplink. Non-blocking: enqueues and returns, so
 * it is safe from the gateway's dispatch path. Drops the OLDEST queued fix if
 * the queue is full -- a stale position is worthless in an RTLS. */
void net_uplink_submit(const struct pos_fix *fix);

/* Human-readable connection state for `net show`. Never NULL. */
const char *net_uplink_state_str(void);

/* Copy the DHCP-assigned IPv4 address into `buf`. Returns false and leaves
 * `buf` untouched when no address is assigned. */
bool net_uplink_get_ip(char *buf, size_t len);

#endif /* NET_UPLINK_H */
```

- [ ] **Step 3: Write the WiFi association thread**

Create `src/net_uplink.c`. MQTT arrives in Task 5; this version associates and stops.

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "net_uplink.h"

#include "net_config.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <string.h>

LOG_MODULE_REGISTER(net_uplink, LOG_LEVEL_INF);

/* Below the WiFi driver's threads (capped at ESP32_WIFI_MAX_THREAD_PRIORITY,
 * default 7) so the driver drains its packet queues before we hand it more,
 * and far below the promoted gateway loop at K_PRIO_COOP(0). */
#define UPLINK_PRIO       10
#define UPLINK_STACK_SIZE 4096

/* Reconnect backoff ladder, in seconds. */
#define BACKOFF_START_S 1u
#define BACKOFF_MAX_S   32u

enum uplink_state {
	ST_UNCONFIGURED = 0,
	ST_WIFI_CONNECTING,
	ST_WIFI_CONNECTED,
	ST_MQTT_CONNECTING,
	ST_CONNECTED,
};

static const char *const state_names[] = {
	[ST_UNCONFIGURED]    = "unconfigured",
	[ST_WIFI_CONNECTING] = "wifi-connecting",
	[ST_WIFI_CONNECTED]  = "wifi-connected",
	[ST_MQTT_CONNECTING] = "mqtt-connecting",
	[ST_CONNECTED]       = "connected",
};

static enum uplink_state g_state = ST_UNCONFIGURED;

static K_SEM_DEFINE(ip_sem, 0, 1);
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void wifi_evt(struct net_mgmt_event_callback *cb, uint64_t event,
		     struct net_if *iface)
{
	ARG_UNUSED(iface);

	switch (event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		const struct wifi_status *st = (const struct wifi_status *)cb->info;

		if (st->status) {
			LOG_WRN("WiFi association failed (status %d)", st->status);
		} else {
			LOG_INF("WiFi associated");
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_WRN("WiFi disconnected");
		g_state = ST_WIFI_CONNECTING;
		break;
	default:
		break;
	}
}

static void ipv4_evt(struct net_mgmt_event_callback *cb, uint64_t event,
		     struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (event == NET_EVENT_IPV4_ADDR_ADD) {
		k_sem_give(&ip_sem);
	}
}

bool net_uplink_get_ip(char *buf, size_t len)
{
	struct net_if *iface = net_if_get_first_wifi();
	struct net_if_ipv4 *ipv4;

	if (iface == NULL || iface->config.ip.ipv4 == NULL) {
		return false;
	}

	ipv4 = iface->config.ip.ipv4;
	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (ipv4->unicast[i].ipv4.addr_state != NET_ADDR_PREFERRED) {
			continue;
		}
		if (net_addr_ntop(AF_INET, &ipv4->unicast[i].ipv4.address.in_addr,
				  buf, len) == NULL) {
			return false;
		}
		return true;
	}
	return false;
}

const char *net_uplink_state_str(void)
{
	return state_names[g_state];
}

/* Associate and wait for DHCP. Returns true once an IPv4 address is assigned. */
static bool wifi_connect(const net_config_t *cfg)
{
	struct wifi_connect_req_params params = {0};
	struct net_if *iface = net_if_get_first_wifi();
	int ret;

	if (iface == NULL) {
		LOG_ERR("no WiFi interface — is the &wifi node enabled?");
		return false;
	}

	params.ssid        = (const uint8_t *)cfg->ssid;
	params.ssid_length = strlen(cfg->ssid);
	params.psk         = (const uint8_t *)cfg->psk;
	params.psk_length  = strlen(cfg->psk);
	params.security    = WIFI_SECURITY_TYPE_PSK;
	params.channel     = WIFI_CHANNEL_ANY;
	params.band        = WIFI_FREQ_BAND_2_4_GHZ;
	params.mfp         = WIFI_MFP_OPTIONAL;

	g_state = ST_WIFI_CONNECTING;
	LOG_INF("associating with \"%s\"", cfg->ssid);

	/* The request fails while the interface is still coming up after boot,
	 * so retry rather than treating the first failure as fatal -- this is
	 * what samples/net/cloud/tagoio_http_post/src/wifi.c does, and skipping
	 * it makes the first association after every cold boot fail. */
	for (int tries = 10; tries > 0; tries--) {
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params,
			       sizeof(params));
		if (ret == 0) {
			break;
		}
		LOG_DBG("connect request failed (%d) — waiting for the iface", ret);
		k_msleep(500);
	}

	if (ret) {
		LOG_WRN("NET_REQUEST_WIFI_CONNECT failed (%d)", ret);
		return false;
	}

	net_dhcpv4_start(iface);

	/* 30 s covers association plus a DHCP exchange on a slow AP. Failing
	 * here is not fatal -- the caller backs off and retries. */
	if (k_sem_take(&ip_sem, K_SECONDS(30)) != 0) {
		LOG_WRN("no IPv4 address within 30 s");
		return false;
	}

	g_state = ST_WIFI_CONNECTED;
	return true;
}

static void uplink_thread(void *a, void *b, void *c)
{
	const net_config_t *cfg = net_config_get();
	uint32_t backoff_s = BACKOFF_START_S;
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	if (!net_config_is_provisioned(cfg)) {
		/* Logged exactly once. An unprovisioned gateway is a normal
		 * state, not an error to repeat forever. */
		LOG_INF("no network configuration — uplink idle "
			"(set `net ssid` and `net broker`, then reboot)");
		g_state = ST_UNCONFIGURED;
		return;
	}

	net_mgmt_init_event_callback(&wifi_cb, wifi_evt,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	net_mgmt_init_event_callback(&ipv4_cb, ipv4_evt, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	while (1) {
		if (!wifi_connect(cfg)) {
			LOG_WRN("retrying in %u s", backoff_s);
			k_sleep(K_SECONDS(backoff_s));
			backoff_s = MIN(backoff_s * 2u, BACKOFF_MAX_S);
			continue;
		}

		backoff_s = BACKOFF_START_S;

		if (net_uplink_get_ip(ip, sizeof(ip))) {
			LOG_INF("{\"wifi\":\"connected\",\"ip\":\"%s\"}", ip);
		}

		/* Task 5 replaces this with the MQTT connect and poll loop. */
		while (g_state == ST_WIFI_CONNECTED) {
			k_sleep(K_SECONDS(1));
		}
	}
}

K_THREAD_STACK_DEFINE(uplink_stack, UPLINK_STACK_SIZE);
static struct k_thread uplink_thread_data;

void net_uplink_start(void)
{
	k_thread_create(&uplink_thread_data, uplink_stack, UPLINK_STACK_SIZE,
			uplink_thread, NULL, NULL, NULL,
			UPLINK_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&uplink_thread_data, "net_uplink");
}
```

- [ ] **Step 4: Promote the gateway loop and start the thread**

In `src/main.c`, replace the mode dispatch:

```c
	if (cfg->mode == UWB_MODE_GATEWAY) {
		/* Run the beacon loop cooperatively so no network thread can
		 * preempt a delayed-TX arm. The loop already yields at its
		 * k_sem_take(), which is where every other thread gets time.
		 *
		 * This makes any unbounded busy-wait on this path fatal: no
		 * lower-priority thread -- including the shell -- can ever run
		 * again. Every spin here must be bounded (see
		 * uwb_wait_for_sysstatus_lo).
		 *
		 * SLAVE mode is deliberately left at the default priority: it
		 * runs no network thread, so the change would buy nothing and
		 * would perturb a bench-confirmed path. */
		k_thread_priority_set(k_current_get(), K_PRIO_COOP(0));
		net_uplink_start();
		uwb_gateway_run(cfg);
	} else {
		uwb_slave_run(cfg);
	}
```

Add the include:

```c
#include "net_uplink.h"
```

- [ ] **Step 5: Add live state to `net show`**

In `src/net_shell.c`, add `#include "net_uplink.h"` and `#include <zephyr/net/net_ip.h>`, then extend `cmd_show`:

```c
static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	const net_config_t *c = net_config_get();
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!net_uplink_get_ip(ip, sizeof(ip))) {
		strcpy(ip, "none");
	}

	shell_print(sh,
		    "{\"ssid\":\"%s\",\"psk\":\"%s\",\"broker\":\"%s\",\"port\":%u,"
		    "\"user\":\"%s\",\"mqttpass\":\"%s\",\"provisioned\":%u,"
		    "\"state\":\"%s\",\"ip\":\"%s\"}",
		    c->ssid, secret_state(c->psk), c->broker, c->port,
		    c->mqtt_user, secret_state(c->mqtt_pass),
		    net_config_is_provisioned(c) ? 1u : 0u,
		    net_uplink_state_str(), ip);
	return 0;
}
```

Add `#include <string.h>` for `strcpy`.

- [ ] **Step 6: Add the source and build**

Add `src/net_uplink.c` to `CMakeLists.txt`, then:

```powershell
west build -b ancla_esp32s3/esp32s3/procpu
```

Expected: builds. **Record the flash and RAM figures from the build output** — this is the first build with the network stack linked, and spec §10 requires the deltas.

- [ ] **Step 7: Verify on target**

Set `anchor mode gateway`, ensure `anchor pos` is set, reboot, then:

```
uwb:~$ net show
```

Expected: `"state":"wifi-connected"` and a real `"ip"`. The boot log shows `{"wifi":"connected","ip":"..."}`.

**Then check the beacon.** Watch the console for at least a minute with WiFi associated:
- No `grant missed its slot`
- No `beacon started but TXFRS never completed`
- A slave still observes the beacon `counter` incrementing every superframe

If any of those fail, stop — the threading model is wrong and no later task will fix it.

- [ ] **Step 8: Commit**

```bash
git add prj.conf src/net_uplink.c src/net_uplink.h src/main.c src/net_shell.c CMakeLists.txt
git commit -m "feat(net): WiFi association and beacon-safe thread priorities

NET_TC_THREAD_PREEMPTIVE is load-bearing: the traffic-class threads
otherwise default to K_PRIO_COOP(0), the same priority the gateway loop is
promoted to, and cannot be preempted. The gateway loop runs cooperatively
so no network thread can preempt a delayed-TX arm."
```

---

## Task 5: MQTT connection and the retained anchors stub

Deliverable: `mosquitto_sub` receives the retained anchors document on a late subscribe.

**Files:**
- Modify: `src/net_uplink.c`

**Interfaces:**
- Consumes: `pos_json_anchors()` (Task 1), `net_config_get()` (Task 2), the state machine from Task 4.
- Produces: an established `struct mqtt_client` and an internal `publish()` helper reused by Task 6.

- [ ] **Step 1: Add the MQTT includes and state**

At the top of `src/net_uplink.c`, add:

```c
#include "pos_json.h"

#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>

#include <errno.h>
#include <stdio.h>
```

And after the existing state definitions:

```c
#define MQTT_RX_BUF_SIZE 256
#define MQTT_TX_BUF_SIZE 256

/* Poll timeout while connected. Short enough that a queued fix waits at most
 * this long (Task 6) -- Zephyr cannot wait on a socket and a k_msgq in one
 * call without an eventfd, so the queue is drained on this cadence instead.
 * 50 ms against a 200 ms superframe is not a meaningful added latency. */
#define POLL_TIMEOUT_MS 50

#define LOCATION_TOPIC "testtopic/1/position"
#define ANCHOR_TOPIC   "testtopic/1/anchors"

static struct mqtt_client client;
static struct sockaddr_storage broker_addr;
static uint8_t mqtt_rx_buf[MQTT_RX_BUF_SIZE];
static uint8_t mqtt_tx_buf[MQTT_TX_BUF_SIZE];
static char    client_id[16];
static char    payload_buf[POS_JSON_MAX_LEN];
static uint16_t next_msg_id = 1u;
static bool    mqtt_connected;
static bool    connack_seen;
```

- [ ] **Step 2: Add the event handler and connect logic**

Add before `uplink_thread()`:

```c
static void mqtt_evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	ARG_UNUSED(c);

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_WRN("CONNACK refused (%d)", evt->result);
			break;
		}
		connack_seen = true;
		mqtt_connected = true;
		LOG_INF("MQTT connected");
		break;
	case MQTT_EVT_DISCONNECT:
		LOG_WRN("MQTT disconnected (%d)", evt->result);
		mqtt_connected = false;
		break;
	case MQTT_EVT_PUBACK:
		break;
	default:
		break;
	}
}

/* Resolve the broker into broker_addr. Accepts a hostname or an IPv4 literal;
 * getaddrinfo() handles both, which is why CONFIG_DNS_RESOLVER is enabled --
 * a literal-only implementation would make a moved broker a reflash. */
static bool resolve_broker(const net_config_t *cfg)
{
	struct zsock_addrinfo hints = {
		.ai_family   = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct zsock_addrinfo *res = NULL;
	struct sockaddr_in *in = (struct sockaddr_in *)&broker_addr;
	char port_str[6];
	int ret;

	snprintf(port_str, sizeof(port_str), "%u", cfg->port);

	ret = zsock_getaddrinfo(cfg->broker, port_str, &hints, &res);
	if (ret != 0 || res == NULL) {
		LOG_WRN("cannot resolve broker \"%s\" (%d)", cfg->broker, ret);
		return false;
	}

	memcpy(in, res->ai_addr, sizeof(*in));
	in->sin_port = htons(cfg->port);
	in->sin_family = AF_INET;
	zsock_freeaddrinfo(res);
	return true;
}

static int publish(const char *topic, const char *msg, enum mqtt_qos qos,
		   bool retain)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.qos          = qos;
	p.message.topic.topic.utf8   = (uint8_t *)topic;
	p.message.topic.topic.size   = strlen(topic);
	p.message.payload.data       = (uint8_t *)msg;
	p.message.payload.len        = strlen(msg);
	p.message_id                 = next_msg_id++;
	p.dup_flag                   = 0u;
	p.retain_flag                = retain ? 1u : 0u;

	/* message_id 0 is reserved by the protocol. */
	if (next_msg_id == 0u) {
		next_msg_id = 1u;
	}

	return mqtt_publish(&client, &p);
}

/* Connect and wait for CONNACK. Returns true with mqtt_connected set. */
static bool mqtt_bring_up(const net_config_t *cfg)
{
	static struct mqtt_utf8 user, pass;
	const uwb_config_t *ucfg = uwb_config_get();
	int64_t deadline;
	int ret;

	if (!resolve_broker(cfg)) {
		return false;
	}

	g_state = ST_MQTT_CONNECTING;
	connack_seen = false;
	mqtt_connected = false;

	snprintf(client_id, sizeof(client_id), "uwb-gw-%04X",
		 uwb_config_short_addr(ucfg));

	mqtt_client_init(&client);

	client.broker           = &broker_addr;
	client.evt_cb           = mqtt_evt_handler;
	client.client_id.utf8   = (uint8_t *)client_id;
	client.client_id.size   = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.transport.type   = MQTT_TRANSPORT_NON_SECURE;
	client.rx_buf           = mqtt_rx_buf;
	client.rx_buf_size      = sizeof(mqtt_rx_buf);
	client.tx_buf           = mqtt_tx_buf;
	client.tx_buf_size      = sizeof(mqtt_tx_buf);

	/* Empty credentials mean an anonymous broker: leave the pointers NULL
	 * rather than sending zero-length fields. */
	if (cfg->mqtt_user[0] != '\0') {
		user.utf8 = (uint8_t *)cfg->mqtt_user;
		user.size = strlen(cfg->mqtt_user);
		client.user_name = &user;
	} else {
		client.user_name = NULL;
	}
	if (cfg->mqtt_pass[0] != '\0') {
		pass.utf8 = (uint8_t *)cfg->mqtt_pass;
		pass.size = strlen(cfg->mqtt_pass);
		client.password = &pass;
	} else {
		client.password = NULL;
	}

	ret = mqtt_connect(&client);
	if (ret) {
		LOG_WRN("mqtt_connect failed (%d)", ret);
		return false;
	}

	/* Wait for CONNACK, pumping the socket. 10 s is generous for a LAN
	 * broker and still bounded. */
	deadline = k_uptime_get() + 10000;
	while (!connack_seen && k_uptime_get() < deadline) {
		struct zsock_pollfd fds = {
			.fd = client.transport.tcp.sock,
			.events = ZSOCK_POLLIN,
		};

		if (zsock_poll(&fds, 1, POLL_TIMEOUT_MS) > 0) {
			if (mqtt_input(&client) != 0) {
				break;
			}
		}
	}

	if (!connack_seen) {
		LOG_WRN("no CONNACK within 10 s");
		mqtt_abort(&client);
		return false;
	}

	g_state = ST_CONNECTED;
	return true;
}

/* Publish the stubbed zone/anchor map. Retained and QoS 1: it is slow-changing
 * state that a late subscriber needs, which is exactly the opposite of a
 * position fix. */
static void publish_anchor_stub(void)
{
	int n = pos_json_anchors(payload_buf, sizeof(payload_buf));

	if (n < 0) {
		LOG_ERR("anchors payload does not fit POS_JSON_MAX_LEN");
		return;
	}

	if (publish(ANCHOR_TOPIC, payload_buf, MQTT_QOS_1_AT_LEAST_ONCE, true)) {
		LOG_WRN("anchors publish failed");
	} else {
		LOG_INF("published retained anchor map");
	}
}
```

Add `#include "uwb_config.h"` at the top for `uwb_config_short_addr()`.

- [ ] **Step 3: Replace the placeholder inner loop**

In `uplink_thread()`, replace the `while (g_state == ST_WIFI_CONNECTED) { k_sleep(...); }` block:

```c
		if (!mqtt_bring_up(cfg)) {
			LOG_WRN("MQTT retry in %u s", backoff_s);
			k_sleep(K_SECONDS(backoff_s));
			backoff_s = MIN(backoff_s * 2u, BACKOFF_MAX_S);
			continue;
		}

		backoff_s = BACKOFF_START_S;
		publish_anchor_stub();

		while (mqtt_connected) {
			struct zsock_pollfd fds = {
				.fd = client.transport.tcp.sock,
				.events = ZSOCK_POLLIN,
			};

			if (zsock_poll(&fds, 1, POLL_TIMEOUT_MS) > 0 &&
			    mqtt_input(&client) != 0) {
				break;
			}

			/* Drives the keepalive PINGREQ. */
			if (mqtt_live(&client) != 0 && errno != EAGAIN) {
				break;
			}

			/* Task 6 drains the fix queue here. */
		}

		LOG_WRN("MQTT connection lost — reconnecting");
		mqtt_abort(&client);
		mqtt_connected = false;
		g_state = ST_WIFI_CONNECTED;
```

- [ ] **Step 4: Build and flash**

```powershell
west build -b ancla_esp32s3/esp32s3/procpu
west flash
```

- [ ] **Step 5: Verify against a broker**

On a machine on the same network, before booting the gateway:

```bash
mosquitto_sub -h 10.0.0.5 -t 'testtopic/1/#' -v
```

Expected: `testtopic/1/anchors {"name":"Zona","anchors":[...]}` appears on connect.

Then **verify the retain flag**: kill the subscriber, restart it, and confirm the anchors message arrives immediately without the gateway republishing.

Also verify `net show` reports `"state":"connected"`.

- [ ] **Step 6: Commit**

```bash
git add src/net_uplink.c
git commit -m "feat(net): MQTT connection and the retained anchor stub

Anchors are QoS 1 and retained -- slow-changing state a late subscriber
needs. getaddrinfo() so the broker can be a hostname or a literal."
```

---

## Task 6: Position queue and the `pos_sink` rewrite

Deliverable: fixes flow from the UWB dispatch path to the broker without ever blocking the beacon.

**Files:**
- Modify: `src/pos_sink.c`, `src/net_uplink.c`

**Interfaces:**
- Consumes: `net_uplink_submit()` (declared Task 4), `pos_json_fix()` (Task 1), `publish()` (Task 5).
- Produces: nothing new. `pos_sink.h` and `src/uwb_gateway.c` are unchanged.

- [ ] **Step 1: Add the queue and the submit implementation**

In `src/net_uplink.c`, after the buffer declarations:

```c
/* ~1.5 superframes at GW_N_CFP (11) seats: enough to absorb a publish stalling
 * behind one TCP retransmit, small enough that a real outage discards rather
 * than accumulates. */
#define FIX_QUEUE_DEPTH 16

K_MSGQ_DEFINE(fix_q, sizeof(struct pos_fix), FIX_QUEUE_DEPTH, 4);

static uint32_t dropped_fixes;
static int64_t  last_drop_warn_ms;

/* A sustained outage drops ~55 fixes per second. An unthrottled warning would
 * flood the very console being used to diagnose it. */
#define DROP_WARN_INTERVAL_MS 10000
```

And the submit function:

```c
void net_uplink_submit(const struct pos_fix *fix)
{
	struct pos_fix discarded;

	/* Called from the gateway's dispatch path: never block, never allocate,
	 * never touch a socket. */
	if (k_msgq_put(&fix_q, fix, K_NO_WAIT) == 0) {
		return;
	}

	/* Full: drop the OLDEST and take the newest. A stale position is
	 * worthless in an RTLS, so the newest sample is always the one worth
	 * keeping. */
	(void)k_msgq_get(&fix_q, &discarded, K_NO_WAIT);
	if (k_msgq_put(&fix_q, fix, K_NO_WAIT) != 0) {
		/* Only reachable if the uplink thread raced us to the slot; the
		 * fix is dropped either way. */
	}

	dropped_fixes++;

	int64_t now = k_uptime_get();

	if (now - last_drop_warn_ms >= DROP_WARN_INTERVAL_MS) {
		last_drop_warn_ms = now;
		LOG_WRN("uplink queue full — %u fixes dropped so far", dropped_fixes);
	}
}
```

- [ ] **Step 2: Drain the queue in the poll loop**

Add a drain helper before `uplink_thread()`:

```c
/* Publish everything queued. QoS 0 and not retained: the topic is flat, so
 * retaining would only preserve whichever tag published last, and QoS 0
 * matches the lossy UWB link underneath -- a fix needing a retransmit is
 * stale by the time it lands. Returns false if the connection died. */
static bool drain_fix_queue(void)
{
	struct pos_fix fix;

	while (k_msgq_get(&fix_q, &fix, K_NO_WAIT) == 0) {
		int n = pos_json_fix(payload_buf, sizeof(payload_buf), &fix);

		if (n < 0) {
			/* A formatting bug, not a network problem. Never publish
			 * a truncated JSON document. */
			LOG_ERR("position payload truncated — dropping fix from 0x%04X",
				fix.src_addr);
			continue;
		}

		if (publish(LOCATION_TOPIC, payload_buf,
			    MQTT_QOS_0_AT_MOST_ONCE, false) != 0) {
			LOG_WRN("position publish failed — reconnecting");
			return false;
		}
	}
	return true;
}
```

Replace the `/* Task 6 drains the fix queue here. */` comment in the inner loop with:

```c
			if (!drain_fix_queue()) {
				break;
			}
```

- [ ] **Step 3: Rewrite `pos_sink.c` as the producer**

Replace `src/pos_sink.c` entirely:

```c
#include "pos_sink.h"
#include "net_uplink.h"
#include "uwb_frame_802_15_4z.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pos_sink, LOG_LEVEL_INF);

void pos_sink_publish(const struct pos_fix *fix)
{
    /* Defensive: n_anchors is documented (pos_sink.h) as 3 or 4. A corrupt or
     * malicious frame could carry anything in that byte, and publishing it
     * unchecked would emit a bogus "n" into the JSON telemetry stream.
     * Rejected here so a bad frame never occupies a queue slot either. */
    if (fix->n_anchors < 3 || fix->n_anchors > UWB_FRAME_MAX_ANCHORS) {
        LOG_WRN("POS from 0x%04X: n_anchors=%u out of range, dropping",
                fix->src_addr, fix->n_anchors);
        return;
    }

    /* The console line stays. It is the ONLY place residual, n_anchors and
     * batt_soc remain visible -- the MQTT payload carries none of them (see
     * pos_json.c) -- and it is what makes the gateway debuggable over USB with
     * no broker present.
     *
     * An unknown battery is reported as JSON null, not as 0 or 255: a consumer
     * must be able to tell "no reading" from "flat battery". */
    if (fix->batt_soc == UWB_FRAME_POS_SOC_UNKNOWN) {
        LOG_INF("{\"tag\":\"0x%04X\",\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":null}",
                fix->src_addr, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors);
    } else {
        LOG_INF("{\"tag\":\"0x%04X\",\"x\":%.2f,\"y\":%.2f,"
                "\"residual\":%.3f,\"n\":%u,\"batt\":%u}",
                fix->src_addr, (double)fix->x, (double)fix->y,
                (double)fix->residual_m, fix->n_anchors, fix->batt_soc);
    }

    /* Non-blocking hand-off to the uplink thread. Safe on the dispatch path. */
    net_uplink_submit(fix);
}
```

- [ ] **Step 4: Build and flash**

```powershell
west build -b ancla_esp32s3/esp32s3/procpu
west flash
```

Expected: builds clean; `src/uwb_gateway.c` was never edited.

- [ ] **Step 5: Verify end to end**

With the gateway, at least three slaves and a tag powered:

```bash
mosquitto_sub -h 10.0.0.5 -t 'testtopic/1/position' -v
```

Expected: a stream of `{"tagId":"XXXX","x":...,"y":...,"z":0,"zoneName":"Zona"}`, matching the console `pos_sink` lines one for one.

Confirm the console still shows `residual` and `batt` on its own line while the broker payload does not.

- [ ] **Step 6: Commit**

```bash
git add src/pos_sink.c src/net_uplink.c
git commit -m "feat(net): publish position fixes over MQTT

pos_sink_publish() enqueues and returns -- no socket on the dispatch path.
Drop-oldest on a full queue, with a rate-limited warning: an outage drops
~55 fixes/s and an unthrottled log would flood the console being used to
diagnose it. pos_sink.h and uwb_gateway.c are unchanged."
```

---

## Task 7: Documentation

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add the new modules to the Layout section**

Insert, keeping the existing ordering style:

```markdown
- `src/pos_json.{c,h}` — MQTT payload formatting. Pure C, host-tested in
  `tests/pos_json/`. The position payload is a **fixed contract** with an
  existing Python consumer: `tagId` is bare uppercase hex with no `0x` prefix,
  `z` is the integer `0`, and the diagnostic fields are deliberately absent.
- `src/net_config.{c,h}` — WiFi and MQTT settings. Pure C, host-tested in
  `tests/net_config/`. Explicitly initialised from `main()`, **not** lazily like
  `uwb_config_get()`, because `net_uplink` is a second thread.
- `src/net_store.{c,h}` — the above persisted under the `net/` settings subtree.
- `src/net_shell.c` — the `net` console command tree.
- `src/net_uplink.{c,h}` — the WiFi + MQTT uplink thread and the bounded fix
  queue. GATEWAY mode only.
```

Update the `src/pos_sink.{c,h}` entry:

```markdown
- `src/pos_sink.{c,h}` — consumes decoded tag position fixes. Logs one JSON line
  per fix (the only place `residual`/`batt` stay visible) and hands the fix to
  `net_uplink` through a bounded queue.
```

- [ ] **Step 2: Add the console commands**

Under the Console section:

```
net show                       network config and live state as JSON
net ssid <ssid>                WiFi SSID
net pass <psk>                 WiFi passphrase (8..63) — NOT the MQTT password
net broker <host> [port]       MQTT broker; port defaults to 1883
net user <username>            MQTT username
net mqttpass <password>        MQTT password — NOT the WiFi passphrase
net reset                      restore network defaults
```

- [ ] **Step 3: Add the hard-won facts**

Append to the "Hard-won facts" section:

```markdown
- **`CONFIG_NET_TC_THREAD_PREEMPTIVE=y` is load-bearing, not tuning.** The
  `NET_TC_THREAD_TYPE` choice has no explicit default, so it resolves to its
  first entry, `NET_TC_THREAD_COOPERATIVE`, at base priority 0 — i.e.
  `K_PRIO_COOP(0)`, the *same* priority `main()` is promoted to in GATEWAY mode.
  A cooperative thread cannot be preempted, so an RX burst runs to completion
  and can overrun `BEACON_ARM_MARGIN_UUS` (5 ms). Removing this line does not
  fail to build and does not fail immediately — it makes the beacon
  intermittently late under network load.
- **The GATEWAY loop runs at `K_PRIO_COOP(0)`, so every busy-wait on that path
  must be bounded.** No lower-priority thread — including the shell — can run
  while it spins. An unbounded TXFRS wait already froze this board once at
  priority 0; cooperatively it is unrecoverable by construction.
- **WiFi is not on its own core and cannot be.** `WIFI_ESP32` is
  `depends on !SMP`, and the AMP alternative cannot host the network stack
  because the blobs bind to procpu. Isolation is by thread priority: the
  gateway loop cooperative, the WiFi blob tasks preemptible at ≤7
  (`ESP32_WIFI_MAX_THREAD_PRIORITY`), and `net_uplink` at 10.
- **`WIFI_ESP32` selects `MBEDTLS` and `PSA_CRYPTO` regardless of MQTT.** They
  are needed for WPA2 supplicant crypto, so mbedTLS is linked even with a
  plain-TCP broker. Adding MQTT TLS later therefore costs the TLS heap arena, a
  CA certificate and SNTP — not the library itself.
- **There are two passwords and they are not interchangeable.** `net pass` is
  the WiFi PSK (NVS `net/psk`, JSON `"psk"`); `net mqttpass` is the MQTT
  password (NVS `net/mqttpass`, JSON `"mqttpass"`). Command, key and JSON field
  agree in each case, deliberately.
```

- [ ] **Step 4: Add the host tests**

Under "Host tests":

```powershell
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c
./tests/pos_json/test_pos_json.exe              # PASSED, exits 0

gcc -Wall -Wextra -Isrc -o tests/net_config/test_net_config.exe tests/net_config/test_net_config.c src/net_config.c
./tests/net_config/test_net_config.exe          # PASSED, exits 0
```

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record the WiFi/MQTT uplink and its threading constraints"
```

---

## Task 8: On-target verification

No code. Runs the spec §11 acceptance criteria and records the results. **Requires a gateway, at least three slaves, a tag, an AP and a broker.**

**Files:**
- Create: `docs/verification-2026-08-12-mqtt-uplink.md`

- [ ] **Step 1: Run every host test**

```powershell
./tests/uwb_config/test_uwb_config.exe
./tests/uwb_frame/test_uwb_frame.exe
./tests/disc_schedule/test_disc_schedule.exe
./tests/gw_core/test_gw_core.exe
./tests/beacon_guard/test_beacon_guard.exe
./tests/pos_json/test_pos_json.exe
./tests/net_config/test_net_config.exe
```

Expected: all `PASSED`, all exit 0. Record the output.

- [ ] **Step 2: Provisioning and secrecy**

Set every `net` field, reboot, confirm all non-secret values survive and both secrets read `<set>`. Confirm `net show` never prints a secret value anywhere.

- [ ] **Step 3: Payload contract**

```bash
mosquitto_sub -h <broker> -t 'testtopic/1/#' -v
```

Capture one position message verbatim and confirm it matches
`{"tagId":"XXXX","x":N.NN,"y":N.NN,"z":0,"zoneName":"Zona"}` exactly — field
order, no `0x` prefix, integer `z`, no extra keys.

Subscribe *after* the gateway has connected and confirm the retained anchors
message still arrives.

- [ ] **Step 4: Beacon integrity under network load (acceptance gate)**

With WiFi associated and positions publishing, run for at least five minutes and confirm:
- No `grant missed its slot` in the console
- No `beacon started but TXFRS never completed`
- Slaves hold their CFP seats — no repeated JOIN→GRANT for the same tag
- A sniffer (or a slave's log) shows the beacon `counter` incrementing every superframe with no gaps

**If this fails, the threading model is wrong.** Do not tune timeouts to mask it.

- [ ] **Step 5: Outage and recovery**

Power off the AP. Confirm:
- The gateway keeps beaconing and slaves keep their seats
- The queue-full warning appears at most once per 10 s
- `net show` reports a disconnected state

Restore the AP. Confirm publishing resumes with no reboot and the retained anchors message is republished on reconnect.

- [ ] **Step 6: Record flash and RAM**

From the `west build` output, record the flash and RAM totals and the delta against a build with the networking Kconfig removed. Spec §10 requires measured figures, not estimates.

- [ ] **Step 7: Write and commit the verification record**

Create `docs/verification-2026-08-12-mqtt-uplink.md` with each step's result, the captured payload, and the memory figures. State plainly anything that did not pass.

```bash
git add docs/verification-2026-08-12-mqtt-uplink.md
git commit -m "docs: on-target verification of the WiFi/MQTT uplink"
```

---

## Notes for the implementer

- **Task 4 Step 7 is the real gate.** Everything after it assumes the beacon survives having a network stack in the image. If it does not, stop and report rather than proceeding to Task 5.
- **Do not edit `src/uwb_gateway.c`.** If a change there seems necessary, the design has drifted — raise it instead.
- The Zephyr WiFi and MQTT APIs move between releases. This plan targets Zephyr 4.4.x; if a struct field or `net_mgmt` event name does not exist, check `$ZEPHYR_BASE/include/zephyr/net/` and the samples under `$ZEPHYR_BASE/samples/net/` rather than guessing.
- `CONFIG_CBPRINTF_FP_SUPPORT=y` is already set, which is what makes `%.2f` work in both `LOG_INF` and `shell_print`. Do not remove it.

# Stable Tag Identity (Tid) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the `Tid` field the gateway publishes to the platform a stable per-physical-device identifier derived from the tag's EUI, instead of the ephemeral, reallocated MAC short address it is today.

**Architecture:** Today `pos_json_fix()` emits `Tid` as `fix->src_addr` (a 16-bit MAC short address, `gw_core.c`'s `alloc_short_addr()`). That address is not stable across a tag's lifetime: `gw_core_superframe_tick()` wipes a seat's entire record — including its EUI — the instant its lease ages to zero, so any rejoin after that (a battery disconnect, or simply going quiet longer than the lease) gets a brand-new, monotonically-higher address. The platform then sees one physical tag as several different `Tid` values over time.

The fix stays entirely inside this repo (`ANCLA_ESP32S3`, the gateway) and touches no wire format: the gateway already learns a tag's full 8-byte EUI at JOIN (`gw_core.c`'s `struct gw_seat.eui`), and a tag can only be transmitting a `0xEA` POS frame while some gateway is currently tracking it. So at the point the gateway decodes a POS frame (`uwb_gateway.c`'s `dispatch()`), it looks up that frame's `src_addr` in its own seat table to recover the EUI, hashes the EUI into a 32-bit value with FNV-1a (a new, tiny, pure-C, host-tested module — `tag_id_from_eui()`), and publishes that as `Tid` instead of the raw address. `Tid` stays a plain decimal integer (unchanged JSON schema, unchanged consumer contract), it just now comes from something that never gets reallocated.

One documented gap does not disappear and must be preserved, not "fixed" away: `uwb_gateway.c:236`'s existing comment says POS frames are deliberately **not gated on seat state** — a fix can legitimately arrive after the sender's lease (and therefore its seat, and the EUI stored in it) has already expired. For that narrow, already-accepted straggler case, the EUI lookup will fail; the fallback is `tag_id = src_addr` for that one fix only (today's behavior, not a new failure), logged so it's visible, and the fix is still published — never dropped.

**Tech Stack:** C (Zephyr RTOS project), no new dependencies. `tag_id_from_eui()` is a leaf pure-C function with the same host-test story as `cal_solve.c`/`apos_geom.c` in this repo (plain `gcc`, no Zephyr).

**Spec:** none (converged through live discussion with the maintainer, no separate written spec document — this plan is the design record).

## Global Constraints

- The JSON schema on the wire to the platform does not change: `Tid` stays an unquoted plain decimal integer in the same position. Only which C value feeds it changes.
- No `tag_testting` (the tag firmware, separate repo) file changes — this is gateway-only, per the maintainer's explicit direction that the frame format must stay untouched.
- POS frames must continue to publish even when the sender's seat cannot be found (`uwb_gateway.c:236`'s existing "not gated on seat state" comment stays true) — the EUI-lookup miss is a fallback, never a dropped fix.
- `tag_id_from_eui()` takes a raw byte buffer and length (`const uint8_t *, size_t`), not a `UWB_FRAME_EUI_LEN`-sized array — this keeps it a generic, standalone, host-testable leaf with no dependency on `uwb_frame_802_15_4z.h`.
- FNV-1a 32-bit exactly: offset basis `2166136261u` (`0x811c9dc5`), prime `16777619u` (`0x01000193`), the standard byte-at-a-time algorithm (`hash ^= byte; hash *= prime;` per byte, in that order).
- Existing host test suites for files this plan does not touch must still build and pass unchanged.

---

### Task 1: `tag_id_from_eui()` — FNV-1a 32-bit hash module

**Files:**
- Create: `src/tag_id.h`
- Create: `src/tag_id.c`
- Test: `tests/tag_id/test_tag_id.c`

**Interfaces:**
- Produces: `uint32_t tag_id_from_eui(const uint8_t *data, size_t len);` — later tasks call this as `tag_id_from_eui(eui, UWB_FRAME_EUI_LEN)` where `eui` is an 8-byte buffer, but the function itself takes no dependency on that constant.

- [ ] **Step 1: Write the failing test, using independently-computed FNV-1a reference vectors**

Compute the expected values yourself first, independently of the C code you're about to write, so the test is a real check and not a tautology. Run this in Bash to get exact reference values (these are the standard published FNV-1a 32-bit test vectors for the empty string, `"a"`, and `"foobar"`, plus one for an 8-byte all-zero EUI and one for a specific 8-byte pattern — compute all five so the test has real coverage):

```bash
python3 -c "
def fnv1a32(data):
    h = 2166136261
    for b in data:
        h ^= b
        h = (h * 16777619) & 0xFFFFFFFF
    return h

print('empty:', fnv1a32(b''))
print('a:', fnv1a32(b'a'))
print('foobar:', fnv1a32(b'foobar'))
print('eui_zero:', fnv1a32(bytes([0,0,0,0,0,0,0,0])))
print('eui_pattern:', fnv1a32(bytes([0xDE,0xAD,0xBE,0xEF,0x00,0x11,0x22,0x33])))
"
```

Record the five printed decimal values — you will hardcode them as the expected results in the test below (replace the `/* FILL IN FROM PYTHON OUTPUT ABOVE */` placeholders with the actual numbers your command printed; do not guess them).

Create `tests/tag_id/test_tag_id.c`:

```c
#include "tag_id.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_known_fnv1a_vectors(void)
{
    CHECK(tag_id_from_eui((const uint8_t *)"", 0) == /* FILL IN FROM PYTHON OUTPUT ABOVE: empty */);
    CHECK(tag_id_from_eui((const uint8_t *)"a", 1) == /* FILL IN FROM PYTHON OUTPUT ABOVE: a */);
    CHECK(tag_id_from_eui((const uint8_t *)"foobar", 6) == /* FILL IN FROM PYTHON OUTPUT ABOVE: foobar */);
}

static void test_eui_vectors(void)
{
    uint8_t eui_zero[8] = {0,0,0,0,0,0,0,0};
    uint8_t eui_pattern[8] = {0xDE,0xAD,0xBE,0xEF,0x00,0x11,0x22,0x33};

    CHECK(tag_id_from_eui(eui_zero, 8) == /* FILL IN FROM PYTHON OUTPUT ABOVE: eui_zero */);
    CHECK(tag_id_from_eui(eui_pattern, 8) == /* FILL IN FROM PYTHON OUTPUT ABOVE: eui_pattern */);
}

static void test_deterministic_and_distinguishes_inputs(void)
{
    uint8_t eui_a[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    uint8_t eui_b[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09};   /* differs in last byte only */

    uint32_t a1 = tag_id_from_eui(eui_a, 8);
    uint32_t a2 = tag_id_from_eui(eui_a, 8);
    uint32_t b  = tag_id_from_eui(eui_b, 8);

    CHECK(a1 == a2);     /* same input, same output, every time */
    CHECK(a1 != b);      /* a one-byte difference must not collide here */
}

int main(void)
{
    test_known_fnv1a_vectors();
    test_eui_vectors();
    test_deterministic_and_distinguishes_inputs();

    if (g_fail == 0) {
        printf("OK\n");
        return 0;
    }
    printf("%d failure(s)\n", g_fail);
    return 1;
}
```

- [ ] **Step 2: Run the test to verify it fails to build (the header/function don't exist yet)**

Run: `gcc -Wall -Wextra -Isrc -o tests/tag_id/test_tag_id.exe tests/tag_id/test_tag_id.c`
Expected: FAIL — compiler error, `tag_id.h: No such file or directory` (or similar), since `src/tag_id.h` does not exist yet.

- [ ] **Step 3: Write `src/tag_id.h`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FNV-1a 32-bit hash, used to derive a stable per-tag platform identity
 * (Tid) from a tag's EUI. See CLAUDE.md's "stable tag identity" entry for
 * why this exists and why it must not be replaced by anything that reads
 * only part of the EUI.
 */

#ifndef TAG_ID_H
#define TAG_ID_H

#include <stddef.h>
#include <stdint.h>

/* FNV-1a 32-bit over `len` bytes at `data`. Deterministic: the same bytes
 * always produce the same result, on any run, on any board. Not a
 * cryptographic hash -- it is chosen for even bit distribution across an
 * 8-byte EUI at negligible cost, not for collision resistance against an
 * adversary. */
uint32_t tag_id_from_eui(const uint8_t *data, size_t len);

#endif /* TAG_ID_H */
```

- [ ] **Step 4: Write `src/tag_id.c`**

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tag_id.h"

uint32_t tag_id_from_eui(const uint8_t *data, size_t len)
{
    uint32_t hash = 2166136261u;   /* FNV-1a 32-bit offset basis */

    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619u;         /* FNV-1a 32-bit prime */
    }
    return hash;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/tag_id/test_tag_id.exe tests/tag_id/test_tag_id.c src/tag_id.c && tests/tag_id/test_tag_id.exe`
Expected: `OK`, exit code 0.

- [ ] **Step 6: Commit**

```bash
git add src/tag_id.h src/tag_id.c tests/tag_id/test_tag_id.c
git commit -m "$(cat <<'EOF'
feat(pos): add tag_id_from_eui(), FNV-1a 32-bit over a tag's EUI

Pure-C, host-tested leaf. Will be used to derive the platform-facing Tid
from a tag's stable EUI instead of its reallocated MAC short address.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: `gw_core_find_eui()` — expose the seat table's EUI lookup

**Files:**
- Modify: `src/gw_core.h`
- Modify: `src/gw_core.c`
- Test: `tests/gw_core/test_gw_core.c`

**Interfaces:**
- Consumes: nothing new (uses the existing `struct gw_core_ctx`, `struct gw_seat`, and the existing private `find_seat_by_addr()` already in `gw_core.c`).
- Produces: `bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr, uint8_t eui_out[UWB_FRAME_EUI_LEN]);` — Task 4 calls this from `uwb_gateway.c`.

- [ ] **Step 1: Write the failing test**

Add to the end of `tests/gw_core/test_gw_core.c` (before its `main()` — find the existing `int main(void) { ... }` at the bottom and add this new `test_...` function above it, then add a call to it inside `main()` alongside the other `test_...()` calls):

```c
static void test_find_eui_by_addr(void)
{
    struct gw_core_ctx c;
    gw_core_init(&c);

    uint8_t eui[UWB_FRAME_EUI_LEN];
    struct gw_grant g;
    mk_eui(eui, 0x42);

    CHECK(gw_core_join(&c, eui, 1, &g));

    uint8_t out[UWB_FRAME_EUI_LEN];
    CHECK(gw_core_find_eui(&c, g.short_addr, out));
    CHECK(memcmp(out, eui, UWB_FRAME_EUI_LEN) == 0);

    /* An address with no live seat must fail cleanly, not read garbage. */
    uint8_t out2[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, (uint16_t)(g.short_addr + 999), out2));

    /* Address 0 is never a valid seat (0 means "free" in struct gw_seat). */
    uint8_t out3[UWB_FRAME_EUI_LEN];
    CHECK(!gw_core_find_eui(&c, 0, out3));
}
```

(`mk_eui()` and `CHECK` already exist at the top of this file — reuse them, do not redefine them.)

- [ ] **Step 2: Run the test to verify it fails**

Run: `gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe tests/gw_core/test_gw_core.c src/gw_core.c && tests/gw_core/test_gw_core.exe`
Expected: FAIL to compile — `gw_core_find_eui` is not declared (it doesn't exist yet).

- [ ] **Step 3: Add the declaration to `src/gw_core.h`**

Add this line directly after the existing `void gw_core_build_slotmap(...)` declaration, before the closing `#endif`:

```c
/* Look up the EUI of whichever seat currently holds `short_addr`. Returns
 * false (and leaves eui_out untouched) if no live seat holds that address --
 * this can legitimately happen for a POS frame that arrives just after its
 * sender's lease expired (see uwb_gateway.c's dispatch(), which does not
 * gate POS on seat state). Callers must have a fallback for false, not
 * treat it as an error. */
bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN]);
```

- [ ] **Step 4: Implement it in `src/gw_core.c`**

Add this function after `gw_core_build_slotmap()` (or anywhere after `find_seat_by_addr()` is defined, since it calls that existing static helper):

```c
bool gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr,
                      uint8_t eui_out[UWB_FRAME_EUI_LEN])
{
    int idx = find_seat_by_addr(c, short_addr);

    if (idx < 0) {
        return false;
    }
    memcpy(eui_out, c->seats[idx].eui, UWB_FRAME_EUI_LEN);
    return true;
}
```

Check the top of `src/gw_core.c` already has `#include <string.h>` (it does, for the existing `memcmp`/`memset` calls) — no new include needed.

- [ ] **Step 5: Run the test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe tests/gw_core/test_gw_core.c src/gw_core.c && tests/gw_core/test_gw_core.exe`
Expected: `PASSED` (or this suite's existing pass message — match whatever the current suite prints on success), exit code 0.

- [ ] **Step 6: Commit**

```bash
git add src/gw_core.h src/gw_core.c tests/gw_core/test_gw_core.c
git commit -m "$(cat <<'EOF'
feat(gw): expose gw_core_find_eui(), a seat-table lookup by short address

Lets a caller recover a currently-seated tag's EUI from its short
address. First consumer: the gateway's POS-frame dispatch path, to
derive a stable platform-facing Tid instead of publishing the reallocated
MAC address.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: `pos_fix.tag_id` — switch the JSON contract's source field

**Files:**
- Modify: `src/pos_sink.h`
- Modify: `src/pos_json.h`
- Modify: `src/pos_json.c`
- Test: `tests/pos_json/test_pos_json.c`

**Interfaces:**
- Consumes: nothing from Tasks 1-2 directly (this task only changes the struct field and formatting; Task 4 is what actually computes a real `tag_id` value at runtime). Every test in this task sets `.tag_id` explicitly on a `struct pos_fix` literal, exactly like the existing tests already set `.src_addr` explicitly.
- Produces: `struct pos_fix` gains a `uint32_t tag_id` field. `pos_json_fix()`'s `Tid` output now reads `fix->tag_id`, not `fix->src_addr`.

- [ ] **Step 1: Read the existing pinned-contract tests so you don't break their intent**

Open `tests/pos_json/test_pos_json.c` and read `test_fix_exact_contract()`, `test_fix_drops_diagnostics()`, and `test_tid_is_plain_decimal_not_hex()` (and any other test constructing a `struct pos_fix` literal). Every one of them currently sets `.src_addr` and expects `Tid` in the output to match it. Since `Tid` is about to come from a *different* field, each of these literals needs a `.tag_id` added, set to whatever value that specific test currently expects to see in the JSON's `Tid` position — and each test's `.src_addr` value should change to something visibly different from its `.tag_id` value, so the test would actually fail if the code accidentally still read `src_addr`. For example, `test_fix_exact_contract()` currently has `.src_addr = 0x1234` and expects `"Tid":4660` (4660 decimal = 0x1234) — change it to `.src_addr = 0x1234, .tag_id = 4660` (keeping the expected JSON string unchanged), and do the equivalent for every other `struct pos_fix` literal in this file that currently drives a `Tid` assertion. Do not change what each test asserts about the *shape* of the JSON (field order, absence of diagnostics, decimal-not-hex) — only which field number the `Tid` value comes from.

- [ ] **Step 2: Add one more test proving `Tid` comes from `tag_id`, not `src_addr`**

Add this new test to `tests/pos_json/test_pos_json.c` (near the other `test_tid_...` test), and call it from `main()` alongside the others:

```c
static void test_tid_is_tag_id_not_src_addr(void)
{
    /* If this ever regresses back to reading src_addr, this is the test
     * that catches it: src_addr and tag_id are deliberately different
     * values here, and only one of them may appear as Tid. */
    struct pos_fix f = { .src_addr = 0x0001, .tag_id = 999888777,
                         .x = 0.0f, .y = 0.0f,
                         .residual_m = 0.0f, .n_anchors = 3, .batt_soc = 0 };
    char buf[POS_JSON_MAX_LEN];

    CHECK(pos_json_fix(buf, sizeof(buf), &f) > 0);
    CHECK(strstr(buf, "\"Tid\":999888777") != NULL);
    CHECK(strstr(buf, "\"Tid\":1,") == NULL);   /* src_addr's decimal value must NOT appear */
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c && tests/pos_json/test_pos_json.exe`
Expected: FAIL to compile at first (`struct pos_fix` has no member `tag_id` yet — that's `pos_sink.h`, changed in the next step), or if you add the field first, the tests FAIL at runtime because `pos_json_fix()` still reads `src_addr`.

- [ ] **Step 4: Add the field to `struct pos_fix` in `src/pos_sink.h`**

Change:

```c
struct pos_fix {
    uint16_t src_addr;    /* tag short address, from the frame header */
    float    x;           /* metres */
    float    y;           /* metres */
    float    residual_m;  /* RMS range residual; larger means less trustworthy */
    uint8_t  n_anchors;   /* 3 or 4 */
    uint8_t  batt_soc;    /* 0-100, or UWB_FRAME_POS_SOC_UNKNOWN */
};
```

to:

```c
struct pos_fix {
    uint16_t src_addr;    /* tag short address, from the frame header -- MAC
                           * layer only; reallocated across a rejoin, so this
                           * must never be published to the platform as Tid.
                           * See tag_id below. */
    uint32_t tag_id;      /* stable per-physical-tag id: tag_id_from_eui() of
                           * the seat's EUI, resolved by the caller (the
                           * gateway's dispatch path) before publish. This is
                           * what pos_json_fix() emits as Tid. */
    float    x;           /* metres */
    float    y;           /* metres */
    float    residual_m;  /* RMS range residual; larger means less trustworthy */
    uint8_t  n_anchors;   /* 3 or 4 */
    uint8_t  batt_soc;    /* 0-100, or UWB_FRAME_POS_SOC_UNKNOWN */
};
```

- [ ] **Step 5: Update `pos_json_fix()` in `src/pos_json.c`**

Change:

```c
	/* Tid is fix->src_addr as a PLAIN DECIMAL NUMBER, not hex and not a
	 * quoted string -- 0x1234 formats as 4660, not "1234". This matches the
	 * downstream consumer's actual schema; there is no zoneName here, the
	 * consumer gets the zone from the anchors topic instead.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and there is
	 * no z measurement yet.
	 *
	 * residual_m, n_anchors and batt_soc are deliberately absent. They stay
	 * on pos_sink.c's console log line. */
	n = snprintf(buf, len,
		     "{\"Tid\":%u,\"x\":%.2f,\"y\":%.2f,\"z\":0}",
		     (unsigned int)fix->src_addr,
		     (double)fix->x, (double)fix->y);
```

to:

```c
	/* Tid is fix->tag_id as a PLAIN DECIMAL NUMBER, not hex and not a
	 * quoted string. tag_id is a stable per-physical-tag value derived
	 * from the tag's EUI (tag_id_from_eui(), src/tag_id.c) -- NOT
	 * fix->src_addr, which is only the tag's current MAC short address
	 * and gets reallocated across a rejoin (gw_core.c's seat table wipes
	 * a tag's record, EUI included, once its lease expires). Publishing
	 * src_addr here would make the platform see one physical tag as many
	 * different Tid values over its lifetime -- this is why the fixed
	 * contract's *format* (plain decimal, unquoted) survives unchanged
	 * while its *source field* does not. This matches the downstream
	 * consumer's actual schema; there is no zoneName here, the consumer
	 * gets the zone from the anchors topic instead.
	 *
	 * z is the integer literal 0, not %.2f: the solver is 2D and there is
	 * no z measurement yet.
	 *
	 * residual_m, n_anchors and batt_soc are deliberately absent. They stay
	 * on pos_sink.c's console log line. */
	n = snprintf(buf, len,
		     "{\"Tid\":%u,\"x\":%.2f,\"y\":%.2f,\"z\":0}",
		     (unsigned int)fix->tag_id,
		     (double)fix->x, (double)fix->y);
```

- [ ] **Step 6: Update the header doc comment in `src/pos_json.h`**

Change:

```c
/* Format one fix as the position payload:
 *   {"Tid":4660,"x":1.23,"y":4.56,"z":0}
 *
 * Tid is fix->src_addr as a plain decimal number (NOT hex, NOT a string) --
 * 0x1234 formats as 4660. z is the integer literal 0: the solver is 2D and
 * there is no z measurement yet.
 *
 * Returns the number of bytes written excluding the NUL, or -1 if the buffer
 * was too small. On -1 the caller MUST drop the message: publishing a
 * truncated JSON document is worse than publishing nothing. */
```

to:

```c
/* Format one fix as the position payload:
 *   {"Tid":4660,"x":1.23,"y":4.56,"z":0}
 *
 * Tid is fix->tag_id (a stable per-physical-tag id derived from the tag's
 * EUI -- see src/tag_id.c) as a plain decimal number (NOT hex, NOT a
 * string). It is NOT fix->src_addr, which is only the tag's current MAC
 * short address and is reallocated across a rejoin. z is the integer
 * literal 0: the solver is 2D and there is no z measurement yet.
 *
 * Returns the number of bytes written excluding the NUL, or -1 if the buffer
 * was too small. On -1 the caller MUST drop the message: publishing a
 * truncated JSON document is worse than publishing nothing. */
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c && tests/pos_json/test_pos_json.exe`
Expected: PASSED / all tests pass (match this suite's existing success output), exit code 0.

- [ ] **Step 8: Commit**

```bash
git add src/pos_sink.h src/pos_json.h src/pos_json.c tests/pos_json/test_pos_json.c
git commit -m "$(cat <<'EOF'
fix(pos): publish Tid from a stable tag_id, not the reallocated src_addr

struct pos_fix gains tag_id (uint32_t): a stable per-physical-tag value,
meant to be tag_id_from_eui() of the tag's EUI. pos_json_fix() now emits
Tid from tag_id instead of src_addr -- the JSON schema and format are
unchanged (Tid stays a plain decimal integer), only which field feeds it.

src_addr alone was never a valid platform identity: gw_core.c's seat
table wipes a tag's record (EUI included) the moment its lease expires,
so a rejoin after any gap -- including a battery disconnect -- got a
new, monotonically higher short address, and the platform saw one
physical tag as several different Tid values over time.

This task only changes the contract's source field and its tests; the
caller that actually computes a real tag_id at runtime (the gateway's
POS dispatch path) is a separate task, since uwb_gateway.c has no host
test harness.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Wire it up in the gateway's dispatch path

**Files:**
- Modify: `src/uwb_gateway.c:233-244` (the `uwb_frame_is_pos(buf, len)` branch inside `dispatch()`)

**Interfaces:**
- Consumes: `tag_id_from_eui(const uint8_t *data, size_t len)` (Task 1, `src/tag_id.h`), `gw_core_find_eui(const struct gw_core_ctx *c, uint16_t short_addr, uint8_t eui_out[UWB_FRAME_EUI_LEN])` (Task 2, `src/gw_core.h`), `struct pos_fix` with its new `tag_id` field (Task 3, `src/pos_sink.h`).
- Produces: nothing further consumed by later tasks — this is the last functional task; Task 5 is documentation only.

- [ ] **Step 1: Add the include**

`src/uwb_gateway.c` already includes `"gw_core.h"` and `"pos_sink.h"` (confirmed at the top of `dispatch()`'s enclosing file). Add:

```c
#include "tag_id.h"
```

alongside those existing includes near the top of `src/uwb_gateway.c`.

- [ ] **Step 2: Change the POS-frame branch inside `dispatch()`**

Find this exact block (currently `src/uwb_gateway.c:233-244`):

```c
	} else if (uwb_frame_is_pos(buf, len)) {
		struct pos_fix fix;

		/* Deliberately not gated on gw_core seat state: a fix from a tag
		 * whose lease just expired is still a real measurement, and
		 * silently dropping it would be close to undebuggable from the
		 * broker's side. */
		uwb_frame_parse_pos(buf, len, &fix.src_addr, &fix.x, &fix.y,
				    &fix.residual_m, &fix.n_anchors,
				    &fix.batt_soc);
		pos_sink_publish(&fix);
	}
```

Replace it with:

```c
	} else if (uwb_frame_is_pos(buf, len)) {
		struct pos_fix fix;
		uint8_t eui[UWB_FRAME_EUI_LEN];

		/* Deliberately not gated on gw_core seat state: a fix from a tag
		 * whose lease just expired is still a real measurement, and
		 * silently dropping it would be close to undebuggable from the
		 * broker's side. */
		uwb_frame_parse_pos(buf, len, &fix.src_addr, &fix.x, &fix.y,
				    &fix.residual_m, &fix.n_anchors,
				    &fix.batt_soc);

		/* Tid must be the tag's stable EUI-derived id, not its
		 * reallocatable short address (see pos_json.h). The seat table
		 * is the only place that EUI lives -- look it up by the
		 * address this frame just arrived from. A miss here means the
		 * sender's lease expired between its last KEEPALIVE and this
		 * POS frame (the "not gated on seat state" comment above): the
		 * fix is still real and must still be published, just without
		 * the stability guarantee for this one straggler. Falling back
		 * to fix.src_addr matches this path's old (pre-tag_id)
		 * behavior exactly, so this is a narrowing of a known gap, not
		 * a new failure mode. */
		if (gw_core_find_eui(ctx, fix.src_addr, eui)) {
			fix.tag_id = tag_id_from_eui(eui, UWB_FRAME_EUI_LEN);
		} else {
			LOG_WRN("POS from 0x%04X: no live seat, Tid falls back to short address",
				fix.src_addr);
			fix.tag_id = fix.src_addr;
		}
		pos_sink_publish(&fix);
	}
```

- [ ] **Step 3: Verify by inspection, not a host test**

`uwb_gateway.c` has no host-test harness in this repo (it depends on the DW3000 driver and Zephyr radio calls throughout, like `uwb_net_runner.c` in the sibling `tag_testting` project). Confirm correctness by re-reading the changed block against the four interfaces it now calls (`gw_core_find_eui`, `tag_id_from_eui`, the new `fix.tag_id` field, and the existing `LOG_WRN` macro already used elsewhere in this same file — check an existing `LOG_WRN(...)` call above in `dispatch()`, e.g. the JOIN branch, to confirm the format matches this file's existing logging style) rather than inventing a test for a file this project has never host-tested.

- [ ] **Step 4: Build the firmware to confirm it compiles clean**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu
```

If this environment's toolchain cannot complete a `west build` (a known, pre-existing, unrelated issue has affected other builds in adjacent sessions), report that plainly rather than guessing at the result — do not fabricate a build outcome.

- [ ] **Step 5: Run every host test suite this repo has, to confirm nothing broke**

```powershell
gcc -Wall -Wextra -o tests/uwb_config/test_uwb_config.exe tests/uwb_config/test_uwb_config.c src/uwb_config.c
./tests/uwb_config/test_uwb_config.exe
gcc -Wall -Wextra -Isrc -o tests/uwb_frame/test_uwb_frame.exe tests/uwb_frame/test_uwb_frame.c src/uwb_frame_802_15_4z.c
./tests/uwb_frame/test_uwb_frame.exe
gcc -Wall -Wextra -Isrc -o tests/disc_schedule/test_disc_schedule.exe tests/disc_schedule/test_disc_schedule.c src/disc_schedule.c
./tests/disc_schedule/test_disc_schedule.exe
gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe tests/gw_core/test_gw_core.c src/gw_core.c
./tests/gw_core/test_gw_core.exe
gcc -Wall -Wextra -Isrc -o tests/beacon_guard/test_beacon_guard.exe tests/beacon_guard/test_beacon_guard.c src/beacon_guard.c
./tests/beacon_guard/test_beacon_guard.exe
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c
./tests/pos_json/test_pos_json.exe
gcc -Wall -Wextra -Isrc -o tests/net_config/test_net_config.exe tests/net_config/test_net_config.c src/net_config.c
./tests/net_config/test_net_config.exe
gcc -Wall -Wextra -Isrc -o tests/cal_solve/test_cal_solve.exe tests/cal_solve/test_cal_solve.c src/cal_solve.c src/cal_math.c
./tests/cal_solve/test_cal_solve.exe
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
./tests/apos_geom/test_apos_geom.exe
gcc -Wall -Wextra -Isrc -o tests/apos_table/test_apos_table.exe tests/apos_table/test_apos_table.c src/apos_table.c src/apos_geom.c -lm
./tests/apos_table/test_apos_table.exe
gcc -Wall -Wextra -Isrc -o tests/apos_frame/test_apos_frame.exe tests/apos_frame/test_apos_frame.c src/apos_frame.c
./tests/apos_frame/test_apos_frame.exe
gcc -Wall -Wextra -Isrc -o tests/tag_id/test_tag_id.exe tests/tag_id/test_tag_id.c src/tag_id.c
./tests/tag_id/test_tag_id.exe
```

Expected: every suite reports its existing pass condition, exit code 0. None of these except `gw_core`, `pos_json`, and `tag_id` (already verified in Tasks 1-3) should be affected by this task's change — this step is a regression guard for the rest of the repo, matching this project's own documented full test list.

- [ ] **Step 6: Commit**

```bash
git add src/uwb_gateway.c
git commit -m "$(cat <<'EOF'
fix(gw): derive Tid from the tag's EUI at POS-frame dispatch

Wires up tag_id_from_eui() and gw_core_find_eui(): on every decoded POS
frame, look up the sender's EUI from the seat table by its current short
address and hash it into fix.tag_id, which pos_json_fix() now publishes
as Tid. Falls back to fix.tag_id = fix.src_addr (today's behavior) only
when the seat can't be found -- the documented case of a straggler POS
frame arriving after its sender's lease already expired -- so the fix is
still published, just without the stability guarantee for that one frame.

uwb_gateway.c has no host-test harness (DW3000/Zephyr radio calls
throughout); this task is verified by inspection against the interfaces
from Tasks 1-3 and a firmware build, not a new automated test.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Document the design in `CLAUDE.md`

**Files:**
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: nothing (documentation only).
- Produces: nothing consumed by other tasks; terminal task.

- [ ] **Step 1: Locate the existing `pos_json.{c,h}` bullet**

Run: `grep -n "pos_json.{c,h}" CLAUDE.md`

- [ ] **Step 2: Update that bullet's `Tid` sentence**

In the bullet found above, change:

```markdown
  downstream consumer: `{"Tid":<decimal>,"x":...,"y":...,"z":0}` — `Tid` is
  `fix->src_addr` as a **plain decimal number** (not hex, not a string; e.g.
  `0x1234` → `4660`), `z` is the integer `0`, there is no `zoneName` (the
```

to:

```markdown
  downstream consumer: `{"Tid":<decimal>,"x":...,"y":...,"z":0}` — `Tid` is
  `fix->tag_id` (`src/tag_id.c`'s `tag_id_from_eui()` of the tag's EUI, a
  **stable per-physical-device id**, resolved from the gateway's seat table
  at dispatch time — see the "stable tag identity" entry below for why this
  is not `fix->src_addr`) as a **plain decimal number** (not hex, not a
  string; e.g. `0x1234` → `4660`), `z` is the integer `0`, there is no
  `zoneName` (the
```

(the rest of that sentence and bullet — starting from `consumer gets the zone from the anchors topic` — is unchanged; only insert the new clause where shown).

- [ ] **Step 3: Add a new hard-won-fact bullet**

Find the "Hard-won facts (do not re-derive)" section (search `grep -n "Hard-won facts"`) and add this new bullet at the end of that section, immediately before the `## System context` heading that follows it:

```markdown
- **Stable tag identity (Tid) is derived from the tag's EUI, not its MAC
  short address — the two are not interchangeable, and this was a real
  field bug.** `gw_core_superframe_tick()` wipes a seat's entire record,
  EUI included, the instant its lease ages to zero, and `alloc_short_addr()`
  is a bare monotonic counter with no memory of previously-issued
  addresses. So any rejoin after a lease gap — including a plain battery
  disconnect, which guarantees the old lease can't be renewed in time —
  got a brand-new, permanently higher short address, and the platform saw
  one physical tag as several different `Tid` values over its lifetime
  (`0x0101`, `0x0102`, `0x0103`, ... observed directly on the bench).
  Fixed without touching the wire format or the tag firmware at all: the
  gateway already learns a tag's full EUI at JOIN (`struct gw_seat.eui`),
  and a tag can only be transmitting a POS frame while some gateway is
  tracking it, so `uwb_gateway.c`'s POS dispatch looks up the sender's EUI
  from the seat table (`gw_core_find_eui()`) and hashes it (FNV-1a 32-bit,
  `tag_id_from_eui()`) into `fix.tag_id`, which `pos_json_fix()` now
  publishes as `Tid` instead of `fix.src_addr`. A single EUI half (either
  32-bit `NRF_FICR->DEVICEID` word alone) was deliberately rejected as the
  source: Nordic only guarantees the *combined* 64-bit value unique, and a
  single half can plausibly repeat within one production batch since these
  words are typically wafer/lot/die-position derived — hash the whole
  8-byte EUI, never a slice of it. **`gw_core`'s POS dispatch is
  deliberately not gated on seat state** (`uwb_gateway.c`'s existing
  comment on the `uwb_frame_is_pos` branch) — a fix can arrive after its
  sender's lease, and therefore its seat and the EUI in it, has already
  expired. `gw_core_find_eui()` returning false is this documented case,
  not a new error: the fallback is `tag_id = src_addr` for that one
  straggler fix (today's pre-fix behavior, not a regression), logged, and
  the fix is still published — never dropped. This is why `Tid`'s
  stability is a strong guarantee for a live tag and a best-effort one for
  a fix that lands in the same superframe as a lease expiry, not an
  absolute one.
```

- [ ] **Step 4: Confirm it reads correctly**

Run: `grep -n -B2 -A2 "Stable tag identity" CLAUDE.md`
Expected: the new bullet appears once, in the Hard-won facts section, correctly formatted.

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: record the stable-Tid design and the amnesia bug it fixes

Companion to the tag_id_from_eui() / gw_core_find_eui() / pos_json
changes -- so the next person who sees a tag's platform id change across
a reconnect finds this instead of re-deriving it, and so nobody "fixes"
the not-gated-on-seat-state POS dispatch path into something that drops
straggler fixes.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

# Anchor Auto-Positioning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace hand-entered anchor coordinates with a gateway-orchestrated survey in which anchors range each other, the gateway solves the 3D geometry, and the result is pushed back to every anchor and published as the real anchors payload.

**Architecture:** The gateway enumerates anchors by EUI-64 over the air, commands each one to range a single peer at a time with the already-calibrated SS-TWR path, accumulates a sparse directed range matrix, and solves it with a gauge-locked Levenberg–Marquardt fit before pushing coordinates back to persist. All geometry, table and codec logic is pure C and host-tested; the Zephyr layers are thin radio shims.

**Tech Stack:** Zephyr 4.4.x, ESP32-S3, Qorvo DW3220 via the vendored br101 decadriver, Zephyr settings/NVS, plain gcc for host tests.

**Spec:** `docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md`

## Global Constraints

- `$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"` is **required** for every `west` command — this project lives outside the west workspace.
- Build: `west build -b ancla_esp32s3/esp32s3/procpu`. Calibration image: `west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf`.
- `src/uwb_frame_802_15_4z.{c,h}` is **copied byte-for-byte** from the tag (`tag_testting/src/`) and MUST NOT be modified. New frames go in `src/apos_frame.{c,h}`.
- `src/cal_math.{c,h}` is likewise byte-identical to the tag's copy. Do not modify.
- The position MQTT payload `{"Tid":<decimal>,"x":...,"y":...,"z":0}` is a **fixed contract** with a downstream consumer. This plan does not change it. Only the *anchors* payload changes.
- Every source file starts with the project header: `Copyright (c) 2026 Innovaforce` / `SPDX-License-Identifier: Apache-2.0` / a comment explaining the file's purpose.
- Pure-C modules (`apos_frame`, `apos_geom`, `apos_table`, and the header of `apos_store`) must include **no Zephyr headers** — they carry the host-test burden.
- Tabs for indentation in Zephyr-side `src/*.c`; the existing pure-C host-tested files (`uwb_frame_802_15_4z.c`, `pos_json.c`, `cal_math.c`) use 4 spaces. Match the file you are in.
- Host tests link with `-lm` where they use `sqrtf`/`acosf`/`cosf`.
- `APOS_MAX_NODES` is 8 and is deliberately **not** `UWB_MAX_ANCHORS` (4). Never substitute one for the other.
- Anchor short address is `UWB_ANCHOR_ADDR_BASE + anchor_id` = `0x0001 + id`. `0x0000` is the gateway and is never a survey node.
- All multi-byte wire fields are little-endian, matching `put_u16`/`put_u32`/`put_f32` in `uwb_frame_802_15_4z.c`.

---

## File Structure

**Created — pure C, host-tested, no Zephyr:**

| File | Responsibility |
|---|---|
| `src/apos_geom.{c,h}` | Sparse 3D geometry solver: gauge validation, closed-form seed, LM refinement, diagnostics. |
| `src/apos_table.{c,h}` | Discovered-peer table, directed-measurement accumulator, symmetrisation into edges. |
| `src/apos_frame.{c,h}` | Codec for the seven `0xEB` APOS subtypes. |
| `src/apos_store.h` | `struct apos_survey` (pure type, consumed by `pos_json.c`). |

**Created — Zephyr:**

| File | Responsibility |
|---|---|
| `src/apos_store.c` | Persists `struct apos_survey` under the `apos/` settings subtree. |
| `src/apos_node.{c,h}` | Anchor side: survey window, `ENUM_RSP`, one commanded range batch, `SETPOS` persistence. |
| `src/apos_gw.{c,h}` | Gateway orchestration as a step function driven from `uwb_gateway_run()`. |
| `src/apos_shell.c` | The `apos` console tree. |

**Renamed:**

| From | To | Why |
|---|---|---|
| `src/cal_initiator.{c,h}` | `src/ss_initiator.{c,h}` | Shared by the calibration and production images; the name should not claim it is calibration-only. |

**Modified:**

| File | Change |
|---|---|
| `CMakeLists.txt` | Add the new sources; move `ss_initiator.c` out of the cal-only block. |
| `src/anchor_respond.{c,h}` | `anchor_respond_wave_poll()` gains an `allow_unpositioned` parameter and refuses to answer when unpositioned. |
| `src/uwb_slave.c` | Offer frames to `apos_node_on_rx()`; pass `apos_node_window_open()` to the poll responder. |
| `src/uwb_gateway.c` | Dispatch APOS frames to `apos_gw_on_rx()`; call `apos_gw_step()` from the idle path. |
| `src/cal_run.c` | Follow the `ss_initiator` rename; pass `allow_unpositioned = true`. |
| `src/pos_json.{c,h}` | `pos_json_anchors()` takes a `const struct apos_survey *`; emits surveyed geometry when present. |
| `src/net_uplink.c` | Pass `apos_store_get()` at the `pos_json_anchors()` call site. |
| `CLAUDE.md` | Status, console reference, layout, host tests, hard-won facts. |

**Created — docs and tests:**

`tests/apos_geom/`, `tests/apos_table/`, `tests/apos_frame/`, `docs/anchor-auto-positioning.md`.

---

### Task 1: `apos_geom` — types, gauge validation, closed-form seed

**Files:**
- Create: `src/apos_geom.h`, `src/apos_geom.c`
- Test: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Consumes: nothing.
- Produces: `APOS_MAX_NODES` (8), `APOS_MAX_EDGES` (28), `struct apos_edge {uint8_t i, j; float d_m, sd_m;}`, `struct apos_gauge {uint8_t origin, xaxis, plane, up;}`, `enum apos_node_state {APOS_NODE_UNPLACED=0, APOS_NODE_PLACED=1, APOS_NODE_AMBIGUOUS=2}`, `struct apos_node_out`, `struct apos_result`, `bool apos_geom_gauge_valid(const struct apos_gauge *g, uint8_t n_nodes)`, `int apos_geom_seed(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes, const struct apos_gauge *g, struct apos_result *out)`, `void apos_geom_zoff(struct apos_result *r, float dz)`.

- [ ] **Step 1: Write the header**

Create `src/apos_geom.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sparse 3D geometry solver for the anchor survey.
 *
 * Input is a flat list of undirected edges, NOT a matrix: a deployment larger
 * than a few anchors cannot range every pair, and a matrix representation would
 * force a "missing" sentinel into every consumer. A hole is simply an absent
 * edge, and the fit works around it.
 *
 * Inter-anchor ranges determine the array's shape but not its placement -- the
 * solution is free up to translation, rotation and reflection. struct apos_gauge
 * pins all seven degrees of freedom from four operator-designated nodes; see
 * docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md section 3.
 *
 * Pure C -- no Zephyr, no radio headers -- so the whole solver is host-testable
 * with synthetic geometry and never needs hardware to develop.
 */

#ifndef APOS_GEOM_H
#define APOS_GEOM_H

#include <stdbool.h>
#include <stdint.h>

/* Survey capacity. Deliberately NOT UWB_MAX_ANCHORS (4): the survey has no
 * reason to inherit the tag-facing ranging MAC's limit. Growing the deployment
 * past 4 RANGING anchors is separate work needing disc_schedule.h's stagger and
 * anchor_respond.c's TX_COMPLETE_TIMEOUT_MS re-derived, plus the tag's
 * UWB_FRAME_MAX_ANCHORS -- see the design doc section 7.1. */
#define APOS_MAX_NODES 8
#define APOS_MAX_EDGES ((APOS_MAX_NODES * (APOS_MAX_NODES - 1)) / 2)

/* Gauge needs four distinct nodes, so a survey below this cannot be solved. */
#define APOS_MIN_NODES 4

struct apos_edge {
	uint8_t i;    /* node index, < n_nodes */
	uint8_t j;    /* node index, < n_nodes, != i */
	float   d_m;  /* symmetrised distance, metres */
	float   sd_m; /* standard deviation, metres; must be > 0 */
};

/* Four operator designations, as node indices into the same array the edges
 * index. All four must be distinct. */
struct apos_gauge {
	uint8_t origin; /* -> (0, 0, 0) */
	uint8_t xaxis;  /* -> (d, 0, 0), d > 0 */
	uint8_t plane;  /* -> (x, y, 0), y > 0 */
	uint8_t up;     /* -> z > 0 */
};

enum apos_node_state {
	APOS_NODE_UNPLACED  = 0, /* fewer than 3 edges to placed nodes */
	APOS_NODE_PLACED    = 1,
	APOS_NODE_AMBIGUOUS = 2, /* placed, but its reflection was a guess */
};

struct apos_node_out {
	float   x, y, z;
	uint8_t state;      /* enum apos_node_state */
	float   residual_m; /* RMS over this node's edges; 0 while unplaced */
};

struct apos_result {
	struct apos_node_out node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	uint8_t  n_placed;
	uint8_t  n_ambiguous;
	float    rms_m;        /* RMS residual over all usable edges */
	float    worst_edge_m; /* largest |residual| over usable edges */
	uint8_t  worst_i;      /* the pair that produced worst_edge_m */
	uint8_t  worst_j;
	float    planarity_m;  /* RMS distance of placed nodes from their own
				* best-fit plane. Small means the array is
				* near-coplanar and z is not trustworthy. */
	uint16_t iterations;   /* LM iterations used; 0 after seed alone */
};

/* Four distinct indices, each < n_nodes, and n_nodes in range. */
bool apos_geom_gauge_valid(const struct apos_gauge *g, uint8_t n_nodes);

/* Closed-form initial placement. Fills out->node[].{x,y,z,state}, n_nodes,
 * n_placed and n_ambiguous. Does not compute residuals or planarity -- that is
 * apos_geom_refine()'s job.
 *
 * Returns 0 on success, -EINVAL on a bad argument or invalid gauge, or -ENODATA
 * if the gauge nodes lack the edges needed to place them (the three gauge-plane
 * edges plus at least three edges from `up`). Nodes that cannot be placed are
 * left APOS_NODE_UNPLACED and are NOT an error -- they are reported. */
int apos_geom_seed(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		   const struct apos_gauge *g, struct apos_result *out);

/* Shift every placed node's z by dz, moving the z = 0 plane. Applied after
 * solving so the operator can put z = 0 on the floor rather than on the plane
 * through the three gauge anchors. */
void apos_geom_zoff(struct apos_result *r, float dz);

#endif /* APOS_GEOM_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/apos_geom/test_apos_geom.c`:

```c
#include "apos_geom.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

#define CLOSE(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

/* A 4-node reference layout: three corners of a right triangle in z = 0 plus
 * one node lifted above it. Distances are exact, so a correct seed must
 * reproduce the coordinates to float precision. */
static const float ref_xyz[4][3] = {
    {0.0f, 0.0f, 0.0f},  /* origin */
    {3.0f, 0.0f, 0.0f},  /* xaxis  */
    {0.0f, 4.0f, 0.0f},  /* plane  */
    {1.0f, 1.0f, 2.0f},  /* up     */
};

static float dist3(const float a[3], const float b[3])
{
    float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];

    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* Every pair of the reference layout, exact, sd 1 mm. */
static uint16_t build_full_mesh(struct apos_edge *out, uint8_t n)
{
    uint16_t k = 0;

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
            out[k].i = i;
            out[k].j = j;
            out[k].d_m = dist3(ref_xyz[i], ref_xyz[j]);
            out[k].sd_m = 0.001f;
            k++;
        }
    }
    return k;
}

static const struct apos_gauge g_ref = {
    .origin = 0, .xaxis = 1, .plane = 2, .up = 3
};

static void test_gauge_requires_four_distinct_nodes(void)
{
    struct apos_gauge bad = {.origin = 0, .xaxis = 1, .plane = 1, .up = 3};

    CHECK(apos_geom_gauge_valid(&g_ref, 4));
    CHECK(!apos_geom_gauge_valid(&bad, 4));
    /* An index at or past n_nodes is out of range. */
    CHECK(!apos_geom_gauge_valid(&g_ref, 3));
}

static void test_seed_reproduces_the_exact_layout(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_seed(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.n_placed == 4);
    CHECK(r.n_ambiguous == 0);

    for (int i = 0; i < 4; i++) {
        CHECK(r.node[i].state == APOS_NODE_PLACED);
        CLOSE(r.node[i].x, ref_xyz[i][0], 1e-3f);
        CLOSE(r.node[i].y, ref_xyz[i][1], 1e-3f);
        CLOSE(r.node[i].z, ref_xyz[i][2], 1e-3f);
    }
}

/* The gauge is what makes the answer unique: origin AT the origin, xaxis on
 * +x with y and z exactly zero, plane in z = 0 with y > 0, up with z > 0. */
static void test_gauge_constraints_hold_exactly(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_seed(e, n, 4, &g_ref, &r) == 0);

    CLOSE(r.node[0].x, 0.0f, 1e-6f);
    CLOSE(r.node[0].y, 0.0f, 1e-6f);
    CLOSE(r.node[0].z, 0.0f, 1e-6f);
    CHECK(r.node[1].x > 0.0f);
    CLOSE(r.node[1].y, 0.0f, 1e-6f);
    CLOSE(r.node[1].z, 0.0f, 1e-6f);
    CHECK(r.node[2].y > 0.0f);
    CLOSE(r.node[2].z, 0.0f, 1e-6f);
    CHECK(r.node[3].z > 0.0f);
}

/* A node with only two edges cannot be trilaterated. It is reported unplaced,
 * not treated as a failure -- that distinction is what tells the operator to
 * move an anchor rather than to retry the run. */
static void test_node_with_two_edges_is_unplaced_not_an_error(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    /* Node 4 exists but only reaches nodes 0 and 1. */
    e[n].i = 0; e[n].j = 4; e[n].d_m = 2.0f; e[n].sd_m = 0.001f; n++;
    e[n].i = 1; e[n].j = 4; e[n].d_m = 2.0f; e[n].sd_m = 0.001f; n++;

    CHECK(apos_geom_seed(e, n, 5, &g_ref, &r) == 0);
    CHECK(r.n_placed == 4);
    CHECK(r.node[4].state == APOS_NODE_UNPLACED);
}

/* Exactly three edges leaves the node mirrored about the plane through its
 * three neighbours. It is still placed so the fit can proceed, but the guess is
 * flagged, because acceptance has to be able to reject it. */
static void test_node_with_three_edges_is_flagged_ambiguous(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);
    const float p4[3] = {2.0f, 2.0f, 1.5f};

    /* Reaches 0, 1, 2 -- all three in the z = 0 plane, so the two solutions
     * are +z and -z and nothing in its own edges can choose between them. */
    for (uint8_t k = 0; k < 3; k++) {
        e[n].i = k; e[n].j = 4;
        e[n].d_m = dist3(ref_xyz[k], p4);
        e[n].sd_m = 0.001f;
        n++;
    }

    CHECK(apos_geom_seed(e, n, 5, &g_ref, &r) == 0);
    CHECK(r.node[4].state == APOS_NODE_AMBIGUOUS);
    CHECK(r.n_ambiguous == 1);
    /* Placed at the right horizontal position regardless of which branch. */
    CLOSE(r.node[4].x, 2.0f, 1e-3f);
    CLOSE(r.node[4].y, 2.0f, 1e-3f);
    CLOSE(fabsf(r.node[4].z), 1.5f, 1e-3f);
}

/* A fourth edge resolves the mirror, so the same node becomes unambiguous. */
static void test_fourth_edge_resolves_the_mirror(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);
    const float p4[3] = {2.0f, 2.0f, 1.5f};

    for (uint8_t k = 0; k < 4; k++) {
        e[n].i = k; e[n].j = 4;
        e[n].d_m = dist3(ref_xyz[k], p4);
        e[n].sd_m = 0.001f;
        n++;
    }

    CHECK(apos_geom_seed(e, n, 5, &g_ref, &r) == 0);
    CHECK(r.node[4].state == APOS_NODE_PLACED);
    CHECK(r.n_ambiguous == 0);
    CLOSE(r.node[4].z, 1.5f, 1e-3f);
}

/* Missing a gauge edge is a hard failure: without it there is no frame to
 * place anything else in. */
static void test_missing_gauge_edge_is_enodata(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    /* Drop the origin-xaxis edge by shifting the rest down over it. */
    uint16_t w = 0;
    for (uint16_t k = 0; k < n; k++) {
        if (e[k].i == 0 && e[k].j == 1) {
            continue;
        }
        e[w++] = e[k];
    }

    CHECK(apos_geom_seed(e, w, 4, &g_ref, &r) == -ENODATA);
}

static void test_zoff_shifts_only_placed_nodes(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_seed(e, n, 4, &g_ref, &r) == 0);
    apos_geom_zoff(&r, 2.5f);

    CLOSE(r.node[0].z, 2.5f, 1e-6f);
    CLOSE(r.node[3].z, 4.5f, 1e-6f);
}

static void test_rejects_bad_arguments(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_seed(NULL, n, 4, &g_ref, &r) == -EINVAL);
    CHECK(apos_geom_seed(e, n, 4, &g_ref, NULL) == -EINVAL);
    CHECK(apos_geom_seed(e, n, APOS_MAX_NODES + 1, &g_ref, &r) == -EINVAL);
    /* Below the gauge's own requirement of four distinct nodes. */
    CHECK(apos_geom_seed(e, n, 3, &g_ref, &r) == -EINVAL);
}

int main(void)
{
    test_gauge_requires_four_distinct_nodes();
    test_seed_reproduces_the_exact_layout();
    test_gauge_constraints_hold_exactly();
    test_node_with_two_edges_is_unplaced_not_an_error();
    test_node_with_three_edges_is_flagged_ambiguous();
    test_fourth_edge_resolves_the_mirror();
    test_missing_gauge_edge_is_enodata();
    test_zoff_shifts_only_placed_nodes();
    test_rejects_bad_arguments();

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
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
```

Expected: FAIL — `src/apos_geom.c` does not exist yet (`No such file or directory`).

- [ ] **Step 4: Implement the seed**

Create `src/apos_geom.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_geom.h.
 */

#include "apos_geom.h"

#include <errno.h>
#include <math.h>
#include <string.h>

/* Below this a length is treated as degenerate rather than divided by. At
 * 1 um it is far under any real ranging resolution, so it only ever fires on
 * genuinely coincident or collinear inputs. */
#define EPS 1.0e-6f

/* Find the edge joining a and b in either order. Returns its distance, or a
 * negative value if the pair was not measured. The linear scan is deliberate:
 * n_edges is at most 28, and an index would cost more code than it saves. */
static float edge_d(const struct apos_edge *e, uint16_t n, uint8_t a, uint8_t b)
{
	for (uint16_t k = 0; k < n; k++) {
		if ((e[k].i == a && e[k].j == b) || (e[k].i == b && e[k].j == a)) {
			return e[k].d_m;
		}
	}
	return -1.0f;
}

static void vsub(const float a[3], const float b[3], float out[3])
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

static float vdot(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float vnorm(const float a[3])
{
	return sqrtf(vdot(a, a));
}

static void vscale(float a[3], float s)
{
	a[0] *= s;
	a[1] *= s;
	a[2] *= s;
}

static void vcross(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

/* Intersect three spheres centred on p[0..2] with radii r[0..2].
 *
 * Writes the two solutions mirrored about the plane through the three centres
 * into sa (the +normal side) and sb. Returns 0, or -EDOM if the centres are
 * collinear or the spheres do not reach a common point.
 *
 * A non-intersection is clamped rather than rejected when it is within
 * measurement noise: real ranges are noisy and a layout that is geometrically
 * just barely non-intersecting still has a well-defined nearest point, which
 * the LM refinement will then pull into place. Only a gross miss is -EDOM. */
static int trilaterate3(const float p[3][3], const float r[3],
			float sa[3], float sb[3])
{
	float ex[3], ey[3], ez[3], t[3], p1p0[3], p2p0[3];
	float d, i, j;

	vsub(p[1], p[0], p1p0);
	d = vnorm(p1p0);
	if (d < EPS) {
		return -EDOM;
	}
	ex[0] = p1p0[0]; ex[1] = p1p0[1]; ex[2] = p1p0[2];
	vscale(ex, 1.0f / d);

	vsub(p[2], p[0], p2p0);
	i = vdot(ex, p2p0);

	t[0] = p2p0[0] - i * ex[0];
	t[1] = p2p0[1] - i * ex[1];
	t[2] = p2p0[2] - i * ex[2];
	j = vnorm(t);
	if (j < EPS) {
		return -EDOM; /* the three centres are collinear */
	}
	ey[0] = t[0]; ey[1] = t[1]; ey[2] = t[2];
	vscale(ey, 1.0f / j);

	vcross(ex, ey, ez);

	float x = (r[0] * r[0] - r[1] * r[1] + d * d) / (2.0f * d);
	float y = (r[0] * r[0] - r[2] * r[2] + i * i + j * j) / (2.0f * j)
		  - (i / j) * x;
	float z2 = r[0] * r[0] - x * x - y * y;
	float z;

	if (z2 < 0.0f) {
		/* Tolerate a shortfall up to 10 % of the smallest radius as
		 * noise; anything larger means the three ranges genuinely
		 * disagree and placing the node would invent data. */
		float slack = 0.1f * r[0];

		if (-z2 > slack * slack) {
			return -EDOM;
		}
		z = 0.0f;
	} else {
		z = sqrtf(z2);
	}

	for (int k = 0; k < 3; k++) {
		sa[k] = p[0][k] + x * ex[k] + y * ey[k] + z * ez[k];
		sb[k] = p[0][k] + x * ex[k] + y * ey[k] - z * ez[k];
	}
	return 0;
}

bool apos_geom_gauge_valid(const struct apos_gauge *g, uint8_t n_nodes)
{
	if (!g || n_nodes < APOS_MIN_NODES || n_nodes > APOS_MAX_NODES) {
		return false;
	}

	const uint8_t idx[4] = {g->origin, g->xaxis, g->plane, g->up};

	for (int a = 0; a < 4; a++) {
		if (idx[a] >= n_nodes) {
			return false;
		}
		for (int b = a + 1; b < 4; b++) {
			if (idx[a] == idx[b]) {
				return false;
			}
		}
	}
	return true;
}

void apos_geom_zoff(struct apos_result *r, float dz)
{
	if (!r) {
		return;
	}
	for (uint8_t k = 0; k < r->n_nodes; k++) {
		if (r->node[k].state != APOS_NODE_UNPLACED) {
			r->node[k].z += dz;
		}
	}
}

/* Count edges from node `n` to already-placed nodes, filling nb[] with their
 * indices and rr[] with the measured distances. Returns the count, capped at
 * `cap`. */
static uint8_t placed_neighbours(const struct apos_edge *e, uint16_t n_edges,
				 const struct apos_result *r, uint8_t n,
				 uint8_t *nb, float *rr, uint8_t cap)
{
	uint8_t cnt = 0;

	for (uint16_t k = 0; k < n_edges && cnt < cap; k++) {
		uint8_t other;

		if (e[k].i == n) {
			other = e[k].j;
		} else if (e[k].j == n) {
			other = e[k].i;
		} else {
			continue;
		}
		if (other >= r->n_nodes) {
			continue;
		}
		if (r->node[other].state == APOS_NODE_UNPLACED) {
			continue;
		}
		nb[cnt] = other;
		rr[cnt] = e[k].d_m;
		cnt++;
	}
	return cnt;
}

static void node_xyz(const struct apos_result *r, uint8_t n, float out[3])
{
	out[0] = r->node[n].x;
	out[1] = r->node[n].y;
	out[2] = r->node[n].z;
}

/* Centroid of every currently placed node. Used only to break a 3-neighbour
 * reflection tie, so an approximate answer is fine. */
static void placed_centroid(const struct apos_result *r, float out[3])
{
	uint8_t cnt = 0;

	out[0] = out[1] = out[2] = 0.0f;
	for (uint8_t k = 0; k < r->n_nodes; k++) {
		if (r->node[k].state == APOS_NODE_UNPLACED) {
			continue;
		}
		out[0] += r->node[k].x;
		out[1] += r->node[k].y;
		out[2] += r->node[k].z;
		cnt++;
	}
	if (cnt) {
		vscale(out, 1.0f / (float)cnt);
	}
}

static void set_node(struct apos_result *r, uint8_t n, const float p[3],
		     enum apos_node_state st)
{
	r->node[n].x = p[0];
	r->node[n].y = p[1];
	r->node[n].z = p[2];
	r->node[n].state = (uint8_t)st;
	r->n_placed++;
	if (st == APOS_NODE_AMBIGUOUS) {
		r->n_ambiguous++;
	}
}

/* Place one node from its placed neighbours. Returns true if it was placed. */
static bool place_from_neighbours(const struct apos_edge *e, uint16_t n_edges,
				  struct apos_result *r, uint8_t n)
{
	uint8_t nb[APOS_MAX_NODES];
	float rr[APOS_MAX_NODES];
	uint8_t cnt = placed_neighbours(e, n_edges, r, n, nb, rr,
					APOS_MAX_NODES);

	if (cnt < 3) {
		return false;
	}

	float p[3][3], rad[3], sa[3], sb[3];

	for (int k = 0; k < 3; k++) {
		node_xyz(r, nb[k], p[k]);
		rad[k] = rr[k];
	}
	if (trilaterate3(p, rad, sa, sb) != 0) {
		return false;
	}

	if (cnt >= 4) {
		/* A fourth range breaks the mirror: keep whichever branch
		 * agrees with it better. This is the only disambiguation that
		 * uses measured data rather than a heuristic. */
		float p4[3], da, db;

		node_xyz(r, nb[3], p4);
		da = fabsf(vnorm((float[3]){sa[0] - p4[0], sa[1] - p4[1],
					    sa[2] - p4[2]}) - rr[3]);
		db = fabsf(vnorm((float[3]){sb[0] - p4[0], sb[1] - p4[1],
					    sb[2] - p4[2]}) - rr[3]);
		set_node(r, n, (da <= db) ? sa : sb, APOS_NODE_PLACED);
		return true;
	}

	/* Exactly three: nothing in this node's own edges can choose a side.
	 * Take the branch on the same side as the placed centroid -- anchors in
	 * one deployment are usually on one side of any three of their peers --
	 * and flag it, because acceptance must be able to reject the guess. */
	float c[3], nrm[3], u[3], v[3];

	placed_centroid(r, c);
	vsub(p[1], p[0], u);
	vsub(p[2], p[0], v);
	vcross(u, v, nrm);

	float cs = vdot((float[3]){c[0] - p[0][0], c[1] - p[0][1],
				   c[2] - p[0][2]}, nrm);
	float as = vdot((float[3]){sa[0] - p[0][0], sa[1] - p[0][1],
				   sa[2] - p[0][2]}, nrm);

	set_node(r, n, ((cs >= 0.0f) == (as >= 0.0f)) ? sa : sb,
		 APOS_NODE_AMBIGUOUS);
	return true;
}

int apos_geom_seed(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		   const struct apos_gauge *g, struct apos_result *out)
{
	if (!e || !out || !g) {
		return -EINVAL;
	}
	if (!apos_geom_gauge_valid(g, n_nodes)) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->n_nodes = n_nodes;

	float d01 = edge_d(e, n_edges, g->origin, g->xaxis);
	float d02 = edge_d(e, n_edges, g->origin, g->plane);
	float d12 = edge_d(e, n_edges, g->xaxis, g->plane);

	if (d01 <= 0.0f || d02 < 0.0f || d12 < 0.0f) {
		return -ENODATA;
	}

	/* origin at the origin, xaxis on +x: the frame's translation and two of
	 * its three rotations, by definition rather than by fitting. */
	set_node(out, g->origin, (float[3]){0.0f, 0.0f, 0.0f},
		 APOS_NODE_PLACED);
	set_node(out, g->xaxis, (float[3]){d01, 0.0f, 0.0f}, APOS_NODE_PLACED);

	/* plane in z = 0 with y > 0: two circles in the plane, +y branch. */
	float px = (d02 * d02 - d12 * d12 + d01 * d01) / (2.0f * d01);
	float py2 = d02 * d02 - px * px;
	float py = (py2 > 0.0f) ? sqrtf(py2) : 0.0f;

	set_node(out, g->plane, (float[3]){px, py, 0.0f}, APOS_NODE_PLACED);

	/* up: the only node whose reflection the operator resolved for us. */
	uint8_t nb[APOS_MAX_NODES];
	float rr[APOS_MAX_NODES];
	uint8_t cnt = placed_neighbours(e, n_edges, out, g->up, nb, rr,
					APOS_MAX_NODES);

	if (cnt < 3) {
		return -ENODATA;
	}

	float p[3][3], rad[3], sa[3], sb[3];

	for (int k = 0; k < 3; k++) {
		node_xyz(out, nb[k], p[k]);
		rad[k] = rr[k];
	}
	if (trilaterate3(p, rad, sa, sb) != 0) {
		return -ENODATA;
	}
	set_node(out, g->up, (sa[2] >= sb[2]) ? sa : sb, APOS_NODE_PLACED);

	/* Everything else, best-determined first: re-scan after each placement
	 * so a node that gains a fourth neighbour is placed unambiguously
	 * rather than being guessed at from three. */
	for (;;) {
		uint8_t best = APOS_MAX_NODES;
		uint8_t best_cnt = 0;

		for (uint8_t n = 0; n < n_nodes; n++) {
			if (out->node[n].state != APOS_NODE_UNPLACED) {
				continue;
			}
			uint8_t c = placed_neighbours(e, n_edges, out, n, nb,
						      rr, APOS_MAX_NODES);

			if (c >= 3 && c > best_cnt) {
				best = n;
				best_cnt = c;
			}
		}
		if (best == APOS_MAX_NODES) {
			break;
		}
		if (!place_from_neighbours(e, n_edges, out, best)) {
			/* Geometrically unplaceable despite having the edges;
			 * leave it unplaced and stop retrying it. */
			out->node[best].state = APOS_NODE_UNPLACED;
			break;
		}
	}

	return 0;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
./tests/apos_geom/test_apos_geom.exe
```

Expected: `PASSED`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/apos_geom.h src/apos_geom.c tests/apos_geom/test_apos_geom.c
git commit -m "feat(apos): gauge-locked closed-form seed for the anchor survey"
```

---

### Task 2: `apos_geom` — LM refinement, residuals, planarity

**Files:**
- Modify: `src/apos_geom.h` (add `APOS_LM_MAX_ITER`, `apos_geom_refine`, `apos_geom_solve`)
- Modify: `src/apos_geom.c`
- Modify: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Consumes: everything Task 1 produced.
- Produces: `int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges, const struct apos_gauge *g, struct apos_result *io)` and `int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes, const struct apos_gauge *g, struct apos_result *out)`. Both fill `rms_m`, `worst_edge_m`, `worst_i`, `worst_j`, `planarity_m`, `iterations` and every `node[].residual_m`.

- [ ] **Step 1: Add the declarations to the header**

Insert into `src/apos_geom.h` immediately before `#endif`:

```c
/* LM iteration cap. Reached only on a pathological input; a clean full mesh
 * converges in well under ten. */
#define APOS_LM_MAX_ITER 200

/* Refine an existing seed in place and fill every diagnostic field.
 *
 * The gauge is enforced by construction, not by penalty: the origin node
 * contributes no free parameters, xaxis only x, plane only x and y. There are
 * therefore 3N-6 free parameters and the fit cannot translate, rotate or mirror
 * the frame while minimising.
 *
 * Unplaced nodes, and any edge touching one, are excluded entirely.
 *
 * Returns 0, -EINVAL on a bad argument, or -ENODATA if no usable edge remains.
 * Non-convergence is NOT an error: the caller judges the result on rms_m, which
 * is what acceptance is defined against. */
int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io);

/* apos_geom_seed() then apos_geom_refine(). The normal entry point. */
int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		    const struct apos_gauge *g, struct apos_result *out);
```

- [ ] **Step 2: Write the failing tests**

Add to `tests/apos_geom/test_apos_geom.c` before `main()`:

```c
/* Deterministic PRNG: a test must not depend on the host rand(), or the same
 * plan yields different results on different machines. */
static uint32_t prng_state = 12345u;

static float noise_m(float amplitude)
{
    prng_state = prng_state * 1664525u + 1013904223u;
    float u = (float)(prng_state >> 8) / (float)(1u << 24);

    return (2.0f * u - 1.0f) * amplitude;
}

static void test_refine_leaves_an_exact_solution_alone(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.rms_m < 1e-4f);
    for (int i = 0; i < 4; i++) {
        CLOSE(r.node[i].x, ref_xyz[i][0], 1e-3f);
        CLOSE(r.node[i].y, ref_xyz[i][1], 1e-3f);
        CLOSE(r.node[i].z, ref_xyz[i][2], 1e-3f);
    }
}

/* 20 mm of range noise must stay inside a few centimetres of coordinate error,
 * and rms_m must land in the same ballpark as the noise that caused it -- that
 * correspondence is what makes rms_m usable as an acceptance criterion. */
static void test_refine_absorbs_realistic_noise(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    prng_state = 999u;
    for (uint16_t k = 0; k < n; k++) {
        e[k].d_m += noise_m(0.020f);
        e[k].sd_m = 0.020f;
    }

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.n_placed == 4);
    CHECK(r.rms_m < 0.030f);
    for (int i = 0; i < 4; i++) {
        CLOSE(r.node[i].x, ref_xyz[i][0], 0.06f);
        CLOSE(r.node[i].y, ref_xyz[i][1], 0.06f);
        CLOSE(r.node[i].z, ref_xyz[i][2], 0.06f);
    }
}

/* The gauge must survive refinement exactly, not approximately: those
 * coordinates contribute no free parameters at all. */
static void test_refine_preserves_the_gauge_exactly(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    prng_state = 4242u;
    for (uint16_t k = 0; k < n; k++) {
        e[k].d_m += noise_m(0.050f);
        e[k].sd_m = 0.050f;
    }

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CLOSE(r.node[0].x, 0.0f, 1e-6f);
    CLOSE(r.node[0].y, 0.0f, 1e-6f);
    CLOSE(r.node[0].z, 0.0f, 1e-6f);
    CLOSE(r.node[1].y, 0.0f, 1e-6f);
    CLOSE(r.node[1].z, 0.0f, 1e-6f);
    CLOSE(r.node[2].z, 0.0f, 1e-6f);
}

/* One bad edge drags the fit -- that is what least squares does -- but
 * worst_i/worst_j must name the culprit so the operator re-ranges that pair
 * instead of re-running everything. */
static void test_worst_edge_names_the_bad_pair(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    for (uint16_t k = 0; k < n; k++) {
        if ((e[k].i == 1 && e[k].j == 2) || (e[k].i == 2 && e[k].j == 1)) {
            e[k].d_m += 0.40f;
        }
    }

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.worst_edge_m > 0.05f);
    CHECK((r.worst_i == 1 && r.worst_j == 2) ||
          (r.worst_i == 2 && r.worst_j == 1));
}

/* A ceiling-mounted array is nearly coplanar, which is exactly when z stops
 * meaning anything. planarity_m is the number that has to say so. */
static void test_planarity_is_small_for_a_coplanar_array(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t k = 0;
    const float flat[4][3] = {
        {0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f}, {3.0f, 3.0f, 0.0f},
    };

    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 4; j++) {
            e[k].i = i;
            e[k].j = j;
            e[k].d_m = dist3(flat[i], flat[j]);
            e[k].sd_m = 0.010f;
            k++;
        }
    }

    CHECK(apos_geom_solve(e, k, 4, &g_ref, &r) == 0);
    CHECK(r.planarity_m < 0.02f);
}

/* A genuinely 3D array must NOT be flagged, or the diagnostic is useless. */
static void test_planarity_is_large_for_a_3d_array(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.planarity_m > 0.30f);
}

/* Sparse: 6 nodes with two pairs unmeasured -- the case that killed the
 * sequential-bootstrap approach this design replaces. */
static void test_solves_a_sparse_mesh_with_holes(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t k = 0;
    const float six[6][3] = {
        {0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}, {0.0f, 6.0f, 0.0f},
        {1.0f, 1.0f, 2.0f}, {5.0f, 6.0f, 0.5f}, {2.5f, 3.0f, 1.0f},
    };

    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 6; j++) {
            if ((i == 1 && j == 2) || (i == 0 && j == 4)) {
                continue;   /* out of range of each other */
            }
            e[k].i = i;
            e[k].j = j;
            e[k].d_m = dist3(six[i], six[j]);
            e[k].sd_m = 0.010f;
            k++;
        }
    }

    CHECK(apos_geom_solve(e, k, 6, &g_ref, &r) == 0);
    CHECK(r.n_placed == 6);
    CHECK(r.rms_m < 0.01f);
    for (int i = 0; i < 6; i++) {
        CLOSE(r.node[i].x, six[i][0], 0.02f);
        CLOSE(r.node[i].y, six[i][1], 0.02f);
        CLOSE(r.node[i].z, six[i][2], 0.02f);
    }
}

/* An unplaced node must not poison the fit for the nodes that were measured. */
static void test_unplaced_node_does_not_break_the_fit(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    e[n].i = 0; e[n].j = 4; e[n].d_m = 2.0f; e[n].sd_m = 0.010f; n++;

    CHECK(apos_geom_solve(e, n, 5, &g_ref, &r) == 0);
    CHECK(r.n_placed == 4);
    CHECK(r.node[4].state == APOS_NODE_UNPLACED);
    CHECK(r.rms_m < 1e-3f);
}
```

Register in `main()` after the existing calls:

```c
    test_refine_leaves_an_exact_solution_alone();
    test_refine_absorbs_realistic_noise();
    test_refine_preserves_the_gauge_exactly();
    test_worst_edge_names_the_bad_pair();
    test_planarity_is_small_for_a_coplanar_array();
    test_planarity_is_large_for_a_3d_array();
    test_solves_a_sparse_mesh_with_holes();
    test_unplaced_node_does_not_break_the_fit();
```

- [ ] **Step 3: Run to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
```

Expected: FAIL — `undefined reference to apos_geom_solve` and `apos_geom_refine`.

- [ ] **Step 4: Implement the refinement**

Add near the top of `src/apos_geom.c`, after the existing includes:

```c
/* M_PI is not in ISO C; gcc exposes it from math.h only with GNU extensions
 * enabled. Defined here so the host build does not depend on the -std= flag. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
```

Append to `src/apos_geom.c`:

```c
/* ---- Levenberg-Marquardt refinement ---- */

/* Free-parameter map. The gauge is structural: origin has no free parameters,
 * xaxis only x, plane only x and y. -1 means "this coordinate is fixed". */
struct pmap {
	int8_t  idx[APOS_MAX_NODES][3];
	uint8_t n_params;
};

static void pmap_build(const struct apos_result *r, const struct apos_gauge *g,
		       struct pmap *m)
{
	uint8_t p = 0;

	for (uint8_t n = 0; n < APOS_MAX_NODES; n++) {
		m->idx[n][0] = m->idx[n][1] = m->idx[n][2] = -1;
	}

	for (uint8_t n = 0; n < r->n_nodes; n++) {
		if (r->node[n].state == APOS_NODE_UNPLACED || n == g->origin) {
			continue;
		}
		m->idx[n][0] = (int8_t)p++;
		if (n == g->xaxis) {
			continue;
		}
		m->idx[n][1] = (int8_t)p++;
		if (n == g->plane) {
			continue;
		}
		m->idx[n][2] = (int8_t)p++;
	}
	m->n_params = p;
}

static bool edge_usable(const struct apos_result *r, const struct apos_edge *e)
{
	if (e->i >= r->n_nodes || e->j >= r->n_nodes || e->i == e->j) {
		return false;
	}
	return r->node[e->i].state != APOS_NODE_UNPLACED &&
	       r->node[e->j].state != APOS_NODE_UNPLACED;
}

/* Floored so a zero-variance edge cannot produce an infinite weight. */
static float edge_sd(const struct apos_edge *e)
{
	return (e->sd_m > 1.0e-4f) ? e->sd_m : 1.0e-4f;
}

static float cost(const struct apos_edge *e, uint16_t n_edges,
		  const struct apos_result *r)
{
	float c = 0.0f;

	for (uint16_t k = 0; k < n_edges; k++) {
		if (!edge_usable(r, &e[k])) {
			continue;
		}

		float a[3], b[3], d[3];

		node_xyz(r, e[k].i, a);
		node_xyz(r, e[k].j, b);
		vsub(a, b, d);

		float res = (vnorm(d) - e[k].d_m) / edge_sd(&e[k]);

		c += res * res;
	}
	return c;
}

/* Solve A x = b in place, Gauss elimination with partial pivoting. n <= 18
 * here (3*8-6), so a dense O(n^3) solve is trivial and a sparse one would be
 * pure complexity. False on a singular system, which LM answers by raising
 * lambda. */
static bool solve_dense(float A[][3 * APOS_MAX_NODES], float *b, uint8_t n)
{
	for (uint8_t c = 0; c < n; c++) {
		uint8_t piv = c;

		for (uint8_t rr = (uint8_t)(c + 1); rr < n; rr++) {
			if (fabsf(A[rr][c]) > fabsf(A[piv][c])) {
				piv = rr;
			}
		}
		if (fabsf(A[piv][c]) < 1.0e-12f) {
			return false;
		}
		if (piv != c) {
			for (uint8_t k = 0; k < n; k++) {
				float t = A[c][k];

				A[c][k] = A[piv][k];
				A[piv][k] = t;
			}

			float t = b[c];

			b[c] = b[piv];
			b[piv] = t;
		}
		for (uint8_t rr = (uint8_t)(c + 1); rr < n; rr++) {
			float f = A[rr][c] / A[c][c];

			if (f == 0.0f) {
				continue;
			}
			for (uint8_t k = c; k < n; k++) {
				A[rr][k] -= f * A[c][k];
			}
			b[rr] -= f * b[c];
		}
	}
	for (int8_t rr = (int8_t)(n - 1); rr >= 0; rr--) {
		float s = b[rr];

		for (uint8_t k = (uint8_t)(rr + 1); k < n; k++) {
			s -= A[rr][k] * b[k];
		}
		b[rr] = s / A[rr][rr];
	}
	return true;
}

static void apply_step(struct apos_result *r, const struct pmap *m,
		       const float *step)
{
	for (uint8_t n = 0; n < r->n_nodes; n++) {
		if (m->idx[n][0] >= 0) {
			r->node[n].x += step[m->idx[n][0]];
		}
		if (m->idx[n][1] >= 0) {
			r->node[n].y += step[m->idx[n][1]];
		}
		if (m->idx[n][2] >= 0) {
			r->node[n].z += step[m->idx[n][2]];
		}
	}
}

/* Smallest eigenvalue of a symmetric 3x3, closed-form trigonometric solution.
 * Used only for the planarity metric, where the matrix is a covariance and so
 * positive semi-definite. */
static float smallest_eig_sym3(const float A[3][3])
{
	float p1 = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];

	if (p1 <= 0.0f) {
		float mn = A[0][0];

		if (A[1][1] < mn) {
			mn = A[1][1];
		}
		if (A[2][2] < mn) {
			mn = A[2][2];
		}
		return mn;
	}

	float q = (A[0][0] + A[1][1] + A[2][2]) / 3.0f;
	float d0 = A[0][0] - q, d1 = A[1][1] - q, d2 = A[2][2] - q;
	float p2 = d0 * d0 + d1 * d1 + d2 * d2 + 2.0f * p1;
	float p = sqrtf(p2 / 6.0f);

	if (p < EPS) {
		return q;
	}

	float B[3][3];

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			B[i][j] = (A[i][j] - ((i == j) ? q : 0.0f)) / p;
		}
	}

	float det = B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1])
		  - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0])
		  + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0]);
	float rr = det / 2.0f;

	if (rr < -1.0f) {
		rr = -1.0f;
	} else if (rr > 1.0f) {
		rr = 1.0f;
	}

	/* The largest root sits at phi; the smallest is one third of a turn on. */
	return q + 2.0f * p * cosf(acosf(rr) / 3.0f + 2.0f * (float)M_PI / 3.0f);
}

/* RMS distance of the placed nodes from their own best-fit plane. Near zero
 * means the array is coplanar and the solved z values carry no information. */
static float planarity(const struct apos_result *r)
{
	float c[3] = {0.0f, 0.0f, 0.0f};
	uint8_t cnt = 0;

	for (uint8_t n = 0; n < r->n_nodes; n++) {
		if (r->node[n].state == APOS_NODE_UNPLACED) {
			continue;
		}
		c[0] += r->node[n].x;
		c[1] += r->node[n].y;
		c[2] += r->node[n].z;
		cnt++;
	}
	if (cnt < 4) {
		/* Three or fewer points are always exactly coplanar, so the
		 * metric would be identically zero and would mean nothing. */
		return 0.0f;
	}
	vscale(c, 1.0f / (float)cnt);

	float C[3][3] = {{0}};

	for (uint8_t n = 0; n < r->n_nodes; n++) {
		if (r->node[n].state == APOS_NODE_UNPLACED) {
			continue;
		}

		float v[3] = {r->node[n].x - c[0], r->node[n].y - c[1],
			      r->node[n].z - c[2]};

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				C[i][j] += v[i] * v[j];
			}
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			C[i][j] /= (float)cnt;
		}
	}

	float ev = smallest_eig_sym3(C);

	return (ev > 0.0f) ? sqrtf(ev) : 0.0f;
}

static void fill_diagnostics(const struct apos_edge *e, uint16_t n_edges,
			     struct apos_result *r)
{
	float sum = 0.0f;
	uint16_t used = 0;
	float node_sum[APOS_MAX_NODES] = {0};
	uint16_t node_cnt[APOS_MAX_NODES] = {0};

	r->worst_edge_m = 0.0f;
	r->worst_i = 0;
	r->worst_j = 0;

	for (uint16_t k = 0; k < n_edges; k++) {
		if (!edge_usable(r, &e[k])) {
			continue;
		}

		float a[3], b[3], d[3];

		node_xyz(r, e[k].i, a);
		node_xyz(r, e[k].j, b);
		vsub(a, b, d);

		/* Unweighted, deliberately: rms_m is compared against a
		 * millimetre threshold by a human, so it has to be a distance
		 * and not a chi-square. */
		float res = vnorm(d) - e[k].d_m;
		float ares = fabsf(res);

		sum += res * res;
		used++;

		node_sum[e[k].i] += res * res;
		node_cnt[e[k].i]++;
		node_sum[e[k].j] += res * res;
		node_cnt[e[k].j]++;

		if (ares > r->worst_edge_m) {
			r->worst_edge_m = ares;
			r->worst_i = e[k].i;
			r->worst_j = e[k].j;
		}
	}

	r->rms_m = used ? sqrtf(sum / (float)used) : 0.0f;

	for (uint8_t n = 0; n < r->n_nodes; n++) {
		r->node[n].residual_m = node_cnt[n]
			? sqrtf(node_sum[n] / (float)node_cnt[n]) : 0.0f;
	}

	r->planarity_m = planarity(r);
}

int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io)
{
	if (!e || !g || !io) {
		return -EINVAL;
	}
	if (!apos_geom_gauge_valid(g, io->n_nodes)) {
		return -EINVAL;
	}

	struct pmap m;
	uint16_t usable = 0;

	pmap_build(io, g, &m);

	for (uint16_t k = 0; k < n_edges; k++) {
		if (edge_usable(io, &e[k])) {
			usable++;
		}
	}
	if (usable == 0) {
		return -ENODATA;
	}
	if (m.n_params == 0) {
		fill_diagnostics(e, n_edges, io);
		return 0;
	}

	float lambda = 1.0e-3f;
	float c_cur = cost(e, n_edges, io);

	io->iterations = 0;

	for (uint16_t it = 0; it < APOS_LM_MAX_ITER; it++) {
		static float JtJ[3 * APOS_MAX_NODES][3 * APOS_MAX_NODES];
		static float A[3 * APOS_MAX_NODES][3 * APOS_MAX_NODES];
		float Jtr[3 * APOS_MAX_NODES] = {0};
		float bvec[3 * APOS_MAX_NODES];

		/* static, not automatic: two 18x18 float matrices are 2.6 kB
		 * and CONFIG_MAIN_STACK_SIZE is 4096. This runs only from the
		 * gateway loop, single-threaded, so there is no reentrancy. */
		memset(JtJ, 0, sizeof(JtJ));

		for (uint16_t k = 0; k < n_edges; k++) {
			if (!edge_usable(io, &e[k])) {
				continue;
			}

			float a[3], b[3], d[3];

			node_xyz(io, e[k].i, a);
			node_xyz(io, e[k].j, b);
			vsub(a, b, d);

			float len = vnorm(d);

			if (len < EPS) {
				continue; /* coincident: no usable gradient */
			}

			float w = 1.0f / edge_sd(&e[k]);
			float res = (len - e[k].d_m) * w;
			int8_t col[6];
			float grad[6];
			uint8_t nc = 0;

			/* d(len)/d(a) = (a-b)/len, opposite sign for b. Only
			 * free coordinates get a column. */
			for (int axis = 0; axis < 3; axis++) {
				if (m.idx[e[k].i][axis] >= 0) {
					col[nc] = m.idx[e[k].i][axis];
					grad[nc] = (d[axis] / len) * w;
					nc++;
				}
				if (m.idx[e[k].j][axis] >= 0) {
					col[nc] = m.idx[e[k].j][axis];
					grad[nc] = -(d[axis] / len) * w;
					nc++;
				}
			}

			for (uint8_t p = 0; p < nc; p++) {
				Jtr[col[p]] -= grad[p] * res;
				for (uint8_t q = 0; q < nc; q++) {
					JtJ[col[p]][col[q]] += grad[p] * grad[q];
				}
			}
		}

		/* Damping scaled per parameter, so the step is invariant to how
		 * differently conditioned the axes are. */
		for (uint8_t p = 0; p < m.n_params; p++) {
			for (uint8_t q = 0; q < m.n_params; q++) {
				A[p][q] = JtJ[p][q];
			}
			A[p][p] += lambda *
				   (JtJ[p][p] > 0.0f ? JtJ[p][p] : 1.0f);
			bvec[p] = Jtr[p];
		}

		if (!solve_dense(A, bvec, m.n_params)) {
			lambda *= 10.0f;
			if (lambda > 1.0e12f) {
				break;
			}
			continue;
		}

		struct apos_result trial = *io;

		apply_step(&trial, &m, bvec);

		float c_new = cost(e, n_edges, &trial);

		io->iterations = (uint16_t)(it + 1);

		if (c_new < c_cur) {
			float rel = (c_cur - c_new) / (c_cur + EPS);

			*io = trial;
			c_cur = c_new;
			lambda *= 0.1f;
			if (lambda < 1.0e-12f) {
				lambda = 1.0e-12f;
			}
			if (rel < 1.0e-9f) {
				break; /* converged */
			}
		} else {
			lambda *= 10.0f;
			if (lambda > 1.0e12f) {
				break; /* no further progress available */
			}
		}
	}

	fill_diagnostics(e, n_edges, io);
	return 0;
}

int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		    const struct apos_gauge *g, struct apos_result *out)
{
	int rc = apos_geom_seed(e, n_edges, n_nodes, g, out);

	if (rc) {
		return rc;
	}
	return apos_geom_refine(e, n_edges, g, out);
}
```

- [ ] **Step 5: Run to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
./tests/apos_geom/test_apos_geom.exe
```

Expected: `PASSED`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/apos_geom.h src/apos_geom.c tests/apos_geom/test_apos_geom.c
git commit -m "feat(apos): LM refinement, edge residuals and the coplanarity metric"
```

---

### Task 3: `apos_table` — peer table, directed measurements, symmetrisation

**Files:**
- Create: `src/apos_table.h`, `src/apos_table.c`
- Test: `tests/apos_table/test_apos_table.c`

**Interfaces:**
- Consumes: `APOS_MAX_NODES`, `struct apos_edge` from `apos_geom.h`.
- Produces: `APOS_EUI_LEN` (8), `APOS_MAX_MEAS`, `APOS_SD_FLOOR_MM` (1), `APOS_ONEWAY_SD_INFLATE` (2.0f), `struct apos_peer`, `struct apos_dir_meas`, `struct apos_table`, and the functions `apos_table_init`, `apos_table_add_peer`, `apos_table_find_addr`, `apos_table_find_eui`, `apos_table_add_meas`, `apos_table_symmetrise`, `apos_table_missing_pairs`.

- [ ] **Step 1: Write the header**

Create `src/apos_table.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's working set for one survey: which anchors answered enumeration,
 * every directed range measurement reported back, and the reduction of those
 * into the undirected edges apos_geom.c fits.
 *
 * Peers are keyed by EUI-64, not by anchor_id or short address. That is the
 * whole point: an `anchor id` swap once left coordinates stranded on the wrong
 * board because the deployment's state was keyed by id. EUI-64 is assigned at
 * manufacture and travels with the board.
 *
 * Pure C -- no Zephyr -- so the accumulation and symmetrisation rules are
 * host-testable.
 */

#ifndef APOS_TABLE_H
#define APOS_TABLE_H

#include "apos_geom.h"

#include <stdbool.h>
#include <stdint.h>

#define APOS_EUI_LEN 8

/* Every ordered pair, since both directions of each pair are measured
 * separately: N*(N-1) = 56 at APOS_MAX_NODES. */
#define APOS_MAX_MEAS (APOS_MAX_NODES * (APOS_MAX_NODES - 1))

/* A reported sd of 0 would give an edge infinite weight and let one pair
 * dictate the entire fit. 1 mm is far below this hardware's real resolution, so
 * the floor never binds on an honest measurement. */
#define APOS_SD_FLOOR_MM 1u

/* Applied to an edge measured in only ONE direction. Averaging A->B with B->A
 * is what cancels antenna-delay asymmetry; with a single direction that error
 * is still in the number, so the edge is trusted less rather than being
 * silently treated as equal in quality to a symmetrised one. */
#define APOS_ONEWAY_SD_INFLATE 2.0f

struct apos_peer {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t short_addr;
	bool     pos_valid; /* what the anchor reported at enumeration */
	float    x, y, z;   /* its position BEFORE this survey */
};

/* One anchor's report of ranging one peer: `from` polled `to`. */
struct apos_dir_meas {
	uint8_t  from;    /* node index of the initiator */
	uint8_t  to;      /* node index of the responder */
	int32_t  mean_mm;
	uint16_t sd_mm;
	uint8_t  n_ok;    /* exchanges that produced a distance */
};

struct apos_table {
	struct apos_peer     peer[APOS_MAX_NODES];
	uint8_t              n_peers;
	struct apos_dir_meas meas[APOS_MAX_MEAS];
	uint16_t             n_meas;
};

void apos_table_init(struct apos_table *t);

/* Register an enumerated anchor. Returns its node index (>= 0).
 *
 * Idempotent on EUI: re-registering the same EUI updates its fields and returns
 * the same index, so repeated SURVEY_BEGIN broadcasts are harmless.
 *
 * Returns -EADDRINUSE if short_addr is already claimed by a DIFFERENT EUI --
 * two boards configured with the same `anchor id`. That is a fault which today
 * produces silently wrong ranging with no indication anywhere, so it is
 * reported rather than tolerated. Returns -ENOSPC when the table is full. */
int apos_table_add_peer(struct apos_table *t, const uint8_t eui[APOS_EUI_LEN],
			uint16_t short_addr, bool pos_valid,
			float x, float y, float z);

/* Node index, or -ENOENT. */
int apos_table_find_addr(const struct apos_table *t, uint16_t short_addr);
int apos_table_find_eui(const struct apos_table *t,
			const uint8_t eui[APOS_EUI_LEN]);

/* Record one directed measurement. A repeat of the same (from, to) replaces the
 * earlier one, so a retried RANGE_CMD does not double-count.
 *
 * Returns 0, -EINVAL on a bad index pair, or -ENOSPC. */
int apos_table_add_meas(struct apos_table *t, uint8_t from, uint8_t to,
			int32_t mean_mm, uint16_t sd_mm, uint8_t n_ok);

/* Reduce the directed measurements to undirected edges in metres.
 *
 * Measurements with n_ok < min_n_ok are discarded as too thin to trust. A pair
 * with both directions surviving is combined inverse-variance weighted; a pair
 * with one direction is kept with its sd inflated by APOS_ONEWAY_SD_INFLATE; a
 * pair with none produces no edge at all.
 *
 * Returns the number of edges written, never more than out_cap. */
uint16_t apos_table_symmetrise(const struct apos_table *t,
			       struct apos_edge *out, uint16_t out_cap,
			       uint8_t min_n_ok);

/* Unordered pairs with no usable measurement in either direction. These are the
 * holes the fit works around, and the number an operator needs to see before
 * deciding whether to move an anchor. */
uint16_t apos_table_missing_pairs(const struct apos_table *t, uint8_t min_n_ok);

#endif /* APOS_TABLE_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/apos_table/test_apos_table.c`:

```c
#include "apos_table.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

#define CLOSE(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

/* EUIs that differ only in the last byte, so a comparison bug that only looks
 * at the first bytes is caught. */
static void mk_eui(uint8_t out[APOS_EUI_LEN], uint8_t tail)
{
    for (int i = 0; i < APOS_EUI_LEN - 1; i++) {
        out[i] = (uint8_t)(0xA0 + i);
    }
    out[APOS_EUI_LEN - 1] = tail;
}

static int add(struct apos_table *t, uint8_t tail, uint16_t addr)
{
    uint8_t eui[APOS_EUI_LEN];

    mk_eui(eui, tail);
    return apos_table_add_peer(t, eui, addr, false, 0.0f, 0.0f, 0.0f);
}

static void test_peers_get_sequential_indices(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 2, 0x0002) == 1);
    CHECK(add(&t, 3, 0x0003) == 2);
    CHECK(t.n_peers == 3);
}

/* Repeated SURVEY_BEGIN broadcasts mean the same anchor answers more than
 * once. That must be idempotent, not a duplicate row. */
static void test_readding_the_same_eui_is_idempotent(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(t.n_peers == 1);
}

/* The duplicate-anchor-id fault. Two distinct boards claiming one short address
 * must be reported, because everything downstream addresses by short address. */
static void test_same_addr_from_a_different_eui_is_reported(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0002) == 0);
    CHECK(add(&t, 9, 0x0002) == -EADDRINUSE);
    CHECK(t.n_peers == 1);
}

/* An anchor that had its id changed between surveys keeps its identity: same
 * EUI, new address, same node index. */
static void test_same_eui_with_a_new_addr_updates_in_place(void)
{
    struct apos_table t;

    apos_table_init(&t);
    CHECK(add(&t, 1, 0x0001) == 0);
    CHECK(add(&t, 1, 0x0004) == 0);
    CHECK(t.n_peers == 1);
    CHECK(t.peer[0].short_addr == 0x0004);
}

static void test_lookup_by_addr_and_eui(void)
{
    struct apos_table t;
    uint8_t eui[APOS_EUI_LEN];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_find_addr(&t, 0x0002) == 1);
    CHECK(apos_table_find_addr(&t, 0x0009) == -ENOENT);

    mk_eui(eui, 2);
    CHECK(apos_table_find_eui(&t, eui) == 1);
    mk_eui(eui, 7);
    CHECK(apos_table_find_eui(&t, eui) == -ENOENT);
}

static void test_table_full_is_enospc(void)
{
    struct apos_table t;

    apos_table_init(&t);
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        CHECK(add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k)) == k);
    }
    CHECK(add(&t, 200, 0x00FF) == -ENOSPC);
}

/* A retried RANGE_CMD must replace, not accumulate: two rows for one direction
 * would double that direction's weight in the fit. */
static void test_repeat_measurement_replaces(void)
{
    struct apos_table t;

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_add_meas(&t, 0, 1, 1000, 10, 40) == 0);
    CHECK(apos_table_add_meas(&t, 0, 1, 2000, 20, 38) == 0);
    CHECK(t.n_meas == 1);
    CHECK(t.meas[0].mean_mm == 2000);
    CHECK(t.meas[0].n_ok == 38);
}

static void test_measurement_rejects_bad_indices(void)
{
    struct apos_table t;

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);

    CHECK(apos_table_add_meas(&t, 0, 0, 1000, 10, 40) == -EINVAL);
    CHECK(apos_table_add_meas(&t, 0, 5, 1000, 10, 40) == -EINVAL);
}

/* Both directions average, and the combined sd must be tighter than either --
 * that improvement is the reason both directions are measured. */
static void test_both_directions_average_and_tighten(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 40);
    apos_table_add_meas(&t, 1, 0, 1040, 20, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CLOSE(e[0].d_m, 1.020f, 1e-4f);
    CHECK(e[0].sd_m < 0.020f);
}

/* One direction only: kept, but explicitly trusted less. */
static void test_one_direction_is_kept_with_inflated_sd(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CLOSE(e[0].d_m, 1.000f, 1e-4f);
    CLOSE(e[0].sd_m, 0.020f * APOS_ONEWAY_SD_INFLATE, 1e-4f);
}

/* A handful of successful exchanges is not a measurement. Dropping it is what
 * keeps a pair that barely reaches out of the geometry. */
static void test_thin_measurements_are_discarded(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 20, 3);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 0);
    CHECK(apos_table_missing_pairs(&t, 10) == 1);
}

/* A zero sd must not give one edge infinite weight. */
static void test_zero_sd_is_floored(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    add(&t, 1, 0x0001);
    add(&t, 2, 0x0002);
    apos_table_add_meas(&t, 0, 1, 1000, 0, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 1);
    CHECK(e[0].sd_m > 0.0f);
}

static void test_missing_pairs_counts_holes(void)
{
    struct apos_table t;
    struct apos_edge e[APOS_MAX_EDGES];

    apos_table_init(&t);
    for (uint8_t k = 0; k < 4; k++) {
        add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k));
    }
    /* Measure 5 of the 6 pairs, one direction each. */
    apos_table_add_meas(&t, 0, 1, 1000, 10, 40);
    apos_table_add_meas(&t, 0, 2, 1000, 10, 40);
    apos_table_add_meas(&t, 0, 3, 1000, 10, 40);
    apos_table_add_meas(&t, 1, 2, 1000, 10, 40);
    apos_table_add_meas(&t, 1, 3, 1000, 10, 40);

    CHECK(apos_table_symmetrise(&t, e, APOS_MAX_EDGES, 10) == 5);
    CHECK(apos_table_missing_pairs(&t, 10) == 1);
}

/* Symmetrise must respect out_cap rather than overrunning the caller. */
static void test_symmetrise_respects_out_cap(void)
{
    struct apos_table t;
    struct apos_edge e[2];

    apos_table_init(&t);
    for (uint8_t k = 0; k < 4; k++) {
        add(&t, (uint8_t)(k + 1), (uint16_t)(0x0001 + k));
    }
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 4; j++) {
            apos_table_add_meas(&t, i, j, 1000, 10, 40);
        }
    }

    CHECK(apos_table_symmetrise(&t, e, 2, 10) == 2);
}

int main(void)
{
    test_peers_get_sequential_indices();
    test_readding_the_same_eui_is_idempotent();
    test_same_addr_from_a_different_eui_is_reported();
    test_same_eui_with_a_new_addr_updates_in_place();
    test_lookup_by_addr_and_eui();
    test_table_full_is_enospc();
    test_repeat_measurement_replaces();
    test_measurement_rejects_bad_indices();
    test_both_directions_average_and_tighten();
    test_one_direction_is_kept_with_inflated_sd();
    test_thin_measurements_are_discarded();
    test_zero_sd_is_floored();
    test_missing_pairs_counts_holes();
    test_symmetrise_respects_out_cap();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}
```

- [ ] **Step 3: Run to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_table/test_apos_table.exe tests/apos_table/test_apos_table.c src/apos_table.c src/apos_geom.c -lm
```

Expected: FAIL — `src/apos_table.c` does not exist.

- [ ] **Step 4: Implement**

Create `src/apos_table.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_table.h.
 */

#include "apos_table.h"

#include <errno.h>
#include <string.h>

void apos_table_init(struct apos_table *t)
{
	if (!t) {
		return;
	}
	memset(t, 0, sizeof(*t));
}

int apos_table_find_eui(const struct apos_table *t,
			const uint8_t eui[APOS_EUI_LEN])
{
	if (!t || !eui) {
		return -EINVAL;
	}
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (memcmp(t->peer[k].eui, eui, APOS_EUI_LEN) == 0) {
			return k;
		}
	}
	return -ENOENT;
}

int apos_table_find_addr(const struct apos_table *t, uint16_t short_addr)
{
	if (!t) {
		return -EINVAL;
	}
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (t->peer[k].short_addr == short_addr) {
			return k;
		}
	}
	return -ENOENT;
}

int apos_table_add_peer(struct apos_table *t, const uint8_t eui[APOS_EUI_LEN],
			uint16_t short_addr, bool pos_valid,
			float x, float y, float z)
{
	if (!t || !eui) {
		return -EINVAL;
	}

	int existing = apos_table_find_eui(t, eui);

	/* A different board already claims this address. Checked before the
	 * insert so the table is never left holding two rows for one address --
	 * every later step addresses peers by short address. */
	for (uint8_t k = 0; k < t->n_peers; k++) {
		if (t->peer[k].short_addr == short_addr &&
		    (existing < 0 || (int)k != existing)) {
			return -EADDRINUSE;
		}
	}

	uint8_t slot;

	if (existing >= 0) {
		slot = (uint8_t)existing;
	} else {
		if (t->n_peers >= APOS_MAX_NODES) {
			return -ENOSPC;
		}
		slot = t->n_peers++;
		memcpy(t->peer[slot].eui, eui, APOS_EUI_LEN);
	}

	t->peer[slot].short_addr = short_addr;
	t->peer[slot].pos_valid = pos_valid;
	t->peer[slot].x = x;
	t->peer[slot].y = y;
	t->peer[slot].z = z;

	return slot;
}

int apos_table_add_meas(struct apos_table *t, uint8_t from, uint8_t to,
			int32_t mean_mm, uint16_t sd_mm, uint8_t n_ok)
{
	if (!t || from == to || from >= t->n_peers || to >= t->n_peers) {
		return -EINVAL;
	}

	for (uint16_t k = 0; k < t->n_meas; k++) {
		if (t->meas[k].from == from && t->meas[k].to == to) {
			t->meas[k].mean_mm = mean_mm;
			t->meas[k].sd_mm = sd_mm;
			t->meas[k].n_ok = n_ok;
			return 0;
		}
	}

	if (t->n_meas >= APOS_MAX_MEAS) {
		return -ENOSPC;
	}

	t->meas[t->n_meas].from = from;
	t->meas[t->n_meas].to = to;
	t->meas[t->n_meas].mean_mm = mean_mm;
	t->meas[t->n_meas].sd_mm = sd_mm;
	t->meas[t->n_meas].n_ok = n_ok;
	t->n_meas++;
	return 0;
}

/* The usable measurement for the ordered pair (a -> b), or NULL. */
static const struct apos_dir_meas *find_meas(const struct apos_table *t,
					     uint8_t a, uint8_t b,
					     uint8_t min_n_ok)
{
	for (uint16_t k = 0; k < t->n_meas; k++) {
		if (t->meas[k].from == a && t->meas[k].to == b &&
		    t->meas[k].n_ok >= min_n_ok) {
			return &t->meas[k];
		}
	}
	return NULL;
}

static float sd_m_of(const struct apos_dir_meas *m)
{
	uint16_t sd = (m->sd_mm >= APOS_SD_FLOOR_MM) ? m->sd_mm
						     : (uint16_t)APOS_SD_FLOOR_MM;

	return (float)sd / 1000.0f;
}

uint16_t apos_table_symmetrise(const struct apos_table *t,
			       struct apos_edge *out, uint16_t out_cap,
			       uint8_t min_n_ok)
{
	uint16_t n = 0;

	if (!t || !out) {
		return 0;
	}

	for (uint8_t i = 0; i < t->n_peers && n < out_cap; i++) {
		for (uint8_t j = (uint8_t)(i + 1);
		     j < t->n_peers && n < out_cap; j++) {
			const struct apos_dir_meas *f =
				find_meas(t, i, j, min_n_ok);
			const struct apos_dir_meas *r =
				find_meas(t, j, i, min_n_ok);

			if (!f && !r) {
				continue; /* a hole; the fit works around it */
			}

			out[n].i = i;
			out[n].j = j;

			if (f && r) {
				/* Inverse-variance weighted, which reduces to
				 * the plain mean when the two sds match -- the
				 * usual case, since both directions run the
				 * same number of exchanges. */
				float sf = sd_m_of(f), sr = sd_m_of(r);
				float wf = 1.0f / (sf * sf);
				float wr = 1.0f / (sr * sr);
				float df = (float)f->mean_mm / 1000.0f;
				float dr = (float)r->mean_mm / 1000.0f;

				out[n].d_m = (wf * df + wr * dr) / (wf + wr);
				out[n].sd_m = 1.0f / sqrtf(wf + wr);
			} else {
				const struct apos_dir_meas *m = f ? f : r;

				out[n].d_m = (float)m->mean_mm / 1000.0f;
				out[n].sd_m = sd_m_of(m) *
					      APOS_ONEWAY_SD_INFLATE;
			}
			n++;
		}
	}
	return n;
}

uint16_t apos_table_missing_pairs(const struct apos_table *t, uint8_t min_n_ok)
{
	uint16_t missing = 0;

	if (!t) {
		return 0;
	}
	for (uint8_t i = 0; i < t->n_peers; i++) {
		for (uint8_t j = (uint8_t)(i + 1); j < t->n_peers; j++) {
			if (!find_meas(t, i, j, min_n_ok) &&
			    !find_meas(t, j, i, min_n_ok)) {
				missing++;
			}
		}
	}
	return missing;
}
```

`sqrtf` needs `#include <math.h>` alongside the includes above.

- [ ] **Step 5: Run to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_table/test_apos_table.exe tests/apos_table/test_apos_table.c src/apos_table.c src/apos_geom.c -lm
./tests/apos_table/test_apos_table.exe
```

Expected: `PASSED`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/apos_table.h src/apos_table.c tests/apos_table/test_apos_table.c
git commit -m "feat(apos): EUI-keyed peer table and directed-range symmetrisation"
```

---

### Task 4: `apos_frame` — the `0xEB` codec

**Files:**
- Create: `src/apos_frame.h`, `src/apos_frame.c`
- Test: `tests/apos_frame/test_apos_frame.c`

**Interfaces:**
- Consumes: `APOS_EUI_LEN` from `apos_table.h`.
- Produces: `APOS_FRAME_TYPE` (0xEB), the seven `APOS_SUB_*` constants, the seven `APOS_LEN_*` constants, `apos_frame_is_apos()`, `apos_frame_subtype()`, `apos_frame_src()`, `apos_frame_dest()`, `apos_frame_set_seq()`, and one `_build` plus one `_parse` per subtype (exact signatures in the header below).

- [ ] **Step 1: Write the header**

Create `src/apos_frame.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Codec for the auto-positioning frames: one message type (0xEB) carrying a
 * subtype byte, rather than seven entries in the 0xE_ type space.
 *
 * Deliberately NOT added to uwb_frame_802_15_4z.c. That file is copied
 * byte-for-byte from the tag and any divergence is a wire-format bug waiting to
 * happen; the survey is anchor-and-gateway-only traffic the tag never parses, so
 * it has no business in the shared codec.
 *
 * The header is the ADDRESSED 10-byte form the module frames use --
 * 0x41 0x88 seq 0xCA 0xDE dest_lo dest_hi src_lo src_hi type -- not the
 * WAVE/VEWA literal-ident form, because the survey needs real src/dest
 * addressing. Byte 10 is the subtype; payload starts at 11.
 *
 * Pure C -- no Zephyr -- so every round-trip is host-testable.
 */

#ifndef APOS_FRAME_H
#define APOS_FRAME_H

#include "apos_table.h"   /* APOS_EUI_LEN */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Continues the 0xE_ series after POS (0xEA). */
#define APOS_FRAME_TYPE 0xEBu

#define APOS_SUB_SURVEY_BEGIN 0x01u
#define APOS_SUB_ENUM_RSP     0x02u
#define APOS_SUB_RANGE_CMD    0x03u
#define APOS_SUB_RANGE_RSP    0x04u
#define APOS_SUB_SETPOS       0x05u
#define APOS_SUB_SETPOS_ACK   0x06u
#define APOS_SUB_SURVEY_END   0x07u

/* 10 common header bytes plus the subtype. */
#define APOS_HDR_LEN 11u

/* Payload lengths, excluding the FCS -- callers add FCS_LEN in
 * dwt_writetxfctrl() exactly as the module frames do. */
#define APOS_LEN_SURVEY_BEGIN (APOS_HDR_LEN + 4u)  /* session, window_s     */
#define APOS_LEN_ENUM_RSP     (APOS_HDR_LEN + 23u) /* session, eui, pv, xyz */
#define APOS_LEN_RANGE_CMD    (APOS_HDR_LEN + 5u)  /* session, peer, n_exch */
#define APOS_LEN_RANGE_RSP    (APOS_HDR_LEN + 11u) /* session, peer, mean,
						    * sd, n_ok             */
#define APOS_LEN_SETPOS       (APOS_HDR_LEN + 14u) /* session, xyz          */
#define APOS_LEN_SETPOS_ACK   (APOS_HDR_LEN + 15u) /* session, xyz, ok      */
#define APOS_LEN_SURVEY_END   (APOS_HDR_LEN + 2u)  /* session               */

/* Largest of the above, for RX buffer sizing. */
#define APOS_LEN_MAX APOS_LEN_ENUM_RSP

/* Broadcast destination, matching UWB_FRAME_ADDR_BCAST. Redeclared rather than
 * included so this module stays free of the shared codec's header. */
#define APOS_ADDR_BCAST 0xFFFFu

/* True if this is a well-formed APOS frame of at least APOS_HDR_LEN bytes with
 * a known subtype. Every parser below re-checks its own length, so a caller may
 * dispatch on apos_frame_subtype() straight after this returns true. */
bool    apos_frame_is_apos(const uint8_t *buf, size_t len);
uint8_t apos_frame_subtype(const uint8_t *buf);
uint16_t apos_frame_src(const uint8_t *buf);
uint16_t apos_frame_dest(const uint8_t *buf);
void    apos_frame_set_seq(uint8_t *buf, uint8_t seq);

/* Builders return the payload length written (excluding FCS), or a negative
 * errno: -EINVAL on a NULL pointer, -EMSGSIZE when buf_len is too small. */
int apos_frame_survey_begin_build(uint8_t *buf, size_t buf_len, uint16_t src,
				  uint16_t session, uint16_t window_s);
int apos_frame_enum_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			      uint16_t dest, uint16_t session,
			      const uint8_t eui[APOS_EUI_LEN], bool pos_valid,
			      float x, float y, float z);
int apos_frame_range_cmd_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, uint8_t n_exchanges);
int apos_frame_range_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, int32_t mean_mm,
			       uint16_t sd_mm, uint8_t n_ok);
int apos_frame_setpos_build(uint8_t *buf, size_t buf_len, uint16_t src,
			    uint16_t dest, uint16_t session,
			    float x, float y, float z);
int apos_frame_setpos_ack_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t dest, uint16_t session,
				float x, float y, float z, bool ok);
int apos_frame_survey_end_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t session);

/* Parsers return 0, or -EINVAL on a NULL pointer, a length that does not match
 * the subtype exactly, or a subtype mismatch. Out parameters are untouched on
 * failure, so a caller cannot act on half-parsed values. */
int apos_frame_parse_survey_begin(const uint8_t *buf, size_t len,
				  uint16_t *session, uint16_t *window_s);
int apos_frame_parse_enum_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			      uint8_t eui_out[APOS_EUI_LEN], bool *pos_valid,
			      float *x, float *y, float *z);
int apos_frame_parse_range_cmd(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, uint8_t *n_exchanges);
int apos_frame_parse_range_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, int32_t *mean_mm,
			       uint16_t *sd_mm, uint8_t *n_ok);
int apos_frame_parse_setpos(const uint8_t *buf, size_t len, uint16_t *session,
			    float *x, float *y, float *z);
int apos_frame_parse_setpos_ack(const uint8_t *buf, size_t len,
				uint16_t *session, float *x, float *y, float *z,
				bool *ok);
int apos_frame_parse_survey_end(const uint8_t *buf, size_t len,
				uint16_t *session);

#endif /* APOS_FRAME_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/apos_frame/test_apos_frame.c`:

```c
#include "apos_frame.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static uint8_t buf[64];

static const uint8_t eui_ref[APOS_EUI_LEN] = {
    0xDE, 0xCA, 0x01, 0x02, 0x03, 0x04, 0x05, 0x77
};

/* Every APOS frame must carry the same first five bytes as every other frame on
 * this network, or a peer's is_valid() check rejects it before the subtype is
 * ever looked at. */
static void test_header_matches_the_network_convention(void)
{
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0x1234, 30);

    CHECK(n == (int)APOS_LEN_SURVEY_BEGIN);
    CHECK(buf[0] == 0x41);
    CHECK(buf[1] == 0x88);
    CHECK(buf[3] == 0xCA);
    CHECK(buf[4] == 0xDE);
    CHECK(buf[9] == APOS_FRAME_TYPE);
    CHECK(buf[10] == APOS_SUB_SURVEY_BEGIN);
}

static void test_addresses_are_little_endian(void)
{
    int n = apos_frame_range_cmd_build(buf, sizeof(buf), 0x0000, 0x0203,
                                       0x1234, 0x0004, 40);

    CHECK(n == (int)APOS_LEN_RANGE_CMD);
    CHECK(buf[5] == 0x03);   /* dest lo */
    CHECK(buf[6] == 0x02);   /* dest hi */
    CHECK(buf[7] == 0x00);   /* src  lo */
    CHECK(buf[8] == 0x00);   /* src  hi */
    CHECK(apos_frame_dest(buf) == 0x0203);
    CHECK(apos_frame_src(buf) == 0x0000);
}

static void test_survey_begin_round_trip(void)
{
    uint16_t session = 0, window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0xBEEF, 45);

    CHECK(n == (int)APOS_LEN_SURVEY_BEGIN);
    CHECK(apos_frame_dest(buf) == APOS_ADDR_BCAST);
    CHECK(apos_frame_parse_survey_begin(buf, (size_t)n, &session, &window) == 0);
    CHECK(session == 0xBEEF);
    CHECK(window == 45);
}

static void test_enum_rsp_round_trip(void)
{
    uint16_t session = 0;
    uint8_t eui[APOS_EUI_LEN] = {0};
    bool pv = false;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int n = apos_frame_enum_rsp_build(buf, sizeof(buf), 0x0003, 0x0000,
                                      0xBEEF, eui_ref, true,
                                      1.25f, -2.5f, 3.75f);

    CHECK(n == (int)APOS_LEN_ENUM_RSP);
    CHECK(apos_frame_parse_enum_rsp(buf, (size_t)n, &session, eui, &pv,
                                    &x, &y, &z) == 0);
    CHECK(session == 0xBEEF);
    CHECK(memcmp(eui, eui_ref, APOS_EUI_LEN) == 0);
    CHECK(pv == true);
    /* IEEE-754 round-trips exactly for these values -- no tolerance needed,
     * and an exact check catches a byte-order bug a tolerance would hide. */
    CHECK(x == 1.25f);
    CHECK(y == -2.5f);
    CHECK(z == 3.75f);
}

static void test_range_cmd_round_trip(void)
{
    uint16_t session = 0, peer = 0;
    uint8_t nex = 0;
    int n = apos_frame_range_cmd_build(buf, sizeof(buf), 0x0000, 0x0001,
                                       0xBEEF, 0x0004, 40);

    CHECK(n == (int)APOS_LEN_RANGE_CMD);
    CHECK(apos_frame_parse_range_cmd(buf, (size_t)n, &session, &peer, &nex) == 0);
    CHECK(session == 0xBEEF);
    CHECK(peer == 0x0004);
    CHECK(nex == 40);
}

/* mean_mm is signed: the initiator reports a negative distance when antenna
 * delays over-correct at very short range, and clamping it at zero would hide
 * a real calibration fault. */
static void test_range_rsp_round_trip_including_negative_mean(void)
{
    uint16_t session = 0, peer = 0, sd = 0;
    int32_t mean = 0;
    uint8_t n_ok = 0;
    int n = apos_frame_range_rsp_build(buf, sizeof(buf), 0x0001, 0x0000,
                                       0xBEEF, 0x0004, -37, 21, 39);

    CHECK(n == (int)APOS_LEN_RANGE_RSP);
    CHECK(apos_frame_parse_range_rsp(buf, (size_t)n, &session, &peer, &mean,
                                     &sd, &n_ok) == 0);
    CHECK(session == 0xBEEF);
    CHECK(peer == 0x0004);
    CHECK(mean == -37);
    CHECK(sd == 21);
    CHECK(n_ok == 39);
}

static void test_setpos_round_trip(void)
{
    uint16_t session = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int n = apos_frame_setpos_build(buf, sizeof(buf), 0x0000, 0x0002,
                                    0xBEEF, 3.5f, 4.25f, -0.5f);

    CHECK(n == (int)APOS_LEN_SETPOS);
    CHECK(apos_frame_parse_setpos(buf, (size_t)n, &session, &x, &y, &z) == 0);
    CHECK(session == 0xBEEF);
    CHECK(x == 3.5f);
    CHECK(y == 4.25f);
    CHECK(z == -0.5f);
}

static void test_setpos_ack_round_trip(void)
{
    uint16_t session = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    bool ok = false;
    int n = apos_frame_setpos_ack_build(buf, sizeof(buf), 0x0002, 0x0000,
                                        0xBEEF, 3.5f, 4.25f, -0.5f, true);

    CHECK(n == (int)APOS_LEN_SETPOS_ACK);
    CHECK(apos_frame_parse_setpos_ack(buf, (size_t)n, &session, &x, &y, &z,
                                      &ok) == 0);
    CHECK(session == 0xBEEF);
    CHECK(x == 3.5f);
    CHECK(ok == true);
}

static void test_survey_end_round_trip(void)
{
    uint16_t session = 0;
    int n = apos_frame_survey_end_build(buf, sizeof(buf), 0x0000, 0xBEEF);

    CHECK(n == (int)APOS_LEN_SURVEY_END);
    CHECK(apos_frame_dest(buf) == APOS_ADDR_BCAST);
    CHECK(apos_frame_parse_survey_end(buf, (size_t)n, &session) == 0);
    CHECK(session == 0xBEEF);
}

static void test_is_apos_rejects_other_traffic(void)
{
    /* A VEWA response: right PAN, wrong type. */
    const uint8_t vewa[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
                            0xE1, 0x02};

    CHECK(!apos_frame_is_apos(vewa, sizeof(vewa)));

    /* Right type, unknown subtype. */
    apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);
    buf[10] = 0x7F;
    CHECK(!apos_frame_is_apos(buf, APOS_LEN_SURVEY_BEGIN));

    /* Too short to hold a subtype at all. */
    CHECK(!apos_frame_is_apos(buf, APOS_HDR_LEN - 1u));
    CHECK(!apos_frame_is_apos(NULL, APOS_LEN_SURVEY_BEGIN));
}

/* A truncated frame must be refused, not parsed from whatever follows in the
 * caller's buffer. */
static void test_truncated_frames_are_refused(void)
{
    uint16_t session = 0xAAAA, window = 0xAAAA;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    CHECK(apos_frame_parse_survey_begin(buf, (size_t)(n - 1), &session,
                                        &window) == -EINVAL);
    /* Untouched on failure. */
    CHECK(session == 0xAAAA);
    CHECK(window == 0xAAAA);
}

/* An FCS left on the end is the classic bug on this project: flen from
 * dwt_getframelength() includes it. An over-long frame must be refused so the
 * mistake shows up immediately instead of shifting every field. */
static void test_extra_trailing_bytes_are_refused(void)
{
    uint16_t session = 0;
    uint16_t window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    CHECK(apos_frame_parse_survey_begin(buf, (size_t)(n + 2), &session,
                                        &window) == -EINVAL);
}

static void test_parser_rejects_the_wrong_subtype(void)
{
    uint16_t session = 0, peer = 0;
    uint8_t nex = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 1, 1);

    /* Right length would never match anyway, but the subtype check must be
     * what rejects it -- otherwise a length collision between two subtypes
     * would silently mis-parse. */
    CHECK(apos_frame_parse_range_cmd(buf, (size_t)n, &session, &peer,
                                     &nex) == -EINVAL);
}

static void test_builders_reject_a_short_buffer(void)
{
    CHECK(apos_frame_enum_rsp_build(buf, 4, 0x0003, 0x0000, 1, eui_ref,
                                    true, 0.0f, 0.0f, 0.0f) == -EMSGSIZE);
    CHECK(apos_frame_survey_begin_build(NULL, sizeof(buf), 0x0000, 1,
                                        1) == -EINVAL);
    CHECK(apos_frame_enum_rsp_build(buf, sizeof(buf), 0x0003, 0x0000, 1, NULL,
                                    true, 0.0f, 0.0f, 0.0f) == -EINVAL);
}

static void test_seq_is_settable_without_disturbing_the_payload(void)
{
    uint16_t session = 0, window = 0;
    int n = apos_frame_survey_begin_build(buf, sizeof(buf), 0x0000, 0xBEEF, 45);

    apos_frame_set_seq(buf, 0x5A);
    CHECK(buf[2] == 0x5A);
    CHECK(apos_frame_parse_survey_begin(buf, (size_t)n, &session, &window) == 0);
    CHECK(session == 0xBEEF);
    CHECK(window == 45);
}

int main(void)
{
    test_header_matches_the_network_convention();
    test_addresses_are_little_endian();
    test_survey_begin_round_trip();
    test_enum_rsp_round_trip();
    test_range_cmd_round_trip();
    test_range_rsp_round_trip_including_negative_mean();
    test_setpos_round_trip();
    test_setpos_ack_round_trip();
    test_survey_end_round_trip();
    test_is_apos_rejects_other_traffic();
    test_truncated_frames_are_refused();
    test_extra_trailing_bytes_are_refused();
    test_parser_rejects_the_wrong_subtype();
    test_builders_reject_a_short_buffer();
    test_seq_is_settable_without_disturbing_the_payload();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}
```

- [ ] **Step 3: Run to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_frame/test_apos_frame.exe tests/apos_frame/test_apos_frame.c src/apos_frame.c src/apos_table.c src/apos_geom.c -lm
```

Expected: FAIL — `src/apos_frame.c` does not exist.

- [ ] **Step 4: Implement**

Create `src/apos_frame.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_frame.h. The field helpers mirror uwb_frame_802_15_4z.c's rather
 * than being shared with it: that file is byte-identical to the tag's copy and
 * exports none of them, and four one-line helpers are a smaller price than
 * making the shared codec export internals.
 */

#include "apos_frame.h"

#include <errno.h>
#include <string.h>

#define OFF_SEQ     2u
#define OFF_DEST    5u
#define OFF_SRC     7u
#define OFF_TYPE    9u
#define OFF_SUB     10u
#define OFF_PAYLOAD 11u

/* ---- Little-endian field helpers ---- */
static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* memcpy through a uint32_t rather than a cast: type-punning a float* to a
 * uint32_t* is undefined behaviour and gcc does miscompile it at -O2. */
static void put_f32(uint8_t *p, float v)
{
	uint32_t u;

	memcpy(&u, &v, sizeof(u));
	put_u32(p, u);
}

static float get_f32(const uint8_t *p)
{
	uint32_t u = get_u32(p);
	float v;

	memcpy(&v, &u, sizeof(v));
	return v;
}

static void put_xyz(uint8_t *p, float x, float y, float z)
{
	put_f32(p, x);
	put_f32(p + 4, y);
	put_f32(p + 8, z);
}

static void get_xyz(const uint8_t *p, float *x, float *y, float *z)
{
	*x = get_f32(p);
	*y = get_f32(p + 4);
	*z = get_f32(p + 8);
}

static int write_hdr(uint8_t *buf, size_t buf_len, size_t need, uint16_t dest,
		     uint16_t src, uint8_t subtype)
{
	if (!buf) {
		return -EINVAL;
	}
	if (buf_len < need) {
		return -EMSGSIZE;
	}
	buf[0] = 0x41;
	buf[1] = 0x88;
	buf[OFF_SEQ] = 0; /* caller sets via apos_frame_set_seq() */
	buf[3] = 0xCA;
	buf[4] = 0xDE;
	put_u16(&buf[OFF_DEST], dest);
	put_u16(&buf[OFF_SRC], src);
	buf[OFF_TYPE] = APOS_FRAME_TYPE;
	buf[OFF_SUB] = subtype;
	return 0;
}

static bool known_subtype(uint8_t s)
{
	return s >= APOS_SUB_SURVEY_BEGIN && s <= APOS_SUB_SURVEY_END;
}

bool apos_frame_is_apos(const uint8_t *buf, size_t len)
{
	if (!buf || len < APOS_HDR_LEN) {
		return false;
	}
	if (buf[0] != 0x41 || buf[1] != 0x88) {
		return false;
	}
	if (buf[3] != 0xCA || buf[4] != 0xDE) {
		return false;
	}
	if (buf[OFF_TYPE] != APOS_FRAME_TYPE) {
		return false;
	}
	return known_subtype(buf[OFF_SUB]);
}

uint8_t apos_frame_subtype(const uint8_t *buf)
{
	return buf[OFF_SUB];
}

uint16_t apos_frame_src(const uint8_t *buf)
{
	return get_u16(&buf[OFF_SRC]);
}

uint16_t apos_frame_dest(const uint8_t *buf)
{
	return get_u16(&buf[OFF_DEST]);
}

void apos_frame_set_seq(uint8_t *buf, uint8_t seq)
{
	buf[OFF_SEQ] = seq;
}

/* Shared entry check for every parser: an APOS frame of exactly this subtype
 * and exactly this length. Exact, not >=, because a frame length that does not
 * match is either a truncation or an unsubtracted FCS -- both are bugs worth
 * surfacing rather than parsing around. */
static int parse_check(const uint8_t *buf, size_t len, uint8_t subtype,
		       size_t expect)
{
	if (!apos_frame_is_apos(buf, len)) {
		return -EINVAL;
	}
	if (buf[OFF_SUB] != subtype || len != expect) {
		return -EINVAL;
	}
	return 0;
}

/* ---- SURVEY_BEGIN ---- */
int apos_frame_survey_begin_build(uint8_t *buf, size_t buf_len, uint16_t src,
				  uint16_t session, uint16_t window_s)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SURVEY_BEGIN, APOS_ADDR_BCAST,
			   src, APOS_SUB_SURVEY_BEGIN);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], window_s);
	return (int)APOS_LEN_SURVEY_BEGIN;
}

int apos_frame_parse_survey_begin(const uint8_t *buf, size_t len,
				  uint16_t *session, uint16_t *window_s)
{
	int rc = parse_check(buf, len, APOS_SUB_SURVEY_BEGIN,
			     APOS_LEN_SURVEY_BEGIN);

	if (rc || !session || !window_s) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*window_s = get_u16(&buf[OFF_PAYLOAD + 2]);
	return 0;
}

/* ---- ENUM_RSP ---- */
int apos_frame_enum_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			      uint16_t dest, uint16_t session,
			      const uint8_t eui[APOS_EUI_LEN], bool pos_valid,
			      float x, float y, float z)
{
	if (!eui) {
		return -EINVAL;
	}

	int rc = write_hdr(buf, buf_len, APOS_LEN_ENUM_RSP, dest, src,
			   APOS_SUB_ENUM_RSP);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	memcpy(&buf[OFF_PAYLOAD + 2], eui, APOS_EUI_LEN);
	buf[OFF_PAYLOAD + 10] = pos_valid ? 1u : 0u;
	put_xyz(&buf[OFF_PAYLOAD + 11], x, y, z);
	return (int)APOS_LEN_ENUM_RSP;
}

int apos_frame_parse_enum_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			      uint8_t eui_out[APOS_EUI_LEN], bool *pos_valid,
			      float *x, float *y, float *z)
{
	int rc = parse_check(buf, len, APOS_SUB_ENUM_RSP, APOS_LEN_ENUM_RSP);

	if (rc) {
		return rc;
	}
	if (!session || !eui_out || !pos_valid || !x || !y || !z) {
		return -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	memcpy(eui_out, &buf[OFF_PAYLOAD + 2], APOS_EUI_LEN);
	*pos_valid = buf[OFF_PAYLOAD + 10] != 0u;
	get_xyz(&buf[OFF_PAYLOAD + 11], x, y, z);
	return 0;
}

/* ---- RANGE_CMD ---- */
int apos_frame_range_cmd_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, uint8_t n_exchanges)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_RANGE_CMD, dest, src,
			   APOS_SUB_RANGE_CMD);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], peer_addr);
	buf[OFF_PAYLOAD + 4] = n_exchanges;
	return (int)APOS_LEN_RANGE_CMD;
}

int apos_frame_parse_range_cmd(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, uint8_t *n_exchanges)
{
	int rc = parse_check(buf, len, APOS_SUB_RANGE_CMD, APOS_LEN_RANGE_CMD);

	if (rc || !session || !peer_addr || !n_exchanges) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*peer_addr = get_u16(&buf[OFF_PAYLOAD + 2]);
	*n_exchanges = buf[OFF_PAYLOAD + 4];
	return 0;
}

/* ---- RANGE_RSP ---- */
int apos_frame_range_rsp_build(uint8_t *buf, size_t buf_len, uint16_t src,
			       uint16_t dest, uint16_t session,
			       uint16_t peer_addr, int32_t mean_mm,
			       uint16_t sd_mm, uint8_t n_ok)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_RANGE_RSP, dest, src,
			   APOS_SUB_RANGE_RSP);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_u16(&buf[OFF_PAYLOAD + 2], peer_addr);
	put_u32(&buf[OFF_PAYLOAD + 4], (uint32_t)mean_mm);
	put_u16(&buf[OFF_PAYLOAD + 8], sd_mm);
	buf[OFF_PAYLOAD + 10] = n_ok;
	return (int)APOS_LEN_RANGE_RSP;
}

int apos_frame_parse_range_rsp(const uint8_t *buf, size_t len, uint16_t *session,
			       uint16_t *peer_addr, int32_t *mean_mm,
			       uint16_t *sd_mm, uint8_t *n_ok)
{
	int rc = parse_check(buf, len, APOS_SUB_RANGE_RSP, APOS_LEN_RANGE_RSP);

	if (rc) {
		return rc;
	}
	if (!session || !peer_addr || !mean_mm || !sd_mm || !n_ok) {
		return -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	*peer_addr = get_u16(&buf[OFF_PAYLOAD + 2]);
	*mean_mm = (int32_t)get_u32(&buf[OFF_PAYLOAD + 4]);
	*sd_mm = get_u16(&buf[OFF_PAYLOAD + 8]);
	*n_ok = buf[OFF_PAYLOAD + 10];
	return 0;
}

/* ---- SETPOS ---- */
int apos_frame_setpos_build(uint8_t *buf, size_t buf_len, uint16_t src,
			    uint16_t dest, uint16_t session,
			    float x, float y, float z)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SETPOS, dest, src,
			   APOS_SUB_SETPOS);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	return (int)APOS_LEN_SETPOS;
}

int apos_frame_parse_setpos(const uint8_t *buf, size_t len, uint16_t *session,
			    float *x, float *y, float *z)
{
	int rc = parse_check(buf, len, APOS_SUB_SETPOS, APOS_LEN_SETPOS);

	if (rc || !session || !x || !y || !z) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	get_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	return 0;
}

/* ---- SETPOS_ACK ---- */
int apos_frame_setpos_ack_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t dest, uint16_t session,
				float x, float y, float z, bool ok)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SETPOS_ACK, dest, src,
			   APOS_SUB_SETPOS_ACK);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	put_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	buf[OFF_PAYLOAD + 14] = ok ? 1u : 0u;
	return (int)APOS_LEN_SETPOS_ACK;
}

int apos_frame_parse_setpos_ack(const uint8_t *buf, size_t len,
				uint16_t *session, float *x, float *y, float *z,
				bool *ok)
{
	int rc = parse_check(buf, len, APOS_SUB_SETPOS_ACK,
			     APOS_LEN_SETPOS_ACK);

	if (rc || !session || !x || !y || !z || !ok) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	get_xyz(&buf[OFF_PAYLOAD + 2], x, y, z);
	*ok = buf[OFF_PAYLOAD + 14] != 0u;
	return 0;
}

/* ---- SURVEY_END ---- */
int apos_frame_survey_end_build(uint8_t *buf, size_t buf_len, uint16_t src,
				uint16_t session)
{
	int rc = write_hdr(buf, buf_len, APOS_LEN_SURVEY_END, APOS_ADDR_BCAST,
			   src, APOS_SUB_SURVEY_END);

	if (rc) {
		return rc;
	}
	put_u16(&buf[OFF_PAYLOAD], session);
	return (int)APOS_LEN_SURVEY_END;
}

int apos_frame_parse_survey_end(const uint8_t *buf, size_t len,
				uint16_t *session)
{
	int rc = parse_check(buf, len, APOS_SUB_SURVEY_END,
			     APOS_LEN_SURVEY_END);

	if (rc || !session) {
		return rc ? rc : -EINVAL;
	}
	*session = get_u16(&buf[OFF_PAYLOAD]);
	return 0;
}
```

- [ ] **Step 5: Run to verify it passes**

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_frame/test_apos_frame.exe tests/apos_frame/test_apos_frame.c src/apos_frame.c src/apos_table.c src/apos_geom.c -lm
./tests/apos_frame/test_apos_frame.exe
```

Expected: `PASSED`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/apos_frame.h src/apos_frame.c tests/apos_frame/test_apos_frame.c
git commit -m "feat(apos): 0xEB survey frame codec with seven subtypes"
```

---

### Task 5: Extract `ss_initiator` from `cal_initiator`

Pure rename plus a build-set move. No behaviour change — the calibration procedure must still work identically afterwards, and that is the gate on this task.

**Files:**
- Rename: `src/cal_initiator.h` → `src/ss_initiator.h`, `src/cal_initiator.c` → `src/ss_initiator.c`
- Modify: `src/ss_initiator.h`, `src/ss_initiator.c`, `src/cal_run.c`, `CMakeLists.txt`

**Interfaces:**
- Consumes: nothing new.
- Produces: `void ss_initiator_enter(void)`, `void ss_initiator_leave(void)`, `int32_t ss_initiator_range(uint8_t peer_wire_id)` — identical semantics to the `cal_initiator_*` functions they replace, including the `INT32_MIN` failure return.

- [ ] **Step 1: Rename the files, preserving history**

```bash
git mv src/cal_initiator.h src/ss_initiator.h
git mv src/cal_initiator.c src/ss_initiator.c
```

- [ ] **Step 2: Rename the symbols and update the header comment**

In `src/ss_initiator.h` replace the guard `CAL_INITIATOR_H` with `SS_INITIATOR_H`, rename the three functions from `cal_initiator_*` to `ss_initiator_*`, and replace the second paragraph of the file comment (currently "Used only by the calibration image.") with:

```c
 * One SS-TWR exchange with this board as the initiator -- the role the tag
 * normally plays. Shared by the calibration image (cal_run.c) and the
 * production image (apos_node.c): it is the exact exchange the antenna delays
 * were calibrated through, so a second copy would be a second thing to
 * calibrate.
 *
 * Compiled into BOTH images, but a production anchor must never poll
 * unsolicited. That safety property is enforced by the CALLER, not here:
 * apos_node.c enters this only while a gateway-opened survey window is live AND
 * a RANGE_CMD for the current session has arrived from 0x0000. See
 * docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md 6.3.
```

In `src/ss_initiator.c` change `#include "cal_initiator.h"` to `#include "ss_initiator.h"`, rename `LOG_MODULE_REGISTER(cal_initiator, ...)` to `LOG_MODULE_REGISTER(ss_initiator, ...)`, rename the three function definitions, and change the two log strings `"cal diag: ..."` to `"ss diag: ..."`.

Rename nothing else. In particular leave `diag_counts`, the two response-layout constants and the clock-offset correction exactly as they are — this task must not change what the calibration measures.

- [ ] **Step 3: Update the only existing caller**

In `src/cal_run.c`: change `#include "cal_initiator.h"` to `#include "ss_initiator.h"`, and in `run_batch()` change `cal_initiator_enter()` → `ss_initiator_enter()`, `cal_initiator_range(...)` → `ss_initiator_range(...)`, `cal_initiator_leave()` → `ss_initiator_leave()`.

- [ ] **Step 4: Move the source out of the cal-only build set**

In `CMakeLists.txt`, add `src/ss_initiator.c` to the main `target_sources(app PRIVATE ...)` list in alphabetical position (between `src/pos_sink.c` and `src/uwb_config.c`), and delete `src/cal_initiator.c` from the `target_sources_ifdef(CONFIG_ANCLA_CAL_MODE ...)` block. Update that block's comment, which currently claims the whole set is excluded from production:

```cmake
# Calibration image only (see cal.conf, docs/superpowers/specs/
# 2026-08-13-antenna-delay-calibration-design.md). ss_initiator.c is
# deliberately NOT here: the production image needs it for the anchor survey
# (apos_node.c), which gates it on a gateway-opened window rather than on the
# build. What must stay out of production is the `cal` shell tree and cal_run's
# unsolicited-poll loop.
```

- [ ] **Step 5: Verify both images still build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
```

Expected: both complete with no errors and no new warnings. Confirm with `git grep -n cal_initiator` that it returns **no** matches outside `docs/`.

- [ ] **Step 6: Commit**

```bash
git add -A src CMakeLists.txt
git commit -m "refactor(apos): share the SS-TWR initiator between both images

cal_initiator -> ss_initiator, moved into the production build set. No
behaviour change: the same exchange, the same clock-offset correction, the
same INT32_MIN failure return. The survey needs an initiator on a production
anchor, and duplicating this file would mean two things to calibrate rather
than one. The safety property that used to come from the build set now comes
from apos_node.c's two gates."
```

---

### Task 6: `apos_node` — survey window, enumeration, and `SETPOS`

The anchor side, minus the ranging batch (Task 7). At the end of this task an anchor answers `SURVEY_BEGIN` with a staggered `ENUM_RSP` and persists a pushed position.

**Files:**
- Create: `src/apos_node.h`, `src/apos_node.c`
- Modify: `CMakeLists.txt`, `prj.conf`

**Interfaces:**
- Consumes: `apos_frame.h` (all builders/parsers), `uwb_config.h`, `uwb_store.h`, `beacon_guard.h`, `uwb_dwtime.h`.
- Produces: `APOS_ENUM_SLOTS` (8), `APOS_ENUM_SLOT_MS` (30), `void apos_node_init(void)`, `bool apos_node_window_open(void)`, `bool apos_node_on_rx(const uint8_t *buf, uint16_t plen, uwb_config_t *cfg, uint8_t *seq, struct beacon_guard *bg)`, `const uint8_t *apos_node_eui(void)`.

- [ ] **Step 1: Confirm `hwinfo` is available for the EUI**

The enumeration stagger and the peer table are both keyed on a unique per-board id, and this project has never generated one (`gw_core` only ever *receives* tag EUIs). Zephyr's `hwinfo` reads it from the ESP32-S3 eFuse MAC.

Add to `prj.conf`:

```
# EUI-64 for the anchor survey: apos_node keys enumeration and the gateway's
# coordinate table on a per-board id, and the stagger that keeps enumeration
# replies from colliding is derived from it. hwinfo reads the ESP32-S3 eFuse MAC.
CONFIG_HWINFO=y
```

Verify it links before writing anything else:

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: builds clean. If `CONFIG_HWINFO` is not available on this board, stop and report it — the fallback (deriving an id from `anchor_id`) would reintroduce exactly the id coupling this whole design removes, so it is a decision for the human, not a silent substitution.

- [ ] **Step 2: Write the header**

Create `src/apos_node.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Anchor side of the auto-positioning survey. Offered every received frame by
 * the SLAVE loop; ignores anything that is not an APOS frame.
 *
 * This board never decides its own role, never holds a deployment list, and
 * never initiates anything. It answers enumeration, runs exactly the ranging
 * batch it is told to run, and stores exactly the coordinates it is given. The
 * gateway holds all the state -- which is what makes an `anchor id` swap
 * harmless.
 *
 * The survey window is the safety boundary. SURVEY_BEGIN opens it for a bounded
 * number of seconds; any later APOS frame refreshes it; SURVEY_END closes it
 * early. Outside the window this module transmits nothing and this board will
 * not act as an initiator, which is what keeps a production anchor from
 * colliding with tag ranging traffic.
 */

#ifndef APOS_NODE_H
#define APOS_NODE_H

#include "beacon_guard.h"
#include "uwb_config.h"

#include <stdbool.h>
#include <stdint.h>

/* Enumeration reply stagger. SURVEY_BEGIN is a broadcast, so without a stagger
 * every anchor answers at once and the gateway hears a collision.
 *
 * The slot is chosen by hashing this board's EUI-64, NOT by anchor_id: assuming
 * anything about ids is the coupling this design exists to remove, and a
 * duplicate id must remain DETECTABLE rather than being silently folded into
 * one reply. EUI-64 is unique by manufacture, so the stagger is collision-free
 * with no configuration and is stable across reboots.
 *
 * 8 slots x 30 ms bounds the worst-case reply delay at 210 ms, which is about
 * one superframe -- short enough for the SLAVE loop to simply sleep through it. */
#define APOS_ENUM_SLOTS   8u
#define APOS_ENUM_SLOT_MS 30u

/* Clear to the closed-window state. Call once before the SLAVE loop starts. */
void apos_node_init(void);

/* True while a gateway-opened survey window is live.
 *
 * Read by anchor_respond_wave_poll() via uwb_slave.c: during a survey every
 * anchor is still unpositioned yet must answer its peers' polls, so this is
 * what lets the poll responder refuse when unpositioned without breaking a
 * survey of a cold deployment. Also gated on by the RANGE_CMD path.
 *
 * Evaluates the deadline on each call, so an expired window closes itself with
 * no tick or timer. */
bool apos_node_window_open(void);

/* This board's EUI-64. Never NULL; 8 bytes. */
const uint8_t *apos_node_eui(void);

/* Offer one received frame. Returns true if it was an APOS frame handled here,
 * so the caller can skip the remaining responders.
 *
 * cfg must be the SLAVE loop's own mutable snapshot: a SETPOS is applied to it
 * immediately, not deferred to the next reboot. Unlike ant_delay_tx there is no
 * radio register to keep in step with it, and an operator who has just run
 * `apos apply` expects the anchor to report its new coordinates at once.
 *
 * May block for up to APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS on an ENUM_RSP and,
 * once Task 7 lands, for the length of one ranging batch on a RANGE_CMD. Both
 * are bounded and both happen only during commissioning. */
bool apos_node_on_rx(const uint8_t *buf, uint16_t plen, uwb_config_t *cfg,
		     uint8_t *seq, struct beacon_guard *bg);

#endif /* APOS_NODE_H */
```

- [ ] **Step 3: Implement**

Create `src/apos_node.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_node.h.
 */

#include "apos_node.h"

#include "apos_frame.h"
#include "uwb_dwtime.h"
#include "uwb_store.h"

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <deca_device_api.h>

#include <string.h>

LOG_MODULE_REGISTER(apos_node, LOG_LEVEL_INF);

/* Bound for the post-dwt_starttx() TXFRS wait. Every APOS transmission from
 * this module is IMMEDIATE, not delayed, so unlike anchor_respond.c's 18 ms
 * this only has to cover a frame's airtime -- the longest APOS frame is 34
 * bytes, well under 2 ms at 850 kbps with a PLEN_1024 preamble. 8 ms is
 * generous and still far short of anything that would matter to the loop. */
#define TX_COMPLETE_TIMEOUT_MS 8

/* How far ahead of an immediate TX to test the beacon guard. dwt_starttx()
 * plus the SPI write is well under this; it exists so the guard is asked about
 * the instant the frame actually leaves rather than the instant we asked. */
#define TX_GUARD_LOOKAHEAD_UUS 500u

/* The open window's deadline in kernel uptime, and the session it belongs to.
 * A session of 0 is never issued by the gateway, so it doubles as "no session". */
static int64_t window_deadline_ms;
static uint16_t window_session;

static uint8_t eui[APOS_EUI_LEN];
static bool eui_ready;

static uint8_t tx_buf[APOS_LEN_MAX];

void apos_node_init(void)
{
	window_deadline_ms = 0;
	window_session = 0;

	/* hwinfo may return fewer than 8 bytes. Zero-pad rather than leaving
	 * the tail uninitialised: the tail is hashed for the stagger and
	 * compared for identity, so uninitialised bytes would make this board's
	 * identity change between boots. */
	memset(eui, 0, sizeof(eui));

	ssize_t n = hwinfo_get_device_id(eui, sizeof(eui));

	if (n <= 0) {
		LOG_ERR("hwinfo_get_device_id failed (%d) — this board has no "
			"unique id and cannot take part in a survey",
			(int)n);
		eui_ready = false;
		return;
	}
	eui_ready = true;
	LOG_INF("{\"apos\":\"ready\",\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\"}",
		eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
}

const uint8_t *apos_node_eui(void)
{
	return eui;
}

bool apos_node_window_open(void)
{
	if (window_session == 0) {
		return false;
	}
	if (k_uptime_get() > window_deadline_ms) {
		/* Self-closing: no timer and no tick, so a gateway that dies
		 * mid-survey cannot leave this board permanently willing to
		 * transmit. */
		window_session = 0;
		LOG_INF("{\"apos\":\"window closed\",\"reason\":\"expired\"}");
		return false;
	}
	return true;
}

static void window_open(uint16_t session, uint16_t window_s)
{
	window_session = session;
	window_deadline_ms = k_uptime_get() + (int64_t)window_s * 1000;
}

/* Push the deadline out on any in-session APOS frame, so a long survey does not
 * need the gateway to re-broadcast SURVEY_BEGIN just to keep the window alive. */
static void window_refresh(void)
{
	if (window_session != 0) {
		window_deadline_ms = k_uptime_get() + (int64_t)1000 *
				     APOS_NODE_REFRESH_S;
	}
}

/* Transmit immediately. Returns false if the frame did not go out.
 *
 * Safe to poll TXFRS: uwb_slave.c enables DWT_INT_RX only, so dwt_isr() never
 * runs for TXFRS and cannot clear the bit ahead of this wait. Bounded anyway --
 * an unbounded wait here once froze the whole console. */
static bool tx_now(const uint8_t *buf, uint16_t len, struct beacon_guard *bg)
{
	uint32_t now = dwt_readsystimestamphi32();

	if (bg && !beacon_guard_tx_allowed(bg,
			now + UUS_TO_HI32(TX_GUARD_LOOKAHEAD_UUS))) {
		LOG_WRN("APOS TX suppressed — would land on the beacon");
		return false;
	}

	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	dwt_writetxdata(len, (uint8_t *)(uintptr_t)buf, 0);
	dwt_writetxfctrl((uint16_t)(len + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX failed to start");
		return false;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX started but TXFRS never completed — forced off");
		return false;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	return true;
}

/* FNV-1a over the EUI, reduced to a slot. Any cheap avalanche would do; what
 * matters is that it depends on every byte, since boards from one batch differ
 * only in the last one or two. */
static uint32_t enum_slot(void)
{
	uint32_t h = 2166136261u;

	for (int i = 0; i < APOS_EUI_LEN; i++) {
		h ^= eui[i];
		h *= 16777619u;
	}
	return h % APOS_ENUM_SLOTS;
}

static void handle_survey_begin(const uint8_t *buf, uint16_t plen,
				const uwb_config_t *cfg, uint8_t *seq,
				struct beacon_guard *bg)
{
	uint16_t session = 0, window_s = 0;

	if (apos_frame_parse_survey_begin(buf, plen, &session, &window_s) != 0) {
		return;
	}
	if (session == 0 || !eui_ready) {
		return;
	}

	window_open(session, window_s);
	LOG_INF("{\"apos\":\"window open\",\"session\":%u,\"window_s\":%u}",
		session, window_s);

	/* Sleep out our stagger slot before replying. Blocking the SLAVE loop
	 * for up to 210 ms is acceptable here and nowhere else: this happens
	 * only when an operator triggers a survey, and the alternative -- a
	 * timer plus a deferred TX path -- would add a second thread touching
	 * the SPI bus. */
	k_sleep(K_MSEC(enum_slot() * APOS_ENUM_SLOT_MS));

	int n = apos_frame_enum_rsp_build(tx_buf, sizeof(tx_buf),
					 uwb_config_short_addr(cfg),
					 UWB_ADDR_GATEWAY_RESERVED, session,
					 eui, cfg->position_valid,
					 cfg->x, cfg->y, cfg->z);

	if (n < 0) {
		LOG_ERR("ENUM_RSP build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);
	tx_now(tx_buf, (uint16_t)n, bg);
}

static void handle_setpos(const uint8_t *buf, uint16_t plen,
			  uwb_config_t *cfg, uint8_t *seq,
			  struct beacon_guard *bg)
{
	uint16_t session = 0;
	float x = 0.0f, y = 0.0f, z = 0.0f;

	if (apos_frame_parse_setpos(buf, plen, &session, &x, &y, &z) != 0) {
		return;
	}
	if (session != window_session) {
		LOG_WRN("SETPOS for session %u ignored — current is %u",
			session, window_session);
		return;
	}

	/* Applied to the loop's snapshot AND to the shared singleton, then
	 * persisted. The snapshot is what anchor_respond.c reports to the tag,
	 * so updating it is what makes the new coordinates take effect without
	 * a reboot; the singleton is what uwb_store_save_pos() reads. */
	uwb_config_set_pos(cfg, x, y, z);
	uwb_config_set_pos(uwb_config_get(), x, y, z);

	int rc = uwb_store_save_pos();
	bool ok = (rc == 0);

	if (!ok) {
		LOG_ERR("SETPOS applied but NOT persisted (%d) — will be lost "
			"on reboot", rc);
	} else {
		LOG_INF("{\"apos_setpos\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
			(double)x, (double)y, (double)z);
	}

	/* The ACK reports what was actually stored, not what was asked for, so
	 * a failed NVS write is visible at the gateway rather than only in this
	 * board's log. A half-applied survey is the one outcome worth being
	 * loud about. */
	int n = apos_frame_setpos_ack_build(tx_buf, sizeof(tx_buf),
					   uwb_config_short_addr(cfg),
					   UWB_ADDR_GATEWAY_RESERVED, session,
					   cfg->x, cfg->y, cfg->z, ok);

	if (n < 0) {
		LOG_ERR("SETPOS_ACK build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);
	tx_now(tx_buf, (uint16_t)n, bg);
}

static void handle_survey_end(const uint8_t *buf, uint16_t plen)
{
	uint16_t session = 0;

	if (apos_frame_parse_survey_end(buf, plen, &session) != 0) {
		return;
	}
	if (session != window_session) {
		return;
	}
	window_session = 0;
	LOG_INF("{\"apos\":\"window closed\",\"reason\":\"survey end\"}");
}

bool apos_node_on_rx(const uint8_t *buf, uint16_t plen, uwb_config_t *cfg,
		     uint8_t *seq, struct beacon_guard *bg)
{
	if (!apos_frame_is_apos(buf, plen)) {
		return false;
	}

	/* Only the gateway issues survey traffic. Filtering here rather than
	 * per-subtype means a stray or spoofed APOS frame from another anchor
	 * cannot open a window or move this board's coordinates. */
	if (apos_frame_src(buf) != UWB_ADDR_GATEWAY_RESERVED) {
		return true;
	}

	uint16_t dest = apos_frame_dest(buf);

	if (dest != APOS_ADDR_BCAST && dest != uwb_config_short_addr(cfg)) {
		return true;
	}

	switch (apos_frame_subtype(buf)) {
	case APOS_SUB_SURVEY_BEGIN:
		handle_survey_begin(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_SETPOS:
		window_refresh();
		handle_setpos(buf, plen, cfg, seq, bg);
		break;
	case APOS_SUB_SURVEY_END:
		handle_survey_end(buf, plen);
		break;
	default:
		/* RANGE_CMD lands here until Task 7. Every other subtype is
		 * anchor-to-gateway and should never arrive with src 0x0000. */
		break;
	}
	return true;
}
```

Add to `src/apos_node.h`, next to the other constants — `window_refresh()` above needs it:

```c
/* Seconds a window is extended by on each in-session APOS frame. Long enough
 * that a slow ranging phase never lets the window lapse mid-survey, short
 * enough that a gateway which stops talking releases this board promptly. */
#define APOS_NODE_REFRESH_S 60u
```

- [ ] **Step 4: Add to the build**

In `CMakeLists.txt`, add to the main `target_sources(app PRIVATE ...)` list in alphabetical order: `src/apos_frame.c`, `src/apos_geom.c`, `src/apos_node.c`, `src/apos_table.c` (all four go before `src/beacon_guard.c`).

- [ ] **Step 5: Verify it builds**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: builds clean. `apos_node_on_rx()` has no caller yet — that is Task 8 — so expect no behaviour change on hardware from this task alone.

Also re-run every host test, because `apos_table.h` and `apos_frame.h` were included by new code and a header change would show up here first:

```powershell
./tests/apos_geom/test_apos_geom.exe
./tests/apos_table/test_apos_table.exe
./tests/apos_frame/test_apos_frame.exe
```

Expected: three `PASSED`.

- [ ] **Step 6: Commit**

```bash
git add src/apos_node.h src/apos_node.c CMakeLists.txt prj.conf
git commit -m "feat(apos): anchor-side survey window, EUI enumeration and SETPOS"
```

---

### Task 7: `apos_node` — the commanded ranging batch

**Files:**
- Modify: `src/apos_node.h` (add `APOS_MAX_EXCHANGES`, `APOS_MIN_N_OK`)
- Modify: `src/apos_node.c` (add `handle_range_cmd()` and its `RANGE_CMD` case)

**Interfaces:**
- Consumes: `ss_initiator_enter/_range/_leave` from Task 5, `apos_frame_parse_range_cmd`, `apos_frame_range_rsp_build`.
- Produces: no new public symbols — `apos_node_on_rx()` gains the `APOS_SUB_RANGE_CMD` case.

- [ ] **Step 1: Add the two constants to the header**

Add to `src/apos_node.h` beside the others:

```c
/* Hard cap on one commanded batch, independent of what the RANGE_CMD asks for.
 * The gateway owns the tradeoff and sends 40, but this board must not be
 * talkable into an unbounded transmit run by a malformed or hostile command --
 * that is the failure mode the survey window exists to prevent, and a count
 * cap is the second half of it.
 *
 * 64 exchanges at ~5 ms is ~320 ms, during which this board is an initiator and
 * therefore blind to the beacon. Its beacon_guard prediction goes stale over
 * that span but stays trustworthy: BEACON_GUARD_MAX_MISSES is 4 superframes
 * (~800 ms). Raising this past ~150 exchanges would cross that line and the
 * guard would drop its lock mid-batch. */
#define APOS_MAX_EXCHANGES 64u

/* Below this many successful exchanges the batch is reported with its real
 * n_ok and the gateway discards it (apos_table_symmetrise's min_n_ok). Reported
 * rather than suppressed: "the pair cannot range" and "the command never
 * arrived" must not collapse into the same silence at the gateway. */
#define APOS_MIN_N_OK 10u
```

- [ ] **Step 2: Implement the handler**

Add `#include "ss_initiator.h"` to `src/apos_node.c`'s includes, and this above `apos_node_on_rx()`:

```c
/* Static, not on the stack: CONFIG_MAIN_STACK_SIZE is 4096 and this is 256 B. */
static int32_t batch[APOS_MAX_EXCHANGES];

/* Mean and standard deviation of the batch, in mm. Returns the count kept.
 *
 * A median-absolute-deviation filter rather than cal_math.c's cal_filtered_mean:
 * that one is tuned for a batch measured against a known reference at a known
 * distance, and here nothing is known in advance. The sd is the whole point --
 * it becomes the edge weight in the fit and the operator's per-pair quality
 * signal -- so it must describe the kept samples, not the raw ones. */
static uint8_t batch_stats(const int32_t *s, uint8_t n, int32_t *mean_out,
			   uint16_t *sd_out)
{
	if (n == 0) {
		*mean_out = 0;
		*sd_out = 0;
		return 0;
	}

	/* Median via a partial insertion sort into a scratch copy: n <= 64, so
	 * O(n^2) is irrelevant and a real sort would be more code. */
	int32_t tmp[APOS_MAX_EXCHANGES];

	memcpy(tmp, s, (size_t)n * sizeof(tmp[0]));
	for (uint8_t i = 1; i < n; i++) {
		int32_t v = tmp[i];
		int8_t j = (int8_t)(i - 1);

		while (j >= 0 && tmp[j] > v) {
			tmp[j + 1] = tmp[j];
			j--;
		}
		tmp[j + 1] = v;
	}

	int32_t median = tmp[n / 2];

	/* Reject anything more than 300 mm from the median. A reflection or a
	 * half-decoded frame lands far outside that; honest ranging noise on
	 * this hardware is tens of millimetres. */
	int64_t sum = 0;
	uint8_t kept = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= 300) {
			sum += s[i];
			kept++;
		}
	}
	if (kept == 0) {
		*mean_out = median;
		*sd_out = 0;
		return 0;
	}

	int32_t mean = (int32_t)(sum / kept);
	int64_t var = 0;

	for (uint8_t i = 0; i < n; i++) {
		int32_t dev = s[i] - median;

		if (dev < 0) {
			dev = -dev;
		}
		if (dev <= 300) {
			int64_t d = (int64_t)s[i] - mean;

			var += d * d;
		}
	}

	/* Population sd over the kept samples. Divide by kept, not kept-1: with
	 * 40 samples the difference is under 2 % and this feeds a weight, not a
	 * hypothesis test. */
	uint32_t sd = 0;

	if (kept > 1) {
		int64_t v = var / kept;

		/* Integer sqrt: no float needed and this runs on the SLAVE
		 * loop's stack. */
		while ((int64_t)(sd + 1) * (sd + 1) <= v) {
			sd++;
		}
	}

	*mean_out = mean;
	*sd_out = (sd > UINT16_MAX) ? UINT16_MAX : (uint16_t)sd;
	return kept;
}

static void handle_range_cmd(const uint8_t *buf, uint16_t plen,
			     const uwb_config_t *cfg, uint8_t *seq,
			     struct beacon_guard *bg)
{
	uint16_t session = 0, peer_addr = 0;
	uint8_t n_exchanges = 0;

	if (apos_frame_parse_range_cmd(buf, plen, &session, &peer_addr,
				       &n_exchanges) != 0) {
		return;
	}

	/* Gate one: the window. Gate two: the session. src == 0x0000 was
	 * already checked by the caller. All three must hold before this board
	 * transmits a single unsolicited poll -- see apos_node.h. */
	if (!apos_node_window_open() || session != window_session) {
		LOG_WRN("RANGE_CMD refused — window closed or session mismatch");
		return;
	}
	if (peer_addr == UWB_ADDR_GATEWAY_RESERVED ||
	    peer_addr == uwb_config_short_addr(cfg)) {
		/* The gateway never answers a poll, and polling ourselves is
		 * meaningless. Either means the gateway's table is wrong. */
		LOG_WRN("RANGE_CMD refused — peer 0x%04X is not rangeable",
			peer_addr);
		return;
	}

	uint8_t want = (n_exchanges > APOS_MAX_EXCHANGES)
		       ? (uint8_t)APOS_MAX_EXCHANGES : n_exchanges;
	uint8_t valid = 0;

	/* The responder filters on the low byte of its short address, which is
	 * what the tag polls with too (anchor_respond.c:122). */
	uint8_t peer_wire_id = (uint8_t)(peer_addr & 0xFFu);

	ss_initiator_enter();
	for (uint8_t i = 0; i < want; i++) {
		int32_t mm = ss_initiator_range(peer_wire_id);

		if (mm != INT32_MIN) {
			batch[valid++] = mm;
		}
		/* Yields to the shell so the console survives the batch, and
		 * decorrelates consecutive exchanges slightly. */
		k_sleep(K_MSEC(2));
	}
	ss_initiator_leave();

	int32_t mean = 0;
	uint16_t sd = 0;
	uint8_t kept = batch_stats(batch, valid, &mean, &sd);

	LOG_INF("{\"apos_range\":{\"peer\":\"0x%04X\",\"tried\":%u,\"ok\":%u,"
		"\"kept\":%u,\"mean_mm\":%d,\"sd_mm\":%u}}",
		peer_addr, want, valid, kept, mean, sd);

	/* n_ok reports the KEPT count, not the raw count: the gateway's
	 * min_n_ok threshold is about how many samples the mean rests on, and
	 * outliers that were filtered out did not contribute to it. */
	int n = apos_frame_range_rsp_build(tx_buf, sizeof(tx_buf),
					  uwb_config_short_addr(cfg),
					  UWB_ADDR_GATEWAY_RESERVED, session,
					  peer_addr, mean, sd, kept);

	if (n < 0) {
		LOG_ERR("RANGE_RSP build failed (%d)", n);
		return;
	}
	apos_frame_set_seq(tx_buf, (*seq)++);

	/* ss_initiator_leave() has already restored the interrupt mask and
	 * cleared the RX timeouts, so a normal immediate TX is legal again. The
	 * SLAVE loop re-arms RX when this returns. */
	tx_now(tx_buf, (uint16_t)n, bg);
}
```

Replace the `default:` comment block in `apos_node_on_rx()`'s switch with a real case:

```c
	case APOS_SUB_RANGE_CMD:
		window_refresh();
		handle_range_cmd(buf, plen, cfg, seq, bg);
		break;
	default:
		/* Every remaining subtype is anchor-to-gateway and should never
		 * arrive with src 0x0000. */
		break;
```

`handle_range_cmd()` takes `const uwb_config_t *cfg` while `apos_node_on_rx()` holds a mutable one — that is fine and deliberate: only `handle_setpos()` has any business writing to it.

- [ ] **Step 3: Verify it builds and the host tests still pass**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: builds clean, no new warnings. `memcpy`/`UINT16_MAX` need `<string.h>` (already included) and `<stdint.h>` (via the header).

- [ ] **Step 4: Commit**

```bash
git add src/apos_node.h src/apos_node.c
git commit -m "feat(apos): commanded ranging batch with median-filtered mean and sd

Bounded twice over: the survey window plus a hard APOS_MAX_EXCHANGES cap, so
no command can turn a production anchor into an unbounded transmitter. The
cap is also sized against BEACON_GUARD_MAX_MISSES, since an initiator is
blind to the beacon for the length of its batch."
```

---

### Task 8: Wire the survey into the SLAVE loop and close the `position_valid` hole

This is the task that fixes the hazard `CLAUDE.md` records: an unpositioned SLAVE currently answers ranging polls reporting `(0, 0)`, and three anchors all claiming the origin yield a confidently meaningless fix with no error anywhere.

**Files:**
- Modify: `src/anchor_respond.h`, `src/anchor_respond.c`
- Modify: `src/uwb_slave.c`
- Modify: `src/cal_run.c`

**Interfaces:**
- Consumes: `apos_node_on_rx()`, `apos_node_window_open()`, `apos_node_init()`.
- Produces: `anchor_respond_wave_poll()` gains a trailing `bool allow_unpositioned` parameter. Every caller must be updated — there are exactly two.

- [ ] **Step 1: Change the responder signature and add the gate**

In `src/anchor_respond.h`, change the declaration to:

```c
/* Answer a legacy WAVE/0xE0 poll addressed to this anchor with a VEWA/0xE1
 * response carrying our (x, y). Ignores anything else.
 *
 * allow_unpositioned relaxes the position_valid requirement. An anchor with no
 * position MUST NOT answer a tag: the response encodes (0, 0) and the tag
 * cannot tell that apart from a real coordinate, so three unpositioned anchors
 * produce a confident meaningless fix with no error reported anywhere.
 *
 * But during an anchor survey every anchor is unpositioned and must still
 * answer its PEERS, or the survey can never bootstrap a cold deployment. The
 * caller resolves that: uwb_slave.c passes apos_node_window_open(), so the
 * relaxation lasts exactly as long as the gateway-opened survey window;
 * cal_run.c passes true, because calibrating an unpositioned board is normal
 * and there is no gateway or tag on air during calibration. */
void anchor_respond_wave_poll(const uint8_t *buf, uint16_t len,
			      uint64_t poll_rx_ts, const uwb_config_t *cfg,
			      uint8_t *seq, struct beacon_guard *bg,
			      bool allow_unpositioned);
```

In `src/anchor_respond.c`, add the matching parameter to the definition and insert the gate immediately after the `wire_id` line, before the length check:

```c
	/* Refusing is strictly better than answering with (0, 0): a silent
	 * wrong coordinate is undebuggable from the tag's side, while a silent
	 * anchor shows up immediately as a missing anchor. Mirrors the
	 * gateway's existing refusal to beacon unpositioned
	 * (uwb_gateway.c:253). */
	if (!cfg->position_valid && !allow_unpositioned) {
		return;
	}
```

- [ ] **Step 2: Update `cal_run.c`**

In `src/cal_run.c`, the single call becomes:

```c
					anchor_respond_wave_poll(rx_buf, plen, rx_ts,
								 &cfg_live,
								 &frame_seq_nb, NULL,
								 true);
```

- [ ] **Step 3: Wire `apos_node` into the SLAVE loop**

In `src/uwb_slave.c`:

Add `#include "apos_node.h"` to the includes.

The loop currently makes `cfg` point at a `const` snapshot. `apos_node_on_rx()` needs a mutable one for `SETPOS`, so keep both names. Replace:

```c
	uwb_config_t cfg_snapshot = *cfg;
	cfg = &cfg_snapshot;
```

with:

```c
	/* Snapshot at mode-entry -- see the comment below for why. Kept as a
	 * named mutable object as well as the const view: apos_node_on_rx()
	 * writes a surveyed position straight into it so `apos apply` takes
	 * effect without a reboot, while every other consumer keeps the const
	 * view and the documented "changes take effect only on reboot"
	 * contract for id and antenna delay. */
	static uwb_config_t cfg_snapshot;

	cfg_snapshot = *cfg;
	cfg = &cfg_snapshot;
```

(`static` because the loop never returns and a 24-byte struct on a 4096-byte main stack is fine either way — but `apos_node` holds no pointer to it, so automatic storage would also be correct. `static` is chosen to make it obvious the object outlives every call.)

Add `apos_node_init();` immediately before `beacon_guard_init(...)`.

Replace the dispatch chain:

```c
			/* Offered to each in turn; each ignores what is not its own. */
			anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
						 &bguard);
			anchor_respond_discovery(rx_buf, plen, rx_ts, cfg, &frame_seq_nb,
						 cir_power, cir_quality, &bguard);
			observe_beacon(rx_buf, plen, cfg, rx_ts);
```

with:

```c
			/* Offered to each in turn; each ignores what is not its
			 * own. APOS goes first and short-circuits: a survey
			 * frame is never also a ranging poll or a beacon, and a
			 * RANGE_CMD blocks for a whole batch, so there is no
			 * point offering it to anyone else afterwards. */
			if (!apos_node_on_rx(rx_buf, plen, &cfg_snapshot,
					     &frame_seq_nb, &bguard)) {
				anchor_respond_wave_poll(rx_buf, plen, rx_ts, cfg,
							 &frame_seq_nb, &bguard,
							 apos_node_window_open());
				anchor_respond_discovery(rx_buf, plen, rx_ts, cfg,
							 &frame_seq_nb, cir_power,
							 cir_quality, &bguard);
				observe_beacon(rx_buf, plen, cfg, rx_ts);
			}
```

- [ ] **Step 4: Verify both images build**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
west build -b ancla_esp32s3/esp32s3/procpu --pristine -- -DEXTRA_CONF_FILE=cal.conf
```

Expected: both clean. Confirm the compiler found every call site — `git grep -n anchor_respond_wave_poll` must show exactly three hits (the declaration, the definition, and two calls: `uwb_slave.c` and `cal_run.c`), all with seven arguments.

- [ ] **Step 5: Hardware check — an unpositioned anchor now goes silent**

This is the first behaviour change an operator can observe, and it is a deliberate regression for unconfigured boards, so verify it explicitly rather than assuming.

```powershell
west flash
west espressif monitor -p COM5
```

On the console:

```
anchor reset
kernel reboot cold
```

With a tag and two other anchors running, confirm on the sniffer that this board answers **no** WAVE poll and **no** DISCOVERY. Then:

```
anchor pos 0 0 0
kernel reboot cold
```

Confirm it answers again. Note that `anchor pos 0 0 0` is a *legitimate* position — `position_valid` is what changed, not the coordinates — which is exactly why the flag and not the value is what the gate tests.

- [ ] **Step 6: Commit**

```bash
git add src/anchor_respond.h src/anchor_respond.c src/uwb_slave.c src/cal_run.c
git commit -m "fix(anchor): refuse ranging polls when unpositioned

An unpositioned SLAVE answered with (0, 0), which a tag cannot tell from a
real coordinate -- three such anchors give a confident meaningless fix with
no error anywhere. This has already cost a bench session.

The reason it could not simply be gated on position_valid is that a survey of
a cold deployment needs every (unpositioned) anchor to answer its peers. The
gateway-opened survey window resolves it: relaxed for exactly as long as a
survey is running, strict otherwise."
```

---

### Task 9: `apos_store` — the gateway's persistent EUI→coords table

**Files:**
- Create: `src/apos_store.h` (pure C — no Zephyr, because `pos_json.c` includes it), `src/apos_store.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `APOS_MAX_NODES` from `apos_geom.h`, `APOS_EUI_LEN` from `apos_table.h`.
- Produces: `struct apos_survey_node {uint8_t eui[APOS_EUI_LEN]; uint16_t short_addr; float x, y, z;}`, `struct apos_survey`, `void apos_store_init(void)`, `const struct apos_survey *apos_store_get(void)`, `int apos_store_save(const struct apos_survey *s)`, `int apos_store_set_ref(double lat, double lon)`, `int apos_store_clear(void)`.

- [ ] **Step 1: Write the header**

Create `src/apos_store.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The gateway's record of the last completed anchor survey, persisted so the
 * anchors MQTT payload survives a reboot and so a replaced board can be
 * recognised as new.
 *
 * Keyed by EUI-64, not by anchor_id or short address. That is the point: an
 * `anchor id` swap once left a coordinate stranded on the wrong board because
 * the deployment's state was keyed by id. EUI-64 travels with the board.
 *
 * The header is deliberately pure C with no Zephyr includes: pos_json.c consumes
 * struct apos_survey and is host-tested. Only apos_store.c touches settings.
 */

#ifndef APOS_STORE_H
#define APOS_STORE_H

#include "apos_geom.h"    /* APOS_MAX_NODES */
#include "apos_table.h"   /* APOS_EUI_LEN   */

#include <stdbool.h>
#include <stdint.h>

struct apos_survey_node {
	uint8_t  eui[APOS_EUI_LEN];
	uint16_t short_addr;
	float    x, y, z;
};

struct apos_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	bool     valid;      /* false until a survey has been applied */

	/* Geographic anchor for the local frame, from `apos ref`. The platform
	 * needs one real lat/long to place the survey on a map; every other
	 * anchor is positioned relative to it in metres. Belongs to the origin
	 * node, which is node[0] by construction (see apos_store_save). */
	double   ref_lat, ref_lon;
	bool     ref_valid;
};

/* Load the persisted survey into the module cache. Call from main() before the
 * uplink thread starts -- net_uplink publishes the anchors payload on connect,
 * which reads this. */
void apos_store_init(void);

/* The cached survey. Never NULL; check ->valid. */
const struct apos_survey *apos_store_get(void);

/* Replace the cached survey and persist it. node[0] MUST be the gauge origin:
 * the geographic reference belongs to it, and pos_json relies on that ordering.
 *
 * ref_lat/ref_lon/ref_valid in *s are ignored -- the reference is set
 * independently by apos_store_set_ref() and survives a re-survey, because
 * re-measuring the geometry does not move the building. Returns 0 or a negative
 * errno from the settings layer. */
int apos_store_save(const struct apos_survey *s);

/* Set and persist the geographic reference. Independent of the survey so it can
 * be entered once for a site and then left alone. */
int apos_store_set_ref(double lat, double lon);

/* Invalidate and erase. `apos_store_get()->valid` becomes false, so the anchors
 * payload falls back to the stub. Returns 0 or a negative errno. */
int apos_store_clear(void);

#endif /* APOS_STORE_H */
```

- [ ] **Step 2: Implement**

Create `src/apos_store.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_store.h.
 *
 * Written as two blobs under "apos/", deliberately unlike uwb_store.c's
 * one-key-per-field scheme. That scheme exists so a new field does not force a
 * version byte and a wipe -- the right call for a config struct that grows. A
 * survey is different: it is one atomic measurement result in which a partially
 * applied set of coordinates is worse than none, and the whole record is always
 * written and read together. The geographic reference is the second blob because
 * it is entered once per site and must survive a re-survey.
 */

#include "apos_store.h"

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(apos_store, LOG_LEVEL_INF);

#define KEY_SURVEY "apos/survey"
#define KEY_REF    "apos/ref"

/* Only the geometry, so adding a field to struct apos_survey does not silently
 * change the stored layout: this record is what is written, and the size check
 * on load is what catches a layout change. */
struct stored_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t                 n_nodes;
	uint8_t                 valid;
};

struct stored_ref {
	double  lat;
	double  lon;
	uint8_t valid;
};

static struct apos_survey g_survey;

static int read_val(settings_read_cb read_cb, void *cb_arg, void *dst,
		    size_t len)
{
	ssize_t n = read_cb(cb_arg, dst, len);

	if (n != (ssize_t)len) {
		return -EINVAL;
	}
	return 0;
}

static int apos_settings_set(const char *key, size_t len,
			     settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(key, "survey") == 0) {
		struct stored_survey s;

		if (len != sizeof(s)) {
			LOG_WRN("stored survey size %u invalid — expected %u, "
				"treating as no survey",
				(unsigned int)len, (unsigned int)sizeof(s));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &s, sizeof(s))) {
			return -EINVAL;
		}
		if (s.n_nodes > APOS_MAX_NODES) {
			LOG_WRN("stored survey claims %u nodes, max is %u — "
				"treating as no survey", s.n_nodes,
				APOS_MAX_NODES);
			return -EINVAL;
		}
		memcpy(g_survey.node, s.node, sizeof(g_survey.node));
		g_survey.n_nodes = s.n_nodes;
		g_survey.valid = s.valid != 0u;
		return 0;
	}

	if (strcmp(key, "ref") == 0) {
		struct stored_ref r;

		if (len != sizeof(r)) {
			LOG_WRN("stored ref size %u invalid — expected %u, "
				"keeping none",
				(unsigned int)len, (unsigned int)sizeof(r));
			return -EINVAL;
		}
		if (read_val(read_cb, cb_arg, &r, sizeof(r))) {
			return -EINVAL;
		}
		g_survey.ref_lat = r.lat;
		g_survey.ref_lon = r.lon;
		g_survey.ref_valid = r.valid != 0u;
		return 0;
	}

	/* An unrecognised key is a field from a newer firmware; ignore it. */
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(apos, "apos", NULL, apos_settings_set, NULL,
			       NULL);

void apos_store_init(void)
{
	/* No settings_subsys_init()/settings_load() here: uwb_store_init()
	 * already does both from main(), and settings_load() runs EVERY
	 * registered handler including this module's. Calling it again would
	 * re-read the whole subtree for no gain. This function exists so the
	 * ordering requirement is explicit at the call site rather than
	 * implicit in a static initialiser. */
	if (g_survey.valid) {
		LOG_INF("{\"apos_store\":{\"nodes\":%u,\"ref_valid\":%u}}",
			g_survey.n_nodes, g_survey.ref_valid ? 1u : 0u);
	} else {
		LOG_INF("{\"apos_store\":\"no survey — anchors payload will "
			"use the stub\"}");
	}
}

const struct apos_survey *apos_store_get(void)
{
	return &g_survey;
}

int apos_store_save(const struct apos_survey *s)
{
	if (!s || s->n_nodes > APOS_MAX_NODES) {
		return -EINVAL;
	}

	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));
	memcpy(rec.node, s->node, sizeof(rec.node));
	rec.n_nodes = s->n_nodes;
	rec.valid = s->valid ? 1u : 0u;

	int ret = settings_save_one(KEY_SURVEY, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_SURVEY, ret);
		return ret;
	}

	/* Cache updated only after the write succeeded, so a failed save leaves
	 * apos_store_get() reporting what is actually on flash. */
	memcpy(g_survey.node, s->node, sizeof(g_survey.node));
	g_survey.n_nodes = s->n_nodes;
	g_survey.valid = s->valid;

	return 0;
}

int apos_store_set_ref(double lat, double lon)
{
	struct stored_ref rec = {.lat = lat, .lon = lon, .valid = 1u};

	int ret = settings_save_one(KEY_REF, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to persist %s (%d)", KEY_REF, ret);
		return ret;
	}

	g_survey.ref_lat = lat;
	g_survey.ref_lon = lon;
	g_survey.ref_valid = true;
	return 0;
}

int apos_store_clear(void)
{
	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));

	int ret = settings_save_one(KEY_SURVEY, &rec, sizeof(rec));

	if (ret) {
		LOG_ERR("failed to clear %s (%d)", KEY_SURVEY, ret);
		return ret;
	}

	memset(g_survey.node, 0, sizeof(g_survey.node));
	g_survey.n_nodes = 0;
	g_survey.valid = false;
	return 0;
}
```

- [ ] **Step 3: Call `apos_store_init()` from `main()`**

In `src/main.c`, add `#include "apos_store.h"` and insert the call immediately after `uwb_store_init();` — it must run after that, because `uwb_store_init()`'s `settings_load()` is what actually populates this module's cache:

```c
	uwb_store_init();
	/* After uwb_store_init(): its settings_load() is what runs this
	 * module's handler and fills the cache. Before net_uplink_start():
	 * the uplink publishes the anchors payload on connect and reads it. */
	apos_store_init();
	log_config(cfg);
```

- [ ] **Step 4: Add to the build and verify**

Add `src/apos_store.c` to the main `target_sources(app PRIVATE ...)` list, after `src/apos_node.c`.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: builds clean. On boot the log now carries `{"apos_store":"no survey — anchors payload will use the stub"}`.

Confirm the pure header really is Zephyr-free, since Task 13 depends on it:

```powershell
gcc -Wall -Wextra -Isrc -fsyntax-only -x c src/apos_store.h
```

Expected: no output. A failure here means a Zephyr include leaked in and `pos_json`'s host test will not compile.

- [ ] **Step 5: Commit**

```bash
git add src/apos_store.h src/apos_store.c src/main.c CMakeLists.txt
git commit -m "feat(apos): persist the survey table keyed by EUI-64

Binding coordinates to EUI rather than anchor_id is what stops an id swap
stranding a coordinate on the wrong board. Stored as one atomic blob, unlike
uwb_store's per-field keys: a partially applied survey is worse than none. The
geographic reference is a separate key so it survives a re-survey."
```

---

### Task 10: `apos_gw` — the step machine and the enumeration phase

**Files:**
- Create: `src/apos_gw.h`, `src/apos_gw.c`
- Modify: `src/uwb_gateway.c`, `CMakeLists.txt`

**Interfaces:**
- Consumes: `apos_frame.h`, `apos_table.h`, `apos_geom.h`, `apos_store.h`, `uwb_dwtime.h`.
- Produces: `enum apos_gw_phase {APOS_GW_IDLE=0, APOS_GW_ENUM, APOS_GW_RANGE, APOS_GW_APPLY}`, `struct apos_gw_status`, `void apos_gw_init(void)`, `void apos_gw_on_rx(const uint8_t *buf, uint16_t plen)`, `void apos_gw_step(uint32_t avail_uus, uint8_t *seq)`, `bool apos_gw_busy(void)`, `void apos_gw_get_status(struct apos_gw_status *out)`, `const struct apos_table *apos_gw_table(void)`, `int apos_gw_start_enum(void)`, `int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane, uint16_t up)`, `bool apos_gw_gauge_set(void)`.

- [ ] **Step 1: Write the header**

Create `src/apos_gw.h`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gateway orchestration for the anchor survey: enumerate, command every pair to
 * range, solve the geometry, push the answer back.
 *
 * A STEP MACHINE, not a thread and not a blocking routine. The gateway loop runs
 * at K_PRIO_COOP(0) and the beacon is the whole network's time base, so nothing
 * on that path may hold it for longer than BEACON_ARM_MARGIN_UUS. apos_gw_step()
 * emits at most ONE frame and returns; a survey therefore unfolds over many
 * superframes and the beacon cadence is never disturbed. Survey timeouts are
 * measured with k_uptime_get() deltas observed across steps, so a step that gets
 * skipped costs latency and never correctness.
 *
 * The shell never transmits. `apos run` sets state and returns; this module does
 * the work from the gateway loop and logs the result as JSON when it completes.
 * Two threads on the DW3220's SPI bus at once would corrupt both.
 */

#ifndef APOS_GW_H
#define APOS_GW_H

#include "apos_geom.h"
#include "apos_table.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum apos_gw_phase {
	APOS_GW_IDLE  = 0,
	APOS_GW_ENUM  = 1,
	APOS_GW_RANGE = 2,
	APOS_GW_APPLY = 3,
};

/* How long a survey window anchors are told to hold open. Generously longer
 * than the ranging phase for the largest supported deployment: 56 ordered pairs
 * at ~300 ms each is under 20 s, and every APOS frame refreshes the window
 * anyway (APOS_NODE_REFRESH_S). */
#define APOS_GW_WINDOW_S 120u

/* SURVEY_BEGIN is broadcast this many times, spaced this far apart, and the
 * replies are unioned. One round would be enough if no two EUIs hashed to the
 * same stagger slot; three makes a slot collision cost a retry instead of a
 * missing anchor. The gap must exceed the worst-case stagger
 * (APOS_ENUM_SLOTS * APOS_ENUM_SLOT_MS = 210 ms) plus reply airtime. */
#define APOS_GW_ENUM_ROUNDS  3u
#define APOS_GW_ENUM_GAP_MS  400u

/* Rough cost of one apos_gw_step() transmission, used to decide whether there
 * is room before the next beacon. One frame plus the bounded TXFRS wait. */
#define APOS_GW_STEP_BUDGET_UUS 3000u

struct apos_gw_status {
	uint8_t  phase;        /* enum apos_gw_phase */
	uint16_t session;
	uint8_t  n_peers;
	uint16_t meas_done;    /* ordered pairs attempted so far */
	uint16_t meas_total;   /* ordered pairs in this run */
	uint8_t  applied_ok;
	uint8_t  applied_fail;
	bool     have_result;
};

void apos_gw_init(void);

/* Consume one received frame. Ignores anything that is not an APOS frame from a
 * known peer for the current session. */
void apos_gw_on_rx(const uint8_t *buf, uint16_t plen);

/* Advance the survey by at most one transmission. avail_uus is how much time is
 * left before the beacon must be armed; the step does nothing if that is less
 * than APOS_GW_STEP_BUDGET_UUS. seq is the gateway's shared frame sequence
 * counter, so survey frames stay in the same numbering as beacons and grants --
 * which is what makes a sniffer capture readable. */
void apos_gw_step(uint32_t avail_uus, uint8_t *seq);

bool apos_gw_busy(void);
void apos_gw_get_status(struct apos_gw_status *out);

/* The live table, for `apos enum` to print. Never NULL. */
const struct apos_table *apos_gw_table(void);

/* Begin an enumeration-only pass. Returns 0, or -EBUSY if a survey is running. */
int apos_gw_start_enum(void);

/* Record the gauge as SHORT ADDRESSES, not node indices: indices are an artefact
 * of the order anchors happened to answer enumeration in, and would silently
 * point at different boards after a re-enumeration. Addresses are resolved to
 * indices at solve time.
 *
 * Returns 0, -EINVAL if the four are not distinct, or -EBUSY while a survey
 * runs. Does NOT require the addresses to be enumerated yet -- an operator may
 * legitimately set the gauge from a site sketch before powering the array. */
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      uint16_t up);

bool apos_gw_gauge_set(void);

#endif /* APOS_GW_H */
```

- [ ] **Step 2: Implement the skeleton and the ENUM phase**

Create `src/apos_gw.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See apos_gw.h.
 */

#include "apos_gw.h"

#include "apos_frame.h"
#include "apos_node.h"    /* APOS_MIN_N_OK, APOS_ENUM_* for the gap derivation */
#include "apos_store.h"
#include "uwb_config.h"
#include "uwb_dwtime.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(apos_gw, LOG_LEVEL_INF);

/* Same reasoning as apos_node's: every APOS transmission from here is
 * IMMEDIATE, so this only covers a frame's airtime, not a scheduled delay. */
#define TX_COMPLETE_TIMEOUT_MS 8

static struct apos_table tbl;
static struct apos_result res;
static bool have_result;

static uint8_t phase = APOS_GW_IDLE;
static uint16_t session;

/* Gauge as short addresses; 0 means unset, since 0x0000 is the gateway and can
 * never be a survey node. */
static uint16_t gauge_addr[4];

static float zoff_m;

/* ENUM phase */
static uint8_t enum_round;
static int64_t next_action_ms;

static uint8_t tx_buf[APOS_LEN_MAX];

void apos_gw_init(void)
{
	apos_table_init(&tbl);
	memset(&res, 0, sizeof(res));
	memset(gauge_addr, 0, sizeof(gauge_addr));
	have_result = false;
	phase = APOS_GW_IDLE;
	session = 0;
	zoff_m = 0.0f;
}

bool apos_gw_busy(void)
{
	return phase != APOS_GW_IDLE;
}

const struct apos_table *apos_gw_table(void)
{
	return &tbl;
}

bool apos_gw_gauge_set(void)
{
	return gauge_addr[0] != 0u && gauge_addr[1] != 0u &&
	       gauge_addr[2] != 0u && gauge_addr[3] != 0u;
}

void apos_gw_get_status(struct apos_gw_status *out)
{
	if (!out) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->phase = phase;
	out->session = session;
	out->n_peers = tbl.n_peers;
	out->have_result = have_result;
}

int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      uint16_t up)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	const uint16_t a[4] = {origin, xaxis, plane, up};

	for (int i = 0; i < 4; i++) {
		if (a[i] == 0u || a[i] == UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int j = i + 1; j < 4; j++) {
			if (a[i] == a[j]) {
				return -EINVAL;
			}
		}
	}
	memcpy(gauge_addr, a, sizeof(gauge_addr));
	return 0;
}

/* Transmit immediately. No beacon guard here, unlike apos_node: the caller only
 * calls apos_gw_step() when avail_uus leaves room before the beacon must be
 * armed, and the gateway owns the beacon rather than predicting it. */
static bool tx_now(uint16_t len, uint8_t *seq)
{
	apos_frame_set_seq(tx_buf, (*seq)++);

	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	dwt_writetxdata(len, tx_buf, 0);
	dwt_writetxfctrl((uint16_t)(len + FCS_LEN), 0, 0);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX failed to start");
		return false;
	}
	if (!uwb_wait_for_sysstatus_lo(DWT_INT_TXFRS_BIT_MASK,
				       TX_COMPLETE_TIMEOUT_MS)) {
		dwt_forcetrxoff();
		LOG_WRN("APOS TX started but TXFRS never completed — forced off");
		return false;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	return true;
}

/* A session of 0 is reserved as "none", so retry until non-zero. */
static uint16_t new_session(void)
{
	uint16_t s = 0;

	while (s == 0u) {
		s = (uint16_t)sys_rand32_get();
	}
	return s;
}

static void send_survey_begin(uint8_t *seq)
{
	int n = apos_frame_survey_begin_build(tx_buf, sizeof(tx_buf),
					     UWB_ADDR_GATEWAY_RESERVED,
					     session, APOS_GW_WINDOW_S);

	if (n < 0) {
		LOG_ERR("SURVEY_BEGIN build failed (%d)", n);
		return;
	}
	tx_now((uint16_t)n, seq);
}

int apos_gw_start_enum(void)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	apos_table_init(&tbl);
	have_result = false;
	session = new_session();
	enum_round = 0;
	next_action_ms = 0; /* fire on the very next step */
	phase = APOS_GW_ENUM;

	LOG_INF("{\"apos\":\"enumerating\",\"session\":%u}", session);
	return 0;
}

/* ---- RX ---- */

static void on_enum_rsp(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0;
	uint8_t eui[APOS_EUI_LEN];
	bool pv = false;
	float x = 0.0f, y = 0.0f, z = 0.0f;

	if (apos_frame_parse_enum_rsp(buf, plen, &sess, eui, &pv, &x, &y, &z)
	    != 0) {
		return;
	}
	if (sess != session) {
		return;
	}

	uint16_t addr = apos_frame_src(buf);
	int idx = apos_table_add_peer(&tbl, eui, addr, pv, x, y, z);

	if (idx == -EADDRINUSE) {
		/* Two distinct boards claiming one short address: both are
		 * configured with the same `anchor id`. Everything downstream
		 * addresses peers by short address, so continuing would range
		 * whichever board answered first and then push coordinates to
		 * whichever answered second. Abort loudly. */
		LOG_ERR("{\"apos_error\":\"duplicate short address 0x%04X — two "
			"boards share an `anchor id`. Fix with `anchor id` on "
			"one of them and re-run.\"}", addr);
		phase = APOS_GW_IDLE;
		return;
	}
	if (idx == -ENOSPC) {
		LOG_WRN("more than %u anchors answered — ignoring 0x%04X",
			APOS_MAX_NODES, addr);
		return;
	}
	if (idx < 0) {
		return;
	}

	LOG_INF("{\"apos_peer\":{\"idx\":%d,\"addr\":\"0x%04X\","
		"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\",\"pos_valid\":%u,"
		"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
		idx, addr, eui[0], eui[1], eui[2], eui[3], eui[4], eui[5],
		eui[6], eui[7], pv ? 1u : 0u, (double)x, (double)y, (double)z);
}

void apos_gw_on_rx(const uint8_t *buf, uint16_t plen)
{
	if (!apos_frame_is_apos(buf, plen)) {
		return;
	}
	if (phase == APOS_GW_IDLE) {
		return;
	}
	if (apos_frame_dest(buf) != UWB_ADDR_GATEWAY_RESERVED) {
		return;
	}

	switch (apos_frame_subtype(buf)) {
	case APOS_SUB_ENUM_RSP:
		on_enum_rsp(buf, plen);
		break;
	default:
		/* RANGE_RSP and SETPOS_ACK arrive in Tasks 11 and 12. */
		break;
	}
}

/* ---- Step ---- */

static void step_enum(uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (now < next_action_ms) {
		return;
	}

	if (enum_round >= APOS_GW_ENUM_ROUNDS) {
		LOG_INF("{\"apos_enum_done\":{\"peers\":%u}}", tbl.n_peers);
		phase = APOS_GW_IDLE;
		return;
	}

	send_survey_begin(seq);
	enum_round++;
	next_action_ms = now + APOS_GW_ENUM_GAP_MS;
}

void apos_gw_step(uint32_t avail_uus, uint8_t *seq)
{
	if (phase == APOS_GW_IDLE) {
		return;
	}
	if (avail_uus < APOS_GW_STEP_BUDGET_UUS) {
		/* Not enough room before the beacon. Skipping costs latency
		 * only: every deadline below is absolute wall-clock. */
		return;
	}

	switch (phase) {
	case APOS_GW_ENUM:
		step_enum(seq);
		break;
	default:
		break;
	}
}
```

- [ ] **Step 3: Wire it into the gateway loop**

In `src/uwb_gateway.c`:

Add `#include "apos_gw.h"` to the includes.

Add `apos_gw_init();` immediately after `gw_core_init(&ctx);`.

In `dispatch()`, add an APOS branch as the **first** test, so a survey frame is never mistaken for anything else:

```c
static void dispatch(struct gw_core_ctx *ctx, const uint8_t *buf, uint16_t len,
		     uint64_t rx_ts)
{
	if (apos_frame_is_apos(buf, len)) {
		apos_gw_on_rx(buf, len);
	} else if (uwb_frame_is_join(buf, len)) {
```

...leaving the rest of the chain unchanged. Add `#include "apos_frame.h"` for `apos_frame_is_apos()`.

At the **end** of the inner `for (;;)` body — after the `dispatch(...)` call, as the last statement before the closing brace — add the step:

```c
			/* Advance any running survey by at most one frame.
			 * Recomputed here rather than reusing to_beacon from
			 * the top of the loop: servicing an RX has consumed an
			 * unknown slice of the window, and arming the beacon
			 * late costs every node in the network. */
			uint32_t now2 = dwt_readsystimestamphi32();
			int32_t left = (int32_t)(next_beacon - now2);
			int32_t reserve = (int32_t)UUS_TO_HI32(
				BEACON_ARM_MARGIN_UUS + APOS_GW_STEP_BUDGET_UUS);

			if (left > reserve) {
				uint32_t span = (uint32_t)left -
					UUS_TO_HI32(BEACON_ARM_MARGIN_UUS);
				uint32_t avail_uus = (uint32_t)(
					((uint64_t)span << 8) / UUS_TO_DWT_TIME);

				apos_gw_step(avail_uus, &gw_seq);
			}
```

`next_beacon` and `gw_seq` are both already in scope there — `next_beacon` is declared at the top of the outer `while (1)`, and `gw_seq` is the file-scope counter `tx_beacon()` and `send_grant()` already share.

- [ ] **Step 4: Add to the build and verify**

Add `src/apos_gw.c` to the main `target_sources(app PRIVATE ...)` list, after `src/apos_frame.c` and before `src/apos_node.c`.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: builds clean. `sys_rand32_get()` needs `CONFIG_ENTROPY_GENERATOR` or Zephyr's fallback PRNG; if the link fails on it, add `CONFIG_TEST_RANDOM_GENERATOR=y` is **not** the fix — add `CONFIG_ENTROPY_GENERATOR=y` to `prj.conf`, since the ESP32-S3 has a hardware RNG. Report it if neither is available rather than substituting a counter: a predictable session id would let a stale reply from an abandoned run be accepted by a new one, which is the exact failure the session field exists to prevent.

- [ ] **Step 5: Add the shell commands**

Create `src/apos_shell.c`:

```c
/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `apos` command tree. Gateway-only in practice -- a SLAVE has no survey to
 * orchestrate -- but registered unconditionally, because refusing at the shell
 * with a clear message beats a command that silently does not exist on the board
 * the operator happens to be plugged into.
 *
 * No command transmits. Each sets state and returns; the gateway loop does the
 * radio work and logs the outcome. Two threads on the DW3220's SPI bus at once
 * would corrupt both -- the same rule cal_shell.c follows.
 */

#include "apos_gw.h"
#include "apos_store.h"
#include "uwb_config.h"

#include <zephyr/shell/shell.h>

#include <errno.h>
#include <stdlib.h>

/* Same reasoning as anchor_shell.c's parse_ul: strtoul() reports a non-numeric
 * argument as 0, and 0 must be rejected here rather than accepted as an
 * address. Accepts 0x-prefixed input, which is how `apos enum` prints them. */
static bool parse_addr(const char *arg, uint16_t *out)
{
	char *endptr;
	unsigned long v = strtoul(arg, &endptr, 0);

	if (endptr == arg || *endptr != '\0' || v == 0u || v > 0xFFFFu) {
		return false;
	}
	*out = (uint16_t)v;
	return true;
}

static int require_gateway(const struct shell *sh)
{
	if (uwb_config_get()->mode != UWB_MODE_GATEWAY) {
		shell_error(sh, "error: `apos` runs on the GATEWAY — this board "
				"is a %s. Set `anchor mode gateway` and reboot.",
			    uwb_config_mode_name(uwb_config_get()->mode));
		return -ENOTSUP;
	}
	return 0;
}

static int cmd_enum(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	rc = apos_gw_start_enum();
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: enumeration refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "enumerating — peers are logged as they answer, then "
			"run `apos show`");
	return 0;
}

/* origin=<addr> xaxis=<addr> plane=<addr> up=<addr>, in any order. Named rather
 * than positional because four bare hex addresses in a row is exactly the kind
 * of argument list that gets silently transposed, and a transposed gauge
 * produces a plausible-looking but wrong coordinate frame. */
static int cmd_gauge(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const keys[4] = {"origin=", "xaxis=", "plane=",
					    "up="};
	uint16_t val[4] = {0, 0, 0, 0};

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	for (size_t a = 1; a < argc; a++) {
		bool matched = false;

		for (int k = 0; k < 4; k++) {
			size_t klen = strlen(keys[k]);

			if (strncmp(argv[a], keys[k], klen) != 0) {
				continue;
			}
			if (!parse_addr(argv[a] + klen, &val[k])) {
				shell_error(sh, "error: bad address in \"%s\"",
					    argv[a]);
				return -EINVAL;
			}
			matched = true;
			break;
		}
		if (!matched) {
			shell_error(sh, "error: unexpected argument \"%s\" — "
					"expected origin=/xaxis=/plane=/up=",
				    argv[a]);
			return -EINVAL;
		}
	}

	for (int k = 0; k < 4; k++) {
		if (val[k] == 0u) {
			shell_error(sh, "error: missing %s<addr>", keys[k]);
			return -EINVAL;
		}
	}

	rc = apos_gw_set_gauge(val[0], val[1], val[2], val[3]);
	if (rc == -EINVAL) {
		shell_error(sh, "error: the four addresses must be distinct and "
				"none may be 0x0000 (the gateway)");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: gauge refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "{\"apos_gauge\":{\"origin\":\"0x%04X\","
			"\"xaxis\":\"0x%04X\",\"plane\":\"0x%04X\","
			"\"up\":\"0x%04X\"}}",
		    val[0], val[1], val[2], val[3]);
	return 0;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct apos_table *t = apos_gw_table();
	const struct apos_survey *s = apos_store_get();
	struct apos_gw_status st;

	apos_gw_get_status(&st);

	shell_print(sh, "{\"phase\":%u,\"session\":%u,\"gauge_set\":%u,"
			"\"have_result\":%u}",
		    st.phase, st.session, apos_gw_gauge_set() ? 1u : 0u,
		    st.have_result ? 1u : 0u);

	shell_print(sh, "enumerated (%u):", t->n_peers);
	for (uint8_t k = 0; k < t->n_peers; k++) {
		shell_print(sh, "  {\"idx\":%u,\"addr\":\"0x%04X\","
				"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\","
				"\"pos_valid\":%u}",
			    k, t->peer[k].short_addr,
			    t->peer[k].eui[0], t->peer[k].eui[1],
			    t->peer[k].eui[2], t->peer[k].eui[3],
			    t->peer[k].eui[4], t->peer[k].eui[5],
			    t->peer[k].eui[6], t->peer[k].eui[7],
			    t->peer[k].pos_valid ? 1u : 0u);
	}

	shell_print(sh, "stored survey: %s", s->valid ? "yes" : "none");
	for (uint8_t k = 0; k < s->n_nodes; k++) {
		shell_print(sh, "  {\"addr\":\"0x%04X\",\"x\":%.3f,\"y\":%.3f,"
				"\"z\":%.3f}",
			    s->node[k].short_addr, (double)s->node[k].x,
			    (double)s->node[k].y, (double)s->node[k].z);
	}
	if (s->ref_valid) {
		shell_print(sh, "  {\"ref_lat\":%.6f,\"ref_lon\":%.6f}",
			    s->ref_lat, s->ref_lon);
	} else {
		shell_print(sh, "  {\"ref\":\"unset — run `apos ref <lat> <lon>` "
				"or the platform cannot place the map\"}");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_apos,
	SHELL_CMD_ARG(enum,  NULL,
		      "enum — discover anchors over the air and print their "
		      "EUI-64 and short address",
		      cmd_enum, 1, 0),
	SHELL_CMD_ARG(gauge, NULL,
		      "gauge origin=<addr> xaxis=<addr> plane=<addr> up=<addr> "
		      "— pin the coordinate frame",
		      cmd_gauge, 5, 0),
	SHELL_CMD_ARG(show,  NULL,
		      "show — current phase, enumerated anchors and the stored "
		      "survey, as JSON",
		      cmd_show, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(apos, &sub_apos, "Anchor auto-positioning", NULL);
```

`strlen`/`strncmp` need `#include <string.h>` alongside the other includes.

Add `src/apos_shell.c` to `CMakeLists.txt` after `src/apos_node.c`.

- [ ] **Step 6: Hardware check — enumeration**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
west flash
west espressif monitor -p COM5
```

With one gateway and at least two SLAVE anchors powered:

```
apos enum
apos show
```

Expected: one `{"apos_peer":...}` line per anchor, each with a distinct EUI-64 and its short address, followed by `{"apos_enum_done":{"peers":N}}`. Confirm on the sniffer that the beacon `counter` keeps incrementing every superframe throughout — that is the property this task's step-machine design exists to preserve, and a stalled counter means the step is holding the loop too long.

Then verify the duplicate-id detection, since it is a headline benefit and cheap to test: set two anchors to the same `anchor id`, reboot both, and re-run `apos enum`. Expected: the `{"apos_error":"duplicate short address ..."}` line and the run aborting.

- [ ] **Step 7: Commit**

```bash
git add src/apos_gw.h src/apos_gw.c src/apos_shell.c src/uwb_gateway.c CMakeLists.txt
git commit -m "feat(apos): gateway step machine and EUI-64 enumeration

A step machine driven from the gateway loop rather than a thread: the beacon
is the network's time base and nothing on that path may hold it past
BEACON_ARM_MARGIN_UUS, so a step emits at most one frame and returns.

Enumeration reports two boards sharing an anchor id as a hard error, which
today produces silently wrong ranging with no indication anywhere."
```

---

### Task 11: `apos_gw` — the ranging phase, the solve, and `apos run`

**Files:**
- Modify: `src/apos_gw.h` (add range/accept constants, `apos_gw_start_run`, `apos_gw_result`, `apos_gw_accepted`)
- Modify: `src/apos_gw.c`
- Modify: `src/apos_shell.c` (add `apos run`)

**Interfaces:**
- Consumes: `apos_frame_range_cmd_build`, `apos_frame_parse_range_rsp`, `apos_table_add_meas`, `apos_table_symmetrise`, `apos_table_missing_pairs`, `apos_geom_solve`, `apos_geom_zoff`.
- Produces: `APOS_GW_N_EXCHANGES` (40), `APOS_GW_RANGE_TIMEOUT_MS` (3000), `APOS_GW_RANGE_RETRIES` (1), `APOS_ACCEPT_RMS_MM` (50), `APOS_ACCEPT_WORST_FACTOR` (3.0f), `APOS_ACCEPT_PLANARITY_MM` (100), `int apos_gw_start_run(void)`, `const struct apos_result *apos_gw_result(void)`, `bool apos_gw_accepted(void)`, `void apos_gw_set_zoff(float dz)`.

- [ ] **Step 1: Add the constants and declarations to the header**

Add to `src/apos_gw.h` beside the existing constants:

```c
/* Exchanges per commanded pair. The gateway owns this tradeoff, which is why it
 * is a RANGE_CMD field and not a constant on the anchor. 40 at ~5 ms is the
 * ~200 ms batch apos_node's beacon-staleness budget is sized around, and 40
 * samples make the reported sd a usable quality signal rather than noise. */
#define APOS_GW_N_EXCHANGES 40u

/* How long to wait for a RANGE_RSP. The batch itself is ~200 ms plus the
 * anchor's own beacon-guard suppressions, so this is generous by design: a
 * spurious timeout costs a retry and a wrong measurement costs the geometry. */
#define APOS_GW_RANGE_TIMEOUT_MS 3000u

/* One retry per ordered pair. A pair that fails twice is reported as a hole
 * rather than retried indefinitely -- the fit works around holes, and an
 * operator needs the run to finish so they can see WHICH pair failed. */
#define APOS_GW_RANGE_RETRIES 1u

/* Acceptance thresholds. Anchored on the antenna-delay cross-check, which
 * accepts |error| < 30 mm per pair: a fit over many such edges should land
 * inside 50 mm RMS, and anything much worse means a bad edge or a wrong gauge
 * rather than accumulated noise.
 *
 * These are the numbers most likely to need adjusting after the first real
 * bench run. They are thresholds on a REPORTED result, so raising one never
 * changes what was measured -- only whether `apos apply` will proceed. */
#define APOS_ACCEPT_RMS_MM       50u
#define APOS_ACCEPT_WORST_FACTOR 3.0f
/* Below this, the array is too close to coplanar for the solved z values to
 * mean anything. Does NOT block acceptance on its own -- x and y are still
 * good -- but it is reported, and the operator is told z is not survey-quality. */
#define APOS_ACCEPT_PLANARITY_MM 100u

/* Begin a full run: re-enumerate, range every ordered pair, solve, report.
 * Persists NOTHING. Returns 0, -EBUSY if a survey runs, or -EINVAL if the gauge
 * has not been set. */
int apos_gw_start_run(void);

/* The last solved result. Never NULL; check apos_gw_get_status()->have_result. */
const struct apos_result *apos_gw_result(void);

/* Whether the last result met every threshold in this header. */
bool apos_gw_accepted(void);

/* Shift z on every subsequent solve, moving z = 0 off the plane through the
 * three gauge anchors and onto the floor. Applied at solve time, so changing it
 * requires a re-run rather than silently rewriting a reported result. */
void apos_gw_set_zoff(float dz);
```

Also extend `struct apos_gw_status`'s use: `meas_done` and `meas_total` are already declared in Task 10 and get filled from this task on.

- [ ] **Step 2: Implement the ranging phase and the solve**

Add to `src/apos_gw.c`, above `apos_gw_step()`:

```c
/* RANGE phase cursor. `pair` walks every ORDERED pair in a fixed row-major
 * order: for from = 0..n-1, for to = 0..n-1, skipping from == to. A single
 * index rather than two counters, so the phase state is one number to reason
 * about and meas_done/meas_total are trivially derived. */
static uint16_t pair_idx;
static uint16_t pair_total;
static uint8_t pair_retries;
static bool awaiting_rsp;
static uint16_t await_from_addr;
static uint16_t await_peer_addr;
static bool accepted;

/* Decompose an ordered-pair index into (from, to), skipping the diagonal. */
static void pair_of(uint16_t idx, uint8_t n, uint8_t *from, uint8_t *to)
{
	uint16_t per_row = (uint16_t)(n - 1u);

	*from = (uint8_t)(idx / per_row);

	uint8_t col = (uint8_t)(idx % per_row);

	*to = (col >= *from) ? (uint8_t)(col + 1u) : col;
}

void apos_gw_set_zoff(float dz)
{
	zoff_m = dz;
}

const struct apos_result *apos_gw_result(void)
{
	return &res;
}

bool apos_gw_accepted(void)
{
	return accepted;
}

int apos_gw_start_run(void)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (!apos_gw_gauge_set()) {
		return -EINVAL;
	}

	apos_table_init(&tbl);
	memset(&res, 0, sizeof(res));
	have_result = false;
	accepted = false;
	session = new_session();
	enum_round = 0;
	next_action_ms = 0;
	pair_idx = 0;
	pair_total = 0;
	pair_retries = 0;
	awaiting_rsp = false;

	/* Starts in ENUM: a run always re-enumerates rather than trusting a
	 * table from an earlier `apos enum`, because a board may have been
	 * rebooted, re-addressed or swapped in between. The gauge survives that
	 * because it is stored as addresses, not indices. */
	phase = APOS_GW_ENUM;

	LOG_INF("{\"apos\":\"run\",\"session\":%u}", session);
	return 0;
}

/* Resolve the four gauge addresses to node indices in the freshly enumerated
 * table. Returns 0, or -ENOENT with the offending address logged. */
static int resolve_gauge(struct apos_gauge *g)
{
	uint8_t *out[4] = {&g->origin, &g->xaxis, &g->plane, &g->up};

	for (int k = 0; k < 4; k++) {
		int idx = apos_table_find_addr(&tbl, gauge_addr[k]);

		if (idx < 0) {
			LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did not "
				"answer enumeration\"}", gauge_addr[k]);
			return -ENOENT;
		}
		*out[k] = (uint8_t)idx;
	}
	return 0;
}

static void judge_result(void)
{
	bool planar = res.planarity_m * 1000.0f <
		      (float)APOS_ACCEPT_PLANARITY_MM;

	accepted = res.n_placed == res.n_nodes &&
		   res.n_ambiguous == 0u &&
		   res.rms_m * 1000.0f < (float)APOS_ACCEPT_RMS_MM &&
		   res.worst_edge_m <= APOS_ACCEPT_WORST_FACTOR * res.rms_m +
				       (float)APOS_ACCEPT_RMS_MM / 1000.0f;

	LOG_INF("{\"apos_solve\":{\"nodes\":%u,\"placed\":%u,\"ambiguous\":%u,"
		"\"rms_mm\":%d,\"worst_mm\":%d,\"worst_pair\":[%u,%u],"
		"\"planarity_mm\":%d,\"iters\":%u,\"accepted\":%u}}",
		res.n_nodes, res.n_placed, res.n_ambiguous,
		(int)(res.rms_m * 1000.0f), (int)(res.worst_edge_m * 1000.0f),
		res.worst_i, res.worst_j,
		(int)(res.planarity_m * 1000.0f), res.iterations,
		accepted ? 1u : 0u);

	for (uint8_t k = 0; k < res.n_nodes; k++) {
		LOG_INF("{\"apos_node\":{\"idx\":%u,\"addr\":\"0x%04X\","
			"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"state\":%u,"
			"\"resid_mm\":%d}}",
			k, tbl.peer[k].short_addr, (double)res.node[k].x,
			(double)res.node[k].y, (double)res.node[k].z,
			res.node[k].state,
			(int)(res.node[k].residual_m * 1000.0f));
	}

	if (planar && res.n_placed >= 4u) {
		LOG_WRN("{\"apos_warn\":\"array is near-coplanar (%d mm) — x and "
			"y are good but the solved z values are not "
			"survey-quality\"}",
			(int)(res.planarity_m * 1000.0f));
	}
	if (res.n_ambiguous) {
		LOG_WRN("{\"apos_warn\":\"%u node(s) reflection-ambiguous — each "
			"needs a fourth measured edge or a repositioned "
			"anchor\"}", res.n_ambiguous);
	}
}

static void do_solve(void)
{
	struct apos_gauge g;
	struct apos_edge edges[APOS_MAX_EDGES];

	if (resolve_gauge(&g) != 0) {
		phase = APOS_GW_IDLE;
		return;
	}

	uint16_t n_edges = apos_table_symmetrise(&tbl, edges, APOS_MAX_EDGES,
						(uint8_t)APOS_MIN_N_OK);
	uint16_t missing = apos_table_missing_pairs(&tbl,
						   (uint8_t)APOS_MIN_N_OK);

	/* Logged, never silent: a hole changes what the fit rests on, and
	 * "covered everything" is exactly the wrong impression to leave. */
	LOG_INF("{\"apos_edges\":{\"usable\":%u,\"missing_pairs\":%u}}",
		n_edges, missing);

	int rc = apos_geom_solve(edges, n_edges, tbl.n_peers, &g, &res);

	if (rc == -ENODATA) {
		LOG_ERR("{\"apos_error\":\"the gauge anchors are not mutually "
			"ranged — move them into line of sight of each other "
			"and re-run\"}");
		phase = APOS_GW_IDLE;
		return;
	}
	if (rc) {
		LOG_ERR("{\"apos_error\":\"solve failed (errno %d)\"}", rc);
		phase = APOS_GW_IDLE;
		return;
	}

	if (zoff_m != 0.0f) {
		apos_geom_zoff(&res, zoff_m);
	}

	have_result = true;
	judge_result();
	phase = APOS_GW_IDLE;
}

static void on_range_rsp(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0, peer = 0, sd = 0;
	int32_t mean = 0;
	uint8_t n_ok = 0;

	if (apos_frame_parse_range_rsp(buf, plen, &sess, &peer, &mean, &sd,
				       &n_ok) != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp) {
		return;
	}
	/* Both endpoints must match what we asked for. Without the peer check a
	 * late reply from the PREVIOUS pair would be recorded against this one,
	 * which is a wrong edge rather than a missing one -- far worse. */
	if (apos_frame_src(buf) != await_from_addr || peer != await_peer_addr) {
		return;
	}

	int from = apos_table_find_addr(&tbl, await_from_addr);
	int to = apos_table_find_addr(&tbl, await_peer_addr);

	if (from >= 0 && to >= 0 && n_ok >= APOS_MIN_N_OK) {
		apos_table_add_meas(&tbl, (uint8_t)from, (uint8_t)to, mean, sd,
				    n_ok);
	} else if (n_ok < APOS_MIN_N_OK) {
		LOG_WRN("{\"apos_thin\":{\"from\":\"0x%04X\",\"to\":\"0x%04X\","
			"\"n_ok\":%u}}", await_from_addr, await_peer_addr, n_ok);
	}

	awaiting_rsp = false;
	pair_retries = 0;
	pair_idx++;
	next_action_ms = 0; /* next pair on the very next step */
}

static void step_range(uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (awaiting_rsp) {
		if (now < next_action_ms) {
			return;
		}
		/* Timed out. */
		if (pair_retries < APOS_GW_RANGE_RETRIES) {
			pair_retries++;
			awaiting_rsp = false;
			LOG_WRN("{\"apos_retry\":{\"from\":\"0x%04X\","
				"\"to\":\"0x%04X\"}}", await_from_addr,
				await_peer_addr);
		} else {
			LOG_WRN("{\"apos_hole\":{\"from\":\"0x%04X\","
				"\"to\":\"0x%04X\"}}", await_from_addr,
				await_peer_addr);
			awaiting_rsp = false;
			pair_retries = 0;
			pair_idx++;
		}
		return;
	}

	if (pair_idx >= pair_total) {
		do_solve();
		return;
	}

	uint8_t from, to;

	pair_of(pair_idx, tbl.n_peers, &from, &to);
	await_from_addr = tbl.peer[from].short_addr;
	await_peer_addr = tbl.peer[to].short_addr;

	int n = apos_frame_range_cmd_build(tx_buf, sizeof(tx_buf),
					  UWB_ADDR_GATEWAY_RESERVED,
					  await_from_addr, session,
					  await_peer_addr,
					  (uint8_t)APOS_GW_N_EXCHANGES);

	if (n < 0) {
		LOG_ERR("RANGE_CMD build failed (%d)", n);
		phase = APOS_GW_IDLE;
		return;
	}
	if (!tx_now((uint16_t)n, seq)) {
		/* The command never left. Retry on the next step rather than
		 * counting it against this pair's retry budget. */
		return;
	}

	awaiting_rsp = true;
	next_action_ms = now + APOS_GW_RANGE_TIMEOUT_MS;
}
```

In `on_enum_rsp()`, nothing changes. In `step_enum()`, replace the completion branch so a full run advances into RANGE instead of stopping:

```c
	if (enum_round >= APOS_GW_ENUM_ROUNDS) {
		LOG_INF("{\"apos_enum_done\":{\"peers\":%u}}", tbl.n_peers);

		/* A run sets the gauge before starting; a bare `apos enum` does
		 * not touch it. That is what distinguishes the two here, rather
		 * than a second phase enum value. */
		if (!apos_gw_gauge_set()) {
			phase = APOS_GW_IDLE;
			return;
		}
		if (tbl.n_peers < APOS_MIN_NODES) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) answered; a 3D "
				"gauge needs at least %u\"}",
				tbl.n_peers, APOS_MIN_NODES);
			phase = APOS_GW_IDLE;
			return;
		}

		pair_total = (uint16_t)tbl.n_peers *
			     (uint16_t)(tbl.n_peers - 1u);
		pair_idx = 0;
		pair_retries = 0;
		awaiting_rsp = false;
		next_action_ms = 0;
		phase = APOS_GW_RANGE;
		LOG_INF("{\"apos\":\"ranging\",\"ordered_pairs\":%u}",
			pair_total);
		return;
	}
```

Add the `APOS_GW_RANGE` case to `apos_gw_step()`'s switch and the `APOS_SUB_RANGE_RSP` case to `apos_gw_on_rx()`'s switch:

```c
	case APOS_GW_RANGE:
		step_range(seq);
		break;
```

```c
	case APOS_SUB_RANGE_RSP:
		on_range_rsp(buf, plen);
		break;
```

Fill the two progress fields in `apos_gw_get_status()`:

```c
	out->meas_done = pair_idx;
	out->meas_total = pair_total;
```

`apos_gw_start_enum()` must now also clear the run cursor so a bare `apos enum` after an aborted run cannot inherit stale state — add `pair_idx = 0; pair_total = 0; pair_retries = 0; awaiting_rsp = false; accepted = false;` alongside its existing initialisation.

- [ ] **Step 3: Add `apos run` and `apos zoff` to the shell**

Add to `src/apos_shell.c`:

```c
static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	rc = apos_gw_start_run();
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc == -EINVAL) {
		shell_error(sh, "error: set the frame first — `apos gauge "
				"origin=<addr> xaxis=<addr> plane=<addr> "
				"up=<addr>` (run `apos enum` to list addresses)");
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: run refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "running — this takes a few seconds per anchor pair and "
			"reports as JSON when it finishes. NOTHING is persisted; "
			"run `apos apply` afterwards to commit.");
	return 0;
}

static int cmd_zoff(const struct shell *sh, size_t argc, char **argv)
{
	char *endptr;
	double v;

	ARG_UNUSED(argc);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	v = strtod(argv[1], &endptr);
	if (endptr == argv[1] || *endptr != '\0') {
		return -EINVAL;
	}

	apos_gw_set_zoff((float)v);
	shell_print(sh, "{\"apos_zoff_m\":%.3f} — takes effect on the next "
			"`apos run`", v);
	return 0;
}
```

Register them in `sub_apos`:

```c
	SHELL_CMD_ARG(run,   NULL,
		      "run — range every anchor pair, solve, and REPORT ONLY "
		      "(persists nothing)",
		      cmd_run, 1, 0),
	SHELL_CMD_ARG(zoff,  NULL,
		      "zoff <metres> — shift z so z=0 is the floor rather than "
		      "the plane through the gauge anchors",
		      cmd_zoff, 2, 0),
```

`strtod` needs `<stdlib.h>`, already included.

- [ ] **Step 4: Verify it builds**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: clean. Note the solver runs on the gateway's main stack — if the build warns about stack usage, raise `CONFIG_MAIN_STACK_SIZE` in `prj.conf` rather than shrinking `APOS_MAX_NODES`; the two 18×18 matrices in `apos_geom_refine()` are already `static` for this reason.

- [ ] **Step 5: Hardware check — a full run over three anchors**

Place three anchors at tape-measured spacing, at least one at a clearly different height so the array is not coplanar. All three must already be antenna-delay calibrated, or a survey error cannot be told from a delay error.

```
apos enum
apos gauge origin=0x0001 xaxis=0x0002 plane=0x0003 up=0x0004
apos run
```

Expected: `{"apos":"ranging","ordered_pairs":12}`, a run of `{"apos_range":...}` lines from the anchors, then `{"apos_edges":...}` and `{"apos_solve":...}`. Check the solved pairwise distances against the tape and confirm `rms_mm` is inside `APOS_ACCEPT_RMS_MM`. Confirm the beacon `counter` still increments on the sniffer throughout.

A four-anchor array is needed for the gauge (`origin`, `xaxis`, `plane`, `up` are four distinct nodes). With only three anchors available, `apos run` correctly refuses with `{"apos_error":"3 anchor(s) answered; a 3D gauge needs at least 4"}` — verify that message too, since it is the case an operator hits first.

- [ ] **Step 6: Commit**

```bash
git add src/apos_gw.h src/apos_gw.c src/apos_shell.c
git commit -m "feat(apos): pairwise ranging phase, sparse solve and reporting

Ranges every ORDERED pair, one commanded pair at a time, so exactly one
initiator exists at any instant and every other anchor is a plain responder.
A pair that fails twice becomes a reported hole rather than a stalled run --
the fit works around holes and the operator needs to see which pair failed.

apos run persists nothing, mirroring the cal ref / cal peer discipline."
```

---

### Task 12: `apos_gw` — the apply phase, `apos ref`, and `SURVEY_END`

**Files:**
- Modify: `src/apos_gw.h` (add `APOS_GW_APPLY_*` constants, `apos_gw_start_apply`)
- Modify: `src/apos_gw.c`
- Modify: `src/apos_shell.c` (add `apos apply`, `apos ref`)

**Interfaces:**
- Consumes: `apos_frame_setpos_build`, `apos_frame_parse_setpos_ack`, `apos_frame_survey_end_build`, `apos_store_save`, `apos_store_set_ref`.
- Produces: `APOS_GW_APPLY_TIMEOUT_MS` (1500), `APOS_GW_APPLY_RETRIES` (3), `int apos_gw_start_apply(bool force)`.

- [ ] **Step 1: Add the constants and declaration**

Add to `src/apos_gw.h`:

```c
/* A SETPOS_ACK comes straight back -- no ranging batch in between -- so this is
 * tight compared with APOS_GW_RANGE_TIMEOUT_MS. */
#define APOS_GW_APPLY_TIMEOUT_MS 1500u

/* Three attempts per anchor. Unlike a failed range, a failed SETPOS cannot be
 * shrugged off as a hole: an anchor left on its old coordinates while its peers
 * move to new ones is a silently inconsistent deployment, so this retries hard
 * and then reports the anchor by address. */
#define APOS_GW_APPLY_RETRIES 3u

/* Push the last solved result to every anchor, persist it locally, and close the
 * survey window.
 *
 * Refuses a result that failed acceptance unless force is true. Returns 0,
 * -EBUSY if a survey runs, -ENODATA if there is no result to apply, or
 * -EPERM if the result failed acceptance and force was not given. */
int apos_gw_start_apply(bool force);
```

- [ ] **Step 2: Implement the apply phase**

Add to `src/apos_gw.c`:

```c
/* APPLY phase cursor. */
static uint8_t apply_idx;
static uint8_t apply_retries;
static uint8_t applied_ok;
static uint8_t applied_fail;
static bool apply_ending;   /* all nodes done; SURVEY_END still to send */

int apos_gw_start_apply(bool force)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}
	if (!have_result) {
		return -ENODATA;
	}
	if (!accepted && !force) {
		return -EPERM;
	}

	apply_idx = 0;
	apply_retries = 0;
	applied_ok = 0;
	applied_fail = 0;
	apply_ending = false;
	awaiting_rsp = false;
	next_action_ms = 0;
	phase = APOS_GW_APPLY;

	LOG_INF("{\"apos\":\"applying\",\"nodes\":%u,\"forced\":%u}",
		res.n_nodes, force ? 1u : 0u);
	return 0;
}

/* Persist the survey locally, with the gauge origin first.
 *
 * node[0] must be the origin: apos_store.h documents that the geographic
 * reference belongs to it, and pos_json.c relies on the ordering. The solver's
 * node order is the order anchors answered enumeration in, which is arbitrary,
 * so the origin is moved to the front here rather than being assumed. */
static void persist_survey(void)
{
	struct apos_survey s;
	struct apos_gauge g;

	memset(&s, 0, sizeof(s));

	if (resolve_gauge(&g) != 0) {
		LOG_ERR("{\"apos_error\":\"cannot persist — gauge no longer "
			"resolves\"}");
		return;
	}

	uint8_t w = 0;

	/* Origin first, then everything else in solver order. */
	for (uint8_t pass = 0; pass < 2; pass++) {
		for (uint8_t k = 0; k < res.n_nodes && w < APOS_MAX_NODES; k++) {
			bool is_origin = (k == g.origin);

			if ((pass == 0) != is_origin) {
				continue;
			}
			if (res.node[k].state == APOS_NODE_UNPLACED) {
				continue;
			}
			memcpy(s.node[w].eui, tbl.peer[k].eui, APOS_EUI_LEN);
			s.node[w].short_addr = tbl.peer[k].short_addr;
			s.node[w].x = res.node[k].x;
			s.node[w].y = res.node[k].y;
			s.node[w].z = res.node[k].z;
			w++;
		}
	}

	s.n_nodes = w;
	s.valid = w > 0u;

	int rc = apos_store_save(&s);

	if (rc) {
		LOG_ERR("{\"apos_error\":\"survey applied to the anchors but NOT "
			"persisted on the gateway (errno %d) — the anchors "
			"topic will fall back to the stub after a reboot\"}",
			rc);
	} else {
		LOG_INF("{\"apos_store\":\"survey saved\",\"nodes\":%u}", w);
	}
}

static void on_setpos_ack(const uint8_t *buf, uint16_t plen)
{
	uint16_t sess = 0;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	bool ok = false;

	if (apos_frame_parse_setpos_ack(buf, plen, &sess, &x, &y, &z, &ok)
	    != 0) {
		return;
	}
	if (sess != session || !awaiting_rsp) {
		return;
	}
	if (apos_frame_src(buf) != await_from_addr) {
		return;
	}

	if (ok) {
		applied_ok++;
		LOG_INF("{\"apos_applied\":{\"addr\":\"0x%04X\",\"x\":%.3f,"
			"\"y\":%.3f,\"z\":%.3f}}", await_from_addr,
			(double)x, (double)y, (double)z);
	} else {
		/* The anchor applied it but could not persist it. Counted as a
		 * failure: it will silently revert on its next reboot, which is
		 * exactly the kind of half-applied state worth being loud
		 * about. */
		applied_fail++;
		LOG_ERR("{\"apos_error\":\"0x%04X applied but could not persist "
			"— it will revert on reboot\"}", await_from_addr);
	}

	awaiting_rsp = false;
	apply_retries = 0;
	apply_idx++;
	next_action_ms = 0;
}

static void step_apply(uint8_t *seq)
{
	int64_t now = k_uptime_get();

	if (awaiting_rsp) {
		if (now < next_action_ms) {
			return;
		}
		if (apply_retries < APOS_GW_APPLY_RETRIES) {
			apply_retries++;
			awaiting_rsp = false;
			LOG_WRN("{\"apos_retry\":{\"setpos\":\"0x%04X\","
				"\"attempt\":%u}}", await_from_addr,
				apply_retries);
		} else {
			applied_fail++;
			LOG_ERR("{\"apos_error\":\"0x%04X never acknowledged "
				"SETPOS after %u attempts — it is still on its "
				"OLD coordinates\"}", await_from_addr,
				APOS_GW_APPLY_RETRIES + 1u);
			awaiting_rsp = false;
			apply_retries = 0;
			apply_idx++;
		}
		return;
	}

	if (apply_ending) {
		int n = apos_frame_survey_end_build(tx_buf, sizeof(tx_buf),
						  UWB_ADDR_GATEWAY_RESERVED,
						  session);

		if (n > 0) {
			tx_now((uint16_t)n, seq);
		}

		LOG_INF("{\"apos_apply_done\":{\"ok\":%u,\"failed\":%u}}",
			applied_ok, applied_fail);
		if (applied_fail) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) did NOT take the "
				"new coordinates — the deployment is "
				"inconsistent. Re-run `apos apply`.\"}",
				applied_fail);
		}
		apply_ending = false;
		phase = APOS_GW_IDLE;
		return;
	}

	/* Skip unplaced nodes: there is nothing to push, and pushing (0, 0, 0)
	 * would be exactly the silent-wrong-coordinate failure this whole
	 * design exists to remove. */
	while (apply_idx < res.n_nodes &&
	       res.node[apply_idx].state == APOS_NODE_UNPLACED) {
		LOG_WRN("{\"apos_skip\":{\"addr\":\"0x%04X\",\"reason\":"
			"\"unplaced\"}}", tbl.peer[apply_idx].short_addr);
		apply_idx++;
	}

	if (apply_idx >= res.n_nodes) {
		/* Persist before SURVEY_END, so a gateway that dies between the
		 * two still has the survey on flash. */
		persist_survey();
		apply_ending = true;
		return;
	}

	await_from_addr = tbl.peer[apply_idx].short_addr;

	int n = apos_frame_setpos_build(tx_buf, sizeof(tx_buf),
				       UWB_ADDR_GATEWAY_RESERVED,
				       await_from_addr, session,
				       res.node[apply_idx].x,
				       res.node[apply_idx].y,
				       res.node[apply_idx].z);

	if (n < 0) {
		LOG_ERR("SETPOS build failed (%d)", n);
		phase = APOS_GW_IDLE;
		return;
	}
	if (!tx_now((uint16_t)n, seq)) {
		return;
	}

	awaiting_rsp = true;
	next_action_ms = now + APOS_GW_APPLY_TIMEOUT_MS;
}
```

Add the switch cases:

```c
	case APOS_GW_APPLY:
		step_apply(seq);
		break;
```

```c
	case APOS_SUB_SETPOS_ACK:
		on_setpos_ack(buf, plen);
		break;
```

And fill the two counters in `apos_gw_get_status()`:

```c
	out->applied_ok = applied_ok;
	out->applied_fail = applied_fail;
```

**Note on the survey window during apply.** `apos run` ends with `phase = APOS_GW_IDLE` but does *not* send `SURVEY_END`, so the anchors' windows stay open (refreshed to `APOS_NODE_REFRESH_S` = 60 s by the last `RANGE_CMD`). That is deliberate: `apos apply` reuses the same `session`, and the anchors must still accept it. An operator who waits longer than the refresh window before applying will see `SETPOS` refused for a session mismatch — re-run `apos run` in that case. Do not paper over it by making `SETPOS` session-agnostic: accepting a coordinate push from a stale session is how an abandoned run overwrites a good survey.

- [ ] **Step 3: Add `apos apply` and `apos ref` to the shell**

Add to `src/apos_shell.c`:

```c
static int cmd_apply(const struct shell *sh, size_t argc, char **argv)
{
	bool force = (argc > 1) && (strcmp(argv[1], "force") == 0);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}
	if (argc > 1 && !force) {
		shell_error(sh, "error: the only argument is `force`");
		return -EINVAL;
	}

	rc = apos_gw_start_apply(force);
	if (rc == -EBUSY) {
		shell_error(sh, "error: a survey is already running");
		return rc;
	}
	if (rc == -ENODATA) {
		shell_error(sh, "error: no result to apply — run `apos run` first");
		return rc;
	}
	if (rc == -EPERM) {
		const struct apos_result *r = apos_gw_result();

		shell_error(sh, "error: the last run FAILED acceptance "
				"(rms=%d mm, worst=%d mm on pair [%u,%u], "
				"placed=%u/%u, ambiguous=%u). Fix the geometry "
				"and re-run, or `apos apply force` to commit it "
				"anyway.",
			    (int)(r->rms_m * 1000.0f),
			    (int)(r->worst_edge_m * 1000.0f),
			    r->worst_i, r->worst_j, r->n_placed, r->n_nodes,
			    r->n_ambiguous);
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: apply refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "applying — each anchor is pushed its coordinates and "
			"must acknowledge; watch for apos_apply_done");
	return 0;
}

static int cmd_ref(const struct shell *sh, size_t argc, char **argv)
{
	char *e1, *e2;
	double lat, lon;

	ARG_UNUSED(argc);

	int rc = require_gateway(sh);

	if (rc) {
		return rc;
	}

	lat = strtod(argv[1], &e1);
	lon = strtod(argv[2], &e2);
	if (e1 == argv[1] || *e1 != '\0' || e2 == argv[2] || *e2 != '\0') {
		shell_error(sh, "error: lat and lon must be decimal degrees");
		return -EINVAL;
	}
	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
		shell_error(sh, "error: lat must be -90..90 and lon -180..180");
		return -EINVAL;
	}

	rc = apos_store_set_ref(lat, lon);
	if (rc) {
		shell_error(sh, "error: reference NOT persisted (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "{\"apos_ref\":{\"lat\":%.6f,\"lon\":%.6f}} — this is "
			"the origin anchor's real-world position; the platform "
			"places the whole survey against it",
		    lat, lon);
	return 0;
}
```

Register them:

```c
	SHELL_CMD_ARG(apply, NULL,
		      "apply [force] — push the last result to every anchor, "
		      "persist it, and close the survey",
		      cmd_apply, 1, 1),
	SHELL_CMD_ARG(ref,   NULL,
		      "ref <lat> <lon> — the origin anchor's real-world "
		      "position, for the platform map",
		      cmd_ref, 3, 0),
```

- [ ] **Step 4: Verify it builds**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: clean.

- [ ] **Step 5: Hardware check — apply and survive a reboot**

With four calibrated anchors and a completed `apos run` that passed acceptance:

```
apos apply
apos show
```

Expected: one `{"apos_applied":...}` per anchor, then `{"apos_apply_done":{"ok":4,"failed":0}}` and `{"apos_store":"survey saved","nodes":4}`.

On each anchor's console, `anchor show` must report the pushed coordinates with `pos_valid` set. Then `kernel reboot cold` on every board — gateway and anchors — and confirm `anchor show` and `apos show` still report the same coordinates. This is the gate that proves the survey is persisted on both sides, not just applied in RAM.

Also verify the refusal path, since it is the one an operator meets when the geometry is poor: deliberately mis-measure by moving one anchor between `apos run` and reading the result, confirm `apos apply` reports the acceptance failure with the offending pair named, and confirm `apos apply force` then proceeds.

- [ ] **Step 6: Commit**

```bash
git add src/apos_gw.h src/apos_gw.c src/apos_shell.c
git commit -m "feat(apos): apply phase, geographic reference and survey teardown

Retries a SETPOS hard and names any anchor that never acknowledges: unlike a
failed range, an anchor left on old coordinates while its peers move is a
silently inconsistent deployment. Unplaced nodes are skipped rather than
pushed (0,0,0), and the survey is persisted before SURVEY_END so a gateway
that dies between the two still has it on flash."
```

---

### Task 13: The anchors MQTT payload stops being a stub

**Files:**
- Modify: `src/pos_json.h`, `src/pos_json.c`
- Modify: `src/net_uplink.c:455`
- Modify: `tests/pos_json/test_pos_json.c`

**Interfaces:**
- Consumes: `struct apos_survey` from `apos_store.h` (pure header, verified Zephyr-free in Task 9), `apos_store_get()`.
- Produces: `int pos_json_anchors(char *buf, size_t len, const struct apos_survey *s)` — **signature change**, one extra parameter. `s == NULL` or `!s->valid` keeps the existing stub byte-for-byte.

- [ ] **Step 1: Change the declaration**

In `src/pos_json.h`, add `#include "apos_store.h"` and replace the declaration:

```c
/* Format the zone/anchor map for the retained anchors topic.
 *
 * With a valid survey, emits the surveyed geometry: one entry per surveyed
 * anchor, named ANC-<zone>-<NNN> from its short address, with s->node[0] as the
 * axis/reference anchor carrying s->ref_lat/ref_lon. Every other anchor is
 * local-only (latitude/longitude 0) and positioned relative to the reference in
 * metres, which is what the schema already meant.
 *
 * With s == NULL or !s->valid, emits the original four-anchor stub unchanged, so
 * a gateway that has never been surveyed still publishes a schema-valid document
 * rather than an empty one. The stub's coordinates are placeholders; its schema
 * is the contract.
 *
 * Same return contract as pos_json_fix(): bytes written excluding the NUL, or -1
 * if the buffer was too small. On -1 the caller MUST drop the message --
 * publishing truncated JSON is worse than publishing nothing. */
int pos_json_anchors(char *buf, size_t len, const struct apos_survey *s);
```

- [ ] **Step 2: Write the failing tests**

Add to `tests/pos_json/test_pos_json.c`:

```c
/* A gateway that was never surveyed must still publish a valid document, so the
 * stub is a fallback and not dead code. */
static void test_anchors_falls_back_to_the_stub(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = false;

    CHECK(pos_json_anchors(buf, sizeof(buf), NULL) > 0);
    CHECK(strstr(buf, "ANC-LOBBY-001") != NULL);

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
    CHECK(strstr(buf, "ANC-LOBBY-001") != NULL);
}

static void test_anchors_emits_the_surveyed_geometry(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 3;
    s.node[0].short_addr = 0x0001;
    s.node[0].x = 0.0f;  s.node[0].y = 0.0f;  s.node[0].z = 0.0f;
    s.node[1].short_addr = 0x0002;
    s.node[1].x = 3.25f; s.node[1].y = 0.0f;  s.node[1].z = 0.1f;
    s.node[2].short_addr = 0x0004;
    s.node[2].x = 1.5f;  s.node[2].y = 4.75f; s.node[2].z = 2.0f;
    s.ref_lat = 21.016042;
    s.ref_lon = -89.652129;
    s.ref_valid = true;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);

    /* Named from the short address, so the platform's anchor names track the
     * boards rather than a hand-kept list. */
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-001") != NULL);
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-002") != NULL);
    CHECK(strstr(buf, "ANC-" POS_JSON_ZONE_NAME "-004") != NULL);
    /* The stub must be entirely gone -- a document with both would draw twice. */
    CHECK(strstr(buf, "ANC-LOBBY-001") == NULL);
    /* Surveyed coordinates, including a real z. */
    CHECK(strstr(buf, "\"x\":3.25") != NULL);
    CHECK(strstr(buf, "\"y\":4.75") != NULL);
    CHECK(strstr(buf, "\"z\":2.00") != NULL);
    /* Exactly one axis/reference anchor, and it is node[0]. */
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
    CHECK(strstr(buf, "21.016042") != NULL);
}

/* Without `apos ref` there is no lat/long to publish. The document must still be
 * valid -- zeroes, exactly as the stub does for its non-reference anchors -- so
 * the platform draws the geometry even before the site is geo-referenced. */
static void test_anchors_without_a_geo_reference(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = 1;
    s.node[0].short_addr = 0x0001;
    s.ref_valid = false;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
    CHECK(strstr(buf, "\"latitude\":0") != NULL);
    CHECK(strstr(buf, "\"isReferenceAxis\":true") != NULL);
}

/* POS_JSON_MAX_LEN must still hold the largest real document, which is now a
 * full APOS_MAX_NODES survey rather than the four-anchor stub. */
static void test_full_survey_fits_the_buffer(void)
{
    char buf[POS_JSON_MAX_LEN];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = APOS_MAX_NODES;
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        s.node[k].short_addr = (uint16_t)(0x0001 + k);
        /* Widest plausible values, so the check is on the real worst case. */
        s.node[k].x = -123.456f;
        s.node[k].y = -123.456f;
        s.node[k].z = -123.456f;
    }
    s.ref_lat = -89.123456;
    s.ref_lon = -179.123456;
    s.ref_valid = true;

    CHECK(pos_json_anchors(buf, sizeof(buf), &s) > 0);
}

/* A survey too large for the buffer must be REFUSED, not truncated: half a JSON
 * document published retained would poison the topic until the next connect. */
static void test_anchors_refuses_a_short_buffer(void)
{
    char small[64];
    struct apos_survey s;

    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.n_nodes = APOS_MAX_NODES;
    for (uint8_t k = 0; k < APOS_MAX_NODES; k++) {
        s.node[k].short_addr = (uint16_t)(0x0001 + k);
    }

    CHECK(pos_json_anchors(small, sizeof(small), &s) == -1);
}
```

Update the three existing calls at `tests/pos_json/test_pos_json.c:94`, `:128` and `:135` to pass `NULL` as the third argument, and register the five new tests in `main()`. Add `#include <string.h>` if it is not already there.

- [ ] **Step 3: Run to verify it fails**

```powershell
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c -lm
```

Expected: FAIL — too many arguments to `pos_json_anchors`.

- [ ] **Step 4: Implement**

In `src/pos_json.c`, rename the existing function body to a static helper and add the survey path:

```c
/* The original four-anchor placeholder. Kept as the fallback for a gateway that
 * has never been surveyed: a schema-valid document with placeholder numbers is
 * strictly better than no document, because a downstream consumer written
 * against this stays valid either way. */
static int anchors_stub(char *buf, size_t len)
{
	static const char doc[] =
		"{\"name\":\"" POS_JSON_ZONE_NAME "\",\"anchors\":["
		"{\"name\":\"ANC-LOBBY-001\",\"isAxis\":true,\"isReferenceAxis\":true,"
		"\"latitude\":21.01604164655441,\"longitude\":-89.6521292940793,"
		"\"x\":0.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-002\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":2.0,\"y\":0.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-003\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":2.0,\"y\":2.0,\"z\":0.0},"
		"{\"name\":\"ANC-LOBBY-004\",\"isAxis\":false,\"isReferenceAxis\":false,"
		"\"latitude\":0,\"longitude\":0,\"x\":0.0,\"y\":2.0,\"z\":0.0}"
		"]}";
	int n = snprintf(buf, len, "%s", doc);

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	return n;
}

int pos_json_anchors(char *buf, size_t len, const struct apos_survey *s)
{
	if (!buf || len == 0) {
		return -1;
	}
	if (!s || !s->valid || s->n_nodes == 0) {
		return anchors_stub(buf, len);
	}

	/* Accumulated with a running offset rather than one giant snprintf: the
	 * node count is variable. Every append is bounds-checked, and any
	 * overflow returns -1 for the whole document -- a truncated retained
	 * publish would poison the topic until the next connect. */
	size_t off = 0;
	int n = snprintf(buf, len, "{\"name\":\"" POS_JSON_ZONE_NAME
				   "\",\"anchors\":[");

	if (n < 0 || (size_t)n >= len) {
		return -1;
	}
	off = (size_t)n;

	for (uint8_t k = 0; k < s->n_nodes; k++) {
		bool is_ref = (k == 0);

		/* node[0] is the gauge origin by construction (apos_store.h),
		 * so it is the axis anchor and the only one carrying a real
		 * lat/long. Everything else is local-only, in metres relative
		 * to it -- which is what the schema already meant. */
		n = snprintf(buf + off, len - off,
			     "%s{\"name\":\"ANC-" POS_JSON_ZONE_NAME "-%03u\","
			     "\"isAxis\":%s,\"isReferenceAxis\":%s,"
			     "\"latitude\":%.8f,\"longitude\":%.8f,"
			     "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
			     (k == 0) ? "" : ",",
			     (unsigned int)s->node[k].short_addr,
			     is_ref ? "true" : "false",
			     is_ref ? "true" : "false",
			     (is_ref && s->ref_valid) ? s->ref_lat : 0.0,
			     (is_ref && s->ref_valid) ? s->ref_lon : 0.0,
			     (double)s->node[k].x, (double)s->node[k].y,
			     (double)s->node[k].z);

		if (n < 0 || (size_t)n >= len - off) {
			return -1;
		}
		off += (size_t)n;
	}

	n = snprintf(buf + off, len - off, "]}");
	if (n < 0 || (size_t)n >= len - off) {
		return -1;
	}
	return (int)(off + (size_t)n);
}
```

`bool` needs `<stdbool.h>` and `uint8_t` needs `<stdint.h>` — both arrive via `apos_store.h` through `pos_json.h`, but add them explicitly to `pos_json.c` rather than relying on a transitive include.

**Check `POS_JSON_MAX_LEN`.** An eight-anchor document is roughly `8 × 150 + 40 ≈ 1240` bytes at the widest values, which exceeds the current 640. Raise it in `pos_json.h` and say why:

```c
/* Buffer size that fits either document plus its NUL. Sized on the LARGER of the
 * two: a full APOS_MAX_NODES surveyed document (~150 bytes per anchor at the
 * widest coordinate and lat/long values), not the four-anchor stub it used to be
 * sized on. tests/pos_json/ asserts the worst case still fits. */
#define POS_JSON_MAX_LEN 1536
```

`net_uplink.c`'s `payload_buf` is sized from this macro; confirm with `git grep -n POS_JSON_MAX_LEN` that every buffer derives from it rather than from a literal.

- [ ] **Step 5: Update the caller**

At `src/net_uplink.c:455`:

```c
	int n = pos_json_anchors(payload_buf, sizeof(payload_buf),
				 apos_store_get());
```

Add `#include "apos_store.h"` to `src/net_uplink.c`.

- [ ] **Step 6: Run the tests and build**

```powershell
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c -lm
./tests/pos_json/test_pos_json.exe
```

Expected: `PASSED`, exit 0.

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: clean.

- [ ] **Step 7: Hardware check — the platform draws the deployment**

With a surveyed and applied gateway, reboot it and watch the broker on `uwb/anchor/setup/852541`. The retained payload must carry `ANC-852541-001..` with the surveyed coordinates, not `ANC-LOBBY-001..`. Confirm on the platform that the map now shows the real geometry.

- [ ] **Step 8: Commit**

```bash
git add src/pos_json.h src/pos_json.c src/net_uplink.c tests/pos_json/test_pos_json.c
git commit -m "feat(apos): publish the surveyed geometry on the anchors topic

The stub placed four anchors at the corners of a 2 m square, so the platform
drew stub geometry while the tag solved against whatever was hand-entered --
map and solver disagreed by construction. Now driven by the stored survey,
with the stub kept as the never-surveyed fallback so the document is always
schema-valid. POS_JSON_MAX_LEN resized for the eight-anchor worst case."
```

---

### Task 14: The MQTT trigger stub

**Files:**
- Modify: `src/pos_json.h` (reserve the topic), `src/apos_gw.h`, `src/apos_gw.c`

**Interfaces:**
- Consumes: `apos_gw_start_run()`, `apos_gw_start_apply()`.
- Produces: `POS_JSON_TOPIC_SURVEY_FMT`, `int apos_gw_trigger_from_mqtt(const char *payload, size_t len)`.

- [ ] **Step 1: Reserve the topic**

`net_uplink.c` has no subscribe path at all — only `MQTT_EVT_CONNACK`, `MQTT_EVT_DISCONNECT` and `MQTT_EVT_PUBACK`. So this task deliberately ships a parser and no transport.

Add to `src/pos_json.h`, beside `POS_JSON_ZONE_NAME`:

```c
/* Topic the survey trigger will arrive on. Declared here with the other topics,
 * composed from POS_JSON_ZONE_NAME, so a topic can never disagree with the zone
 * in its payload -- the same rule the position and anchors topics follow.
 *
 * NOT SUBSCRIBED YET: net_uplink.c has no subscribe path. Reserved so the name
 * is settled and visible next to its siblings rather than being invented later
 * in whichever file happens to add the subscription. */
#define POS_JSON_TOPIC_SURVEY "uwb/anchor/survey/" POS_JSON_ZONE_NAME
```

- [ ] **Step 2: Declare and implement the trigger**

Add to `src/apos_gw.h`:

```c
/* Parse and act on a survey trigger document.
 *
 * UNWIRED: net_uplink.c does not subscribe to anything, so nothing calls this
 * yet. It exists complete and tested-by-inspection so that wiring a subscribe
 * path later is a transport change only, with no protocol decisions left open.
 *
 * Accepts a minimal document, matched by substring rather than parsed with a
 * JSON library -- this project links no JSON parser and one command word does
 * not justify adding one:
 *   {"cmd":"run"}     -> apos_gw_start_run()
 *   {"cmd":"apply"}   -> apos_gw_start_apply(false)
 *
 * Deliberately NOT accepted: "apply force" and any form of setting the gauge.
 * The gauge is a physical claim about which box is where and a forced apply
 * overrides a failed acceptance check -- neither should be assertable by a
 * broker message. Both stay console-only.
 *
 * Returns 0, -EINVAL on an unrecognised document, or whatever the underlying
 * start function returned. */
int apos_gw_trigger_from_mqtt(const char *payload, size_t len);
```

Add to `src/apos_gw.c`:

```c
int apos_gw_trigger_from_mqtt(const char *payload, size_t len)
{
	/* Bounded copy so strstr() cannot run off a payload the broker did not
	 * NUL-terminate. 64 bytes is far more than the accepted documents need
	 * and anything longer is not one of them. */
	char tmp[64];

	if (!payload || len == 0) {
		return -EINVAL;
	}
	if (len >= sizeof(tmp)) {
		LOG_WRN("survey trigger payload too long (%u) — ignored",
			(unsigned int)len);
		return -EINVAL;
	}
	memcpy(tmp, payload, len);
	tmp[len] = '\0';

	if (strstr(tmp, "\"run\"") != NULL) {
		LOG_INF("{\"apos\":\"triggered by MQTT\",\"cmd\":\"run\"}");
		return apos_gw_start_run();
	}
	if (strstr(tmp, "\"apply\"") != NULL) {
		LOG_INF("{\"apos\":\"triggered by MQTT\",\"cmd\":\"apply\"}");
		return apos_gw_start_apply(false);
	}

	LOG_WRN("unrecognised survey trigger: %s", tmp);
	return -EINVAL;
}
```

- [ ] **Step 3: Verify it builds, and that the compiler does not drop it**

```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```

Expected: clean, with **no** `defined but not used` warning — the function is non-static and declared in the header, so it links even with no caller. If a warning does appear, do not silence it with an attribute: it would mean the declaration and definition disagree.

- [ ] **Step 4: Commit**

```bash
git add src/pos_json.h src/apos_gw.h src/apos_gw.c
git commit -m "feat(apos): MQTT survey trigger stub, unwired

net_uplink.c has no subscribe path, so this ships a parser and no transport.
Reserving the topic beside its siblings keeps it composed from
POS_JSON_ZONE_NAME. Gauge-setting and forced apply stay console-only: the
gauge is a physical claim about which box is where, and a forced apply
overrides a failed acceptance check."
```

---

### Task 15: Operator documentation and `CLAUDE.md`

**Files:**
- Create: `docs/anchor-auto-positioning.md`
- Modify: `CLAUDE.md`

- [ ] **Step 1: Write the operator procedure**

Create `docs/anchor-auto-positioning.md`, in the style of `docs/antenna-delay-calibration.md`. It must contain, in this order:

1. **Prerequisites.** Antenna-delay calibration completed on **every** anchor first — an uncalibrated array cannot distinguish a survey error from a delay error, and the acceptance thresholds are derived from the calibrated per-pair figure. At least four anchors, since the 3D gauge needs four distinct nodes. One gateway in `anchor mode gateway` with its own position set (it is not a survey node — see §2).
2. **The gateway is not surveyed.** It never answers a ranging poll, so nothing can measure a distance to it. Its coordinate stays hand-entered with `anchor pos`. A 1 + 4 deployment surveys four nodes.
3. **Physical setup.** Mount every anchor in its final position. At least one at a clearly different height, or the array is coplanar and the solved z values are not survey-quality (`planarity_mm` in the report says so). Note which physical box is which, and pick the four gauge roles against a site sketch: `origin` at the corner you want as (0,0,0), `xaxis` along the direction you want as +x, `plane` on the +y side, `up` above the plane through the other three.
4. **Walkthrough**, with the exact console session and the expected output at each step:
   ```
   apos enum
   apos gauge origin=0x0001 xaxis=0x0002 plane=0x0003 up=0x0004
   apos zoff 2.4
   apos run
   apos apply
   apos ref 21.016042 -89.652129
   apos show
   kernel reboot cold
   ```
5. **Reading the report.** What `rms_mm`, `worst_pair`, `planarity_mm`, `resid_mm`, `state` and `missing_pairs` each mean, and which of them block `apos apply`. Include the thresholds by name (`APOS_ACCEPT_RMS_MM` = 50 mm, worst edge ≤ 3× RMS + 50 mm, all nodes placed, no ambiguous node) and note that these are the numbers most likely to need adjusting after the first real bench session.
6. **Troubleshooting**, one heading per failure the code actually emits: `duplicate short address` (two boards share an `anchor id`), `gauge address … did not answer enumeration`, `the gauge anchors are not mutually ranged`, `apos_hole` (a pair that cannot reach — move an anchor or accept the hole), `apos_thin` (fewer than `APOS_MIN_N_OK` good exchanges), `reflection-ambiguous` (needs a fourth edge), `near-coplanar` (x and y good, z not), `never acknowledged SETPOS`, and `applied but could not persist`.
7. **The `position_valid` change.** An anchor with no position now answers nothing at all — this is deliberate and replaces the old silent `(0, 0)`. A freshly reset board is silent until it is surveyed or given `anchor pos`.
8. **Acceptance gates**, copied from the design doc §13, as a checklist an operator ticks off.

- [ ] **Step 2: Update `CLAUDE.md`**

Make these edits:

- **Status section:** the second URGENT item ("Anchor auto-positioning") becomes implemented-but-not-hardware-verified, in the same shape the antenna-delay entry took: what exists, what has not been run, and the hardware gates in the order a human hits them. Point at `docs/anchor-auto-positioning.md`.
- **Remove the now-false claims** in that section: `position_valid` no longer "gates only the GATEWAY's beaconing" (Task 8), and the anchors payload is no longer "still the stub" (Task 13). Both were the stated motivation for the work; leaving them would send the next reader to fix something already fixed.
- **Also correct** the sentence proposing "DS-TWR between anchors only" as the approach to evaluate — record that it was evaluated and deferred in favour of the already-calibrated SS-TWR path, with a pointer to the design doc §4, so the decision is not silently re-litigated.
- **Console section:** add the `apos` tree (`enum`, `gauge`, `run`, `apply`, `ref`, `zoff`, `show`), noting it is GATEWAY-only and that `run` persists nothing.
- **Layout section:** one bullet per new file, matching the existing style — `apos_frame`, `apos_geom`, `apos_table`, `apos_store`, `apos_gw`, `apos_node`, `apos_shell`, and the `cal_initiator` → `ss_initiator` rename with the note that it is now in both images.
- **Host tests section:** the three new gcc lines, and the `-lm` requirement.
- **Hard-won facts:** add these three, each stating the consequence rather than just the rule:
  - **An unpositioned anchor is now silent, not `(0, 0)`.** `anchor_respond_wave_poll()` refuses unless `position_valid` or a survey window is open. A freshly `anchor reset` board answers nothing until surveyed or given `anchor pos`, which looks like a dead board if you do not know this. The survey window is why the gate could exist at all: a cold deployment's anchors are all unpositioned yet must answer each other.
  - **`ss_initiator.c` is compiled into the PRODUCTION image.** The old "a deployed anchor must never initiate a poll" safety property no longer comes from the build set; it comes from `apos_node.c`'s two gates (a gateway-opened survey window plus a session-matched `RANGE_CMD` from `0x0000`) and a hard `APOS_MAX_EXCHANGES` cap. Removing either gate re-creates the collision hazard the cal image was kept separate to avoid.
  - **`APOS_MAX_NODES` (8) is not `UWB_MAX_ANCHORS` (4), deliberately.** The survey handles eight nodes; the tag-facing ranging MAC still caps at four. Growing the deployment past four ranging anchors needs `disc_schedule.h`'s stagger and `anchor_respond.c`'s `TX_COMPLETE_TIMEOUT_MS` re-derived (18 ms is sized for `disc_resp_delay_uus(3)` = 12500 uus; id 5 would need 19.5 ms and would silently lose every DISCOVERY response) plus the tag's `UWB_FRAME_MAX_ANCHORS`. Surveying eight anchors does not make eight of them rangeable.
  - **The gateway's `apos_gw_step()` may emit at most one frame per call.** It runs on the `K_PRIO_COOP(0)` loop where the beacon is armed, so a step that transmits twice, or blocks, delays the beacon for the whole network. This is the same class of hazard as the unbounded TXFRS wait that froze the console.

- [ ] **Step 3: Verify the docs match the code**

```powershell
git grep -n "still the stub" CLAUDE.md
git grep -n "gates only the GATEWAY" CLAUDE.md
```

Expected: no matches — both claims were removed.

Re-run every host test and both builds one final time:

```powershell
gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm
gcc -Wall -Wextra -Isrc -o tests/apos_table/test_apos_table.exe tests/apos_table/test_apos_table.c src/apos_table.c src/apos_geom.c -lm
gcc -Wall -Wextra -Isrc -o tests/apos_frame/test_apos_frame.exe tests/apos_frame/test_apos_frame.c src/apos_frame.c src/apos_table.c src/apos_geom.c -lm
gcc -Wall -Wextra -Isrc -o tests/pos_json/test_pos_json.exe tests/pos_json/test_pos_json.c src/pos_json.c -lm
./tests/apos_geom/test_apos_geom.exe
./tests/apos_table/test_apos_table.exe
./tests/apos_frame/test_apos_frame.exe
./tests/pos_json/test_pos_json.exe
```

Expected: four `PASSED`. Then re-run the pre-existing suites (`uwb_config`, `uwb_frame`, `disc_schedule`, `gw_core`, `beacon_guard`, `net_config`, `cal_solve`) exactly as `CLAUDE.md` lists them, and both `west build` invocations.

- [ ] **Step 4: Commit**

```bash
git add docs/anchor-auto-positioning.md CLAUDE.md
git commit -m "docs: anchor auto-positioning procedure and CLAUDE.md updates

Records that DS-TWR between anchors was evaluated and deferred rather than
skipped, so the decision is not silently re-litigated. Removes the two claims
this branch made false: position_valid no longer gates only the gateway's
beaconing, and the anchors payload is no longer the stub."
```

---

## Self-Review

Run against `docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md`.

**Spec coverage:**

| Spec section | Task |
|---|---|
| 2.1 Gateway orchestrates and solves | 10, 11, 12 |
| 2.2 EUI-64 identity | 3, 6 |
| 3 Coordinate frame, four designations | 1 (seed), 2 (gauge preserved by construction), 10 (`apos gauge`) |
| 3 `apos zoff` | 1 (`apos_geom_zoff`), 11 (`apos zoff`) |
| 4 SS-TWR both directions averaged | 5, 7, 11 (ordered pairs), 3 (symmetrisation) |
| 5 Module layout | 1, 3, 4, 6, 9, 10 |
| 5.1 Step-machine integration | 10 |
| 6 Seven subtypes | 4 |
| 6.1 EUI-derived enumeration stagger | 6 |
| 6.2 One peer per RANGE_CMD | 7, 11 |
| 6.2.1 Gateway not a survey node | 15 (documented); enforced by it never answering a poll |
| 6.3 Two TX gates | 7 |
| 6.4 Serialisation | 11 (`awaiting_rsp` — one outstanding command) |
| 6.5 `position_valid` hazard | 8 |
| 7 Sparse LM solver + diagnostics | 1, 2 |
| 7.1 `APOS_MAX_NODES` independent | 1, 15 |
| 8 Operator flow | 10, 11, 12 |
| 9 Persistence, both sides | 6 (anchor), 9 + 12 (gateway) |
| 10.1 Anchors payload | 13 |
| 10.2 MQTT trigger stub | 14 |
| 11 Tag unchanged | no task touches `anchor_respond`'s VEWA layout or the tag |
| 12 Host tests | 1, 2, 3, 4, 13 |
| 13 Hardware gates | 8, 10, 11, 12, 13 step-level checks; 15 collects them |

No gaps.

**Type consistency, checked across tasks:**
- `apos_geom_solve/_seed/_refine/_zoff/_gauge_valid` — Tasks 1, 2, 11 agree.
- `struct apos_result` field names (`rms_m`, `worst_edge_m`, `worst_i/j`, `planarity_m`, `n_placed`, `n_ambiguous`, `iterations`, `node[].state`, `node[].residual_m`) — used identically in Tasks 2, 11, 12.
- `apos_table_add_peer` returning an index and `-EADDRINUSE` — Tasks 3 and 10 agree.
- `apos_table_symmetrise(t, out, cap, min_n_ok)` — Tasks 3 and 11 agree; `min_n_ok` is `APOS_MIN_N_OK` from `apos_node.h` in both the anchor's reporting (Task 7) and the gateway's filtering (Task 11).
- `anchor_respond_wave_poll()` seven-argument form — Task 8 updates both call sites.
- `pos_json_anchors()` three-argument form — Task 13 updates the one production call site and three test call sites.
- `apos_frame_*` builder/parser signatures — Task 4 defines them; Tasks 6, 7, 10, 11, 12 use exactly those names and argument orders.
- `tx_now()` exists separately in `apos_node.c` (beacon-guarded, takes a buffer) and `apos_gw.c` (not guarded, uses the module's `tx_buf`). Different signatures in different translation units, both `static` — intentional, and the asymmetry is explained in Task 10's comment.

**Placeholder scan:** no `TBD`, no `TODO`, no "add appropriate error handling", no "similar to Task N". Every code step carries real code. Task 15 is prose-only by nature (it produces documentation), and its content is enumerated point by point rather than left as "write the docs".

**Two things flagged for the implementer, not defects in the plan:**
1. Task 6 Step 1 and Task 10 Step 4 each contain a **stop-and-report** condition (`CONFIG_HWINFO` unavailable; no entropy source). Both would otherwise be resolved by a substitution that quietly undermines the design — an id-derived EUI, or a predictable session. Neither should be worked around without the human.
2. `APOS_ACCEPT_RMS_MM` (50), `APOS_ACCEPT_WORST_FACTOR` (3.0) and `APOS_ACCEPT_PLANARITY_MM` (100) are estimates anchored on the antenna-delay cross-check's 30 mm per-pair figure. They gate `apos apply` only and never change what was measured, so they are safe to tune after the first bench session.

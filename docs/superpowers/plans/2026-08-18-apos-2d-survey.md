# Anchor Survey 2D Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 2D survey mode (3-anchor gauge, `2N-3` degrees of freedom) alongside the existing 3D mode (4-anchor gauge, `3N-6`), reusing the existing Levenberg-Marquardt solver core unchanged, and switch the `apos gauge` console command to take anchor ids instead of short addresses.

**Architecture:** A `dim` field (`APOS_GEOM_2D` / `APOS_GEOM_3D`) travels on `struct apos_gauge` and `struct apos_result`. The LM refinement loop (`cost()`, the Jacobian assembly, `apply_step()`, `solve_dense()`) is untouched — it already treats "no free parameter for this coordinate" as "fixed at its seeded value" for the existing gauge nodes, so 2D mode is simply "no node ever gets a `z` free parameter," decided in one place (`pmap_build()`). New code is confined to: skipping the 3D-only `up` placement in `apos_geom_seed()`, a new small `trilaterate2()` (2-circle intersection) for placing ordinary nodes in 2D, a `-1`-sentinel gauge address at the orchestration layer, and an id-based `apos gauge` command.

**Tech Stack:** Zephyr RTOS (nRF/ESP32-S3 port), C11, pure-C host-tested solver module (gcc, no Zephyr), Zephyr shell subsystem, Zephyr settings subsystem for persistence.

**Spec:** `docs/superpowers/specs/2026-08-18-apos-2d-survey-design.md`

## Global Constraints

- `APOS_GEOM_3D` must be enum value `0` (not `APOS_GEOM_2D`) — every pre-existing `struct apos_gauge` literal predates the `dim` field and zero-initializes; `0` must mean today's actual (3D) behavior. See spec §3.1.
- The gateway's gauge storage (`apos_gw.c`'s `gauge_addr[4]`) uses `-1` as the "no `up`, 2D mode" sentinel, not `0` — `0x0000` is `UWB_ADDR_GATEWAY_RESERVED` and must stay rejected as an address on its own terms. See spec §4.1.
- `apos gauge` takes anchor ids (`0..UWB_MAX_ANCHORS-1`, the same space `anchor id` uses), not short addresses. Conversion to address (`UWB_ANCHOR_ADDR_BASE + id`) happens only inside `apos_shell.c`; nothing below `apos_gw_set_gauge()` ever sees an id. See spec §5.1.
- No changes to `apos_frame.c` (wire format) or `apos_table.c` (peer bookkeeping) — both are dimension-agnostic by nature. See spec §7.
- `apos_store.c` persists a deliberately separate, narrower `struct stored_survey`, not `struct apos_survey` itself — a new field on the public struct does not persist without also being added to `stored_survey` and threaded through `apos_settings_set()`/`apos_store_save()` explicitly. See spec §6.
- Host tests build with: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm`, run with `./tests/apos_geom/test_apos_geom.exe`, expect `PASSED` and exit 0.

---

## Task 1: `apos_geom.h` data model, and `apos_geom_free_params()`

**Files:**
- Modify: `src/apos_geom.h`
- Modify: `src/apos_geom.c`
- Test: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Produces: `enum apos_geom_dim { APOS_GEOM_3D = 0, APOS_GEOM_2D = 1 };`, `struct apos_gauge.dim`, `struct apos_result.dim`, `#define APOS_MIN_NODES_2D 3`, `#define APOS_MIN_NODES_3D 4`, `int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed);`

- [ ] **Step 1: Write the failing test for `apos_geom_free_params()`**

Add to `tests/apos_geom/test_apos_geom.c`, just above `int main(void)`:

```c
static void test_free_params_matches_each_dimensionality(void)
{
    /* 3D: 3N-6. 2D: 2N-3. */
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 4) == 6);
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 5) == 9);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 3) == 3);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 4) == 5);
}
```

Add `test_free_params_matches_each_dimensionality();` to the call list in `main()`, right after `test_gauge_requires_four_distinct_nodes();`.

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm`
Expected: compile error — `apos_geom_free_params` and `APOS_GEOM_3D`/`APOS_GEOM_2D` are not declared.

- [ ] **Step 3: Add the data model to `apos_geom.h`**

Replace this block in `src/apos_geom.h`:

```c
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
```

with:

```c
/* APOS_GEOM_3D is deliberately the zero value, not APOS_GEOM_2D: every
 * struct apos_gauge literal that predates this field (in apos_gw.c and in
 * tests/apos_geom/test_apos_geom.c) does not mention .dim and therefore
 * zero-initializes it. Zero must mean "today's existing 3D behaviour", or
 * every one of those untouched call sites would silently become a 2D solve
 * the moment this field exists. */
enum apos_geom_dim {
	APOS_GEOM_3D = 0,
	APOS_GEOM_2D = 1,
};

/* A 3D gauge needs four distinct nodes (origin/xaxis/plane/up); a 2D gauge
 * needs three (origin/xaxis/plane, no up -- there is no reflection to
 * resolve in a plane). */
#define APOS_MIN_NODES_3D 4
#define APOS_MIN_NODES_2D 3

struct apos_edge {
	uint8_t i;    /* node index, < n_nodes */
	uint8_t j;    /* node index, < n_nodes, != i */
	float   d_m;  /* symmetrised distance, metres */
	float   sd_m; /* standard deviation, metres; must be > 0 */
};

/* Operator designations, as node indices into the same array the edges
 * index. origin/xaxis/plane must always be distinct. up is a fourth,
 * additionally distinct designation used only when dim == APOS_GEOM_3D --
 * apos_geom_gauge_valid() does not read it at all in 2D mode, so its value
 * is a "don't care" there, not a sentinel that needs separate validation. */
struct apos_gauge {
	uint8_t origin; /* -> (0, 0, 0) */
	uint8_t xaxis;  /* -> (d, 0, 0), d > 0 */
	uint8_t plane;  /* -> (x, y, 0), y > 0 in 3D; z is always 0 in 2D */
	uint8_t up;     /* -> z > 0; ignored when dim == APOS_GEOM_2D */
	enum apos_geom_dim dim;
};
```

Then add `enum apos_geom_dim dim;` as a new member of `struct apos_result`, right after `uint8_t n_ambiguous;`:

```c
struct apos_result {
	struct apos_node_out node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	uint8_t  n_placed;
	uint8_t  n_ambiguous;
	enum apos_geom_dim dim;
	float    rms_m;        /* RMS residual over all usable edges */
	...
```

Finally, add this declaration right after the `apos_geom_zoff()` declaration:

```c
/* Free parameters for a solve of this dimensionality: translation +
 * rotation only, the gauge having already fixed the rest. 2N-3 in 2D (2
 * translation + 1 rotation), 3N-6 in 3D (3 translation + 3 rotation).
 * Centralizes the formula apos_gw.c otherwise duplicates as a literal
 * expression. */
int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed);
```

- [ ] **Step 4: Implement `apos_geom_free_params()` in `apos_geom.c`**

Add near the top of `src/apos_geom.c`, right after the `#define EPS` block:

```c
int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed)
{
	return (dim == APOS_GEOM_2D) ? (2 * (int)n_placed - 3)
				      : (3 * (int)n_placed - 6);
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm`
Then: `./tests/apos_geom/test_apos_geom.exe`
Expected: `PASSED`, exit 0. (Every pre-existing test must still pass unchanged — this step is also the regression check that `g_ref`'s missing `.dim` really did default to `APOS_GEOM_3D`.)

- [ ] **Step 6: Fix two stale comment references to the old `APOS_MIN_NODES` name**

`APOS_MIN_NODES` no longer exists as a single symbol (split into `APOS_MIN_NODES_2D`/`APOS_MIN_NODES_3D` above), but two prose comments still name it — not a compile break (they're comments, not code), but stale. In `src/pos_json.c`, change:

```c
	 * APOS_MIN_NODES (4) is isostatic (6 edges == 3N-6 free parameters), so
```

to:

```c
	 * APOS_MIN_NODES_3D (4) is isostatic (6 edges == 3N-6 free parameters), so
```

In `tests/pos_json/test_pos_json.c`, change:

```c
/* Node counts APOS_MIN_NODES-1 (1, 3, 8) are exercised elsewhere; fill in the
```

to:

```c
/* Node counts APOS_MIN_NODES_3D-1 (1, 3, 8) are exercised elsewhere; fill in the
```

and change:

```c
 * is not hypothetical: at APOS_MIN_NODES the accept criterion is isostatic
```

to:

```c
 * is not hypothetical: at APOS_MIN_NODES_3D the accept criterion is isostatic
```

- [ ] **Step 7: Commit**

```bash
git add src/apos_geom.h src/apos_geom.c tests/apos_geom/test_apos_geom.c src/pos_json.c tests/pos_json/test_pos_json.c
git commit -m "feat(apos): add 2D/3D dim to the gauge and result, plus apos_geom_free_params()"
```

---

## Task 2: `apos_geom_gauge_valid()` becomes dim-aware

**Files:**
- Modify: `src/apos_geom.c`
- Test: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Consumes: `enum apos_geom_dim`, `APOS_MIN_NODES_2D`/`APOS_MIN_NODES_3D` from Task 1.
- Produces: `apos_geom_gauge_valid()` now validates only 3 designations (and only requires 3 nodes) when `g->dim == APOS_GEOM_2D`.

- [ ] **Step 1: Write the failing test**

Add to `tests/apos_geom/test_apos_geom.c`, right after `test_gauge_requires_four_distinct_nodes()`:

```c
static void test_2d_gauge_needs_only_three_distinct_nodes(void)
{
    struct apos_gauge g2 = {.origin = 0, .xaxis = 1, .plane = 2, .up = 0,
			    .dim = APOS_GEOM_2D};
    struct apos_gauge bad2 = {.origin = 0, .xaxis = 1, .plane = 1, .up = 0,
			      .dim = APOS_GEOM_2D};

    /* Three nodes is enough in 2D -- unlike the 3D case in the previous
     * test, which correctly rejects n_nodes == 3. */
    CHECK(apos_geom_gauge_valid(&g2, 3));
    /* origin/xaxis/plane must still be distinct; up (0, same as origin)
     * is not checked in 2D mode and must NOT be the reason this fails. */
    CHECK(!apos_geom_gauge_valid(&bad2, 3));
    /* Below 3 nodes is still invalid. */
    CHECK(!apos_geom_gauge_valid(&g2, 2));
}
```

Add `test_2d_gauge_needs_only_three_distinct_nodes();` to `main()`, right after `test_gauge_requires_four_distinct_nodes();` and before `test_free_params_matches_each_dimensionality();`.

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `CHECK(apos_geom_gauge_valid(&g2, 3))` FAILs — today's code always requires 4 distinct nodes and unconditionally reads `up`.

- [ ] **Step 3: Implement the dim-aware validation**

Replace `apos_geom_gauge_valid()` in `src/apos_geom.c`:

```c
bool apos_geom_gauge_valid(const struct apos_gauge *g, uint8_t n_nodes)
{
	if (!g) {
		return false;
	}

	uint8_t min_nodes = (g->dim == APOS_GEOM_2D) ? APOS_MIN_NODES_2D
						      : APOS_MIN_NODES_3D;

	if (n_nodes < min_nodes || n_nodes > APOS_MAX_NODES) {
		return false;
	}

	/* up is the 4th designation and is not read at all in 2D mode -- its
	 * value there is a "don't care", not a sentinel. */
	uint8_t n_check = (g->dim == APOS_GEOM_2D) ? 3u : 4u;
	const uint8_t idx[4] = {g->origin, g->xaxis, g->plane, g->up};

	for (uint8_t a = 0; a < n_check; a++) {
		if (idx[a] >= n_nodes) {
			return false;
		}
		for (uint8_t b = (uint8_t)(a + 1); b < n_check; b++) {
			if (idx[a] == idx[b]) {
				return false;
			}
		}
	}
	return true;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `PASSED`, exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/apos_geom.c tests/apos_geom/test_apos_geom.c
git commit -m "feat(apos): apos_geom_gauge_valid() validates only origin/xaxis/plane in 2D mode"
```

---

## Task 3: 2D node placement in `apos_geom_seed()` — `trilaterate2()` and the 2D branch of `place_from_neighbours()`

**Files:**
- Modify: `src/apos_geom.c`
- Test: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Consumes: `enum apos_geom_dim`, `struct apos_gauge.dim`, `struct apos_result.dim` from Task 1; `apos_geom_gauge_valid()` from Task 2.
- Produces: `apos_geom_seed()` stamps `out->dim` and skips the `up` placement in 2D mode; `place_from_neighbours()` places from 2 neighbours (not 3) in 2D mode via a new static `trilaterate2()`.

- [ ] **Step 1: Write the failing test — minimal 3-node 2D triangle**

Add to `tests/apos_geom/test_apos_geom.c`, after the existing 3D `ref_xyz`/`g_ref` block (near line 51):

```c
/* A 3-node 2D reference layout: same origin/xaxis convention as the 3D
 * gauge, but z is always 0 -- there is no up. */
static const float ref_xy[5][2] = {
    {0.0f, 0.0f},  /* origin */
    {3.0f, 0.0f},  /* xaxis  */
    {0.0f, 4.0f},  /* plane  */
    {3.0f, 4.0f},  /* extra node #1, riding along */
    {1.5f, 2.0f},  /* extra node #2, riding along */
};

static const struct apos_gauge g_ref_2d = {
    .origin = 0, .xaxis = 1, .plane = 2, .up = 0, .dim = APOS_GEOM_2D
};

static float dist2(const float a[2], const float b[2])
{
    float dx = a[0] - b[0], dy = a[1] - b[1];

    return sqrtf(dx * dx + dy * dy);
}

static uint16_t build_full_mesh_2d(struct apos_edge *out, uint8_t n)
{
    uint16_t k = 0;

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
            out[k].i = i;
            out[k].j = j;
            out[k].d_m = dist2(ref_xy[i], ref_xy[j]);
            out[k].sd_m = 0.001f;
            k++;
        }
    }
    return k;
}

static void test_2d_seed_reproduces_the_exact_layout(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 3);

    CHECK(apos_geom_seed(e, n, 3, &g_ref_2d, &r) == 0);
    CHECK(r.dim == APOS_GEOM_2D);
    CHECK(r.n_placed == 3);
    CHECK(r.n_ambiguous == 0);

    for (int i = 0; i < 3; i++) {
        CHECK(r.node[i].state == APOS_NODE_PLACED);
        CLOSE(r.node[i].x, ref_xy[i][0], 1e-3f);
        CLOSE(r.node[i].y, ref_xy[i][1], 1e-3f);
        CLOSE(r.node[i].z, 0.0f, 1e-6f);
    }
}

/* A 4th node with 3 placed neighbours: enough to disambiguate the mirror
 * across the plane line, the 2D analogue of the 3D 4th-neighbour case. */
static void test_2d_fourth_neighbour_resolves_the_mirror(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 4);

    CHECK(apos_geom_seed(e, n, 4, &g_ref_2d, &r) == 0);
    CHECK(r.node[3].state == APOS_NODE_PLACED);
    CLOSE(r.node[3].x, ref_xy[3][0], 1e-3f);
    CLOSE(r.node[3].y, ref_xy[3][1], 1e-3f);
}

/* Exactly two neighbours: nothing in this node's own edges can choose a
 * side of the line through them. The centroid heuristic guesses and must
 * flag the guess, the 2D analogue of the 3D 3-neighbour case. */
static void test_2d_two_neighbours_is_flagged_ambiguous(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = 0;

    /* Only the gauge triangle plus node 3's two edges to origin/xaxis --
     * not the full mesh, so node 3 has exactly 2 placed neighbours. */
    n = build_full_mesh_2d(e, 3);
    e[n].i = 0; e[n].j = 3; e[n].d_m = dist2(ref_xy[0], ref_xy[3]);
    e[n].sd_m = 0.001f; n++;
    e[n].i = 1; e[n].j = 3; e[n].d_m = dist2(ref_xy[1], ref_xy[3]);
    e[n].sd_m = 0.001f; n++;

    CHECK(apos_geom_seed(e, n, 4, &g_ref_2d, &r) == 0);
    CHECK(r.node[3].state == APOS_NODE_AMBIGUOUS);
    CHECK(r.n_ambiguous == 1);
}
```

Add these three calls to `main()`, right after `test_2d_gauge_needs_only_three_distinct_nodes();`:

```c
    test_2d_seed_reproduces_the_exact_layout();
    test_2d_fourth_neighbour_resolves_the_mirror();
    test_2d_two_neighbours_is_flagged_ambiguous();
```

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `test_2d_seed_reproduces_the_exact_layout` FAILs — `apos_geom_seed()` still unconditionally tries the `up`-trilateration block, which returns `-ENODATA` on this 3-node input (no edges touch `g->up`'s own placed-neighbour count of 3, since `up == origin == 0` here and it is already placed, so `placed_neighbours()` finds 0 UNPLACED-state candidates matching `n == g->up`... in practice this returns `-ENODATA` well before reaching the per-node placement loop). `apos_geom_seed()`'s return value will not be `0`.

- [ ] **Step 3: Skip the `up` block in 2D mode, and stamp `dim`**

In `src/apos_geom.c`'s `apos_geom_seed()`, right after `memset(out, 0, sizeof(*out));`, add:

```c
	out->dim = g->dim;
```

Then wrap the existing `up`-trilateration block (from the comment `/* up: the only node whose reflection...` through the `set_node(out, g->up, ...)` call) in:

```c
	if (g->dim == APOS_GEOM_3D) {
		/* up: the only node whose reflection the operator resolved
		 * for us. */
		uint8_t nb[APOS_MAX_NODES];
		float rr[APOS_MAX_NODES];
		uint8_t cnt = placed_neighbours(e, n_edges, out, g->up, nb,
						rr, APOS_MAX_NODES);

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
		set_node(out, g->up, (sa[2] >= sb[2]) ? sa : sb,
			 APOS_NODE_PLACED);
	}
```

(This is the existing code, unchanged, only wrapped in the `if`.)

- [ ] **Step 4: Add `trilaterate2()` and the 2D branch of `place_from_neighbours()`**

Add `trilaterate2()` right after `trilaterate3()` in `src/apos_geom.c`:

```c
/* Intersect two circles, both implicitly at z = 0, centred on p0, p1 (2D)
 * with radii r0, r1. Writes the two solutions, mirrored across the line
 * p0-p1, into sa, sb. Returns 0, or -EDOM if the centres coincide or the
 * circles do not reach a common point. Same noise-tolerant clamping as
 * trilaterate3(): a near-miss within measurement noise is clamped to the
 * tangent point rather than rejected. Roughly a third the size of
 * trilaterate3() -- there is no third axis to resolve. */
static int trilaterate2(const float p0[2], const float p1[2], float r0,
			float r1, float sa[2], float sb[2])
{
	float dx = p1[0] - p0[0];
	float dy = p1[1] - p0[1];
	float d = sqrtf(dx * dx + dy * dy);

	if (d < EPS) {
		return -EDOM;
	}

	float exx = dx / d, exy = dy / d;   /* unit vector p0 -> p1 */
	float eyx = -exy, eyy = exx;        /* rotated 90 deg, in-plane */

	float x = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
	float y2 = r0 * r0 - x * x;
	float y;

	if (y2 < 0.0f) {
		float slack = 0.1f * r0;

		if (-y2 > slack * slack) {
			return -EDOM;
		}
		y = 0.0f;
	} else {
		y = sqrtf(y2);
	}

	sa[0] = p0[0] + x * exx + y * eyx;
	sa[1] = p0[1] + x * exy + y * eyy;
	sb[0] = p0[0] + x * exx - y * eyx;
	sb[1] = p0[1] + x * exy - y * eyy;
	return 0;
}
```

Replace `place_from_neighbours()`:

```c
static bool place_from_neighbours(const struct apos_edge *e, uint16_t n_edges,
				  struct apos_result *r, uint8_t n)
{
	uint8_t nb[APOS_MAX_NODES];
	float rr[APOS_MAX_NODES];
	uint8_t min_needed = (r->dim == APOS_GEOM_2D) ? 2u : 3u;
	uint8_t cnt = placed_neighbours(e, n_edges, r, n, nb, rr,
					APOS_MAX_NODES);

	if (cnt < min_needed) {
		return false;
	}

	if (r->dim == APOS_GEOM_2D) {
		float p0[2] = {r->node[nb[0]].x, r->node[nb[0]].y};
		float p1[2] = {r->node[nb[1]].x, r->node[nb[1]].y};
		float sa[2], sb[2];

		if (trilaterate2(p0, p1, rr[0], rr[1], sa, sb) != 0) {
			return false;
		}

		float pa[3] = {sa[0], sa[1], 0.0f};
		float pb[3] = {sb[0], sb[1], 0.0f};

		if (cnt >= 3) {
			/* A third range breaks the mirror across the line
			 * p0-p1: keep whichever branch agrees with it
			 * better, the 2D analogue of the 3D 4th-neighbour
			 * disambiguation below. */
			float p2[2] = {r->node[nb[2]].x, r->node[nb[2]].y};
			float da = fabsf(hypotf(sa[0] - p2[0], sa[1] - p2[1])
					  - rr[2]);
			float db = fabsf(hypotf(sb[0] - p2[0], sb[1] - p2[1])
					  - rr[2]);

			set_node(r, n, (da <= db) ? pa : pb, APOS_NODE_PLACED);
			return true;
		}

		/* Exactly two: nothing in this node's own edges can choose a
		 * side of the line p0-p1. Take the branch on the same side
		 * as the placed centroid, and flag it -- the 2D analogue of
		 * the 3-neighbour 3D fallback below. */
		float c[3];
		float u[2] = {p1[0] - p0[0], p1[1] - p0[1]};
		float v[2];

		placed_centroid(r, c);
		v[0] = c[0] - p0[0];
		v[1] = c[1] - p0[1];

		float cs = u[0] * v[1] - u[1] * v[0]; /* 2D cross, signed area */
		float va[2] = {sa[0] - p0[0], sa[1] - p0[1]};
		float as = u[0] * va[1] - u[1] * va[0];

		set_node(r, n, ((cs >= 0.0f) == (as >= 0.0f)) ? pa : pb,
			 APOS_NODE_AMBIGUOUS);
		return true;
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
```

(The 3D half of the function, below the 2D `if` block, is the pre-existing code, unchanged — only re-pasted here so the whole function is unambiguous to apply as one edit.)

`<math.h>` already provides `hypotf()`; no new include is needed.

- [ ] **Step 5: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `PASSED`, exit 0 — including every pre-existing 3D test, confirming the `if (g->dim == APOS_GEOM_3D)` wrap didn't change 3D behavior.

- [ ] **Step 6: Commit**

```bash
git add src/apos_geom.c tests/apos_geom/test_apos_geom.c
git commit -m "feat(apos): 2D node placement in apos_geom_seed() via a new trilaterate2()"
```

---

## Task 4: `pmap_build()` 2D branch, `dim` stamping in `apos_geom_refine()`, and end-to-end 2D solve tests

**Files:**
- Modify: `src/apos_geom.c`
- Test: `tests/apos_geom/test_apos_geom.c`

**Interfaces:**
- Consumes: everything from Tasks 1-3.
- Produces: `apos_geom_refine()`/`apos_geom_solve()` correctly solve a 2D gauge, never assigning any node a `z` free parameter, with `planarity_m` and `apos_geom_free_params()`'s formula both correct for the result's `dim`.

- [ ] **Step 1: Write the failing tests**

Add to `tests/apos_geom/test_apos_geom.c`, after the Task 3 tests:

```c
/* The core reuse claim: cost()/the Jacobian/apply_step() need no 2D-specific
 * code at all, because pmap_build() never hands out a z slot in 2D mode. A
 * 5-node 2D layout with real edge redundancy (2*5-3 = 7 free parameters
 * against a full 10-edge mesh) exercises the LM loop exactly as the 3D tests
 * already do. */
static void test_2d_solve_refines_a_five_node_layout(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 5);

    CHECK(apos_geom_solve(e, n, 5, &g_ref_2d, &r) == 0);
    CHECK(r.dim == APOS_GEOM_2D);
    CHECK(r.n_placed == 5);
    CHECK(r.rms_m < 1e-3f);

    for (int i = 0; i < 5; i++) {
        CLOSE(r.node[i].x, ref_xy[i][0], 1e-2f);
        CLOSE(r.node[i].y, ref_xy[i][1], 1e-2f);
        /* The core claim: z was never a free parameter, so it never
         * moved off its seeded 0.0f, regardless of how many LM
         * iterations ran. */
        CHECK(r.node[i].z == 0.0f);
    }
}

/* A bare 3-anchor 2D survey is exactly the degenerate isostatic case,
 * mirroring the existing 4-anchor 3D case: 2*3-3 = 3 free parameters
 * against exactly 3 edges, so rms_m reproduces any input exactly. */
static void test_2d_three_node_survey_is_degenerate(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 3);

    /* Deliberately wrong: a real triangle cannot have these three side
     * lengths (violates the triangle inequality against ref_xy's true
     * distances), yet with 0 spare edges the fit must still report
     * rms_m ~ 0 -- it has re-embedded whatever it was given, not
     * validated it. */
    e[0].d_m = 10.0f; /* origin-xaxis, was 3.0 */

    CHECK(apos_geom_solve(e, n, 3, &g_ref_2d, &r) == 0);
    CHECK(apos_geom_free_params(r.dim, r.n_placed) == 3);
    CHECK(r.rms_m < 1e-3f);
}

/* apos_geom_refine() called directly, without apos_geom_seed() first, still
 * gets the right dim -- the belt-and-suspenders stamp from g->dim, not
 * reliance on a caller having already run seed(). */
static void test_refine_stamps_dim_without_seed(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 3);

    memset(&r, 0, sizeof(r));
    r.n_nodes = 3;
    r.dim = APOS_GEOM_3D; /* deliberately wrong, to prove refine() corrects it */
    for (int i = 0; i < 3; i++) {
        r.node[i].x = ref_xy[i][0];
        r.node[i].y = ref_xy[i][1];
        r.node[i].z = 0.0f;
        r.node[i].state = APOS_NODE_PLACED;
    }
    r.n_placed = 3;

    CHECK(apos_geom_refine(e, n, &g_ref_2d, &r) == 0);
    CHECK(r.dim == APOS_GEOM_2D);
    for (int i = 0; i < 3; i++) {
        CHECK(r.node[i].z == 0.0f);
    }
}
```

Add the three calls to `main()`, right after the Task 3 test calls:

```c
    test_2d_solve_refines_a_five_node_layout();
    test_2d_three_node_survey_is_degenerate();
    test_refine_stamps_dim_without_seed();
```

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `test_2d_solve_refines_a_five_node_layout` FAILs (`pmap_build()` still hands every non-gauge node a `z` slot, and `apos_geom_refine()`'s own `apos_geom_gauge_valid()` guard call has no `io->dim` stamped yet at entry, so `io->dim` is still whatever `apos_geom_seed()` set it to from Task 3 -- in this specific case that part already works via Task 3's stamp, so the actual failure here is that `pmap_build()` gives node z-parameters and the fit moves them: `r.node[i].z == 0.0f` fails for at least one node). `test_refine_stamps_dim_without_seed` FAILs outright: `apos_geom_refine()` does not yet stamp `io->dim` from `g->dim` itself.

- [ ] **Step 3: Add the `pmap_build()` 2D branch**

In `src/apos_geom.c`'s `pmap_build()`, change:

```c
		m->idx[n][0] = (int8_t)p++;
		if (n == g->xaxis) {
			continue;
		}
		m->idx[n][1] = (int8_t)p++;
		if (n == g->plane) {
			continue;
		}
		m->idx[n][2] = (int8_t)p++;
```

to:

```c
		m->idx[n][0] = (int8_t)p++;
		if (n == g->xaxis) {
			continue;
		}
		m->idx[n][1] = (int8_t)p++;
		/* No node, gauge or otherwise, ever gets a z slot in 2D mode
		 * -- this one condition is the entire 2D LM story. Every
		 * downstream consumer (cost(), the Jacobian assembly,
		 * apply_step()) only ever walks pmap, so z stays pinned at
		 * its seeded 0.0f with no other code needing to change. */
		if (n == g->plane || r->dim == APOS_GEOM_2D) {
			continue;
		}
		m->idx[n][2] = (int8_t)p++;
```

- [ ] **Step 4: Stamp `dim` in `apos_geom_refine()`**

In `src/apos_geom.c`'s `apos_geom_refine()`, change:

```c
int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io)
{
	if (!e || !g || !io) {
		return -EINVAL;
	}
	if (!apos_geom_gauge_valid(g, io->n_nodes)) {
		return -EINVAL;
	}
```

to:

```c
int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io)
{
	if (!e || !g || !io) {
		return -EINVAL;
	}
	/* Stamped here independently of apos_geom_seed() (which also stamps
	 * it): a caller that builds a struct apos_result by hand and calls
	 * refine() directly, without seeding first, still gets the right
	 * dim from the one authoritative source, g->dim. */
	io->dim = g->dim;
	if (!apos_geom_gauge_valid(g, io->n_nodes)) {
		return -EINVAL;
	}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Isrc -o tests/apos_geom/test_apos_geom.exe tests/apos_geom/test_apos_geom.c src/apos_geom.c -lm && ./tests/apos_geom/test_apos_geom.exe`
Expected: `PASSED`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/apos_geom.c tests/apos_geom/test_apos_geom.c
git commit -m "feat(apos): pmap_build() never frees z in 2D mode; refine() stamps dim independently"
```

---

## Task 5: Gateway orchestration (`apos_gw.h` / `apos_gw.c`)

**Files:**
- Modify: `src/apos_gw.h`
- Modify: `src/apos_gw.c`

**Interfaces:**
- Consumes: `enum apos_geom_dim`, `apos_geom_free_params()` from Task 1; `apos_geom_gauge_valid()` from Task 2; the full 2D solve path from Tasks 3-4.
- Produces: `int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane, int32_t up);` (signature changes: `up` is now `int32_t`, `-1` means 2D), `enum apos_geom_dim apos_gw_gauge_dim(void);` (new).

This task is not host-testable — `apos_gw.c` includes `deca_device_api.h` and Zephyr headers directly. Verification is a full `west build`, deferred to Task 8's hardware checklist; there is no unit test step here.

- [ ] **Step 1: `apos_gw.h` — change `apos_gw_set_gauge()`'s signature, add `apos_gw_gauge_dim()`**

Change:

```c
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
```

to:

```c
/* Record the gauge as SHORT ADDRESSES, not node indices: indices are an artefact
 * of the order anchors happened to answer enumeration in, and would silently
 * point at different boards after a re-enumeration. Addresses are resolved to
 * indices at solve time.
 *
 * up == -1 selects 2D mode (no up designation, no reflection to resolve).
 * -1 is a dedicated sentinel, not 0: 0x0000 is UWB_ADDR_GATEWAY_RESERVED and
 * must stay rejected as an address on its own terms, distinct from "not
 * given".
 *
 * Returns 0, -EINVAL if origin/xaxis/plane are not distinct (or up, when
 * given, is not additionally distinct from them), or -EBUSY while a survey
 * runs. Does NOT require the addresses to be enumerated yet -- an operator
 * may legitimately set the gauge from a site sketch before powering the
 * array. */
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      int32_t up);

bool apos_gw_gauge_set(void);

/* The dimensionality implied by the currently-stored gauge. Only ever
 * meaningful once apos_gw_gauge_set() is true -- see apos_gw.c's
 * step_enum() for the one call site, which is only reached after that
 * precondition already holds. */
enum apos_geom_dim apos_gw_gauge_dim(void);
```

- [ ] **Step 2: `apos_gw.c` — gauge storage becomes signed, `-1` sentinel**

Change:

```c
static uint16_t gauge_addr[4];
```

to:

```c
static int32_t gauge_addr[4]; /* gauge_addr[3] (up) == -1 means 2D mode */
```

- [ ] **Step 2b: `apos_gw_init()` — keep `gauge_addr[3]` at the "unset" sentinel, not 0**

`apos_gw_init()` currently does `memset(gauge_addr, 0, sizeof(gauge_addr));`, which would leave the `up` slot at `0` rather than `-1`. `apos_gw_gauge_set()` (Step 3 below) never looks at `gauge_addr[3]`, so this is harmless in practice, but `apos_gw_gauge_dim()` (Step 5 below) would misreport `APOS_GEOM_3D` if ever called before a gauge is configured. Change:

```c
	memset(gauge_addr, 0, sizeof(gauge_addr));
```

to:

```c
	memset(gauge_addr, 0, sizeof(gauge_addr));
	gauge_addr[3] = -1; /* up unset means 2D, not "address 0x0000" */
```

- [ ] **Step 3: `apos_gw_gauge_set()` — only origin/xaxis/plane matter**

Change:

```c
bool apos_gw_gauge_set(void)
{
	return gauge_addr[0] != 0u && gauge_addr[1] != 0u &&
	       gauge_addr[2] != 0u && gauge_addr[3] != 0u;
}
```

to:

```c
bool apos_gw_gauge_set(void)
{
	return gauge_addr[0] != 0 && gauge_addr[1] != 0 && gauge_addr[2] != 0;
}
```

- [ ] **Step 4: `apos_gw_set_gauge()` — `up` optional, `-1` sentinel**

Change:

```c
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
```

to:

```c
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      int32_t up)
{
	if (apos_gw_busy()) {
		return -EBUSY;
	}

	const int32_t a[3] = {origin, xaxis, plane};

	for (int i = 0; i < 3; i++) {
		if (a[i] == 0 || a[i] == (int32_t)UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int j = i + 1; j < 3; j++) {
			if (a[i] == a[j]) {
				return -EINVAL;
			}
		}
	}
	if (up != -1) {
		if (up == 0 || up == (int32_t)UWB_ADDR_GATEWAY_RESERVED) {
			return -EINVAL;
		}
		for (int i = 0; i < 3; i++) {
			if (up == a[i]) {
				return -EINVAL;
			}
		}
	}

	gauge_addr[0] = a[0];
	gauge_addr[1] = a[1];
	gauge_addr[2] = a[2];
	gauge_addr[3] = up;
	return 0;
}
```

(`memcpy()` is dropped in favor of explicit assignment since the source is no longer a matching-width array; `<string.h>` stays included for other uses in the file.)

- [ ] **Step 5: Add `apos_gw_gauge_dim()`**

Add right after `apos_gw_set_gauge()`:

```c
enum apos_geom_dim apos_gw_gauge_dim(void)
{
	return (gauge_addr[3] == -1) ? APOS_GEOM_2D : APOS_GEOM_3D;
}
```

- [ ] **Step 6: `resolve_gauge()` — resolve `up` only when set, stamp `dim`**

Change:

```c
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
```

to:

```c
static int resolve_gauge(struct apos_gauge *g)
{
	uint8_t *out3[3] = {&g->origin, &g->xaxis, &g->plane};

	for (int k = 0; k < 3; k++) {
		int idx = apos_table_find_addr(&tbl, (uint16_t)gauge_addr[k]);

		if (idx < 0) {
			LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did "
				"not answer enumeration\"}",
				(uint16_t)gauge_addr[k]);
			return -ENOENT;
		}
		*out3[k] = (uint8_t)idx;
	}

	if (gauge_addr[3] == -1) {
		g->dim = APOS_GEOM_2D;
		/* Never read by apos_geom_gauge_valid()/apos_geom_seed() in
		 * 2D mode, but must still be a valid index (< n_nodes) since
		 * struct apos_gauge carries no separate "up is absent" bit at
		 * this layer -- see apos_geom.h. g->origin is always valid. */
		g->up = g->origin;
		return 0;
	}

	int idx = apos_table_find_addr(&tbl, (uint16_t)gauge_addr[3]);

	if (idx < 0) {
		LOG_ERR("{\"apos_error\":\"gauge address 0x%04X did not "
			"answer enumeration\"}", (uint16_t)gauge_addr[3]);
		return -ENOENT;
	}
	g->up = (uint8_t)idx;
	g->dim = APOS_GEOM_3D;
	return 0;
}
```

- [ ] **Step 7: `step_enum()` — the node-count gate uses the right minimum**

Change (inside `step_enum()`):

```c
		if (tbl.n_peers < APOS_MIN_NODES) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) answered; a 3D "
				"gauge needs at least %u\"}",
				tbl.n_peers, APOS_MIN_NODES);
			phase = APOS_GW_IDLE;
			return;
		}
```

to:

```c
		uint8_t min_nodes = (apos_gw_gauge_dim() == APOS_GEOM_2D)
					    ? APOS_MIN_NODES_2D
					    : APOS_MIN_NODES_3D;

		if (tbl.n_peers < min_nodes) {
			LOG_ERR("{\"apos_error\":\"%u anchor(s) answered; a "
				"%s gauge needs at least %u\"}",
				tbl.n_peers,
				(min_nodes == APOS_MIN_NODES_2D) ? "2D" : "3D",
				min_nodes);
			phase = APOS_GW_IDLE;
			return;
		}
```

- [ ] **Step 8: `do_solve()` — use `apos_geom_free_params()`**

Change:

```c
	solve_redundancy = (int16_t)((int)n_edges -
				     (3 * (int)res.n_placed - 6));
```

to:

```c
	solve_redundancy = (int16_t)((int)n_edges -
				     apos_geom_free_params(res.dim,
							   res.n_placed));
```

- [ ] **Step 9: `judge_result()` — free-params formula and the coplanarity warning**

Change:

```c
		"\"max_sd_mm\":%u,\"accepted\":%u}}",
		res.n_nodes, res.n_placed, res.n_ambiguous,
		(int)(res.rms_m * 1000.0f), (int)(res.worst_edge_m * 1000.0f),
		res.worst_i, res.worst_j,
		(int)(res.planarity_m * 1000.0f), res.iterations,
		solve_redundancy, solve_unverified ? 0u : 1u,
		max_recip_mm, max_sd_mm, accepted ? 1u : 0u);
```

is unchanged (it already just prints `solve_redundancy`, computed correctly in Step 8) — but the `LOG_WRN` a few lines below, inside `if (solve_unverified) { ... }`, has the literal formula in its format-argument list:

```c
		LOG_WRN("{\"apos_warn\":\"rms_mm and worst_mm are NOT a quality "
			"check on this array: %u usable edge(s) against %d free "
			"parameters (3N-6) leaves %d spare. With no spare edge "
			"the fit reproduces ANY set of ranges exactly, so "
			"rms_mm is ~0 however bad the ranging was.\"}",
			solve_n_edges, 3 * (int)res.n_placed - 6,
			solve_redundancy);
```

Change the free-parameter formatting to be dim-aware:

```c
		LOG_WRN("{\"apos_warn\":\"rms_mm and worst_mm are NOT a quality "
			"check on this array: %u usable edge(s) against %d free "
			"parameters (%s) leaves %d spare. With no spare edge "
			"the fit reproduces ANY set of ranges exactly, so "
			"rms_mm is ~0 however bad the ranging was.\"}",
			solve_n_edges,
			apos_geom_free_params(res.dim, res.n_placed),
			(res.dim == APOS_GEOM_2D) ? "2N-3" : "3N-6",
			solve_redundancy);
```

Then, further down, change:

```c
	if (planar && res.n_placed >= 4u) {
```

to:

```c
	if (res.dim == APOS_GEOM_3D && planar && res.n_placed >= 4u) {
```

(In 2D mode `planarity_m` is always exactly `0.0f` by construction — see Task 3/4 — so this warning would otherwise fire, meaninglessly, on every single 2D result.)

- [ ] **Step 10: `apos_gw_start_apply()` — same formula fix**

Change:

```c
		LOG_WRN("{\"apos_warn\":\"COMMITTING AN UNVERIFIED SURVEY: %u "
			"usable edge(s) against %d free parameters (3N-6), "
			"leaving %d spare edge(s), so rms_mm was ~0 however bad "
			"the ranging was.\"}", solve_n_edges,
			3 * (int)res.n_placed - 6, solve_redundancy);
```

to:

```c
		LOG_WRN("{\"apos_warn\":\"COMMITTING AN UNVERIFIED SURVEY: %u "
			"usable edge(s) against %d free parameters (%s), "
			"leaving %d spare edge(s), so rms_mm was ~0 however bad "
			"the ranging was.\"}", solve_n_edges,
			apos_geom_free_params(res.dim, res.n_placed),
			(res.dim == APOS_GEOM_2D) ? "2N-3" : "3N-6",
			solve_redundancy);
```

- [ ] **Step 11: `on_enum_rsp()` — add the `id` field to the per-peer log line**

Change:

```c
	LOG_INF("{\"apos_peer\":{\"idx\":%d,\"addr\":\"0x%04X\","
		"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\",\"pos_valid\":%u,"
		"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
		idx, addr, eui[0], eui[1], eui[2], eui[3], eui[4], eui[5],
		eui[6], eui[7], pv ? 1u : 0u, (double)x, (double)y, (double)z);
```

to:

```c
	LOG_INF("{\"apos_peer\":{\"idx\":%d,\"id\":%d,\"addr\":\"0x%04X\","
		"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\",\"pos_valid\":%u,"
		"\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
		idx, (int)(addr - UWB_ANCHOR_ADDR_BASE), addr, eui[0], eui[1],
		eui[2], eui[3], eui[4], eui[5], eui[6], eui[7],
		pv ? 1u : 0u, (double)x, (double)y, (double)z);
```

`uwb_config.h` (already included via `#include "uwb_config.h"` at the top of `apos_gw.c`) provides `UWB_ANCHOR_ADDR_BASE`.

- [ ] **Step 12: Build the firmware**

Run:
```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```
Expected: builds clean, no errors or new warnings.

- [ ] **Step 13: Commit**

```bash
git add src/apos_gw.h src/apos_gw.c
git commit -m "feat(apos): gateway orchestration supports 2D gauges, -1 up sentinel"
```

---

## Task 6: Console (`apos_shell.c`) — id-based `apos gauge`, `id`/`dim` in `apos enum`/`apos show`

**Files:**
- Modify: `src/apos_shell.c`

**Interfaces:**
- Consumes: `apos_gw_set_gauge(uint16_t, uint16_t, uint16_t, int32_t)`, `apos_gw_gauge_dim()` from Task 5.
- Produces: `apos gauge origin=<id> xaxis=<id> plane=<id> [up=<id>]`.

Not host-testable (`<zephyr/shell/shell.h>`). Verification is a full `west build` plus a documented manual console check.

- [ ] **Step 1: Replace `parse_addr()` with `parse_id()`**

`parse_addr()` (lines 29-39) is used only by `cmd_gauge()`; once that command is rewritten below, `parse_addr()` becomes dead code (and would trip `-Wall -Wextra` as an unused static function). Replace it entirely:

```c
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
```

with:

```c
/* Anchor id, 0..UWB_MAX_ANCHORS-1 -- the same space `anchor id` uses, so the
 * operator never does hex arithmetic to use `apos gauge`. allow_none permits
 * exactly "-1" as a second valid parse, for up=, which is the only one of
 * the four gauge designations that may be omitted (selecting 2D mode).
 * strtol() reports a non-numeric argument as 0, which must be rejected as
 * ambiguous with a real id 0 -- same reasoning as the address parser this
 * replaces. */
static bool parse_id(const char *arg, bool allow_none, int32_t *out)
{
	char *endptr;
	long v = strtol(arg, &endptr, 0);

	if (endptr == arg || *endptr != '\0') {
		return false;
	}
	if (allow_none && v == -1) {
		*out = -1;
		return true;
	}
	if (v < 0 || v >= (long)UWB_MAX_ANCHORS) {
		return false;
	}
	*out = (int32_t)v;
	return true;
}
```

- [ ] **Step 2: Rewrite `cmd_gauge()`**

Replace the entire `cmd_gauge()` function:

```c
static int cmd_gauge(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const keys[4] = {"origin=", "xaxis=", "plane=",
					    "up="};
	static const char *const req_keys[3] = {"origin=", "xaxis=",
						"plane="};
	/* -1 in every slot: for origin/xaxis/plane it means "not yet given"
	 * (parse_id() with allow_none=false can never itself produce -1 for
	 * those, so this is unambiguous with a real parse) and is rejected
	 * below if still -1 after the argument loop. For up= it is a valid
	 * parse result meaning "2D mode", exactly like an omitted key. */
	int32_t val[4] = {-1, -1, -1, -1};

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
			if (!parse_id(argv[a] + klen, k == 3, &val[k])) {
				shell_error(sh, "error: bad id in \"%s\" — "
						"expected 0..%u%s", argv[a],
					    UWB_MAX_ANCHORS - 1u,
					    (k == 3) ? " or -1" : "");
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

	for (int k = 0; k < 3; k++) {
		if (val[k] == -1) {
			shell_error(sh, "error: missing %s<id>", req_keys[k]);
			return -EINVAL;
		}
	}

	uint16_t addr[3];

	for (int k = 0; k < 3; k++) {
		addr[k] = (uint16_t)(UWB_ANCHOR_ADDR_BASE + val[k]);
	}

	rc = apos_gw_set_gauge(addr[0], addr[1], addr[2],
			       (val[3] == -1)
				       ? (int32_t)-1
				       : (int32_t)(UWB_ANCHOR_ADDR_BASE +
						   val[3]));
	if (rc == -EINVAL) {
		shell_error(sh, "error: the given ids must be distinct and "
				"in range 0..%u", UWB_MAX_ANCHORS - 1u);
		return rc;
	}
	if (rc) {
		shell_error(sh, "error: gauge refused (errno %d)", rc);
		return rc;
	}

	shell_print(sh, "{\"apos_gauge\":{\"origin\":%d,\"xaxis\":%d,"
			"\"plane\":%d,\"up\":%d,\"dim\":\"%s\"}}",
		    val[0], val[1], val[2], val[3],
		    (val[3] == -1) ? "2D" : "3D");
	return 0;
}
```

- [ ] **Step 3: Update the `gauge` subcommand's help text and argument-count bounds**

In the `SHELL_STATIC_SUBCMD_SET_CREATE(sub_apos, ...)` block, change:

```c
	SHELL_CMD_ARG(gauge, NULL,
		      "gauge origin=<addr> xaxis=<addr> plane=<addr> up=<addr> "
		      "— pin the coordinate frame",
		      cmd_gauge, 5, 0),
```

to:

```c
	SHELL_CMD_ARG(gauge, NULL,
		      "gauge origin=<id> xaxis=<id> plane=<id> [up=<id>] "
		      "— pin the coordinate frame. ids are 0..3, the same "
		      "space `anchor id` uses. Omit up= (or pass up=-1) for "
		      "a 2D (3-anchor) survey.",
		      cmd_gauge, 4, 1),
```

- [ ] **Step 4: `cmd_show()` — add `id` per peer and `dim` to the status line**

Change:

```c
	shell_print(sh, "{\"phase\":%u,\"session\":%u,\"gauge_set\":%u,"
			"\"have_result\":%u}",
		    st.phase, st.session, apos_gw_gauge_set() ? 1u : 0u,
		    st.have_result ? 1u : 0u);
```

to:

```c
	const char *dim_str = "unset";

	if (apos_gw_gauge_set()) {
		dim_str = (apos_gw_gauge_dim() == APOS_GEOM_2D) ? "2D" : "3D";
	}

	shell_print(sh, "{\"phase\":%u,\"session\":%u,\"gauge_set\":%u,"
			"\"dim\":\"%s\",\"have_result\":%u}",
		    st.phase, st.session, apos_gw_gauge_set() ? 1u : 0u,
		    dim_str, st.have_result ? 1u : 0u);
```

Then change:

```c
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
```

to:

```c
	shell_print(sh, "enumerated (%u):", t->n_peers);
	for (uint8_t k = 0; k < t->n_peers; k++) {
		shell_print(sh, "  {\"idx\":%u,\"id\":%u,\"addr\":\"0x%04X\","
				"\"eui\":\"%02X%02X%02X%02X%02X%02X%02X%02X\","
				"\"pos_valid\":%u}",
			    k,
			    (unsigned)(t->peer[k].short_addr -
				       UWB_ANCHOR_ADDR_BASE),
			    t->peer[k].short_addr,
			    t->peer[k].eui[0], t->peer[k].eui[1],
			    t->peer[k].eui[2], t->peer[k].eui[3],
			    t->peer[k].eui[4], t->peer[k].eui[5],
			    t->peer[k].eui[6], t->peer[k].eui[7],
			    t->peer[k].pos_valid ? 1u : 0u);
	}
```

- [ ] **Step 5: Build the firmware**

Run:
```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```
Expected: builds clean. Confirm no "unused function `parse_addr`" or similar warning survives.

- [ ] **Step 6: Manual console check (requires the gateway board flashed and at least 3 anchors enumerable)**

On the gateway's console:
```
apos enum
```
Expected: each `apos_peer` log line now includes `"id":<0-3>` alongside `"addr":"0x000N"`.

```
apos gauge origin=0 xaxis=1 plane=2
```
Expected: `{"apos_gauge":{"origin":0,"xaxis":1,"plane":2,"up":-1,"dim":"2D"}}`

```
apos show
```
Expected: the status line includes `"dim":"2D"`, and each enumerated peer line includes `"id"`.

```
apos gauge origin=0 xaxis=1 plane=2 up=3
```
Expected: `{"apos_gauge":{"origin":0,"xaxis":1,"plane":2,"up":3,"dim":"3D"}}` (only if a 4th anchor, id 3, is available to designate — otherwise this step confirms the command accepts the 4th argument, and `apos run` will report the missing peer at enumeration time exactly as it does today for a 3D gauge).

- [ ] **Step 7: Commit**

```bash
git add src/apos_shell.c
git commit -m "feat(apos): apos gauge takes anchor ids, up is optional (2D mode)"
```

---

## Task 7: Persistence (`apos_store.h` / `apos_store.c`)

**Files:**
- Modify: `src/apos_store.h`
- Modify: `src/apos_store.c`
- Modify: `src/apos_gw.c`

**Interfaces:**
- Consumes: `enum apos_geom_dim` from Task 1.
- Produces: `struct apos_survey.dim`, persisted.

Not host-testable (`apos_store.c` uses `<zephyr/settings/settings.h>`). Verification is a full `west build` plus the manual hardware check in Step 5.

- [ ] **Step 1: `apos_store.h` — add `dim` to `struct apos_survey`**

Change:

```c
struct apos_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	bool     valid;      /* false until a survey has been applied */
```

to:

```c
struct apos_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t  n_nodes;
	bool     valid;      /* false until a survey has been applied */
	enum apos_geom_dim dim; /* whether this survey solved z, or fixed it
				 * at 0 -- see apos_geom.h */
```

- [ ] **Step 2: `apos_store.c` — add `dim` to the persisted `stored_survey`, wire the read/write path**

Change:

```c
struct stored_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t                 n_nodes;
	uint8_t                 valid;
};
```

to:

```c
struct stored_survey {
	struct apos_survey_node node[APOS_MAX_NODES];
	uint8_t                 n_nodes;
	uint8_t                 valid;
	uint8_t                 dim; /* enum apos_geom_dim, stored as a plain
				      * uint8_t like valid above */
};
```

In `apos_settings_set()`'s `"survey"` branch, change:

```c
		memcpy(g_survey.node, s.node, sizeof(g_survey.node));
		g_survey.n_nodes = s.n_nodes;
		g_survey.valid = s.valid != 0u;
		return 0;
```

to:

```c
		memcpy(g_survey.node, s.node, sizeof(g_survey.node));
		g_survey.n_nodes = s.n_nodes;
		g_survey.valid = s.valid != 0u;
		g_survey.dim = (enum apos_geom_dim)s.dim;
		return 0;
```

In `apos_store_save()`, change:

```c
	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));
	memcpy(rec.node, s->node, sizeof(rec.node));
	rec.n_nodes = s->n_nodes;
	rec.valid = s->valid ? 1u : 0u;
```

to:

```c
	struct stored_survey rec;

	memset(&rec, 0, sizeof(rec));
	memcpy(rec.node, s->node, sizeof(rec.node));
	rec.n_nodes = s->n_nodes;
	rec.valid = s->valid ? 1u : 0u;
	rec.dim = (uint8_t)s->dim;
```

and, further down in the same function, change:

```c
	memcpy(g_survey.node, s->node, sizeof(g_survey.node));
	g_survey.n_nodes = s->n_nodes;
	g_survey.valid = s->valid;

	return 0;
```

to:

```c
	memcpy(g_survey.node, s->node, sizeof(g_survey.node));
	g_survey.n_nodes = s->n_nodes;
	g_survey.valid = s->valid;
	g_survey.dim = s->dim;

	return 0;
```

- [ ] **Step 3: `apos_gw.c`'s `persist_survey()` — populate `dim` on the record it builds**

In `persist_survey()`, right after `s.n_nodes = w;`, add:

```c
	s.dim = res.dim;
```

- [ ] **Step 4: Build the firmware**

Run:
```powershell
$env:ZEPHYR_BASE = "C:\Users\JoseAntonioLaraPerez\zephyrproject\zephyr"
west build -b ancla_esp32s3/esp32s3/procpu --pristine
```
Expected: builds clean.

- [ ] **Step 5: Manual hardware check**

After a successful `apos apply` in 2D mode (3-anchor gauge), run `apos show` on the gateway and confirm the stored-survey section reports coordinates with `z: 0.000` for every node, and (from Task 6's `cmd_show()` change) that the live status line's `"dim"` matches whatever gauge is currently configured. Then run `kernel reboot cold` and `apos show` again — the persisted survey must still report the same coordinates, confirming `dim` (and everything else) survived the round trip through flash.

- [ ] **Step 6: Commit**

```bash
git add src/apos_store.h src/apos_store.c src/apos_gw.c
git commit -m "feat(apos): persist which dimensionality a survey was solved in"
```

---

## Task 8: Documentation and end-to-end hardware verification

**Files:**
- Modify: `CLAUDE.md`
- Modify: `docs/anchor-auto-positioning.md`

- [ ] **Step 1: Update `CLAUDE.md`'s `apos` console section**

In the `apos gauge` line of the console command tree, change:

```
apos gauge origin=<addr> xaxis=<addr> plane=<addr> up=<addr>
                                pin the coordinate frame; named arguments, any
                                order, four distinct non-zero short addresses
```

to:

```
apos gauge origin=<id> xaxis=<id> plane=<id> [up=<id>]
                                pin the coordinate frame; named arguments, any
                                order, ids are 0..3 (the same space `anchor id`
                                uses). Omit up= (or pass up=-1) for a 2D
                                (3-anchor) survey; a 4th id there selects the
                                existing full 3D solve.
```

- [ ] **Step 2: Add a note to the "Hard-won facts" section**

Add, near the existing `APOS_MIN_NODES`/degenerate-mesh entry:

```
- **A 2D survey (3-anchor gauge) is exactly as degenerate as the 4-anchor 3D
  case, for the same reason.** `2N-3` free parameters against exactly 3 edges
  at `N = 3` is isostatic, so `rms_mm` reproduces any input exactly regardless
  of ranging quality -- identical to the 4-anchor 3D floor this file already
  documents at length. A 4th (or 5th+) anchor riding along in 2D mode is what
  turns it verified, exactly as a 5th does in 3D. See
  `docs/superpowers/specs/2026-08-18-apos-2d-survey-design.md`.
```

- [ ] **Step 3: Update `docs/anchor-auto-positioning.md`'s gauge-setup walkthrough**

Read the current file first (`docs/anchor-auto-positioning.md`) to find the exact `apos gauge` example command and update it in place to the id-based form, and add a short paragraph before it explaining the 2D/3D choice (mirroring the design spec's §1 motivation: a 2D, 3-anchor survey is the better fit when a 4th anchor is unreliable or when height data is not needed yet, and how to opt into the existing 3D solve by adding a 4th id as `up=`).

- [ ] **Step 4: Full hardware verification**

With whichever anchors currently answer enumeration reliably (3 minimum):
```
apos enum
apos gauge origin=<id> xaxis=<id> plane=<id>
apos run
apos apply
kernel reboot cold
```
Expected: `apos run` no longer refuses at the enumeration-count check with only 3 anchors present; the solve reports `2N-3` in its free-parameter accounting; `apos apply` persists successfully; coordinates survive the reboot. If a 4th anchor answers reliably on a given day, repeat the same procedure with `up=<id>` added, to confirm the existing 3D path is untouched.

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md docs/anchor-auto-positioning.md
git commit -m "docs(apos): document the 2D survey mode and the id-based gauge command"
```

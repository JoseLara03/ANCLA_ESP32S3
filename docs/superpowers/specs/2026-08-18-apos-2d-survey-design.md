# Anchor survey: 2D mode alongside the existing 3D solve

Status: approved for implementation planning.

## 1. Motivation

The anchor survey (`docs/superpowers/specs/2026-08-14-anchor-auto-positioning-design.md`)
is a full 3D solve, gauged by four operator-designated anchors (`origin`,
`xaxis`, `plane`, `up`), and therefore requires a minimum of four anchors to
answer enumeration before anything can be solved (`APOS_MIN_NODES = 4`).

On the actual bench array, one anchor (`0x0002`) intermittently fails to
answer enumeration — believed to be either a marginal battery supply or a
still-imperfect PA solder joint, under separate investigation. Because the
current design has no slack between "enough anchors to solve" and "all
anchors present", a single flaky board blocks the whole survey even though
three anchors consistently answer every time.

Separately: the tag's own position fix is already 2D (`pos_json.c`'s payload
contract hardcodes `z: 0`), so a 3D anchor survey has arguably been solving
for more than the current deployment consumes. A 2D survey — minimum 3
anchors, real DOF count `2N-3` instead of `3N-6` — is a better match for what
this deployment actually needs today, without giving up the ability to do a
real 3D survey later (e.g. anchors at different heights, or once the flaky
board is fixed for good and a 4th anchor is reliably present).

## 2. Goals / non-goals

| | |
|---|---|
| Goal | A 2D survey mode: 3-anchor gauge (`origin`, `xaxis`, `plane`), no `up`, minimum 3 nodes to solve. |
| Goal | The existing 3D mode keeps working unchanged, selected by the same command. |
| Goal | Maximum reuse of the existing LM solver — this is a mode switch, not a second solver. |
| Goal | Extra anchors beyond the 3 (or 4) gauge points ride along in the solve exactly as they do today, whichever mode is active. |
| Goal | `apos gauge` takes anchor **ids** (`0..3`, the same space as `anchor id`), not short addresses — see §6. |
| Non-goal | Automatic mode selection (e.g. "use 2D if only 3 anchors answered"). The operator picks the mode explicitly via whether `up=` is given, exactly as designed in §6. |
| Non-goal | Fixing the `0x0002` flakiness itself — that's the hardware/firmware investigation already in progress. This feature reduces its blast radius; it doesn't address its cause. |
| Non-goal | Migrating already-persisted 3D survey data. See §9. |

## 3. Solver core (`apos_geom.h` / `apos_geom.c`)

### 3.1 Data model

```c
/* APOS_GEOM_3D is deliberately the zero value, not APOS_GEOM_2D: every
 * existing struct apos_gauge literal (in apos_gw.c and in
 * tests/apos_geom/test_apos_geom.c) predates this field and does not
 * mention .dim, so it zero-initializes. Zero must mean "today's existing
 * 3D behaviour" or every one of those untouched call sites silently
 * becomes a 2D solve the moment this field exists. */
enum apos_geom_dim {
	APOS_GEOM_3D = 0,
	APOS_GEOM_2D = 1,
};

struct apos_gauge {
	uint8_t origin;
	uint8_t xaxis;
	uint8_t plane;
	uint8_t up;             /* ignored entirely when dim == APOS_GEOM_2D */
	enum apos_geom_dim dim;
};
```

`dim` lives on the gauge, not inferred from whether `up` "looks unset" —
`apos_geom`'s gauge stores **node indices** into the edge list, and `0` is a
legitimate index, so it cannot double as a sentinel at this layer (that
sentinel problem belongs to `apos_gw.c`, addressed separately in §5).

`struct apos_result` also gains a `dim` field, stamped independently by both
`apos_geom_seed()` and `apos_geom_refine()` from `g->dim` (not carried over by
assumption from one to the other — see §3.3), so any caller holding a
`struct apos_result` always knows which kind of result it is without
inferring it from whether `z` happens to be zero.

`APOS_MIN_NODES` (today a single constant, 4) becomes two:

```c
#define APOS_MIN_NODES_2D 3
#define APOS_MIN_NODES_3D 4
```

### 3.2 Why the LM refinement loop itself does not change

`pmap_build()` is the single place that decides which coordinates are free
parameters in the fit; `cost()`, the Jacobian assembly, `apply_step()`, and
`solve_dense()` only ever consult `pmap`. Today, `xaxis` already gets no `y`or `z` parameter slot and `plane` already gets no `z` slot — by
construction, that's already "this coordinate is fixed at its seeded value,
never perturbed." A 2D solve is simply: *no node, gauge or otherwise, ever
gets a `z` slot.* One added condition in `pmap_build()`:

```c
if (r->dim == APOS_GEOM_2D) {
	/* never assign axis 2 (z) to anyone */
} else if (n != g->plane) {
	m->idx[n][2] = (int8_t)p++;
}
```

With every node's `z` pinned at its seeded value of `0.0f`, `cost()`'s
`vnorm(d)` naturally computes `sqrt(dx² + dy² + 0²)` — the correct planar
distance — with **zero changes** to `cost()`, the gradient loop, or
`apply_step()`. `fill_diagnostics()`'s `planarity()` also needs no change:
with every `z` at exactly 0, its covariance matrix already has a zero
eigenvalue along `z`, so it already returns `0.0f` — correctly, if now
definitionally rather than diagnostically (see §4 for how the *caller*
handles that distinction).

### 3.3 What does need new code

**`apos_geom_gauge_valid()`:** in 2D mode, validate only that `origin`,
`xaxis`, `plane` are distinct and `< n_nodes`; do not read `g->up` at all.
The node-count floor becomes `n_nodes < (g->dim == APOS_GEOM_2D ?
APOS_MIN_NODES_2D : APOS_MIN_NODES_3D)`.

**`apos_geom_seed()`:**
- Stamp `out->dim = g->dim` immediately after the `memset`, before anything
  else runs.
- `origin`/`xaxis`/`plane` placement is unchanged — the existing `px, py`
  computation already places `plane` in the `z = 0` plane via a 2-circle
  intersection. This is already exactly the math a 2D solve needs.
- Skip the `up`-trilateration block entirely when `dim == APOS_GEOM_2D` —
  there is no reflection to resolve in 2D, `up` is not read.
- The general "place everything else" loop calls `place_from_neighbours()`
  unchanged in shape; that function gains an internal `dim` branch (reading
  `r->dim`, no new parameter needed):
  - **2D:** minimum 2 placed neighbours (not 3). Placement via a new
    `trilaterate2()` (§3.4). Disambiguate the line-mirror using a 3rd
    neighbour if one is available (compare its measured distance against
    both candidate points, keep the closer match — the 2D analogue of
    today's 4th-neighbour check), else fall back to the existing
    placed-centroid heuristic (projected onto the 2D plane: which side of
    the line joining the two neighbours the centroid falls on) and flag
    `APOS_NODE_AMBIGUOUS`, exactly mirroring today's 3-neighbour 3D
    fallback.
  - **3D:** unchanged, byte-for-byte.

**New: `trilaterate2()`** — a 2-circle intersection, modeled directly on the
existing `px`/`py` formula already used to seed `plane`:

```c
/* Intersect two circles (both implicitly at z = 0) centred on p0, p1 with
 * radii r0, r1. Writes the two solutions mirrored across the line p0-p1
 * into sa, sb. Returns 0, or -EDOM if the centres coincide or the circles
 * do not reach a common point, with the same noise-tolerant clamping
 * trilaterate3() already applies (a near-miss within measurement noise is
 * clamped to a tangent point rather than rejected). */
static int trilaterate2(const float p0[2], const float p1[2], float r0,
			float r1, float sa[2], float sb[2]);
```

Roughly a third the size of `trilaterate3()` (no `ez`/cross-product term,
since there is no third axis to resolve).

**`apos_geom_refine()`:** stamp `io->dim = g->dim` at entry (independently
of whatever `apos_geom_seed()` already stamped) — this is the "don't trust
call-order discipline" belt-and-suspenders fix: a host test that constructs
a `struct apos_result` by hand and calls `apos_geom_refine()` directly
(without going through `apos_geom_seed()` first) still gets the right `dim`
without needing to remember to set it itself.

**New: `apos_geom_free_params()`** — replaces the `3 * n_placed - 6` literal
expression currently duplicated three times in `apos_gw.c`:

```c
/* Free parameters for a solve of this dimensionality: translation + rotation
 * only, the gauge already having removed the rest. 2N-3 in 2D (2 translation
 * + 1 rotation), 3N-6 in 3D (3 translation + 3 rotation). */
int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed);
```

### 3.4 A 3-anchor 2D survey is still exactly the degenerate case

Worth stating explicitly, since it's a direct consequence of §3.3's DOF
formula and validates the motivating request in §1: at `N = 3` in 2D,
`2·3 - 3 = 3` free parameters against exactly 3 edges (a full triangle) —
isostatic, no spare equation, `rms_m` reproduces any input exactly regardless
of quality. This is the *same* condition the existing 4-anchor 3D deployment
already runs into (`3·4 - 6 = 6` against 6 edges), and the existing
transparency machinery (`solve_unverified`, the `spare_edges` diagnostic, the
two `LOG_WRN`s in `judge_result()`, the `shell_warn()` in `apos apply`)
requires no new logic — only the updated DOF formula from
`apos_geom_free_params()`. A 4th anchor riding along in 2D mode is precisely
what turns this verified, exactly as a 5th does in 3D.

## 4. Gateway orchestration (`apos_gw.c` / `apos_gw.h`)

### 4.1 Gauge storage and the `-1` sentinel

`gauge_addr[4]` changes from `uint16_t` to `int32_t`. `origin`/`xaxis`/`plane`
remain mandatory and keep `0` as their "missing" convention (a real address
can never validly be `0`, which is also `UWB_ADDR_GATEWAY_RESERVED`). `up`
uses a dedicated sentinel, `-1`, meaning "not provided" — distinguishable
from every valid `uint16_t` address including `0x0000`, so there is no
overlap between "operator didn't specify `up`" and any address value that
could ever legitimately appear there.

```c
int apos_gw_set_gauge(uint16_t origin, uint16_t xaxis, uint16_t plane,
		      int32_t up); /* up == -1: 2D mode */
```

Validation: `origin`/`xaxis`/`plane` must be non-zero, not
`UWB_ADDR_GATEWAY_RESERVED`, and pairwise distinct. If `up != -1`, it must
additionally be non-zero, not the gateway's address, and distinct from the
other three.

`apos_gw_gauge_set()` (the bool gating `apos run`) becomes "origin, xaxis,
and plane are all set" — `up` being `-1` is not a reason to refuse.

### 4.2 `resolve_gauge()`

Resolves `origin`/`xaxis`/`plane` to node indices unconditionally. Resolves
`up` and sets `dim = APOS_GEOM_3D` only if `gauge_addr[3] != -1`; otherwise
leaves `g->up` untouched (its value is never read downstream once
`dim == APOS_GEOM_2D` — see §3.3) and sets `dim = APOS_GEOM_2D`.

### 4.3 Enumeration-count gate

`step_enum()`'s check (`tbl.n_peers < APOS_MIN_NODES`) needs the minimum for
whichever gauge is actually configured. A small accessor,
`apos_gw_gauge_dim()`, returns the `dim` implied by the currently-stored
`gauge_addr[3]` (`APOS_GEOM_2D` if `-1`, `APOS_GEOM_3D` otherwise). It is
only ever called from the node-count check inside `step_enum()`'s
`is_run == true` branch, and `apos_gw_start_run()` already refuses to start
(`-EINVAL`) unless `apos_gw_gauge_set()` is true — i.e. `origin`/`xaxis`/
`plane` are already guaranteed present by the time this accessor is ever
consulted, so it never needs to represent "no gauge at all". The bare
`apos enum` path (no `is_run`) never reaches this check and is unaffected.

### 4.4 `do_solve()` / `judge_result()`

- `solve_redundancy = (int16_t)((int)n_edges - apos_geom_free_params(res.dim, res.n_placed))`
  — replaces the literal `3 * (int)res.n_placed - 6`, used identically in
  the two `LOG_WRN` messages in `judge_result()` and the one in
  `apos_gw_start_apply()`.
- The "array is near-coplanar" warning in `judge_result()` is gated on
  `res.dim == APOS_GEOM_3D` in addition to its existing `planar &&
  n_placed >= 4` condition — in 2D mode `planarity_m` is always exactly
  `0.0f` by construction (§3.2), so printing this warning would fire on
  every single 2D result and tell the operator nothing true about their
  array.

## 5. Console (`apos_shell.c`)

### 5.1 `apos gauge` takes anchor ids, not addresses

```
apos gauge origin=<id> xaxis=<id> plane=<id> [up=<id>]
```

`<id>` is the same `0..UWB_MAX_ANCHORS-1` space `anchor id` already uses —
the operator never does hex arithmetic. `cmd_gauge()` converts each id to a
short address (`UWB_ANCHOR_ADDR_BASE + id`, the same conversion
`uwb_config_short_addr()` already performs) before calling
`apos_gw_set_gauge()` exactly as designed in §4.1; everything below that
call is unaffected by this change; it is purely an input-presentation layer.

`up=` is optional: omitting the key, or writing `up=-1` explicitly, both
select 2D mode. `origin=`/`xaxis=`/`plane=` remain mandatory; a bare integer
outside `0..UWB_MAX_ANCHORS-1` (or `-1` for one of these three) is a parse
error, matching the existing address-validation error-message style.

`SHELL_CMD_ARG`'s argument-count bounds change from `(5, 0)` (4 mandatory
args) to `(4, 1)` (3 mandatory, 1 optional).

A structural side effect worth stating plainly: since the id space `0..3`
never includes any encoding of the gateway's own address, a mistyped gauge
designation can no longer land on the gateway at all — this whole class of
input error is now impossible rather than merely rejected at validation
time.

### 5.2 `apos enum` / `apos show` print the id too

Both `on_enum_rsp()`'s per-peer `LOG_INF` and `cmd_show()`'s enumerated-peer
listing gain an `"id"` field (`short_addr - UWB_ANCHOR_ADDR_BASE`) alongside
the existing hex `"addr"` field. The operator needs the id to type into
`apos gauge`; the hex address stays because it is what's actually on the
wire, and this whole investigation has repeatedly cross-referenced it
against sniffer captures.

`apos show` also reports whichever mode the current gauge (or last solved
result) is in, so an operator seeing `z: 0.000` on every anchor can tell
"this was a deliberate 2D survey" from "something is wrong."

## 6. Persistence (`apos_store.h`)

`struct apos_survey` gains a `dim` field (`enum apos_geom_dim`), populated
from `res.dim` when `persist_survey()` (`apos_gw.c`) builds the record. This
lets `apos show` and any future re-survey workflow distinguish "this survey
never solved Z" from "Z happened to come out zero" — directly serving the
stated future intent to use real Z once the deployment is ready for it.

`apos_store.c` deliberately persists a *separate*, narrower `struct
stored_survey` (geometry only — `node[]`, `n_nodes`, `valid`), not
`struct apos_survey` itself, specifically so a new field on the public
struct never silently changes the on-flash layout. `dim` must therefore be
added to `stored_survey` too (as a plain `uint8_t`, matching the style of
its existing `valid` field) and threaded through `apos_settings_set()`'s
read path and `apos_store_save()`'s write path explicitly — it does not
persist "for free" just by existing on `apos_survey`.

**Compatibility:** no anchor survey has yet been successfully `apos
apply`'d and persisted on real hardware (the `0x0002` flakiness has blocked
every attempt so far), so there is no existing persisted data to migrate.
If that assumption is wrong, `apos_settings_set()`'s existing
`len != sizeof(s)` guard is what actually protects against it: adding a
field changes `sizeof(struct stored_survey)`, so an old-format record fails
that check and is treated as "no survey" (falling back to the stub) rather
than being loaded with a wrong or garbage `dim` — no new code needed for
that case. The `APOS_GEOM_3D == 0` ordering from §3.1 is a second,
independent layer of the same protection, for the parts of the codebase
that zero-initialize a `struct apos_survey` in memory rather than reading
one off flash (e.g. test fixtures).

## 7. What does not change

- **Wire format (`apos_frame.c`).** No frame gains or loses a field. A
  2D-solved node's `z` is simply `0.0f` when it goes out in `SETPOS` —
  exactly the same wire representation a 3D solve already produces for a
  node that happens to sit at `z = 0`. `RANGE_CMD`/`RANGE_RSP`/`ENUM_RSP`
  carry no concept of dimensionality at all; ranging is dimension-agnostic
  by nature.
- **`apos_table.c`.** Peer bookkeeping, EUI-keyed lookup, and directed→
  undirected symmetrisation have no notion of 2D vs 3D and need no changes.
- **`anchor_respond.c`, `pos_json.c`, the tag.** All already treat every
  anchor's `z` as "whatever was solved" and already publish `z: 0` in the
  tag's own fix regardless. Nothing downstream of the survey needs to know
  which mode produced the anchors payload.

## 8. Testing

**Host-tested (`tests/apos_geom/`, no hardware):**
- A minimal 3-node 2D triangle, mirroring the existing 4-node 3D degenerate
  case: confirm `rms_m` is identically `0` regardless of input quality
  (§3.4), and that `solve_redundancy`/`apos_geom_free_params()` reports the
  correct `2N-3` figure.
- A 4-5 node 2D layout with real edge redundancy, exercising
  `trilaterate2()`'s 3rd-neighbour disambiguation path and the 2-neighbour
  centroid-heuristic/`APOS_NODE_AMBIGUOUS` fallback path.
- A synthetic check that no node ever receives a `z` free parameter when
  `dim == APOS_GEOM_2D` (constructible by feeding deliberately
  z-inconsistent edges and confirming the fit still converges with every
  `z` at exactly `0`, since `z` was never free to move).
- `apos_geom_refine()` called directly (without `apos_geom_seed()` first)
  against a hand-built `struct apos_result`, confirming `dim` is correctly
  stamped from `g->dim` regardless of call order (§3.3).

**Unaffected test suites:** `tests/apos_frame/`, `tests/apos_table/`,
`tests/pos_json/` need no new cases or fixture edits — see §7.
`tests/pos_json/`'s fixtures all `memset(&s, 0, sizeof(s))` before setting
individual fields, so the new `dim` field zero-initializes to
`APOS_GEOM_3D` automatically and compiles unchanged.

**Hardware verification:** the existing procedure, unchanged in shape —
`apos enum` → `apos gauge origin=<id> xaxis=<id> plane=<id>` (3 ids, no
`up=`) → `apos run` → `apos apply` → `kernel reboot cold` — run against
whichever 3 (or 4, if `0x0002` cooperates that day) anchors actually answer.

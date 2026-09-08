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

/* Survey capacity: the full 32-anchor deployment UWB_MAX_ANCHORS now supports.
 *
 * Deliberately still a SEPARATE constant rather than `#define APOS_MAX_NODES
 * UWB_MAX_ANCHORS`, even though the two values now agree. Nothing in this file
 * may depend on the tag-facing ranging MAC: this module is pure C, host-tested
 * with synthetic geometry, and its node count is a property of the solver's
 * storage, not of how many anchors a tag can poll. The values being equal is a
 * fact about today's deployment, not a coupling.
 *
 * Sized structures at 32 nodes, all of which are deliberately kept OFF the
 * gateway loop's 4 kB stack (CONFIG_MAIN_STACK_SIZE) -- see the .bss note on
 * apos_geom_refine()'s working matrices below, and apos_gw.c's file-scope
 * `edges`/`survey_rec`:
 *   APOS_MAX_EDGES  = 32*31/2 = 496 edges  -> struct apos_edge[496]  ~5.8 kB
 *   APOS_MAX_MEAS   = 32*31   = 992 rows   -> struct apos_table      ~12.7 kB
 *   APOS_MAX_PARAMS = 3*32-6  = 90 params  -> one 90x90 float matrix ~32 kB
 *
 * Measured with xtensa-...-size on the objects: apos_geom.c 33792 B of .bss,
 * apos_gw.c 20361, net_uplink.c 11311 -- about +52 kB over the 8-node sizing,
 * against 127164 B free in dram0_0_seg (0x61704 = 389 kB, of which the last
 * pre-change image left 0x1F0BC unused after .dram0.bss). It fits with ~73 kB
 * to spare, and it is now the single largest RAM item in the firmware. Check
 * the map before adding another node-count-squared array here. NOTE: those
 * are per-object figures, not a linked image -- the vendored decadriver
 * currently fails to compile against this Zephyr checkout
 * (platform/dw3000_spi.c's `#include "version.h"`, unrelated to the survey),
 * so a whole-image RAM report has not been produced.
 */
#define APOS_MAX_NODES 32
#define APOS_MAX_EDGES ((APOS_MAX_NODES * (APOS_MAX_NODES - 1)) / 2)

/* Free parameters at full capacity, i.e. the largest value
 * apos_geom_free_params() can return: 3N-6 in 3D, which dominates 2N-3.
 * apos_geom_refine()'s normal-equation matrices are sized on this, and its
 * column indices are int8_t -- so this must stay under 128. */
#define APOS_MAX_PARAMS (3 * APOS_MAX_NODES - 6)

/* The rigidity check and apos_table's enumeration-adjacency bitmap both key a
 * uint32_t by node index / anchor id, so 32 is a hard ceiling rather than a
 * tuning choice. Raising APOS_MAX_NODES past it must widen those bitmaps
 * first; fail the build rather than silently losing the high nodes. */
_Static_assert(APOS_MAX_NODES <= 32,
	       "APOS_MAX_NODES > 32 needs the uint32_t adjacency bitmaps in "
	       "apos_geom.c and apos_table.h widened first");
_Static_assert(APOS_MAX_PARAMS < 128,
	       "APOS_MAX_PARAMS >= 128 overflows struct pmap's int8_t column "
	       "indices in apos_geom.c");

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

enum apos_node_state {
	APOS_NODE_UNPLACED  = 0, /* fewer than 3 edges to placed nodes in 3D,
				 * fewer than 2 in 2D (no reflection to resolve
				 * out of the plane) */
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
	enum apos_geom_dim dim;
	float    rms_m;        /* RMS residual over all usable edges */
	float    worst_edge_m; /* largest |residual| over usable edges */
	uint8_t  worst_i;      /* the pair that produced worst_edge_m */
	uint8_t  worst_j;
	float    planarity_m;  /* RMS distance of placed nodes from their own
				* best-fit plane. Small means the array is
				* near-coplanar and z is not trustworthy.
				* 3D only -- identically 0 in 2D mode, where
				* three or fewer distinct z values make the
				* metric meaningless by construction. */
	float    gauge_collinearity_ratio; /* 2D only; 0 in 3D mode. Perpendicular
				* distance of `plane` from the origin-xaxis
				* line, divided by the origin-xaxis baseline
				* length. Small means the three gauge nodes are
				* nearly in a line, so the solved +y direction
				* (and therefore every placed node's y) is
				* noise-dominated -- the 2D analogue of
				* planarity_m above. Origin sits at (0,0,0) and
				* xaxis always has y = z = 0 by construction
				* (neither is a free LM parameter on that axis),
				* so this reduces to |plane.y| / xaxis.x. */
	uint16_t iterations;   /* LM iterations used; 0 after seed alone */
};

/* Four distinct indices (origin/xaxis/plane/up), each < n_nodes, and n_nodes
 * in range -- or three (origin/xaxis/plane, up unchecked) when dim ==
 * APOS_GEOM_2D. */
bool apos_geom_gauge_valid(const struct apos_gauge *g, uint8_t n_nodes);

/* Closed-form initial placement. Fills out->node[].{x,y,z,state}, n_nodes,
 * n_placed and n_ambiguous. Does not compute residuals or planarity -- that is
 * apos_geom_refine()'s job.
 *
 * Returns 0 on success, -EINVAL on a bad argument or invalid gauge, or -ENODATA
 * if the gauge nodes lack the edges needed to place them (the three gauge-plane
 * edges, plus in 3D mode at least three more edges from `up` -- 2D has no `up`
 * to place). Nodes that cannot be placed are left APOS_NODE_UNPLACED and are
 * NOT an error -- they are reported. */
int apos_geom_seed(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		   const struct apos_gauge *g, struct apos_result *out);

/* Shift every placed node's z by dz, moving the z = 0 plane. Applied after
 * solving so the operator can put z = 0 on the floor rather than on the plane
 * through the three gauge anchors.
 *
 * In 2D mode every z is 0 before this call, so a nonzero dz here shifts the
 * whole 2D survey off the z = 0 plane it is otherwise defined to occupy. This
 * function does not know or care which mode produced `r` -- a stale nonzero
 * zoff_m left over from a previous 3D survey applies just as literally to the
 * next 2D one. Callers that want 2D surveys to stay at z = 0 must not call
 * this with a nonzero dz for them; apos_gw.c does not currently special-case
 * this. */
void apos_geom_zoff(struct apos_result *r, float dz);

/* Free parameters for a solve of this dimensionality: translation +
 * rotation only, the gauge having already fixed the rest. 2N-3 in 2D (2
 * translation + 1 rotation), 3N-6 in 3D (3 translation + 3 rotation).
 * The one place the formula lives -- apos_geom_rigidity() below reports it as
 * struct apos_rigidity.free_params, which is what apos_gw.c and apos_shell.c
 * read rather than each re-deriving the expression. */
int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed);

/* Spatial dimensions the mode solves in: 2 or 3.
 *
 * NOT the enum value. APOS_GEOM_3D is 0 and APOS_GEOM_2D is 1 (see the note on
 * enum apos_geom_dim above), so `(int)dim` is exactly backwards and using it as
 * a dimension count is the obvious way to get every rigidity bound wrong by a
 * factor that still looks plausible. Every place that needs `d` goes through
 * here. */
int apos_geom_dims(enum apos_geom_dim dim);

/* ---- Rigidity ---- */

/* Whether the framework the edge list describes CAN determine a shape at all,
 * independently of what any particular fit produced.
 *
 * Three NECESSARY conditions for generic rigidity in R^d, checked over the
 * graph alone -- no coordinates, so this is valid before or after a solve:
 *
 *   1. CONNECTED. A framework in two or more pieces has a relative motion
 *      between them by construction, whatever each piece measures.
 *   2. MINIMUM DEGREE >= d. A vertex with fewer than d edges is not pinned: in
 *      2D a degree-1 vertex swings on a circle, in 3D a degree-2 vertex swings
 *      about the axis through its two neighbours. (Exactly d edges pins it up
 *      to a REFLECTION through its neighbours' (d-1)-flat -- a discrete
 *      ambiguity, not a flex, and already reported separately as
 *      APOS_NODE_AMBIGUOUS. d+1 edges is what removes that.)
 *   3. EDGE COUNT >= apos_geom_free_params(dim, n_nodes). Fewer independent
 *      distance constraints than free parameters leaves the fit
 *      under-determined however good the ranging was.
 *
 * NOT SUFFICIENT, and the gap is real rather than theoretical. None of the
 * three sees a HINGE: two densely-meshed clusters joined at a single shared
 * anchor (two K5s sharing a vertex, say -- 9 nodes, 20 edges against 2N-3 = 15
 * in 2D) is connected, has min degree 4, passes the count, and still rotates
 * freely about the shared node. The general statement is that d-connectivity is
 * necessary -- removing d-1 vertices must not disconnect the graph, since the
 * pieces could then rotate about the (d-2)-flat those vertices span -- and only
 * 1-cuts would be cheap to detect here (Tarjan lowlink); 2-cuts, which is what
 * 3D needs, are not. Beyond that, no purely combinatorial characterisation of
 * generic rigidity in R^3 exists at all (Laman's theorem does not extend past
 * 2D; the "double banana" satisfies every subgraph count and is still
 * flexible), so the rigorous test is numerical -- the rank of the rigidity
 * matrix, i.e. of the undamped JtJ apos_geom_refine() already assembles. That
 * was deliberately NOT implemented: it needs a singular-value tolerance, and
 * every candidate threshold is a guess until a real 32-anchor array has been
 * measured. These three conditions are what can be stated as facts today.
 *
 * So: `rigid` false is a definite verdict -- the framework really is flexible.
 * `rigid` true means "nothing here contradicts rigidity", which is weaker.
 * That asymmetry is why `redundant` gates rms_mm rather than replacing the
 * tape measure (see apos_gw_result_unverified()).
 *
 * Duplicate edges for one pair are counted ONCE. A repeated pair would
 * otherwise inflate n_edges and turn an isostatic mesh into an apparently
 * redundant one -- i.e. make rms_mm look trustworthy when it is not.
 * apos_table_symmetrise() never emits duplicates, but a caller building an
 * edge list by hand can. */
struct apos_rigidity {
	uint8_t  n_nodes;
	uint16_t n_edges;         /* DISTINCT structurally valid undirected pairs */
	uint8_t  n_components;    /* connected components over all n_nodes */
	uint8_t  min_degree;
	uint8_t  min_degree_node; /* which node had it; 0 when n_nodes == 0 */
	int16_t  free_params;     /* apos_geom_free_params(dim, n_nodes) */
	int16_t  spare_edges;     /* n_edges - free_params; may be negative */
	bool     connected;       /* n_components == 1 */
	bool     degree_ok;       /* min_degree >= apos_geom_dims(dim) */
	bool     rigid;           /* connected && degree_ok && spare_edges >= 0 */
	bool     redundant;       /* rigid && spare_edges > 0 -- rms_mm has
				   * something to disagree with */
};

/* Fill *out for the framework (e, n_edges) over n_nodes in `dim`. Edges naming
 * a node index at or past n_nodes, or joining a node to itself, are ignored
 * exactly as apos_geom_refine() ignores them.
 *
 * Returns 0, or -EINVAL on a NULL out, a NULL edge list with n_edges > 0, or
 * n_nodes outside 1..APOS_MAX_NODES. *out is zeroed first, so a rejected call
 * leaves an all-false (non-rigid) verdict rather than stale data.
 *
 * n_nodes below the gauge's minimum (APOS_MIN_NODES_2D / _3D) comes back
 * non-rigid via condition 2 rather than via a special case: the 2N-3 / 3N-6
 * count only means anything from N = d+1 upward, and a gauge that small is
 * already refused by apos_geom_gauge_valid(). */
int apos_geom_rigidity(const struct apos_edge *e, uint16_t n_edges,
		       uint8_t n_nodes, enum apos_geom_dim dim,
		       struct apos_rigidity *out);

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
 * is what acceptance is defined against.
 *
 * IMPORTANT: rms_m and worst_edge_m are only meaningful once the usable edge
 * count exceeds 3N-6 (N = usable nodes), AND the framework passes
 * apos_geom_rigidity() -- a flexible mesh can have edges to spare in a region
 * that is over-measured while another region is unconstrained, and the fit will
 * happily report a small rms_m for a shape it never determined. At N = 4 with a
 * full mesh, 6 edges exactly match the 6 free parameters -- an isostatic system
 * with no spare equation for a bad range to disagree with -- so LM finds an
 * exact re-embedding of whatever distances it is given and rms_m/worst_edge_m
 * come back identically zero regardless of the input's quality. From N = 5 to 7
 * there is enough redundancy for rms_m to read nonzero, but not always enough
 * to keep worst_i/worst_j pointing at the actual bad pair: least-squares
 * "masking" can let the fit shift two good nodes just enough to spread the
 * disagreement onto a different, merely-correlated edge instead. Both were
 * hit and confirmed while writing this module's own tests (see
 * tests/apos_geom/test_apos_geom.c); only a mesh with real edge redundancy
 * well past 3N-6 is a trustworthy witness. A four-anchor array solved in 3D
 * sits exactly in that under-determined regime; a 32-anchor sparse mesh
 * normally does not, which is what makes rms_m an acceptance signal at scale --
 * but only once apos_geom_rigidity() says the framework is rigid AND redundant.
 *
 * NOT REENTRANT: the LM working matrix and its trial result are function-local
 * `static` storage -- at APOS_MAX_PARAMS (90) the normal-equation matrix alone
 * is 90*90*4 ~= 32 kB of .bss, far past any stack this runs on -- so only one
 * call, across apos_geom_refine() and
 * apos_geom_solve() below which calls it, may be in flight at a time. Fine for
 * the single-threaded gateway loop this is written for; do not call either from
 * more than one thread.
 *
 * COST GROWS AS THE CUBE OF THE NODE COUNT. Each iteration runs a dense
 * Gaussian elimination over m.n_params = 3N-6 unknowns, so going from the
 * 4-anchor case (6 params) to a full 32-anchor 3D survey (90 params) is a
 * ~3400x increase in the elimination alone. APOS_GW_SOLVE_BUDGET_UUS was
 * estimated against the small case and has never been timed on hardware at
 * either size; read its comment in apos_gw.h before trusting it at 32
 * nodes. */
int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io);

/* apos_geom_seed() then apos_geom_refine(). The normal entry point. Not
 * reentrant -- see apos_geom_refine() above. */
int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		    const struct apos_gauge *g, struct apos_result *out);

#endif /* APOS_GEOM_H */

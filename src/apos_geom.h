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
 * count exceeds 3N-6 (N = usable nodes). At N = 4 with a full mesh, 6 edges
 * exactly match the 6 free parameters -- an isostatic system with no spare
 * equation for a bad range to disagree with -- so LM finds an exact
 * re-embedding of whatever distances it is given and rms_m/worst_edge_m come
 * back identically zero regardless of the input's quality. From N = 5 to 7
 * there is enough redundancy for rms_m to read nonzero, but not always enough
 * to keep worst_i/worst_j pointing at the actual bad pair: least-squares
 * "masking" can let the fit shift two good nodes just enough to spread the
 * disagreement onto a different, merely-correlated edge instead. Both were
 * hit and confirmed while writing this module's own tests (see
 * tests/apos_geom/test_apos_geom.c); only a mesh with real edge redundancy
 * well past 3N-6 is a trustworthy witness. The real deployment (4 ranging
 * slaves + 1 gateway) sits right in this under-determined regime, so an
 * acceptance check against rms_m alone can pass on a badly-ranged pair that a
 * richer mesh would have caught.
 *
 * NOT REENTRANT: the LM working matrices are function-local `static` storage
 * (too large for a comfortable stack frame), so only one call -- across
 * apos_geom_refine() and apos_geom_solve() below, which calls it -- may be in
 * flight at a time. Fine for the single-threaded gateway loop this is written
 * for; do not call either from more than one thread. */
int apos_geom_refine(const struct apos_edge *e, uint16_t n_edges,
		     const struct apos_gauge *g, struct apos_result *io);

/* apos_geom_seed() then apos_geom_refine(). The normal entry point. Not
 * reentrant -- see apos_geom_refine() above. */
int apos_geom_solve(const struct apos_edge *e, uint16_t n_edges, uint8_t n_nodes,
		    const struct apos_gauge *g, struct apos_result *out);

#endif /* APOS_GEOM_H */

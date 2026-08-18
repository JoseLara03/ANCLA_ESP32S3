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

/* M_PI is not in ISO C; gcc exposes it from math.h only with GNU extensions
 * enabled. Defined here so the host build does not depend on the -std= flag. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Below this a length is treated as degenerate rather than divided by. At
 * 1 um it is far under any real ranging resolution, so it only ever fires on
 * genuinely coincident or collinear inputs. */
#define EPS 1.0e-6f

int apos_geom_free_params(enum apos_geom_dim dim, uint8_t n_placed)
{
	return (dim == APOS_GEOM_2D) ? (2 * (int)n_placed - 3)
				      : (3 * (int)n_placed - 6);
}

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
	out->dim = g->dim;
	out->n_nodes = n_nodes;

	float d01 = edge_d(e, n_edges, g->origin, g->xaxis);
	float d02 = edge_d(e, n_edges, g->origin, g->plane);
	float d12 = edge_d(e, n_edges, g->xaxis, g->plane);

	/* All three <= 0, not just d01. edge_d() returns a negative sentinel for
	 * "not measured", but zero is a distinct and reachable failure: a
	 * genuinely zero-length gauge edge. A d02 of exactly 0 used to pass,
	 * after which py2 = -px*px is negative, py clamps to 0, and the `plane`
	 * node is placed COLLINEAR with origin and xaxis -- a silently
	 * degenerate gauge reported as a successful solve. */
	if (d01 <= 0.0f || d02 <= 0.0f || d12 <= 0.0f) {
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

	uint8_t nb[APOS_MAX_NODES];
	float rr[APOS_MAX_NODES];

	if (g->dim == APOS_GEOM_3D) {
		/* up: the only node whose reflection the operator resolved
		 * for us. */
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

	/* Everything else, best-determined first: re-scan after each placement
	 * so a node that gains a fourth neighbour is placed unambiguously
	 * rather than being guessed at from three. `tried[]` remembers a
	 * candidate that failed trilateration (collinear or non-intersecting
	 * neighbours) so it is excluded from the best_cnt competition on later
	 * scans, without stopping the loop from placing every other node. */
	bool tried[APOS_MAX_NODES] = {0};
	uint8_t min_needed = (g->dim == APOS_GEOM_2D) ? 2u : 3u;

	for (;;) {
		uint8_t best = APOS_MAX_NODES;
		uint8_t best_cnt = 0;

		for (uint8_t n = 0; n < n_nodes; n++) {
			if (out->node[n].state != APOS_NODE_UNPLACED || tried[n]) {
				continue;
			}
			uint8_t c = placed_neighbours(e, n_edges, out, n, nb,
						      rr, APOS_MAX_NODES);

			if (c >= min_needed && c > best_cnt) {
				best = n;
				best_cnt = c;
			}
		}
		if (best == APOS_MAX_NODES) {
			break;
		}
		if (!place_from_neighbours(e, n_edges, out, best)) {
			/* Geometrically unplaceable despite having the edges;
			 * leave it unplaced and stop retrying just this
			 * candidate -- other unplaced nodes may still be
			 * well-conditioned and must still get their turn. */
			out->node[best].state = APOS_NODE_UNPLACED;
			tried[best] = true;
			continue;
		}
	}

	return 0;
}

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
		/* No node, gauge or otherwise, ever gets a z slot in 2D mode
		 * -- this one condition is the entire 2D LM story. Every
		 * downstream consumer (cost(), the Jacobian assembly,
		 * apply_step()) only ever walks pmap, so z stays pinned at
		 * its seeded 0.0f with no other code needing to change. */
		if (n == g->plane || r->dim == APOS_GEOM_2D) {
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

/* Perpendicular distance of `plane` from the origin-xaxis line, divided by
 * the origin-xaxis baseline length. 2D only -- see apos_geom.h. Origin is
 * always (0,0,0) and xaxis always has y = z = 0 (neither is ever a free LM
 * parameter on those axes), so the general point-to-line formula collapses to
 * |plane.y| / xaxis.x. Returns 0 (indistinguishable from "perfectly spread")
 * on a degenerate zero-length baseline -- apos_geom_seed() already refuses a
 * zero origin-xaxis edge (d01 <= 0.0f), so that baseline is never actually
 * zero once a result exists, but a caller who hand-builds a struct
 * apos_result should not get a divide-by-zero for one that is. */
static float gauge_collinearity(const struct apos_result *r,
				const struct apos_gauge *g)
{
	if (r->dim != APOS_GEOM_2D) {
		return 0.0f;
	}

	float base = r->node[g->xaxis].x;

	if (fabsf(base) < EPS) {
		return 0.0f;
	}
	return fabsf(r->node[g->plane].y) / fabsf(base);
}

static void fill_diagnostics(const struct apos_edge *e, uint16_t n_edges,
			     struct apos_result *r, const struct apos_gauge *g)
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
	r->gauge_collinearity_ratio = gauge_collinearity(r, g);
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
	/* Stamped here independently of apos_geom_seed() (which also stamps
	 * it): a caller that builds a struct apos_result by hand and calls
	 * refine() directly, without seeding first, still gets the right
	 * dim from the one authoritative source, g->dim. Stamped only after
	 * the gauge validates, so a rejected call leaves io->dim exactly as
	 * the caller passed it in rather than mutating it on a -EINVAL path. */
	io->dim = g->dim;

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
		fill_diagnostics(e, n_edges, io, g);
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

		/* static, not automatic: declared [3*APOS_MAX_NODES][3*APOS_MAX_NODES]
		 * = 24x24 for indexing convenience, so the two of them are
		 * 2 * 24*24*4 = 4.6 kB of .bss (only the 18x18 corner --
		 * 3*APOS_MAX_NODES-6 free parameters -- is ever read or
		 * written), against a CONFIG_MAIN_STACK_SIZE of 4096. This
		 * runs only from the gateway loop, single-threaded, so there
		 * is no reentrancy. */
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

	fill_diagnostics(e, n_edges, io, g);
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

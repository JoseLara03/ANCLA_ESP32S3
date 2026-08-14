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
	 * rather than being guessed at from three. `tried[]` remembers a
	 * candidate that failed trilateration (collinear or non-intersecting
	 * neighbours) so it is excluded from the best_cnt competition on later
	 * scans, without stopping the loop from placing every other node. */
	bool tried[APOS_MAX_NODES] = {0};

	for (;;) {
		uint8_t best = APOS_MAX_NODES;
		uint8_t best_cnt = 0;

		for (uint8_t n = 0; n < n_nodes; n++) {
			if (out->node[n].state != APOS_NODE_UNPLACED || tried[n]) {
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

#include "apos_geom.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

/* A gauge edge of length exactly zero is degenerate, not merely unmeasured, and
 * must be refused just as a missing one is. The zero-length d02 case is the
 * dangerous one: it used to pass the guard, after which py clamps to 0 and the
 * `plane` node is placed collinear with origin and xaxis -- a degenerate frame
 * reported as a successful solve. A non-positive measured distance is reachable
 * on this hardware while the antenna delays are uncalibrated. */
static void test_zero_length_gauge_edge_is_enodata(void)
{
    for (int which = 0; which < 3; which++) {
        struct apos_edge e[APOS_MAX_EDGES];
        struct apos_result r;
        uint16_t n = build_full_mesh(e, 4);
        /* 0-1 is d01, 0-2 is d02, 1-2 is d12. */
        const uint8_t ii[3] = {0, 0, 1};
        const uint8_t jj[3] = {1, 2, 2};

        for (uint16_t k = 0; k < n; k++) {
            if (e[k].i == ii[which] && e[k].j == jj[which]) {
                e[k].d_m = 0.0f;
            }
        }

        CHECK(apos_geom_seed(e, n, 4, &g_ref, &r) == -ENODATA);
    }
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

/* Regression for the fix-round-1 finding: a geometrically unplaceable
 * candidate must not stop the placement loop from reaching an unrelated,
 * well-conditioned candidate. Node 4 has 4 edges to placed nodes but three
 * of them (spheres of radius 0.01 m centred metres apart) cannot mutually
 * intersect, so trilateration must fail for it. Node 5 also has 4 edges,
 * to the same four placed nodes, but with the true exact distances, so it
 * is fully determined. Both have neighbour count 4, so the candidate-
 * selection loop's first-found-wins tie-break picks node 4 (lower index)
 * first, exactly the ordering that exposed the bug: the old code's `break`
 * on node 4's failure left node 5 permanently UNPLACED even though nothing
 * about its own geometry was wrong. */
static void test_bad_candidate_does_not_block_good_candidate(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);
    const float p5[3] = {1.5f, 1.0f, 1.0f};

    e[n].i = 0; e[n].j = 4; e[n].d_m = 0.01f; e[n].sd_m = 0.001f; n++;
    e[n].i = 1; e[n].j = 4; e[n].d_m = 0.01f; e[n].sd_m = 0.001f; n++;
    e[n].i = 2; e[n].j = 4; e[n].d_m = 0.01f; e[n].sd_m = 0.001f; n++;
    e[n].i = 3; e[n].j = 4; e[n].d_m = 5.0f;  e[n].sd_m = 0.001f; n++;

    e[n].i = 0; e[n].j = 5; e[n].d_m = dist3(ref_xyz[0], p5); e[n].sd_m = 0.001f; n++;
    e[n].i = 1; e[n].j = 5; e[n].d_m = dist3(ref_xyz[1], p5); e[n].sd_m = 0.001f; n++;
    e[n].i = 2; e[n].j = 5; e[n].d_m = dist3(ref_xyz[2], p5); e[n].sd_m = 0.001f; n++;
    e[n].i = 3; e[n].j = 5; e[n].d_m = dist3(ref_xyz[3], p5); e[n].sd_m = 0.001f; n++;

    CHECK(apos_geom_seed(e, n, 6, &g_ref, &r) == 0);
    CHECK(r.node[4].state == APOS_NODE_UNPLACED);
    CHECK(r.node[5].state == APOS_NODE_PLACED);
    CLOSE(r.node[5].x, p5[0], 1e-3f);
    CLOSE(r.node[5].y, p5[1], 1e-3f);
    CLOSE(r.node[5].z, p5[2], 1e-3f);
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
/* NOTE (deviation from the brief): the brief's version of this test called
 * build_full_mesh(e, 4) -- 4 nodes, 6 edges. For n = 4, a full mesh has
 * exactly 3*4-6 = 6 free parameters for 6 edges: an isostatic system with
 * zero redundancy. LM therefore finds an *exact* re-embedding of the
 * perturbed distances (rms_m == 0, worst_edge_m == 0) rather than
 * distributing a residual -- there is no redundant equation for the
 * corrupted edge to disagree with. Verified with a standalone repro before
 * changing anything. A 5..7-node mesh reproduces the classic least-squares
 * "masking" effect instead: with too little redundancy, the fit can shift
 * node 1 or node 2 just enough to spread the disagreement onto a different,
 * merely-correlated edge, so worst_i/worst_j names the wrong pair. Only at
 * the full 8-node mesh (28 edges against 3*8-6 = 18 free parameters, 10
 * redundant equations) is node 1 and node 2's position pinned tightly enough
 * by other measurements that the corrupted (1,2) edge reliably carries the
 * largest residual. Confirmed by trying 4, 5, 7 and 8 node meshes; only 8
 * passed. */
static void test_worst_edge_names_the_bad_pair(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t k = 0;
    const float eight[8][3] = {
        {0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f},
        {0.0f, 4.0f, 0.0f}, {1.0f, 1.0f, 2.0f},
        {2.0f, 1.0f, 1.0f}, {-1.0f, 3.0f, 1.0f},
        {1.5f, -1.0f, 1.5f}, {0.5f, 2.0f, 2.5f},
    };

    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 8; j++) {
            e[k].i = i;
            e[k].j = j;
            e[k].d_m = dist3(eight[i], eight[j]);
            e[k].sd_m = 0.001f;
            k++;
        }
    }

    for (uint16_t m = 0; m < k; m++) {
        if ((e[m].i == 1 && e[m].j == 2) || (e[m].i == 2 && e[m].j == 1)) {
            e[m].d_m += 0.40f;
        }
    }

    CHECK(apos_geom_solve(e, k, 8, &g_ref, &r) == 0);
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
            if ((i == 2 && j == 4) || (i == 0 && j == 5)) {
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

    /* Deliberately wrong (disagrees with ref_xy's true origin-xaxis
     * distance of 3.0), yet still a legal triangle against the unchanged
     * 0-2/1-2 edges (4.0/5.0: 4+5=9 > 6), so with 0 spare edges the fit
     * must still report rms_m ~ 0 -- it has re-embedded whatever it was
     * given, not validated it.
     * DEVIATION FROM BRIEF: the brief used 10.0f here, but 10.0f together
     * with the unchanged 4.0/5.0 edges violates the triangle inequality
     * (4+5=9 < 10), which is a property of any three mutual distances
     * regardless of embedding dimension -- no solver, correct or not, can
     * re-embed it with near-zero residual. Verified by hand (closest
     * achievable unweighted sum-of-squares over the 0-2/1-2 pair is 0.5,
     * giving rms_m ~ 0.41, not the LM implementation failing to converge)
     * and by a standalone repro before changing the value. 6.0f preserves
     * the test's intent -- a wrong-but-isostatic edge -- while keeping the
     * triangle realizable, and reproduces rms_m == 0.0 exactly. */
    e[0].d_m = 6.0f; /* origin-xaxis, was 3.0 */

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

/* The 2D analogue of test_planarity_is_small_for_a_coplanar_array(): a gauge
 * triangle with `plane` almost on the origin-xaxis line carries almost no
 * information about which way +y points, so a range error on an unrelated
 * edge can swing every placed node's y sharply with nothing else flagging it
 * (rms_m stays ~0 -- this is the isostatic 3-node case, same as
 * test_2d_three_node_survey_is_degenerate()). gauge_collinearity_ratio is the
 * diagnostic that has to say so. */
static void test_gauge_collinearity_is_small_for_a_near_collinear_triangle(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    const float thin[3][2] = {
        {0.0f, 0.0f}, {3.0f, 0.0f}, {3.0f, 0.05f}, /* plane 50 mm off the line */
    };
    uint16_t k = 0;

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < 3; j++) {
            e[k].i = i;
            e[k].j = j;
            e[k].d_m = dist2(thin[i], thin[j]);
            e[k].sd_m = 0.001f;
            k++;
        }
    }

    CHECK(apos_geom_solve(e, k, 3, &g_ref_2d, &r) == 0);
    CHECK(r.gauge_collinearity_ratio < 0.05f);
}

/* A well-spread triangle (the existing g_ref_2d/ref_xy layout, a 3-4-5 right
 * triangle) must NOT be flagged, or the diagnostic is useless. */
static void test_gauge_collinearity_is_large_for_a_well_spread_triangle(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh_2d(e, 3);

    CHECK(apos_geom_solve(e, n, 3, &g_ref_2d, &r) == 0);
    CHECK(r.gauge_collinearity_ratio > 0.5f);
}

/* 3D mode does not compute this metric at all -- planarity_m is its analogue
 * there. */
static void test_gauge_collinearity_is_zero_in_3d_mode(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_solve(e, n, 4, &g_ref, &r) == 0);
    CHECK(r.gauge_collinearity_ratio == 0.0f);
}

static void test_free_params_matches_each_dimensionality(void)
{
    /* 3D: 3N-6. 2D: 2N-3. */
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 4) == 6);
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 5) == 9);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 3) == 3);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 4) == 5);
}

/* apos_geom_dims() must NOT be `(int)dim`: APOS_GEOM_3D is the zero value and
 * APOS_GEOM_2D is 1, so the enum reads exactly backwards as a dimension count
 * and every rigidity bound derived from it would be wrong in a way that still
 * looked plausible. */
static void test_dims_is_not_the_enum_value(void)
{
    CHECK(apos_geom_dims(APOS_GEOM_2D) == 2);
    CHECK(apos_geom_dims(APOS_GEOM_3D) == 3);
    CHECK((int)APOS_GEOM_3D == 0);
    CHECK((int)APOS_GEOM_2D == 1);
}

/* ---- Rigidity ---- */

/* Add an undirected edge with an arbitrary distance. The rigidity check reads
 * only the graph, never a length, so these helpers do not need real geometry. */
static uint16_t add_edge(struct apos_edge *e, uint16_t n, uint8_t i, uint8_t j)
{
    e[n].i = i;
    e[n].j = j;
    e[n].d_m = 1.0f;
    e[n].sd_m = 0.001f;
    return (uint16_t)(n + 1u);
}

/* Every pair within [lo, hi). */
static uint16_t add_clique(struct apos_edge *e, uint16_t n, uint8_t lo, uint8_t hi)
{
    for (uint8_t i = lo; i < hi; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < hi; j++) {
            n = add_edge(e, n, i, j);
        }
    }
    return n;
}

/* The three conditions must be independently observable, or a caller cannot
 * tell an operator WHICH one to fix. Start from the case everything else is
 * measured against: a 4-node 3D full mesh, rigid and exactly isostatic. */
static void test_rigidity_accepts_the_isostatic_four_node_mesh(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_rigidity(e, n, 4, APOS_GEOM_3D, &r) == 0);
    CHECK(r.n_edges == 6);
    CHECK(r.n_components == 1);
    CHECK(r.connected);
    CHECK(r.min_degree == 3);
    CHECK(r.degree_ok);
    CHECK(r.free_params == 6);
    CHECK(r.spare_edges == 0);
    /* Rigid, but with nothing spare -- so rms_m proves nothing here. That
     * pairing is exactly what apos_gw_result_unverified() reports. */
    CHECK(r.rigid);
    CHECK(!r.redundant);
}

/* FAILURE MODE 1: disconnected. Deliberately built so the EDGE COUNT PASSES,
 * proving connectivity is a separate gate rather than something the DOF count
 * happens to catch: two 5-cliques in 2D are 20 edges against 2*10-3 = 17 free
 * parameters, i.e. 3 spare -- and still two pieces that can slide and rotate
 * freely relative to each other. */
static void test_rigidity_rejects_a_disconnected_mesh(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = add_clique(e, 0, 0, 5);

    n = add_clique(e, n, 5, 10);
    CHECK(n == 20);

    CHECK(apos_geom_rigidity(e, n, 10, APOS_GEOM_2D, &r) == 0);
    CHECK(r.n_components == 2);
    CHECK(!r.connected);
    /* The count and the degree are both fine -- only connectivity is not. */
    CHECK(r.spare_edges == 3);
    CHECK(r.degree_ok);
    CHECK(r.min_degree == 4);
    CHECK(!r.rigid);
    CHECK(!r.redundant);

    /* Same story in 3D, where the count needs bigger pieces to pass: two
     * 7-cliques are 42 edges against 3*14-6 = 36. */
    n = add_clique(e, 0, 0, 7);
    n = add_clique(e, n, 7, 14);
    CHECK(n == 42);
    CHECK(apos_geom_rigidity(e, n, 14, APOS_GEOM_3D, &r) == 0);
    CHECK(r.n_components == 2);
    CHECK(r.spare_edges == 6);
    CHECK(r.degree_ok);
    CHECK(!r.rigid);
}

/* FAILURE MODE 2: flexible for want of edges. A 12-node ring in 2D is
 * connected, has min degree 2 (enough to pin a vertex), and is still a
 * mechanism -- 12 edges against 2*12-3 = 21 free parameters. It must be FLAGGED
 * rather than accepted, because LM will still return coordinates for it. */
static void test_rigidity_rejects_an_under_constrained_ring(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = 0;

    for (uint8_t k = 0; k < 12; k++) {
        n = add_edge(e, n, k, (uint8_t)((k + 1u) % 12u));
    }
    CHECK(n == 12);

    CHECK(apos_geom_rigidity(e, n, 12, APOS_GEOM_2D, &r) == 0);
    CHECK(r.connected);          /* not this one */
    CHECK(r.degree_ok);          /* nor this one */
    CHECK(r.min_degree == 2);
    CHECK(r.free_params == 21);
    CHECK(r.spare_edges == -9);  /* THIS one */
    CHECK(!r.rigid);
    CHECK(!r.redundant);
}

/* FAILURE MODE 3: a node with fewer edges than the solve has dimensions. Built
 * so the EDGE COUNT PASSES, again proving the gates are independent: a 5-node
 * 2D clique plus a 6th node hanging off a single edge is 11 edges against
 * 2*6-3 = 9, i.e. 2 spare, connected -- and that 6th node swings on a circle. */
static void test_rigidity_rejects_an_under_degree_node(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = add_clique(e, 0, 0, 5);

    n = add_edge(e, n, 0, 5);
    CHECK(n == 11);

    CHECK(apos_geom_rigidity(e, n, 6, APOS_GEOM_2D, &r) == 0);
    CHECK(r.connected);
    CHECK(r.spare_edges == 2);
    CHECK(r.min_degree == 1);
    CHECK(r.min_degree_node == 5);
    CHECK(!r.degree_ok);
    CHECK(!r.rigid);
    CHECK(!r.redundant);

    /* Degree 2 is enough in 2D and NOT enough in 3D -- the same graph gives
     * opposite verdicts, which is the whole reason apos_geom_dims() exists. */
    n = add_edge(e, n, 1, 5);
    CHECK(apos_geom_rigidity(e, n, 6, APOS_GEOM_2D, &r) == 0);
    CHECK(r.min_degree == 2);
    CHECK(r.degree_ok);
    CHECK(apos_geom_rigidity(e, n, 6, APOS_GEOM_3D, &r) == 0);
    CHECK(r.min_degree == 2);
    CHECK(!r.degree_ok);
}

/* A repeated pair must count ONCE. Otherwise a duplicated edge inflates
 * n_edges, turns an isostatic mesh into an apparently redundant one, and makes
 * rms_mm look trustworthy when nothing has changed about the measurements. */
static void test_rigidity_counts_a_duplicate_edge_once(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = build_full_mesh(e, 4);

    /* The 0-1 edge again, and again with the endpoints swapped. */
    n = add_edge(e, n, 0, 1);
    n = add_edge(e, n, 1, 0);

    CHECK(apos_geom_rigidity(e, n, 4, APOS_GEOM_3D, &r) == 0);
    CHECK(r.n_edges == 6);       /* not 8 */
    CHECK(r.spare_edges == 0);
    CHECK(!r.redundant);
}

/* Edges naming a node past n_nodes, or joining a node to itself, are ignored --
 * exactly as apos_geom_refine() ignores them, so the graph the check reasons
 * about is the graph the fit uses. An isolated node then shows up as a second
 * component, which is the honest report: it cannot be placed. */
static void test_rigidity_ignores_structurally_invalid_edges(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = build_full_mesh(e, 4);

    n = add_edge(e, n, 2, 2);   /* self-loop */
    n = add_edge(e, n, 3, 9);   /* node 9 is past n_nodes = 5 */

    CHECK(apos_geom_rigidity(e, n, 5, APOS_GEOM_3D, &r) == 0);
    CHECK(r.n_edges == 6);
    CHECK(r.n_components == 2); /* node 4 is isolated */
    CHECK(!r.connected);
    CHECK(r.min_degree == 0);
    CHECK(r.min_degree_node == 4);
    CHECK(!r.rigid);
}

static void test_rigidity_rejects_bad_arguments(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_rigidity r;
    uint16_t n = build_full_mesh(e, 4);

    CHECK(apos_geom_rigidity(e, n, 4, APOS_GEOM_3D, NULL) == -EINVAL);
    CHECK(apos_geom_rigidity(NULL, n, 4, APOS_GEOM_3D, &r) == -EINVAL);
    CHECK(apos_geom_rigidity(e, n, 0, APOS_GEOM_3D, &r) == -EINVAL);
    CHECK(apos_geom_rigidity(e, n, APOS_MAX_NODES + 1, APOS_GEOM_3D, &r)
          == -EINVAL);
    /* A rejected call must leave a NON-rigid verdict, never stale data: a
     * caller that ignores the return code has to fail safe. */
    memset(&r, 0xFF, sizeof(r));
    CHECK(apos_geom_rigidity(e, n, 0, APOS_GEOM_3D, &r) == -EINVAL);
    CHECK(!r.rigid);
    CHECK(!r.redundant);
    CHECK(!r.connected);
    /* An empty edge list with a NULL pointer is fine -- nothing to read. */
    CHECK(apos_geom_rigidity(NULL, 0, 4, APOS_GEOM_3D, &r) == 0);
    CHECK(r.n_edges == 0);
    CHECK(!r.rigid);
}

/* The point of the whole exercise: at 32 anchors a SPARSE mesh has real
 * redundancy, so rms_mm stops being vacuous. A 12-node sparse 3D layout --
 * every pair within a 16 m radius of a ~20 m x 20 m site, so well under the
 * 66-edge full mesh -- must solve, place every node unambiguously, and come
 * back rigid AND redundant. This is the 4-anchor isostatic floor being left
 * behind, and it is the case the acceptance thresholds finally mean something
 * on. */
static const float twelve[12][3] = {
    { 0.0f,  0.0f, 0.0f},  /* origin */
    {10.0f,  0.0f, 0.0f},  /* xaxis  */
    { 0.0f, 10.0f, 0.0f},  /* plane  */
    { 3.0f,  3.0f, 4.0f},  /* up     */
    {10.0f, 10.0f, 0.0f},
    { 5.0f,  5.0f, 0.0f},
    {20.0f,  0.0f, 1.0f},
    {20.0f, 10.0f, 1.0f},
    {15.0f,  5.0f, 4.0f},
    {10.0f, 20.0f, 0.0f},
    { 0.0f, 20.0f, 1.0f},
    { 5.0f, 15.0f, 4.0f},
};

/* Edges only between nodes within `radius` of each other -- the shape a real
 * 100 m site produces, where distant pairs simply never range. */
static uint16_t build_radius_mesh(struct apos_edge *out, uint8_t n, float radius,
                                  float sd)
{
    uint16_t k = 0;

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
            if (dist3(twelve[i], twelve[j]) > radius) {
                continue;
            }
            out[k].i = i;
            out[k].j = j;
            out[k].d_m = dist3(twelve[i], twelve[j]);
            out[k].sd_m = sd;
            k++;
        }
    }
    return k;
}

static void test_sparse_twelve_node_mesh_solves(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result r;
    struct apos_rigidity rg;
    uint16_t n = build_radius_mesh(e, 12, 16.0f, 0.010f);

    /* Genuinely sparse: fewer edges than the 66 a full 12-node mesh has, and
     * more than the 30 free parameters 3*12-6 leaves. */
    CHECK(n < 66);
    CHECK(n > 30);

    CHECK(apos_geom_rigidity(e, n, 12, APOS_GEOM_3D, &rg) == 0);
    CHECK(rg.connected);
    CHECK(rg.degree_ok);
    CHECK(rg.free_params == 30);
    CHECK(rg.spare_edges > 0);
    CHECK(rg.rigid);
    /* THE claim: a sparse mesh at this size is redundant, so rms_mm carries
     * information -- unlike the 4-node isostatic case above. */
    CHECK(rg.redundant);

    CHECK(apos_geom_solve(e, n, 12, &g_ref, &r) == 0);
    CHECK(r.n_placed == 12);
    CHECK(r.n_ambiguous == 0);
    CHECK(r.rms_m < 0.01f);
    for (int i = 0; i < 12; i++) {
        CHECK(r.node[i].state == APOS_NODE_PLACED);
        CLOSE(r.node[i].x, twelve[i][0], 0.02f);
        CLOSE(r.node[i].y, twelve[i][1], 0.02f);
        CLOSE(r.node[i].z, twelve[i][2], 0.02f);
    }
}

/* And rms_m on that same sparse mesh must actually MOVE when a range is wrong
 * -- otherwise "redundant" would be a label with nothing behind it. This is the
 * property that is identically false on a 4-node 3D full mesh. */
static void test_sparse_twelve_node_mesh_reports_a_bad_edge(void)
{
    struct apos_edge e[APOS_MAX_EDGES];
    struct apos_result clean, bad;
    uint16_t n = build_radius_mesh(e, 12, 16.0f, 0.010f);

    CHECK(apos_geom_solve(e, n, 12, &g_ref, &clean) == 0);
    CHECK(clean.rms_m < 0.01f);

    /* Corrupt one edge between two non-gauge nodes by 300 mm. */
    for (uint16_t k = 0; k < n; k++) {
        if (e[k].i == 4 && e[k].j == 5) {
            e[k].d_m += 0.300f;
        }
    }

    CHECK(apos_geom_solve(e, n, 12, &g_ref, &bad) == 0);
    CHECK(bad.rms_m > clean.rms_m);
    CHECK(bad.rms_m > 0.02f);
    CHECK(bad.worst_edge_m > 0.05f);
    /* With 13 spare edges the fit cannot mask the culprit onto a neighbour --
     * the "least-squares masking" the existing 5..7-node note describes needs
     * a mesh close to isostatic. Observed: rms 36.5 mm, worst 191 mm on
     * exactly the corrupted pair. */
    CHECK((bad.worst_i == 4 && bad.worst_j == 5) ||
          (bad.worst_i == 5 && bad.worst_j == 4));
}

int main(void)
{
    test_gauge_requires_four_distinct_nodes();
    test_2d_gauge_needs_only_three_distinct_nodes();
    test_2d_seed_reproduces_the_exact_layout();
    test_2d_fourth_neighbour_resolves_the_mirror();
    test_2d_two_neighbours_is_flagged_ambiguous();
    test_2d_solve_refines_a_five_node_layout();
    test_2d_three_node_survey_is_degenerate();
    test_gauge_collinearity_is_small_for_a_near_collinear_triangle();
    test_gauge_collinearity_is_large_for_a_well_spread_triangle();
    test_gauge_collinearity_is_zero_in_3d_mode();
    test_refine_stamps_dim_without_seed();
    test_free_params_matches_each_dimensionality();
    test_dims_is_not_the_enum_value();
    test_rigidity_accepts_the_isostatic_four_node_mesh();
    test_rigidity_rejects_a_disconnected_mesh();
    test_rigidity_rejects_an_under_constrained_ring();
    test_rigidity_rejects_an_under_degree_node();
    test_rigidity_counts_a_duplicate_edge_once();
    test_rigidity_ignores_structurally_invalid_edges();
    test_rigidity_rejects_bad_arguments();
    test_sparse_twelve_node_mesh_solves();
    test_sparse_twelve_node_mesh_reports_a_bad_edge();
    test_seed_reproduces_the_exact_layout();
    test_gauge_constraints_hold_exactly();
    test_node_with_two_edges_is_unplaced_not_an_error();
    test_node_with_three_edges_is_flagged_ambiguous();
    test_fourth_edge_resolves_the_mirror();
    test_missing_gauge_edge_is_enodata();
    test_zero_length_gauge_edge_is_enodata();
    test_bad_candidate_does_not_block_good_candidate();
    test_zoff_shifts_only_placed_nodes();
    test_rejects_bad_arguments();
    test_refine_leaves_an_exact_solution_alone();
    test_refine_absorbs_realistic_noise();
    test_refine_preserves_the_gauge_exactly();
    test_worst_edge_names_the_bad_pair();
    test_planarity_is_small_for_a_coplanar_array();
    test_planarity_is_large_for_a_3d_array();
    test_solves_a_sparse_mesh_with_holes();
    test_unplaced_node_does_not_break_the_fit();

    if (g_fail) {
        printf("%d CHECK(s) FAILED\n", g_fail);
        return 1;
    }
    printf("PASSED\n");
    return 0;
}

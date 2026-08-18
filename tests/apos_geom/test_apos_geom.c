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

static void test_free_params_matches_each_dimensionality(void)
{
    /* 3D: 3N-6. 2D: 2N-3. */
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 4) == 6);
    CHECK(apos_geom_free_params(APOS_GEOM_3D, 5) == 9);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 3) == 3);
    CHECK(apos_geom_free_params(APOS_GEOM_2D, 4) == 5);
}

int main(void)
{
    test_gauge_requires_four_distinct_nodes();
    test_free_params_matches_each_dimensionality();
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

#pragma once

#include <bvh/v2/bvh.h>
#include <bvh/v2/node.h>
#include <bvh/v2/vec.h>
#include <bvh/v2/bbox.h>
#include <bvh/v2/tri.h>
#include <bvh/v2/default_builder.h>
#include <bvh/v2/utils.h>

#include <geogram/basic/geometry.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>

#include <algorithm>
#include <vector>
#include <limits>

namespace floatTetWild {
namespace bvh_queries {

using Scalar = double;
using Vec3   = bvh::v2::Vec<Scalar, 3>;
using BBox3  = bvh::v2::BBox<Scalar, 3>;
using Tri3   = bvh::v2::Tri<Scalar, 3>;
using Node3  = bvh::v2::Node<Scalar, 3>;
using Bvh3   = bvh::v2::Bvh<Node3>;

constexpr std::size_t kMaxStackDepth = 64;

// Optimized branchless scalar AABB distance check using robust_max
inline Scalar point_box_sq_dist(const Vec3& p, const BBox3& box) {
    Scalar d = 0;
    Scalar d_min, d_max, dist;

    d_min = box.min[0] - p[0];
    d_max = p[0] - box.max[0];
    dist  = bvh::v2::robust_max(Scalar(0), bvh::v2::robust_max(d_min, d_max));
    d += dist * dist;

    d_min = box.min[1] - p[1];
    d_max = p[1] - box.max[1];
    dist  = bvh::v2::robust_max(Scalar(0), bvh::v2::robust_max(d_min, d_max));
    d += dist * dist;

    d_min = box.min[2] - p[2];
    d_max = p[2] - box.max[2];
    dist  = bvh::v2::robust_max(Scalar(0), bvh::v2::robust_max(d_min, d_max));
    d += dist * dist;

    return d;
}

inline GEO::vec3 to_geo(const Vec3& v) { return GEO::vec3(v[0], v[1], v[2]); }

inline Scalar point_tri_sq_dist(const GEO::vec3& p, const Tri3& t, GEO::vec3& out_nearest) {
    GEO::vec3 a(t.p0[0], t.p0[1], t.p0[2]);
    GEO::vec3 b(t.p1[0], t.p1[1], t.p1[2]);
    GEO::vec3 c(t.p2[0], t.p2[1], t.p2[2]);
    double l1, l2, l3;
    return GEO::Geom::point_triangle_squared_distance<GEO::vec3>(p, a, b, c, out_nearest, l1, l2, l3);
}

// Construction returns the BVH and the leaf-ordered triangles
inline Bvh3 build_from_geo_mesh(const GEO::Mesh& mesh, std::vector<Tri3>& out_tris) {
    const std::size_t n = mesh.facets.nb();
    std::vector<BBox3> bboxes(n);
    std::vector<Vec3>  centers(n);
    std::vector<Tri3>  tris_orig(n);

    for (std::size_t f = 0; f < n; ++f) {
        GEO::index_t c0 = mesh.facets.corners_begin(f);
        const GEO::vec3& a = GEO::Geom::mesh_vertex(mesh, mesh.facet_corners.vertex(c0));
        const GEO::vec3& b = GEO::Geom::mesh_vertex(mesh, mesh.facet_corners.vertex(c0 + 1));
        const GEO::vec3& d = GEO::Geom::mesh_vertex(mesh, mesh.facet_corners.vertex(c0 + 2));
        tris_orig[f] = Tri3(Vec3{a.x, a.y, a.z}, Vec3{b.x, b.y, b.z}, Vec3{d.x, d.y, d.z});
        bboxes[f]    = tris_orig[f].get_bbox();
        centers[f]   = tris_orig[f].get_center();
    }

    typename bvh::v2::DefaultBuilder<Node3>::Config config;
    config.quality = bvh::v2::DefaultBuilder<Node3>::Quality::High;
    config.max_leaf_size = 12;
    Bvh3 bvh = bvh::v2::DefaultBuilder<Node3>::build(bboxes, centers, config);

    out_tris.resize(n);
    for (std::size_t i = 0; i < n; ++i)
        out_tris[i] = tris_orig[bvh.prim_ids[i]];

    return bvh;
}

struct ClosestPointResult {
    std::size_t prim_id;
    Scalar      sq_dist;
    GEO::vec3   nearest_point;
};

struct StackNode {
    std::uint32_t id;
    Scalar sq_dist;
};

inline ClosestPointResult closest_point(
    const Bvh3& bvh, const std::vector<Tri3>& tris, const GEO::vec3& query_geo,
    Scalar init_sq_dist = std::numeric_limits<Scalar>::max(),
    std::size_t init_prim_id = 0, const GEO::vec3& init_nearest = GEO::vec3(0, 0, 0)) 
{
    Vec3 query{query_geo.x, query_geo.y, query_geo.z};
    ClosestPointResult best{init_prim_id, init_sq_dist, init_nearest};

    if (bvh.nodes.empty()) return best;

    StackNode stack[kMaxStackDepth];
    int sp = 0;
    stack[sp++] = {0, point_box_sq_dist(query, bvh.nodes[0].get_bbox())};

    while (sp > 0) {
        auto [node_idx, node_sq_dist] = stack[--sp];
        if (node_sq_dist >= best.sq_dist) continue;

        const Node3& node = bvh.nodes[node_idx];
        if (node.is_leaf()) {
            const std::size_t first = node.index.first_id();
            const std::size_t end = first + node.index.prim_count();
            for (std::size_t i = first; i < end; ++i) {
                GEO::vec3 nearest;
                Scalar d2 = point_tri_sq_dist(query_geo, tris[i], nearest);
                if (d2 < best.sq_dist) {
                    best.sq_dist = d2;
                    best.prim_id = i;
                    best.nearest_point = nearest;
                }
            }
            continue;
        }

        const std::size_t left_idx = node.index.first_id();
        const std::size_t right_idx = left_idx + 1;
        Scalar dl = point_box_sq_dist(query, bvh.nodes[left_idx].get_bbox());
        Scalar dr = point_box_sq_dist(query, bvh.nodes[right_idx].get_bbox());

        if (dl < dr) {
            if (dr < best.sq_dist) stack[sp++] = {static_cast<std::uint32_t>(right_idx), dr};
            if (dl < best.sq_dist) stack[sp++] = {static_cast<std::uint32_t>(left_idx), dl};
        } else {
            if (dl < best.sq_dist) stack[sp++] = {static_cast<std::uint32_t>(left_idx), dl};
            if (dr < best.sq_dist) stack[sp++] = {static_cast<std::uint32_t>(right_idx), dr};
        }
    }
    return best;
}

inline bool point_in_envelope(const Bvh3& bvh, const std::vector<Tri3>& tris, const GEO::vec3& query_geo, Scalar sq_eps) {
    if (bvh.nodes.empty()) return false;
    Vec3 query{query_geo.x, query_geo.y, query_geo.z};
    if (point_box_sq_dist(query, bvh.nodes[0].get_bbox()) > sq_eps) return false;

    std::uint32_t stack[kMaxStackDepth];
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0) {
        const Node3& node = bvh.nodes[stack[--sp]];
        if (node.is_leaf()) {
            const std::size_t first = node.index.first_id();
            const std::size_t end = first + node.index.prim_count();
            for (std::size_t i = first; i < end; ++i) {
                GEO::vec3 nearest;
                if (point_tri_sq_dist(query_geo, tris[i], nearest) <= sq_eps) return true;
            }
            continue;
        }
        const std::size_t left_idx = node.index.first_id();
        const std::size_t right_idx = left_idx + 1;
        if (point_box_sq_dist(query, bvh.nodes[right_idx].get_bbox()) <= sq_eps) stack[sp++] = static_cast<std::uint32_t>(right_idx);
        if (point_box_sq_dist(query, bvh.nodes[left_idx].get_bbox()) <= sq_eps) stack[sp++] = static_cast<std::uint32_t>(left_idx);
    }
    return false;
}

inline bool point_in_envelope_with_hit(const Bvh3& bvh, const std::vector<Tri3>& tris, const GEO::vec3& query_geo, Scalar sq_eps,
                                       std::size_t& out_prim_id, Scalar& out_sq_dist, GEO::vec3& out_nearest) {
    if (bvh.nodes.empty()) return false;
    Vec3 query{query_geo.x, query_geo.y, query_geo.z};
    if (point_box_sq_dist(query, bvh.nodes[0].get_bbox()) > sq_eps) return false;

    std::uint32_t stack[kMaxStackDepth];
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0) {
        const Node3& node = bvh.nodes[stack[--sp]];
        if (node.is_leaf()) {
            const std::size_t first = node.index.first_id();
            const std::size_t end = first + node.index.prim_count();
            for (std::size_t i = first; i < end; ++i) {
                GEO::vec3 nearest;
                Scalar d2 = point_tri_sq_dist(query_geo, tris[i], nearest);
                if (d2 <= sq_eps) {
                    out_prim_id = i; out_sq_dist = d2; out_nearest = nearest;
                    return true;
                }
            }
            continue;
        }
        const std::size_t left_idx = node.index.first_id();
        const std::size_t right_idx = left_idx + 1;
        if (point_box_sq_dist(query, bvh.nodes[right_idx].get_bbox()) <= sq_eps) stack[sp++] = static_cast<std::uint32_t>(right_idx);
        if (point_box_sq_dist(query, bvh.nodes[left_idx].get_bbox()) <= sq_eps) stack[sp++] = static_cast<std::uint32_t>(left_idx);
    }
    return false;
}

} // namespace bvh_queries
} // namespace floatTetWild
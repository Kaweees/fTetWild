#pragma once

#include <floattetwild/Mesh.hpp>
#include <floattetwild/bvh_distance_queries.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>

#include <limits>

//#define NEW_ENVELOPE //fortest

#ifdef NEW_ENVELOPE
#include <fastenvelope/FastEnvelope.h>
#endif

namespace floatTetWild {

class AABBWrapper {
    // Co-located BVH and triangle caches. The two parallel triangle arrays:
    //   - tris_leaf: in BVH-leaf order (BVH leaves index into this).
    //   - tris_mesh: in original GEO::Mesh facet-id order, so the prev_facet
    //                hint shortcut becomes one indexed read instead of six
    //                GEO::Mesh corner-pointer chases.
    struct BvhData {
        bvh_queries::Bvh3 bvh;
        std::vector<bvh_queries::Tri3> tris_leaf;
        std::vector<bvh_queries::Tri3> tris_mesh;

        void init(const GEO::Mesh& mesh) {
            bvh = bvh_queries::build_from_geo_mesh(mesh, tris_leaf);
            const std::size_t n = mesh.facets.nb();
            tris_mesh.resize(n);
            for (std::size_t i = 0; i < n; ++i)
                tris_mesh[bvh.prim_ids[i]] = tris_leaf[i];
        }

        // Exact squared distance from p to the triangle with the given
        // GEO::Mesh facet id, using the cached vertices (no GEO::Mesh access).
        inline Scalar hint_sq_dist(const GEO::vec3& p, GEO::index_t f,
                                   GEO::vec3& out_nearest) const {
            return bvh_queries::point_tri_sq_dist(p, tris_mesh[f], out_nearest);
        }
    };

    BvhData sf_data;
    BvhData b_data;
    BvhData tmp_data;

public:
    GEO::Mesh b_mesh;
    GEO::Mesh tmp_b_mesh;
    const GEO::Mesh& sf_mesh;

    inline Scalar get_sf_diag() const { return GEO::bbox_diagonal(sf_mesh); }

    AABBWrapper(const GEO::Mesh& sf_mesh) : sf_mesh(sf_mesh) {
        sf_data.init(sf_mesh);
    }

#ifdef NEW_ENVELOPE
    fastEnvelope::FastEnvelope b_tree_exact;
    fastEnvelope::FastEnvelope tmp_b_tree_exact;
    fastEnvelope::FastEnvelope sf_tree_exact;
    fastEnvelope::FastEnvelope sf_tree_exact_simplify;

    inline void init_sf_tree(const std::vector<Vector3>& vs, const std::vector<Vector3i>& fs, double eps) {
        sf_tree_exact.init(vs, fs, eps);
        sf_tree_exact_simplify.init(vs, fs, 0.8 * eps);
    }
    inline void init_sf_tree(const std::vector<Vector3>& vs, const std::vector<Vector3i>& fs,
                             std::vector<double>& eps, double bbox_diag_length) {
        for (auto& e : eps) e *= bbox_diag_length;
        sf_tree_exact.init(vs, fs, eps);
        std::vector<double> eps_simplify = eps;
        for (auto& e : eps_simplify) e *= 0.8;
        sf_tree_exact_simplify.init(vs, fs, eps_simplify);
    }
#endif

    void init_b_mesh_and_tree(const std::vector<Vector3>& input_vertices,
                              const std::vector<Vector3i>& input_faces, Mesh& mesh);

    void init_tmp_b_mesh_and_tree(const std::vector<Vector3>& input_vertices,
                                  const std::vector<Vector3i>& input_faces,
                                  const std::vector<std::array<int, 2>>& b_edges1,
                                  const Mesh& mesh,
                                  const std::vector<std::array<int, 2>>& b_edges2);

    //// projection
    inline Scalar project_to_sf(Vector3& p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        auto r = bvh_queries::closest_point(sf_data.bvh, sf_data.tris_leaf, geo_p);
        p[0] = r.nearest_point.x;
        p[1] = r.nearest_point.y;
        p[2] = r.nearest_point.z;
        return r.sq_dist;
    }

    inline Scalar project_to_b(Vector3& p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        auto r = bvh_queries::closest_point(b_data.bvh, b_data.tris_leaf, geo_p);
        p[0] = r.nearest_point.x;
        p[1] = r.nearest_point.y;
        p[2] = r.nearest_point.z;
        return r.sq_dist;
    }

    inline Scalar project_to_tmp_b(Vector3& p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        auto r = bvh_queries::closest_point(tmp_data.bvh, tmp_data.tris_leaf, geo_p);
        p[0] = r.nearest_point.x;
        p[1] = r.nearest_point.y;
        p[2] = r.nearest_point.z;
        return r.sq_dist;
    }

    inline int get_nearest_face_sf(const Vector3& p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        auto r = bvh_queries::closest_point(sf_data.bvh, sf_data.tris_leaf, geo_p);
        return static_cast<int>(sf_data.bvh.prim_ids[r.prim_id]);
    }

    inline Scalar get_sq_dist_to_sf(const Vector3& p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        return bvh_queries::closest_point(sf_data.bvh, sf_data.tris_leaf, geo_p).sq_dist;
    }

    //// envelope check - triangle (vector of sample points)
    inline bool is_out_sf_envelope(const std::vector<GEO::vec3>& ps, const Scalar eps_2,
                                   GEO::index_t prev_facet = GEO::NO_FACET) const {
        return is_out_envelope_impl(sf_data, ps, eps_2, prev_facet);
    }

    inline bool is_out_b_envelope(const std::vector<GEO::vec3>& ps, const Scalar eps_2,
                                  GEO::index_t prev_facet = GEO::NO_FACET) const {
        return is_out_envelope_impl(b_data, ps, eps_2, prev_facet);
    }

    inline bool is_out_tmp_b_envelope(const std::vector<GEO::vec3>& ps, const Scalar eps_2,
                                      GEO::index_t prev_facet = GEO::NO_FACET) const {
        return is_out_envelope_impl(tmp_data, ps, eps_2, prev_facet);
    }

#ifdef NEW_ENVELOPE
    inline bool is_out_sf_envelope_exact(const std::array<Vector3, 3>& triangle) const {
        return sf_tree_exact.is_outside(triangle);
    }
    inline bool is_out_sf_envelope_exact_simplify(const std::array<Vector3, 3>& triangle) const {
        return sf_tree_exact_simplify.is_outside(triangle);
    }
    inline bool is_out_b_envelope_exact(const std::array<Vector3, 3>& triangle) const {
        return b_tree_exact.is_outside(triangle);
    }
    inline bool is_out_tmp_b_envelope_exact(const std::array<Vector3, 3>& triangle) const {
        return tmp_b_tree_exact.is_outside(triangle);
    }
#endif

    //// envelope check - point
    inline bool is_out_sf_envelope(const Vector3& p, const Scalar eps_2, GEO::index_t& prev_facet) const {
        return is_out_envelope_point_impl(sf_data, GEO::vec3(p[0], p[1], p[2]), eps_2, prev_facet);
    }

    inline bool is_out_sf_envelope(const Vector3& p, const Scalar eps_2,
                                   GEO::index_t& prev_facet, double& sq_dist, GEO::vec3& nearest_p) const {
        GEO::vec3 geo_p(p[0], p[1], p[2]);
        return is_out_sf_envelope(geo_p, eps_2, prev_facet, sq_dist, nearest_p);
    }

    inline bool is_out_sf_envelope(const GEO::vec3& geo_p, const Scalar eps_2,
                                   GEO::index_t& prev_facet, double& sq_dist, GEO::vec3& nearest_p) const {
        if (prev_facet != GEO::NO_FACET) {
            sq_dist = sf_data.hint_sq_dist(geo_p, prev_facet, nearest_p);
            if (Scalar(sq_dist) <= eps_2) return false;
        }
        std::size_t prim_id;
        bool inside = bvh_queries::point_in_envelope_with_hit(
            sf_data.bvh, sf_data.tris_leaf, geo_p, eps_2, prim_id, sq_dist, nearest_p);
        if (inside) {
            prev_facet = static_cast<GEO::index_t>(sf_data.bvh.prim_ids[prim_id]);
            return false;
        }
        prev_facet = GEO::NO_FACET;
        return true;
    }

    inline bool is_out_b_envelope(const Vector3& p, const Scalar eps_2, GEO::index_t& prev_facet) const {
        return is_out_envelope_point_impl(b_data, GEO::vec3(p[0], p[1], p[2]), eps_2, prev_facet);
    }

    inline bool is_out_tmp_b_envelope(const Vector3& p, const Scalar eps_2, GEO::index_t& prev_facet) const {
        return is_out_envelope_point_impl(tmp_data, GEO::vec3(p[0], p[1], p[2]), eps_2, prev_facet);
    }

#ifdef NEW_ENVELOPE
    inline bool is_out_sf_envelope_exact(const Vector3& p) const {
        return sf_tree_exact.is_outside(p);
    }
    inline bool is_out_b_envelope_exact(const Vector3& p) const {
        return b_tree_exact.is_outside(p);
    }
    inline bool is_out_tmp_b_envelope_exact(const Vector3& p) const {
        return tmp_b_tree_exact.is_outside(p);
    }
#endif

    //fortest
    inline Scalar dist_sf_envelope(const std::vector<GEO::vec3>& ps, const Scalar eps_2,
                                   GEO::index_t prev_facet = GEO::NO_FACET) const {
        GEO::vec3 nearest_point;
        double sq_dist = std::numeric_limits<double>::max();
        for (const GEO::vec3& current_point : ps) {
            if (prev_facet != GEO::NO_FACET) {
                sq_dist = sf_data.hint_sq_dist(current_point, prev_facet, nearest_point);
            }
            if (Scalar(sq_dist) > eps_2) {
                std::size_t prim_id;
                bvh_queries::point_in_envelope_with_hit(
                    sf_data.bvh, sf_data.tris_leaf, current_point, eps_2, prim_id, sq_dist, nearest_point);
            }
            if (Scalar(sq_dist) > eps_2) {
                return sq_dist;
            }
        }
        return 0;
    }
    //fortest

private:
    static inline bool is_out_envelope_impl(
        const BvhData& data,
        const std::vector<GEO::vec3>& ps,
        const Scalar eps_2,
        GEO::index_t prev_facet)
    {
        GEO::vec3 nearest_point;
        double sq_dist = std::numeric_limits<double>::max();
        for (const GEO::vec3& current_point : ps) {
            if (prev_facet != GEO::NO_FACET) {
                sq_dist = data.hint_sq_dist(current_point, prev_facet, nearest_point);
                if (Scalar(sq_dist) <= eps_2) continue;
            }
            std::size_t prim_id;
            if (!bvh_queries::point_in_envelope_with_hit(
                    data.bvh, data.tris_leaf, current_point, eps_2,
                    prim_id, sq_dist, nearest_point)) {
                return true;
            }
            prev_facet = static_cast<GEO::index_t>(data.bvh.prim_ids[prim_id]);
        }
        return false;
    }

    static inline bool is_out_envelope_point_impl(
        const BvhData& data,
        const GEO::vec3& geo_p,
        const Scalar eps_2,
        GEO::index_t& prev_facet)
    {
        std::size_t prim_id;
        double sq_dist;
        GEO::vec3 nearest_p;
        bool inside = bvh_queries::point_in_envelope_with_hit(
            data.bvh, data.tris_leaf, geo_p, eps_2, prim_id, sq_dist, nearest_p);
        if (inside) {
            prev_facet = static_cast<GEO::index_t>(data.bvh.prim_ids[prim_id]);
            return false;
        }
        prev_facet = GEO::NO_FACET;
        return true;
    }
};

} // namespace floatTetWild

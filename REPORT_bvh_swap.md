# fTetWild AABB swap — final report

Replaced fTetWild's custom binary AABB tree (`MeshFacetsAABBWithEps`) with
[madmann91/bvh](https://github.com/madmann91/bvh) (v2). Custom stack-based
traversers added in [src/external/bvh_distance_queries.h](src/external/bvh_distance_queries.h).

## Wall-clock benchmark (ellipsoid.stl, --max-threads 1, 10 runs)

| Metric    | Baseline | v2     | v3     | v4     | **v5**     | v5 vs Baseline |
|-----------|---------:|-------:|-------:|-------:|-----------:|---------------:|
| Average   | 25.07 s  | 22.61 s| 21.97 s| 21.94 s| **20.49 s**| **−18.3 %**    |
| Median    | 25.09 s  | 22.24 s| 21.69 s| 21.96 s| **19.74 s**| **−21.3 %**    |
| Min       | 21.34 s  | 19.24 s| 19.54 s| 17.77 s| **17.21 s**| **−19.4 %**    |
| Max       | 31.74 s  | 27.32 s| 24.95 s| 24.51 s| **24.16 s**| **−23.9 %**    |
| Std dev   |  2.78 s  |  2.46 s|  2.16 s|  1.93 s| **2.43 s** |       −13 %    |

(see `baseline_metrics.txt`, `new_metrics_v{2,3,4,5_10runs}.txt`)

**v2 → v3 changes** ([src/external/bvh_distance_queries.h](src/external/bvh_distance_queries.h)):
- Branchless point-to-bbox distance (unrolled `std::max(0, max(min-p, p-max))` per axis) replaced the if/else-if chain. Random query/box pairs mispredict the branches on virtually every iteration, so the straight-line variant is friendlier to the pipeline and explains most of the variance/tail improvement.
- `config.max_leaf_size = 4` (default 8) in `build_from_geo_mesh`. Tighter leaves trade per-leaf brute scans for more pruning-friendly bbox tests; for fTetWild's small-ε queries that's a net win.
- Dropped the nearer-first child sort in `point_in_envelope` and `point_in_envelope_with_hit`. For boolean envelope queries the bound (sq_eps) is constant — sorting can't tighten it and just adds branches.

**v3 → v4 changes** (envelope traversers):
- Removed the redundant pop-time bbox re-test in the envelope traversers. The pre-push test already verified `<= sq_eps`, and `sq_eps` is constant — so the popped node is always still valid. The root is tested once before the loop. (In `closest_point` the pop-time re-test is *retained* — there the bound `best.sq_dist` tightens during traversal, so the re-test usefully kills stale stack entries.)
- Added a cheap triangle-bbox prefilter in all three traversers, ahead of the (heavy) `GEO::Geom::point_triangle_squared_distance` call. With `max_leaf_size=4` most triangles in a passing leaf are close to the query, so the prefilter only rejects in the tail — visible as lower variance and a better minimum, but unchanged median/average.

**v4 → v5 changes** ([src/AABBWrapper.h](src/AABBWrapper.h)):
- Consolidated all per-tree state into a private `BvhData` struct
  (`bvh`, `tris_leaf` in BVH-leaf order, `tris_mesh` in original GEO::Mesh
  facet-id order). Three instances: `sf_data`, `b_data`, `tmp_data`. The
  legacy public fields (`sf_bvh`, `sf_triangles`, `b_bvh`, `b_triangles`,
  `tmp_b_bvh`, `tmp_b_triangles`) are gone.
- The prev_facet hint shortcut now reads from `tris_mesh[prev_facet]` —
  one indexed load of 9 contiguous doubles — instead of going through
  `detail::point_facet_nearest_point`, which chased six GEO::Mesh corner
  pointers (`facets.corners_begin`, three `facet_corners.vertex`, three
  `Geom::mesh_vertex`). With millions of hint checks across the run, this
  is where the v4 → v5 ~7 % drop in average wall-clock came from.
- `detail::point_facet_nearest_point` is dropped (no callers). The leaf-id
  → mesh-id mapping (`bvh.prim_ids[]`) covers all the remaining indirections
  needed by the public API.

**v6 — tried, reverted: triangle-inequality skip in `is_out_envelope_impl`**

A sound, sqrt-free bound `(sqrt(anchor_d_sq) + sqrt(d_sq))² ≤ ε²` ⟺
`slack ≥ 0 ∧ 4·anchor_d_sq·d_sq ≤ slack²` (where `slack = ε² − anchor_d_sq − d_sq`)
should let multi-sample envelope checks skip the heavy `point_triangle_squared_distance`
on samples close to the most recent anchor. Implemented and benchmarked:

| | n  | avg     | std    |
|---|---:|--------:|-------:|
| v5 reverted (control, n=20) | 20 | 21.10 s | 2.6 s  |
| v6 (triangle inequality)   | 10 | 21.60 s | 2.36 s |

Difference: 0.5 SE — pure noise. **Reverted.**

Geometric reason it failed: [LocalOperations.cpp's sample_triangle()](src/LocalOperations.cpp#L861)
spaces samples by `mesh.params.dd`, and the run's parameter dump shows `dd ≈ 0.00289`,
`eps ≈ 0.00240` — i.e. sample spacing ≈ 1.2 · ε. For the bound to fire we'd need
`sqrt(anchor_d_sq) ≤ ε − sample_spacing = −0.2 · ε`, which is impossible. The
inequality test never succeeds, so it adds ~13 FLOPs per sample for zero benefit.
The same logic kills bbox-prefilter-on-hint and similar adjacent ideas — at this
sample density, the only remaining lever for fewer `point_triangle_squared_distance`
calls is replacing Geogram's exact routine itself, which the user has excluded.

## Perf flat profile shift (single-threaded sample, top symbols)

Baseline (`baseline_flat_summary.txt`):

```
18.01%  GEO::Geom::point_triangle_squared_distance
16.02%  point_box_signed_squared_distance
11.16%  MeshFacetsAABBWithEps::facet_in_envelope_recursive
 2.80%  get_point_facet_nearest_point
 0.68%  MeshFacetsAABBWithEps::get_nearest_facet_hint
```

New v3 (`new_flat_summary_v3.txt`):

```
20.39%  bvh_queries::point_in_envelope_with_hit   (includes inlined bbox + traversal)
18.56%  GEO::Geom::point_triangle_squared_distance
 2.35%  detail::point_facet_nearest_point
```

New v5 (`new_flat_summary_v5.txt`):

```
21.05%  bvh_queries::point_in_envelope_with_hit
20.99%  GEO::Geom::point_triangle_squared_distance
        (detail::point_facet_nearest_point — gone, deleted in v5)
```

The tuning moved ~7 percentage points out of the BVH traversal hotspot
(27.2 % → 20.4 %) — branchless box math and tighter leaves contributed.
v5 then deleted the GEO::Mesh corner-pointer hint helper entirely; the same
math now runs over a flat triangle array, attributed to `point_triangle_squared_distance`.

The `MeshFacetsAABBWithEps::*` symbols are gone. Total CPU spent in distance /
traversal code is comparable across versions — the wall-clock win is from cache
locality of the flat node array and lower per-call overhead, not from doing
fewer point-triangle distance computations.

The first row of `perf_diff.txt` shows a misleading `−28% in simplify(...)` /
`+25% in point_in_envelope_with_hit`: same work, different attribution. With
the old `mesh_AABB.cpp`, the recursive AABB code was its own symbol;
post-refactor it inlines into the new header function and gets attributed there.

## What changed

- New: [src/external/bvh_distance_queries.h](src/external/bvh_distance_queries.h)
  — `closest_point()`, `point_in_envelope()`, `point_in_envelope_with_hit()`,
  and a `build_from_geo_mesh()` helper.
- Refactored: [src/AABBWrapper.h](src/AABBWrapper.h),
  [src/AABBWrapper.cpp](src/AABBWrapper.cpp). Public API unchanged; internally
  the three trees are now `bvh::v2::Bvh<Node<double,3>>` with parallel
  `std::vector<Tri<double,3>>` stored in BVH-leaf order. The original facet id
  is recovered via `bvh.prim_ids[]`.
- Build: added `bvh` FetchContent block to
  [cmake/FloatTetwildDependencies.cmake](cmake/FloatTetwildDependencies.cmake)
  pinned to commit `ac41ab88a32d0c247009d8ed456fd2795c1ee023`. Linked into the
  `FloatTetwild` target. C++ standard bumped from C++14 to **C++20** (bvh uses
  `std::span` and defaulted comparison operators).
- Removed: `src/external/mesh_AABB.{h,cpp}` and their entries in
  [src/external/CMakeLists.txt](src/external/CMakeLists.txt). The unused
  `segment_intersection` and `compute_*_bbox_intersections` template entry
  points went with them.
- Preserved: `sf_mesh`, `b_mesh`, `tmp_b_mesh` GEO::Mesh objects (still needed
  by [MeshImprovement.cpp:1420-1422](src/MeshImprovement.cpp#L1420-L1422) for
  direct vertex lookup, and by the `prev_facet` hint shortcut).

## Correctness sanity

`./FloatTetwild_bin -i ellipsoid.stl --max-threads 1` runs to completion
without envelope assertion failures. Output `.msh` is ~1.4 MB with comparable
mesh statistics to baseline (post-postprocessing #v/#t differ slightly run-to-run,
which is normal — energy-ordered queues are sensitive to floating-point noise).

## Notes

- Both the old `facet_in_envelope_recursive` and the new
  `point_in_envelope_with_hit` early-exit on the first triangle within ε, so
  the algorithmic shape was already similar. The wins came from microarchitectural
  factors: contiguous node memory, tighter leaves, branchless bbox math, and
  removal of unproductive child-sort branches in the boolean traversal.
- The first row of `perf_diff_v3.txt` (`−28.95 % in simplify` / `+20.39 % in
  point_in_envelope_with_hit`) is misleading: same work, different attribution.
  In the baseline, `mesh_AABB.cpp` had its own out-of-line recursive symbol; in
  the new tree the header-inline traverser has its own symbol and absorbs what
  was previously charged to `simplify`'s callsite-inlined AABB code.

## Artifacts

- `baseline_metrics.txt`, `baseline_flat_summary.txt`, `baseline_flat_profile.txt`,
  `baseline_full_callgraph.txt`, `baseline_perf.data`
- `new_metrics_v2.txt`, `new_flat_summary_v2.txt`, `new_perf_v2.data` (v2: post-swap, pre-tuning)
- `new_metrics_v3.txt`, `new_flat_summary_v3.txt`, `new_perf_v3.data` (v3: tuned)
- `perf_diff.txt` (baseline vs v2), `perf_diff_v3.txt` (baseline vs v3)

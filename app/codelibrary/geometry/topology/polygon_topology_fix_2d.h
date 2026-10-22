//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_H_

#include <unordered_set>

#include "codelibrary/geometry/boolean/polygon_boolean_2d.h"
#include "codelibrary/geometry/boolean/polygon_union_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/even_odd_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Fix vertex sharing issues between the outer boundary and inner holes of a
 * 2D polygon.
 *
 * This function specifically addresses topological inconsistencies caused by
 * shared vertices between the outer ring and inner holes, which commonly occur
 * in polygons extracted from Arrangement2D structures (e.g., via
 * Arrangement2D::GetRegions()).
 *
 * This function assumes the polygon is derived from Arrangement2D, ensuring:
 *  1. Exactly one outer boundary ring (oriented counter-clockwise).
 *  2. Zero or more inner hole rings (each oriented clockwise).
 *  3. No self-intersecting or coincide edges in the polygon.
 *  4. For each inner ring, at most one vertex may be shared with the outer
 *     ring.
 */
template <typename T>
void FixOuterInnerVertexSharing(const MultiPolygon2D<T>& polygon,
                                Array<MultiPolygon2D<T>>* out_polygons) {
    CHECK(out_polygons);

    out_polygons->clear();
    if (polygon.n_boundaries() == 1) {
        out_polygons->push_back(polygon);
        return;
    }

    std::unordered_set<Point2D<T>> hash;
    const auto& boundary = polygon.boundaries()[0];
    CHECK(boundary.is_outer);

    for (auto p : boundary.polygon) {
        hash.insert(p);
    }

    bool found = false;
    for (int i = 1; i < polygon.n_boundaries(); ++i) {
        const auto& boundary = polygon.boundaries()[i];
        CHECK(!boundary.is_outer);

        for (auto p : boundary.polygon) {
            if (hash.find(p) != hash.end()) {
                found = true;
                break;
            }
        }
        if (found) break;
    }
    if (!found) {
        out_polygons->push_back(polygon);
        return;
    }

    Arrangement2D<T> arrangement;
    for (const auto& poly : polygon.boundaries()) {
        for (int i = 0; i < poly.polygon.size(); ++i) {
            arrangement.InsertWithoutCross(poly.polygon[i],
                                           poly.polygon.next_vertex(i));
        }
    }
    arrangement.Arrange();

    const auto& outer_ring = polygon.boundaries()[0].polygon;
    Array<Segment2D<T>> segs;
    for (const auto& p : polygon) {
        for (int i = 0; i < p.polygon.size(); ++i) {
            segs.push_back(p.polygon.edge(i));
        }
    }

    for (int i = 1; i < polygon.n_boundaries(); ++i) {
        const auto& poly = polygon.boundaries()[i].polygon;

        for (auto p : poly) {
            if (hash.find(p) == hash.end()) continue;

            // Found a intersection.
            double max_angle = 0.0;
            Segment2D<T> best_s;
            for (int j = 0; j < poly.size(); ++j) {
                Point2D<T> q = poly[j];
                Vector2D<T> v1 = poly.next_vertex(j) - q;
                Vector2D<T> v2 = poly.prev_vertex(j) - q;

                if (q == p) continue;
                for (int k = 0; k < outer_ring.size(); ++k) {
                    Point2D<T> q1 = outer_ring[k];
                    if (q1 == p) continue;
                    Vector2D<T> v3 = outer_ring.next_vertex(k) - q1;
                    Vector2D<T> v4 = outer_ring.prev_vertex(k) - q1;

                    Vector2D<T> v = q1 - q;
                    double degree1 = std::min(Degree(v, v1), Degree(v, v2));
                    double degree2 = std::min(Degree(-v, v3), Degree(-v, v4));
                    double degree = std::min(degree1, degree2);
                    if (degree <= max_angle) continue;

                    Segment2D<T> s(q, q1);
                    bool can = true;
                    for (auto s1 : segs) {
                        if (s1.lower_point() == q || s1.lower_point() == q1)
                            continue;
                        if (s1.upper_point() == q || s1.upper_point() == q1)
                            continue;
                        if (Intersect(s, s1)) {
                            can = false;
                            break;
                        }
                    }
                    if (can) {
                        max_angle = degree;
                        best_s = s;
                    }
                }
            }
            arrangement.Insert(best_s.lower_point(), best_s.upper_point(),
                               1, 1);
            segs.emplace_back(best_s);
            break;
        }
    }
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, out_polygons);
}

/**
 * Fix the potential self-intersection polygon and convert it into
 * polygon-with-holes structures using even-odd rule.
 */
template <typename T>
void FixEvenOddPolygon(const MultiPolygon2D<T>& polygon, double snap_threshold,
                       Array<MultiPolygon2D<T>>* out_polygons) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(out_polygons);
    CHECK(snap_threshold >= 0.0);

    out_polygons->clear();

    Array<Segment2D<T>> segs;
    for (const auto& b : polygon.boundaries()) {
        for (int i = 0; i < b.polygon.size(); ++i) {
            segs.emplace_back(b.polygon.edge(i));
        }
    }

    MultiPolygon2D<T> tmp_polygon;
    EvenOddPolygon2D(segs, snap_threshold, &tmp_polygon);
    out_polygons->push_back(std::move(tmp_polygon));
}
template <typename T>
void FixEvenOddPolygon(const MultiPolygon2D<T>& polygon,
                       Array<MultiPolygon2D<T>>* out_polygons) {
    FixEvenOddPolygon(polygon, 0.0, out_polygons);
}

/**
 * Fixes topological errors in a polygon-with-holes structure.
 *
 * The input must be a polygon-with-holes, where the first boundary is the outer
 * ring and all subsequent boundaries are inner rings (holes). This function
 * corrects common topological issues including:
 * - Edge sharing (overlapping edges between rings)
 * - Self-intersections (intersections within a single ring)
 *
 * Compared to the FixEvenOddPolygon function, this function follows specific
 * repair rules:
 * - For overlapping parts between inner rings: retain them as part of the inner
 *   rings.
 * - For parts of inner rings that lie outside the outer ring: directly remove
 *   these extraneous parts.
 */
template <typename T>
void FixPolygonWithHoles(const MultiPolygon2D<T>& polygon,
                         double snap_threshold,
                         Array<MultiPolygon2D<T>>* out_polygons) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(out_polygons);
    CHECK(snap_threshold >= 0.0);
    CHECK(polygon.IsPolygonWithHoles());

    out_polygons->clear();
    if (polygon.empty()) return;

    // Get outer polygon.
    MultiPolygon2D<T> outer_polygon;
    Array<MultiPolygon2D<T>> outer_polygons;
    outer_polygons.emplace_back(polygon.boundaries()[0].polygon);
    // Important!
    outer_polygons.back().Trim();
    ExternalPolygonUnion2D(outer_polygons, &outer_polygon);
    if (outer_polygon.empty()) return;

    // Get union inner polygons.
    MultiPolygon2D<T> inner_polygon;
    Array<MultiPolygon2D<T>> inner_polygons;
    for (int i = 1; i < polygon.n_boundaries(); ++i) {
        inner_polygons.emplace_back(polygon.boundaries()[i].polygon);
    }
    ExternalPolygonUnion2D(inner_polygons, &inner_polygon);

    MultiPolygon2D<T> result;
    PolygonBoolean2D<T> boolean(outer_polygon, inner_polygon, snap_threshold);
    boolean.Difference(&result);
    EvenOddPolygon2D(result, snap_threshold, out_polygons);
}
template <typename T>
void FixPolygonWithHoles(const MultiPolygon2D<T>& polygon,
                         Array<MultiPolygon2D<T>>* out_polygons) {
    static_assert(std::is_floating_point<T>::value);

    FixPolygonWithHoles(polygon, 0.0, out_polygons);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_H_

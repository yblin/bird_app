//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_

#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/winding_number.h"

namespace cl {
namespace geometry {

/**
 * Convert 2D Arrangement to multi-polygon via even-odd-rule.
 */
template <typename T>
void EvenOddPolygon2D(const Arrangement2D<T>& arrangement,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);

    using Point    = Point2D<T>;
    using DCEL     = HalfedgeList<Point>;
    using Halfedge = typename DCEL::Halfedge;

    polygon->clear();
    if (arrangement.empty() || arrangement.regions().empty()) return;

    Array<int> winding;
    WindingNumber(arrangement, &winding);

    DCEL list;
    arrangement.ToHalfedgeList(&list);

    auto visited = list.AddHalfedgeProperty(false);
    Array<Point> points;
    for (Halfedge* e : list) {
        int id = arrangement.region_id(e);
        if (id == -1) continue;

        if (winding[id] % 2 == 1 && !visited[e]) {
            points.clear();
            for (Halfedge* e1 : list.circular_list(e)) {
                visited[e1] = true;
                points.push_back(e1->source_point());
            }
            Polygon2D<T> poly(points);
            if (!poly.empty()) {
                polygon->Insert(poly, !poly.IsClockwise());
            }
        }
    }
}

/**
 * Convert 2D Arrangement into multiple polygons with holes based on the
 * even-odd rule.
 */
template <typename T>
void EvenOddPolygon2D(const Arrangement2D<T>& arrangement,
                      Array<MultiPolygon2D<T>>* polygons) {
    CHECK(polygons);

    polygons->clear();
    if (arrangement.empty() || arrangement.regions().empty()) return;

    Array<int> winding;
    WindingNumber(arrangement, &winding);

    for (int i = 0; i < arrangement.regions().size(); ++i) {
        if (winding[i] % 2 == 1) {
            MultiPolygon2D<T> polygon_with_holes;
            arrangement.GetRegion(i, &polygon_with_holes);
            polygons->push_back(polygon_with_holes);
        }
    }
}

/**
 * Get even-odd polygon consisting of multiple line segments.
 */
template <typename T>
void EvenOddPolygon2D(const Array<Segment2D<T>>& segs, double snap_threshold,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    CHECK(snap_threshold >= 0.0);

    polygon->clear();
    if (segs.empty()) return;

    Arrangement2D<T> arrangement(snap_threshold);
    arrangement.Reset(segs);
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, polygon);
}
template <typename T>
void EvenOddPolygon2D(const Array<Segment2D<T>>& segs,
                      MultiPolygon2D<T>* polygon) {
    EvenOddPolygon2D(segs, 0.0, polygon);
}

/**
 * Get even-odd polygon of a given vertices loop.
 */
template <typename T>
void EvenOddPolygon2D(const Array<Point2D<T>>& points, double snap_threshold,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    CHECK(snap_threshold >= 0.0);

    polygon->clear();
    if (points.empty()) return;

    Array<Segment2D<T>> segs;
    for (int i = 0; i + 1 < points.size(); ++i) {
        segs.emplace_back(points[i], points[i + 1]);
    }
    segs.emplace_back(points.back(), points.front());

    EvenOddPolygon2D(segs, snap_threshold, polygon);
}
template <typename T>
void EvenOddPolygon2D(const Array<Point2D<T>>& points,
                      MultiPolygon2D<T>* polygon) {
    EvenOddPolygon2D(points, 0.0, polygon);
}

/**
 * Convert the given polygon to an even-odd polygon.
 * Assume that the input polygon is a valid polygon.
 */
template <typename T>
void EvenOddPolygon2D(const MultiPolygon2D<T>& input, double snap_threshold,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    CHECK(snap_threshold >= 0.0);

    if (input.n_boundaries() <= 1) {
        *polygon = input;
        return;
    }

    Arrangement2D<T> arrangement;
    for (const auto& poly : input.boundaries()) {
        for (int i = 0; i < poly.polygon.size(); ++i) {
            arrangement.InsertWithoutCross(poly.polygon[i],
                                           poly.polygon.next_vertex(i));
        }
    }
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, polygon);
}
template <typename T>
void EvenOddPolygon2D(const MultiPolygon2D<T>& input,
                      MultiPolygon2D<T>* polygon) {
    EvenOddPolygon2D(input, 0.0, polygon);
}

/**
 * Converts the input MultiPolygon2D into multiple polygons with holes
 * based on the even-odd rule.
 *
 * Assume that the input polygon is a valid polygon.
 */
template <typename T>
void EvenOddPolygon2D(const MultiPolygon2D<T>& input, double snap_threshold,
                      Array<MultiPolygon2D<T>>* polygons) {
    CHECK(polygons);
    CHECK(snap_threshold >= 0.0);

    polygons->clear();
    if (input.n_boundaries() <= 1) {
        polygons->push_back(input);
        return;
    }

    Arrangement2D<T> arrangement(snap_threshold);
    for (const auto& poly : input.boundaries()) {
        for (int i = 0; i < poly.polygon.size(); ++i) {
            arrangement.InsertWithoutCross(poly.polygon[i],
                                           poly.polygon.next_vertex(i));
        }
    }
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, polygons);
}
template <typename T>
void EvenOddPolygon2D(const MultiPolygon2D<T>& input,
                      Array<MultiPolygon2D<T>>* polygons) {
    EvenOddPolygon2D(input, 0.0, polygons);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_

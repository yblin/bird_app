//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_OFFSET_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_OFFSET_2D_H_

#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/topology/polygon_offset_2d.h"

namespace cl {
namespace geometry {

/**
 * Fast approximate offset operation for 2D simple (non-self-intersecting)
 * polylines.
 *
 * This implementation prioritizes speed over precision and only works for small
 * offset distances. Input polyline must be non-self-intersecting to ensure
 * reliable results.
 */
template <typename T>
void FastPolylineOffset2D(const Polyline<Point2D<T>>& polyline, double distance,
                          Polyline<Point2D<T>>* offset_polyline) {
    static_assert(std::is_floating_point<T>::value);
    CHECK(offset_polyline);

    *offset_polyline = polyline;
    offset_polyline->Trim();
    if (distance == 0.0) return;

    using Vector = Vector2D<T>;

    Polyline<Point2D<T>>& res = *offset_polyline;

    // Get bisector vectors for each vertex of polyline.
    Array<Vector> normals(res.size());
    normals.front() = polyline_util::GetNormal(res, 0) * distance;
    normals.back() = polyline_util::GetNormal(res, res.size() - 1) * distance;
    for (int i = 1; i + 1 < res.size(); ++i) {
        Vector v1 = res[i - 1] - res[i];
        Vector v2 = res[i + 1] - res[i];
        Vector v = polyline_util::GetNormal(res, i);
        double angle = Radian(v1, v2);
        normals[i] = v * (distance / std::sin(0.5 * angle));
    }

    for (int i = 0; i < res.size(); ++i) {
        res[i] = res[i] + normals[i];
    }
}

/**
 * Perform bidirectional offset operation on a 2D polyline to generate a buffer
 * polygon.
 *
 * Parameters:
 *  polyline       - Input polyline.
 *  distance       - Distance to which to extend the geometry.
 *  round_segments - Number of line segments used to approximate a quarter
 *                   circle.
 *  polygon        - Output offset polygon.
 */
template <typename T>
void PolylineOffset2D(const Polyline<Point2D<T>>& polyline, double distance,
                      int round_segments,
                      MultiPolygon2D<T>* polygon) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(polygon);
    CHECK(round_segments > 0);
    CHECK(distance >= 0.0);

    polygon->clear();
    if (polyline.size() <= 1 || distance == 0.0) return;

    if (polyline.front() == polyline.back()) {
        Polygon2D<T> input(polyline);
        MultiPolygon2D<T> left, right;
        PolygonOffset2D(input, distance, round_segments, &left);
        PolygonOffset2D(input, -distance, round_segments, &right);
        for (auto& b : left.boundaries()) {
            polygon->Insert(b.polygon, b.is_outer);
        }
        for (auto& b : right.boundaries()) {
            polygon->Insert(b.polygon, !b.is_outer);
        }
        return;
    }

    using Point = Point2D<T>;
    Array<Point> vertices = polyline;
    vertices.insert(vertices.end(), polyline.rbegin(), polyline.rend());

    PolygonOffset2D(vertices, distance, round_segments, polygon);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_OFFSET_2D_H_

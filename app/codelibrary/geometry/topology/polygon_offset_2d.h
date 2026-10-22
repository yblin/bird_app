//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_OFFSET_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_OFFSET_2D_H_

#include <cmath>
#include <limits>

#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/generator_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/winding_number.h"
#include "codelibrary/geometry/topology/winding_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Generate raw offset curves for a 2D polygon with specified offset distance
 * and arc segmentation.
 *
 * Reference:
 *  Chen X, McMains S. Polygon offsetting by computing winding numbers[C].
 *  International Design Engineering Technical Conferences and Computers and
 *  Information in Engineering Conference. 2005, 4739: 565-575.
 */
template <typename T>
void GetRawOffsetCurves2D(const Array<Point2D<T>>& polygon, double distance,
                          int round_segments, Array<Point2D<T>>* curves) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(curves);
    CHECK(round_segments > 0);

    curves->clear();
    if (polygon.empty() || distance == 0.0) return;

    using Point = Point2D<T>;
    using Vector = Vector2D<T>;
    using Piece = std::pair<Vector, std::pair<Point, Point>>;

    Array<Point> vertices = polygon;
    Polygon2D<T> tmp(vertices);
    if (tmp.IsClockwise()) {
        std::reverse(vertices.begin(), vertices.end());
    }
    vertices.push_back(vertices.front());

    Array<Piece> offset_segments;
    for (int i = 0; i + 1 < vertices.size(); ++i) {
        if (vertices[i] == vertices[i + 1]) continue;
        Vector2D<T> v = (vertices[i + 1] - vertices[i]).normalized();
        v = Vector2D<T>(v.y, -v.x) * distance;
        offset_segments.emplace_back(v, std::make_pair(vertices[i],
                                                       vertices[i + 1]));
    }
    if (offset_segments.empty()) return;

    if (distance < 0.0) {
        std::reverse(offset_segments.begin(), offset_segments.end());
        for (auto& s : offset_segments) {
            std::swap(s.second.first, s.second.second);
        }
    }
    offset_segments.push_back(offset_segments.front());

    // Connect pieces.
    Array<Point> arc_points;
    for (int i = 0; i + 1 < offset_segments.size(); ++i) {
        auto& s = offset_segments[i];
        curves->push_back(s.second.first + s.first);
        curves->push_back(s.second.second + s.first);

        auto& s1 = offset_segments[i + 1];
        Point start = s.second.second + s.first;
        Point end = s1.second.first + s1.first;
        Point base = s.second.second;
        int o = Orientation(base, start, end);
        if (o > 0 || (o == 0 && start != end)) {
            Vector t = start - base;
            t = Vector(-t.y, t.x);
            Arc2D<T> arc(start, t, end);
            EvenlyPointsOnArc2D(arc, round_segments + 2, &arc_points);
            for (int j = 1; j + 1 < arc_points.size(); ++j) {
                curves->push_back(arc_points[j]);
            }
        } else if (o < 0) {
            curves->push_back(base);
        }
    }
}

/**
 * Perform offset operation on a simple 2D polygon.
 *
 * Parameters:
 *  polygon        - Input polygon.
 *  distance       - Distance to which to extend the geometry.
 *  round_segments - Number of line segments used to approximate a quarter
 *                   circle.
 *  result         - Output offset result.
 */
template <typename T>
void PolygonOffset2D(const Array<Point2D<T>>& polygon, double distance,
                     int round_segments, MultiPolygon2D<T>* result) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(result);
    CHECK(round_segments > 0);

    result->clear();
    if (polygon.empty() || distance == 0.0) return;

    using Point = Point2D<T>;
    Array<Point> curves;
    GetRawOffsetCurves2D(polygon, distance, round_segments, &curves);

    if (curves.empty()) return;
    curves.push_back(curves.front());

    using Line = typename Arrangement2D<T>::Line;
    Array<Line> lines;
    for (int i = 0; i + 1 < curves.size(); ++i) {
        if (distance > 0.0) {
            lines.emplace_back(curves[i], curves[i + 1], 1, 0);
        } else {
            lines.emplace_back(curves[i], curves[i + 1], 0, 1);
        }
    }

    // Snap-round vertices within a floating-point tolerance so near-duplicate
    // vertices (a computed crossing landing next to a curve endpoint) merge
    // instead of surviving a few ULPs apart and forming degenerate slivers.
    // A computed segment intersection has error ~ L*eps/sin(theta), which over
    // the range of resolvable crossing angles (sin(theta) >= sqrt(eps)) is
    // bounded by sqrt(eps)*L -- hence sqrt(machine epsilon), not epsilon.
    // Scaled by the bounding-box diagonal L so it is invariant to where the
    // geometry sits, and uses DBL_EPSILON / FLT_EPSILON per T.
    Box2D<T> box(curves.begin(), curves.end());
    const double dx       = static_cast<double>(box.x_length());
    const double dy       = static_cast<double>(box.y_length());
    const double diagonal = std::sqrt(dx * dx + dy * dy);
    const double snap_threshold =
        std::sqrt(std::numeric_limits<T>::epsilon()) * diagonal;

    Arrangement2D<T> arrangement(snap_threshold);
    arrangement.set_color_policy(Delaunay2D<T>::ColorPolicy::ADDITION);
    arrangement.Reset(lines);
    arrangement.Arrange();

    Array<int> winding;
    WeightedWindingNumber(arrangement, &winding);
    WindingPolygon2D(arrangement, winding, WindingRule::POSITIVE, result);
}
template <typename T>
void PolygonOffset2D(const Polygon2D<T>& polygon, double distance,
                     int round_segments, MultiPolygon2D<T>* result) {
    PolygonOffset2D(polygon.vertices(), distance, round_segments, result);
}
template <typename T>
void PolygonOffset2D(const Polygon2D<T>& polygon, double distance,
                     MultiPolygon2D<T>* result) {
    PolygonOffset2D(polygon.vertices(), distance, 8, result);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_POLYGON_OFFSET_2D_H_

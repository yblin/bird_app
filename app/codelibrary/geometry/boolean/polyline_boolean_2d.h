//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_POLYLINE_BOOLEAN_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_POLYLINE_BOOLEAN_2D_H_

#include "codelibrary/geometry/intersect_2d.h"

namespace cl {
namespace geometry {

/**
 * Compute both the intersection and difference between a polyline and a
 * multi-polygon.
 *
 * This function decomposes the input polyline into two disjoint sets of
 * sub-polylines:
 * 1. Intersection: Sub-polylines that lie inside or on the boundary of the
 *    multi-polygon.
 * 2. Difference: Sub-polylines that lie strictly outside the multi-polygon.
 *
 * A key invariant guaranteed by this algorithm is that the union of all
 * intersection and difference polylines is exactly equivalent to the original
 * input polyline.
 */
template <typename T>
void PolylineBoolean2D(const Polyline<Point2D<T>>& polyline,
                       const MultiPolygon2D<T>& polygon,
                       Array<Polyline<Point2D<T>>>* intersection_polylines,
                       Array<Polyline<Point2D<T>>>* difference_polylines) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(intersection_polylines);
    CHECK(difference_polylines);

    using Point = Point2D<T>;
    using Polyline2D = Polyline<Point>;

    intersection_polylines->clear();
    difference_polylines->clear();

    using Segment = typename Polyline2D::Segment;
    const int n = polyline.size();

    Polyline2D poly1, poly2;
    for (int i = 0; i + 1 < n; ++i) {
        if (polyline[i] == polyline[i + 1]) continue;

        const Segment& seg = polyline.segment(i);
        Array<Point> intersection;
        Intersect(seg, polygon, &intersection);
        CHECK(intersection.size() % 2 == 0);

        const Point& prev = polyline[i];
        const Point& next = polyline[i + 1];

        if (!intersection.empty()) {
            if (prev != intersection.front()) {
                if (poly2.empty()) poly2.push_back(prev);
                poly2.push_back(intersection.front());

                if (!poly1.empty()) {
                    intersection_polylines->push_back(poly1);
                    poly1.clear();
                }
            }

            for (int j = 0; j < intersection.size(); ++j) {
                if (j % 2 == 0) {
                    if (!poly2.empty()) {
                        difference_polylines->push_back(poly2);
                        poly2.clear();
                    }

                    if (intersection[j] != intersection[j + 1]) {
                        if (poly1.empty()) {
                            poly1.push_back(intersection[j]);
                        }
                        poly1.push_back(intersection[j + 1]);
                    }
                } else {
                    if (j + 1 == intersection.size()) break;
                    if (!poly1.empty()) {
                        intersection_polylines->push_back(poly1);
                        poly1.clear();
                    }

                    if (intersection[j] != intersection[j + 1]) {
                        if (poly2.empty()) {
                            poly2.push_back(intersection[j]);
                        }
                        poly2.push_back(intersection[j + 1]);
                    }
                }
            }

            if (next != intersection.back()) {
                if (poly2.empty()) poly2.push_back(intersection.back());
                poly2.push_back(next);

                if (!poly1.empty()) {
                    intersection_polylines->push_back(poly1);
                    poly1.clear();
                }
            }
        } else {
            if (poly2.empty()) poly2.push_back(prev);
            poly2.push_back(next);
            if (!poly1.empty()) {
                intersection_polylines->push_back(poly1);
                poly1.clear();
            }
        }
    }
    if (!poly1.empty()) {
        intersection_polylines->push_back(poly1);
    }
    if (!poly2.empty()) {
        difference_polylines->push_back(poly2);
    }

    // In case where the polyline degenerates into a point.
    if (intersection_polylines->empty() && difference_polylines->empty()) {
        if (Intersect(polyline.front(), polygon)) {
            intersection_polylines->push_back(polyline);
        } else {
            difference_polylines->push_back(polyline);
        }
    }
}

/**
 * Convenience overload clipping a polyline against an axis-aligned box, treating
 * the box as a single-boundary multi-polygon.
 */
template <typename T>
void PolylineBoolean2D(const Polyline<Point2D<T>>& polyline,
                       const Box2D<T>& box,
                       Array<Polyline<Point2D<T>>>* intersection_polylines,
                       Array<Polyline<Point2D<T>>>* difference_polylines) {
    PolylineBoolean2D(polyline,
                      MultiPolygon2D<T>(box),
                      intersection_polylines,
                      difference_polylines);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_POLYLINE_BOOLEAN_2D_H_

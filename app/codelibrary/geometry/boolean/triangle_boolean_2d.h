//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_TRIANGLE_BOOLEAN_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_TRIANGLE_BOOLEAN_2D_H_

#include "codelibrary/geometry/envelope/convex_hull_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/triangle_2d.h"

namespace cl {
namespace geometry {

/**
 * Compute the intersection between two triangles. The result is a convex hull.
 */
template <typename T>
bool TriangleIntersection2D(const Triangle2D<T>& tri1,
                            const Triangle2D<T>& tri2,
                            double snap_threshold,
                            ConvexHull2D<T>* convex_hull) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(snap_threshold > 0.0);

    CHECK(convex_hull);

    if (tri1.empty() || tri2.empty()) {
        convex_hull->clear();
        return false;
    }

    if (!Intersect(tri1.bounding_box(), tri2.bounding_box())) return false;

    Array<Point2D<T>> vertices;
    for (const Point2D<T>& v : tri1.vertices()) {
        if (Intersect(v, tri2))
            vertices.push_back(v);
    }
    for (const Point2D<T>& v : tri2.vertices()) {
        if (Intersect(v, tri1))
            vertices.push_back(v);
    }

    Array<Point2D<T>> nearest_vertices(4);
    Point2D<T> q;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Segment2D<T> s1 = tri1.edge(i);
            const Segment2D<T> s2 = tri2.edge(j);
            if (Cross(s1, s2, &q)) {
                nearest_vertices[0] = s1.lower_point();
                nearest_vertices[1] = s1.upper_point();
                nearest_vertices[2] = s2.lower_point();
                nearest_vertices[3] = s2.upper_point();
                double dis = DBL_MAX;
                int nearest = -1;
                for (int k = 0; k < 4; ++k) {
                    double d = Distance(nearest_vertices[k], q);
                    if (d < dis) {
                        dis = d;
                        nearest = k;
                    }
                }
                if (dis <= snap_threshold) {
                    vertices.push_back(nearest_vertices[nearest]);
                } else {
                    vertices.push_back(q);
                }
            }
        }
    }

    convex_hull->Reset(vertices);
    if (convex_hull->vertices().size() <= 2) {
        convex_hull->clear();
    }
    return !convex_hull->empty();
}

/**
 * Similar to the previous function, but return the triangle set as the result.
 */
template <typename T>
bool TriangleIntersection2D(const Triangle2D<T>& tri1,
                            const Triangle2D<T>& tri2,
                            double snap_threshold,
                            Array<Triangle2D<T>>* result) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(snap_threshold > 0.0);
    CHECK(result);

    result->clear();
    ConvexHull2D<T> convex_hull;
    if (!TriangleIntersection2D(tri1, tri2, snap_threshold, &convex_hull))
        return false;

    const Array<Point2D<T>>& vertices = convex_hull.vertices();
    int n_vertices = vertices.size();
    for (int i = 1; i + 1 < n_vertices; ++i) {
        result->emplace_back(vertices[0], vertices[i], vertices[i + 1]);
    }

    return !result->empty();
}

/**
 * Compute the intersection between two sets of triangles.
 *
 * Assumes that triangles within each input set (triangles1 and triangles2) are
 * non-intersecting.
 *
 * The intersection result is returned as a set of triangles.
 */
template <typename T>
bool TriangleIntersection2D(const Array<Triangle2D<T>>& triangles1,
                            const Array<Triangle2D<T>>& triangles2,
                            double snap_threshold,
                            Array<Triangle2D<T>>* result) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(snap_threshold > 0.0);
    CHECK(result);

    if (triangles1.empty() || triangles2.empty()) return false;

    Array<Triangle2D<T>> res;
    for (const Triangle2D<T>& tri1 : triangles1) {
        for (const Triangle2D<T>& tri2 : triangles2) {
            if (TriangleIntersection2D(tri1, tri2, snap_threshold, &res)) {
                result->insert(res);
            }
        }
    }
    return !result->empty();
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_TRIANGLE_BOOLEAN_2D_H_

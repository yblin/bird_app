//
// Copyright 2014-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Computer the center of geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_CENTER_H_
#define CODELIBRARY_GEOMETRY_CENTER_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/geometry/triangle_3d.h"

namespace cl {

/**
 * Compute the centroid of two points.
 */
template <typename T>
Point2D<T> Center(const Point2D<T>& p1, const Point2D<T>& p2) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point2D<T>((p1.x + p2.x) * T(0.5), (p1.y + p2.y) * T(0.5));
}
template <typename T>
Point3D<T> Center(const Point3D<T>& p1, const Point3D<T>& p2) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>((p1.x + p2.x) * T(0.5), (p1.y + p2.y) * T(0.5),
                      (p1.z + p2.z) * T(0.5));
}

/**
 * Compute the centroid of three points.
 */
template <typename T>
Point2D<T> Center(const Point2D<T>& p1, const Point2D<T>& p2,
                  const Point2D<T>& p3) {
    static_assert(std::is_floating_point<T>::value, "");

    T s = T(1) / T(3);
    return Point2D<T>((p1.x + p2.x + p3.x) * s, (p1.y + p2.y + p3.y) * s);
}
template <typename T>
Point3D<T> Center(const Point3D<T>& p1, const Point3D<T>& p2,
                  const Point3D<T>& p3) {
    static_assert(std::is_floating_point<T>::value, "");

    T s = T(1) / T(3);
    return Point3D<T>((p1.x + p2.x + p3.x) * s, (p1.y + p2.y + p3.y) * s,
                      (p1.z + p2.z + p3.z) * s);
}

/**
 * Compute the centroid of a bounding box.
 */
template <typename T>
Point2D<T> Center(const Box2D<T>& box) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point2D<T>((box.x_min() + box.x_max()) * T(0.5),
                      (box.y_min() + box.y_max()) * T(0.5));
}
template <typename T>
Point3D<T> Center(const Box3D<T>& box) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>((box.x_min() + box.x_max()) * T(0.5),
                      (box.y_min() + box.y_max()) * T(0.5),
                      (box.z_min() + box.z_max()) * T(0.5));
}

/**
 * Compute the centroid of a line segment.
 */
template <typename T>
Point2D<T> Center(const Segment2D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    return Center(line.lower_point(), line.upper_point());
}
template <typename T>
Point3D<T> Center(const Segment3D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    return Center(line.lower_point(), line.upper_point());
}

/**
 * Compute the centroid of a point set.
 */
template <typename Point>
Point Center(const Array<Point>& points) {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!points.empty());

    const int dimension = points.front().size();

    Array<T> v(dimension, 0);
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < dimension; ++j) {
            v[j] += points[i][j];
        }
    }

    return Point(v.begin(), v.end()) * (T(1) / points.size());
}

/**
 * Compute the weighted centroid of a point set.
 */
template <typename Point, typename T>
Point Center(const Array<Point>& points, const Array<T>& weights) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!points.empty());
    CHECK(points.size() == weights.size());

    const int dimension = points.front().size();

    T sum = 0;
    Array<T> v(dimension, 0);
    for (int i = 0; i < points.size(); ++i) {
        T w = weights[i];
        for (int j = 0; j < dimension; ++j) {
            v[j] += w * points[i][j];
        }
        sum += w;
    }
    CHECK(sum != 0);

    return Point(v.begin(), v.end()) * (T(1) / sum);
}

/**
 * Compute the centroid of a point set.
 */
template <typename T>
Point2D<T> Center(const Triangle2D<T>& triangle) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!triangle.empty());

    return Center(triangle.vertices());
}

/**
 * Compute the centroid of a point set.
 */
template <typename T>
Point3D<T> Center(const Triangle3D<T>& triangle) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!triangle.empty());

    return Center(triangle.vertices());
}

/**
 * Return the center point on the polyline.
 */
template <typename Point>
Point Center(const Polyline<Point>& polyline) {
    CHECK(!polyline.empty());

    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value);

    int n = polyline.size();
    if (n == 1) return polyline.front();

    double half_length = 0.5 * polyline.Length();
    if (half_length == 0.0) return polyline.front();

    Array<double> segment_lengths(n - 1);
    for (int i = 0; i + 1 < n; ++i) {
        segment_lengths[i] = (polyline[i] - polyline[i + 1]).norm();
    }

    double accumulated = 0.0;
    for (int i = 0; i < segment_lengths.size(); ++i) {
        if (segment_lengths[i] == 0.0) continue;

        if (accumulated + segment_lengths[i] >= half_length) {
            double remaining = half_length - accumulated;
            T ratio = remaining / segment_lengths[i];
            return polyline[i] + (polyline[i + 1] - polyline[i]) * ratio;
        }
        accumulated += segment_lengths[i];
    }

    return polyline.back();
}

/**
 * Circumcenter of three 2D points.
 */
template <typename T>
Point2D<T> Circumcenter(const Point2D<T>& p1, const Point2D<T>& p2,
                        const Point2D<T>& p3) {
    static_assert(std::is_floating_point<T>::value, "");

    // Use 'double' to reduce floating point error.
    double ax = (p1.x + p2.x) * 0.5;
    double ay = (p1.y + p2.y) * 0.5;
    double ux = (p1.y - p2.y);
    double uy = (p2.x - p1.x);
    double bx = (p2.x + p3.x) * 0.5;
    double by = (p2.y + p3.y) * 0.5;
    double vx = (p2.y - p3.y);
    double vy = (p3.x - p2.x);
    double dx = ax - bx;
    double dy = ay - by;

    double vu = vx * uy - vy * ux;
    double g = (dx * uy - dy * ux) / vu;
    if (std::isinf(g) || std::isnan(g)) {
        // Points are collinear.
        double d1 = (p1 - p2).dot(p1 - p2);
        double d2 = (p1 - p3).dot(p1 - p3);
        double d3 = (p2 - p3).dot(p2 - p3);
        if (d1 > d2) {
            if (d3 > d1) {
                return Point2D<T>(0.5 * (p2.x + p3.x), 0.5 * (p2.y + p3.y));
            } else {
                return Point2D<T>(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y));
            }
        } else {
            if (d3 > d2) {
                return Point2D<T>(0.5 * (p2.x + p3.x), 0.5 * (p2.y + p3.y));
            } else {
                return Point2D<T>(0.5 * (p1.x + p3.x), 0.5 * (p1.y + p3.y));
            }
        }
    }

    return Point2D<T>(bx + g * vx, by + g * vy);
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CENTER_H_

//
// Copyright 2019-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_
#define CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_

#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/triangle_2d.h"

namespace cl {
namespace geometry {

/**
 * Project point 'p' on a line and return the projection point.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Line2D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point2D<T>& q = line.point();
    const Vector2D<T>& v0 = line.direction();
    Vector2D<T> v1 = p - q;

    T norm = v0.dot(v0);
    if (norm == 0) return q;

    T b = v0.dot(v1) / norm;
    return Point2D<T>(b * v0.x + q.x, b * v0.y + q.y);
}

/**
 * Project point 'p' on a line segment and return the projection point, i.e.,
 * the nearest point on line segment from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Segment2D<T>& seg) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point2D<T>& q = seg.lower_point();
    const Vector2D<T> v0 = seg.direction();
    Vector2D<T> v1 = p - q;

    T norm = v0.dot(v0);
    if (norm == 0) return q;

    T b = v0.dot(v1) / norm;
    Point2D<T> res(b * v0.x + q.x, b * v0.y + q.y);
    if (seg.lower_point() <= res && res <= seg.upper_point()) return res;
    return SquaredDistance(seg.lower_point(), res) <=
           SquaredDistance(seg.upper_point(), res) ? seg.lower_point()
                                                   : seg.upper_point();
}

/**
 * Project the point 'p' on a polyline. The result is the nearest point on the
 * polyline from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Polyline<Point2D<T>>& polyline) {
    if (polyline.empty()) return p;
    if (polyline.size() == 1) return polyline.front();
    int index = polyline_util::FindNearestSegment(polyline, p);
    return Project(p, Segment2D<T>(polyline[index], polyline[index + 1]));
}

/**
 * Project the point 'p' on a circle arc. The result is the nearest point on
 * circle from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Circle2D<T>& circle) {
    T radian = Radian(p - circle.center());
    return circle.GetPoint(radian);
}

/**
 * Project the point 'p' on a circular arc. The result is the nearest point on
 * arc from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Arc2D<T>& arc) {
    if (arc.orientation() == 0) {
        return Project(p, Segment2D<T>(arc.start_point(), arc.end_point()));
    }

    T radian = Radian(p - arc.center());
    if (arc.IsAngleBetween(radian)) {
        return arc.GetCircle().GetPoint(radian);
    } else {
        double d1 = Distance(p, arc.start_point());
        double d2 = Distance(p, arc.end_point());
        return d1 < d2 ? arc.start_point() : arc.end_point();
    }
}

/**
 * Project the point 'p' on a triangle. The result is the nearest point on
 * triangle from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Triangle2D<T>& triangle) {
    if (triangle.empty()) return p;

    Polyline<Point2D<T>> polyline(triangle.vertices());
    polyline.push_back(polyline.front());
    return Project(p, polyline);
}

/**
 * Project the point 'p' on a polygon. The result is the nearest point on
 * polygon from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Polygon2D<T>& polygon) {
    Polyline<Point2D<T>> polyline(polygon.vertices());
    polyline.push_back(polyline.front());
    return Project(p, polyline);
}
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const MultiPolygon2D<T>& polygon) {
    double dis = DBL_MAX;
    Point2D<T> best = p;
    for (const auto& b : polygon.boundaries()) {
        Point2D<T> q = Project(p, b.polygon);
        double d = Distance(p, q);
        if (d < dis) {
            dis = d;
            best = q;
        }
    }
    return best;
}

/**
 * Translate point.
 */
template <typename T>
Point2D<T> Translate(const Point2D<T>& point, const Vector2D<T>& v) {
    return point + v;
}

/**
 * Translate box according to the given vector.
 */
template <typename T>
Box2D<T> Translate(const Box2D<T>& box, const Vector2D<T>& v) {
    if (box.empty()) return box;

    return Box2D<T>(box.x_min() + v.x, box.x_max() + v.x,
                    box.y_min() + v.y, box.y_max() + v.y);
}

/**
 * Translate segment according to the given vector.
 */
template <typename T>
Segment2D<T> Translate(const Segment2D<T>& seg, const Vector2D<T>& v) {
    return Segment2D<T>(seg.lower_point() + v, seg.upper_point() + v);
}

/**
 * Translate line according to the given vector.
 */
template <typename T>
Line2D<T> Translate(const Line2D<T>& line, const Vector2D<T>& v) {
    return Line2D<T>(line.point() + v, line.direction());
}

/**
 * Translate circle.
 */
template <typename T>
Circle2D<T> Translate(const Circle2D<T>& circle, const Vector2D<T>& v) {
    return Circle2D<T>(circle.center() + v, circle.radius());
}

/**
 * Scale the bounding box.
 */
template <typename T>
Box2D<T> Scale(const Box2D<T>& box, const T& v) {
    if (box.empty()) return box;

    return Box2D<T>(std::min(box.x_min() * v, box.x_max() * v),
                    std::max(box.x_min() * v, box.x_max() * v),
                    std::min(box.y_min() * v, box.y_max() * v),
                    std::max(box.y_min() * v, box.y_max() * v));
}

/**
 * Scale the line segment.
 */
template <typename T>
Segment2D<T> Scale(const Segment2D<T>& seg, const T& v) {
    return Segment2D<T>(seg.lower_point() * v, seg.upper_point() * v);
}

/**
 * Counterclockwise rotate a point with 'a' radians.
 */
template <typename T>
Point2D<T> Rotate(const Point2D<T>& p, T a) {
    static_assert(std::is_floating_point<T>::value, "");

    T s = std::sin(a), c = std::cos(a);
    T x = c * p.x - s * p.y;
    T y = s * p.x + c * p.y;
    return Point2D<T>(x, y);
}

/**
 * Counterclockwise rotate polygon with 'a' radians.
 */
template <typename T>
Segment2D<T> Rotate(const Segment2D<T>& seg, T a) {
    static_assert(std::is_floating_point<T>::value, "");

    return Segment2D<T>(Rotate(seg.lower_point(), a),
                        Rotate(seg.upper_point(), a));
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_

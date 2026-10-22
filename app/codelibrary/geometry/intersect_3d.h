//
// Copyright 2014-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Robust intersection test between 3D kernel geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_INTERSECT_3D_H_
#define CODELIBRARY_GEOMETRY_INTERSECT_3D_H_

#include <algorithm>

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/multi_polygon_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/point_compare_2d.h"
#include "codelibrary/geometry/point_compare_3d.h"
#include "codelibrary/geometry/predicate_3d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/range_ray_3d.h"
#include "codelibrary/geometry/transform_3d.h"
#include "codelibrary/geometry/triangle_3d.h"

namespace cl {
namespace geometry {

////////////////////////////////////////////////////////////////////////////////
// All Intersect() functions are used to test weather two objects intersect
// (including touch and contain).
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication and construction.
 */
template <typename T>
bool Intersect(const Box3D<T>& box1, const Box3D<T>& box2,
               Box3D<T>* res = nullptr) {
    if (box1.x_max() < box2.x_min() || box1.x_min() > box2.x_max() ||
        box1.y_max() < box2.y_min() || box1.y_min() > box2.y_max() ||
        box1.z_max() < box2.z_min() || box1.z_min() > box2.z_max())
        return false;

    if (res) {
        *res = Box3D<T>(std::max(box1.x_min(), box2.x_min()),
                        std::min(box1.x_max(), box2.x_max()),
                        std::max(box1.y_min(), box2.y_min()),
                        std::min(box1.y_max(), box2.y_max()),
                        std::max(box1.z_min(), box2.z_min()),
                        std::min(box1.z_max(), box2.z_max()));
    }
    return true;
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point3D<T>& point, const Box3D<T>& box) {
    return (point.x >= box.x_min() && point.x <= box.x_max() &&
            point.y >= box.y_min() && point.y <= box.y_max() &&
            point.z >= box.z_min() && point.z <= box.z_max());
}
template <typename T>
bool Intersect(const Box3D<T>& box, const Point3D<T>& point) {
    return Intersect(point, box);
}

/**
 * Inexact predication.
 */
template <typename T>
bool Intersect(const Box3D<T>& box, const Plane3D<T>& plane) {
    static_assert(std::is_floating_point<T>::value, "");

    if (box.empty() || plane.empty()) return false;

    // Convert AABB to center-extents representation.
    Point3D<T> c = Center(box);
    // Compute positive extents
    Point3D<T> e(box.x_max() - c.x, box.y_max() - c.y, box.z_max() - c.z);

    const Vector3D<T>& n = plane.normal();
    const Point3D<T>& p = plane.point();

    // Compute the projection interval radius of box onto Line(t) = c + t * n.
    double r = static_cast<double>(e.x) * std::fabs(n.x) +
               static_cast<double>(e.y) * std::fabs(n.y) +
               static_cast<double>(e.z) * std::fabs(n.z);

    // Convert plane to the form of Ax + By + Cz + d = 0.
    double d = -static_cast<double>(n.x) * p.x -
                static_cast<double>(n.y) * p.y -
                static_cast<double>(n.z) * p.z;

    // Compute distance of box center from plane.
    double s = static_cast<double>(n.x) * c.x +
               static_cast<double>(n.y) * c.y +
               static_cast<double>(n.z) * c.z + d;

    // Intersection occurs when distance s falls within [-r,+r] interval.
    return std::fabs(s) <= r;
}
template <typename T>
bool Intersect(const Plane3D<T>& plane, const Box3D<T>& box) {
    return Intersect(box, plane);
}

/**
 * Inexact predication.
 *
 * Reference:
 *  Akenine-Möller, Tomas. "Fast 3D triangle-box overlap testing." ACM siggraph
 *  2005 courses. 2005. 8-es.
 */
template <typename T>
bool Intersect(const Box3D<T>& box, const Triangle3D<T>& triangle) {
    static_assert(std::is_floating_point<T>::value, "");

    if (box.empty() || triangle.empty()) return false;

    // First: test If the triangle is entirely inside the bounding box.
    if (Intersect(box, triangle.vertices()[0]) ||
        Intersect(box, triangle.vertices()[1]) ||
        Intersect(box, triangle.vertices()[2])) return true;

    // Convert AABB to center-extents form.
    Point3D<T> box_center = Center(box);
    Vector3D<T> box_halfsize(box.x_length() / 2, box.y_length() / 2,
                             box.z_length() / 2);

    // Move everything so that the boxcenter is in (0,0,0).
    Vector3D<T> v0 = triangle.vertices()[0] - box_center;
    Vector3D<T> v1 = triangle.vertices()[1] - box_center;
    Vector3D<T> v2 = triangle.vertices()[2] - box_center;

    // Compute the edge vectors of the triangle  (ABC)
    Vector3D<T> e0 = v1 - v0;
    Vector3D<T> e1 = v2 - v1;
    Vector3D<T> e2 = v0 - v2;

    T p0, p1, p2, d;

    // 9 tests for axis overlapping

    // Axis test x01
    Vector3D<T> fe(std::fabs(e0.x), std::fabs(e0.y), std::fabs(e0.z));
    p0 = e0.z * v0.y - e0.y * v0.z;
    p2 = e0.z * v2.y - e0.y * v2.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test y02
    p0 = -e0.z * v0.x + e0.x * v0.z;
    p2 = -e0.z * v2.x + e0.x * v2.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test z12
    p1 = e0.y * v1.x - e0.x * v1.y;
    p2 = e0.y * v2.x - e0.x * v2.y;
    d = fe.y * box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p1, p2) > d || std::max(p1, p2) < -d) return false;

    fe = Vector3D<T>(std::fabs(e1.x), std::fabs(e1.y), std::fabs(e1.z));
    // Axis test x01
    p0 = e1.z * v0.y - e1.y * v0.z;
    p2 = e1.z * v2.y - e1.y * v2.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test y02
    p0 = -e1.z * v0.x + e1.x * v0.z;
    p2 = -e1.z * v2.x + e1.x * v2.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test z0
    p0 = e1.y * v0.x - e1.x * v0.y;
    p1 = e1.y * v1.x - e1.x * v1.y;
    d = fe.y* box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    fe = Vector3D<T>(std::fabs(e2.x), std::fabs(e2.y), std::fabs(e2.z));

    // Axis test x02
    p0 = e2.z * v0.y - e2.y * v0.z;
    p1 = e2.z * v1.y - e2.y * v1.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    // Axis test y01
    p0 = -e2.z * v0.x + e2.x * v0.z;
    p1 = -e2.z * v1.x + e2.x * v1.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    // Axis test z12
    p1 = e2.y * v1.x - e2.x * v1.y;
    p2 = e2.y * v2.x - e2.x * v2.y;
    d = fe.y * box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p1, p2) > d || std::max(p1, p2) < -d) return false;

    // Test in X-direction.
    if (std::min(std::min(v0.x, v1.x), v2.x) >  box_halfsize.x ||
        std::max(std::max(v0.x, v1.x), v2.x) < -box_halfsize.x) return false;

    // Test in Y-direction.
    if (std::min(std::min(v0.y, v1.y), v2.y) >  box_halfsize.y ||
        std::max(std::max(v0.y, v1.y), v2.y) < -box_halfsize.y) return false;

    // Test in Z-direction.
    if (std::min(std::min(v0.z, v1.z), v2.z) >  box_halfsize.z ||
        std::max(std::max(v0.z, v1.z), v2.z) < -box_halfsize.z) return false;

    // Test if the box intersects the plane of the triangle.
    Plane3D<T> plane(triangle.vertices());
    return Intersect(plane, box);
}
template <typename T>
bool Intersect(const Triangle3D<T>& triangle, const Box3D<T>& box) {
    return Intersect(box, triangle);
}

////////////////////////////////////////////////////////////////////////////////
// All Cross() functions are used to test weather two objects are cross
// (excluding touch and contain).
//
// Exact construction requires high-precision computation, which can greatly
// slow down the speed. In this library, we completely avoid such operations.
////////////////////////////////////////////////////////////////////////////////
/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether two planes are cross is unnecessary in most cases.
 * Because it is very rare that two planes are strictly parallel.
 *
 * DO NOT use this function to get the intersection line between two nearly
 * parallel planes, it may produce unexpected line.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane1, const Plane3D<T>& plane2,
           Line3D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    if (plane1.empty() || plane2.empty()) return false;

    const Vector3D<T>& v1 = plane1.normal();
    const Vector3D<T>& v2 = plane2.normal();
    const Vector3D<T> u = v1.cross(v2);
    T as[3] = { std::fabs(u.x), std::fabs(u.y), std::fabs(u.z) };

    // First, determine the maximum absolute value of cross production.
    int maxc = static_cast<int>(std::max_element(as + 0, as + 3) - as);

    // Next, compute a point on the intersection line.
    Point3D<T> p;
    T d1 = -(v1.dot(plane1.point().ToVector()));
    T d2 = -(v2.dot(plane2.point().ToVector()));

    switch (maxc) {
    case 0: // Intersected with plane x = 0.
        p.x = 0;
        p.y = (d2 * v1.z - d1 * v2.z) / u.x;
        p.z = (d1 * v2.y - d2 * v1.y) / u.x;
        break;
    case 1: // Intersected with plane y = 0.
        p.x = (d1 * v2.z - d2 * v1.z) / u.y;
        p.y = 0;
        p.z = (d2 * v1.x - d1 * v2.x) / u.y;
        break;
    case 2: // Intersected with plane z = 0.
        p.x = (d2 * v1.y - d1 * v2.y) / u.z;
        p.y = (d1 * v2.x - d2 * v1.x) / u.z;
        p.z = 0;
    }

    if (res) *res = Line3D<T>(p, u);
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/**
 * Exact predicate whether a line segment is cross with a plane determined by
 * three points.
 */
template <typename T>
bool Cross(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c,
           const Segment3D<T>& seg) {
    int o1 = Orientation(a, b, c, seg.lower_point());
    if (o1 == 0) return false;
    int o2 = Orientation(a, b, c, seg.upper_point());
    if (o2 == 0) return false;
    return o1 != o2;
}

/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether a plane cross with a line is unnecessary in most
 * cases. Because it is very rare for a plane and a line to be strictly
 * parallel.
 *
 * If there is a real need, it is better to convert lines into long line
 * segments and then use the above function.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane, const Line3D<T>& line,
           Point3D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RangeRay3D<T> rray(line);
    return rray.Hit(plane, res);
}
template <typename T>
bool Cross(const Line3D<T>& line, const Plane3D<T>& plane,
           Point3D<T>* res = nullptr) {
    return Cross(plane, line, res);
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane, const Ray3D<T>& ray,
           Point3D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RangeRay3D<T> rray(ray);
    return rray.Hit(plane, res);
}
template <typename T>
bool Cross(const Ray3D<T>& ray, const Plane3D<T>& plane,
           Point3D<T>* res = nullptr) {
    return Cross(plane, ray, res);
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane, const Segment3D<T>& seg,
           Point3D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RangeRay3D<T> rray(seg);
    return rray.Hit(plane, res);
}
template <typename T>
bool Cross(const Segment3D<T>& seg, const Plane3D<T>& plane,
           Point3D<T>* res = nullptr) {
    return Cross(plane, seg, res);
}

/**
 * Exact predication and inexact construction.
 *
 * Exact predicate whether a polygon is cross with a Z plane.
 *
 * It optional outputs the unordered intersection points.
 */
template <typename T>
bool Cross(const Polygon3D<T>& polygon, T z, Array<Point3D<T>>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    if (res) res->clear();

    if (polygon.bounding_box().z_min() > z ||
        polygon.bounding_box().z_max() < z) return false;

    if (polygon.empty()) return false;
    if (!Cross(polygon.plane(), Plane3D<T>(Point3D<T>(0, 0, z),
                                           Vector3D<T>(0, 0, 1)))) {
        return false;
    }

    using Point = Point3D<T>;
    const auto& contour = polygon.vertices();

    for (int i = 0; i < contour.size(); ++i) {
        int next = i + 1 == contour.size() ? 0 : i + 1;
        Point p1 = contour[i];
        Point p2 = contour[next];
        int o1 = p1.z <= z ? 0 : 1;
        int o2 = p2.z <= z ? 0 : 1;

        if (o1 != o2) {
            if (res) {
                T t = (z - p1.z) / (p2.z - p1.z);
                res->emplace_back(p1.x + t * (p2.x - p1.x),
                                  p1.y + t * (p2.y - p1.y),
                                  z);
            } else {
                return true;
            }
        }
    }

    return res ? !res->empty() : false;
}
template <typename T>
bool Cross(const Polygon3D<T>& polygon, T z, Array<Segment3D<T>>* res) {
    CHECK(res);

    res->clear();

    if (polygon.bounding_box().z_min() > z ||
        polygon.bounding_box().z_max() < z) return false;

    Line3D<T> line;
    if (!Cross(polygon.plane(), Plane3D<T>(Point3D<T>(0, 0, z),
                                           Vector3D<T>(0, 0, 1)),
               &line)) {
        return false;
    }

    Array<Point3D<T>> points;
    if (!Cross(polygon, z, &points)) return false;
    CHECK(points.size() % 2 == 0);

    PointDotCompare3D<T> compare(line.point1(), line.point2());
    std::sort(points.begin(), points.end(), compare);

    res->clear();
    for (int i = 0; i + 1 < points.size(); i += 2) {
        res->emplace_back(points[i], points[i + 1]);
    }
    return !res->empty();
}

/**
 * Exact predication and inexact construction.
 *
 * Exact predicate whether a polygon is cross with a Z plane.
 */
template <typename T>
bool Cross(const MultiPolygon3D<T>& polygon, T z,
           Array<Point3D<T>>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    if (res) res->clear();

    if (polygon.bounding_box().z_min() > z ||
        polygon.bounding_box().z_max() < z) return false;

    for (const auto& contour : polygon.vertices()) {
        Polygon3D<T> poly(contour, polygon.plane());
        if (res) {
            Array<Point3D<T>> points;
            Cross(poly, z, &points);
            res->insert(points);
        } else {
            if (Cross(poly, z)) return true;
        }
    }

    return res ? !res->empty() : false;
}
template <typename T>
bool Cross(const MultiPolygon3D<T>& polygon, T z, Array<Segment3D<T>>* res) {
    CHECK(res);

    res->clear();

    Line3D<T> line;
    if (!Cross(polygon.plane(), Plane3D<T>(Point3D<T>(0, 0, z),
                                           Vector3D<T>(0, 0, 1)),
               &line)) {
        return false;
    }

    Array<Point3D<T>> points;
    if (!Cross(polygon, z, &points)) return false;
    CHECK(points.size() % 2 == 0);

    PointDotCompare3D<T> compare(line.point1(), line.point2());
    std::sort(points.begin(), points.end(), compare);

    for (int i = 0; i + 1 < points.size(); i += 2) {
        res->emplace_back(points[i], points[i + 1]);
    }
    return !res->empty();
}

/**
 * Inexact predication and construction.
 *
 * It also outputs the ordered intersection points.
 */
template <typename T>
bool Cross(const MultiPolygon3D<T>& polygon1, const MultiPolygon3D<T>& polygon2,
           Array<Point3D<T>>* res) {
    CHECK(res);

    res->clear();

    if (!Intersect(polygon1.bounding_box(), polygon2.bounding_box()))
        return false;

    Line3D<T> line;
    if (!Cross(polygon1.plane(), polygon2.plane(), &line)) {
        return false;
    }

    using Point = Point3D<T>;

    const Pad3D<T>& pad = polygon1.pad();
    T z = pad.z_offset();
    Quaternion<T> rot = pad.rotation();

    Point plane_p = Rotate(polygon2.plane().point(), rot);
    Vector3D<T> v = rot * polygon2.plane().normal();
    Plane3D<T> plane_after_rotation(plane_p, v);

    // The intersection points betweeen polygon2 and the plane of polygon1.
    // The points are projected to the plane.
    Array<Point2D<T>> points1;
    for (const auto& contour : polygon2.vertices()) {
        Array<Point> contour_after_rotation(contour.size());
        for (int i = 0; i < contour.size(); ++i) {
            contour_after_rotation[i] = Rotate(contour[i], rot);
        }
        Array<RPoint3D> points;
        Polygon3D<T> poly(contour_after_rotation, plane_after_rotation);
        if (Cross(poly, z, &points)) {
            CHECK(points.size() % 2 == 0);
            for (auto p : points) {
                points1.emplace_back(p.x, p.y);
            }
        }
    }
    if (points1.empty()) return false;

    // Get the intersection points between polygon1 and the plane of polygon2.
    Array<Point2D<T>> points2;
    Point2D<T> p1 = pad.Project(line.point1());
    Point2D<T> p2 = pad.Project(line.point2());
    Line2D<T> line_2d(p1, p2);
    if (!Intersect(line_2d, polygon1.projected_polygon(), &points2))
        return false;

    CHECK(points1.size() % 2 == 0);
    CHECK(points2.size() % 2 == 0);

    PointDotCompare2D<T> compare(p1, p2);
    std::sort(points1.begin(), points1.end(), compare);
    std::sort(points2.begin(), points2.end(), compare);

    // Compute the intersection between two line segements.
    Array<Point2D<T>> intersection;
    int i = 0, j = 0;
    while (i < points1.size() && j < points2.size()) {
        Point2D<T> a1 = points1[i], a2 = points1[i + 1];
        Point2D<T> b1 = points2[j], b2 = points2[j + 1];

        // b2 >= a1 and a2 >= b1
        if (!compare(b2, a1) && !compare(a2, b1)) {
            if (compare(a1, b1)) {
                intersection.push_back(b1);
            } else {
                intersection.push_back(a1);
            }

            if (compare(a2, b2)) {
                intersection.push_back(a2);
            } else {
                intersection.push_back(b2);
            }
        }

        if (compare(b2, a1))
            j += 2;
        else
            i += 2;
    }

    for (const auto& p : intersection) {
        res->push_back(pad.Unproject(p));
    }
    return !res->empty();
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Triangle3D<T>& t1, const Triangle3D<T>& t2,
           Segment3D<T>* res = nullptr) {
    Plane3D<T> plane1(t1.vertices()[0], t1.normal());
    Plane3D<T> plane2(t2.vertices()[0], t2.normal());

    Line3D<T> line;
    if (!Cross(plane1, plane2, &line))
        return false;

    Vector3D<T> u = line.direction();
    T min1 = std::numeric_limits<T>::max();
    T max1 = std::numeric_limits<T>::lowest();
    for (int i = 0; i < 3; ++i) {
        int a = i, b = i == 2 ? 0 : i + 1;
        Point3D<T> p;

        if (Cross(plane2, Segment3D<T>(t1.vertices()[a], t1.vertices()[b]),
                  &p)) {
            T t = u.dot(p - line.point());
            min1 = std::min(min1, t);
            max1 = std::max(max1, t);
        }
    }
    if (min1 >= max1)
        return false;

    T min2 = std::numeric_limits<T>::max();
    T max2 = std::numeric_limits<T>::lowest();
    for (int i = 0; i < 3; ++i) {
        int a = i, b = i == 2 ? 0 : i + 1;
        Point3D<T> p;
        if (Cross(plane1, Segment3D<T>(t2.vertices()[a], t2.vertices()[b]),
                  &p)) {
            T t = u.dot(p - line.point());
            min2 = std::min(min2, t);
            max2 = std::max(max2, t);
        }
    }
    if (min2 >= max2)
        return false;

    min1 = std::max(min1, min2);
    max1 = std::min(max1, max2);
    if (min1 >= max1)
        return false;

    if (res) {
        T squared_norm = u.squared_norm();
        *res = Segment3D<T>(line.point() + min1 / squared_norm * u,
                            line.point() + max1 / squared_norm * u);
    }

    return true;
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_INTERSECT_3D_H_

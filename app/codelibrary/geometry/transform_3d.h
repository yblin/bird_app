//
// Copyright 2019-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_
#define CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_

#include <limits>

#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/geometry/sphere_3d.h"
#include "codelibrary/math/matrix/matrix4.h"

namespace cl {
namespace geometry {

/**
 * Return the orthogonal projection of a point 'p' on a line.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Line3D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point3D<T>& q = line.point();
    const Vector3D<T>& v = line.direction();

    Vector3D<T> v0(v.x, v.y, v.z);
    T norm = v0.dot(v0);
    if (norm == 0) return line.point();

    Vector3D<T> v1 = p - q;

    T b = v0.dot(v1) / norm;
    return Point3D<T>(b * v0.x + q.x, b * v0.y + q.y, b * v0.z + q.z);
}

/**
 * Return the projection point (i.e., the nearest point) from point 'p' to line
 * segment 'seg'.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Segment3D<T>& seg) {
    static_assert(std::is_floating_point<T>::value, "");

    Point3D<T> q = Project(p, Line3D<T>(seg.lower_point(), seg.upper_point()));
    if ((q - seg.lower_point()).dot(q - seg.upper_point()) < 0) {
        return q;
    }

    return Distance(p, seg.lower_point()) < Distance(p, seg.upper_point()) ?
               seg.lower_point() : seg.upper_point();
}

/**
 * Return the orthogonal projection of a point 'p' on a plane.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Plane3D<T>& plane) {
    static_assert(std::is_floating_point<T>::value, "");

    Line3D<T> line(p, plane.normal());
    return Project(plane.point(), line);
}

/**
 * Return the projection of a point 'p' on a sphere.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Sphere3D<T>& sphere) {
    static_assert(std::is_floating_point<T>::value, "");

    T radius = sphere.radius();

    if (p == sphere.center()) {
        // If p is the center of sphere, return the point that has with maximal
        // Z value.
        return sphere.center() + Vector3D<T>(0, 0, radius);
    }

    Vector3D<T> v = p - sphere.center();
    v *= radius / v.norm();

    return sphere.center() + v;
}

/**
 * Project line1 onto line2. The result is a 3D point.
 */
template <typename T>
Point3D<T> Project(const Line3D<T>& line1, const Line3D<T>& line2) {
    static_assert(std::is_floating_point<T>::value, "");

    RVector3D v1(line1.direction().x, line1.direction().y,
                 line1.direction().z);
    RVector3D v2(line2.direction().x, line2.direction().y,
                 line2.direction().z);

    RVector3D v = v1.cross(v2);
    double norm = v.norm();

    if (norm < std::numeric_limits<T>::epsilon()) {
        return Project(line1.point(), line2);
    }

    RVector3D ab(static_cast<double>(line1.point().x) - line2.point().x,
                 static_cast<double>(line1.point().y) - line2.point().y,
                 static_cast<double>(line1.point().z) - line2.point().z);
    double t = -ab.cross(v1).dot(v);
    t /= norm * norm;
    return line2.point() + line2.direction() * static_cast<T>(t);
}

/**
 * Translate point.
 */
template <typename T>
Point3D<T> Translate(const Point3D<T>& point, const Vector3D<T>& v) {
    return point + v;
}

/**
 * Translate box according to the given vector.
 */
template <typename T>
Box3D<T> Translate(const Box3D<T>& box, const Vector3D<T>& v) {
    if (box.empty()) return box;

    return Box3D<T>(box.x_min() + v.x, box.x_max() + v.x,
                    box.y_min() + v.y, box.y_max() + v.y,
                    box.z_min() + v.z, box.z_max() + v.z);
}

/**
 * Translate segment according to the given vector.
 */
template <typename T>
Segment3D<T> Translate(const Segment3D<T>& seg, const Vector3D<T>& v) {
    return Segment3D<T>(Translate(seg.lower_point(), v),
                        Translate(seg.upper_point(), v));
}

/**
 * Translate line according to the given vector.
 */
template <typename T>
Line3D<T> Translate(const Line3D<T>& line, const Vector3D<T>& v) {
    return Line3D<T>(line.point() + v, line.direction());
}

/**
 * Translate sphere.
 */
template <typename T>
Sphere3D<T> Translate(const Sphere3D<T>& sphere, const Vector3D<T>& v) {
    return Sphere3D<T>(sphere.center() + v, sphere.radius());
}

/**
 * Translate plane.
 */
template <typename T>
Plane3D<T> Translate(const Plane3D<T>& plane, const Vector3D<T>& v) {
    return Plane3D<T>(plane.point() + v, plane.normal());
}

/**
 * Rotate the given point.
 */
template <typename T>
Point3D<T> Rotate(const Point3D<T>& point, const Quaternion<T>& rotation) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>(rotation * point.ToVector());
}

/**
 * Rotate the given plane.
 */
template <typename T>
Plane3D<T> Rotate(const Plane3D<T>& plane, const Quaternion<T>& rotation) {
    static_assert(std::is_floating_point<T>::value, "");

    return Plane3D<T>(Rotate(plane.point(), rotation),
                      rotation * plane.normal());
}

/**
 * Rotate the given segment.
 */
template <typename T>
Segment3D<T> Rotate(const Segment3D<T>& seg, const Quaternion<T>& rotation) {
    return Segment3D<T>(Rotate(seg.lower_point(), rotation),
                        Rotate(seg.upper_point(), rotation));
}

/**
 * Rotate the given line.
 */
template <typename T>
Line3D<T> Rotate(const Line3D<T>& line, const Quaternion<T>& rotation) {
    return Line3D<T>(Rotate(line.point(), rotation),
                     rotation * line.direction());
}

/**
 * Scale the box.
 */
template <typename T>
Box3D<T> Scale(const Box3D<T>& box, T scale) {
    if (box.empty()) return box;

    return Box3D<T>(box.x_min() * scale, box.x_max() * scale,
                    box.y_min() * scale, box.y_max() * scale,
                    box.z_min() * scale, box.z_max() * scale);
}

/**
 * Rigid transform: x = Rx + T.
 */
template <typename T>
Point3D<T> Rigid(const Point3D<T>& p, const Matrix4<T>& mat) {
    Vector4D<T> v(p.x, p.y, p.z, 1.0);
    v = mat * v;
    return {v[0], v[1], v[2]};
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_

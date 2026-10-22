//
// Copyright 2023-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_3D_H_
#define CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_3D_H_

#include <algorithm>

#include "codelibrary/geometry/circle_3d.h"
#include "codelibrary/geometry/envelope/min_enclosing_circle_2d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/transform_3d.h"

namespace cl {
namespace geometry {

/**
 * Computing the smallest enclosing circle for the projected points on the
 * specified plane.
 */
template <typename T>
void MinEnclosingCircle3D(const Array<Point3D<T>>& points,
                          const Plane3D<T>& plane,
                          Circle3D<T>* circle) {
    CHECK(circle);

    Quaternion<T> rotation(plane.normal(), Vector3D<T>(0, 0, 1));
    Point3D<T> p = Rotate(plane.point(), rotation);
    T z_offset = p.z;

    Array<Point2D<T>> points_2d;
    for (auto p : points) {
        Point3D<T> q = Rotate(p, rotation);
        points_2d.emplace_back(q.x, q.y);
    }
    Circle2D<T> circle_2d;
    MinEnclosingCircle2D(points_2d, &circle_2d);

    Point3D<T> q(circle_2d.center().x, circle_2d.center().y, z_offset);
    *circle = Circle3D<T>(Rotate(q, rotation.Inverse()), plane.normal(),
                          circle_2d.radius());
}

/**
 * Computes the smallest enclosing circle for the projected points on the
 * specified plane defined by its center 'c' and normal vector.
 *
 * Note that, the center of circle, i.e., 'c', is already provided.
 */
template <typename T>
void MinEnclosingCircle3D(const Array<Point3D<T>>& points,
                          const Point3D<T>& center,
                          const Vector3D<T>& normal,
                          Circle3D<T>* circle) {
    CHECK(circle);

    Plane3D<T> plane(center, normal);
    T radius = 0;
    for (auto p : points) {
        p = Project(p, plane);
        radius = std::max(radius, Distance(p, center));
    }

    *circle = Circle3D<T>(center, normal, radius);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_3D_H_

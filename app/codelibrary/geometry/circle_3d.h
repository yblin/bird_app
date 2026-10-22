//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CIRCLE_3D_H_
#define CODELIBRARY_GEOMETRY_CIRCLE_3D_H_

#include "codelibrary/base/log.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * 3D circle i.e., 2D circle on the specific 3D plane.
 */
template <typename T>
class Circle3D {
    static_assert(std::is_floating_point<T>::value,
                  "We only allow floating point type for 3D circle.");

public:
    using value_type = T;

    Circle3D() = default;

    Circle3D(const Point3D<T>& center, const Vector3D<T>& normal, T radius)
        : normal_(normal), center_(center), radius_(radius) {
        CHECK(radius_ >= 0);
    }

    /**
     * Flip the normal of the circle.
     */
    void Flip() {
        normal_ = -normal_;
    }

    /**
     * Return true if the circle is invalid.
     */
    bool empty() const {
        return normal_ == Vector3D<T>(0, 0, 0);
    }

    /**
     * The radius of the circle.
     */
    T radius() const {
        return radius_;
    }

    void set_radius(const T& radius) {
        CHECK(radius >= 0);

        radius_ = radius;
    }

    void set_center(const Point3D<T>& center) {
        center_ = center;
    }

    void set_normal(const Vector3D<T>& normal) {
        normal_ = normal;
    }

    /**
     * The center of the circle.
     */
    const Point3D<T>& center() const {
        return center_;
    }

    /**
     * The normal vector of 3D circle.
     */
    const Vector3D<T>& normal() const {
        return normal_;
    }

    /**
     * Plane of circle.
     */
    Plane3D<T> plane() const {
        return Plane3D<T>(center_, normal_);
    }

protected:
    Vector3D<T> normal_;    // The normal vector of 3D circle.
    Point3D<T> center_;     // The center of circle.
    T radius_ = 0;          // The radius of circle.
};

using FCircle3D = Circle3D<float>;
using RCircle3D = Circle3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CIRCLE_3D_H_

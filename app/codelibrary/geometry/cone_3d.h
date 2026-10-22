//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CONE_3D_H_
#define CODELIBRARY_GEOMETRY_CONE_3D_H_

#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * Solid cone.
 */
template <typename T>
class Cone3D {
    using Point = Point3D<T>;

public:
    Cone3D() = default;

    Cone3D(const Point& tip, const Point& base_center, const T& base_radius)
        : tip_(tip), base_center_(base_center), base_radius_(base_radius) {
        CHECK(base_radius_ >= 0);
    }

    const Point& tip() const {
        return tip_;
    }

    const Point& base_center() const {
        return base_center_;
    }

    const T& base_radius() const {
        return base_radius_;
    }

private:
    // Cone tip.
    Point tip_;

    // Cone base center point.
    Point base_center_;

    // Cone base radius.
    T base_radius_ = 0;
};

using ICone3D = Cone3D<int>;
using FCone3D = Cone3D<float>;
using RCone3D = Cone3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CONE_3D_H_

//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CYLINDER_3D_H_
#define CODELIBRARY_GEOMETRY_CYLINDER_3D_H_

#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * Solid cylinder.
 */
template <typename T>
class Cylinder3D {
public:
    Cylinder3D() = default;

    Cylinder3D(const Point3D<T>& p1, const Point3D<T>& p2, const T& radius)
        : point1_(p1), point2_(p2), radius_(radius) {
        CHECK(radius >= 0);
    }

    const Point3D<T>& point1() const {
        return point1_;
    }

    const Point3D<T>& point2() const {
        return point2_;
    }

    const T& radius() const {
        return radius_;
    }

    Box3D<T> bounding_box() const {
        Point3D<T> ps[2] = {point1_, point2_};
        Box3D<T> box(ps + 0, ps + 2);
        box.set_x_range(box.x_min() - radius_, box.x_max() + radius_);
        box.set_y_range(box.y_min() - radius_, box.y_max() + radius_);
        box.set_z_range(box.z_min() - radius_, box.z_max() + radius_);
        return box;
    }

private:
    // Two end points of cylinder.
    Point3D<T> point1_, point2_;

    T radius_ = 0;
};

using ICylinder3D = Cylinder3D<int>;
using FCylinder3D = Cylinder3D<float>;
using RCylinder3D = Cylinder3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CYLINDER_3D_H_

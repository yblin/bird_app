//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_PAD_3D_H_
#define CODELIBRARY_GEOMETRY_UTIL_PAD_3D_H_

#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/transform_3d.h"

namespace cl {
namespace geometry {

/**
 * Used to convert 3D planar points to 2D points.
 */
template <typename T>
class Pad3D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    /**
     * Construct an empty pad.
     *
     * Users can use Reset() to initialize the pad latter.
     */
    Pad3D() = default;

    explicit Pad3D(const Plane3D<T>& plane) {
        ResetPlane(plane);
    }

    /**
     * Reset pad's plane.
     */
    void ResetPlane(const Plane3D<T>& plane) {
        if (plane.empty()) {
            plane_ = Plane3D<T>();
            rotation_ = Quaternion<T>();
            z_offset_ = 0;
            return;
        }

        plane_ = plane;
        rotation_ = Quaternion<T>(plane.normal(), Vector3D<T>(0, 0, 1));
        Point3D<T> p = Rotate(plane.point(), rotation_);
        z_offset_ = p.z;
    }

    /**
     * Project a point in world space to local plane space.
     */
    Point2D<T> Project(const Point3D<T>& p) const {
        Point3D<T> q = Rotate(p, rotation_);
        return Point2D<T>(q.x, q.y);
    }

    /**
     * Unproject a point in local plane space to world space.
     */
    Point3D<T> Unproject(const Point2D<T>& p) const {
        Point3D<T> q(p.x, p.y, z_offset_);
        return Rotate(q, rotation_.Inverse());
    }

    const Plane3D<T>& plane() const {
        return plane_;
    }

    const Quaternion<T>& rotation() const {
        return rotation_;
    }

    T z_offset() const {
        return z_offset_;
    }

protected:
    // The plane for arrangement.
    Plane3D<T> plane_;

    // Rotation from plane's normal to XY plane.
    Quaternion<T> rotation_;

    // z_offset is used to unproject local plane coordiante to world coordinate.
    T z_offset_ = 0;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_PAD_3D_H_

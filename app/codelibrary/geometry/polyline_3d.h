//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POLYLINE_3D_H_
#define CODELIBRARY_GEOMETRY_POLYLINE_3D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * 3D polyline. It is implemented like Array<Point3D>.
 */
template <typename T>
class Polyline3D {
    using Point = Point3D<T>;

public:
    Polyline3D() = default;

    explicit Polyline3D(const Array<Point>& points)
        : vertices_(points) {}

    bool empty() const {
        return vertices_.empty();
    }

    void clear() {
        vertices_.clear();
    }

    /**
     * Equal to Array's push_back.
     */
    void push_back(const Point& p) {
        vertices_.push_back(p);
    }

    /**
     * Equal to Array's emplace_back().
     */
    template <typename... Args>
    void emplace_back(Args&&... args) {
        vertices_.emplace_back(std::forward<Args>(args)...);
    }

    const Array<Point>& vertices() const {
        return vertices_;
    }

    int size() const {
        return vertices_.size();
    }

protected:
    Array<Point> vertices_;
};

using IPolyline3D = Polyline3D<int>;
using FPolyline3D = Polyline3D<float>;
using RPolyline3D = Polyline3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POLYLINE_3D_H_

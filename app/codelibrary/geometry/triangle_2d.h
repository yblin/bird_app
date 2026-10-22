//
// Copyright 2018-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_
#define CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {

/**
 * 2D triangle.
 */
template <typename T>
class  Triangle2D {
public:
    Triangle2D() = default;

    Triangle2D(const Point2D<T>& p1, const Point2D<T>& p2,
               const Point2D<T>& p3)
        : vertices_(3) {
        vertices_[0] = p1;
        vertices_[1] = p2;
        vertices_[2] = p3;
    }

    explicit Triangle2D(const Array<Point2D<T>>& vertices)
        : vertices_(vertices) {
        CHECK(vertices.size() == 3);
    }

    /**
     * Return the area of the triangle.
     */
    double Area() {
        if (empty()) return 0.0;

        double t1 = (static_cast<double>(vertices_[0].x) - vertices_[2].x) *
                    (static_cast<double>(vertices_[1].y) - vertices_[0].y);
        double t2 = (static_cast<double>(vertices_[0].x) - vertices_[1].x) *
                    (static_cast<double>(vertices_[2].y) - vertices_[0].y);
        return 0.5 * std::fabs(t1 - t2);
    }

    Segment2D<T> edge(int i) const {
        CHECK(!empty() && i >= 0 && i < 3);

        return i == 0 ? Segment2D<T>(vertices_[0], vertices_[1])
                      : i == 1 ? Segment2D<T>(vertices_[1], vertices_[2])
                               : Segment2D<T>(vertices_[2], vertices_[0]);
    }

    const Array<Point2D<T>>& vertices() const {
        return vertices_;
    }

    Box2D<T> bounding_box() const {
        return Box2D<T>(vertices_.begin(), vertices_.end());
    }

    bool empty() const {
        return vertices_.empty();
    }

private:
    Array<Point2D<T>> vertices_;
};

using ITriangle2D = Triangle2D<int>;
using FTriangle2D = Triangle2D<float>;
using RTriangle2D = Triangle2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_

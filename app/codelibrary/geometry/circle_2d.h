//
// Copyright 2012-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CIRCLE_2D_H_
#define CODELIBRARY_GEOMETRY_CIRCLE_2D_H_

#include "codelibrary/base/log.h"
#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {

/**
 * 2D circle.
 */
template <typename T>
class Circle2D {
public:
    using value_type = T;

    Circle2D() = default;

    Circle2D(const Point2D<T>& center, const T& radius)
        : center_(center), radius_(radius) {
        CHECK(radius_ >= 0) << radius_;
    }

    Circle2D(const T& x, const T& y, const T& radius)
        : Circle2D(Point2D<T>(x, y), radius) {}

    /**
     * Get the point on the circle at the given angle (in radians).
     */
    Point2D<T> GetPoint(T angle) const {
        return Point2D<T>(center_.x + std::cos(angle) * radius_,
                          center_.y + std::sin(angle) * radius_);
    }

    /**
     * Return the tangent vector at the given radian angle.
     * Assume the circle is in clockwise orientation.
     */
    Vector2D<T> tangent(T radian) const {
        return Vector2D<T>{std::sin(radian), -std::cos(radian)};
    }

    /**
     * Return true if the circle is invalid.
     */
    bool empty() const {
        return radius_ == 0;
    }

    /**
     * The radius of the circle.
     */
    const T& radius() const {
        return radius_;
    }

    /**
     * Update radius.
     */
    void set_radius(const T& radius) {
        CHECK(radius >= 0);

        radius_ = radius;
    }

    /**
     * The bounding box of the circle.
     */
    const Box2D<T>& bounding_box() const {
        CHECK(std::isfinite(radius_));

        return Box2D<T>(center_.x - radius_, center_.x + radius_,
                        center_.y - radius_, center_.y + radius_);
    }

    /**
     * The center of the circle.
     */
    const Point2D<T>& center() const {
        return center_;
    }

    /**
     * Set center point of the circle.
     */
    void set_center(const Point2D<T>& p) {
        center_ = p;
    }

    friend std::ostream& operator <<(std::ostream& os, const Circle2D& circle) {
        Message msg(circle.center(), circle.radius());
        return os << msg;
    }

protected:
    Point2D<T> center_;     // The center of circle.
    T radius_ = 0;          // The radius of circle.
};

using ICircle2D = Circle2D<int>;
using FCircle2D = Circle2D<float>;
using RCircle2D = Circle2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CIRCLE_2D_H_

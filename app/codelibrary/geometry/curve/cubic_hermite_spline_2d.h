//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CURVE_CUBIC_HERMITE_SPLINE_2D_H_
#define CODELIBRARY_GEOMETRY_CURVE_CUBIC_HERMITE_SPLINE_2D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {

/**
 * Cubic Hermite spline is a spline where each piece is a third-degree
 * polynomial specified in Hermite form, that is, by its values and first
 * derivatives at the end points of the corresponding domain interval.
 */
template <typename T>
class CubicHermiteSpline2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;
    using Vector = Vector2D<T>;

public:
    CubicHermiteSpline2D() = default;

    /**
     * On the unit interval [0, 1], given a starting point p0 at t = 0 and an
     * ending point p1 at t = 1 with starting tangent m0 at t = 0 and ending
     * tangent m1 at t = 1, the cubic hermite spline can be defined by:
     *
     * p(t) = (2t^3 - 3t^2 + 1) p_0 + (t^3 - 2t^2 + t) m_0 +
     *        (-2t^3 + 3t^2) p_1    + (t^3 - t^2) m_1
     */
    CubicHermiteSpline2D(const Point& start_point, const Vector& start_tangent,
                         const Point& end_point, const Vector& end_tangent)
        : start_point_(start_point),
          end_point_(end_point),
          start_tangent_(start_tangent),
          end_tangent_(end_tangent) {}

    bool empty() const {
        return start_point_ == end_point_;
    }

    /**
     * Sample the point on curve at ratio t.
     */
    Point Sample(T t) const {
        CHECK(!empty());
        CHECK(t >= 0 && t <= 1);

        if (t == 0) return start_point_;
        if (t == 1) return end_point_;

        const T t2 = t * t;
        const T t3 = t2 * t;
        Point p1 = start_point_   * (2 * t3 - 3 * t2 + 1) +
                   start_tangent_ * (t3 - 2 * t2 + t);
        Point p2 = end_point_     * (-2 * t3 + 3 * t2) +
                   end_tangent_   * (t3 - t2);
        return p1 + p2.ToVector();
    }

    /**
     * Uniformly sample n curve points. n should be no less than 2.
     *
     * Note that the sample points may not be uniformly distributed.
     */
    void Sample(int n, Array<Point>* points) const {
        CHECK(n >= 2);
        CHECK(!empty());
        CHECK(points);

        points->clear();
        for (int i = 0; i < n; ++i) {
            T t = static_cast<T>(i) / (n - 1);
            points->push_back(Sample(t));
        }
    }

    /**
     * Get the tangent vector on curve at ratio t.
     */
    Vector GetTangent(T t) const {
        CHECK(!empty());
        CHECK(t >= 0 && t <= 1) << t;

        const T t2 = t * t;
        return (start_point_ - end_point_) * (6 * (t2 - t)) +
                start_tangent_ * (3 * t2 - 4 * t + 1) +
                end_tangent_   * (3 * t2 - 2 * t);
    }

    /**
     * Reverse the curve.
     */
    void Reverse() {
        std::swap(start_point_, end_point_);
        std::swap(start_tangent_, end_tangent_);
        start_tangent_ = -start_tangent_;
        end_tangent_ = -end_tangent_;
    }

    const Point& start_point() const {
        return start_point_;
    }

    const Point& end_point() const {
        return end_point_;
    }

    const Vector& start_tangent() const {
        return start_tangent_;
    }

    const Vector& end_tangent() const {
        return end_tangent_;
    }

protected:
    Point start_point_, end_point_;
    Vector start_tangent_, end_tangent_;
};

using FCubicHermiteSpline2D = CubicHermiteSpline2D<float>;
using RCubicHermiteSpline2D = CubicHermiteSpline2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CURVE_CUBIC_HERMITE_SPLINE_2D_H_

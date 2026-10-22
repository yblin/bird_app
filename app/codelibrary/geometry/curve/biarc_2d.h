//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CURVE_BIARC_2D_H_
#define CODELIBRARY_GEOMETRY_CURVE_BIARC_2D_H_


#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/center.h"

namespace cl {

/**
 * A biarc is a smooth curve formed from two circular arcs.
 */
template <typename T>
class Biarc2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using Arc = Arc2D<T>;
    using Point = Point2D<T>;
    using Vector = Vector2D<T>;

    Biarc2D() = default;

    /**
     * This choice of three points and two tangents determines a unique pair of
     * circular arcs.
     *
     * In order to make the biarc smooth (G1 continuous), the two arcs should
     * have the same tangent at the control point where they meet.
     * Since we set the control point as a parameter, the user can decide
     * whether to make the biarc smooth or not.
     *
     * To make biarc to smooth, see GetTransitionArc() for more detils.
     */
    Biarc2D(const Point& start, const Vector& start_tangent,
            const Point& end, const Vector& end_tangent,
            const Point& control)
        : start_tangent_(start_tangent), end_tangent_(end_tangent) {
        arc1_ = Arc(start, start_tangent, control);
        arc2_ = Arc(control, end, end_tangent);
    }

    /**
     * Note that all the control points that make the biarc smooth can form a
     * circle, which we call it transition circle.
     *
     * Reference:
     *   Chandrupatla, Tirupathi R., and Thomas J. Osler. "Planar Biarc Curves–A
     *   Geometric View." The Mathematical Scientist 36.01 (2011).
     */
    Arc GetTransitionArc() const {
        const Point& s = arc1_.start_point();
        const Point& t = arc2_.end_point();

        if (t == s) return Arc(s, s);

        Vector v = t - s;
        T a = Radian(start_tangent_);
        T b = Radian(end_tangent_);
        T c = v.norm();
        T r = c / (2.0 * sin(std::abs(a - b) * 0.5));

        if (std::isfinite(r) && std::isfinite(r * r)) {
            T d = std::sqrt(r * r - 0.25 * c * c);

            b = NormalizeAngle(b - numbers::pi_v<T>);
            if (b <= numbers::pi_v<T>) {
                if (a > b && a < b + numbers::pi_v<T>) {
                    d = -d;
                }
            } else {
                if (a > b || a < b - numbers::pi_v<T>) {
                    d = -d;
                }
            }
            Vector v1(v.y, -v.x);
            Point c = Center(s, t) + d * v1.normalized();
            Point m = s + 2 * (c - s);

            // Return the full circle.
            return Arc(s, m, s);
        }

        // If the transition circle is infinite, return the line segment from
        // start point to end point.
        return Arc(arc1_.start_point(), arc2_.end_point());
    }

    const Arc& arc1() const {
        return arc1_;
    }

    const Arc& arc2() const {
        return arc2_;
    }

    const Point& start_point() const {
        return arc1_.start_point();
    }

    const Point& end_point() const {
        return arc2_.end_point();
    }

    const Vector& start_tangent() const {
        return start_tangent_;
    }

    const Vector& end_tangent() const {
        return end_tangent_;
    }

    /**
     * Return the current control point.
     */
    Point control_point() const {
        return arc1_.end_point();
    }

private:
    Vector start_tangent_, end_tangent_;
    Arc arc1_, arc2_;
};

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CURVE_BIARC_2D_H_

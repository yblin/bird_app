//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ARC_2D_H_
#define CODELIBRARY_GEOMETRY_ARC_2D_H_

#include <algorithm>
#include <utility>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/predicate_2d.h"

namespace cl {

/**
 * Arc represents a 2D circular arc, including degenerate cases. It is uniquely
 * determined by its start point, end point, and an additional point on the arc.
 *
 * This allows for natural handling of all cases:
 * (1) Normal circular arcs;
 * (2) Full circles: when start and end points are the same, but the third point
 *     differs;
 * (3) Points: when all three points coincide;
 * (4) Line segments: when the third point lies on the line between start and
 *     end points.
 */
template <typename T>
class Arc2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;

public:
    /**
     * The default arc is a point (0, 0).
     */
    Arc2D() = default;

    /**
     * Constructs a circular arc using the start point (s), an additional
     * point (p), and the end point (t). The arc is uniquely defined by these
     * three points.
     *
     * The arc passes through all three points and its orientation (clockwise or
     * counterclockwise) is determined based on the relative position of the
     * points.
     */
    Arc2D(const Point& s, const Point& p, const Point& t)
        : start_point_(s), end_point_(t), point_on_arc_(p) {
        // Use 'double' to reduce floating point error.
        double cx = (s.x + t.x) * 0.5;
        double cy = (s.y + t.y) * 0.5;

        if (s == t) {
            if (p != s) {
                // Case 2: a full circle.
                center_ = Point(0.5 * (s.x + p.x), 0.5 * (s.y + p.y));
                radius_ = Distance(center_, p);
                orientation_ = 1;
                start_angle_ = end_angle_ = Radian(s - center_);
            } else {
                // Case 3: a single point.
                center_ = s;
                radius_ = 0;
                orientation_ = 0;
                start_angle_ = end_angle_ = 0;
            }

            return;
        }

        orientation_ = geometry::Orientation(s, t, p);
        if (orientation_ == 0) {
            // Case 4: line segment.
            center_ = Point(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
            radius_ = std::numeric_limits<T>::infinity();
            return;
        }

        double ux = (s.y - t.y);
        double uy = (t.x - s.x);
        double bx = (t.x + p.x) * 0.5;
        double by = (t.y + p.y) * 0.5;
        double vx = (t.y - p.y);
        double vy = (p.x - t.x);
        double dx = cx - bx;
        double dy = cy - by;

        double vu = vx * uy - vy * ux;
        double g = (dx * uy - dy * ux) / vu;
        T cx1 = bx + g * vx, cy1 = by + g * vy;
        if (std::isfinite(cx1) && std::isfinite(cy1)) {
            // Case 1: normal arc.
            center_ = Point(cx1, cy1);
            double r1 = Distance(center_, s);
            double r2 = Distance(center_, p);
            double r3 = Distance(center_, t);
            radius_ = static_cast<T>(std::max(std::max(r1, r2), r3));
            start_angle_ = Radian(s - center_);
            end_angle_ = Radian(t - center_);
        } else {
            // Case 4: line segment.
            center_ = Point(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
            radius_ = std::numeric_limits<T>::infinity();
            orientation_ = 0;
        }
    }

    /**
     * Construct an arc that degenerates into a straight line segment.
     *
     * It is a special case of an arc where the circle's radius tends to
     * infinity, effectively modeling a straight line between the two points.
     */
    Arc2D(const Point& s, const Point& t)
        : start_point_(s), end_point_(t), point_on_arc_(s) {
        orientation_ = 0;
        if (s == t) {
            // Degenerate to point.
            center_ = s;
        } else {
            radius_ = std::numeric_limits<T>::infinity();
            center_ = Point(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
        }
    }

    /**
     * Construct a full circle arc.
     */
    Arc2D(const Point& c, T r)
        : center_(c), radius_(r) {
        CHECK(r >= 0);

        if (r == 0) {
            // Degenerate to point.
            orientation_ = 0;
            start_point_ = end_point_ = c;
        } else {
            orientation_ = 1;
            point_on_arc_ = Point(c.x + r, c.y);
            start_point_ = end_point_ = Point(c.x - r, c.y);
        }
    }

    /**
     * Construct a circular arc using the start point (s), end point (t),
     * and the tangent vector at the start point.
     */
    Arc2D(const Point& s, const Vector2D<T>& tangent, const Point& t)
        : start_point_(s), end_point_(t) {
        if (s == t) {
            // Degenerate to point.
            point_on_arc_ = s;
            center_ = s;
            return;
        }

        orientation_ = geometry::Orientation(s, t, s + tangent);
        if (orientation_ == 0) {
            // Degenerate to line.
            point_on_arc_ = s;
            center_ = Point(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
            radius_ = std::numeric_limits<T>::infinity();
            return;
        }

        Vector2D<T> v1 = t - s;
        Vector2D<T> offset;
        T angle = 0.5 * numbers::pi_v<T> - Radian(tangent, v1);
        if (orientation_ > 0) {
            offset = Vector2D<T>(v1.y, -v1.x);
        } else {
            offset = Vector2D<T>(-v1.y, v1.x);
        }
        offset.normalize();

        T d = 0.5 * v1.norm() * std::tan(angle);
        center_ = Point(0.5 * (t.x + s.x), 0.5 * (t.y + s.y));
        center_ += offset * d;

        if (std::isfinite(center_.x) && std::isfinite(center_.y)) {
            radius_ = Distance(center_, s);
            start_angle_ = Radian(s - center_);
            end_angle_ = Radian(t - center_);
        } else {
            // Case 4: line segment.
            center_ = Point(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
            radius_ = std::numeric_limits<T>::infinity();
            point_on_arc_ = s;
            orientation_ = 0;
        }
        point_on_arc_ = this->midpoint();
    }

    /**
     * Construct a circular arc using the start point (s), end point (t),
     * and the tangent vector at the end point.
     */
    Arc2D(const Point& s, const Point& t, const Vector2D<T>& tangent)
        : Arc2D(t, -tangent, s) {
        this->Reverse();
    }

    /**
     * Check if the angle (in radians) is between in start angle and end angle
     * (on the arc).
     */
    bool IsAngleBetween(T angle) const {
        CHECK(angle >= 0 && angle < numbers::pi_v<T> * 2);

        if (orientation_ == 0) return false;
        if (is_circle()) return true;

        if (orientation_ < 0) {
            if (start_angle_ < end_angle_) {
                return angle >= start_angle_ && angle <= end_angle_;
            } else {
                return angle >= start_angle_ || angle <= end_angle_;
            }
        } else {
            if (start_angle_ > end_angle_) {
                return angle >= end_angle_ && angle <= start_angle_;
            } else {
                return angle >= end_angle_ || angle <= start_angle_;
            }
        }
    }

    /**
     * Clamp the input angle to the nearest valid angle on the arc.
     */
    T Clamp(T angle) const {
        if (orientation_ == 0) return 0;

        angle = NormalizeAngle(angle);
        if (IsAngleBetween(angle)) return angle;

        if (orientation_ < 0) {
            if (start_angle_ < end_angle_) {
                return cl::Clamp(angle, start_angle_, end_angle_);
            } else {
                return start_angle_ - angle < angle - end_angle_ ? start_angle_
                                                                 : end_angle_;
            }
        } else {
            if (start_angle_ > end_angle_) {
                return cl::Clamp(angle, end_angle_, start_angle_);
            } else {
                return end_angle_ - angle < angle - start_angle_ ? end_angle_
                                                                 : start_angle_;
            }
        }
    }

    /**
     * Reverse the arc.
     */
    void Reverse() {
        std::swap(start_point_, end_point_);
        std::swap(start_angle_, end_angle_);
        orientation_ = -orientation_;
    }

    /**
     * Return the circle of this arc.
     */
    Circle2D<T> GetCircle() const {
        return Circle2D<T>(center_, radius_);
    }

    /**
     * Check if this arc degenerates to a point.
     */
    bool empty() const {
        return radius_ == 0;
    }

    /**
     * Return true if this arc is a full circle.
     */
    bool is_circle() const {
        return start_point_ == end_point_ && radius_ > 0;
    }

    /**
     * Return true if this arc is degenerate to a line segment (or a point).
     */
    bool is_line() const {
        return orientation_ == 0;
    }

    /**
     * Return the center point of the circle.
     */
    const Point& center() const {
        return center_;
    }

    /**
     * Return the radius of the circle.
     */
    T radius() const {
        return radius_;
    }

    /**
     * The starting angle of the arc, in radians. The range is [0, 2PI).
     */
    T start_angle() const {
        if (orientation_ == 0) return 0;
        return Radian(start_point_ - center_);
    }

    /**
     * The ending angle of the arc, in radians. The range is [0, 2PI).
     */
    T end_angle() const {
        if (orientation_ == 0) return 0;
        return Radian(end_point_ - center_);
    }

    /**
     * Return the start point of the arc.
     */
    const Point& start_point() const {
        return start_point_;
    }

    /**
     * Return the end point of the arc.
     */
    const Point& end_point() const {
        return end_point_;
    }

    /**
     * Return the third point on the arc.
     */
    const Point& point_on_arc() const {
        return point_on_arc_;
    }

    /**
     * Return the tangent at the start point.
     */
    Vector2D<T> start_tangent() const {
        if (orientation_ == 0) {
            return end_point_ - start_point_;
        }

        if (orientation_ > 0)
            return {std::sin(start_angle_), -std::cos(start_angle_)};
        else
            return {-std::sin(start_angle_), std::cos(start_angle_)};
    }

    /**
     * Return the tangent at the end point.
     */
    Vector2D<T> end_tangent() const {
        if (orientation_ == 0) {
            return end_point_ - start_point_;
        }

        if (orientation_ > 0)
            return {std::sin(end_angle_), -std::cos(end_angle_)};
        else
            return {-std::sin(end_angle_), std::cos(end_angle_)};
    }

    /**
     * Return the tangent vector at the given radian angle.
     */
    Vector2D<T> tangent(T radian) const {
        if (orientation_ == 0) {
            return end_point_ - start_point_;
        }

        T angle = this->Clamp(radian);
        if (orientation_ > 0)
            return {std::sin(angle), -std::cos(angle)};
        else
            return {-std::sin(angle), std::cos(angle)};
    }

    /**
     * Return the tangent vector at a specified point on the arc.
     */
    Vector2D<T> tangent(const Point& p) const {
        T radian = 0;
        if (orientation_ != 0) {
            radian = Radian(p - center_);
        }

        return tangent(radian);
    }

    /**
     * Return the middle point of the arc.
     */
    Point midpoint() const {
        if (orientation_ == 0) {
            return Point(0.5 * (start_point_.x + end_point_.x),
                         0.5 * (start_point_.y + end_point_.y));
        }

        T total = total_radians();
        T angle = orientation_ > 0 ? start_angle_ - total * 0.5
                                   : start_angle_ + total * 0.5;
        return Point(center_.x + std::cos(angle) * radius_,
                     center_.y + std::sin(angle) * radius_);
    }

    /**
     * Return the orientation of the arc.
     * 0 for line segment, 1 for clockwise and -1 for counterclockwise.
     */
    int orientation() const {
        return orientation_;
    }

    /**
     * The total radians covered by the arc.
     */
    T total_radians() const {
        if (orientation_ == 0) return 0;

        T s = start_angle_;
        T t = end_angle_;

        if (orientation_ > 0) {
            if (s >= t)
                return s - t;
            else
                return s + 2 * numbers::pi_v<T> - t;
        } else {
            if (s <= t)
                return t - s;
            else
                return t + 2 * numbers::pi_v<T> - s;
        }
    }

    /**
     * Return the length of the arc.
     */
    double length() const {
        if (orientation_ == 0) {
            return Distance(start_point_, end_point_);
        }

        return total_radians() * radius_;
    }

protected:
    /**
     * Compute the distance between (x1, y1) and (x2, y2).
     */
    static double Distance(const Point& s, const Point& t) {
        return std::sqrt(static_cast<double>(s.x - t.x) * (s.x - t.x) +
                         static_cast<double>(s.y - t.y) * (s.y - t.y));
    }

    Point center_;       // Center of circle.
    T radius_ = 0;       // The radius of the arc.
    Point start_point_;  // The starting point of the arc.
    Point end_point_;    // The end point of the arc.
    Point point_on_arc_; // A point on the arc used to uniquely define the arc.
    T start_angle_ = 0;  // The starting angle of the arc, in radians.
    T end_angle_ = 0;    // The ending angle of the arc, in radians.

    // The orientation of the arc.
    // 0 for line segment, 1 for clockwise and -1 for counterclockwise.
    int orientation_ = 0;
};

using FArc2D = Arc2D<float>;
using RArc2D = Arc2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ARC_2D_H_

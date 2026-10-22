//
// Copyright 2022-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_
#define CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_

#include <algorithm>
#include <limits>
#include <utility>

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/circle_3d.h"
#include "codelibrary/geometry/cone_3d.h"
#include "codelibrary/geometry/cylinder_3d.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/ray_3d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/sphere_3d.h"
#include "codelibrary/geometry/triangle_3d.h"

namespace cl {

/**
 * A range ray is a special ray. It is defined by an original point o, a
 * direction vector v, and a range pair p representing the current ray range.
 *
 * Specifically, a range ray (o, v, (l, u)) represents the point set:
 *   {q = o + v * t | t \in [l, u]}
 */
template <typename T>
class RangeRay3D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point3D<T>;
    using Vector = Vector3D<T>;

public:
    /**
     * A default range ray is a invalid ray.
     */
    RangeRay3D() = default;

    explicit RangeRay3D(const Ray3D<T>& ray)
        : origin_(ray.origin()),
          direction_(ray.direction()),
          lower_(0),
          upper_(std::numeric_limits<T>::max()) {}

    explicit RangeRay3D(const Segment3D<T>& seg)
        : origin_(seg.lower_point()),
          direction_(seg.direction()),
          lower_(0),
          upper_(1) {}

    explicit RangeRay3D(const Line3D<T>& line)
        : origin_(line.point()),
          direction_(line.direction()),
          lower_(-std::numeric_limits<T>::max()),
          upper_(std::numeric_limits<T>::max()) {}

    RangeRay3D(const Point3D<T>& origin, const Vector3D<T>& direction)
        : origin_(origin),
          direction_(direction),
          lower_(0),
          upper_(std::numeric_limits<T>::max()) {}

    /**
     * Segment defined by (p1, p2).
     */
    RangeRay3D(const Point& p1, const Point& p2)
        : origin_(p1),
          direction_(p2 - p1),
          lower_(0),
          upper_(1) {}

    /**
     * Check if the current ray is empty.
     */
    bool empty() const {
        return lower_ > upper_;
    }

    /**
     * Reset this RangeRay to the overlapping part with a box.
     *
     * Smits' method.
     */
    bool Intersect(const Box3D<T>& box) {
        if (box.empty()) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        for (int i = 0; i < 3; ++i) {
            // It handles the INF case correctly.
            T inverse_direction = 1 / direction_[i];
            T t1 = (box.min(i) - origin_[i]) * inverse_direction;
            T t2 = (box.max(i) - origin_[i]) * inverse_direction;
            if (inverse_direction < 0) std::swap(t1, t2);

            lower_ = std::max(t1, lower_);
            upper_ = std::min(t2, upper_);
        }

        return lower_ <= upper_;
    }

    /**
     * Reset this RangeRay to the overlapping part with a sphere.
     */
    bool Intersect(const Sphere3D<T>& sphere) {
        T a = direction_.dot(direction_);
        if (a == 0) return false;

        Vector diff = sphere.center() - origin_;
        T b = direction_.dot(diff);
        T c = diff.dot(diff) - sphere.radius() * sphere.radius();

        T discriminant = b * b - a * c;
        if (discriminant < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        T root = std::sqrt(discriminant);
        if (b + root < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        lower_ = std::max(lower_, (b - root) / a);
        upper_ = std::min(upper_, (b + root) / a);
        return true;
    }

    /**
     * Reset this RangeRay to the overlapping part with a cylinder.
     */
    bool Intersect(const Cylinder3D<T>& cylinder) {
        // Calculate quadratic equation to solve.
        Vector dp = (cylinder.point2() - cylinder.point1()).normalized();
        Vector o = origin_ - cylinder.point1();
        const Vector& dir = direction_;

        const T dd = dir.dot(dir);
        const T od = dir.dot(o);
        const T oo = o.dot(o);
        const T dz = dp.dot(dir);
        const T oz = dp.dot(o);

        const T a = dd - dz * dz;
        const T b = 2 * (od - dz * oz);
        const T c = oo - oz * oz - cylinder.radius() * cylinder.radius();

        // We miss the cylinder surface if determinant is smaller than zero.
        const T d = b * b - 4 * a * c;

        if (d < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        const Vector w1 = cylinder.point1() - origin_;
        const Vector w2 = cylinder.point2() - origin_;

        T d1 = dp.dot(direction_);
        T n1 = dp.dot(w1);
        T n2 = dp.dot(w2);
        T t1 = n1 / d1;
        T t2 = n2 / d1;

        T ulp = std::numeric_limits<T>::epsilon();
        const T eps = 16 * ulp * std::max(std::abs(dd), std::abs(dz * dz));
        if (std::abs(a) < eps) {
            // Special case for rays that are parallel to the cylinder.
            if (c <= 0) {
                lower_ = std::max(lower_, std::min(t1, t2));
                upper_ = std::min(upper_, std::max(t1, t2));
                return lower_ <= upper_;
            } else {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }
        }

        // Standard case for rays that are not parallel to the cylinder.
        const T q = std::sqrt(d);
        const T rcp_2a = 1 / (2 * a);
        lower_ = std::max(lower_, (-b - q) * rcp_2a);
        upper_ = std::min(upper_, (-b + q) * rcp_2a);

        if (std::isfinite(t1) && std::isfinite(t2)) {
            lower_ = std::max(lower_, std::min(t1, t2));
            upper_ = std::min(upper_, std::max(t1, t2));
        } else {
            T t1 = (this->lower_point() - cylinder.point1()).dot(dp);
            T t2 = (this->lower_point() - cylinder.point2()).dot(-dp);
            if (t1 < 0 || t2 < 0) {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }

            T t3 = (this->upper_point() - cylinder.point1()).dot(dp);
            T t4 = (this->upper_point() - cylinder.point2()).dot(-dp);
            if (t3 < 0 || t4 < 0) {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }
        }

        return lower_ <= upper_;
    }

    /**
     * Reset this RangeRay to the overlapping part with a cone.
     */
    bool Intersect(const Cone3D<T>& cone) {
        // First, identify intersections between a line and an infinite cone.
        // An infinite cone has no base and extends in both directions --
        // downward from the tip as you would expect, but also inside-out upward
        // from the tip.
        using Vector = Vector3D<T>;
        Vector rl = origin_ - cone.tip();
        Vector cs = cone.tip() - cone.base_center(); // Cone spine.
        T ch = cs.norm();                            // Cone height.
        if (ch == 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        // Cone axis.
        Vector ca = cs * (T(1) / ch);

        // Cone ratio.
        T cq = (cone.base_radius() * cone.base_radius()) / (ch * ch);

        T norm = direction_.norm();
        Vector direction = direction_.normalized();
        T rd_dot_ca = direction.dot(ca);
        T rl_dot_rl = rl.dot(rl);
        T rl_dot_ca = rl.dot(ca);

        T a = direction.dot(direction) - (cq + 1) * (rd_dot_ca * rd_dot_ca);
        T b = T(2) * (direction.dot(rl) - (cq + 1) * rl_dot_ca * rd_dot_ca);
        T c = rl_dot_rl - (cq + 1) * rl_dot_ca * rl_dot_ca;

        T hit_away, hit_near;
        int count = 0;
        const T epsilon = std::numeric_limits<T>::epsilon();
        T discriminant = (b * b) - (T(4) * a * c);
        if (discriminant > epsilon) {
            T b_term = b < epsilon ? -b + std::sqrt(discriminant)
                                   : -b - std::sqrt(discriminant);

            hit_near = b_term / (T(2) * a); // quadratic formula
            hit_away = (T(2) * c) / b_term; // citardauq formula

            if (hit_near > hit_away)
                // Use of both formulae, plus this, avoids catastrophic
                // cancellation due to floating-point limits.
                std::swap(hit_near, hit_away);

            count = 2;
        } else if (discriminant > -epsilon && discriminant <= epsilon) {
           hit_near = -(b / (2.0 * a));
           hit_away = hit_near;
           count = 1;
        } else {
            // There is no intersection between a line (i.e. a "double-sided"
            // ray) and the infinite cone that matches our finite cone.
            // This means that we cannot be hitting any part of the cone: if we
            // were hitting the base from the inside, for example, then the
            // "back of our ray" would be hitting the upper part of the infinite
            // cone.
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        // Now, we need to take our intersection points and ensure that they lie
        // on the surface of a finite cone. If one of them is below the finite
        // cone, then we need to check for a valid intersection with the cone's
        // base.
        bool valid2 = true;

        Point hp1 = origin_ + direction * hit_near;
        Point hp2 = origin_ + direction * hit_away;
        T ho1 = (cone.tip() - hp1).dot(ca); // height offset
        T ho2 = (cone.tip() - hp2).dot(ca);

        int valid_count = count;
        if (hit_near < lower_ || ho1 < epsilon || ho1 > ch) {
           --valid_count;
        }
        if (hit_away < lower_ || ho2 < epsilon || ho2 > ch) {
           valid2 = false;
           if (count > 1) --valid_count;
        }

        if (valid_count == 0) {
            // The ray never hits the bounded cone's curved surface. If it
            // originates from inside the cone and points "downward," however,
            // it could still hit the cone's endcap from inside.
            Point p;
            Circle3D<T> circle(cone.base_center(), ca, cone.base_radius());
            if (Hit(circle, &p)) {
                T dis = Distance(p, origin_) / norm;
                upper_ = std::min(dis, upper_);
                return lower_ <= upper_;
            }
            return false;
        }
        if (valid_count == 1) {
            // The ray hits the cone's curved surface only once. This can only
            // happen under two cases: the ray originates from inside the cone,
            // and points outward; or the ray passes through the bounded cone
            // once and through the cone's endcap.
            if (valid2) {
                hit_near = hit_away;
            }
            Circle3D<T> circle(cone.base_center(), ca, cone.base_radius());
            Point p;
            if (!Hit(circle, &p)) {
                upper_ = std::min(hit_near / norm, upper_);
                return lower_ <= upper_;
            }

            hit_away = Distance(p, origin_);
        }

        if (hit_near > hit_away) {
            std::swap(hit_near, hit_away);
        }

        lower_ = std::max(lower_, hit_near / norm);
        upper_ = std::min(upper_, hit_away / norm);
        return lower_ <= upper_;
    }

    /**
     * Check if the range ray hits on an object. It optional returns the nearest
     * hit position.
     */
    bool Hit(const Plane3D<T>& plane, Point* p = nullptr) const {
        if (plane.empty()) return false;

        const Vector w = plane.point() - origin_;

        T d = plane.normal().dot(direction_);
        T n = plane.normal().dot(w);
        T t = n / d;

        if (t >= lower_ && t <= upper_) {
            if (p) *p = origin_ + t * direction_;
            return true;
        }
        return false;
    }
    bool Hit(const Triangle3D<T>& triangle, Point* p = nullptr) const {
        if (triangle.empty()) return false;

        Point r;
        if (!Hit(Plane3D<T>(triangle.vertices()), &r)) return false;

        // We use fast point inside triangle test here.
        Vector o0 = (triangle.vertices()[1] - triangle.vertices()[0]).cross(
                     r - triangle.vertices()[0]);
        Vector o1 = (triangle.vertices()[2] - triangle.vertices()[1]).cross(
                     r - triangle.vertices()[1]);
        if (o0.dot(o1) < 0) return false;

        Vector o2 = (triangle.vertices()[0] - triangle.vertices()[2]).cross(
                     r - triangle.vertices()[2]);
        if (o0.dot(o2) < 0) return false;

        if (p) *p = r;
        return true;
    }
    bool Hit(const Circle3D<T>& circle, Point* p = nullptr) const {
        Plane3D<T> plane(circle.center(), circle.normal());

        Point q;
        if (!Hit(plane, &q)) return false;

        if (SquaredDistance(q, circle.center()) >
            circle.radius() * circle.radius()) {
            return false;
        }

        if (p) *p = q;
        return true;
    }

    /**
     * Return the first end point of range ray.
     */
    Point lower_point() const {
        return origin_ + direction_ * lower_;
    }

    /**
     * Return the second end point of range ray.
     */
    Point upper_point() const {
        return origin_ + direction_ * upper_;
    }

    T lower()                 const { return lower_;     }
    T upper()                 const { return upper_;     }
    void set_lower(const T& lower)  { lower_ = lower;    }
    void set_upper(const T& upper)  { upper_ = upper;    }
    const Point& origin()     const { return origin_;    }
    const Vector& direction() const { return direction_; }

private:
    Point origin_;
    Vector direction_;
    T lower_ =  std::numeric_limits<T>::max();
    T upper_ = -std::numeric_limits<T>::max();
};

using FRangeRay3D = RangeRay3D<float>;
using RRangeRay3D = RangeRay3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_

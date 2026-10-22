//
// Copyright 2012-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ANGLE_H_
#define CODELIBRARY_GEOMETRY_ANGLE_H_

#include <algorithm>
#include <type_traits>
#include <limits>

#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/vector_2d.h"
#include "codelibrary/math/number/exact_float.h"

namespace cl {

/**
 * This function normalizes a given angle to ensure it falls within the range
 * of [0, 2 PI).
 */
template <typename T>
inline T NormalizeAngle(T radian) {
    T two_pi = numbers::pi_v<T> * 2;
    radian = std::fmod(radian, two_pi);
    if (radian < 0) {
        radian += two_pi;
    }
    // Ensure strictly less than 2 PI
    if (radian >= two_pi) {
        radian -= two_pi;
    }

    return radian;
}

/**
 * Convert degree to radian.
 */
template <typename T>
inline T DegreeToRadian(const T& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return v * numbers::pi_v<T> / 180;
}

/**
 * Convert radian to degree.
 */
template <typename T>
inline T RadianToDegree(const T& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return v * 180 / numbers::pi_v<T>;
}

/**
 * Get the angle of 2D vector in radians.
 *
 * The return value is in the range: [0, 2 * PI).
 */
template <typename T>
T Radian(const Vector2D<T>& v) {
    static_assert(std::is_floating_point<T>::value, "");

    T radian = std::atan2(v.y, v.x);
    radian = radian < 0 ? radian + 2 * numbers::pi_v<T> : radian;
    return (radian >= 2 * numbers::pi_v<T>) ? 0 : radian;
}

/**
 * Get the degree angle of 2D vector.
 *
 * The return value is in the range: [0, 360).
 */
template <typename T>
T Degree(const Vector2D<T>& v) {
    T degree = RadianToDegree(Radian(v));
    return degree >= 360 ? 0 : degree;
}

/**
 * Get the angle between two vectors in degrees.
 */
template <typename Vector>
typename Vector::value_type Degree(const Vector& v1, const Vector& v2) {
    return RadianToDegree(Radian(v1, v2));
}

/**
 * Numerically stable calculation of the angle between two vectors in radians.
 *
 * Also it is faster than cosine law version.
 *
 * The return value is in the range: [0, PI].
 */
template <typename Vector>
typename Vector::value_type Radian(const Vector& v1, const Vector& v2) {
    using T = typename Vector::value_type;
    static_assert(std::is_floating_point<T>::value, "");

    T a = v1.norm();
    T b = v2.norm();
    T c = (v2 - v1).norm();
    if (a == 0 || b == 0) return 0;

    T u = b >= c ? c - (a - b) : b - (a - c);
    T t1 = std::max(((a - b) + c) * u, T(0));
    T t2 = ((b + c) + a) * ((a - c) + b);
    return t2 <= 0 ? numbers::pi_v<T>
                   : T(2) * std::atan(std::sqrt(t1 / t2));
}

/**
 * Robust compute the radian angle of a 2D vector (p2 - p1).
 *
 * The return value is in the range: [0, 2 * PI).
 */
template <typename T>
T RobustRadian(const Point2D<T>& p1, const Point2D<T>& p2) {
    static_assert(std::is_floating_point<T>::value, "");

    ExactFloat a(p1.x), b(p1.y), c(p2.x), d(p2.y);
    c -= a;
    d -= b;

    if (c.sign() == 0) {
        if (d.sign() == 0) return std::numeric_limits<T>::quiet_NaN();
        return d.sign() > 0 ? 0.5 * numbers::pi_v<T> : 1.5 * numbers::pi_v<T>;
    }

    int sign = c.sign() * d.sign();
    if (sign == 0) return c.sign() > 0 ? 0 : numbers::pi_v<T>;

    double v = static_cast<double>(d.significand().ToUInt64()) /
               static_cast<double>(c.significand().ToUInt64()) *
               std::pow(2.0, d.exponent() - c.exponent());
    double angle = std::atan(v);
    if (sign <= 0) angle = 2 * numbers::pi_v<T> - angle;
    return angle < 0 ? T(0) : angle >= 2 * numbers::pi_v<T> ? T(0) : T(angle);
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ANGLE_H_

//
// Copyright 2011-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_VECTOR_2D_H_
#define CODELIBRARY_GEOMETRY_VECTOR_2D_H_

#include <cmath>
#include <limits>
#include <type_traits>

#include "codelibrary/base/message.h"

namespace cl {

/**
 * 2D vector.
 */
template<typename T>
class Vector2D {
public:
    using value_type = T;

    Vector2D() = default;

    Vector2D(const T& x1, const T& y1)
        : x(x1), y(y1) {}

    bool operator ==(const Vector2D& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    bool operator !=(const Vector2D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Vector2D& rhs) const {
        return x < rhs.x || (x == rhs.x && y < rhs.y);
    }

    bool operator <=(const Vector2D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Vector2D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Vector2D& rhs) const {
        return !(*this < rhs);
    }

    const Vector2D& operator +=(const Vector2D& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    const Vector2D& operator -=(const Vector2D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    const Vector2D& operator *=(const T& rhs) {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    const Vector2D& operator *=(const Vector2D& rhs) {
        x *= rhs.x;
        y *= rhs.y;
        return *this;
    }

    /**
     * Return the squared euclidean norm of the vector.
     */
    T squared_norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return x * x + y * y;
    }

    /**
     * Return the euclidean norm of the vector.
     */
    T norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return std::sqrt(squared_norm());
    }

    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector2D& normalize() {
        static_assert(std::is_floating_point<T>::value, "");

        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            x = 0;
            y = 1;
        } else {
            T t = T(1) / len;
            x *= t;
            y *= t;
        }
        return *this;
    }

    /**
     * Return the normalized vector.
     * The length of the normalized vector is always one.
     */
    Vector2D normalized() const {
        static_assert(std::is_floating_point<T>::value);

        Vector2D v;
        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            v.x = 0;
            v.y = 0;
        } else {
            T t = T(1) / len;
            v.x = x * t;
            v.y = y * t;
        }

        return v;
    }

    /**
     * Set all vector elements to zero.
     */
    void set_zero() {
        x = 0;
        y = 0;
    }

    /**
     * Return the i-th component value of vector.
     */
    const T& operator[] (int i) const {
        return i == 0 ? x : y;
    }

    /**
     * Return the reference value of the i-th component of vector.
     */
    T& operator[] (int i) {
        return i == 0 ? x : y;
    }

    /**
     * Return the dot product of two vectors.
     */
    T dot(const Vector2D& rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    /**
     * Return the cross product of two vectors.
     */
    T cross(const Vector2D<T>& rhs) const {
        return x * rhs.y - rhs.x * y;
    }

    int size() const { return 2; }

    friend Vector2D operator +(const Vector2D& lhs, const Vector2D& rhs) {
        return Vector2D(lhs.x + rhs.x, lhs.y + rhs.y);
    }

    friend Vector2D operator -(const Vector2D& lhs, const Vector2D& rhs) {
        return Vector2D(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    friend Vector2D operator -(const Vector2D& rhs) {
        return Vector2D(-rhs.x, -rhs.y);
    }

    friend Vector2D operator *(const T& lhs, const Vector2D& rhs) {
        return Vector2D(lhs * rhs.x, lhs * rhs.y);
    }

    friend Vector2D operator *(const Vector2D& lhs, const T& rhs) {
        return Vector2D(lhs.x * rhs, lhs.y * rhs);
    }

    friend Vector2D operator *(const Vector2D& lhs, const Vector2D& rhs) {
        return Vector2D(lhs.x * rhs.x, lhs.y * rhs.y);
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector2D& v) {
        Message msg(v.x, v.y);
        return os << msg;
    }

    T x = 0; // X component.
    T y = 0; // Y component.
};

using IVector2D = Vector2D<int>;
using FVector2D = Vector2D<float>;
using RVector2D = Vector2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_VECTOR_2D_H_

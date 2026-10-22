//
// Copyright 2022-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_VECTOR_4D_H_
#define CODELIBRARY_GEOMETRY_VECTOR_4D_H_

#include <cmath>
#include <limits>
#include <type_traits>

#include "codelibrary/base/message.h"

namespace cl {

/**
 * 4D vector.
 */
template <typename T>
class Vector4D {
public:
    using value_type = T;

    Vector4D() = default;

    Vector4D(const T& x1, const T& y1, const T& z1, const T& w1)
        : x(x1), y(y1), z(z1), w(w1) {}

    bool operator ==(const Vector4D& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
    }

    bool operator !=(const Vector4D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Vector4D& rhs) const {
        return x == rhs.x ? y == rhs.y ? z == rhs.z ? w < rhs.w
                                                    : z < rhs.z
                                       : y < rhs.y
                          : x < rhs.x;
    }

    bool operator <=(const Vector4D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Vector4D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Vector4D& rhs) const {
        return !(*this < rhs);
    }

    const Vector4D& operator +=(const Vector4D& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w += rhs.w;
        return *this;
    }

    const Vector4D& operator -=(const Vector4D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w -= rhs.w;
        return *this;
    }

    const Vector4D& operator *=(const T& rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        w *= rhs;
        return *this;
    }

    const Vector4D& operator *=(const Vector4D& rhs) {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        w *= rhs.w;
        return *this;
    }

    /**
     * Return the squared euclidean norm of the vector.
     */
    T squared_norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return x * x + y * y + z * z + w * w;
    }

    /**
     * Return the euclidean norm of the vector.
     */
    T norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return std::sqrt(squared_norm());
    }

    /**
     * Set all vector elements to zero.
     */
    void set_zero() {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }

    /**
     * Return the i-th component value of vector.
     */
    const T& operator[] (int i) const {
        switch (i) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return w;
        default:
            CHECK(false) << "The input index is " << i
                         << ", which is not in [0, 4).";
            return w;
        }
    }

    /**
     * Return the reference value of the i-th component of vector.
     */
    T& operator[] (int i) {
        switch (i) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return w;
        default:
            CHECK(false) << "The input index is " << i
                         << ", which is not in [0, 4).";
            return w;
        }
    }

    int size() const { return 4; }

    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector4D& normalize() {
        static_assert(std::is_floating_point<T>::value);

        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            x = 0;
            y = 0;
            z = 0;
            w = 1;
        } else {
            T t = T(1) / len;
            x *= t;
            y *= t;
            z *= t;
            w *= t;
        }

        return *this;
    }

    /**
     * Return the normalized vector.
     * The length of the normalized vector is always one.
     */
    Vector4D normalized() const {
        static_assert(std::is_floating_point<T>::value);

        Vector4D v;
        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            v.x = 0;
            v.y = 0;
            v.z = 0;
            v.w = 1;
        } else {
            T t = T(1) / len;
            v.x = x * t;
            v.y = y * t;
            v.z = z * t;
            v.w = w * t;
        }

        return v;
    }

    /**
     * Return the dot product of two vectors.
     */
    T dot(const Vector4D& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
    }

    friend Vector4D operator +(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z,
                        lhs.w + rhs.w);
    }

    friend Vector4D operator -(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z,
                        lhs.w - rhs.w);
    }

    friend Vector4D operator -(const Vector4D& rhs) {
        return Vector4D(-rhs.x, -rhs.y, -rhs.z, -rhs.w);
    }

    friend Vector4D operator *(const T& lhs, const Vector4D& rhs) {
        return Vector4D(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
    }

    friend Vector4D operator *(const Vector4D& lhs, const T& rhs) {
        return Vector4D(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
    }

    friend Vector4D operator *(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z,
                        lhs.w * rhs.w);
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector4D& v) {
        Message msg(v.x, v.y, v.z, v.w);
        return os << msg;
    }

    T x = 0; // X component.
    T y = 0; // Y component.
    T z = 0; // Z component.
    T w = 0; // W component.
};

using IVector4D = Vector4D<int>;
using FVector4D = Vector4D<float>;
using RVector4D = Vector4D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_VECTOR_4D_H_

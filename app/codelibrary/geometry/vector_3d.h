//
// Copyright 2013-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_VECTOR_3D_H_
#define CODELIBRARY_GEOMETRY_VECTOR_3D_H_

#include <cmath>
#include <limits>
#include <type_traits>

#include "codelibrary/base/message.h"

namespace cl {

/**
 * 3D vector.
 */
template<typename T>
class Vector3D {
public:
    using value_type = T;

    Vector3D() = default;

    Vector3D(const T& x1, const T& y1, const T& z1)
        : x(x1), y(y1), z(z1) {}

    bool operator ==(const Vector3D& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator !=(const Vector3D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Vector3D& rhs) const {
        return x == rhs.x ? (y == rhs.y ? z < rhs.z : y < rhs.y) : x < rhs.x;
    }

    bool operator <=(const Vector3D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Vector3D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Vector3D& rhs) const {
        return !(*this < rhs);
    }

    const Vector3D& operator +=(const Vector3D& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    const Vector3D& operator -=(const Vector3D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    const Vector3D& operator *=(const T& rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        return *this;
    }

    const Vector3D& operator *=(const Vector3D& rhs) {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }

    /**
     * Return the squared euclidean norm of the vector.
     */
    T squared_norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return x * x + y * y + z * z;
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
    }

    /**
     * Return the i-th component value of vector.
     */
    const T& operator[] (int i) const {
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    /**
     * Return the reference value of the i-th component of vector.
     */
    T& operator[] (int i) {
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    int size() const { return 3; }

    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector3D& normalize() {
        static_assert(std::is_floating_point<T>::value);

        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            x = 0;
            y = 0;
            z = 1;
        } else {
            T t = T(1) / len;
            x *= t;
            y *= t;
            z *= t;
        }

        return *this;
    }

    /**
     * Return the normalized vector.
     * The length of the normalized vector is always one.
     */
    Vector3D normalized() const {
        static_assert(std::is_floating_point<T>::value);

        Vector3D v;
        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            v.x = 0;
            v.y = 0;
            v.z = 1;
        } else {
            T t = T(1) / len;
            v.x = x * t;
            v.y = y * t;
            v.z = z * t;
        }

        return v;
    }

    /**
     * Return the dot product of two vectors.
     */
    T dot(const Vector3D& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    /**
     * Return the cross product of two vectors.
     */
    Vector3D<T> cross(const Vector3D<T>& rhs) const {
        return Vector3D<T>(y * rhs.z - z * rhs.y,
                           z * rhs.x - x * rhs.z,
                           x * rhs.y - y * rhs.x);
    }

    friend Vector3D operator +(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    friend Vector3D operator -(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    friend Vector3D operator -(const Vector3D& rhs) {
        return Vector3D(-rhs.x, -rhs.y, -rhs.z);
    }

    friend Vector3D operator *(const T& lhs, const Vector3D& rhs) {
        return Vector3D(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
    }

    friend Vector3D operator *(const Vector3D& lhs, const T& rhs) {
        return Vector3D(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
    }

    friend Vector3D operator *(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector3D& v) {
        Message msg(v.x, v.y, v.z);
        return os << msg;
    }

    T x = 0; // X component.
    T y = 0; // Y component.
    T z = 0; // Z component.
};

using IVector3D = Vector3D<int>;
using FVector3D = Vector3D<float>;
using RVector3D = Vector3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_VECTOR_3D_H_

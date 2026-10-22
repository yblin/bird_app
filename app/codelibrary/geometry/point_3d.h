//
// Copyright 2013-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_3D_H_
#define CODELIBRARY_GEOMETRY_POINT_3D_H_

#include "codelibrary/base/message.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/vector_3d.h"

namespace cl {

/**
 * 3D point.
 */
template <typename T>
struct Point3D {
public:
    using value_type = T;

    Point3D() = default;

    Point3D(const Point2D<T>& p, const T& z)
        : x(p.x), y(p.y), z(z) {}

    explicit Point3D(const Vector3D<T>& v)
        : x(v.x), y(v.y), z(v.z) {}

    Point3D(const T& x1, const T& y1, const T& z1)
        : x(x1), y(y1), z(z1) {}

    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Point3D(Iter first, Iter last) {
        CHECK(std::distance(first, last) == 3);

        x = *first++;
        y = *first++;
        z = *first;
    }

    bool operator ==(const Point3D& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator !=(const Point3D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Point3D& rhs) const {
        return x == rhs.x ? (y == rhs.y ? z < rhs.z : y < rhs.y) : x < rhs.x;
    }

    bool operator <=(const Point3D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Point3D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Point3D& rhs) const {
        return !(*this < rhs);
    }

    const Point3D& operator +=(const Vector3D<T>& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    const Point3D& operator -=(const Vector3D<T>& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    const Point3D& operator *=(const T& value) {
        x *= value;
        y *= value;
        z *= value;
        return *this;
    }

    const Point3D& operator *=(const Vector3D<T>& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    /**
     * Return the i-th component value of point.
     */
    const T& operator[] (int i) const {
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    /**
     * Return the reference value of the i-th component of point.
     */
    T& operator[] (int i) {
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    /**
     * Return the bounding box of this point.
     */
    const Box3D<T> bounding_box() const {
        return Box3D<T>(x, x, y, y, z, z);
    }

    /**
     * Return the dimension.
     */
    int size() const {
        return 3;
    }

    Vector3D<T> ToVector() const {
        return Vector3D<T>(x, y, z);
    }

    friend Point3D operator +(const Point3D& lhs, const Vector3D<T>& rhs) {
        return Point3D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    friend Point3D operator +(const Vector3D<T>& lhs, const Point3D& rhs) {
        return Point3D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    friend Point3D operator -(const Point3D<T>& lhs, const Vector3D<T>& rhs) {
        return Point3D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    friend Vector3D<T> operator -(const Point3D& lhs, const Point3D& rhs) {
        return Vector3D<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    friend Point3D operator -(const Point3D<T>& p) {
        return Point3D(-p.x, -p.y, -p.z);
    }

    friend Point3D operator *(const Point3D& lhs, const T& v) {
        return Point3D(lhs.x * v, lhs.y * v, lhs.z * v);
    }

    friend Point3D operator *(const T& v, const Point3D& rhs) {
        return Point3D(rhs.x * v, rhs.y * v, rhs.z * v);
    }

    friend Point3D operator *(const Point3D& lhs, const Vector3D<T>& v) {
        return Point3D(lhs.x * v.x, lhs.y * v.y, lhs.z * v.z);
    }

    friend std::ostream& operator <<(std::ostream& os, const Point3D& p) {
        Message msg(p.x, p.y, p.z);
        return os << msg;
    }

    T x = 0; // X coordinate.
    T y = 0; // Y coordinate.
    T z = 0; // Z coordinate.
};

using IPoint3D = Point3D<int>;
using FPoint3D = Point3D<float>;
using RPoint3D = Point3D<double>;

} // namespace cl

namespace std {

/**
 * Hash of 3D point. Using boost hash_combine algorithm.
 */
template<typename T>
struct hash<cl::Point3D<T>> {
    size_t operator()(const cl::Point3D<T>& p) const {
        std::hash<T> hasher;
        size_t seed = 0;
        seed ^= hasher(p.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hasher(p.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hasher(p.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

} // namespace std

#endif // CODELIBRARY_GEOMETRY_POINT_3D_H_

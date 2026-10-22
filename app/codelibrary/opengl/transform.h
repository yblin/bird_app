//
// Copyright 2020-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_TRANSFORM_H_
#define CODELIBRARY_OPENGL_TRANSFORM_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/math/matrix/matrix4.h"

namespace cl {
namespace gl {

/**
 * 4x4 transform matrix for OpenGL.
 *
 * Note that since opengl's matrix is column-major, thus we always store the
 * transpose of matrix in 'Transform'.
 */
template <typename T>
class Transform : public Matrix4<T> {
    static_assert(std::is_floating_point<T>::value,
                  "T must be a floating point type.");

    using Matrix4<T>::data_;

public:
    /**
     * The default transform is a unit matrix.
     */
    Transform() {
        this->SetIdentity();
    }

    explicit Transform(const Array<T>& data)
        : Matrix4<T>(data) {}

    Transform(const Matrix4<T>& mat)
        : Matrix4<T>(mat) {}

    Transform(T m1,  T m2,  T m3,  T m4,
              T m5,  T m6,  T m7,  T m8,
              T m9,  T m10, T m11, T m12,
              T m13, T m14, T m15, T m16)
        : Matrix4<T>(m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12,
                     m13, m14, m15, m16) {}

    Transform(const Quaternion<T>& rot) {
        this->SetIdentity();
        this->SetRotation(rot);
    }

    /**
     *                                   [1 0 0 x]
     * Multiply by a translation matrix: [0 1 0 y]
     *                                   [0 0 1 z]
     *                                   [0 0 0 1]
     */
    void Translate(T x, T y, T z) {
        data_[12] += x;
        data_[13] += y;
        data_[14] += z;
    }
    void Translate(const Vector3D<T>& v) {
        Translate(v.x, v.y, v.z);
    }

    /**
     * Set the translation vector.
     */
    void SetTranslation(T x, T y, T z) {
        data_[12] = x;
        data_[13] = y;
        data_[14] = z;
    }
    void SetTranslation(const Vector3D<T>& v) {
        this->SetTranslation(v.x, v.y, v.z);
    }

    /**
     * Return the translation vector.
     */
    Vector3D<T> GetTranslation() const {
        return Vector3D<T>{data_[12], data_[13], data_[14]};
    }

    /**
     * Multiply by a rotation matrix defined by two vectors: v1 and v2
     * (rotation from v1 to v2).
     */
    void Rotate(const Vector3D<T>& v1, const Vector3D<T>& v2) {
        Quaternion<T> rot(v1, v2);
        this->Rotate(rot);
    }

    /**
     * Multiply by a rotation matrix defined by a rotation axis and rotation
     * angle.
     */
    void Rotate(const Vector3D<T>& rotation_axis, T rotation_angle) {
        Quaternion<T> rot(rotation_axis, rotation_angle);
        this->Rotate(rot);
    }

    /**
     * Multiply by a quernion.
     */
    void Rotate(const Quaternion<T>& rot) {
        this->Rotate(rot.ToRotationMatrix());
    }

    /**
     * Multiply by a rotation matrix.
     */
    void Rotate(const Matrix3<T>& rot) {
        Transform t;
        t.SetRotation(rot);
        *this = *this * t;
    }

    /**
     * Set rotation matrix.
     */
    void SetRotation(const Matrix3<T>& rot) {
        data_[0] = rot[0]; data_[4] = rot[1]; data_[8]  = rot[2];
        data_[1] = rot[3]; data_[5] = rot[4]; data_[9]  = rot[5];
        data_[2] = rot[6]; data_[6] = rot[7]; data_[10] = rot[8];
    }

    /**
     * Set rotation matrix.
     */
    void SetRotation(const Quaternion<T>& rot) {
        this->SetRotation(rot.ToRotationMatrix());
    }

    /**
     * Set rotation matrix form pitch, yaw, roll.
     * In roll-yaw-pitch order.
     */
    void SetRotation(T pitch, T yaw, T roll) {
        SetRotation(Quaternion<T>(pitch, yaw, roll));
    }

    /**
     * Get rotation matrix.
     */
    void GetRotation(Matrix3<T>* rot) const {
        CHECK(rot);

        *rot = Matrix3<T>(data_[0], data_[4], data_[8],
                          data_[1], data_[5], data_[9],
                          data_[2], data_[6], data_[10]);
    }

    /**
     * Get quaternion.
     */
    Quaternion<T> GetRotation() const {
        Matrix3<T> rot_matrix;
        GetRotation(&rot_matrix);
        return Quaternion<T>(rot_matrix);
    }

    /**
     * Get euler angles in radians.
     */
    void GetRotation(T* pitch, T* yaw, T* roll) const {
        CHECK(pitch);
        CHECK(yaw);
        CHECK(roll);

        Quaternion<T> rot = this->GetRotation();
        rot.ToEulerAngles(pitch, yaw, roll);
    }

    /**
     * Scale the transform.
     */
    void Scale(T x, T y, T z) {
        data_[0]  *= x;
        data_[1]  *= x;
        data_[2]  *= x;

        data_[4]  *= y;
        data_[5]  *= y;
        data_[6]  *= y;

        data_[8]  *= z;
        data_[9]  *= z;
        data_[10] *= z;
    }

    /**
     * Counterclockwise rotation about the local X-axis.
     */
    void Pitch(T radian) {
        // [a00 a01 a02]   [ 1 0  0]
        // [a10 a11 a12] * [ 0 c -s]
        // [a20 a21 a22]   [ 0 s  c]
        const T c = std::cos(radian), s = std::sin(radian);
        const T a01 = data_[4], a02 = data_[8];
        const T a11 = data_[5], a12 = data_[9];
        const T a21 = data_[6], a22 = data_[10];

        data_[4] =  a01 * c + a02 * s;
        data_[8] = -a01 * s + a02 * c;

        data_[5] =  a11 * c + a12 * s;
        data_[9] = -a11 * s + a12 * c;

        data_[6]  =  a21 * c + a22 * s;
        data_[10] = -a21 * s + a22 * c;
    }

    /**
     * Counterclockwise rotation about the local Y-axis.
     */
    void Yaw(T radian) {
        // [a00 a01 a02]   [ c  0 s]
        // [a10 a11 a12] * [ 0  1 0]
        // [a20 a21 a22]   [ -s 0 c]
        const T c = std::cos(radian), s = std::sin(radian);
        const T a00 = data_[0], a02 = data_[8];
        const T a10 = data_[1], a12 = data_[9];
        const T a20 = data_[2], a22 = data_[10];

        data_[0] = a00 * c - a02 * s;
        data_[8] = a00 * s + a02 * c;

        data_[1] = a10 * c - a12 * s;
        data_[9] = a10 * s + a12 * c;

        data_[2]  = a20 * c - a22 * s;
        data_[10] = a20 * s + a22 * c;
    }

    /**
     * Counterclockwise rotation about the local Z-axis.
     */
    void Roll(T radian) {
        // [a00 a01 a02]   [ c -s 0]
        // [a10 a11 a12] * [ s  c 0]
        // [a20 a21 a22]   [ 0  0 1]
        const T c = std::cos(radian), s = std::sin(radian);
        const T a00 = data_[0], a01 = data_[4];
        const T a10 = data_[1], a11 = data_[5];
        const T a20 = data_[2], a21 = data_[6];

        data_[0] =  a00 * c + a01 * s;
        data_[4] = -a00 * s + a01 * c;

        data_[1] =  a10 * c + a11 * s;
        data_[5] = -a10 * s + a11 * c;

        data_[2] =  a20 * c + a21 * s;
        data_[6] = -a20 * s + a21 * c;
    }

    // O(1) functions.
    T translation_x() const { return data_[12]; }
    T translation_y() const { return data_[13]; }
    T translation_z() const { return data_[14]; }

    /**
     * Transform the given point.
     */
    Point3D<T> operator()(const Point3D<T>& p) const {
        T x = data_[0] * p.x + data_[4] * p.y + data_[8]  * p.z + data_[12];
        T y = data_[1] * p.x + data_[5] * p.y + data_[9]  * p.z + data_[13];
        T z = data_[2] * p.x + data_[6] * p.y + data_[10] * p.z + data_[14];
        T w = data_[3] * p.x + data_[7] * p.y + data_[11] * p.z + data_[15];
        return {x / w, y / w, z / w};
    }
    Point3D<T> operator *(const Point3D<T>& p) const {
        return this->operator()(p);
    }

    /**
     * Transform the given vector.
     */
    Vector3D<T> operator()(const Vector3D<T>& v) const {
        T x = data_[0] * v.x + data_[4] * v.y + data_[8]  * v.z;
        T y = data_[1] * v.x + data_[5] * v.y + data_[9]  * v.z;
        T z = data_[2] * v.x + data_[6] * v.y + data_[10] * v.z;
        return {x, y, z};
    }
    Vector3D<T> operator *(const Vector3D<T>& v) const {
        return this->operator()(v);
    }

    /**
     * Transform the given box.
     */
    Box3D<T> operator()(const Box3D<T>& box) const {
        if (box.empty()) return Box3D<T>();

        Array<Point3D<T>> ps;
        for (int i = 0; i < 2; ++i) {
            T x = i == 0 ? box.x_min() : box.x_max();
            for (int j = 0; j < 2; ++j) {
                T y = j == 0 ? box.y_min() : box.y_max();
                for (int k = 0; k < 2; ++k) {
                    T z = k == 0 ? box.z_min() : box.z_max();
                    Point3D<T> p(x, y, z);
                    ps.push_back(this->operator()(p));
                }
            }
        }
        return Box3D<T>(ps.begin(), ps.end());
    }

    friend Transform operator*(const Transform& lhs, const Transform& rhs) {
        Transform res;
        T* c = res.data();

        // A^-1 * B^-1 = (BA)^-1
        const T* b = lhs.data();
        const T* a = rhs.data();

        c[0] = a[0] * b[0] + a[1] * b[4] + a[2] * b[8]  + a[3] * b[12];
        c[1] = a[0] * b[1] + a[1] * b[5] + a[2] * b[9]  + a[3] * b[13];
        c[2] = a[0] * b[2] + a[1] * b[6] + a[2] * b[10] + a[3] * b[14];
        c[3] = a[0] * b[3] + a[1] * b[7] + a[2] * b[11] + a[3] * b[15];

        c[4] = a[4] * b[0] + a[5] * b[4] + a[6] * b[8]  + a[7] * b[12];
        c[5] = a[4] * b[1] + a[5] * b[5] + a[6] * b[9]  + a[7] * b[13];
        c[6] = a[4] * b[2] + a[5] * b[6] + a[6] * b[10] + a[7] * b[14];
        c[7] = a[4] * b[3] + a[5] * b[7] + a[6] * b[11] + a[7] * b[15];

        c[8]  = a[8] * b[0] + a[9] * b[4] + a[10] * b[8]  + a[11] * b[12];
        c[9]  = a[8] * b[1] + a[9] * b[5] + a[10] * b[9]  + a[11] * b[13];
        c[10] = a[8] * b[2] + a[9] * b[6] + a[10] * b[10] + a[11] * b[14];
        c[11] = a[8] * b[3] + a[9] * b[7] + a[10] * b[11] + a[11] * b[15];

        c[12] = a[12] * b[0] + a[13] * b[4] + a[14] * b[8]  + a[15] * b[12];
        c[13] = a[12] * b[1] + a[13] * b[5] + a[14] * b[9]  + a[15] * b[13];
        c[14] = a[12] * b[2] + a[13] * b[6] + a[14] * b[10] + a[15] * b[14];
        c[15] = a[12] * b[3] + a[13] * b[7] + a[14] * b[11] + a[15] * b[15];

        return res;
    }

    /**
     * Output the matrix, just for debug.
     */
    friend Message& operator <<(Message& msg, const Transform& rhs) {
        return msg.Append(rhs.data_, rhs.data_ + 16, {4, 4});
    }
};

using FTransform = Transform<float>;
using RTransform = Transform<double>;

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_TRANSFORM_H_

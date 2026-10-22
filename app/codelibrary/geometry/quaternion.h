//
// Copyright 2013-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_QUATERNION_H_
#define CODELIBRARY_GEOMETRY_QUATERNION_H_

#include <limits>

#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/geometry/vector_4d.h"
#include "codelibrary/math/matrix/matrix3.h"

namespace cl {

/**
 * Quaternions provide a convenient mathematical notation for representing
 * orientations and rotations of objects in three dimensions.
 *
 * Note that, Quaternion q and -q represent the same rotation.
 */
template <typename T>
class Quaternion : public Vector4D<T> {
    static_assert(std::is_floating_point<T>::value, "");

    using Base = Vector4D<T>;

public:
    /**
     * The default quaternion is all zero (invalid).
     */
    Quaternion() = default;

    Quaternion(T w, T x, T y, T z)
        : Base(w, x, y, z) {}

    explicit Quaternion(const Vector4D<T>& v)
        : Base(v) {}

    /**
     * Construct quaternion that rotate the specified angle (in radians) around
     * the rotation_axis.
     */
    Quaternion(const Vector3D<T>& rotation_axis, T rotation_angle) {
        T norm = rotation_axis.norm();
        if (norm < std::numeric_limits<T>::epsilon()) {
            *this = this->Identity();
            return;
        }

        T c = std::cos(T(0.5) * rotation_angle);
        T s = std::sin(T(0.5) * rotation_angle) / norm;
        this->x = rotation_axis.x * s;
        this->y = rotation_axis.y * s;
        this->z = rotation_axis.z * s;
        this->w = c;
    }

    /**
     * Construct quaternion from two vectors.
     * Represent rotation from v1 to v2.
     */
    Quaternion(const Vector3D<T>& v1, const Vector3D<T>& v2) {
        Vector3D<T> rotation_axis = v1.cross(v2);
        T phi = Radian(v1, v2);
        *this = Quaternion(rotation_axis, phi);
    }

    /**
     * Construct quaternion from 3 x 3 rotation matrix.
     */
    explicit Quaternion(const Matrix3<T>& rotation) {
        T trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
        if (trace > 0) {
            T t = T(1) + trace;
            T s = T(0.5) / std::sqrt(t);
            this->w = s * t;
            this->x = (rotation(2, 1) - rotation(1, 2)) * s;
            this->y = (rotation(0, 2) - rotation(2, 0)) * s;
            this->z = (rotation(1, 0) - rotation(0, 1)) * s;
        } else {
            int next[3] = { 1, 2, 0 };

            int i = 0;
            if (rotation(1, 1) > rotation(0, 0)) {
                i = 1;
            }
            if (rotation(2, 2) > rotation(i, i)) {
                i = 2;
            }
            int j = next[i];
            int k = next[j];
            T t = (rotation(i, i) - rotation(j, j) - rotation(k, k)) + 1;
            T s = T(0.5) / std::sqrt(t);
            T q[4];
            q[i] = s * t;
            q[3] = (rotation(k, j) - rotation(j, k)) * s;
            q[j] = (rotation(j, i) + rotation(i, j)) * s;
            q[k] = (rotation(k, i) + rotation(i, k)) * s;
            this->x = q[0];
            this->y = q[1];
            this->z = q[2];
            this->w = q[3];
        }

        this->normalize();
    }

    /**
     * Construct Quaternion from Euler angles in radians.
     *
     * In pitch-yaw-roll order.
     */
    Quaternion(T pitch, T yaw, T roll) {
        T cy = std::cos(roll * T(0.5));
        T sy = std::sin(roll * T(0.5));
        T cp = std::cos(yaw * T(0.5));
        T sp = std::sin(yaw * T(0.5));
        T cr = std::cos(pitch * T(0.5));
        T sr = std::sin(pitch * T(0.5));

        this->w = cy * cp * cr + sy * sp * sr;
        this->x = cy * cp * sr - sy * sp * cr;
        this->y = sy * cp * sr + cy * sp * cr;
        this->z = sy * cp * cr - cy * sp * sr;
    }

    /**
     * Return an identity quaternion.
     */
    static Quaternion Identity() {
        return Quaternion(0, 0, 0, 1);
    }

    /**
     * Return the reverse of this quaternion.
     */
    Quaternion Inverse() const {
        return Quaternion(-this->x, -this->y, -this->z, this->w);
    }

    /**
     * Convert quaternion to a rotation angle(in radians) and rotation axis.
     */
    void ToRotation(Vector3D<T>* rotation_axis, T* rotation_angle) const {
        CHECK(rotation_angle);
        CHECK(rotation_axis);

        T norm = this->norm();
        if (norm < std::numeric_limits<T>::epsilon()) {
            *rotation_angle = 0;
            *rotation_axis = Vector3D<T>(0, 0, 1);
            return;
        }

        *rotation_angle = T(2) * std::acos(this->w / norm);
        *rotation_axis = Vector3D<T>(this->x, this->y, this->z);
        rotation_axis->normalize();
    }

    /**
     * Convert quaternion to rotation matrix.
     */
    Matrix3<T> ToRotationMatrix() const {
        Matrix3<T> rotation;

        Quaternion q = *this;
        q.normalize();

        T sqw = q.w * q.w;
        T sqx = q.x * q.x;
        T sqy = q.y * q.y;
        T sqz = q.z * q.z;

        rotation[0] =  sqx - sqy - sqz + sqw;
        rotation[4] = -sqx + sqy - sqz + sqw;
        rotation[8] = -sqx - sqy + sqz + sqw;

        T tmp1 = q.x * q.y;
        T tmp2 = q.z * q.w;
        rotation[3] = T(2) * (tmp1 + tmp2);
        rotation[1] = T(2) * (tmp1 - tmp2);

        tmp1 = q.x * q.z;
        tmp2 = q.y * q.w;
        rotation[6] = T(2) * (tmp1 - tmp2);
        rotation[2] = T(2) * (tmp1 + tmp2);

        tmp1 = q.y * q.z;
        tmp2 = q.x * q.w;
        rotation[7] = T(2) * (tmp1 + tmp2);
        rotation[5] = T(2) * (tmp1 - tmp2);

        return rotation;
    }

    /**
     * Convert quaternion to Euler angles (in radian form).
     */
    void ToEulerAngles(T* pitch, T* yaw, T* roll) const {
        CHECK(pitch && yaw && roll);

        T x1 = this->x, y1 = this->y, z1 = this->z, w1 = this->w;

        // Pitch (X-axis rotation).
        T sinp = T(2) * (w1 * y1 - z1 * x1);
        if (std::abs(sinp) >= T(1))
            // Use 90 degrees if out of range
            *yaw = std::copysign(numbers::pi_v<T> / 2, sinp);
        else
            *yaw = std::asin(sinp);

        // Yaw (Y-axis rotation).
        T siny = T(2) * (w1 * x1 + y1 * z1);
        T cosy = T(1) - T(2) * (x1 * x1 + y1 * y1);
        *pitch = std::atan2(siny, cosy);

        // Roll (Z-axis rotation).
        T sinr = T(2) * (w1 * z1 + x1 * y1);
        T cosr = T(1) - T(2) * (y1 * y1 + z1 * z1);
        *roll = std::atan2(sinr, cosr);
    }

    /**
     * Set this quaternion to the product of two quaternions, and return the
     * product.
     * Two rotation quaternions can be combined into one equivalent quaternion
     * by multiplication.
     */
    Quaternion& Multiply(const Quaternion& a, const Quaternion& b) {
        T x1 = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        T y1 = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
        T z1 = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x;
        T w1 = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        this->x = x1;
        this->y = y1;
        this->z = z1;
        this->w = w1;
        return *this;
    }

    Quaternion& operator *=(const Quaternion& rhs) {
        return Multiply(*this, rhs);
    }

    /**
     * Rotate the vector by q.
     *
     * It equals q * v * q.inverse().
     */
    friend Vector3D<T> operator *(const Quaternion& q, const Vector3D<T>& v) {
        Quaternion v1(v.x, v.y, v.z, 0);
        v1 = q * v1 * q.Inverse();
        Vector3D<T> r(v1.x, v1.y, v1.z);
        return r;
    }

    friend Quaternion operator *(const Quaternion& lhs, const Quaternion& rhs) {
        return Quaternion().Multiply(lhs, rhs);
    }

    /**
     * Compute the spherical linear interpolation between two quaternions.
     * The parameter 't' argument has to be such that 0 <= t <= 1.
     */
    Quaternion Interpolation(T t, const Quaternion<T>& rhs) const {
        CHECK(t >= 0 && t <= 1);

        T invert = 1;

        // Compute cos(theta) using the quaternion scalar product.
        T cosine_theta = this->dot(rhs);

        // Take care of the sign of cosine_theta.
        if (cosine_theta < 0) {
            cosine_theta = -cosine_theta;
            invert = -1;
        }

        // theta will close to 0 when cos(theta) close to 1. We can rewrite
        // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
        const T epsilon = T(0.00001);
        if (1 - cosine_theta < epsilon) {
            return *this * (1 - t) + rhs * (t * invert);
        }

        // Compute the theta angle.
        T theta = std::acos(cosine_theta);

        // Compute sin(theta).
        T sine_theta = std::sin(theta);

        // Compute the two coefficients that are in the spherical linear
        // interpolation formula
        T coeff1 = std::sin((1 - t) * theta) / sine_theta;
        T coeff2 = std::sin(t * theta) / sine_theta * invert;

        // Compute and return the interpolated quaternion.
        return *this * coeff1 + rhs * coeff2;
    }

    friend std::ostream& operator <<(std::ostream& os, const Quaternion& q) {
        Message msg(q.x, q.y, q.z, q.w);
        return os << msg;
    }
};

using FQuaternion = Quaternion<float>;
using RQuaternion = Quaternion<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_QUATERNION_H_

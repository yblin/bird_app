//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_ISOMETRY_H_
#define CODELIBRARY_MATH_MATRIX_ISOMETRY_H_

#include <limits>

#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/math/matrix/matrix3.h"
#include "codelibrary/math/matrix/matrix4.h"
#include "codelibrary/math/vector.h"

namespace cl {
namespace se3 {

/**
 * Create skew-symmetric metrix from rotation vector.
 */
template <typename T>
Matrix3<T> Skew(const Vector3D<T>& v) {
    Matrix3<T> skew(0.0,   -v[2],   v[1],
                    v[2],    0.0,  -v[0],
                    -v[1],  v[0],   0.0);
    return skew;
}

/**
 * Compute the SE(3) transformation matrix from a twist vector.
 *
 * This function computes the SE(3) transformation matrix corresponding to a
 * twist vector in the form [rx, ry, rz, tx, ty, tz], where [rx, ry, rz]
 * represents the rotation (as an axis-angle vector) and [tx, ty, tz] represents
 * the translation.
 */
template <typename T>
Matrix4<T> Exp(const Vector<T>& twist) {
    CHECK(twist.size() == 6);

    Matrix4<T> se3 = Matrix4<T>::Identity();

    const Vector3D<T> omega(twist[0], twist[1], twist[2]);
    T theta = omega.norm();
    Quaternion<T> q(omega.normalized(), theta);
    Matrix3<T> rot = q.ToRotationMatrix();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            se3(i, j) = rot(i, j);
        }
    }

    Matrix3<T> omega_hat = se3::Skew(omega);
    Matrix3<T> omega_hat_sq = omega_hat * omega_hat;
    Matrix3<T> v;

    T norm2 = omega.squared_norm();
    if (norm2 < std::numeric_limits<T>::epsilon()) {
        v = Matrix3<T>::Identity() + T(0.5) * omega_hat;
    } else {
        T norm = std::sqrt(norm2);
        v = Matrix3<T>::Identity() +
            (T(1) - std::cos(norm)) / norm2 * omega_hat +
            (norm - std::sin(norm)) / (norm2 * norm) * omega_hat_sq;
    }
    Vector3D t(twist[3], twist[4], twist[5]);
    t = v * t;
    se3(0, 3) = t[0];
    se3(1, 3) = t[1];
    se3(2, 3) = t[2];
    return se3;
}
template <typename T>
Matrix4<T> Exp(T rx, T ry, T rz, T tx, T ty, T tz) {
    Vector twist = make_vector(rx, ry, rz, tx, ty, tz);
    return Exp(twist);
}

/**
 * Logarithmic map for SE(3).
 *
 * This function computes the logarithm of an SE(3) transformation, which is
 * the inverse of the group exponential. It maps an element of the group
 * (rigid body transformations) to an element of the tangent space (twist).
 */
template <typename T>
Vector<T> Log(const Matrix4<T>& se3) {
    Vector<T> twist(6);
    Matrix3<T> rot;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot(i, j) = se3(i, j);
        }
    }

    Vector3D<T> omega;
    T theta;
    Quaternion<T> q(rot);
    q.ToRotation(&omega, &theta);

    omega *= theta;
    twist[0] = omega[0];
    twist[1] = omega[1];
    twist[2] = omega[2];

    T theta_sq = theta * theta;
    Matrix3<T> omega_hat = Skew(omega);

    Matrix3<T> v_inv;
    if (theta_sq < std::numeric_limits<T>::epsilon()) {
        v_inv = Matrix3<T>::Identity() - T(0.5) * omega_hat +
                T(1.0 / 12.0) * (omega_hat * omega_hat);
    } else {
        T half_theta = T(0.5) * theta;

        v_inv = Matrix3<T>::Identity() - T(0.5) * omega_hat +
                (T(1) - T(0.5) * theta * std::cos(half_theta) /
                 std::sin(half_theta)) / theta_sq * (omega_hat * omega_hat);
    }

    Vector3D<T> t(se3(0, 3), se3(1, 3), se3(2, 3));
    t = v_inv * t;
    twist[3] = t[0];
    twist[4] = t[1];
    twist[5] = t[2];

    return twist;
}

} // namespace se3
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_ISOMETRY_H_

//
// Copyright 2021-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_MATRIX3_H_
#define CODELIBRARY_MATH_MATRIX_MATRIX3_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/vector_3d.h"

// Unrolling the loop can speed up by 2 times.
#define UNROLL_MATRIX3_OPERATION1(a, b, op) \
    a[0] op b; \
    a[1] op b; \
    a[2] op b; \
    a[3] op b; \
    a[4] op b; \
    a[5] op b; \
    a[6] op b; \
    a[7] op b; \
    a[8] op b

#define UNROLL_MATRIX3_OPERATION2(a, b, op) \
    a[0] op b[0]; \
    a[1] op b[1]; \
    a[2] op b[2]; \
    a[3] op b[3]; \
    a[4] op b[4]; \
    a[5] op b[5]; \
    a[6] op b[6]; \
    a[7] op b[7]; \
    a[8] op b[8]

namespace cl {

template <typename T>
class Matrix3 {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using iterator = T*;
    using const_iterator = const T*;

    Matrix3() {
        std::fill(data_ + 0, data_ + 9, T(0));
    }

    explicit Matrix3(const Array<T>& data) {
        assign(data);
    }

    Matrix3(T m0, T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8) {
        this->assign(m0, m1, m2, m3, m4, m5, m6, m7, m8);
    }

    Matrix3(const Matrix3& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 9, data_);
    }

    static Matrix3 Identity() {
        Matrix3 mat;
        mat[0] = mat[4] = mat[8] = 1;
        return mat;
    }

    /**
     * Setup an identity matrix.
     */
    void SetIdentity() {
        std::fill(data_ + 0, data_ + 9, T(0));
        data_[0] = data_[4] = data_[8] = 1;
    }

    /**
     * Assign matrix.
     */
    void assign(const Array<T>& data) {
        CHECK(data.size() == 9);

        std::copy(data.begin(), data.end(), data_);
    }

    /**
     * Assign matrix.
     */
    void assign(T m0, T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8) {
        data_[0] = m0;
        data_[1] = m1;
        data_[2] = m2;
        data_[3] = m3;
        data_[4] = m4;
        data_[5] = m5;
        data_[6] = m6;
        data_[7] = m7;
        data_[8] = m8;
    }

    /**
     * Return the inverse transform.
     */
    Matrix3 Inverse() const {
        static_assert(std::is_floating_point<T>::value, "");

        Matrix3 t;
        T* dst = t.data();
        const T* src = data_;

        // Compute adjoint:
        dst[0] = + src[4] * src[8] - src[5] * src[7];
        dst[1] = - src[1] * src[8] + src[2] * src[7];
        dst[2] = + src[1] * src[5] - src[2] * src[4];
        dst[3] = - src[3] * src[8] + src[5] * src[6];
        dst[4] = + src[0] * src[8] - src[2] * src[6];
        dst[5] = - src[0] * src[5] + src[2] * src[3];
        dst[6] = + src[3] * src[7] - src[4] * src[6];
        dst[7] = - src[0] * src[7] + src[1] * src[6];
        dst[8] = + src[0] * src[4] - src[1] * src[3];

        // Compute determinant:
        T det = src[0] * dst[0] + src[1] * dst[3] + src[2] * dst[6];

        // Multiply adjoint with reciprocal of determinant:
        T norm = T(1) / det;
        for (int i = 0; i < 9; ++i)
            dst[i] *= norm;

        return t;
    }

    /**
     * Return the transpose transform.
     */
    Matrix3 Transpose() const {
        Matrix3 t;
        t.data_[0] = data_[0];
        t.data_[1] = data_[3];
        t.data_[2] = data_[6];
        t.data_[3] = data_[1];
        t.data_[4] = data_[4];
        t.data_[5] = data_[7];
        t.data_[6] = data_[2];
        t.data_[7] = data_[5];
        t.data_[8] = data_[8];
        return t;
    }

    // O(1) access the members.
    const T* data()        const { return data_;     }
    T* data()                    { return data_;     }
    T operator[] (int i)   const { return data_[i];  }
    T& operator[] (int i)        { return data_[i];  }
    int n_rows()           const { return 3;         }
    int n_columns()        const { return 3;         }
    iterator begin()             { return data_;     }
    const_iterator begin() const { return data_;     }
    iterator end()               { return data_ + 9; }
    const_iterator end()   const { return data_ + 9; }

    const T& operator() (int i, int j) const {
        return data_[i * 3 + j];
    }

    T& operator() (int i, int j) {
        return data_[i * 3 + j];
    }

    Matrix3& operator =(const Matrix3& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 9, data_);
        return *this;
    }

    Matrix3& operator =(Matrix3&& rhs) noexcept {
        std::swap(data_, rhs.data_);
        return *this;
    }

    Matrix3& operator+=(const Matrix3& rhs) {
        UNROLL_MATRIX3_OPERATION2(data_, rhs, +=);
        return *this;
    }

    Matrix3& operator-=(const Matrix3& rhs) {
        UNROLL_MATRIX3_OPERATION2(data_, rhs, -=);
        return *this;
    }

    Matrix3& operator*=(T rhs) {
        UNROLL_MATRIX3_OPERATION1(data_, rhs, *=);
        return *this;
    }

    friend Matrix3 operator *(const Matrix3& lhs, const T& rhs) {
        Matrix3 res = lhs;
        res *= rhs;
        return res;
    }

    friend Matrix3 operator *(const T& lhs, const Matrix3& rhs) {
        Matrix3 res = rhs;
        res *= lhs;
        return res;
    }

    friend Matrix3 operator +(const Matrix3& a, const Matrix3& b) {
        Matrix3 res = a;
        res += b;
        return res;
    }

    friend Matrix3 operator -(const Matrix3& a, const Matrix3& b) {
        Matrix3 res = a;
        res -= b;
        return res;
    }

    friend Matrix3 operator *(const Matrix3& lhs, const Matrix3& rhs) {
        Matrix3 res;
        T* c = res.data();
        const T* a = lhs.data();
        const T* b = rhs.data();

        c[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
        c[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
        c[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

        c[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
        c[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
        c[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

        c[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
        c[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
        c[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

        return res;
    }

    friend Vector3D<T> operator *(const Matrix3& mat, const Vector3D<T>& v) {
        Vector3D<T> res;
        const T* data = mat.data_;
        res.x = data[0] * v.x + data[1] * v.y + data[2] * v.z;
        res.y = data[3] * v.x + data[4] * v.y + data[5] * v.z;
        res.z = data[6] * v.x + data[7] * v.y + data[8] * v.z;
        return res;
    }

    friend Vector3D<T> operator *(const Vector3D<T>& v, const Matrix3& mat) {
        Vector3D<T> res;
        const T* data = mat.data_;
        res.x = data[0] * v.x + data[3] * v.y + data[6] * v.z;
        res.y = data[1] * v.x + data[4] * v.y + data[7] * v.z;
        res.z = data[2] * v.x + data[5] * v.y + data[8] * v.z;
        return res;
    }

    /**
     * Just for debug.
     */
    friend Message& operator <<(Message& msg, const Matrix3& rhs) {
        return msg.Append(rhs.data_, rhs.data_ + 9, {3, 3});
    }

protected:
    T data_[9];
};

using IMatrix3 = Matrix3<int>;
using FMatrix3 = Matrix3<float>;
using RMatrix3 = Matrix3<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_MATRIX3_H_


//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_MATRIX_H_
#define CODELIBRARY_MATH_MATRIX_MATRIX_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/math/linear_algebra.h"
#include "codelibrary/math/vector.h"

namespace cl {

/**
 * Dense row-major matrix.
 *
 * I don't want to implement a complete dense matrix library, but just provide
 * some simple data sturctures and low level operations as the interface for
 * more complicate third party libraries.
 */
template <typename T>
class Matrix {
public:
    using value_type             = T;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    Matrix() = default;

    /**
     * Construct a vector without initialization.
     */
    Matrix(int n_rows, int n_columns, const T& v = 0) {
        this->assign(n_rows, n_columns, v);
    }

    template <typename Iterator>
    Matrix(int n_rows, int n_columns, Iterator first, Iterator last) {
        this->assign(n_rows, n_columns, first, last);
    }

    Matrix(int n_rows, int n_columns, const Array<T>& data)
        : n_rows_(n_rows), n_columns_(n_columns), data_(data) {
        CHECK(n_rows_ * n_columns_ == data.size());
    }

    explicit Matrix(const ArrayND<T>& data) {
        this->assign(data);
    }

    Matrix(const Matrix& rhs) = default;

    Matrix(Matrix&& rhs) = default;

    virtual ~Matrix() = default;

    /**
     * Return an identity matrix.
     */
    static Matrix Identity(int n) {
        Matrix mat(n, n);
        for (int i = 0; i < n; ++i) {
            mat(i, i) = 1;
        }
        return mat;
    }

    Matrix& operator =(const Matrix&) = default;

    Matrix& operator =(Matrix&&) = default;

    Matrix& operator *=(const T& rhs) {
        blas::Scale(this->size(), rhs, data_.data());
        return *this;
    }

    Matrix& operator +=(const Matrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_);
        CHECK(n_columns_ == rhs.n_columns_);

        blas::Add(this->size(), this->data(), rhs.data(), data_.data());
        return *this;
    }

    Matrix& operator -=(const Matrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_);
        CHECK(n_columns_ == rhs.n_columns_);

        blas::Subtract(this->size(), this->data(), rhs.data(), this->data());
        return *this;
    }

    const T& operator() (int i, int j) const {
        return this->data_[i * n_columns_ + j];
    }

    T& operator() (int i, int j) {
        return this->data_[i * n_columns_ + j];
    }

    /**
     * Assign the matrix.
     */
    void assign(int n_rows, int n_columns, const T& v = T(0)) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";

        n_rows_ = n_rows;
        n_columns_ = n_columns;
        data_.assign(n_rows * n_columns, v);
    }

    /**
     * this = [first, last).
     */
    template <typename Iterator>
    void assign(int n_rows, int n_columns, Iterator first, Iterator last) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of the matrix are too large.";
        CHECK(std::distance(first, last) == n_rows_ * n_columns_);

        n_rows_ = n_rows;
        n_columns_ = n_columns;
        data_.assign(n_rows_ * n_columns_, first, last);
    }

    /**
     * Assign the matrix.
     */
    void assign(const ArrayND<T>& data) {
        CHECK(data.n_dimension() == 2);

        this->assign(data.shape(0), data.shape(1), data.begin(), data.end());
    }

    /**
     * Reshape the matrix. The extra elements are filled with the specific
     * value.
     */
    void Resize(int n_rows, int n_columns, const T& value = 0) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";

        if (n_columns == n_columns_) {
            n_rows_ = n_rows;
            data_.resize(n_rows_ * n_columns_, value);
        } else {
            Array<T> tmp = data_;
            data_.assign(n_rows * n_columns, value);

            int min_n_rows = std::min(n_rows_, n_rows);
            int min_n_columns = std::min(n_columns_, n_columns);
            for (int i = 0; i < min_n_rows; ++i) {
                std::copy_n(tmp.begin() + i * n_columns_, min_n_columns,
                            data_ + i * n_columns);
            }
            n_rows_ = n_rows;
            n_columns_ = n_columns;
        }
    }

    /**
     * Return the transpose matrix.
     */
    Matrix Transpose() const {
        Matrix b(n_columns_, n_rows_);
        int i = 0, j = 0;
        int n = n_rows_ * n_columns_;
        for (int t = 0; t < n; ++t) {
            b[t] = this->operator[](n_rows_ * j + i);
            if (++j == n_columns_) {
                j = 0;
                ++i;
            }
        }
        return b;
    }

    /**
     * Fill the matrix.
     */
    void Fill(const T& v) {
        std::fill(data_.begin(), data_.end(), v);
    }

    int n_rows() const {
        return n_rows_;
    }

    int n_columns() const {
        return n_columns_;
    }

    int size() const {
        return n_rows_ * n_columns_;
    }

    const T* data() const {
        return data_.data();
    }

    friend const Matrix& operator+(const Matrix& rhs) {
        return rhs;
    }

    friend Matrix operator -(const Matrix& rhs) {
        Matrix res = rhs;
        blas::Negate(res.size(), res.data());
        return res;
    }

    friend Matrix operator *(const Matrix& lhs, const T& rhs) {
        Matrix res = lhs;
        res *= rhs;
        return res;
    }

    friend Matrix operator *(const T& lhs, const Matrix& rhs) {
        Matrix res = rhs;
        res *= lhs;
        return res;
    }

    friend Matrix operator +(const Matrix& a, const Matrix& b) {
        Matrix c = a;
        c += b;
        return c;
    }

    friend Matrix operator -(const Matrix& a, const Matrix& b) {
        Matrix c = a;
        c -= b;
        return c;
    }

    friend Vector<T> operator *(const Matrix<T>& a, const Vector<T>& b) {
        CHECK(a.n_columns_ == b.size());

        Vector<T> c(a.n_rows_);
        blas::GEMV(a.n_rows_, a.n_columns_, a.data(), b.data(), c.data());
        return c;
    }

    friend Vector<T> operator *(const Vector<T>& b, const Matrix<T>& a) {
        CHECK(a.n_rows_ == b.size());

        Vector<T> c(a.n_columns_);
        blas::GEMVTrans(a.n_rows_, a.n_columns_, a.data(), b.data(), c.data());
        return c;
    }

    /**
     * For debug.
     */
    friend std::ostream& operator <<(std::ostream& os, const Matrix& rhs) {
        ArrayND<T> t(rhs.n_rows_, rhs.n_columns_);
        t.assign(rhs.begin(), rhs.end());
        Message msg;
        msg << t;

        return os << msg;
    }

private:
    // Number of rows in the matrix.
    int n_rows_ = 0;

    // Number of columns in the matrix.
    int n_columns_ = 0;

    // Data of matrix.
    Array<T> data_;
};

using IMatrix = Matrix<int>;
using FMatrix = Matrix<float>;
using RMatrix = Matrix<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_MATRIX_H_

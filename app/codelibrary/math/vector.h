//
// Copyright 2023-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_VECTOR_H_
#define CODELIBRARY_MATH_VECTOR_H_

#include "codelibrary/base/array.h"
#include "codelibrary/math/linear_algebra.h"

namespace cl {

/**
 * N-dimension vector.
 */
template <typename T>
class Vector : public Array<T> {
public:
    /**
     * Default vector constructor.
     */
    Vector() = default;

    /**
     * Construct a vector with an initial value.
     */
    Vector(int size, const T& v = T(0))
        : Array<T>(size, v) {}

    /**
     * Construct vector from data in [first, last).
     *
     * The second template parameter is used to distinguish this function to
     * Vector(int, int).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Vector(Iter first, Iter last)
        : Array<T>(first, last) {}

    /**
     * Build vector from initializer list.
     *
     * Note that this constructor cannot be explicit.
     */
    Vector(std::initializer_list<T> list)
        : Array<T>(list) {}

    Vector(const Vector& rhs)
        : Array<T>(rhs) {}

    Vector(Vector&& rhs)
        : Array<T>(rhs) {}

    virtual ~Vector() = default;

    Vector& operator =(const Vector& rhs) {
        Array<T>::operator=(rhs);
        return *this;
    }

    Vector& operator *=(const T& rhs) {
        blas::Scale(this->size(), rhs, this->data());
        return *this;
    }

    Vector& operator +=(const Vector& rhs) {
        CHECK(this->size() == rhs.size());

        blas::Add(this->size(), this->data(), rhs.data(), this->data());
        return *this;
    }

    Vector& operator -=(const Vector& rhs) {
        CHECK(this->size() == rhs.size());

        blas::Subtract(this->size(), this->data(), rhs.data(), this->data());
        return *this;
    }

    Vector& operator *=(const Vector& rhs) {
        CHECK(this->size() == rhs.size());

        blas::Multiply(this->size(), this->data(), rhs.data(), this->data());
        return *this;
    }

    bool operator ==(const Vector& rhs) const {
        return std::equal(this->begin(), this->end(), rhs.begin(), rhs.size());
    }

    bool operator !=(const Vector& rhs) const {
        return !(*this == rhs);
    }

    /**
     * Set all vector elements to zero.
     */
    void set_zero() {
        std::memset(this->data(), 0, sizeof(T) * this->size());
    }

    /**
     * Fill the vector.
     */
    void fill(const T& v) {
        if (this->data()) std::fill_n(this->data(), this->size(), v);
    }

    /**
     * Return euclidean norm.
     */
    T norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return blas::EuclideanNorm(this->size(), this->data());
    }

    /**
     * Return the squared euclidean norm.
     */
    T squared_norm() const {
        return blas::DotProduct(this->size(), this->data(), this->data());
    }

    /**
     * Infinity norm.
     */
    T infinty_norm() const {
        return blas::ASum(this->size(), this->data());
    }

    /**
     * Return the dot product of two vectors.
     */
    T dot(const Vector& rhs) const {
        CHECK(this->size() == rhs.size());

        return blas::DotProduct(this->size(), this->data(), rhs.data());
    }

    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector& normalize() {
        static_assert(std::is_floating_point<T>::value, "");

        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            this->fill(0);
            this->back() = T(1);
        } else {
            this->operator*=(T(1) / len);
        }
        return *this;
    }

    /**
     * Return the normalized vector.
     * The length of the normalized vector is always one.
     */
    Vector normalized() const {
        static_assert(std::is_floating_point<T>::value);

        T len = this->norm();
        if (len < std::numeric_limits<T>::epsilon()) {
            Vector v(this->size(), T(0));
            v.back() = T(1);
            return v;
        }

        Vector v = *this;
        v *= T(1) / len;
        return v;
    }

    /**
     * Compute the uary positive of a vector.
     */
    friend const Vector& operator+(const Vector& rhs) {
        return rhs;
    }

    /**
     * Compute the negative of a vector.
     */
    friend Vector operator -(const Vector& rhs) {
        Vector res = rhs;
        blas::Negate(res.size(), res.data());
        return res;
    }

    /**
     * Compute the product of a vector by a scalar.
     */
    friend Vector operator *(const Vector& lhs, const T& rhs) {
        Vector res = lhs;
        res *= rhs;
        return res;
    }

    /**
     * Compute the product of a scale by a vector.
     */
    friend Vector operator *(const T& lhs, const Vector& rhs) {
        Vector res = rhs;
        res *= lhs;
        return res;
    }

    /**
     * Perform element by element addition of vector a and vector b.
     */
    friend Vector operator +(const Vector& a, const Vector& b) {
        Vector c(a);
        c += b;
        return c;
    }

    /**
     * Perform element by element subtraction of vector b from vector a.
     */
    friend Vector operator -(const Vector& a, const Vector& b) {
        Vector c = a;
        c -= b;
        return c;
    }

    /**
     * Perform element by element multiplication of vector a and vector b.
     */
    friend Vector operator *(const Vector& a, const Vector& b) {
        Vector c = a;
        c *= b;
        return c;
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector& v) {
        Message msg(v.begin(), v.end());
        return os << msg;
    }
};

using IVector = Vector<int>;
using FVector = Vector<float>;
using RVector = Vector<double>;

/**
 * Similar to the STL's make_pair() function, this utility constructs a vector.
 *
 * The function adopts an STL-style naming convention for consistency.
 */
template <typename T, typename... Args>
Vector<T> make_vector(const T& first, Args... args) {
    std::initializer_list<T> init_list = {first, static_cast<T>(args)...};
    return Vector<T>(init_list);
}

} // namespace cl

#endif // CODELIBRARY_MATH_VECTOR_H_

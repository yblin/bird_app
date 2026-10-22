//
// Copyright 2019-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_ARRAY_ND_H_
#define CODELIBRARY_BASE_ARRAY_ND_H_

#include <limits>
#include <utility>
#include <vector>

#include "codelibrary/base/array.h"

namespace cl {

/**
 * N-dimension array.
 */
template <typename T>
class ArrayND {
public:
    using Index = std::conditional<sizeof(size_t) == sizeof(int),
                                   int, int64_t>::type;
    static_assert(sizeof(Index) == sizeof(size_t),
                  "Index type size does not match size_t size.");

    static constexpr Index MAX_SIZE = std::numeric_limits<Index>::max();

    using value_type             = T;
    using pointer                = T*;
    using const_pointer          = const T *;
    using reference              = T&;
    using const_reference        = const T&;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type              = Index;
    using difference_type        = Index;

    ArrayND() = default;

    explicit ArrayND(int size) {
        reshape(size);
    }

    ArrayND(int size1, int size2) {
        reshape(size1, size2);
    }
    ArrayND(int size1, int size2, int size3) {
        reshape(size1, size2, size3);
    }
    ArrayND(int size1, int size2, int size3, int size4) {
        reshape(size1, size2, size3, size4);
    }
    ArrayND(int size1, int size2, int size3, int size4, int size5) {
        reshape(size1, size2, size3, size4, size5);
    }
    ArrayND(int size1, int size2, int size3, int size4, int size5, int size6) {
        reshape(size1, size2, size3, size4, size5, size6);
    }

    explicit ArrayND(const Array<int>& shape) {
        reshape(shape);
    }

    ArrayND(const Array<int>& shape, const T& v) {
        reshape(shape);
        fill(v);
    }

    template <typename Iterator>
    ArrayND(const Array<int>& shape, Iterator first, Iterator last) {
        auto n = std::distance(first, last);
        CHECK(n <= MAX_SIZE) << "The number of input elements exceeds the "
                                "maximum value of 'Index' type.";

        data_.assign(first, last);
        Index size = static_cast<Index>(data_.size());
        reshape(shape);
        CHECK(size == size_) << "The input dimensions are " << Message(shape)
                             << ", which does not match the input data size ("
                             << size << ").";
    }

    ArrayND(const Array<int>& shape, const Array<T, Index>& data)
        : data_(data) {
        reshape(shape);
        CHECK(data.size() == size_)
            << "The input dimensions are " << Message(shape)
            << ", which does not match the input data size ("
            << data.size() << ").";
    }

    const T& operator()(int a1) const {
        return data_[a1];
    }

    T& operator()(int a1) {
        return data_[a1];
    }

    const T& operator()(int a1, int a2) const {
        return data_[a1 * strides_[0] + a2];
    }

    T& operator()(int a1, int a2) {
        return data_[a1 * strides_[0] + a2];
    }

    const T& operator()(int a1, int a2, int a3) const {
        return data_[a1 * strides_[0] + a2 * strides_[1] + a3];
    }

    T& operator()(int a1, int a2, int a3) {
        return data_[a1 * strides_[0] + a2 * strides_[1] + a3];
    }

    const T& operator()(int a1, int a2, int a3, int a4) const {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4];
    }

    T& operator()(int a1, int a2, int a3, int a4) {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4];
    }

    const T& operator()(int a1, int a2, int a3, int a4, int a5) const {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5];
    }

    T& operator()(int a1, int a2, int a3, int a4, int a5) {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5];
    }

    const T& operator()(int a1, int a2, int a3, int a4, int a5, int a6) const {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5 * strides_[4] + a6];
    }

    T& operator()(int a1, int a2, int a3, int a4, int a5, int a6) {
        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5 * strides_[4] + a6];
    }

    const T& at(int a1) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);

        return data_[a1];
    }

    T& at(int a1) {
        CHECK(a1 >= 0 && a1 < shape_[0]);

        return data_[a1];
    }

    const T& at(int a1, int a2) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);

        return data_[a1 * strides_[0] + a2];
    }

    T& at(int a1, int a2) {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);

        return data_[a1 * strides_[0] + a2];
    }

    const T& at(int a1, int a2, int a3) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);

        return data_[a1 * strides_[0] + a2 * strides_[1] + a3];
    }

    T& at(int a1, int a2, int a3) {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);

        return data_[a1 * strides_[0] + a2 * strides_[1] + a3];
    }

    const T& at(int a1, int a2, int a3, int a4) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4];
    }

    T& at(int a1, int a2, int a3, int a4) {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4];
    }

    const T& at(int a1, int a2, int a3, int a4, int a5) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);
        CHECK(a5 >= 0 && a5 < shape_[4]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5];
    }

    T& at(int a1, int a2, int a3, int a4, int a5) {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);
        CHECK(a5 >= 0 && a5 < shape_[4]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5];
    }

    const T& at(int a1, int a2, int a3, int a4, int a5, int a6) const {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);
        CHECK(a5 >= 0 && a5 < shape_[4]);
        CHECK(a6 >= 0 && a6 < shape_[5]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5 * strides_[4] + a6];
    }

    T& at(int a1, int a2, int a3, int a4, int a5, int a6) {
        CHECK(a1 >= 0 && a1 < shape_[0]);
        CHECK(a2 >= 0 && a2 < shape_[1]);
        CHECK(a3 >= 0 && a3 < shape_[2]);
        CHECK(a4 >= 0 && a4 < shape_[3]);
        CHECK(a5 >= 0 && a5 < shape_[4]);
        CHECK(a6 >= 0 && a6 < shape_[5]);

        return data_[a1 * strides_[0] + a2 * strides_[1] +
                     a3 * strides_[2] + a4 * strides_[3] +
                     a5 * strides_[4] + a6];
    }

    const T& operator[](int index) const {
        return data_[index];
    }

    T& operator[](int index) {
        return data_[index];
    }

    /**
     * Reshape the ND array.
     */
    void reshape(int a1) {
        CHECK(a1 >= 0);

        size_ = a1;
        shape_.resize(1);
        shape_[0] = a1;
        data_.resize(size_, T());
        strides_ = {1};
    }
    void reshape(int a1, int a2) {
        reshape({ a1, a2 });
    }
    void reshape(int a1, int a2, int a3) {
        reshape({ a1, a2, a3 });
    }
    void reshape(int a1, int a2, int a3, int a4) {
        reshape({ a1, a2, a3, a4 });
    }
    void reshape(int a1, int a2, int a3, int a4, int a5) {
        reshape({ a1, a2, a3, a4, a5 });
    }
    void reshape(int a1, int a2, int a3, int a4, int a5, int a6) {
        reshape({ a1, a2, a3, a4, a5, a6});
    }
    void reshape(const Array<int>& shape) {
        CHECK(!shape.empty());

        size_ = 1;
        for (int s : shape) {
            CHECK(s >= 0);
            CHECK(s == 0 || size_ <= MAX_SIZE / s)
                  << "The given shape " << Message(shape.begin(), shape.end())
                  << " is too large.";

            size_ *= s;
        }

        shape_ = shape;
        data_.resize(size_, T());
        strides_.resize(shape_.size());
        strides_.back() = 1;
        for (int i = shape_.size() - 2; i >= 0; --i) {
            strides_[i] = strides_[i + 1] * shape_[i + 1];
        }
    }

    /**
     * Fill the array.
     */
    void fill(const T& v) {
        std::fill(data_.begin(), data_.end(), v);
    }

    iterator begin()                      { return data_.begin();  }
    iterator end()                        { return data_.end();    }
    const_iterator begin()          const { return data_.begin();  }
    const_iterator end()            const { return data_.end();    }
    reverse_iterator rbegin()             { return data_.rbegin(); }
    reverse_iterator rend()               { return data_.rend();   }
    const_reverse_iterator rbegin() const { return data_.rbegin(); }
    const_reverse_iterator rend()   const { return data_.rend();   }

    void clear() {
        size_ = 0;
        shape_.clear();
        data_.clear();
        strides_.clear();
    }

    int n_dimension() const {
        return shape_.size();
    }

    const Array<Index>& strides() const {
        return strides_;
    }

    Index stride(int i) const {
        CHECK(i >= 0 && i < strides_.size());

        return strides_[i];
    }

    const Array<int>& shape() const {
        return shape_;
    }

    int shape(int i) const {
        CHECK(i >= 0 && i < shape_.size());

        return shape_[i];
    }

    bool empty() const {
        return size_ == 0;
    }

    Index size() const {
        return size_;
    }

    T* data() {
        return data_.data();
    }

    const T* data() const {
        return data_.data();
    }

    void swap(ArrayND* t) {
        CHECK(t);

        std::swap(size_, t->size_);
        shape_.swap(t->shape_);
        strides_.swap(t->strides_);
        data_.swap(t->data_);
    }

    void assign(const Array<T, Index>& data) {
        CHECK(data.size() == size_);

        data_ = data;
    }

    template <typename Iterator>
    void assign(Iterator first, Iterator last) {
        auto n = std::distance(first, last);
        CHECK(n == size_);

        data_.assign(first, last);
    }

protected:
    // The total number of the ND array.
    Index size_ = 0;

    // The number of elements in each dimension.
    Array<int> shape_;

    // Array of steps in each dimension when traversing an array.
    // strides_[i] = shape_[i + 1] * ... * shape_[nd - 1].
    Array<Index> strides_;

    // The data of the ND array.
    Array<T, Index> data_;
};

/**
 * Message for ND array.
 */
template <typename T>
Message& operator <<(Message& msg, const ArrayND<T>& v) {
    std::vector<int> shape(v.shape().begin(), v.shape().end());
    return msg.Append(v.begin(), v.end(), shape);
}

} // namespace cl

#endif // CODELIBRARY_BASE_ARRAY_ND_H_

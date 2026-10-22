//
// Copyright 2016-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_ARRAY_H_
#define CODELIBRARY_BASE_ARRAY_H_

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include "codelibrary/base/log.h"
#include "codelibrary/base/memory.h"
#include "codelibrary/base/message.h"

namespace cl {

/**
 * A STL-like 1D array template class.
 *
 * It is essentially a STL vector, but has a 'int' or 'int64_t' typed 'size()'.
 * It means that there are at most 2^31-1 or 2^63-1 elements in the Array.
 *
 * Why 'int' capacity in default ?
 * 1. First, capacity of 31 bits is enough for many cases.
 * 2. Second, it can quickly report the errors before the memory is exhausted.
 *    This is especially useful for infinite loops situations in 64 - bit
 *    environments.
 * 3. Last but not least, in many machine learning tasks, we need to generate a
 *    label for each object. 'int' typed label is obviously more convenient then
 *    'size_t' typed label. Because it can use negative value to represent the
 *    outliers.
 *
 * In the implementation, we choose the growth factor to 1.5. Any number smaller
 * than 2 guarantees that you'll at some point be able to reuse the previous
 * chunks. For instance, choosing 1.5 as the factor allows memory reuse after 4
 * reallocations; 1.45 allows memory reuse after 3 reallocations; and 1.3 allows
 * reuse after only 2 reallocations.
 *
 * Time comparison :
 *
 * push_back :
 *        n    STL vector     Array
 *        1       69 ns       50 ns
 *        2      124 ns       51 ns
 *        4      270 ns       51 ns
 *        8      396 ns       56 ns
 *       10      468 ns       58 ns
 *      100        1 us      340 ns
 *     1000        2 us        1 us
 *    10000       52 us       38 us
 *   100000      343 us      177 us
 *  1000000        5 ms        4 ms
 * 10000000       76 ms       45 ms
 */
template <typename T, typename Index = int>
class Array {
    static_assert(std::is_same<Index, int>::value ||
                  std::is_same<Index, int64_t>::value,
                  "Index must be either int or int64_t");

public:
    using value_type             = T;
    using pointer                = T*;
    using const_pointer          = const T *;
    using reference              = T&;
    using const_reference        = const T &;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type              = Index;
    using difference_type        = Index;

    static constexpr Index MAX_SIZE = std::numeric_limits<Index>::max();
    static constexpr size_t ALIGNMENT = 32;

    /**
     * Default array constructor.
     */
    Array() = default;

    /**
     * Preallocate the 'size' data without initialize value.
     */
    explicit Array(Index size) {
        CHECK(size >= 0);

        Construct(size);
        Initialize(begin_, end_);
    }

    /**
     * Preallocate the 'size' data with initialize value.
     */
    Array(Index size, const T& v) {
        CHECK(size >= 0);

        Construct(size);
        std::uninitialized_fill(begin_, end_, v);
    }

    /**
     * Build array from data in [first, last).
     * The second template parameter is used to distinguish this function to
     * Array(Index, T).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Array(Iter first, Iter last) {
        auto n = std::distance(first, last);
        CHECK(n >= 0);
        CHECK(n <= MAX_SIZE) << "The number of input is " << n
                             << ", which exceeds the MAX_SIZE.";

        Construct(static_cast<Index>(n));
        std::uninitialized_copy(first, last, begin_);
    }

    /**
     * Copy constructor.
     */
    Array(const Array& v) {
        Construct(v.size());
        std::uninitialized_copy(v.begin(), v.end(), begin_);
    }

    /**
     * Move constructor.
     */
    Array(Array&& v) noexcept {
        swap(v);
    }

    /**
     * Build array from initializer list.
     * Note that this constructor cannot be explicit.
     */
    Array(std::initializer_list<T> list)
        : Array(list.begin(), list.end()) {}

    virtual ~Array() {
        Destruct();
    }

    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    void assign(Iterator first, Iterator last) {
        auto n = std::distance(first, last);
        CHECK(n >= 0);
        CHECK(n <= MAX_SIZE) << "The number of input is " << n
                             << ", which exceeds the MAX_SIZE.";

        resize(static_cast<Index>(n));
        std::copy(first, last, begin_);
    }

    /**
     * Assign 'n' elements of value 'v' to this array.
     */
    void assign(Index n, const T& v) {
        CHECK(n >= 0);
        if (n == 0) {
            clear();
        } else if (n < end_ - begin_) {
            std::fill_n(begin_, n, v);
            Destruct(begin_ + n, end_);
            end_ = begin_ + n;
        } else if (n > real_end_ - begin_) {
            // Need relocate the data.
            Destruct();
            Construct(n);
            std::uninitialized_fill(begin_, end_, v);
        } else {
            std::fill(begin_, end_, v);
            std::uninitialized_fill(end_, begin_ + n, v);
            end_ = begin_ + n;
        }
    }

    /**
     * Clear the array, free the data.
     */
    void clear() {
        if (begin_) {
            Destruct(begin_, end_);
            end_ = begin_;
        }
    }

    /**
     * Check if this array is empty.
     */
    bool empty() const {
        return begin_ == end_;
    }

    /**
     * Resize the array to the specified number of elements.
     */
    void resize(Index size) {
        CHECK(size >= 0);

        if (size > real_end_ - begin_) {
            reserve(size);
            Initialize(end_, real_end_);
        } else if (size < end_ - begin_) {
            Destruct(begin_ + size, end_);
        } else {
            Initialize(end_, begin_ + size);
        }
        end_ = begin_ + size;
    }

    /**
     * This function will resize the array to the specified number of elements.
     * If the number is smaller than the array's current size the array is
     * truncated, otherwise the array is extended and new elements are populated
     * with given value.
     */
    void resize(Index size, const T& v) {
        CHECK(size >= 0);

        if (size > real_end_ - begin_) {
            reserve(size);
            std::uninitialized_fill(end_, real_end_, v);
        } else if (size < end_ - begin_) {
            Destruct(begin_ + size, end_);
        } else {
            std::uninitialized_fill(end_, begin_ + size, v);
        }
        end_ = begin_ + size;
    }

    /**
     * Resize the array to the specified number of elements.
     */
    void reserve(Index capacity) {
        CHECK(capacity >= 0);

        if (real_end_ - begin_ >= capacity) return;

        T* begin = reinterpret_cast<T*>(AlignedAllocate(capacity * sizeof(T),
                                                        ALIGNMENT));
        if (begin_) {
            if (std::is_trivially_copyable<T>::value) {
                std::uninitialized_copy(begin_, end_, begin);
            } else {
                std::uninitialized_copy(std::make_move_iterator(begin_),
                                        std::make_move_iterator(end_),
                                        begin);
            }
            Destruct();
        }

        // Note that, end_ += begin - begin_ may overflow!
        end_ = begin + (end_ - begin_);
        begin_ = begin;
        real_end_ = begin_ + capacity;
    }

    /**
     * Swap the data with another array.
     * In order to warp to std::vector, we use reference as the parameter.
     */
    void swap(Array& rhs) {
        std::swap(begin_, rhs.begin_);
        std::swap(end_, rhs.end_);
        std::swap(real_end_, rhs.real_end_);
    }

    /**
     * Return the number of elements.
     */
    Index size() const {
        return static_cast<Index>(end_ - begin_);
    }

    T* data() {
        return begin_;
    }

    const T* data() const {
        return begin_;
    }

    iterator begin() {
        return begin_;
    }

    iterator end() {
        return end_;
    }

    const_iterator begin() const {
        return begin_;
    }

    const_iterator end() const {
        return end_;
    }

    reverse_iterator rbegin() {
        return reverse_iterator(end_);
    }

    reverse_iterator rend() {
        return reverse_iterator(begin_);
    }

    const_reverse_iterator rbegin() const {
        return const_reverse_iterator(end_);
    }

    const_reverse_iterator rend() const {
        return const_reverse_iterator(begin_);
    }

    T& operator[](Index index) {
        return *(begin_ + index);
    }

    const T& operator[](Index index) const {
        return *(begin_ + index);
    }

    T& at(Index index) {
        CHECK(index >= 0 && begin_ + index < end_) << "index = " << index
                                                   << ", size = " << size();

        return *(begin_ + index);
    }

    const T& at(Index index) const {
        CHECK(index >= 0 && begin_ + index < end_) << "index = " << index
                                                   << ", size = " << size();

        return *(begin_ + index);
    }

    /**
     * Move assignment.
     */
    Array& operator =(Array&& rhs) noexcept {
        swap(rhs);
        return *this;
    }

    /**
     * Copy assignment.
     */
    Array& operator =(const Array& rhs) {
        if (!rhs.empty()) {
            if (end_ - begin_ == rhs.size()) {
                std::copy(rhs.begin(), rhs.end(), begin_);
            } else {
                Destruct();
                Construct(rhs.size());
                std::uninitialized_copy(rhs.begin(), rhs.end(), begin_);
            }
        } else {
            Destruct();
            end_ = begin_ = real_end_ = nullptr;
        }
        return *this;
    }

    /**
     * Add an element as the last element of the array.
     */
    template <typename... Args>
    void emplace_back(Args&&... args) {
        if (end_ >= real_end_) {
            EmplaceBackAux(std::forward<Args>(args)...);
        } else {
            ConstructOneElement(end_++, std::forward<Args>(args)...);
        }
    }

    /**
     * Add an element as the last element of the array.
     */
    void push_back(const T& x) {
        if (end_ >= real_end_) {
            EmplaceBackAux(x);
        } else {
            ConstructOneElement(end_++, x);
        }
    }
    void push_back(T&& x) {
        if (end_ >= real_end_) {
            EmplaceBackAux(std::move(x));
        } else {
            ConstructOneElement(end_++, std::move(x));
        }
    }

    /**
     * Remove the last element.
     */
    void pop_back() {
        CHECK(end_ != begin_);

        Destruct(--end_);
    }

    /**
     * Return reference of the last element.
     */
    T& back() {
        CHECK(begin_ != end_);

        return *(end_ - 1);
    }

    /**
     * Return const reference of the last element.
     */
    const T& back() const {
        CHECK(begin_ != end_);

        return *(end_ - 1);
    }

    /**
     * Return reference of the first element.
     */
    T& front() {
        CHECK(begin_ != end_);

        return *begin_;
    }

    /**
     * Return const reference of the first element.
     */
    const T& front() const {
        CHECK(begin_ != end_);

        return *begin_;
    }

    /**
     * Find the first iterator whose element equal to x.
     */
    iterator find(const T& x) {
        for (iterator i = begin_; i != end_; ++i) {
            if (*i == x) {
                return i;
            }
        }
        return end_;
    }

    /**
     * Find the first iterator whose element equal to x.
     */
    const_iterator find(const T& x) const {
        for (const_iterator i = begin_; i != end_; ++i) {
            if (*i == x) {
                return i;
            }
        }
        return end_;
    }

    /**
     * Erase the element.
     */
    iterator erase(const_iterator p) {
        if (p == this->end()) return this->end();
        iterator p1 = (iterator)p;
        iterator p2 = p1;
        ++p1;
        std::copy(p1, this->end(), p2);
        this->pop_back();
        return p2;
    }

    /**
     * Range erase.
     *
     * The returned iterator following the last removed element.
     * If last == end() prior to removal, then the updated end() iterator is
     * returned. If [first, last) is an empty range, then last is returned.
     */
    iterator erase(const_iterator first, const_iterator last) {
        CHECK(first >= begin() && first <= end());
        CHECK(last >= begin() && last <= end());
        CHECK(first <= last);

        if (first == last) {
            return (iterator)first;
        }

        if (last != end()) {
            std::copy(std::make_move_iterator((iterator)last),
                      std::make_move_iterator(end()),
                      (iterator)first);
        }
        this->resize(this->size() - (last - first));
        return (iterator)first;
    }

    /**
     * Erase the first element x from the array.
     */
    void erase(const T& x) {
        for (T* p = begin_; p != end_; ++p) {
            if (*p == x) {
                std::copy(p + 1, end_, p);
                --end_;
                Destruct(end_);
                break;
            }
        }
    }

    /**
     * Unordered erase the i-th element.
     *
     * Note that this function will destroy the sequence of array.
     */
    void unordered_erase(Index index) {
        CHECK(index >= 0 && index < this->size());

        begin_[index] = *(end_ - 1);
        this->pop_back();
    }

    /**
     * Insert [first, last) at the back of this array.
     */
    template <typename Iterator>
    void insert(Iterator first, Iterator last) {
        for (Iterator p = first; p != last; ++p)
            this->push_back(*p);
    }

    /**
     * Insert a new array at the back of this array.
     */
    void insert(const Array& rhs) {
        insert(rhs.begin(), rhs.end());
    }

    /**
     * Insert an element at position.
     */
    iterator insert(const_iterator position, const T& value) {
        CHECK(position >= begin_ && position <= end_);

        size_t index = position - begin_;
        if (position == end_) {
            this->emplace_back(value);
        } else {
            // Need a copy to solve the bug in a.insert(a.front());
            T local_value = value;
            this->resize(this->size() + 1);
            std::copy_backward(begin_ + index, end_ - 1, end_);
            *(begin_ + index) = local_value;
        }

        return begin_ + index;
    }

    /**
     * Insert an element at position. (Move version)
     */
    iterator insert(const_iterator position, T&& value) {
        CHECK(position >= begin_ && position <= end_);

        size_t index = position - begin_;
        if (position == end_) {
            this->emplace_back(std::move(value));
        } else {
            // Need a copy to solve the bug in a.insert(a.front());
            T temp = std::move(value);
            this->resize(this->size() + 1);
            std::move_backward(begin_ + index, end_ - 1, end_);
            *(begin() + index) = temp;
        }

        return begin_ + index;
    }

    /**
     * Insert [first, last) at position.
     */
    template <typename Iterator>
    iterator insert(const_iterator position, Iterator first, Iterator last) {
        CHECK(position >= begin_ && position <= end_);

        size_t index = position - begin_;
        if (position == end_) {
            this->insert(first, last);
        } else {
            size_t count = std::distance(first, last);
            CHECK(index + count <= MAX_SIZE);

            this->resize(static_cast<Index>(this->size() + count));
            std::copy_backward(begin_ + index, end_ - count, end_);
            std::copy(first, last, begin_ + index);
        }

        return begin_ + index;
    }

protected:
    /**
     * Return a new size 'n' of push_pack.
     *
     * std::vector implements a similar function with a different growth
     * strategy: empty() ? 1 : capacity() * 2.
     *
     * Array<T> grows differently on two counts:
     *
     * (1) initial size
     *     Instead of growing to size 1 from empty, Array allocates at least
     *     64 bytes. You may still use reserve to reserve a lesser amount of
     *     memory.
     *
     * (2) 1.5x
     *     For medium sized arrays, the growth strategy is 1.5x.
     *     This does not apply to very small or very large array.
     *     It is a heuristic.
     *
     * (3) 2.0x
     *     For large sized array, the growth strategy is 2.0x.
     */
    static Index ExtendSize(Index size) {
        if (size == 0) {
            return std::max(static_cast<Index>(64 / sizeof(T)), 1);
        }

        if (size < 4096) {
            return 2 * size;
        }

        if (size > 4096 * 32) {
            CHECK(size < MAX_SIZE) << "The array is full.";
            return size > MAX_SIZE - size ? MAX_SIZE : size + size;
        }

        return (size * 3 + 1) / 2;
    }

    /**
     * Help function for emplace_back.
     */
    template <typename... Args>
    void EmplaceBackAux(Args&&... args) {
        Index size = static_cast<Index>(end_ - begin_);
        Index n = ExtendSize(size);
        CHECK(n > size);

        T* begin = reinterpret_cast<T*>(AlignedAllocate(n * sizeof(T),
                                                        ALIGNMENT));
        if (begin_) {
            // is_trivially_copy_assignable identifies if copy assignment would
            // be as valid as move assignment, which means we have the
            // opportunity to memcpy/memmove optimization.
            CopyOrMove(begin_, end_, begin,
                       std::is_trivially_copy_assignable<T>());
        }
        ConstructOneElement(begin + size, std::forward<Args>(args)...);
        Destruct();

        begin_ = begin;
        end_ = begin_ + size + 1;
        real_end_ = begin_ + n;
    }

    /**
     * Construct a new array with 'size' elements.
     */
    void Construct(Index size) {
        if (size != 0) {
            begin_ = reinterpret_cast<T*>(AlignedAllocate(size * sizeof(T),
                                                          ALIGNMENT));
            end_ = real_end_ = begin_ + size;
        }
    }

    /**
     * Construct a new element.
     */
    template <typename... Args>
    void ConstructOneElement(T* p, Args&&... args) const {
        new (p) T(std::forward<Args>(args)...);
    }

    /**
     * Destruct the array.
     */
    void Destruct() {
        if (begin_) {
            Destruct(begin_, end_);
            AlignedDeallocate(begin_);
        }
    }

    /**
     * Destruct [first, last).
     */
    void Destruct(T* first, T* last) const {
        Destruct(first, last, std::is_trivially_destructible<T>());
    }

    /**
     * True means the type has a trivial destructor.
     */
    void Destruct(T*, T*, std::true_type) const {
        // Empty. The type has a trivial destructor.
    }

    /**
     * False means the type has a significant destructor.
     */
    void Destruct(T* first, T* last, std::false_type) const {
        for (; last - first >= 4; first += 4) {
            first->~T();
            (first + 1)->~T();
            (first + 2)->~T();
            (first + 3)->~T();
        }

        for (; first != last; ++first)
            first->~T();
    }

    /**
     * Destroy one element.
     */
    void Destruct(T* p) {
        p->~T();
    }

    /**
     * Default initialization.
     */
    void Initialize(T* first, T* last) {
        Initialize(first, last, std::is_scalar<T>());
    }

    /**
     * For non scalar type data, we should use 'new T()' for the default
     * initialization.
     */
    void Initialize(T* first, T* last, std::false_type) {
        for (T* p = first; p != last; ++p) {
            new (p) T();
        }
    }

    /**
     * For scalar type data, we only clear the memory but not construct it.
     */
    void Initialize(T* first, T* last, std::true_type) {
        std::fill(first, last, T{});
    }

    void CopyOrMove(T* first, T* last, T* out, std::true_type) {
        std::uninitialized_copy(first, last, out);
    }

    void CopyOrMove(T* first, T* last, T* out, std::false_type) {
        std::uninitialized_copy(std::make_move_iterator(first),
                                std::make_move_iterator(last),
                                out);
    }

    T* begin_    = nullptr;
    T* end_      = nullptr;
    T* real_end_ = nullptr;
};

/**
 * Message for array.
 */
template <typename T, typename Index>
Message& operator <<(Message& msg, const Array<T, Index>& v) {
    return msg.Append(v.begin(), v.end());
}

} // namespace cl

#endif // CODELIBRARY_BASE_ARRAY_H_

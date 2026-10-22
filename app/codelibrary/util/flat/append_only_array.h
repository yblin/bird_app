//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_FLAT_APPEND_ONLY_ARRAY_H_
#define CODELIBRARY_UTIL_FLAT_APPEND_ONLY_ARRAY_H_

#include "codelibrary/base/array.h"

namespace cl {

/**
 * A dynamic array with strict "append-only" growth and lazy deletion.
 *
 * Core Characteristics:
 * 1. Strictly append-only: Elements can only be added via push_back() at the
 *    end; no insertions, prepends, or mid-array modifications are allowed.
 *    The array's size (index range) grows monotonically over time.
 *
 * 2. Lazy deletion: Elements are not physically removed when erased. Instead,
 *    they are marked as "deleted" using a boolean flag. This avoids costly
 *    memory shifts or deallocations, keeping erase operations O(1).
 *
 * 3. Random access: Supports O(1) element access via operator[] and at(), with
 *    automatic checks for deleted status to ensure validity of returned
 *    elements.
 *
 * 4. Persistent indices: Once an element is added, its index remains fixed
 *    (never reclaimed or reused), making indices safe for long-term
 *    referencing (e.g., in external data structures).
 */
template <typename T, typename Index = int>
class AppendOnlyArray {
    static_assert(std::is_same<Index, int>::value ||
                  std::is_same<Index, int64_t>::value,
                  "Index must be either int or int64_t");

    friend class Iterator;
    friend class ConstIterator;

public:
    using value_type      = T;
    using size_type       = Index;
    using reference       = T&;
    using const_reference = const T&;
    using pointer         = T*;
    using const_pointer   = const T*;

    static constexpr Index MAX_SIZE = Array<T, Index>::MAX_SIZE;

    /**
     * Iterator to skip the deleted elements.
     */
    class Iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = T;
        using difference_type   = std::ptrdiff_t;
        using pointer           = T*;
        using reference         = T&;

        Iterator(AppendOnlyArray* array, Index index)
            : array_(array), index_(index) {
            SkipDeleted();
        }

        reference operator*() const {
            CHECK(IsValid()) << "Dereferencing invalid iterator.";

            return array_->data_[index_];
        }

        pointer operator->() const {
            CHECK(IsValid()) << "Dereferencing invalid iterator.";

            return array_->data_[index_];
        }

        Iterator& operator++() {
            CHECK(index_ < array_->size()) << "Incrementing end iterator.";

            ++index_;
            SkipDeleted();
            return *this;
        }

        Iterator operator++(int) {
            Iterator temp = *this;
            ++(*this);
            return temp;
        }

        bool operator==(const Iterator& other) const {
            return array_ == other.array_ && index_ == other.index_;
        }

        bool operator!=(const Iterator& other) const {
            return !(*this == other);
        }

        Index index() const {
            return index_;
        }

    private:
        /**
         * Skip deleted elements.
         */
        void SkipDeleted() {
            while (index_ < array_->size() && array_->is_deleted_[index_]) {
                ++index_;
            }
        }

        /**
         * Check if the iterator is valid.
         */
        bool IsValid() const {
            return index_ < array_->size() && !array_->is_deleted_[index_];
        }

        AppendOnlyArray* array_;
        Index index_;
    };

    /**
     * ConstIterator to skip the deleted elements.
     */
    class ConstIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = T;
        using difference_type   = std::ptrdiff_t;
        using pointer           = const T*;
        using reference         = const T&;

        ConstIterator(const AppendOnlyArray* array, Index index)
            : array_(array), index_(index) {
            SkipDeleted();
        }

        reference operator*() const {
            CHECK(IsValid()) << "Dereferencing invalid iterator.";

            return array_->data_[index_];
        }

        pointer operator->() const {
            CHECK(IsValid()) << "Dereferencing invalid iterator.";

            return array_->data_[index_];
        }

        ConstIterator& operator++() {
            CHECK(index_ < array_->size()) << "Incrementing end iterator.";

            ++index_;
            SkipDeleted();
            return *this;
        }

        ConstIterator operator++(int) {
            ConstIterator temp = *this;
            ++(*this);
            return temp;
        }

        bool operator==(const ConstIterator& other) const {
            return array_ == other.array_ && index_ == other.index_;
        }

        bool operator!=(const ConstIterator& other) const {
            return !(*this == other);
        }

        Index index() const {
            return index_;
        }

    private:
        /**
         * Skip deleted elements.
         */
        void SkipDeleted() {
            while (index_ < array_->size() && array_->is_deleted_[index_]) {
                ++index_;
            }
        }

        /**
         * Check if the iterator is valid.
         */
        bool IsValid() const {
            return index_ < array_->size() && !array_->is_deleted_[index_];
        }

        const AppendOnlyArray* array_;
        Index index_;
    };

    AppendOnlyArray() = default;

    explicit AppendOnlyArray(const Array<T, Index>& data)
        : data_(data), is_deleted_(data, false) {
        active_size_ = data_.size();
    }

    AppendOnlyArray(const Array<T, Index>& data, Array<bool, Index>& is_deleted)
        : data_(data), is_deleted_(is_deleted), active_size_(0) {
        CHECK(data_.size() == is_deleted_.size());

        for (int i = 0; i < data.size(); ++i) {
            if (!is_deleted[i]) ++active_size_;
        }
    }

    bool empty() const {
        return data_.empty();
    }

    void clear() {
        active_size_ = 0;
        data_.clear();
        is_deleted_.clear();
    }

    /**
     * Remove an element by its index.
     */
    void erase(Index index) {
        if (!is_deleted_[index]) {
            is_deleted_[index] = true;
            --active_size_;
        }
    }

    /**
     * Check if an index is contained in the array.
     */
    bool contain(Index index) const {
        return index >= 0 && index < data_.size() && !is_deleted_[index];
    }

    /**
     * Construct an element in-place at the end of the array.
     */
    template <typename... Args>
    void emplace_back(Args&&... args) {
        CHECK(data_.size() < MAX_SIZE) <<
            "AppendOnlyArray: Reached maximum capacity!";

        data_.emplace_back(std::forward<Args>(args)...);
        is_deleted_.push_back(false);
        ++active_size_;
    }

    /**
     * Add an element to the end of the array.
     */
    void push_back(const T& value) {
        emplace_back(value);
    }

    /**
     * Move version of push_back.
     */
    void push_back(T&& value) {
        emplace_back(std::move(value));
    }

    const T& at(Index index) const {
        CHECK(index >= 0 && index < data_.size() && !is_deleted_[index]) <<
              "AppendOnlyArray: Access to invalid or deleted element!";

        return data_[index];
    }

    T& at(Index index) {
        CHECK(index >= 0 && index < data_.size() && !is_deleted_[index]) <<
              "AppendOnlyArray: Access to invalid or deleted element!";

        return data_[index];
    }

    const T& operator [](Index index) const {
        return this->at(index);
    }

    T& operator [](Index index) {
        return this->at(index);
    }

    Iterator begin()             { return Iterator(this, 0);                 }
    Iterator end()               { return Iterator(this, this->size());      }
    ConstIterator begin()  const { return ConstIterator(this, 0);            }
    ConstIterator end()    const { return ConstIterator(this, this->size()); }
    ConstIterator cbegin() const { return begin();                           }
    ConstIterator cend()   const { return end();                             }

    const Array<T, Index>& data() const {
        return data_;
    }

    /**
     * Return the total elements (active + deleted) in the array.
     */
    Index size() const {
        return data_.size();
    }

    /**
     * Return the number of active (non-deleted) elements.
     */
    Index active_size() const {
        return active_size_;
    }

    const Array<bool, Index>& is_deleted() const {
        return is_deleted_;
    }

private:
    // Underlying storage for all elements (including deleted).
    Array<T, Index> data_;

    // Flags to track deleted status (parallel to elements).
    Array<bool, Index> is_deleted_;

    // Count of active (non-deleted) elements.
    Index active_size_ = 0;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_FLAT_APPEND_ONLY_ARRAY_H_

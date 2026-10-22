//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_ENUMERATE_H_
#define CODELIBRARY_BASE_ENUMERATE_H_

#include <iterator>

#include "codelibrary/base/container_traits.h"

namespace cl {

/**
 * Similar to Python's enumerate(), the following code is adapted from Folly.
 * enumerate() can be used to iterate a range with a for-range loop, and it also
 * allows to retrieve the count of iterations so far. Can be used in constexpr
 * context.
 *
 * For example:
 *
 * for (auto&& [index, element] : enumerate(vec)) {
 *   // index is a const reference to a decltype(vec) or int containing the
 *   // iteration count.
 *   // element is a reference to the type contained within vec, mutable
 *   // unless vec is const.
 * }
 *
 * If the binding is const, the element reference is too.
 *
 * for (const auto&& [index, element] : enumerate(vec)) {
 *   // element is always a const reference.
 * }
 *
 * It can also be used as follows:
 *
 * for (auto&& it : enumerate(vec)) {
 *   // *it is a reference to the current element. Mutable unless vec is const.
 *   // it->member can be used as well.
 *   // it.index contains the iteration count.
 * }
 *
 * As before, const auto&& it can also be used.
 */
template <class Iterator, typename Index = int>
class Enumerator {
    template <class T>
    struct MakeConst {
        using type = const T;
    };
    template <class T>
    struct MakeConst<T&> {
        using type = const T&;
    };
    template <class T>
    struct MakeConst<T*> {
        using type = const T*;
    };

public:
    constexpr explicit Enumerator(Iterator it) : it_(std::move(it)) {}

    class Proxy {
    public:
        using difference_type = ptrdiff_t;
        using value_type = typename std::iterator_traits<Iterator>::value_type;
        using reference = typename std::iterator_traits<Iterator>::reference;
        using pointer = typename std::iterator_traits<Iterator>::pointer;
        using iterator_category = std::input_iterator_tag;

        constexpr explicit Proxy(const Enumerator& e)
            : index(e.index_), element(*e.it_) {}

        // Non-const Proxy: Forward constness from Iterator.
        constexpr reference operator*() {
            return element;
        }
        constexpr pointer operator->() {
            return std::addressof(element);
        }

        // Const Proxy: Force const references.
        constexpr typename MakeConst<reference>::type operator*() const {
            return element;
        }
        constexpr typename MakeConst<pointer>::type operator->() const {
            return std::addressof(element);
        }

        const Index index;
        reference element;
    };

    constexpr Proxy operator*() const { return Proxy(*this); }

    constexpr Enumerator& operator++() {
        ++it_;
        ++index_;
        return *this;
    }

    template <typename OtherIterator, typename OtherIndex>
    constexpr bool operator==(const Enumerator<OtherIterator,
                              OtherIndex>& rhs) const {
        return it_ == rhs.it_;
    }

    template <typename OtherIterator, typename OtherIndex>
    constexpr bool operator!=(const Enumerator<OtherIterator,
                              OtherIndex>& rhs) const {
        return !(it_ == rhs.it_);
    }

private:
    template <typename OtherIterator, typename OtherIndex>
    friend class Enumerator;

    Iterator it_;
    Index index_ = 0;
};

template <class Range, bool HasSize = container_traits::has_size<Range>::value>
class RangeEnumerator;

template <class Range>
class RangeEnumerator<Range, true> {
    using BeginIteratorType = decltype(std::declval<Range>().begin());
    using EndIteratorType = decltype(std::declval<Range>().end());
    using Index = decltype(std::declval<Range>().size());

public:
    constexpr explicit RangeEnumerator(Range&& r)
        : r_(std::forward<Range>(r)) {}

    constexpr Enumerator<BeginIteratorType, Index> begin() {
        return Enumerator<BeginIteratorType, Index>(r_.begin());
    }

    constexpr Enumerator<EndIteratorType, Index> end() {
        return Enumerator<EndIteratorType, Index>(r_.end());
    }

private:
    Range r_;
};

template <class Range>
class RangeEnumerator<Range, false> {
    using BeginIteratorType = decltype(std::declval<Range>().begin());
    using EndIteratorType = decltype(std::declval<Range>().end());
    using Index = size_t;

public:
    constexpr explicit RangeEnumerator(Range&& r)
        : r_(std::forward<Range>(r)) {}

    constexpr Enumerator<BeginIteratorType, Index> begin() {
        return Enumerator<BeginIteratorType, Index>(r_.begin());
    }
    constexpr Enumerator<EndIteratorType, Index> end() {
        return Enumerator<EndIteratorType, Index>(r_.end());
    }

private:
    Range r_;
};

/**
 * Use lowercase here to make it similar to the Python API.
 */
template <class Range>
constexpr RangeEnumerator<Range> enumerate(Range&& r) {
    return RangeEnumerator<Range>(std::forward<Range>(r));
}

} // namespace cl

#endif // CODELIBRARY_BASE_ENUMERATE_H_

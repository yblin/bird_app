//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_LINE_H_
#define CODELIBRARY_IMAGE_IMAGE_LINE_H_

#include "codelibrary/base/log.h"

namespace cl {

/**
 * Represents a line of image data (either a row or a column).
 *
 * It is defined by a 3-tuple [begin, end, stride), where:
 * - begin:  pointer to the start of the line data.
 * - end:    pointer to one past the end of the line data.
 * - stride: the number of bytes to move to access the next element.
 */
template <typename Byte>
struct ImageLine {
    // Define Pixel as a pointer to Byte, allowing users to access specific
    // channels of image data using Pixel[i] syntax. This enables easy
    // manipulation of pixel values in memory.
    using Pixel = Byte*;

    /**
     * Iterator for a row.
     */
    class Iterator {
    public:
        explicit Iterator(Pixel data, int stride)
            : data_(data), stride_(stride) {}

        bool operator == (const Iterator& rhs) const {
            return data_ == rhs.data_;
        }

        bool operator != (const Iterator& rhs) const {
            return data_ != rhs.data_;
        }

        Pixel operator*() const {
            return data_;
        }

        Iterator& operator++() {
            data_ += stride_;
            return *this;
        }

    private:
        Pixel data_;
        int stride_;
    };

    ImageLine() = default;

    ImageLine(Pixel b, Pixel e, int s)
        : first(b), last(e), stride(s) {
        CHECK(stride >= 1);
        CHECK(e >= b);
        CHECK((e - b) % s == 0);
    }

    /**
     * Return the number of elements in the range defined by [first, last).
     *
     * It should ensure that (last - first) is divisible by stride.
     */
    int size() const {
        return (last - first) / stride;
    }

    Iterator begin() const {
        return Iterator(first, stride);
    }

    Iterator end() const {
        return Iterator(last, stride);
    }

    Pixel first = nullptr;
    Pixel last = nullptr;
    int stride = 1;
};

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_LINE_H_

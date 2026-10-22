//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_DATA_H_
#define CODELIBRARY_IMAGE_IMAGE_DATA_H_

#include <functional>
#include <limits>

#include "codelibrary/image/image_line.h"

namespace cl {

/**
 * ImageData represents the fundamental structure of an image.
 *
 * Importantly, the ImageData class does not store any actual image data; it
 * merely maintains a pointer to the existing data.
 * This design enables image.block() and image.channel() convenient without the
 * need for cropping or creating a new image instance.
 */
template <typename T>
class ImageData {
public:
    // Type representing the byte for each channel of a pixel.
    // We define Pixel as a pointer to Byte, allowing users to access specific
    // channels of image data using Pixel[i] syntax. This enables easy
    // manipulation of pixel values in memory.
    using Byte = T;

    using Index = std::conditional<sizeof(size_t) == sizeof(int),
                                   int, int64_t>::type;
    static_assert(sizeof(Index) == sizeof(size_t),
                  "Index type size does not match size_t size.");
    static constexpr Index MAX_SIZE = std::numeric_limits<Index>::max();

    using value_type      = T;
    using pointer         = T*;
    using const_pointer   = const T *;
    using reference       = T&;
    using const_reference = const T &;

    using Row = ImageLine<T>;
    using ConstRow = ImageLine<const T>;
    using Column = ImageLine<T>;
    using ConstColumn = ImageLine<const T>;

    /**
     * Iterator for rows.
     */
    template <typename ImageRow, typename Data>
    class RowList {
        class Iterator {
        public:
            Iterator(Data* data, int row_index)
                : data_(data), row_index_(row_index) {}

            bool operator == (const Iterator& rhs) const {
                return row_index_ == rhs.row_index_;
            }

            bool operator != (const Iterator& rhs) const {
                return row_index_ != rhs.row_index_;
            }

            ImageRow operator*() const {
                return data_->row(row_index_);
            }

            Iterator& operator++() {
                ++row_index_;
                return *this;
            }

        private:
            Data* data_;
            int row_index_;
        };

    public:
        explicit RowList(Data* data)
            : data_(data) {}

        Iterator begin() const {
            return Iterator(data_, 0);
        }

        Iterator end() const {
            return Iterator(data_, data_->height());
        }

    private:
        Data* data_ = nullptr;
    };

    ImageData(Byte* data, int height, int width, int n_channels = 1)
        : height_(height), width_(width), n_channels_(n_channels), data_(data) {
        CHECK(height >= 0);
        CHECK(width >= 0);
        CHECK(n_channels > 0);
        CHECK(data);

        CHECK(width == 0 || height <= MAX_SIZE / width)
                << Format("Image dimension {} * {} is too large.",
                          height, width);
        CHECK(Index(height) * width <= MAX_SIZE / n_channels)
                << Format("Image dimension {} * {} * {} is too large.",
                          height, width, n_channels);
    }

    virtual ~ImageData() = default;

    /**
     * Check if the image is empty.
     */
    bool empty() const {
        return this->size() == 0;
    }

    /**
     * Get the pixel at (h, w).
     * This function must be implemented in the derived class.
     */
    virtual const Byte* pixel(int h, int w) const {
        return data_ + Index(h) * width_ * n_channels_ + Index(w) * n_channels_;
    }
    virtual Byte* pixel(int h, int w) {
        return data_ + Index(h) * width_ * n_channels_ + Index(w) * n_channels_;
    }

    /**
     * Get the pixel at (h, w).
     */
    const Byte* operator()(int h, int w) const {
        return pixel(h, w);
    }
    Byte* operator()(int h, int w) {
        return pixel(h, w);
    }

    /**
     * Get the pixel at (h, w).
     */
    const Byte* at(int h, int w) const {
        CHECK(h >= 0 && h < height());
        CHECK(w >= 0 && w < width());

        return pixel(h, w);
    }
    Byte* at(int h, int w) {
        CHECK(h >= 0 && h < height());
        CHECK(w >= 0 && w < width());

        return pixel(h, w);
    }

    virtual int height() const {
        return height_;
    }

    virtual int width() const {
        return width_;
    }

    virtual int n_channels() const {
        return n_channels_;
    }

    Index n_pixels() const {
        return Index(height()) * width();
    }

    /**
     * Return the number of bytes.
     */
    Index size() const {
        return Index(height()) * width() * n_channels();
    }

    /**
     * Return the data of the i-th row.
     */
    ConstRow row(int i) const {
        CHECK(i >= 0 && i < height());

        return ConstRow(pixel(i, 0), pixel(i, width()), n_channels_);
    }
    Row row(int i) {
        CHECK(i >= 0 && i < height());

        return Row(pixel(i, 0), pixel(i, width()), n_channels_);
    }

    /**
     * Return the data of the i-th column.
     */
    ConstColumn column(int i) const {
        CHECK(i >= 0 && i < width());

        return ConstColumn(pixel(0, i), pixel(height(), i),
                           width_ * n_channels_);
    }
    Column column(int i) {
        CHECK(i >= 0 && i < width());

        return Column(pixel(0, i), pixel(height(), i), width_ * n_channels_);
    }

    /**
     * Return row list of the image.
     */
    RowList<ConstRow, const ImageData> rows() const {
        return RowList<ConstRow, const ImageData>(this);
    }
    RowList<Row, ImageData> rows() {
        return RowList<Row, ImageData>(this);
    }

protected:
    ImageData() = default;

    // Dimension of the image.
    int height_ = 0, width_ = 0, n_channels_ = 0;

    // Pointer to the actual image data.
    T* data_ = nullptr;
};

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_DATA_H_

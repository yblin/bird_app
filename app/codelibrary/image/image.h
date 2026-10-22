//
// Copyright 2019-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_H_
#define CODELIBRARY_IMAGE_IMAGE_H_

#include <cstdint>
#include <cstring>
#include <string>

#include "codelibrary/base/array_nd.h"
#include "codelibrary/image/image_block.h"
#include "codelibrary/image/image_data.h"
#include "codelibrary/util/io/file_util.h"

#include "third_party/stb/stb_image.h"
#include "third_party/stb/stb_image_write.h"
#include "third_party/stb/stb_image_resize.h"

namespace cl {

/**
 * Basic image class.
 */
template <typename T>
class BaseImage : public ImageData<T> {
public:
    using Byte = typename ImageData<T>::Byte;
    using Index = typename ImageData<T>::Index;
    static constexpr Index MAX_SIZE = ImageData<T>::MAX_SIZE;
    using Block = ImageBlock<T>;

    BaseImage() = default;

    /**
     * Construct an image with the specified dimensions (h, w, c).
     */
    BaseImage(int h, int w, int c, Byte byte = Byte()) {
        Reset(h, w, c);
        Fill(byte);
    }

    BaseImage(int h, int w, Byte byte = Byte())
        : BaseImage(h, w, 1, byte) {}

    /**
     * Create an image from the given file.
     */
    explicit BaseImage(const std::string& filename, bool flip = false) {
        Load(filename, flip);
    }

    /**
     * Deep copy the data from block into this image.
     */
    BaseImage(const ImageBlock<T>& block) {
        this->operator=(block);
    }

    /**
     * Deep copy the data from block into this image.
     */
    BaseImage(const BaseImage& image) {
        this->operator=(image);
    }

    /**
     * Deep copy image data into this image.
     */
    BaseImage(const ImageData<T>& data) {
        this->operator=(data);
    }

    virtual ~BaseImage() = default;

    BaseImage& operator= (const ImageData<T>& data) {
        raw_data_.reshape(data.height(), data.width(), data.n_channels());
        this->ResetImageData();
        std::memcpy(raw_data_.data(), data.data_.data(),
                    sizeof(Byte) * data.size());
        return *this;
    }

    BaseImage& operator= (const BaseImage& rhs) {
        raw_data_ = rhs.raw_data_;
        this->ResetImageData();
        return *this;
    }

    BaseImage& operator= (const ImageBlock<T>& block) {
        CHECK(this->data_ != block.data_);

        this->Reset(block.height(), block.width(), block.n_channels());
        Byte* out = this->data();
        int c = block.n_channels();
        size_t pixel_size = sizeof(Byte) * c;
        for (auto row : block.rows()) {
            for (auto p = row.first; p != row.last; p += row.stride) {
                std::memcpy(out, p, pixel_size);
                out += c;
            }
        }
        return *this;
    }

    /**
     * Resets the image dimensions.
     */
    void Reset(int h, int w, int c = 1) {
        CHECK(h >= 0 && w >= 0);
        CHECK(c > 0);
        CHECK(w == 0 || h <= MAX_SIZE / w)
                << Format("Image dimension {} * {} is too large.", h, w);
        CHECK(Index(h) * w <= MAX_SIZE / c)
                << Format("Image dimension {} * {} * {} is too large.",
                          h, w, c);

        raw_data_.reshape(h, w, c);
        ResetImageData();
    }

    /**
     * Set the data for image.
     */
    void SetData(const Array<T, Index>& data) {
        CHECK(data.size() == this->size());

        std::memcpy(raw_data_.data(), data.data(), sizeof(Byte) * this->size());
    }

    /**
     * Set the data for image.
     */
    void SetData(const Array<T>& data) {
        CHECK(data.size() == this->size());

        std::memcpy(raw_data_.data(), data.data(), sizeof(Byte) * this->size());
    }

    /**
     * Loads an image from the specified file.
     */
    virtual bool Load(const std::string& filename, bool flip = false) {
        stbi_set_flip_vertically_on_load(flip);
        int h, w, c;

        Byte* data = STBILoad(filename.c_str(), &h, &w, &c);
        if (!data) {
            this->clear();
            LOG(ERROR) << "Error when loading the image: '" << filename << "'"
                       << "Error message: " << stbi_failure_reason();
            return false;
        }

        raw_data_.reshape(h, w, c);
        std::memcpy(raw_data_.data(), data, sizeof(Byte) * raw_data_.size());
        stbi_image_free(data);
        ResetImageData();
        return true;
    }

    /**
     * Loads an image from memory buffer.
     */
    virtual bool Load(const uint8_t* buffer, int len, bool flip = false) {
        CHECK(len >= 0);

        stbi_set_flip_vertically_on_load(flip);
        int h, w, c;

        Byte* data = STBILoadFromMemory(buffer, len, &h, &w, &c);
        if (!data) {
            this->clear();
            LOG(ERROR) << "Error when loading the image from memory buffer."
                       << "Error message: " << stbi_failure_reason();
            return false;
        }

        raw_data_.reshape(h, w, c);
        std::memcpy(raw_data_.data(), data, sizeof(Byte) * raw_data_.size());
        stbi_image_free(data);
        ResetImageData();
        return true;
    }

    /**
     * Save this image to the file.
     */
    virtual bool Save(const std::string& filename) const;

    /**
     * Resize the image. Unlike Reset(), this function scales the contents of
     * the image.
     */
    virtual void Resize(int h, int w) {
        CHECK(h >= 0 && w >= 0);

        if (this->empty()) return;
        if (h == 0 || w == 0) {
            this->clear();
            return;
        }
        if (this->height_ == h && this->width_ == w) return;

        CHECK(h <= MAX_SIZE / w)
                << Format("Image dimension {} * {} for Resize() is too large.",
                          h, w);

        int c = this->n_channels_;
        CHECK(h * w <= MAX_SIZE / c)
                << Format("Image dimension {} * {} * {} for Resize() is too "
                          "large.", h, w, c);

        ArrayND<Byte> data(h, w, c);
        STBIRResize(h, w, data.data());
        raw_data_.swap(&data);
        ResetImageData();
    }

    /**
     * Fill the image with the given value.
     */
    void Fill(Byte value) {
        if (value == 0) {
            // When value = 0, memset() is faster than fill().
            std::memset(this->data(), 0, raw_data_.size() * sizeof(Byte));
        } else {
            raw_data_.fill(value);
        }
    }

    /**
     * Flip the image vertically.
     */
    void VerticalFlip() {
        ArrayND<Byte> row(this->width_, this->n_channels_);
        size_t n = sizeof(Byte) * row.size();

        int h = this->height_;
        for (int i = 0; i < h / 2; ++i) {
            Byte* row1 = this->pixel(i, 0);
            Byte* row2 = this->pixel(h - 1 - i, 0);

            std::memcpy(row.data(), row1, n);
            std::memcpy(row1, row2, n);
            std::memcpy(row2, row.data(), n);
        }
    }

    /**
     * Return a block of the image.
     *
     * Note: The returned ImageBlock refers to the original image data.
     *       Modifications made to the pixel values through the returned
     *       ImageBlock will directly affect the original image.
     *
     * Additionally, be aware that if the dimensions of the original image
     * change, any previously generated ImageBlocks may become invalid, leading
     * to undefined behavior if accessed afterward.
     */
    Block block(int start_h, int start_w, int block_h, int block_w) {
        return Block(raw_data_.data(), this->height(), this->width(),
                     this->n_channels(), start_h, start_w,
                     block_h, block_w);
    }

    /**
     * Returns the specified channel of the image as an ImageBlock object.
     *
     * Note: The returned ImageBlock is a view of the original image data,
     *       meaning that any modifications made to the pixel values through
     *       this channel block will directly affect the corresponding channel
     *       in the original image.
     *
     * Additionally, be aware that if the dimensions of the original image
     * change, any previously generated ImageBlocks may become invalid, leading
     * to undefined behavior if accessed afterward.
     */
    Block channel(int i) {
        CHECK(i >= 0 && i < this->n_channels_);

        return Block(raw_data_.data(), this->height_, this->width_,
                     this->n_channels_, 0, 0,
                     this->height_, this->width_, i);
    }

    /**
     * Extracts a fixed-size crop from the image at the specified location.
     * Pixels that lie outside the source image are NOT COPIED and retain their
     * initial value in crop_image.
     *
     * Parameters:
     *  start_h    - Starting row in the source image (can be negative).
     *  start_w    - Starting column in the source image (can be negative).
     *  h          - Height of the output crop.
     *  w          - Width of the output crop.
     *  crop_image - Output cropped image (will be resized).
     */
    void GetCrop(int start_h, int start_w, int h, int w,
                 BaseImage* crop_image) const {
        CHECK(h >= 0);
        CHECK(w >= 0);
        CHECK(crop_image);

        crop_image->Reset(h, w, this->n_channels_);
        size_t pixel_size = sizeof(Byte) * this->n_channels_;
        Byte* out = crop_image->data();

        int img_h = this->height();
        int img_w = this->width();
        int n_channels = this->n_channels();

        for (int y = 0; y < h; ++y) {
            int src_y = start_h + y;

            if (src_y < 0 || src_y >= img_h) {
                out += w * n_channels;
                continue;
            }

            for (int x = 0; x < w; ++x) {
                int src_x = start_w + x;

                if (src_x < 0 || src_x >= img_w) {
                    out += n_channels;
                    continue;
                }

                const Byte* src_pixel = this->pixel(src_y, src_x);
                std::memcpy(out, src_pixel, pixel_size);
                out += n_channels;
            }
        }
    }

    /**
     * Clear the image. Reset it to the empty image.
     */
    void clear() {
        raw_data_.clear();
        this->data_ = nullptr;
        this->height_ = 0;
        this->width_ = 0;
        this->n_channels_ = 0;
    }

    Byte* data()             { return raw_data_.data(); }
    const Byte* data() const { return raw_data_.data(); }

protected:
    /**
     * Load image by STBI.
     */
    Byte* STBILoad(const char* filename, int* h, int* w, int* c);

    /**
     * Load image from memory buffer by STBI.
     */
    Byte* STBILoadFromMemory(const uint8_t* data, int len, int* h, int* w,
                             int* c);

    /**
     * Resize image by STBIR.
     */
    void STBIRResize(int h, int w, Byte* dst);

    /**
     * Reset image data from raw_data_.
     */
    void ResetImageData() {
        this->data_ = raw_data_.data();
        this->height_ = raw_data_.shape(0);
        this->width_ = raw_data_.shape(1);
        this->n_channels_ = raw_data_.shape(2);
    }

    ArrayND<Byte> raw_data_; // Image data.
};

template <>
inline uint8_t* BaseImage<uint8_t>::STBILoad(const char* filename, int* h,
                                             int* w, int* c) {
    return stbi_load(filename, w, h, c, 0);
}

template <>
inline float* BaseImage<float>::STBILoad(const char* filename, int* h, int* w,
                                         int* c) {
    return stbi_loadf(filename, w, h, c, 0);
}

template <>
inline uint8_t* BaseImage<uint8_t>::STBILoadFromMemory(const uint8_t* buffer,
                                                       int len,
                                                       int* h,
                                                       int* w,
                                                       int* c) {
    return stbi_load_from_memory(buffer, len, w, h, c, 0);
}

template <>
inline float* BaseImage<float>::STBILoadFromMemory(const uint8_t* buffer,
                                                   int len,
                                                   int* h,
                                                   int* w,
                                                   int* c) {
    return stbi_loadf_from_memory(buffer, len, w, h, c, 0);
}

template <>
inline bool BaseImage<uint8_t>::Save(const std::string& filename) const {
    if (empty()) {
        LOG(WARNING) << "Warning, you are saving an empty image.";
        return true;
    }

    int w = this->width();
    int h = this->height();
    int c = this->n_channels();

    const std::string suffix = file_util::GetSuffix(filename);
    int success = 0;
    if (suffix == "jpg" || suffix == "jpeg") {
        success = stbi_write_jpg(filename.c_str(), w, h, c, this->data(), 100);
    } else if (suffix == "tga") {
        success = stbi_write_tga(filename.c_str(), w, h, c, this->data());
    } else if (suffix == "png") {
        success = stbi_write_png(filename.c_str(), w, h, c, this->data(), 0);
    } else if (suffix == "bmp") {
        success = stbi_write_bmp(filename.c_str(), w, h, c, this->data());
    } else {
        LOG(INFO) << "Unsupported file suffix: " << suffix;
        return false;
    }

    if (!success) {
        LOG(INFO) << stbi_failure_reason();
    }

    return success;
}

template <>
inline bool BaseImage<float>::Save(const std::string& filename) const {
    if (empty()) {
        LOG(WARNING) << "Warning, you are saving an empty image.";
        return true;
    }

    const std::string suffix = file_util::GetSuffix(filename);
    int success = 0;
    if (suffix == "hdr") {
        success = stbi_write_hdr(filename.c_str(),
                                 this->width(),
                                 this->height(),
                                 this->n_channels(),
                                 this->data());
    } else {
        LOG(INFO) << "Unsupported file suffix: " << suffix;
        return false;
    }

    if (!success) {
        LOG(INFO) << stbi_failure_reason();
    }

    return success;
}

template <>
inline void BaseImage<uint8_t>::STBIRResize(int h, int w, uint8_t* dst) {
    stbir_resize_uint8(this->data(), this->width(), this->height(), 0, dst,
                       w, h, 0, this->n_channels());
}

template <>
inline void BaseImage<float>::STBIRResize(int h, int w, float* dst) {
    stbir_resize_float(this->data(), this->width(), this->height(), 0, dst,
                       w, h, 0, this->n_channels());
}

using Image  = BaseImage<uint8_t>;
using ImageF = BaseImage<float>;

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_H_

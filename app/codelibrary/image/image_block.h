//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_BLOCK_H_
#define CODELIBRARY_IMAGE_IMAGE_BLOCK_H_

#include "codelibrary/image/image_data.h"

namespace cl {

/**
 * Represent a block of an image, allowing for operations on specific
 * subregions of image data. This class facilitates the manipulation
 * of designated areas within an image.
 */
template <typename T>
class ImageBlock : public ImageData<T> {
    template <typename T1>
    friend class BaseImage;

public:
    using Byte = typename ImageData<T>::Byte;

    /**
     * Constructs an ImageBlock object, representing a subregion of an image.
     *
     * Parameters:
     *  data       - pointer to the parent image data.
     *  height     - the height of the parent image.
     *  width      - the width of the parent image.
     *  n_channels - the number of channels of the parent image.
     *  start_h    - the starting vertical position (row index) of the block
     *               within the parent image.
     *  start_w    - the starting horizontal position (column index) of the
     *               block within the parent image.
     *  block_h    - the height of the block being defined. Specifies how many
     *               rows the block spans starting from start_h.
     *  block_w    - the width of the block being defined. Specifies how many
     *               columns the block spans starting from start_w.
     *  channel    - optional parameter specifying the channel to be accessed.
     *               If set to -1, all channels are included; otherwise, it
     *               restricts access to the specified channel.
     */
    ImageBlock(T* data, int height, int width, int n_channels,
               int start_h, int start_w, int block_h, int block_w,
               int channel = -1)
        : ImageData<T>(data, height, width, n_channels),
          start_h_(start_h), start_w_(start_w),
          block_h_(block_h), block_w_(block_w),
          channel_(channel) {
        CHECK(start_h_ >= 0);
        CHECK(start_w_ >= 0);
        CHECK(block_h_ >= 0);
        CHECK(block_w_ >= 0);
        CHECK(start_h_ + block_h_ <= height)
                << Format("The height of the block ({}) starting at row {}"
                          " exceeds the image height ({}).",
                          block_h_, start_h_, height);
        CHECK(start_w_ + block_w_ <= width)
                << Format("The width of the block ({}) starting at column"
                          " {} exceeds the image width ({}).",
                          block_w_, start_w_, width);
        CHECK(channel >= -1 && channel < n_channels);
    }

    virtual ~ImageBlock() = default;

    /**
     * Get the pixel at (h, w).
     */
    virtual const Byte* pixel(int h, int w) const override {
        return ImageData<T>::pixel(start_h_ + h, start_w_ + w) +
               (channel_ == -1 ? 0 : channel_);
    }
    virtual Byte* pixel(int h, int w) override {
        return ImageData<T>::pixel(start_h_ + h, start_w_ + w) +
               (channel_ == -1 ? 0 : channel_);
    }

    virtual int height() const override {
        return block_h_;
    }

    virtual int width() const override {
        return block_w_;
    }

    virtual int n_channels() const override {
        return channel_ == -1 ? ImageData<T>::n_channels() : 1;
    }

    /**
     * Return a subblock of this image block.
     */
    ImageBlock block(int start_h, int start_w, int block_h, int block_w) const {
        CHECK(start_h >= 0);
        CHECK(start_w >= 0);
        CHECK(start_h + block_h <= block_h_)
                << Format("The height of the block ({}) starting at row {}"
                          " exceeds the ImageBlock's height ({}).",
                          block_h, start_h, block_h_);
        CHECK(start_w + block_w <= block_w_)
                << Format("The width of the block ({}) starting at column"
                          " {} exceeds the ImageBlock's width ({}).",
                          block_w, start_w, block_w_);
        ImageBlock b(*this);
        b.start_h_ += start_h;
        b.start_w_ += start_w;
        b.block_h_ = block_h;
        b.block_w_ = block_w;
        return b;
    }

    /**
     * Return a specific channel image of this image block.
     */
    ImageBlock channel(int channel) const {
        CHECK(channel_ == -1);
        CHECK(channel >= 0 && channel < this->n_channels());

        ImageBlock b(*this);
        b.channel_ = channel;
        return b;
    }

protected:
    int start_h_, start_w_, block_h_, block_w_;
    int channel_ = -1;
};

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_BLOCK_H_

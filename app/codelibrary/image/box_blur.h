//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_BOX_BLUR_H_
#define CODELIBRARY_IMAGE_BOX_BLUR_H_

#include "codelibrary/base/clamp.h"
#include "codelibrary/image/image.h"

namespace cl {
namespace image {

/**
 * A box filter is a spatial domain linear filter in which each pixel in the
 * resulting image has a value equal to the average value of its neighboring
 * pixels in the input image.
 *
 * It is a form of low-pass ("blurring") filter. A 3 by 3 box blur ("radius 1")
 * can be written as matrix:
 *     [ 1 1 1 ]
 * 1/9 [ 1 1 1 ]
 *     [ 1 1 1 ]
 *
 * Due to its property of using equal weights, it can be implemented using a
 * much simpler accumulation algorithm, which is significantly faster than using
 * a sliding-window algorithm.
 *
 * Box filter are frequently used to approximate a Gaussian filter. By the
 * central limit theorem, repeated application of a box blur will approximate a
 * Gaussian blur.
 *
 * The following implement use a fast accumulator algorithm. The time cost is
 * O(N + k) not O(kN).
 */
template <typename T>
void BoxBlur(int kernel_radius, ImageData<T>* image) {
    CHECK(kernel_radius >= 1);
    CHECK(kernel_radius < (INT_MAX - 1) / 2)
            << "Kernel radius: " << kernel_radius << " is too large.";
    CHECK(image);

    if (image->empty()) return;

    const int w = image->width(), h = image->height(), c = image->n_channels();
    const int kernel_size = 2 * kernel_radius + 1;
    const int offset = kernel_radius + 1;
    const double norm = 1.0 / kernel_size;

    bool is_integer = std::is_integral<T>::value;

    Array<double> accumulator(std::max(h, w) + kernel_size, 0.0);
    for (int k = 0; k < c; ++k) {
        for (int i = 0; i < h; ++i) {
            for (int j = 1; j < w + kernel_size; ++j) {
                int x = Clamp(j - offset, 0, w - 1);
                accumulator[j] = accumulator[j - 1] + image->pixel(i, x)[k];
            }

            for (int j = 0; j < w; ++j) {
                double t = (accumulator[offset + j + kernel_radius] -
                            accumulator[j]) * norm;
                image->pixel(i, j)[k] =
                        is_integer ? static_cast<T>(t >= 0.0 ? t + 0.5
                                                             : t - 0.5)
                                   : static_cast<T>(t);
            }
        }
    }
    for (int k = 0; k < c; ++k) {
        for (int j = 0; j < w; ++j) {
            for (int i = 1; i < h + kernel_size; ++i) {
                int x = Clamp(i - offset, 0, h - 1);
                accumulator[i] = accumulator[i - 1] + image->pixel(x, j)[k];
            }

            for (int i = 0; i < h; ++i) {
                double t = (accumulator[offset + i + kernel_radius] -
                            accumulator[i]) * norm;
                image->pixel(i, j)[k] =
                        is_integer ? static_cast<T>(t >= 0.0 ? t + 0.5
                                                             : t - 0.5)
                                   : static_cast<T>(t);
            }
        }
    }
}

} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_BOX_BLUR_H_

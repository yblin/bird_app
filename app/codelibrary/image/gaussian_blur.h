//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_GAUSSIAN_BLUR_H_
#define CODELIBRARY_IMAGE_GAUSSIAN_BLUR_H_

#include "codelibrary/image/box_blur.h"
#include "codelibrary/image/image.h"

namespace cl {
namespace image {

/**
 * Fast approximate Gaussian blur implementation for images (linear time
 * complexity).
 *
 * This algorithm approximates a true Gaussian blur by applying multiple
 * sequential box blur passes over the image. Per the Central Limit Theorem,
 * repeated application of a box blur (a simple uniform filter) converges to the
 * effect of a true Gaussian blur.
 *
 * Three box blur passes are sufficient to achieve high-quality approximations
 * of Gaussian blur.
 *
 * Small sigma values (<=2) will result in noticeable quality degradation
 * compared to exact separable Gaussian blur.
 *
 * Reference:
 *   Kovesi P. Fast almost-gaussian filtering[C]. 2010 International conference
 *   on Digital image computing: Techniques and applications.
 *   IEEE, 2010: 121-125.
 */
template <typename T>
void FastGaussianBlur(double sigma, ImageData<T>* image) {
    CHECK(sigma > 0);
    CHECK(image);

    if (image->empty()) return;

    // Converts the standard deviation of Gaussian blur into a box radius for
    // each box blur pass.
    const int n_passes = 3;
    double wi = std::sqrt((12.0 * sigma * sigma / n_passes) + 1);
    int wl = wi;
    if (wl % 2 == 0) --wl;
    int wu = wl + 2;

    double mi = (12.0 * sigma * sigma - n_passes * wl * wl -
                 4.0 * n_passes * wl - 3.0 * n_passes) / (-4.0 * wl - 4.0);
    int m = static_cast<int>(mi + 0.5);

    for (int i = 0; i < n_passes; ++i) {
        int kernel_radius = ((i < m ? wl : wu) - 1) / 2;
        if (kernel_radius == 0) continue;
        BoxBlur(kernel_radius, image);
    }
}

} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_GAUSSIAN_BLUR_H_

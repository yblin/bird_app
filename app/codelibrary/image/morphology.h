//
// Copyright 2020-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_MORPHOLOGY_H_
#define CODELIBRARY_IMAGE_MORPHOLOGY_H_

#include <cmath>

#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/image/image.h"
#include "codelibrary/image/distance_transform.h"

namespace cl {
namespace image {
namespace morphology {

/**
 * In morphology, we only care about the coordinate of pixels.
 */
using Pixel = IPoint2D;

/**
 * In morphology, a structuring element is a shape, used to probe or interact
 * with a given image, with the purpose of drawing conclusions on how this shape
 * fits or misses the shapes in the image.
 *
 * It is typically used in morphological operations, such as dilation, erosion,
 * opening, and closing, as well as the hit-or-miss transform.
 */
using StructuringElement = Array<Pixel>;

/**
 * Generate a disk structuring element.
 */
struct Disk : public StructuringElement {
    explicit Disk(double r) {
        CHECK(r >= 0.0 && r <= INT_MAX);

        double inner_r = r / std::sqrt(2.0);
        double r2 = r * r;
        int radius = static_cast<int>(std::ceil(r));
        for (int x = -radius; x <= radius; ++x) {
            for (int y = -radius; y <= radius; ++y) {
                if ((std::abs(x) <= inner_r && std::abs(y) <= inner_r) ||
                    static_cast<double>(x) * x +
                    static_cast<double>(y) * y <= r2) {
                    this->emplace_back(x, y);
                }
            }
        }
    }
};

/**
 * Generate a square structuring element.
 *
 * Every pixel along the perimeter has a chessboard distance no greater than
 * radius (radius=floor(w/2)) pixels.
 */
struct Square : public StructuringElement {
    explicit Square(int w) {
        CHECK(w > 0);

        int l = -(w - 1) / 2;
        int r = w / 2;
        for (int x = l; x <= r; ++x)
            for (int y = l; y <= r; ++y)
                this->emplace_back(x, y);
    }
};

/**
 * Generate a diamond-shaped structuring element.
 *
 * A pixel is part of the neighborhood if the city block/Manhattan distance
 * between it and the center of the neighborhood is no greater than radius.
 */
struct Diamond : public StructuringElement {
    explicit Diamond(int r) {
        CHECK(r > 0);

        for (int x = -r; x <= r; ++x) {
            for (int y = -r; y <= r; ++y) {
                if (std::abs(x) + std::abs(y) <= r) {
                    this->emplace_back(x, y);
                }
            }
        }
    }
};

/**
 * Get the boundary pixels of an binary image.
 */
template <typename Byte, typename StructuringElement>
void Boundary(const ImageData<Byte>& image, const StructuringElement& neighbor,
              Array<Pixel>* boundary) {
    CHECK(image.n_channels() == 1);
    CHECK(boundary);

    boundary->clear();

    int w = image.width(), h = image.height();
    for (int y = 0; y < h; ++y) {
        auto row = image.row(y);
        int x = 0;
        for (auto p : row) {
            if (*p != 0) {
                for (const Pixel& q : neighbor) {
                    if (q.x == 0 && q.y == 0) continue;
                    int x1 = x + q.x, y1 = y + q.y;
                    if (x1 < 0 || x1 >= w || y1 < 0 || y1 >= h ||
                        *image.pixel(y1, x1) == 0) {
                        boundary->emplace_back(x, y);
                    }
                }
            }
            ++x;
        }
    }
}

/**
 * Similar to the previous one but using Square(3) as the structuring element.
 */
template <typename Byte>
void Boundary(const ImageData<Byte>& image, Array<Pixel>* boundary) {
    CHECK(image.n_channels() == 1);
    CHECK(boundary);

    boundary->clear();

    int w = image.width(), h = image.height();
    int x = 0;
    for (auto p : image.row(0)) {
        if (*p != 0) boundary->emplace_back(x, 0);
        ++x;
    }
    x = 0;
    for (auto p : image.row(h - 1)) {
        if (*p != 0) boundary->emplace_back(x, h - 1);
        ++x;
    }

    for (int y = 1; y + 1 < h; ++y) {
        auto row = image.row(y);
        auto p0 = image.row(y - 1).first;
        auto p2 = image.row(y + 1).first;
        int x = 0;
        for (auto p = row.first; p != row.last; p += row.stride) {
            if (*p != 0) {
                if (x == 0 || x + 1 == w ||
                    *(p - row.stride) == 0 || *(p + row.stride) == 0 ||
                    *(p0 - row.stride) == 0 || *(p0 + row.stride) == 0 ||
                    *(p2 - row.stride) == 0 || *(p2 + row.stride) == 0 ||
                    *p2 == 0 || *p0 == 0) {
                    boundary->emplace_back(x, y);
                }
            }
            p0 += row.stride;
            p2 += row.stride;
            ++x;
        }
    }
}

/**
 * Compute morphological dilation of an image using square kernel.
 *
 * Morphological dilation sets a pixel at (x,y) to the maximum over all pixels
 * in the neighborhood centered at (x,y):
 *
 *   dst(x, y) =        max          src(x + x', y + y')
 *              (x',y') \in N(x, y)
 *
 * Here, we assume that the given image is a binary image, the pixel value is
 * either 0 or 255.
 */
inline void BinaryDilate(const Image& image, int radius, Image* result) {
    CHECK(image.n_channels() == 1);
    CHECK(radius >= 0);
    CHECK(result);

    *result = image;
    if (radius == 0) return;

    ManhattanDistanceTransform dt(image, 255);
    const Array<double>& dis = dt.distance_map();
    Image::Byte* data = result->data();
    for (int i = 0; i < dis.size(); ++i) {
        data[i] = (dis[i] <= radius) ? 255 : 0;
    }
}

/**
 * Compute morphological erosion of an image using square kernel.
 *
 * Morphological dilation sets a pixel at (x,y) to the maximum over all pixels
 * in the neighborhood centered at (x,y):
 *
 *   dst(x, y) =        max          src(x + x', y + y')
 *              (x',y') \in N(x, y)
 *
 * Here, we assume that the given image is a binary image, the pixel value is
 * either 0 or 255.
 */
inline void BinaryErode(const Image& image, int radius, Image* result) {
    CHECK(image.n_channels() == 1);
    CHECK(radius >= 0);
    CHECK(result);

    *result = image;
    if (radius == 0) return;

    ManhattanDistanceTransform dt(image, 0);
    const Array<double>& dis = dt.distance_map();
    Image::Byte* data = result->data();
    for (int i = 0; i < dis.size(); ++i) {
        data[i] = (dis[i] <= radius) ? 0 : 255;
    }
}

/**
 * Identify and label connected components in a single-channel uint8_t image
 * using flood fill.
 *
 * Example:
 *   ConnectedComponent cc;
 *   int n = cc.GetLabelImage(image, neighbors, &labels);
 */
class ConnectedComponent {
public:
    ConnectedComponent() = default;

    /**
     * Assigns a unique integer label to each connected component in the image.
     *
     * Two pixels belong to the same component if they are neighbors (as defined
     * by the structuring element) and have equal intensity values. Labels are
     * assigned in raster scan order starting from 0.
     *
     * Parameters:
     *  image     - single-channel image.
     *  neighbors - structuring element defining pixel neighborhood.
     *  labels    - output label image; reshaped to [height x width].
     *
     * Return the number of connected components found.
     */
    int GetLabelImage(const Image& image, const StructuringElement& neighbors,
                      ArrayND<int>* labels) const {
        CHECK(image.n_channels() == 1);
        CHECK(labels);

        const int w = image.width(), h = image.height();
        labels->reshape(h, w);

        visited_.reshape(h, w);
        visited_.fill(false);

        // Pixels belonging to the component found by the most recent Flood()
        // call.
        Array<Pixel> pixels;

        int n_labels = 0;
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                if (visited_(i, j)) continue;
                Flood(image, Pixel(j, i), neighbors, &pixels);
                for (const Pixel& p : pixels)
                    (*labels)(p.y, p.x) = n_labels;
                ++n_labels;
            }
        }
        return n_labels;
    }

private:
    /**
     * Flood from seed outward, collecting all pixels in the same connected
     * component into pixels. A neighbor is included if its intensity exactly
     * matches that of the pixel it is reached from.
     *
     * On return, all collected pixels are marked true in visited.
     */
    void Flood(const Image& image,
               const Pixel& seed,
               const StructuringElement& neighbors,
               Array<Pixel>* pixels) const {
        const int w = image.width(), h = image.height();

        visited_(seed.y, seed.x) = true;
        pixels->clear();
        pixels->push_back(seed);

        int front = 0;
        while (front < pixels->size()) {
            const Pixel p = (*pixels)[front++];
            const int v = image.pixel(p.y, p.x)[0];

            for (const Pixel& q : neighbors) {
                const int x = p.x + q.x;
                const int y = p.y + q.y;
                if (x < 0 || x >= w || y < 0 || y >= h) continue;
                if (visited_(y, x)) continue;
                if (image.pixel(y, x)[0] != v) continue;
                visited_(y, x) = true;
                pixels->emplace_back(x, y);
            }
        }
    }

    // Track which pixels have been assigned to a component. Persists across
    // Flood() calls within a single GetLabelImage() invocation so each pixel is
    // enqueued at most once, giving O(h*w) total work.
    mutable ArrayND<bool> visited_;
};

} // namespace morphology
} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_MORPHOLOGY_H_

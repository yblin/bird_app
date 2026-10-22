//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_DRAW_H_
#define CODELIBRARY_IMAGE_IMAGE_DRAW_H_

#include <cmath>

#include "codelibrary/geometry/util/rasterizer.h"
#include "codelibrary/image/image.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace image {

/**
 * Draw a color line on an image using raw Bresenham's line algorithm.
 *
 * Assume that All input coordinates (x0, y0) and (x1, y1) are integer and
 * within the valid bounds of the image.
 */
inline void DrawLine(int x0, int y0, int x1, int y1,
                     const RGB32Color& color, Image* image) {
    CHECK(image);
    CHECK(x0 >= 0 && x0 < image->width());
    CHECK(y0 >= 0 && y0 < image->height());
    CHECK(x1 >= 0 && x1 < image->width());
    CHECK(y1 >= 0 && y1 < image->height());
    CHECK(image->n_channels() == 3 || image->n_channels() == 4);

    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (true) {
        image->pixel(y0, x0)[0] = color.red();
        image->pixel(y0, x0)[1] = color.green();
        image->pixel(y0, x0)[2] = color.blue();
        int e2 = 2 * error;
        if (e2 >= dy) {
            if (x0 == x1) break;
            error = error + dy;
            x0 = x0 + sx;
        }
        if (e2 <= dx) {
            if (y0 == y1) break;
            error = error + dx;
            y0 = y0 + sy;
        }
    }
}

/**
 * Draw a grey polygon on an image.
 */
template <typename T>
void DrawPolygon(const MultiPolygon2D<T>& polygon, Image::Byte color,
                 ImageData<uint8_t>* image) {
    CHECK(image);
    CHECK(image->n_channels() == 1);

    geometry::Rasterizer rasterizer(image->height(), image->width());
    Array<geometry::Rasterizer::ScanLine> scan_lines;
    rasterizer.Rasterize(polygon, &scan_lines);

    for (auto& scan_line : scan_lines) {
        for (int x = scan_line.left; x <= scan_line.right; ++x) {
            image->pixel(scan_line.y, x)[0] = color;
        }
    }
}
template <typename T>
void DrawPolygon(const Polygon2D<T>& polygon, Image::Byte color,
                 ImageData<uint8_t>* image) {
    MultiPolygon2D poly(polygon);
    DrawPolygon(poly, color, image);
}

} // namespace image
} // namespace

#endif // CODELIBRARY_IMAGE_IMAGE_DRAW_H_

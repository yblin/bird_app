//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_CONTOUR_H_
#define CODELIBRARY_IMAGE_IMAGE_CONTOUR_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/image/image_data.h"

namespace cl {

/**
 * Image contour for binary images.
 *
 * Reference:
 *  Suzuki, Satoshi. "Topological structural analysis of digitized binary images
 *  by border following." Computer vision, graphics, and image processing 30.1
 *  (1985): 32-46.
 */
class ImageContour {
    using Pixel = IPoint2D;

public:
    struct Border {
        int id;              // ID of this border.
        int parent_id;       // ID of the parent border.
        bool is_outer;       // Outer border or hole border.
        Array<Pixel> points; // Vertices of the border.
    };

    template <typename T>
    explicit ImageContour(const ImageData<T>& image)
        : label_(image.height() + 2, image.width() + 2) {
        CHECK(image.n_channels() == 1);

        height_ = label_.shape(0);
        width_ = label_.shape(1);
        label_.fill(0);

        int i = 1;
        for (auto row : image.rows()) {
            int j = 1;
            for (auto p : row) {
                label_(i, j++) = *p == 0 ? 0 : 1;
            }
            ++i;
        }

        Border root;
        root.id = 0;
        root.parent_id = 0;
        root.is_outer = false;
        borders_.push_back(root);

        Border border;
        border.id = 1;
        border.parent_id = 0;
        border.is_outer = false;
        borders_.push_back(border);
    }

    /**
     * Get all borders and organized them into a full hierarchy of tree.
     */
    void GetBorderTree(Array<Border>* borders) {
        CHECK(borders);

        Scan();
        *borders = borders_;
        for (auto& b : *borders) {
            for (auto& p : b.points) {
                --p.x;
                --p.y;
            }
        }
    }

private:
    /**
     * Step around a pixel clockwise.
     */
    Pixel StepClockwise(const Pixel& p, const Pixel& start_neighbor) {
        if (start_neighbor.x > p.x) return {p.x, p.y + 1};
        if (start_neighbor.x < p.x) return {p.x, p.y - 1};
        if (start_neighbor.y > p.y) return {p.x - 1, p.y};
        return {p.x + 1, p.y};
    }

    /**
     * Step around a pixel counter-clockwise.
     */
    Pixel StepCounterClockwise(const Pixel& p, const Pixel& start_neighbor) {
        if (start_neighbor.x < p.x) return {p.x, p.y + 1};
        if (start_neighbor.x > p.x) return {p.x, p.y - 1};
        if (start_neighbor.y < p.y) return {p.x - 1, p.y};
        return {p.x + 1, p.y};
    }

    /**
     * Return true if the pixel is a 0-pixel.
     */
    bool IsZeroPixel(const Pixel& p) {
        return p.x < 0 || p.x >= width_ || p.y < 0 || p.y >= height_ ||
               label_(p.y, p.x) == 0;
    }

    /**
     * Given the last border and the newly found border, determine the
     * hierarchical relationship between them.
     */
    void DetermineBorderHierarchy(const Border& last_border,
                                  Border* current_border) {
        CHECK(current_border);

        if (current_border->is_outer == last_border.is_outer) {
            current_border->parent_id = last_border.parent_id;
        } else {
            current_border->parent_id = last_border.id;
        }
    }

    /**
     * Border trace.
     */
    void TraceBorder(const Pixel& start, const Pixel& start_neighbor,
                     Border* border) {
        int w = width_;

        // Step 3.1
        // Starting from (i2, j2), look around clockwise the pixels in the
        // neighborhood of (i, j) and find a nonzero pixel.
        // Let (i1, j1) be the first found nonzero pixel. If no nonzero pixel is
        // found, assign -NBD to f_ij and go to Step 4.
        Pixel next_neighbor = start_neighbor;
        do {
            next_neighbor = StepClockwise(start, next_neighbor);
            if (next_neighbor == start_neighbor) {
                label_(start.y, start.x) = -border->id;
                border->points.push_back(start);
                return;
            }
        } while (IsZeroPixel(next_neighbor));
        Pixel p1 = next_neighbor;

        // Step 3.2
        // (i2, j2) <- (i1, j1) and (i3, j3) <- (i, j).
        Pixel p2 = p1;
        Pixel p3 = start;

        // Step 3.3
        // Starting from the next element of the pixel(i2, j2) in the
        // counterclockwise order, examine counterclockwise the pixels in the
        // neighborhood of the current pixel(i3, j3) to find a nonzero pixel and
        // let the first one be(i4, j4).
        while (true) {
            Pixel next_neighbor = p2;

            bool examined = false;

            do {
                if (next_neighbor.x > p3.x) examined = true;
                next_neighbor = StepCounterClockwise(p3, next_neighbor);
            } while (IsZeroPixel(next_neighbor));
            Pixel p4 = next_neighbor;

            // Step 3.4
            // Change the value f_(i3, j3) of the pixel(i3, j3) as follows:
            // - If the pixel(i3, j3 + 1) is a 0 - pixel examined in the
            //   step(3.3) then f_(i3, j3) <- -NBD.
            // - If the pixel(i3, j3 + 1) is not a 0 - pixel examined in the
            //   step(3.3) and f_(i3, j3) = 1, then f_(i3, j3) <- NBD.
            // - Otherwise, do not change f_(i3, j3).
            if (examined && (p3.x + 1 >= w || label_(p3.y, p3.x + 1) == 0)) {
                label_(p3.y, p3.x) = -border->id;
            } else if (p3.x + 1 < w && label_(p3.y, p3.x) == 1) {
                label_(p3.y, p3.x) = border->id;
            }

            border->points.push_back(p3);

            // Step 3.5
            // If(i4, j4) = (i, j) and (i3, j3) = (i1, j1) (coming back to the
            // starting point), then go to Step 4;
            // otherwise, (i2, j2) <- (i3, j3), (i3, j3) <- (i4, j4), and go
            // back to Step 3.3.
            if (p4 == start && p3 == p1) {
                return;
            }

            p2 = p3;
            p3 = p4;
        }
    }

    /**
     * Perform scan line algorithm for i-th row of the image.
     */
    void ScanLine(int i, Border* border) {
        Border last_border;
        last_border.id = 1;
        last_border.parent_id = 0;
        last_border.is_outer = borders_[1].is_outer;
        Pixel p2;

        int* line = label_.data() + width_ * i;
        for (int j = 0; j < width_; ++j) {
            // Step 1.
            bool found_border = false;
            Pixel start;
            if (line[j] == 1 && (j == 0 || line[j - 1] == 0)) {
                border->id++;
                border->is_outer = true;
                start = Pixel{j, i};
                p2 = Pixel{j - 1, i};
                found_border = true;
            } else if (line[j] >= 1 && (j + 1 == width_ || line[j + 1] == 0)) {
                border->id++;
                border->is_outer = false;
                start = Pixel{j, i};
                p2 = Pixel{j + 1, i};

                if (line[j] > 1) {
                    last_border.id = line[j];
                    last_border.parent_id = borders_[line[j]].parent_id;
                    last_border.is_outer = borders_[line[j]].is_outer;
                }

                found_border = true;
            }

            if (found_border) {
                // Step 2.
                DetermineBorderHierarchy(last_border, border);
                // Step 3.
                TraceBorder(start, p2, border);
                borders_.push_back(*border);
                border->points.clear();
            } else {
                // Step 4.
                if (line[j] != 1) {
                    last_border.id = std::abs(line[j]);
                    last_border.parent_id = borders_[last_border.id].parent_id;
                    last_border.is_outer = borders_[last_border.id].is_outer;
                }
            }
        }
    }

    /**
     * Perform scan line algorithm for each row of the image.
     */
    void Scan() {
        Border border;
        border.id = 1;
        for (int i = 0; i < height_; ++i) {
            ScanLine(i, &border);
        }
    }

    int height_ = 0;
    int width_ = 0;
    ArrayND<int> label_;
    Array<Border> borders_;
};

namespace image {

/**
 * Get all contours of non-zero pixel components in the binary image and
 * organize them into a tree hierarchy.
 */
template <typename T>
void FindContours(const ImageData<T>& image,
                  Array<ImageContour::Border>* borders) {
    CHECK(borders);
    if (image.empty()) {
        borders->clear();
        return;
    }
    CHECK(image.n_channels() == 1);

    ImageContour contour(image);
    contour.GetBorderTree(borders);
}

/**
 * Similar to the previous one but converting the borders to the multipolygons.
 */
template <typename T>
void FindContours(const ImageData<T>& image, Array<IMultiPolygon2D>* contours) {
    CHECK(contours);

    if (image.empty()) {
        contours->clear();
        return;
    }

    CHECK(image.n_channels() == 1);

    Array<ImageContour::Border> borders;
    FindContours(image, &borders);

    contours->clear();
    Array<int> map(borders.size(), -1);
    for (int i = 1; i < borders.size(); ++i) {
        const auto& border = borders[i];

        int parent = border.parent_id;
        // If the parent was trimmed away, discard this border and all its
        // descendants by leaving map[i] = -1.
        if (parent != 0 && map[parent] == -1) continue;

        IPolygon2D poly(border.points);

        // The computed image contours may contain coincident edges. These need
        // to be trimmed.
        poly.Trim();
        if (poly.empty()) continue;

        if (parent == 0) {
            map[i] = contours->size();
            contours->emplace_back(poly);
        } else {
            map[i] = map[parent];  // propagate index for potential children
            IMultiPolygon2D& parent_poly = contours->at(map[i]);
            parent_poly.Insert(poly, border.is_outer);
        }
    }
}

} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_CONTOUR_H_

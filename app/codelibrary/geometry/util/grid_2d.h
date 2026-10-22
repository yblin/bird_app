//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_GIRD_2D_H_
#define CODELIBRARY_GEOMETRY_UTIL_GIRD_2D_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D grid index system.
 */
template <typename T>
class Grid2D {
    using Point = Point2D<T>;

public:
    explicit Grid2D() = default;

    Grid2D(const Array<Point>& points) {
        Reset(points);
    }

    /**
     * Reset Grid2D with the bounding box and the dimension of the grids.
     */
    void Reset(const Box2D<T>& box, int n_x_grids, int n_y_grids) {
        CHECK(!box.empty());
        CHECK(n_x_grids > 0);
        CHECK(n_y_grids > 0);

        box_ = box;
        x_resolution_ = box_.x_length() / n_x_grids;
        y_resolution_ = box_.y_length() / n_y_grids;
        indices_.clear();
        indices_.reshape(n_x_grids, n_y_grids);
    }

    /**
     * Reset Grid2D with the given points.
     */
    void Reset(const Array<Point>& points) {
        if (points.empty()) return;

        box_ = Box2D<T>(points.begin(), points.end());
        int n = points.size();
        double sqrt_n = std::ceil(std::sqrt(static_cast<double>(n)));
        n = std::max(static_cast<int>(sqrt_n), 1);

        x_resolution_ = box_.x_length() / n;
        y_resolution_ = box_.y_length() / n;
        indices_.clear();
        indices_.reshape(n, n);

        for (const Point& p : points) {
            InsertVertex(p);
        }
    }

    /**
     * Reset Grid2D with the given points and the dimension of the grids.
     */
    void Reset(const Array<Point>& points, int n_x_grids, int n_y_grids) {
        CHECK(n_x_grids > 0);
        CHECK(n_y_grids > 0);

        if (points.empty()) return;

        box_ = Box2D<T>(points.begin(), points.end());
        x_resolution_ = box_.x_length() / n_x_grids;
        y_resolution_ = box_.y_length() / n_y_grids;
        indices_.clear();
        indices_.reshape(n_x_grids, n_y_grids);

        points_ = points;
        for (int i = 0; i < points_.size(); ++i) {
            int g_x = GetXIndex(points_[i].x), g_y = GetYIndex(points_[i].y);
            indices_(g_x, g_y).push_back(i);
        }
    }

    /**
     * Reset Grid2D with the given points and the resolution of the grids.
     */
    void Reset(const Array<Point>& points, double resolution) {
        RBox2D box(points.begin(), points.end());
        CHECK(box.x_length() / resolution <= INT_MAX);
        CHECK(box.y_length() / resolution <= INT_MAX);

        int x_size = box.x_length() / resolution;
        int y_size = box.y_length() / resolution;
        x_size = std::max(x_size, 1);
        y_size = std::max(y_size, 1);

        Reset(points, x_size, y_size);
    }

    /**
     * Insert a point 'p' into grid.
     * Return the index of this point in the points.
     */
    int InsertVertex(const Point& p) {
        CHECK(!indices_.empty());

        int index = points_.size();
        points_.push_back(p);
        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y);
        indices_(g_x, g_y).push_back(index);
        return index;
    }

    /**
     * Find the index of the given point, or return -1 if the point does not
     * exist.
     */
    int Find(const Point& p) const {
        CHECK(!indices_.empty());

        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y);
        for (int i : indices_(g_x, g_y)) {
            if (points_[i] == p) return i;
        }
        return -1;
    }

    /**
     * Find all indices of vertices in the box.
     */
    void Find(const Box2D<T>& box, Array<int>* indices) const {
        CHECK(indices);

        int x_min = GetXIndex(box.x_min());
        int x_max = GetXIndex(box.x_max());
        int y_min = GetYIndex(box.y_min());
        int y_max = GetYIndex(box.y_max());

        indices->clear();
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                for (int i : indices_(x, y)) {
                    if (Intersect(box, points_[i])) {
                        indices->push_back(i);
                    }
                }
            }
        }
    }

    /**
     * Find all vertices in the box.
     */
    void Find(const Box2D<T>& box, Array<Point>* vertices) const {
        CHECK(vertices);

        Array<int> indices;
        Find(box, &indices);
        vertices->resize(indices.size());

        for (int i = 0; i < indices.size(); ++i) {
            (*vertices)[i] = points_[indices[i]];
        }
    }

    /**
     * Return the x-axis index of the value 'v'.
     */
    int GetXIndex(T v) const {
        if (x_resolution_ == 0.0) return 0;

        int n = indices_.shape(0);
        return Clamp((v - box_.x_min()) / x_resolution_, 0.0, n - 1.0);
    }

    /**
     * Return the y-axis index of the value 'v'.
     */
    int GetYIndex(T v) const {
        if (y_resolution_ == 0.0) return 0;

        int n = indices_.shape(1);
        return Clamp((v - box_.y_min()) / y_resolution_, 0.0, n - 1.0);
    }

    /**
     * Clear the indices and points in the grid but not clear the box.
     */
    void clear() {
        indices_.clear();
        points_.clear();
    }

    const Box2D<T>& box() const {
        return box_;
    }

    const Array<Point>& points() const {
        return points_;
    }

    const ArrayND<Array<int>>& indices() const {
        return indices_;
    }

private:
    // Store indices of points in the 2D spatial grid for fast access.
    ArrayND<Array<int>> indices_;

    // Bounding box of vertices.
    Box2D<T> box_;

    // X and Y resolution.
    double x_resolution_ = 0.0, y_resolution_ = 0.0;

    // Store all points.
    Array<Point> points_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_SNAP_2D_H_

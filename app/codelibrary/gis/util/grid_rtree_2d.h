//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_UTIL_GRID_RTREE_2D_H_
#define CODELIBRARY_GIS_UTIL_GRID_RTREE_2D_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/util/rtree_2d.h"

namespace cl {
namespace gis {

/**
 * A spatial index class designed specifically for GIS applications, combining
 * uniform grid partitioning with R-trees to accelerate spatial queries on
 * large-scale GIS entities.
 */
class GridRTree2D {
    using RTree = geometry::RTree2D<double>;

public:
    explicit GridRTree2D() = default;

    GridRTree2D(const GridRTree2D&) = delete;

    GridRTree2D& operator= (const GridRTree2D&) = delete;

    /**
     * Reset GridRTree2D with the bounding box and the dimension of the grids.
     */
    void Reset(const RBox2D& box, int n_x_grids, int n_y_grids) {
        CHECK(!box.empty());
        CHECK(n_x_grids > 0);
        CHECK(n_y_grids > 0);

        this->clear();

        box_ = box;
        x_resolution_ = box_.x_length() / n_x_grids;
        y_resolution_ = box_.y_length() / n_y_grids;
        indices_.reshape(n_x_grids, n_y_grids);
        rtrees_.reshape(n_x_grids, n_y_grids);
    }

    /**
     * Construc grid-rtree2D from the given segments and the resolution of the
     * grids.
     */
    void Construct(const Array<RSegment2D>& segments, double resolution) {
        CHECK(resolution > 0.0);

        RBox2D box;
        for (const auto& s : segments) {
            box.Union(s.bounding_box());
        }
        CHECK(box.x_length() / resolution <= INT_MAX);
        CHECK(box.y_length() / resolution <= INT_MAX);

        int x_size = box.x_length() / resolution;
        int y_size = box.y_length() / resolution;
        x_size = std::max(x_size, 1);
        y_size = std::max(y_size, 1);

        Reset(box, x_size, y_size);

        for (const auto& s : segments) {
            int index = entities_.size();
            const RBox2D& box = s.bounding_box();
            entities_.push_back(box);

            int x_min = GetXIndex(box.x_min());
            int x_max = GetXIndex(box.x_max());
            int y_min = GetYIndex(box.y_min());
            int y_max = GetYIndex(box.y_max());

            for (int x = x_min; x <= x_max; ++x) {
                for (int y = y_min; y <= y_max; ++y) {
                    indices_(x, y).push_back(index);
                }
            }
        }

        CHECK(indices_.size() == rtrees_.size());
        Array<RBox2D> grid_entities;
        for (int i = 0; i < indices_.size(); ++i) {
            grid_entities.clear();
            for (int j : indices_[i]) {
                grid_entities.push_back(entities_[j]);
            }
            rtrees_[i] = rtree_pool_.Allocate();
            rtrees_[i]->Construct(grid_entities);
        }
    }

    /**
     * Find all indices of entities intersected with the given box.
     */
    void Find(const RBox2D& box, Array<int>* indices) const {
        CHECK(indices);

        int x_min = GetXIndex(box.x_min());
        int x_max = GetXIndex(box.x_max());
        int y_min = GetYIndex(box.y_min());
        int y_max = GetYIndex(box.y_max());

        indices->clear();
        Array<int> entries;
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                if (rtrees_(x, y))
                    rtrees_(x, y)->Find(box, &entries);
                const Array<int>& local_to_global = indices_(x, y);
                for (int i : entries) {
                    indices->push_back(local_to_global[i]);
                }
            }
        }
        std::sort(indices->begin(), indices->end());
        indices->erase(std::unique(indices->begin(), indices->end()),
                       indices->end());
    }

    /**
     * Return the x-axis index of the value 'v'.
     */
    int GetXIndex(double v) const {
        if (x_resolution_ == 0.0) return 0;

        int n = indices_.shape(0);
        return Clamp((v - box_.x_min()) / x_resolution_, 0.0, n - 1.0);
    }

    /**
     * Return the y-axis index of the value 'v'.
     */
    int GetYIndex(double v) const {
        if (y_resolution_ == 0.0) return 0;

        int n = indices_.shape(1);
        return Clamp((v - box_.y_min()) / y_resolution_, 0.0, n - 1.0);
    }

    /**
     * Clear the indices and entities but not clear the box.
     */
    void clear() {
        indices_.clear();
        rtrees_.clear();
        entities_.clear();
        rtree_pool_.clear();
    }

    /**
     * Working space of grid-rtree.
     */
    const RBox2D& box() const {
        return box_;
    }

    /**
     * Indices of entities stored in the 2D spatial grid.
     */
    const ArrayND<Array<int>>& indices() const {
        return indices_;
    }

    /**
     * The input entities.
     */
    const Array<RBox2D>& entities() const {
        return entities_;
    }

private:
    // Working space.
    RBox2D box_;

    // X and Y resolution.
    double x_resolution_ = 0.0, y_resolution_ = 0.0;

    // The stored entities.
    Array<RBox2D> entities_;

    // Store indices of entities in the 2D spatial grid for fast access.
    ArrayND<Array<int>> indices_;

    // Rtrees in each grid.
    ArrayND<RTree*> rtrees_;

    // Rtree allocator.
    Pool<RTree> rtree_pool_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_UTIL_GRID_RTREE_2D_H_

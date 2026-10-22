//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_SAMPLING_H_
#define CODELIBRARY_POINT_CLOUD_SAMPLING_H_

#include <algorithm>
#include <type_traits>
#include <utility>

#include "codelibrary/base/array.h"

namespace cl {
namespace point_cloud {

/**
 * Generate a voxel code for each point (used for sampling or geometric hash).
 *
 * Note: Discretized voxel coordinates must lie within the 21-bit range
 *       [-1048576, 1048575].
 *
 * Parameters:
 *  points     - the input point cloud.
 *  resolution - the resolution of the voxel grid. Smaller values result in
 *               finer grids.
 *  samples    - the output array of voxel codes for each point.
 *
 * Return false if the voxel coordinates are out of the allowed range
 * (resolution is too small).
 */
template <typename Point>
bool Voxelize(const Array<Point>& points, double resolution,
              Array<uint64_t>* codes) {
    CHECK(resolution > 0.0);
    CHECK(codes);

    using T = typename Point::value_type;

    codes->resize(points.size());
    if (points.empty()) {
        return true;
    }

    const double inv_resolution = 1.0 / resolution;

    // Bits to represent each voxel coordinate (pack 21x3=63bits in 64bit int).
    const int coord_bit_size = 21;
    // Bit mask.
    const int coord_bit_mask = (1 << 21) - 1;
    // Coordinate offset to make values positive.
    const int coord_offset = 1 << (coord_bit_size - 1);

    auto compute_voxel_coord = [&](T coord) -> int {
        // The following code is much faster than much faster than
        // (int)floor(x).
        coord *= inv_resolution;
        int voxel_coord = static_cast<int>(coord);
        voxel_coord = voxel_coord - (voxel_coord > coord) + coord_offset;
        if (voxel_coord < 0 || voxel_coord > coord_bit_mask) {
            LOG(WARNING) << "Voxel coordinate is out of range.";
            return -1; // Indicate an out-of-range error.
        }
        return voxel_coord;
    };

    int x, y, z;
    for (int i = 0; i < points.size(); ++i) {
        x = compute_voxel_coord(points[i].x);
        y = compute_voxel_coord(points[i].y);
        z = compute_voxel_coord(points[i].z);

        if (x == -1 || y == -1 || z == -1) {
            return false;
        }

        // Compute voxel coordinate bits.
        (*codes)[i] = (uint64_t(x & coord_bit_mask) << (coord_bit_size * 0)) |
                      (uint64_t(y & coord_bit_mask) << (coord_bit_size * 1)) |
                      (uint64_t(z & coord_bit_mask) << (coord_bit_size * 2));
    }
    return true;
}

/**
 * Voxelgrid downsampling.
 *
 * Discretize the input point cloud into a voxel grid and downsamples it.
 * Note: Discretized voxel coordinates must lie within the 21-bit range
 *       [-1048576, 1048575].
 *
 * Parameters:
 *  points     - Input points to be downsampled.
 *  resolution - The resolution of the voxel grid. Smaller values result in
 *               finer grids.
 *  samples    - The indices of the downsampled points.
 *
 * Return false if the voxel coordinates are out of the allowed range
 * (resolution is too small).
 */
template <typename Point>
bool VoxelSampling(const Array<Point>& points, double resolution,
                   Array<int>* samples) {
    CHECK(resolution > 0.0);
    CHECK(samples);

    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value);

    samples->clear();
    if (points.empty()) {
        return true;
    }

    Array<uint64_t> codes;
    if (!Voxelize(points, resolution, &codes)) return false;

    CHECK(codes.size() == points.size());
    Array<std::pair<uint64_t, int>> coord_pt(points.size());
    for (int i = 0; i < codes.size(); ++i) {
        coord_pt[i] = std::make_pair(codes[i], i);
    }

    // Sort by voxel coordinates.
    const auto compare = [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    };
    std::sort(coord_pt.begin(), coord_pt.end(), compare);

    samples->push_back(coord_pt.front().second);
    for (int i = 1; i < points.size(); ++i) {
        if (coord_pt[i - 1].first != coord_pt[i].first) {
            samples->push_back(coord_pt[i].second);
        }
    }

    return true;
}

/**
 * Voxel grid mean downsampling.
 *
 * This function discretizes the input point cloud into a voxel grid and
 * computes the average (mean) point within each occupied voxel, effectively
 * downsampling the point cloud.
 *
 * Discretize the input point cloud into a voxel grid and downsamples it.
 * Note: Discretized voxel coordinates must lie within the 21-bit range
 *       [-1048576, 1048575].
 *
 * Parameters:
 *  points     - Input points to be downsampled.
 *  resolution - The resolution of the voxel grid. Smaller values result in
 *               finer grids.
 *  samples    - Output downsampled points. The output is the mean point of all
 *               points within each voxel
 *
 * Return false if the voxel coordinates are out of the allowed range
 * (resolution is too small).
 */
template <typename Point>
bool VoxelMeanSampling(const Array<Point>& points, double resolution,
                       Array<Point>* samples) {
    CHECK(resolution > 0.0);
    CHECK(samples);

    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value);

    samples->clear();
    if (points.empty()) {
        return true;
    }

    Array<uint64_t> codes;
    if (!Voxelize(points, resolution, &codes)) return false;

    CHECK(codes.size() == points.size());
    Array<std::pair<uint64_t, int>> coord_pt(points.size());
    for (int i = 0; i < codes.size(); ++i) {
        coord_pt[i] = std::make_pair(codes[i], i);
    }

    // Sort by voxel coordinates.
    const auto compare = [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    };
    std::sort(coord_pt.begin(), coord_pt.end(), compare);

    int n_samples = 0;
    // We allow &samples == &points.
    samples->resize(points.size());

    Point sum_pt = points[coord_pt.front().second];
    int s = 1;
    for (int i = 1; i < points.size(); ++i) {
        if (coord_pt[i - 1].first != coord_pt[i].first) {
            (*samples)[n_samples++] = sum_pt * (T(1) / s);
            sum_pt = Point(0, 0, 0);
            s = 0;
        }
        const Point& p = points[coord_pt[i].second];
        sum_pt.x += p.x;
        sum_pt.y += p.y;
        sum_pt.z += p.z;
        ++s;
    }
    (*samples)[n_samples++] = sum_pt * (T(1) / s);
    samples->resize(n_samples);

    return true;
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_SAMPLING_H_

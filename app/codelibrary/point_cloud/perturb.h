//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_PERTURB_H_
#define CODELIBRARY_POINT_CLOUD_PERTURB_H_

#include <random>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {
namespace point_cloud {

/**
 * Apply Gaussian noise to a point cloud.
 */
template <typename T>
void GaussianPerturb(double mean, double stddev, Array<Point2D<T>>* points) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(points);

    std::default_random_engine generator;
    std::normal_distribution<T> distribution(mean, stddev);

    for (auto& p : *points) {
        p.x += distribution(generator);
        p.y += distribution(generator);
    }
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_PERTURB_H_

//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_FEATURE_FPFH_FEATURE_H_
#define CODELIBRARY_POINT_CLOUD_FEATURE_FPFH_FEATURE_H_

#include <limits>
#include <utility>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/math/vector.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace point_cloud {

/**
 * Compute the 3-tuple representation containing the three angles between two
 * points represented by Cartesian coordinates and normals.
 */
template <class Point, typename T>
bool EstimatePairFeature(const Point& p1, const Vector3D<T>& n1,
                         const Point& p2, const Vector3D<T>& n2,
                         FVector3D* feature) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(feature);

    Vector3D<T> v12(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    T dis = v12.norm();

    if (dis == 0) {
        LOG(WARNING) << "The distance between points is 0!";
        feature->set_zero();
        return false;
    }

    float angle1 = static_cast<float>(n1.dot(v12) / dis);

    Vector3D<T> t1 = n1, t2 = n2;

    // Make sure the same point is selected as 1 and 2 for each pair.
    float angle2 = static_cast<float>(n2.dot(v12) / dis);
    if (std::acos(std::fabs(angle1)) > std::acos(std::fabs(angle2))) {
        // switch p1 and p2.
        std::swap(t1, t2);
        v12 *= T(-1);
        (*feature)[2] = -angle2;
    } else {
        (*feature)[2] = -angle1;
    }

    // Create a Darboux frame coordinate system u-v-w
    // u = n1; v = (p - q) x u / || (p - q) x u ||; w = u x v
    Vector3D<T> v = v12.cross(t1);
    T v_norm = v.norm();
    if (v_norm <= std::numeric_limits<T>::epsilon()) {
        LOG(WARNING) << "Norm of Delta x U is too small: " << v_norm;
        feature->set_zero();
        return false;
    }
    v *= 1.0 / v_norm;

    // Do not have to normalize w - it is a unit vector by construction.
    Vector3D<T> w = t1.cross(v);

    (*feature)[1] = v.dot(t2);

    // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w
    // coordinate system
    (*feature)[0] = std::atan2(w.dot(t2), t1.dot(t2));

    return true;
}

/**
 * Compute the Simplified Point Feature Histogram (SPFH) for a specific point.
 *
 * Parameters:
 *  points      - the input point cloud.
 *  normals     - the normals corresponding to the points.
 *  p_index     - the index of the point for which to compute SPFH feature.
 *  neighbors   - ndices of the neighboring points used for SPFH computation.
 *  n_dimension - the output dimension of SPFH.
 *  feature     - the computed SPFH feature for the point.
 */
template <class Point, typename T>
void EstimateSPFHFeature(const Array<Point>& points,
                         const Array<Vector3D<T>>& normals,
                         int p_index,
                         const Array<int>& neighbors,
                         int n_dimension,
                         FVector* feature) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(points.size() == normals.size());
    CHECK(n_dimension > 0 && n_dimension % 3 == 0);
    CHECK(p_index >= 0 && p_index < points.size());
    CHECK(!neighbors.empty());
    CHECK(feature);

    feature->assign(n_dimension, 0.0f);

    // Factorization constant.
    float hist_incr = 100.0f / static_cast<float>(neighbors.size() - 1);

    const Point& p = points[p_index];
    const Vector3D<T>& n1 = normals[p_index];

    int n_bins = n_dimension / 3;

    // PFH feature.
    FVector3D pfh;

    // Iterate over all the points in the neighbors.
    for (int index : neighbors) {
        if (p_index == index) continue;

        CHECK(index >= 0 && index < points.size());

        const Point& q = points[index];
        const Vector3D<T>& n2 = normals[index];

        // Compute the pair P to NNi.
        if (!EstimatePairFeature(p, n1, q, n2, &pfh)) continue;

        // Normalize PFH and push it into the histogram.

        // The following code is much faster than much faster than
        // (int)floor(x).
        auto fast_floor = [&](float v) {
            int t = static_cast<int>(v);
            return t - (t > v);
        };

        float d_pi = 1.0f / (2.0f * numbers::pi_v<float>);
        int h_index = fast_floor(n_bins * (pfh[0] + numbers::pi_v<float>) *
                                 d_pi);
        h_index = Clamp(h_index, 0, n_bins - 1);
        (*feature)[h_index] += hist_incr;

        h_index = fast_floor(n_bins * (pfh[1] + 1.0) * 0.5);
        h_index = Clamp(h_index, 0, n_bins - 1);
        (*feature)[h_index + n_bins] += hist_incr;

        h_index = fast_floor(n_bins* (pfh[2] + 1.0) * 0.5);
        h_index = Clamp(h_index, 0, n_bins - 1);
        (*feature)[h_index + 2 * n_bins] += hist_incr;
    }
}

/**
 * Estimate SPFH (Simplified Point Feature Histograms) features for the input
 * point cloud.
 *
 * Parameters:
 *  kd_tree     - the input point cloud stored in KD tree.
 *  normals     - the normals corresponding to the points.
 *  n_neighbors - the number of neighbors to compute SPFH.
 *  n_dimension - the output dimension of SPFH.
 *  features    - output array to store the computed SPFH features.
 *
 * Reference:
 *  R.B. Rusu, N. Blodow, M. Beetz. Fast Point Feature Histograms (FPFH) for 3D
 *  Registration. In Proceedings of the IEEE ICRA, Kobe, Japan, May 12-17 2009.
 */
template <class Point, typename T>
bool EstimateSPFHFeatures(const KDTree<Point>& kd_tree,
                          int n_neighbors,
                          const Array<Vector3D<T>>& normals,
                          int n_dimension,
                          Array<FVector>* features) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(kd_tree.size() == normals.size());
    CHECK(n_neighbors > 1);
    CHECK(n_dimension > 0 && n_dimension % 3 == 0);
    CHECK(features);

    features->clear();
    if (kd_tree.size() < n_neighbors) {
        LOG(WARNING) << "The number of input points is smaller than the given "
                        "number of neighbors.";
        return false;
    }

    const Array<Point>& points = kd_tree.points();
    features->reserve(points.size());

    FVector feature;
    Array<int> neighbors;
    for (int i = 0; i < kd_tree.size(); ++i) {
        CHECK(i >= 0 && i < kd_tree.size());

        kd_tree.FindKNearestNeighbors(points[i], n_neighbors, &neighbors);
        EstimateSPFHFeature(points, normals, i, neighbors, n_dimension,
                            &feature);
        features->push_back(feature);
    }

    return true;
}

/**
 * Estimate FPFH (Fast Point Feature Histograms) feature for each point
 * specified in the indices array.
 *
 * The following code is modified from PCL.
 *
 * Parameters:
 *  kd_tree       - the input point cloud stored in KD tree.
 *  normals       - the normals corresponding to the points.
 *  n_neighbors   - the number of neighbors to compute FPFH.
 *  indices       - indices of the points for which to compute FPFH feature.
 *  n_dimension   - the output dimension of SPFH.
 *  spfh_features - the computed SPFH features for all points.
 *  features      - the computed FPFH features for the points specified in
 *                  'indices'.
 *
 * Reference:
 *  R.B. Rusu, N. Blodow, M. Beetz. Fast Point Feature Histograms (FPFH) for 3D
 *  Registration. In Proceedings of the IEEE ICRA, Kobe, Japan, May 12-17 2009.
 */
template <class Point, typename T>
bool EstimateFPFHFeatures(const KDTree<Point>& kd_tree,
                          int n_neighbors,
                          const Array<Vector3D<T>>& normals,
                          const Array<int>& indices,
                          int n_dimension,
                          Array<FVector>* spfh_features,
                          Array<FVector>* features) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(kd_tree.size() == normals.size());
    CHECK(n_neighbors > 1);
    CHECK(n_dimension > 0 && n_dimension % 3 == 0);
    CHECK(spfh_features);
    CHECK(features);

    features->clear();
    if (kd_tree.size() < n_neighbors) {
        LOG(WARNING) << "The number of input points is smaller than the given "
                        "number of neighbors.";
        return false;
    }

    if (!EstimateSPFHFeatures(kd_tree, n_neighbors, normals, n_dimension,
                              spfh_features))
        return false;
    CHECK(spfh_features->size() == kd_tree.size());

    FVector feature(n_dimension, 0.0f);
    features->reserve(indices.size());

    const Array<Point>& points = kd_tree.points();
    Array<int> neighbors;
    for (int i : indices) {
        CHECK(i >= 0 && i < kd_tree.size());

        kd_tree.FindKNearestNeighbors(i, n_neighbors, &neighbors);
        int k = 0;
        feature.set_zero();
        for (int j : neighbors) {
            if (i == j) continue;

            float dis = static_cast<float>(Distance(points[i], points[j]));

            feature += (1.0f / dis) * (*spfh_features)[j];
            ++k;
        }
        feature *= 1.0f / k;

        feature += (*spfh_features)[i];
        features->push_back(feature);
    }

    return true;
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_FEATURE_FPFH_FEATURE_H_

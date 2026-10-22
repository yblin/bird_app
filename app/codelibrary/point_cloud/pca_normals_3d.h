//
// Copyright 2014-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_
#define CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_

#include <algorithm>

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/point_cloud/principal_component_analysis_3d.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace point_cloud {

/**
 * Estimate normal vector by weighted PCA method.
 *
 * The output normal vector is normalize to the unit length. Its orientation is
 * randomly assigned.
 *
 * Note that, we only need to compute the least eigenvector of the covariance
 * matrix.
 */
template <class Point, typename T>
void PCANormal(const Array<Point>& points, const Array<T>& weights,
               Vector3D<T>* normal) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(normal);
    PrincipalComponentAnalysis3D<T> pca(points, weights);
    const Matrix3<T>& eigenvectors = pca.eigenvectors();
    *normal = {eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0)};
}

/**
 * Estimate normal vector by PCA method.
 *
 * The output normal vector is normalize to the unit length. Its orientation is
 * randomly assigned.
 */
template <class Point, typename T>
void PCANormal(const Array<Point>& points, Vector3D<T>* normal) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(normal);
    PrincipalComponentAnalysis3D<T> pca(points);
    const Matrix3<T>& eigenvectors = pca.eigenvectors();
    *normal = {eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0)};
}

/**
 * Estimate normal directions of the 3D points by PCA method over the k nearest
 * neighbors.
 *
 * The output normal vectors are normalize to the unit length. Their orientation
 * are randomly assigned.
 *
 *   kd_tree  - the input points are stored in the KD tree.
 *   k        - used to define the k-nearest neighbors.
 *   normals  - the output normals.
 */
template <class Point, typename T>
void PCANormals(const KDTree<Point>& kd_tree, int k,
                Array<Vector3D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(k > 0);
    CHECK(normals);

    if (kd_tree.empty()) {
        normals->clear();
        return;
    }

    int n = kd_tree.size();
    k = std::min(k, n);

    normals->resize(n);

    const Array<Point>& points = kd_tree.points();
    Array<Point> neighbors;

    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);
        PCANormal(neighbors, &(*normals)[i]);
    }
}

/**
 * Estimate a set of normal vectors by PCA.
 */
template <class Point, typename T>
void PCANormals(const Array<Point>& points, int k,
                Array<Vector3D<T>>* normals) {
    KDTree<Point> kd_tree(points);
    PCANormals(kd_tree, k, normals);
}

/**
 * Orientation-aware PCA normal estimation.
 *
 * This function will re-orient the normal vector for each point according to
 * the neighbors whose normal vector has the same orientation.
 *
 * Parameters:
 *   kd_tree - the input points are stored in the KD tree.
 *   k       - used to define the k-nearest neighbors.
 *   normals - the normals that will be recomputed.
 */
template <class Point, typename T>
void OrientationAwarePCANormals(const KDTree<Point>& kd_tree, int k,
                                Array<Vector3D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(k > 0);
    CHECK(normals);
    CHECK(normals->size() == kd_tree.size());

    if (kd_tree.empty()) return;

    int n = kd_tree.size();
    k = std::min(k, n);

    const Array<Point>& points = kd_tree.points();
    Array<int> neighbors;

    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);

        Array<Point> neighbor_points;
        neighbor_points.reserve(k);
        for (int j = 0; j < k; ++j) {
            if ((*normals)[i].dot((*normals)[neighbors[j]]) >= 0.0) {
                neighbor_points.push_back(points[neighbors[j]]);
            }
        }

        Vector3D<T> normal;
        PCANormal(neighbor_points, &normal);
        if (normal.dot((*normals)[i]) < 0) {
            (*normals)[i] = -normal;
        } else {
            (*normals)[i] =  normal;
        }
    }
}

/**
 * Similar to the previous one.
 */
template <class Point, typename T>
void OrientationAwarePCANormals(const Array<Point>& points, int k,
                                Array<Vector3D<T>>* normals) {
    KDTree<Point> kd_tree(points);
    OrientationAwarePCANormals(kd_tree, k, normals);
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_

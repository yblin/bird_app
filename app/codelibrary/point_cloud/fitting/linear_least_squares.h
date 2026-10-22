//
// Copyright 2017-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_FITTING_LINEAR_LEAST_SQUARES_H_
#define CODELIBRARY_POINT_CLOUD_FITTING_LINEAR_LEAST_SQUARES_H_

#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/point_cloud/principal_component_analysis_3d.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace point_cloud {

/**
 * Linear least Square fitting for 3D plane.
 *
 * Return fitting quality: 0 is best, 1 is worst.
 */
template <typename T>
double LinearLeastSquares(const Array<Point3D<T>>& points, Plane3D<T>* plane) {
    static_assert(std::is_floating_point<T>::value);
    CHECK(plane);

    if (points.size() == 0) {
        // Return an empty plane.
        *plane = Plane3D<T>();
        return 1.0;
    }

    PrincipalComponentAnalysis3D<T> pca(points);

    auto& eigenvalues = pca.eigenvalues();
    if (eigenvalues[0] < 0.0 || eigenvalues[0] >= eigenvalues[1]) {
        // Return a degenerated plane.
        *plane = Plane3D<T>(pca.centroid(), Vector3D<T>(0, 0, 0));
        return 1.0;
    }

    auto& eigenvectors = pca.eigenvectors();
    if (eigenvectors[0].norm() <= std::numeric_limits<T>::epsilon()) {
        Vector3D<T> n = eigenvectors[1].cross(eigenvectors[2]);
        *plane = Plane3D<T>(pca.centroid(), n);
    } else {
        *plane = Plane3D<T>(pca.centroid(), eigenvectors[0]);
    }
    return eigenvalues[0] / eigenvalues[1];
}

/**
 * Weighted linear least Square fitting for 3D plane.
 *
 * Return fitting quality: 0 is best, 1 is worst.
 */
template <typename T>
double LinearLeastSquares(const Array<Point3D<T>>& points,
                          const Array<T>& weights,
                          Plane3D<T>* plane) {
    static_assert(std::is_floating_point<T>::value);
    CHECK(plane);

    if (points.size() == 0) {
        // Return an empty plane.
        *plane = Plane3D<T>();
        return 1.0;
    }

    PrincipalComponentAnalysis3D<T> pca(points, weights);
    return pca.ToPlane(plane);
}

/**
 * Iteratively linear least squares fitting for 3D plane.
 *
 * Reference:
 *   Chauve A L, Labatut P, Pons J P. Robust piecewise-planar 3D reconstruction
 *   and completion from large-scale unstructured point data[C]. IEEE Conference
 *   on Computer Vision and Pattern Recognition (CVPR), 2010: 1261-1268.
 *
 * Parameters:
 *  points     - an array of 3D points to fit the plane.
 *  resolution - used to filter points based on their distance to the plane.
 *  plane      - the estimated 3D plane.
 *
 * Return fitting quality: 0 is best, 1 is worst.
 */
template <typename T>
double IterativelyLinearLeastSquares(const Array<Point3D<T>>& points,
                                     double resolution,
                                     Plane3D<T>* plane) {
    Array<Point3D<T>> ps = points;

    double score = 1.0;
    const int max_iters = 5;
    int iter = 0;
    do {
        score = LinearLeastSquares(ps, plane);

        // Delete the points whose distance to the tangent plane is
        // smaller than resolution / 2.
        int size = 0;
        for (const auto& p : ps) {
            if (Distance(p, *plane) <= resolution * 0.5) {
                ps[size++] = p;
            }
        }
        if (size == ps.size()) break;
        if (size < 3) {
            return 1.0;
        }
        ps.resize(size);
    } while (iter++ < max_iters);
    return score;
}

/**
 * Similar to the previous function, but estimates local planes for all points
 * stored in the KD tree.
 *
 * Reference:
 *   Chauve A L, Labatut P, Pons J P. Robust piecewise-planar 3D reconstruction
 *   and completion from large-scale unstructured point data[C]. IEEE Conference
 *   on Computer Vision and Pattern Recognition (CVPR), 2010: 1261-1268.
 *
 * Parameters:
 *  kd_tree    - a KD tree containing the input points.
 *  resolution - used to determine the local plane for each point.
 *  planes     - the estimated local plane for each point.
 *  scores     - (optional) the fitting quality of each plane.
 */
template <typename T>
void IterativelyReweightedLinearLeastSquares(const KDTree<Point3D<T>>& kd_tree,
                                             double resolution,
                                             Array<Plane3D<T>>* local_planes,
                                             Array<double>* scores = nullptr) {
    CHECK(resolution > 0.0);
    CHECK(local_planes);

    int n_points = kd_tree.size();

    local_planes->resize(n_points);
    if (scores) scores->assign(n_points, 1.0);

    Array<Point3D<T>> neighbor_points;
    double radius = 4.0 * resolution * resolution;

    const Array<Point3D<T>>& points = kd_tree.points();
    for (int i = 0; i < n_points; ++i) {
        kd_tree.FindRadiusNeighbors(points[i], radius, &neighbor_points);

        double s = IterativelyLinearLeastSquares(neighbor_points, resolution,
                                                 &(*local_planes)[i]);
        if (scores) (*scores)[i] = s;
    }
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_FITTING_LINEAR_LEAST_SQUARES_H_

//
// Copyright 2018-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_REGION_GROWING_H_
#define CODELIBRARY_POINT_CLOUD_REGION_GROWING_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/point_cloud/k_nearest_neighbors.h"
#include "codelibrary/point_cloud/fitting/linear_least_squares.h"

namespace cl {
namespace point_cloud {

/**
 * Region growing method to find all 3D planes present in the input point cloud,
 * and outputs an array of planes, as well as an array of labels to determine
 * each point belong to which plane.
 *
 * Only planes with more than 'min_support_points' points are detected.
 *
 * Reference:
 *   Chauve A L, Labatut P, Pons J P. Robust piecewise-planar 3D reconstruction
 *   and completion from large-scale unstructured point data[C]. IEEE Conference
 *   on Computer Vision and Pattern Recognition (CVPR), 2010: 1261-1268.
 *
 * Parameters:
 *  kd_tree            - a KD tree containing the input points.
 *  resolution         - used to determine the local plane for each point.
 *  k_neighbors        - number of neighbors in growing.
 *  angle_threshold    - maximal allowed angle (in degrees) between the local
 *                       plane and the fitting plane.
 *  min_support_points - minimal number of points required to define a plane.
 *  labels             - indicate which plane each point belongs to.
 *  planes             - the detected planes.
 */
template <typename T>
void RegionGrowingPlanes(const KDTree<Point3D<T>>& kd_tree,
                         double resolution,
                         int k_neighbors,
                         double angle_threshold,
                         int min_support_points,
                         Array<int>* labels,
                         Array<Plane3D<T>>* planes) {
    CHECK(resolution > 0.0);
    CHECK(k_neighbors > 0);
    CHECK(angle_threshold > 0.0 && angle_threshold < 45.0);
    CHECK(min_support_points > 0);
    CHECK(labels);
    CHECK(planes);

    int n_points = kd_tree.size();
    labels->assign(n_points, -1);
    planes->clear();

    if (n_points < k_neighbors) return;

    // Then growing each region according to the sequence of smoothness.
    Array<Plane3D<T>> local_planes;
    Array<double> smoothness;
    IterativelyReweightedLinearLeastSquares(kd_tree, resolution, &local_planes,
                                            &smoothness);
    Array<int> seq;
    IndexSort(smoothness.begin(), smoothness.end(), &seq);

    Array<Array<int>> neighbors;
    MutalKNearestNeighbors(kd_tree, k_neighbors, &neighbors);

    const Array<Point3D<T>>& points = kd_tree.points();
    int n_planes = 0;
    for (int index : seq) {
        if ((*labels)[index] != -1) continue;

        Plane3D<T> plane = local_planes[index];

        double best = DBL_MAX;
        Array<int> queue;

        while (true) {
            int front = 0;
            queue.push_back(index);
            (*labels)[index] = n_planes;

            while (front < queue.size()) {
                int cur = queue[front++];
                for (int j : neighbors[cur]) {
                    if ((*labels)[j] != -1) continue;
                    if (local_planes[j].empty()) continue;

                    double angle = Degree(local_planes[j].normal(),
                                          plane.normal());
                    if (angle > angle_threshold &&
                        angle < 180.0 - angle_threshold) continue;

                    if (Distance(points[j], plane) <= resolution) {
                        (*labels)[j] = (*labels)[cur];
                        queue.push_back(j);
                    }
                }
            }

            // If the cluster is too small, drop it.
            if (queue.size() < 3) {
                for (int v : queue) {
                    (*labels)[v] = -1;
                }
                queue.clear();
                break;
            }

            Array<Point3D<T>> ps(queue.size());
            for (int i = 0; i < queue.size(); ++i) {
                ps[i] = points[queue[i]];
            }
            double metrics = LinearLeastSquares(ps, &plane);
            if (metrics + DBL_EPSILON < best) {
                best = metrics;
            } else {
                if (queue.size() < min_support_points) {
                    for (int v : queue) {
                        (*labels)[v] = -1;
                    }
                    queue.clear();
                }
                break;
            }

            for (int v : queue) {
                (*labels)[v] = -1;
            }
            queue.clear();
        }

        if (!queue.empty()) {
            planes->push_back(plane);
            ++n_planes;
        }
    }
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_REGION_GROWING_H_

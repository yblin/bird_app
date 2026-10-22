//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_CLUSTERING_K_MEANS_CLUSTER_H_
#define CODELIBRARY_STATISTICS_CLUSTERING_K_MEANS_CLUSTER_H_

#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace clustering {

/**
 * Assign each point to the nearest seed.
 *
 * Return true if any label have changed.
 */
template <typename Point>
bool KMeansCluster(const Array<Point>& points, const Array<Point>& seeds,
                   Array<int>* labels) {
    CHECK(labels);
    CHECK(!seeds.empty());

    KDTree<Point> kd_tree(seeds);
    labels->resize(points.size(), -1);
    bool changed = false;
    for (int i = 0; i < points.size(); ++i) {
        int c = kd_tree.FindNearestIndex(points[i]);
        if (c != (*labels)[i]) {
            (*labels)[i] = c;
            changed = true;
        }
    }

    return changed;
}

/**
 * Similar the previous function, but return the cluster indices.
 */
template <typename Point>
bool KMeansCluster(const Array<Point>& points, const Array<Point>& seeds,
                   Array<int>* labels, Array<Array<int>>* clusters) {
    CHECK(clusters);

    clusters->clear();
    if (!KMeansCluster(points, seeds, labels)) return false;

    clusters->resize(seeds.size());
    for (int i = 0; i < labels->size(); ++i) {
        auto& cluster = (*clusters)[(*labels)[i]];
        cluster.push_back(i);
    }
    return true;
}

/**
 * Similar the previous function, but return the cluster points.
 */
template <typename Point>
bool KMeansCluster(const Array<Point>& points, const Array<Point>& seeds,
                   Array<int>* labels, Array<Array<Point>>* cluster_points) {
    CHECK(cluster_points);

    cluster_points->clear();

    Array<Array<int>> clusters;
    if (!KMeansCluster(points, seeds, labels, &clusters)) return false;

    cluster_points->resize(seeds.size());
    for (int i = 0; i < clusters.size(); ++i) {
        auto& cluster = (*cluster_points)[i];
        for (int j : clusters[i]) {
            cluster.push_back(points[j]);
        }
    }
    return true;
}

} // namespace clustering
} // namespace cl

#endif // CODELIBRARY_STATISTICS_CLUSTERING_K_MEANS_CLUSTER_H_

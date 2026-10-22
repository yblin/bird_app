//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_CLUSTERING_DBSCAN_CLUSTER_H_
#define CODELIBRARY_STATISTICS_CLUSTERING_DBSCAN_CLUSTER_H_

#include <queue>

#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace clustering {

/**
 * Density-based spatial clustering of applications with noise (DBSCAN) is a
 * density-based clustering non-parametric algorithm: given a set of points in
 * some space, it groups together points that are closely packed together
 * (points with many nearby neighbors), marking as outliers points that lie
 * alone in low-density regions (whose nearest neighbors are too far away).
 */
class DBSCANCluster {
public:
    DBSCANCluster(double epsilon, int min_points)
        : epsilon_(epsilon), min_points_(min_points) {
        CHECK(epsilon_ > 0.0);
        CHECK(min_points_ > 1);
    }

    /**
     * Cluster points.
     *
     * It assigns each point a cluster label, if the point is marked as outlier,
     * its label will set to -1.
     *
     * Return the number of clusters.
     */
    template <typename Point>
    int Cluster(const Array<Point>& points, Array<int>* labels) const {
        CHECK(labels);

        KDTree<Point> kd_tree(points);
        return Cluster(kd_tree, labels);
    }

    /**
     * Similar to the previous function, but the input points are stored in
     * KD-tree.
     */
    template <typename Point>
    int Cluster(const KDTree<Point>& kdtree, Array<int>* labels) const {
        CHECK(labels);

        labels->resize(kdtree.size(), -1);
        Array<bool> visited(kdtree.size(), false);
        Array<int> neighbors;
        double epsilon2 = epsilon_ * epsilon_;
        int n_clusters = 0;
        for (int i = 0; i < kdtree.size(); ++i) {
            const auto& p = kdtree.points()[i];
            if (visited[i]) continue;
            visited[i] = true;

            kdtree.FindRadiusNeighbors(p, epsilon2, &neighbors);
            if (neighbors.size() < min_points_) {
                continue;
            }

            (*labels)[i] = n_clusters;
            // Region growing.
            std::queue<int> queue;
            for (int j : neighbors) {
                queue.push(j);
            }

            while (!queue.empty()) {
                int cur = queue.front();
                queue.pop();
                if (!visited[cur]) {
                    visited[cur] = true;
                    (*labels)[cur] = n_clusters;

                    const Point& q = kdtree.points()[cur];
                    kdtree.FindRadiusNeighbors(q, epsilon2, &neighbors);
                    if (neighbors.size() >= min_points_) {
                        for (int k : neighbors) {
                            if (!visited[k]) queue.push(k);
                        }
                    }
                }
            }
            ++n_clusters;
        }

        return n_clusters;
    }

    /**
     * Cluster points into cluster.
     */
    template <typename Point>
    void Cluster(const Array<Point>& points,
                 Array<Array<int>>* clusters) const {
        CHECK(clusters);

        KDTree<Point> kd_tree(points);
        this->Cluster(kd_tree, clusters);
    }

    /**
     * Similar to the previous function, but the input points are stored in
     * KD-tree.
     */
    template <typename Point>
    void Cluster(const KDTree<Point>& kdtree,
                 Array<Array<int>>* clusters) const {
        CHECK(clusters);

        Array<int> labels;
        int n_clusters = Cluster(kdtree, &labels);

        clusters->clear();
        clusters->resize(n_clusters);
        for (int i = 0; i < labels.size(); ++i) {
            if (labels[i] == -1) continue;
            (*clusters)[labels[i]].push_back(i);
        }
    }

private:
    double epsilon_;
    int min_points_;
};

} // namespace clustering
} // namespace cl

#endif // CODELIBRARY_STATISTICS_CLUSTERING_DBSCAN_CLUSTER_H_

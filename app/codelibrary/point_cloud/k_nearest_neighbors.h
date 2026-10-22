//
// Copyright 2018-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_K_NEAREST_NEIGHBORS_H_
#define CODELIBRARY_POINT_CLOUD_K_NEAREST_NEIGHBORS_H_

#include <algorithm>
#include <unordered_set>

#include "codelibrary/graph/graph.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace point_cloud {

/**
 * Compute mutal k-nearest neighbors graph.
 *
 * Two points p_i and p_j are considered neighbors if and only if p_i \in N_j
 * and p_j \in N_i (N_i is k-nearest neighbors of p_i).
 */
template <typename Point>
void MutalKNearestNeighbors(const KDTree<Point>& kd_tree, int k,
                            Array<Array<int>>* neighbors) {
    CHECK(neighbors);
    CHECK(k >= 0);
    CHECK(k <= kd_tree.size());

    int n = kd_tree.size();

    Array<std::unordered_set<int>> graph(n);

    Array<int> tmp(k);
    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(kd_tree.points()[i], k, &tmp);
        for (int j : tmp) {
            if (i != j) graph[i].insert(j);
        }
    }

    neighbors->resize(n);
    for (int i = 0; i < n; ++i) {
        for (int j : graph[i]) {
            if (j <= i) continue;
            if (graph[j].find(i) != graph[j].end()) {
                (*neighbors)[i].push_back(j);
                (*neighbors)[j].push_back(i);
            }
        }
    }
}
template <typename Point>
void MutalKNearestNeighbors(const KDTree<Point>& kd_tree, int k, Graph* graph) {
    CHECK(graph);
    CHECK(k >= 0);
    CHECK(k <= kd_tree.size());

    int n = kd_tree.size();

    Array<std::unordered_set<int>> neighbors(n);

    Array<int> tmp(k);
    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(kd_tree.points()[i], k, &tmp);
        for (int j : tmp) {
            if (i != j) neighbors[i].insert(j);
        }
    }

    graph->Resize(n);
    for (int i = 0; i < n; ++i) {
        for (int j : neighbors[i]) {
            if (j <= i) continue;
            if (neighbors[j].find(i) != neighbors[j].end()) {
                graph->InsertTwoWayEdge(i, j);
            }
        }
    }
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_K_NEAREST_NEIGHBORS_H_

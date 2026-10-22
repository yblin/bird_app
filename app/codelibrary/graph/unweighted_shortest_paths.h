//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_UNWEIGHTED_SHORTEST_PATHS_H_
#define CODELIBRARY_GRAPH_UNWEIGHTED_SHORTEST_PATHS_H_

#include <functional>
#include <queue>
#include <utility>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * Breadth-first search algorithm solves the single-source shortest-paths
 * problem on a unweighted graph.
 *
 * The time complexity is O(E + V).
 *
 * Parameters:
 *  graph        - a directed or undirected graph.
 *  source       - the source vertex.
 *  predecessors - predecessor[i] records the predecessor of the i-th vertex
 *                 in the input graph, or -1 if the predecessor does not exist.
 *  distances    - distances[i] records the shortest distance from source to
 *                 the i-th vertex.
 */
inline void UnweightedShortestPaths(const Graph& graph, int source,
                                    Array<int>* predecessors,
                                    Array<int>* distances) {
    CHECK(predecessors && distances);

    int n = graph.n_vertices();
    CHECK(0 <= source && source < n);

    predecessors->assign(n, -1);
    distances->assign(n, 0);

    using Edge = Graph::Edge;
    using Pair = std::pair<int, int>;

    std::queue<Pair> q;
    q.push(Pair(0, source));

    Array<bool> is_finish(n, false);

    while (!q.empty()) {
        Pair cur = q.front();
        q.pop();
        if (is_finish[cur.second]) continue;
        is_finish[cur.second] = true;

        for (const Edge* e : graph.edges_from(cur.second)) {
            if (e->target() == source || e->source() == e->target()) continue;

            if ((*predecessors)[e->target()] == -1 ||
                (*distances)[e->source()] + 1 < (*distances)[e->target()]) {
                (*distances)[e->target()] = (*distances)[e->source()] + 1;
                (*predecessors)[e->target()] = e->source();
                q.push(Pair((*distances)[e->target()], e->target()));
            }
        }
    }
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_UNWEIGHTED_SHORTEST_PATHS_H_

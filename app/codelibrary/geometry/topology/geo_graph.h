//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_H_

#include <cfloat>
#include <deque>
#include <queue>
#include <set>
#include <unordered_set>
#include <unordered_map>

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/util/flat/append_only_array.h"

namespace cl {
namespace geometry {

/**
 * GeoGraph is a general graph structure to represent a set of vertices, edges,
 * rings, faces, and their relationships.
 *
 * It is more general than Arrangement because:
 *  1. Multi-face Boundaries: Each edge within the GeoGraph can function as the
 *     boundary for multiple faces, allowing for more complex topological
 *     configurations.
 *
 *  2. Overlapping Structures: The structure accommodates overlapping faces or
 *     edges, which is particularly useful in modeling and visualization where
 *     intersections and shared boundaries are common.
 *
 *  3. Arbitrary Edge Arcs: GeoGraph supports edges that are not just straight
 *     lines but arbitrary arcs defined by control points, providing flexibility
 *     in representing curved boundaries between vertices.
 */
template <typename Point>
class GeoGraph {
    using T = typename Point::value_type;

public:
    using VertexIndex = int;
    using EdgeIndex = int;
    using RingIndex = int;
    using FaceIndex = int;

    /**
     * Vertex of the the graph.
     */
    struct Vertex {
        VertexIndex id = -1;
        Point point;
    };

    /**
     * Edge from vertex s to t.
     */
    struct Edge {
        EdgeIndex id = -1;

        // Source and target points of the edge.
        VertexIndex source = -1, target = -1;

        // The incident rings of this edge.
        // It should be ensured that rings[i].edges contains this edge.
        std::set<RingIndex> rings;
    };

    /**
     * A ring is a loop of edges.
     */
    struct Ring {
        // ID of the ring.
        RingIndex id = -1;

        // Each ring belongs to only one face.
        FaceIndex face = -1;

        // The edges of the ring.
        Array<EdgeIndex> edges;
    };

    /**
     * A face is consisted of one outer ring and several inner rings.
     */
    struct Face {
        // ID of the face.
        FaceIndex id = -1;
        RingIndex outer_ring = -1;
        Array<RingIndex> inner_rings;
    };

    GeoGraph() = default;

    void clear() {
        vertex_map_.clear();
        vertices_.clear();
        edges_.clear();
        rings_.clear();
        faces_.clear();
        edges_from_.clear();
        edges_to_.clear();
    }

    /**
     * Insert a vertex at the specified point 'p'.
     *
     * If there is already a vertex defined at the same position 'p', it returns
     * the index of the existing vertex instead of creating a new one.
     */
    VertexIndex InsertVertex(const Point& p) {
        auto it = vertex_map_.find(p);
        if (it != vertex_map_.end()) {
            return it->second;
        }

        Vertex v;
        v.id = vertices_.size();
        v.point = p;
        vertices_.push_back(v);
        edges_from_.push_back(Array<EdgeIndex>());
        edges_to_.push_back(Array<EdgeIndex>());
        vertex_map_[p] = v.id;
        return v.id;
    }

    /**
     * Insert a directed edge from vertex 's' to vertex 't'.
     *
     * If an edge already exists between these two vertices, the function
     * returns the index of the existing edge instead of creating a new one.
     */
    EdgeIndex InsertEdge(VertexIndex s, VertexIndex t) {
        CHECK(vertices_.contain(s));
        CHECK(vertices_.contain(t));
        CHECK(s != t);

        EdgeIndex eid = this->FindEdge(s, t);
        if (eid != -1) return eid;

        Edge e;
        e.id = edges_.size();
        e.source = s;
        e.target = t;
        edges_from_[s].push_back(e.id);
        edges_to_[t].push_back(e.id);
        edges_.push_back(e);

        if (insert_two_way_edge_) {
            EdgeIndex eid = this->FindEdge(t, s);
            if (eid == -1) {
                Edge e;
                e.id = edges_.size();
                e.source = t;
                e.target = s;
                edges_from_[t].push_back(e.id);
                edges_to_[s].push_back(e.id);
                edges_.push_back(e);
            }
        }

        return e.id;
    }

    /**
     * Insert an edge from point 's' to 't'.
     */
    EdgeIndex InsertEdge(const Point& s, const Point& t) {
        VertexIndex v1 = InsertVertex(s);
        VertexIndex v2 = InsertVertex(t);
        return InsertEdge(v1, v2);
    }

    /**
     * Insert a polyline path.
     */
    void InsertPath(const Array<Point>& path,
                    Array<EdgeIndex>* inserted_edges = nullptr) {
        if (inserted_edges) inserted_edges->clear();

        if (path.size() == 1) InsertVertex(path.front());

        for (int i = 0; i + 1 < path.size(); ++i) {
            if (path[i] != path[i + 1]) {
                EdgeIndex e = this->InsertEdge(path[i], path[i + 1]);
                if (inserted_edges) inserted_edges->push_back(e);
            }
        }
    }

    /**
     * Insert a ring represented by a sequence of points.
     *
     * It returns -1 if the input is not a valid ring (has duplicate edges for
     * example).
     */
    RingIndex InsertRing(const Array<Point>& points) {
        if (points.size() < 2) return -1;

        Array<EdgeIndex> edges;
        for (int i = 0; i + 1 < points.size(); ++i) {
            if (points[i] != points[i + 1]) {
                edges.push_back(this->InsertEdge(points[i], points[i + 1]));
            }
        }
        if (points.back() != points.front()) {
            edges.push_back(this->InsertEdge(points.back(), points.front()));
        }
        return InsertRing(edges);
    }

    /**
     * Insert a ring represented by a sequence of points into the graph.
     *
     * This function validates the input ring before insertion to ensure it does
     * not contain duplicate edges or other topological errors.
     *
     * If the validation passes, the ring is added to the graph and its index is
     * returned. Otherwise, it returns -1 without making any modifications to
     * the graph.
     */
    RingIndex InsertRingWithCheck(const Array<Point>& points) {
        if (points.size() < 2) return -1;

        std::unordered_map<Point, int> vertex_id;
        int n_vertices = vertices_.size();
        for (int i = 0; i < points.size(); ++i) {
            auto it = vertex_map_.find(points[i]);
            if (it == vertex_map_.end()) {
                if (vertex_id.find(points[i]) == vertex_id.end()) {
                    vertex_id[points[i]] = n_vertices++;
                }
            } else {
                vertex_id[points[i]] = it->second;
            }
        }
        Array<int> indices(points.size());
        for (int i = 0; i < points.size(); ++i) {
            indices[i] = vertex_id[points[i]];
        }
        indices.push_back(indices.front());

        // Check if the input ring contains duplicate edges.
        std::set<std::pair<int, int>> duplicate_edge_hash;

        for (int i = 0; i + 1 < indices.size(); ++i) {
            if (indices[i] != indices[i + 1]) {
                if (duplicate_edge_hash.find({indices[i], indices[i + 1]}) !=
                    duplicate_edge_hash.end()) {
                    LOG(WARNING) << "The input contains duplicate edges.";
                    return -1;
                }
                duplicate_edge_hash.emplace(indices[i], indices[i + 1]);
            }
        }

        return InsertRing(points);
    }

    /**
     * Insert a polygon face from an array of points.
     */
    FaceIndex InsertFace(const Array<Point>& polygon) {
        if (polygon.size() < 2) return -1;

        Face face;
        face.id = faces_.size();
        face.outer_ring = InsertRing(polygon);
        if (face.outer_ring == -1) return -1;
        rings_[face.outer_ring].face = face.id;
        faces_.push_back(face);
        return face.id;
    }

    /**
     * Insert a polygon face defined by outer and inner rings points.
     *
     * The first input ring is the outer contour, and any subsequent rings will
     * be treated as holes within the polygon.
     */
    FaceIndex InsertFace(const Array<Array<Point>>& polygon) {
        if (polygon.empty()) return -1;

        Face face;
        face.id = faces_.size();
        face.outer_ring = InsertRing(polygon.front());
        if (face.outer_ring == -1) return -1;
        rings_[face.outer_ring].face = face.id;

        for (int i = 1; i < polygon.size(); ++i) {
            RingIndex r = this->InsertRing(polygon[i]);
            if (r == -1) return -1;
            face.inner_rings.push_back(r);
            rings_[r].face = face.id;
        }
        faces_.push_back(face);
        return face.id;
    }

    /**
     * Find a vertex located at the specified position 'p'.
     */
    VertexIndex FindVertex(const Point& p) const {
        auto itr = vertex_map_.find(p);
        return itr == vertex_map_.end() ? -1 : itr->second;
    }

    /**
     * Find the edge (source, target), or return -1 if it does not exist.
     */
    EdgeIndex FindEdge(VertexIndex source, VertexIndex target) const {
        if (!edges_from_.contain(source)) return -1;

        for (EdgeIndex eid : edges_from_[source]) {
            const Edge& e = edges_[eid];
            if (e.target == target) return eid;
        }
        return -1;
    }

    /**
     * Find the edge (source, target), or return -1 if it does not exist.
     */
    EdgeIndex FindEdge(const Point& source, const Point& target) const {
        VertexIndex s = this->FindVertex(source);
        if (s == -1) return -1;
        VertexIndex t = this->FindVertex(target);
        if (t == -1) return -1;

        return FindEdge(s, t);
    }

    /**
     * Find an edge on ring whose source vertex is 'v'.
     */
    EdgeIndex FindStartEdgeOnRing(RingIndex r, VertexIndex v) const {
        CHECK(rings_.contain(r)) << "Ring " << r
                                 << " is not contained in the graph.";

        const Ring& ring = rings_[r];
        for (auto e : ring.edges) {
            if (this->source(e) == v) return e;
        }
        return -1;
    }

    /**
     * Find an edge on ring whose target vertex is 'v'.
     */
    EdgeIndex FindEndEdgeOnRing(RingIndex r, VertexIndex v) const {
        CHECK(rings_.contain(r)) << "Ring " << r
                                 << " is not contained in the graph.";

        const Ring& ring = rings_[r];
        for (auto e : ring.edges) {
            if (this->target(e) == v) return e;
        }
        return -1;
    }

    /**
     * Find the previous edge of the given edge 'e' within the specified ring.
     */
    EdgeIndex PreviousEdgeOnRing(RingIndex r, EdgeIndex e) const {
        CHECK(rings_.contain(r)) << "Ring " << r
                                 << " is not contained in the graph.";

        const Ring& ring = rings_[r];
        for (int i = 0; i < ring.edges.size(); ++i) {
            if (ring.edges[i] == e) {
                return i == 0 ? ring.edges.back() : ring.edges[i - 1];
            }
        }
        return -1;
    }

    /**
     * Find the next edge of the given edge 'e' within the specified ring.
     */
    EdgeIndex NextEdgeOnRing(RingIndex r, EdgeIndex e) const {
        CHECK(rings_.contain(r)) << "Ring " << r
                                 << " is not contained in the graph.";

        const Ring& ring = rings_[r];
        for (int i = 0; i < ring.edges.size(); ++i) {
            if (ring.edges[i] == e) {
                return i + 1 == ring.edges.size() ? ring.edges[0]
                                                  : ring.edges[i + 1];
            }
        }
        return -1;
    }

    /**
     * Find a path consisting of edges on the ring from source vertex to target
     * vertex.
     *
     * If the given source equals to the target, it returns an entire ring
     * starts at 'soruce'.
     *
     * Return false if we cannot find such path.
     */
    bool FindPathOnRing(RingIndex ring_id,
                        VertexIndex source,
                        VertexIndex target,
                        Array<EdgeIndex>* edges) const {
        CHECK(edges);

        edges->clear();

        if (!rings_.contain(ring_id)) return false;

        const auto& ring = rings_[ring_id];
        if (source == target) {
            int start = -1;
            for (int i = 0; i < ring.edges.size(); ++i) {
                EdgeIndex e = ring.edges[i];
                if (start == -1 && edges_[e].source == source) {
                    start = i;
                }

                if (start != -1) {
                    edges->push_back(e);
                }
            }

            if (start == -1) return false;

            for (int i = 0; i < start; ++i) {
                edges->push_back(ring.edges[i]);
            }

            return true;
        }

        EdgeIndex start = -1, end = -1;
        for (int index = 0; index < ring.edges.size(); ++index) {
            EdgeIndex e = ring.edges[index];
            if (this->source(e) == source) {
                start = index;
                break;
            }
        }
        if (start == -1) return false;

        for (int index = 0; index < ring.edges.size(); ++index) {
            EdgeIndex e = ring.edges[index];
            if (this->target(e) == target) {
                end = index;
                break;
            }
        }
        if (end == -1) return false;

        if (start <= end) {
            edges->insert(ring.edges.begin() + start,
                          ring.edges.begin() + end + 1);
        } else {
            edges->insert(ring.edges.begin() + start, ring.edges.end());
            edges->insert(ring.edges.begin(), ring.edges.begin() + end + 1);
        }

        return !edges->empty();
    }

    /**
     * Similar to the previous one but return the polyline.
     */
    bool FindPathOnRing(RingIndex ring_id,
                        VertexIndex source,
                        VertexIndex target,
                        Array<Point>* polyline) const {
        CHECK(polyline);

        polyline->clear();
        Array<EdgeIndex> edges;
        if (!FindPathOnRing(ring_id, source, target, &edges)) return false;

        for (auto e : edges) {
            polyline->push_back(this->source_point(e));
        }
        polyline->push_back(this->target_point(edges.back()));
        return true;
    }

    /**
     * Find the path with minimal length from source to target, or return false
     * if no path can be found from source to target
     */
    bool FindPath(VertexIndex source, VertexIndex target,
                  Array<EdgeIndex>* edges) const {
        CHECK(vertices_.contain(source));
        CHECK(vertices_.contain(target));
        CHECK(edges);

        edges->clear();
        if (source == target) return true;

        using Pair = std::pair<T, VertexIndex>;

        std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> q;
        q.push(Pair(0, source));

        std::unordered_map<VertexIndex, bool> is_finish;
        std::unordered_map<VertexIndex, VertexIndex> predecessors;
        std::unordered_map<VertexIndex, double> distances;

        while (!q.empty()) {
            Pair cur = q.top();
            q.pop();
            if (is_finish[cur.second]) continue;
            is_finish[cur.second] = true;
            if (cur.second == target) break;

            for (auto e : this->edges_from_[cur.second]) {
                VertexIndex e_target = this->target(e);
                VertexIndex e_source = cur.second;
                if (e_target == source || e_source == e_target) continue;

                double w = Distance(this->source_point(e),
                                    this->target_point(e));
                if (predecessors.find(e_target) == predecessors.end() ||
                    distances[e_source] + w < distances[e_target]) {
                    distances[e_target] = distances[e_source] + w;
                    predecessors[e_target] = e_source;
                    q.push(Pair(distances[e_target], e_target));
                }
            }
        }

        if (!is_finish[target]) return false;

        for (VertexIndex v = target; v != source; v = predecessors[v]) {
            VertexIndex s = predecessors[v];
            edges->push_back(this->FindEdge(s, v));
        }
        std::reverse(edges->begin(), edges->end());

        return true;
    }

    /**
     * Replace a path on ring using minimum matching distance between two paths.
     * Return false if we cannot find such path.
     *
     * The implementation is quite complex, as it requires careful consideration
     * of the topology of the graph.
     */
    bool ReplacePath(VertexIndex source, VertexIndex target,
                     const Array<Point>& path) {
        CHECK(source != target);

        std::unordered_set<Point> hash;
        hash.insert(path.front());
        hash.insert(path.back());
        for (int i = 1; i + 1 < path.size(); ++i) {
            if (path[i] != path[i - 1]) {
                CHECK(hash.find(path[i]) == hash.end())
                        << "Error: The input path contains a loop at path "
                        << "vertex " << i << ".";
                hash.insert(path[i]);
            }
        }

        Array<EdgeIndex> edges;
        if (!FindPath(source, target, &edges)) return false;
        return ReplacePath(edges, path);
    }

    /**
     * Find the optimal vertex matching between the graph's path and the given
     * polyline path.
     * This function assumes that the endpoints of path1 are precisely matched
     * to the endpoints of path2.
     *
     * The primary use of this function is within ReplacePath() to align a
     * graph's path with a given polyline, ensuring the best match in terms of
     * vertex correlation.
     */
    bool FindMatch(const Array<VertexIndex>& path1, const Array<Point>& path2,
                   Array<int>* matches) const {
        CHECK(matches);

        matches->assign(path1.size(), -1);

        const int m = path1.size();
        const int n = path2.size();
        CHECK(m < INT_MAX / n) << "The sizes of path1 and path2 are too large.";

        // dp(i, j) means the minimum matching distance of path1[0, ..., i] and
        // path2[0, ..., j].
        ArrayND<double> dp(m, n);
        dp.fill(DBL_MAX);
        dp(0, 0) = 0.0;

        for (int i = 1; i < m; ++i) {
            double d = dp(i - 1, 0);
            for (int j = 0; j < n; ++j) {
                d = std::min(d, dp(i - 1, j));
                double dis = Distance(this->point(path1[i]), path2[j]);
                dp(i, j) = std::min(dp(i, j), d + dis);
            }
        }

        int best = n - 1;
        (*matches)[m - 1] = n - 1;
        for (int i = m - 2; i >= 1; --i) {
            double min = DBL_MAX;
            for (int j = best; j >= 0; --j) {
                if (dp(i, j) < min) {
                    min = dp(i, j);
                    best = j;
                }
            }
            if (min == DBL_MAX) {
                // Cannot found matches.
                return false;
            }
            (*matches)[i] = best;
        }
        (*matches)[0] = 0;

        return true;
    }

    /**
     * Replace a path using minimum matching distance between two paths.
     * Return false if we cannot find such path.
     *
     * The implementation is quite complex, as it requires careful consideration
     * of the topology of the graph.
     */
    bool ReplacePath(const Array<EdgeIndex>& edges, const Array<Point>& path) {
        // Find branch points on the path.
        Array<VertexIndex> branches;
        branches.push_back(this->source(edges.front()));
        for (int i = 1; i < edges.size(); ++i) {
            VertexIndex v = this->source(edges[i]);
            if (IsBranchVertex(v)) branches.push_back(v);
        }
        branches.push_back(this->target(edges.back()));
        std::unordered_set<VertexIndex> is_branch(branches.begin(),
                                                  branches.end());

        Array<int> matches;
        if (!FindMatch(branches, path, &matches)) return false;

        // Remove all edges on the old path.
        Array<std::set<RingIndex>> piece_rings;
        Array<std::set<RingIndex>> twin_piece_rings;
        Array<bool> has_twin;
        Array<EdgeIndex> piece_edges;
        std::unordered_set<RingIndex> rings_need_to_reset;

        for (auto e : edges) {
            piece_edges.push_back(e);
            VertexIndex v = this->target(e);
            if (is_branch.find(v) != is_branch.end()) {
                // We found a piece.
                std::set<RingIndex> rings1 = edges_[e].rings;
                std::set<RingIndex> rings2;
                EdgeIndex twin = this->TwinEdge(e);
                if (twin != -1) {
                    rings2 = edges_[twin].rings;
                }

                piece_rings.push_back(rings1);
                twin_piece_rings.push_back(rings2);
                has_twin.push_back(twin != -1);
                rings_need_to_reset.insert(rings1.begin(), rings1.end());
                rings_need_to_reset.insert(rings2.begin(), rings2.end());

                Array<VertexIndex> remove_vertices;
                for (int i = 0; i + 1 < piece_edges.size(); ++i) {
                    remove_vertices.push_back(this->target(piece_edges[i]));
                }
                for (auto e1 : piece_edges) {
                    CHECK(rings1 == edges_[e1].rings);
                    edges_[e1].rings.clear();

                    EdgeIndex twin_e1 = this->TwinEdge(e1);
                    if (twin_e1 != -1) {
                        CHECK(rings2 == edges_[twin_e1].rings);
                        edges_[twin_e1].rings.clear();
                    }
                    this->EraseEdge(e1);
                    if (twin_e1 != -1)
                        this->EraseEdge(twin_e1);
                }

                for (auto v : remove_vertices) {
                    this->EraseVertex(v);
                }
                piece_edges.clear();
            }
        }
        CHECK(piece_rings.size() + 1 == branches.size());

        if (branches.front() == branches.back()) {
            for (int i = 0; i + 1 < branches.size(); ++i) {
                branches[i] = this->MoveVertex(branches[i], path[matches[i]]);
            }
            branches.back() = branches.front();
        } else {
            for (int i = 0; i < branches.size(); ++i) {
                branches[i] = this->MoveVertex(branches[i], path[matches[i]]);
            }
        }

        // Insert new path.
        for (int i = 0; i + 1 < branches.size(); ++i) {
            CHECK(matches[i] <= matches[i + 1]);
            if (matches[i] == matches[i + 1]) continue;

            Array<Point> points(path.begin() + matches[i],
                                path.begin() + matches[i + 1] + 1);
            for (int j = 0; j + 1 < points.size(); ++j) {
                if (points[j] == points[j + 1]) continue;

                EdgeIndex e1 = this->InsertEdge(points[j], points[j + 1]);
                Edge& e = edges_[e1];
                e.rings.insert(piece_rings[i].begin(), piece_rings[i].end());
                for (RingIndex r : piece_rings[i]) {
                    rings_[r].edges.push_back(e1);
                }
                if (has_twin[i]) {
                    EdgeIndex e2 = this->InsertEdge(points[j + 1], points[j]);
                    Edge& e_twin = edges_[e2];
                    e_twin.rings.insert(twin_piece_rings[i].begin(),
                                        twin_piece_rings[i].end());
                    for (RingIndex r : twin_piece_rings[i]) {
                        rings_[r].edges.push_back(e2);
                    }
                }
            }
        }

        bool success = true;
        for (auto r : rings_need_to_reset) {
            if (!this->ResetRing(r)) {
                success = false;
            }
        }

        return success;
    }

    /**
     * Replace a path on ring using minimum matching distance between two paths.
     * Return false if we cannot find such path.
     *
     * The implementation is quite complex, as it requires careful consideration
     * of the topology of the graph.
     */
    bool ReplacePathOnRing(RingIndex ring_id,
                           VertexIndex source,
                           VertexIndex target,
                           const Array<Point>& path) {
        // Check if the given path is valid?
        std::unordered_set<Point> hash;
        if (source == target) {
            CHECK(path.front() == path.back())
                    << "Error: The source and target vertices are the same, so "
                    << "the input path must form a loop, meaning the first and "
                    << "last vertices should match.";
            CHECK(path.size() >= 3);
            hash.insert(path.front());
        } else {
            CHECK(path.size() >= 2);
            hash.insert(path.front());
            hash.insert(path.back());
        }
        for (int i = 1; i + 1 < path.size(); ++i) {
            if (path[i] != path[i - 1]) {
                if (hash.find(path[i]) != hash.end()) {
                    LOG(ERROR) << "Error: The input path contains a loop at "
                               << "vertex " << i << ".";
                    return false;
                }
                hash.insert(path[i]);
            }
        }

        Array<EdgeIndex> edges;
        if (!FindPathOnRing(ring_id, source, target, &edges)) return false;

        return ReplacePath(edges, path);
    }

    /**
     * Merge vertex v1 into v2.
     */
    void MergeVertices(VertexIndex v1, VertexIndex v2) {
        CHECK(vertices_.contain(v1));
        CHECK(vertices_.contain(v2));

        if (v1 == v2) return;

        for (EdgeIndex e : edges_from_[v1]) {
            Edge& edge = edges_[e];
            if (edge.target == v2) {
                for (auto r : edge.rings) {
                    Ring& ring = rings_[r];
                    ring.edges.erase(e);
                }
                edge.rings.clear();
            } else {
                EdgeIndex e1 = this->InsertEdge(v2, edge.target);
                ReplaceEdge(e, e1);
            }
        }

        for (EdgeIndex e : edges_to_[v1]) {
            Edge& edge = edges_[e];
            if (edge.source == v2) {
                for (auto r : edge.rings) {
                    Ring& ring = rings_[r];
                    ring.edges.erase(e);
                    if (ring.edges.empty()) {
                        this->EraseRing(ring.id);
                    }
                }
                edge.rings.clear();
            } else {
                EdgeIndex e1 = this->InsertEdge(edge.source, v2);
                ReplaceEdge(e, e1);
            }
        }
        EraseVertex(v1);
    }

    /**
     * Merge two adjacent edges. Return the merged edge or -1 if they cannot be
     * merged.
     */
    EdgeIndex MergeEdges(EdgeIndex e1, EdgeIndex e2) {
        CHECK(e1 != e2);
        CHECK(edges_.contain(e1));
        CHECK(edges_.contain(e2));

        Edge& edge1 = edges_[e1];
        Edge& edge2 = edges_[e2];
        CHECK(edge1.target == edge2.source);
        if (edge1.rings != edge2.rings) return -1;

        CHECK(edge1.source != edge2.target);
        EdgeIndex e = this->InsertEdge(edge1.source, edge2.target);
        // Important! The address may be changed.
        Edge& edge3 = edges_[e1];
        Edge& edge4 = edges_[e2];

        for (RingIndex rid : edge3.rings) {
            Ring& ring = rings_[rid];
            auto itr = ring.edges.find(e1);
            CHECK(itr != ring.edges.end())
                    << "Something wrong in the topology of the GeoGraph.";

            *itr = e;
            ring.edges.erase(e2);
        }
        edges_[e].rings.insert(edge3.rings.begin(), edge3.rings.end());

        edge3.rings.clear();
        edge4.rings.clear();

        VertexIndex v = edge3.target;
        if (edges_from_[v].size() == 1 && edges_to_[v].size() == 1)
            this->EraseVertex(v);
        else {
            this->EraseEdge(e1);
            this->EraseEdge(e2);
        }

        return e;
    }

    /**
     * Merge two adjacent edges and their twin edges (if they exist).
     * Return the merged edge or -1 if they cannot be merged.
     */
    EdgeIndex MergePairEdges(EdgeIndex e1, EdgeIndex e2) {
        if (edges_[e1].rings != edges_[e2].rings) return -1;
        EdgeIndex e3 = TwinEdge(e1);
        EdgeIndex e4 = TwinEdge(e2);

        if (e3 != -1 && e4 != -1) {
            if (edges_[e4].rings != edges_[e3].rings) return -1;
            MergeEdges(e4, e3);
        }
        return MergeEdges(e1, e2);
    }

    /**
     * Split edge 'e' with the split point 'p'.
     * Return the new edges.
     */
    std::pair<EdgeIndex, EdgeIndex> SplitEdge(EdgeIndex e, const Point& p) {
        VertexIndex v = this->InsertVertex(p);
        return SplitEdge(e, v);
    }

    /**
     * If there is a twin edge of 'e', split both edges.
     */
    std::pair<EdgeIndex, EdgeIndex> SplitPairEdge(EdgeIndex e, const Point& p) {
        EdgeIndex e1 = this->TwinEdge(e);
        if (e1 != -1) SplitEdge(e1, p);
        return SplitEdge(e, p);
    }

    /**
     * Split edge 'e' into several edges determined by points.
     */
    void SplitEdge(EdgeIndex e, const Array<Point>& points,
                   Array<EdgeIndex>* split_edges = nullptr) {
        CHECK(edges_.contain(e));

        if (points.empty()) return;

        const Edge& edge = edges_[e];
        Array<VertexIndex> vertices;
        vertices.push_back(edge.source);
        for (auto p : points) {
            vertices.push_back(this->InsertVertex(p));
        }
        vertices.push_back(edge.target);
        vertices.erase(std::unique(vertices.begin(), vertices.end()),
                       vertices.end());
        if (vertices.size() <= 2) return;

        // Check if the input points has loop.
        std::unordered_set<VertexIndex> has_duplicate_vertex;
        for (auto v : vertices) {
            CHECK(has_duplicate_vertex.find(v) == has_duplicate_vertex.end())
                    << "Ths input points has loop.";
            has_duplicate_vertex.insert(v);
        }

        Array<EdgeIndex> path;
        for (int i = 0; i + 1 < vertices.size(); ++i) {
            path.push_back(this->InsertEdge(vertices[i], vertices[i + 1]));
        }
        ReplaceEdge(e, path);

        if (split_edges) *split_edges = path;
    }

    /**
     * Split edge 'e' with the split vertex 'v'.
     * Return the new edges.
     */
    std::pair<EdgeIndex, EdgeIndex> SplitEdge(EdgeIndex e, VertexIndex v) {
        CHECK(edges_.contain(e));
        CHECK(vertices_.contain(v));

        // Copy out by VALUE before InsertEdge — InsertEdge() may
        // push_back into edges_ and invalidate any Edge& references.
        const VertexIndex src = edges_[e].source;
        const VertexIndex tgt = edges_[e].target;
        CHECK(v != src);
        CHECK(v != tgt);

        EdgeIndex e1 = this->InsertEdge(src, v);
        EdgeIndex e2 = this->InsertEdge(v, tgt);
        CHECK(e1 != e2);

        Array<EdgeIndex> path = {e1, e2};
        ReplaceEdge(e, path);
        return std::pair<EdgeIndex, EdgeIndex>(e1, e2);
    }

    /**
     * If there is a twin edge of 'e', split both edges.
     */
    void SplitPairEdge(EdgeIndex e, const Array<Point>& points,
                       Array<EdgeIndex>* split_edges = nullptr) {
        EdgeIndex e1 = this->TwinEdge(e);
        if (e1 != -1) {
            Array<Point> reverse_path = points;
            std::reverse(reverse_path.begin(), reverse_path.end());
            SplitEdge(e1, reverse_path);
        }
        SplitEdge(e, points, split_edges);
    }

    /**
     * Move a vertex to a new poisition. This operator is complex if 'p' is
     * already a vertex.
     */
    VertexIndex MoveVertex(VertexIndex v, const Point& p) {
        CHECK(vertices_.contain(v));

        VertexIndex v1 = this->FindVertex(p);
        if (v1 != -1) {
            MergeVertices(v, v1);
            return v1;
        } else {
            vertex_map_.erase(vertices_[v].point);
            vertices_[v].point = p;
            vertex_map_[p] = v;
        }

        return v;
    }

    /**
     * Erase an vertex.
     */
    void EraseVertex(VertexIndex v) {
        CHECK(vertices_.contain(v));

        Array<EdgeIndex> edges = edges_from_[v];
        edges.insert(edges_to_[v]);
        for (EdgeIndex e : edges) {
            EraseEdge(e);
        }
        vertex_map_.erase(vertices_[v].point);
        vertices_.erase(v);
        edges_from_.erase(v);
        edges_to_.erase(v);
    }

    /**
     * Erase an edge.
     */
    void EraseEdge(EdgeIndex eid) {
        CHECK(edges_.contain(eid));

        auto& e = edges_[eid];
        auto rings = e.rings;
        for (auto rid : rings) {
            EraseRing(rid);
        }
        edges_from_[e.source].erase(eid);
        edges_to_[e.target].erase(eid);
        edges_.erase(eid);
    }

    /**
     * Erase a ring.
     */
    void EraseRing(RingIndex rid) {
        CHECK(rings_.contain(rid));

        Ring& r = rings_[rid];
        if (r.face != -1) {
            Face& face = faces_[r.face];
            if (face.outer_ring == rid) {
                this->EraseFace(r.face);
            } else {
                face.inner_rings.erase(rid);
            }
        }

        for (auto eid : r.edges) {
            auto& e = edges_[eid];
            e.rings.erase(rid);
        }
        rings_.erase(rid);
    }

    /**
     * Erase a face.
     */
    void EraseFace(FaceIndex fid) {
        CHECK(faces_.contain(fid));

        Face& f = faces_[fid];
        CHECK(f.outer_ring != -1);

        Ring& outer_ring = rings_[f.outer_ring];
        outer_ring.face = -1;

        for (auto& ring_id : f.inner_rings) {
            Ring& inner_ring = rings_[ring_id];
            inner_ring.face = -1;
        }

        faces_.erase(fid);
    }

    /**
     * Get all faces containing 'e'.
     */
    void GetFacesOfEdge(EdgeIndex e, Array<FaceIndex>* faces) const {
        CHECK(edges_.contain(e));
        CHECK(faces);

        faces->clear();
        for (RingIndex r : rings_of_edge(e)) {
            faces->push_back(this->ring(r).face);
        }
    }

    /**
     * Get vertices of the face 'f'.
     */
    void GetFaceVertices(FaceIndex f,
                         Array<Array<VertexIndex>>* vertices) const {
        CHECK(faces_.contain(f));
        CHECK(faces_[f].outer_ring != -1);
        CHECK(vertices);

        vertices->clear();
        Array<VertexIndex> vs;
        const Ring& ring = rings_[faces_[f].outer_ring];
        for (auto e : ring.edges) {
            const Edge& edge = edges_[e];
            vs.push_back(edge.source);
        }
        vertices->push_back(vs);

        for (auto r : faces_[f].inner_rings) {
            const Ring& ring = rings_[r];
            Array<VertexIndex> vs;
            for (auto e : ring.edges) {
                const Edge& edge = edges_[e];
                vs.push_back(edge.source);
            }
            vertices->push_back(vs);
        }
    }

    /**
     * Get adjacent vertices of 'v'.
     */
    void GetNeighborVertices(VertexIndex v,
                             Array<VertexIndex>* neighbors) const {
        CHECK(vertices_.contain(v));
        CHECK(neighbors);

        neighbors->clear();
        for (auto e : edges_from_[v]) {
            neighbors->push_back(this->target(e));
        }
        for (auto e : edges_to_[v]) {
            neighbors->push_back(this->source(e));
        }
        std::sort(neighbors->begin(), neighbors->end());
        neighbors->erase(std::unique(neighbors->begin(), neighbors->end()),
                         neighbors->end());
    }

    /**
     * Get ring edge from 'v'.
     */
    EdgeIndex GetRingEdgeFrom(RingIndex ring, VertexIndex v) const {
        CHECK(rings_.contain(ring));
        CHECK(vertices_.contain(v));

        for (auto e : edges_from_[v]) {
            if (edges_[e].rings.count(ring)) {
                return e;
            }
        }
        return -1;
    }

    /**
     * Get ring edge to 'v'.
     */
    EdgeIndex GetRingEdgeTo(RingIndex ring, VertexIndex v) const {
        CHECK(rings_.contain(ring));
        CHECK(vertices_.contain(v));

        for (auto e : edges_to_[v]) {
            if (edges_[e].rings.count(ring)) {
                return e;
            }
        }
        return -1;
    }

    /**
     * Get ring of graph.
     */
    bool GetRing(RingIndex ring_id, Array<Point>* polyline) const {
        CHECK(polyline);
        CHECK(rings_.contain(ring_id));

        polyline->clear();
        const Ring& ring = this->ring(ring_id);
        if (ring.edges.empty()) return false;

        for (auto edge : ring.edges) {
            polyline->push_back(this->source_point(edge));
        }
        polyline->push_back(this->target_point(ring.edges.back()));
        return true;
    }

    /**
     * Get twin edge if it has.
     */
    EdgeIndex TwinEdge(EdgeIndex e) const {
        CHECK(edges_.contain(e));

        return this->FindEdge(edges_[e].target, edges_[e].source);
    }

    /**
     * A vertex is a branch vertex if it has at least three edges connected to
     * it (not counting twin edges).
     */
    bool IsBranchVertex(VertexIndex v) const {
        CHECK(vertices_.contain(v));

        Array<VertexIndex> neighbors;
        neighbors.reserve(3);
        for (EdgeIndex e : edges_to_[v]) {
            VertexIndex v1 = this->source(e);
            if (neighbors.find(v1) == neighbors.end()) {
                neighbors.push_back(v1);
                if (neighbors.size() > 2) return true;
            }
        }
        for (EdgeIndex e : edges_from_[v]) {
            VertexIndex v1 = this->target(e);
            if (neighbors.find(v1) == neighbors.end()) {
                neighbors.push_back(v1);
                if (neighbors.size() > 2) return true;
            }
        }

        return false;
    }

    /**
     * Get the chain containing edge 'e'.
     * A chain of GeoGraph is defined as a path or a cycle where all vertices
     * except the head and tail vertices are not branch vertices.
     *
     * Note that every edges in GeoGraph has a chain that contains it.
     */
    void GetChain(EdgeIndex e, Array<EdgeIndex>* chain) const {
        CHECK(contain_edge(e));
        CHECK(chain);

        std::deque<int> dequeue;

        EdgeIndex e1 = e;
        bool is_cycle = false;
        while (true) {
            dequeue.push_back(e1);
            VertexIndex u = this->source(e1);
            VertexIndex v = this->target(e1);

            if (IsBranchVertex(v)) break;

            bool found = false;
            for (auto e2 : this->edges_from(v)) {
                if (this->target(e2) != u) {
                    e1 = e2;
                    found = true;
                    break;
                }
            }
            if (!found) break;
            if (e1 == e) {
                is_cycle = true;
                break;
            }
        }

        if (is_cycle) {
            *chain = Array<EdgeIndex>(dequeue.begin(), dequeue.end());
            return;
        }

        dequeue.pop_front();
        e1 = e;
        while (true) {
            dequeue.push_front(e1);
            VertexIndex u = this->source(e1);
            VertexIndex v = this->target(e1);

            if (IsBranchVertex(u)) break;

            bool found = false;
            for (auto e2 : this->edges_to(u)) {
                if (this->source(e2) != v) {
                    e1 = e2;
                    found = true;
                    break;
                }
            }
            if (!found || e1 == e) break;
        }

        *chain = Array<EdgeIndex>(dequeue.begin(), dequeue.end());
    }

    /**
     * Get all chains.
     *
     * Note that for a complete undirected graph, this function will obtain
     * all twin chains.
     *
     * See GetChian() for more details.
     */
    void GetChains(Array<Array<EdgeIndex>>* chains) const {
        CHECK(chains);

        chains->clear();
        Array<bool> visited(this->n_allocated_edges(), false);
        Array<EdgeIndex> chain;
        for (auto e : this->edges()) {
            if (visited[e.id]) continue;
            this->GetChain(e.id, &chain);
            for (auto e1 : chain) {
                visited[e1] = true;
            }
            chains->push_back(chain);
        }
    }

    /**
     * Get all chains.
     *
     * See GetChian() for more details.
     */
    void GetChains(Array<Array<Point>>* chains) const {
        CHECK(chains);

        Array<Array<EdgeIndex>> chains_edges;
        GetChains(&chains_edges);

        chains->clear();
        for (const auto& edges : chains_edges) {
            Array<Point> chain;
            for (EdgeIndex e : edges) {
                if (chain.empty()) {
                    chain.push_back(source_point(e));
                }
                chain.push_back(target_point(e));
            }
            chains->push_back(chain);
        }
    }

    // O(1) member functions.
    bool empty()     const { return edges_.empty();          }
    int n_vertices() const { return vertices_.active_size(); }
    int n_edges()    const { return edges_.active_size();    }
    int n_rings()    const { return rings_.active_size();    }
    int n_faces()    const { return faces_.active_size();    }

    // The next generated ID for elements.
    int n_allocated_vertices() const { return vertices_.size(); }
    int n_allocated_edges()    const { return edges_.size();    }
    int n_allocated_rings()    const { return rings_.size();    }
    int n_allocated_faces()    const { return faces_.size();    }

    bool contain_vertex(VertexIndex vertex) const {
        return vertices_.contain(vertex);
    }

    bool contain_edge(EdgeIndex edge) const {
        return edges_.contain(edge);
    }

    bool contain_ring(RingIndex ring) const {
        return rings_.contain(ring);
    }

    bool contain_face(FaceIndex face) const {
        return faces_.contain(face);
    }

    const Vertex& vertex(VertexIndex v) const {
        CHECK(vertices_.contain(v));

        return vertices_[v];
    }

    const Edge& edge(EdgeIndex e) const {
        CHECK(edges_.contain(e));

        return edges_[e];
    }

    const Ring& ring(RingIndex r) const {
        CHECK(rings_.contain(r));

        return rings_[r];
    }

    const Face& face(FaceIndex f) const {
        CHECK(faces_.contain(f));

        return faces_[f];
    }

    const Point& point(VertexIndex v) const {
        CHECK(vertices_.contain(v));

        return vertices_[v].point;
    }

    const Point& source_point(EdgeIndex e) const {
        CHECK(edges_.contain(e));

        return vertices_[edges_[e].source].point;
    }

    const Point& target_point(EdgeIndex e) const {
        CHECK(edges_.contain(e)) << e;

        return vertices_[edges_[e].target].point;
    }

    const Point& source_point(const Edge& e) const {
        CHECK(vertices_.contain(e.source));

        return vertices_[e.source].point;
    }

    const Point& target_point(const Edge& e) const {
        CHECK(vertices_.contain(e.target)) << e.id;

        return vertices_[e.target].point;
    }

    VertexIndex source(EdgeIndex e) const {
        CHECK(edges_.contain(e)) << "Edge " << e << " is not in the graph.";

        return edges_[e].source;
    }

    VertexIndex target(EdgeIndex e) const {
        CHECK(edges_.contain(e));

        return edges_[e].target;
    }

    const AppendOnlyArray<Vertex>& vertices() const {
        return vertices_;
    }

    const AppendOnlyArray<Edge>& edges() const {
        return edges_;
    }

    const AppendOnlyArray<Ring>& rings() const {
        return rings_;
    }

    const AppendOnlyArray<Face>& faces() const {
        return faces_;
    }

    const Array<EdgeIndex>& edges_from(VertexIndex v) const {
        CHECK(vertices_.contain(v));

        return edges_from_[v];
    }

    const Array<EdgeIndex>& edges_to(VertexIndex v) const {
        CHECK(vertices_.contain(v));

        return edges_to_[v];
    }

    /**
     * Return all rings contain the given edge.
     */
    const std::set<RingIndex>& rings_of_edge(EdgeIndex e) const {
        CHECK(edges_.contain(e));

        return edges_[e].rings;
    }

    /**
     * Define the behavior of edge insertion.
     */
     void set_insert_two_way_edge(bool flag) {
         insert_two_way_edge_ = flag;
     }

protected:
    /**
     * Insert a ring defined by a sequence of edges or return -1 if there is an
     * error.
     */
    RingIndex InsertRing(const Array<EdgeIndex>& edges) {
        if (edges.empty()) return -1;

        std::unordered_set<EdgeIndex> seen;
        for (const auto& edge : edges) {
            if (!seen.insert(edge).second) {
                LOG(ERROR) << "Error: The input contains duplicate edges: "
                           << edge << ".";
                return -1;
            }
        }

        Ring ring;
        ring.id = rings_.size();
        ring.edges = edges;
        rings_.push_back(ring);

        for (EdgeIndex eid : edges) {
            Edge& e = edges_[eid];
            e.rings.insert(ring.id);
        }

        return ring.id;
    }

    /**
     * Use edge e2 to replace edge e1. Internal use only.
     */
    void ReplaceEdge(EdgeIndex e1, EdgeIndex e2) {
        CHECK(e1 != e2);

        Edge& edge1 = edges_[e1];
        Edge& edge2 = edges_[e2];
        for (RingIndex r : edge1.rings) {
            Ring& ring = rings_[r];
            auto itr = ring.edges.find(e1);
            CHECK(itr != ring.edges.end());
            CHECK(edge2.rings.find(ring.id) == edge2.rings.end());
            *itr = e2;
        }
        edge2.rings.insert(edge1.rings.begin(), edge1.rings.end());
        edge1.rings.clear();
    }

    /**
     * Replace edge 'e' by edges. Internal use only.
     */
    void ReplaceEdge(EdgeIndex e, const Array<EdgeIndex>& edges) {
        CHECK(!edges.empty());

        Edge& edge = edges_[e];
        auto rings = edge.rings;
        edge.rings.clear();
        for (auto e : edges) {
            Edge& edge = edges_[e];
            for (auto r : rings) {
                CHECK(edge.rings.find(r) == edge.rings.end());
                edge.rings.insert(r);
            }
        }

        for (RingIndex rid : rings) {
            Ring& ring = rings_[rid];
            auto it = ring.edges.find(e);
            CHECK(it != ring.edges.end());

            *it = edges.front();
            ++it;
            ring.edges.insert(it, edges.begin() + 1, edges.end());
        }

        this->EraseEdge(e);
    }

    /**
     * Reset the ring to correct the topology.
     */
    bool ResetRing(RingIndex r) {
        CHECK(rings_.contain(r));

        Ring& ring = rings_[r];

        // Refine ring by euler cycle algorithm.
        EdgeIndex start_e = -1;
        for (auto e : ring.edges) {
            if (edges_.contain(e)) {
                start_e = e;
                break;
            }
        }
        if (start_e == -1) return false;

        // Need to ensure the ring do not have duplicate edges.
        std::unordered_set<EdgeIndex> filter_edge;
        std::unordered_map<VertexIndex, Array<EdgeIndex>> edge_lists;
        for (auto e : ring.edges) {
            if (edges_.contain(e) && filter_edge.find(e) == filter_edge.end()) {
                filter_edge.insert(e);
                edge_lists[this->source(e)].push_back(e);
                edges_[e].rings.erase(r);
            }
        }

        Array<EdgeIndex> cycle;
        Array<VertexIndex> stack;
        stack.push_back(this->source(start_e));

        while (!stack.empty()) {
            VertexIndex cur = stack.back();
            Array<EdgeIndex>& s = edge_lists[cur];

            if (!s.empty()) {
                EdgeIndex e1 = s.back();
                s.pop_back();
                VertexIndex u = this->target(e1);

                // Move to the next vertex.
                stack.push_back(u);
            } else {
                // Backtrack and add edge to euler_cycle
                stack.pop_back();
                if (!stack.empty()) {
                    VertexIndex u = stack.back();
                    EdgeIndex e = this->FindEdge(u, cur);
                    CHECK(e != -1);
                    cycle.push_back(e);
                }
            }
        }

        std::reverse(cycle.begin(), cycle.end());
        ring.edges = cycle;
        CHECK(!cycle.empty());
        CHECK(edges_[cycle.front()].source == edges_[cycle.back()].target);

        for (auto e : ring.edges) {
            edges_[e].rings.insert(r);
        }
        return true;
    }

    // Map each unique point to its corresponding vertex index.
    std::unordered_map<Point, VertexIndex> vertex_map_;

    // Vertices list.
    // We use AppendOnlyArray instead of IndexedList because we do not want to
    // reuse IDs.
    AppendOnlyArray<Vertex> vertices_;

    // Edge list.
    AppendOnlyArray<Edge> edges_;

    // Ring list of the graph.
    AppendOnlyArray<Ring> rings_;

    // Face list of the graph.
    AppendOnlyArray<Face> faces_;

    // edges_from_[i] stores all edges starting from the i-th vertex.
    AppendOnlyArray<Array<EdgeIndex>> edges_from_;

    // edge_to_[i] stores all edges that end at the i-th vertex.
    AppendOnlyArray<Array<EdgeIndex>> edges_to_;

    // By default, InsertEdge() inserts one-way edges.
    // If insert_two_way_edge_ is set to true, InsertEdge() will insert twin
    // edges (bidirectional).
    bool insert_two_way_edge_ = false;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_H_

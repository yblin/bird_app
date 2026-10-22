//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_UTIL_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_UTIL_H_

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/topology/geo_graph.h"
#include "codelibrary/geometry/topology/polygon_topology_fix_2d.h"

namespace cl {
namespace geometry {
namespace geo_graph_util {

using VertexIndex = int;
using EdgeIndex = int;
using RingIndex = int;
using FaceIndex = int;

/**
 * Find the edge closest to 'p' on the given ring of the graph.
 */
template <typename T>
EdgeIndex FindEdgeOnRing(const GeoGraph<Point2D<T>>& graph, RingIndex ring_id,
                         const Point2D<T>& p) {
    const auto& ring = graph.ring(ring_id);

    double dis = DBL_MAX;
    EdgeIndex found_edge = -1;

    for (EdgeIndex e : ring.edges) {
        Segment2D<T> seg(graph.source_point(e), graph.target_point(e));
        double d = Distance(seg, p);
        if (d < dis) {
            dis = d;
            found_edge = e;
        }
    }
    return found_edge;
}

/**
 * Extract connected edge chains from a set of edges in a graph, where each
 * chain consists of sequentially connected edges.
 */
template <typename Point>
bool GetEdgeChains(const GeoGraph<Point>& graph, const Array<int>& edges,
                   Array<Array<int>>* chains) {
    CHECK(chains);

    chains->clear();
    if (edges.empty()) return false;

    std::unordered_map<int, int> next;
    std::unordered_map<int, int> next_edge;
    std::unordered_set<int> vertices;
    std::unordered_map<int, int> in_degree;
    std::unordered_map<int, int> out_degree;

    for (int e : edges) {
        int s = graph.source(e);
        int t = graph.target(e);
        if (next.count(s)) return false;
        next[s] = t;
        next_edge[s] = e;
        in_degree[t]++;
        out_degree[s]++;
        vertices.insert(s);
        vertices.insert(t);
    }

    std::unordered_set<int> visited;
    for (int v : vertices) {
        if (visited.count(v)) continue;

        if (in_degree[v] == 0) {
            Array<int> chain;
            int current = v;
            visited.insert(current);
            while (true) {
                chain.push_back(next_edge[current]);
                current = next[current];
                visited.insert(current);
                if (out_degree[current] == 0) {
                    break;
                }
            }
            chains->push_back(chain);
        }
    }

    for (int v : vertices) {
        if (visited.count(v)) continue;

        Array<int> chain;
        int current = v;
        do {
            chain.push_back(next_edge[current]);
            visited.insert(current);
            current = next[current];
            if (visited.count(current)) break;
        } while (current != v);
        chains->push_back(chain);
    }
    return true;
}

/**
 * Similar to the previous one, but return the polylines.
 */
template <typename Point>
bool GetEdgeChains(const GeoGraph<Point>& graph, const Array<int>& edges,
                   Array<Polyline<Point>>* polylines) {
    CHECK(polylines);

    polylines->clear();

    Array<Array<int>> chains;
    if (!GetEdgeChains(graph, edges, &chains)) return false;
    for (auto& chain : chains) {
        Polyline<Point> polyline;
        for (int e : chain) {
            if (polyline.empty()) {
                polyline.push_back(graph.source_point(e));
            }
            polyline.push_back(graph.target_point(e));
        }
        polylines->push_back(polyline);
    }
    return true;
}

/**
 * Retrieve the polygon corresponding to a specified ring in the GeoGraph.
 *
 * Note: This function does not validate the topological of the output polygon.
 * The resulting polygon may contain self-intersections due to the original
 * ring's structure.
 */
template <typename T>
void GetRingPolygon(const GeoGraph<Point2D<T>>& graph, RingIndex ring_id,
                    Polygon2D<T>* polygon) {
    CHECK(polygon);
    CHECK(graph.contain_ring(ring_id));

    auto ring = graph.ring(ring_id);

    Array<Point2D<T>> points;
    for (auto edge : ring.edges) {
        points.push_back(graph.source_point(edge));
    }
    *polygon = Polygon2D<T>(points);
}

/**
 * This function constructs a multi-polygon for the face by combining its outer
 * ring and inner rings (holes), using GetRingPolygon() to extract each ring's
 * geometry. It does not perform any topological validation, so the result
 * inherits potential issues (e.g., self-intersections) from the underlying
 * rings.
 */
template <typename T>
void GetRawFacePolygon(const GeoGraph<Point2D<T>>& graph, FaceIndex face_id,
                       MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    CHECK(graph.contain_face(face_id));

    polygon->clear();
    auto face = graph.face(face_id);
    auto r = graph.ring(face.outer_ring);
    Polygon2D<T> poly;
    GetRingPolygon(graph, r.id, &poly);

    if (poly.empty()) return;
    polygon->Insert(poly, true);

    for (auto r : face.inner_rings) {
        GetRingPolygon(graph, r, &poly);
        if (!poly.empty())
            polygon->Insert(poly, false);
    }
}

/**
 * Retrieve a topologically valid polygon representation of a specified face in
 * a GeoGraph.
 *
 * The raw face polygon (from GetRawFacePolygon()) may contain
 * self-intersections or other topological errors. This function processes the
 * raw polygon using FixEvenOddPolygon() (from polygon_topology_fix_2d.h) to
 * resolve such issues, producing a collection of valid, non-self-intersecting
 * multi-polygon-with-holes.
 *
 * Note: This function is significantly slower than GetRawFacePolygon() due to
 * the topological fixing step. Use it only when strict geometric validity is
 * required.
 */
template <typename T>
void GetFacePolygon(const GeoGraph<Point2D<T>>& graph, FaceIndex face_id,
                    Array<MultiPolygon2D<T>>* polygons) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(polygons);

    MultiPolygon2D<T> raw_polygon;
    GetRawFacePolygon(graph, face_id, &raw_polygon);
    FixEvenOddPolygon(raw_polygon, polygons);
}

} // namespace geo_graph_util
} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_GEO_GRAPH_UTIL_H_

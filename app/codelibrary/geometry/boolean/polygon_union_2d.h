//
// Copyright 2022-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_

#include <algorithm>
#include <queue>

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace geometry {

/**
 * 2D polygon set union with snapping. It differs from PolygonBoolean that the
 * later one only support merge two polygons at a time. But this one can merge a
 * set of polygons, which is much faster.
 *
 * A suitable snap threshold is important for maintaining the topology of
 * polygons. Otherwise in some cases the inner contour may exceed the outer
 * contour causing topology errors.
 *
 * This class depends on Arrangment2D and Snap2D.
 */
template <typename T>
class PolygonUnion2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point   = Point2D<T>;
    using Polygon = MultiPolygon2D<T>;
    using DCEL    = HalfedgeList<Point>;
    using Vertex  = typename DCEL::Vertex;
    using Halfedge = typename DCEL::Halfedge;
    using Region = typename Arrangement2D<T>::Region;
    using TriFace = typename Arrangement2D<T>::TriFace;
    using EdgePropertyInt = typename DCEL::template HalfedgeProperty<int>;

    // We merge 30 polygons at once to speed up.
    static constexpr int BLOCK_SIZE = 30;

public:
    PolygonUnion2D() = default;

    PolygonUnion2D(const PolygonUnion2D&) = delete;

    PolygonUnion2D& operator=(const PolygonUnion2D&) = delete;

    /**
     * Union of polygon set with snap threshold.
     *
     * A suitable snap threshold is important for maintaining the topology of
     * polygons. Otherwise in some case the inner contour may exceed the outer
     * contour causing topology errors.
     */
    void operator()(const Array<Polygon>& polygons, double threshold,
                    Polygon* result) const {
        CHECK(result);
        CHECK(threshold > 0.0);

        result->clear();
        if (polygons.empty()) return;

        Array<Array<Polygon>> clusters;
        this->GroupClusters(polygons, threshold, &clusters);

        for (const Array<Polygon>& cluster : clusters) {
            Polygon t;
            if (cluster.size() <= BLOCK_SIZE) {
                SubProcess(cluster, threshold, &t);
            } else {
                MainProcess(cluster, threshold, &t);
            }
            for (auto& b : t) {
                result->Insert(b.polygon, b.is_outer);
            }
        }
    }

    /**
     * Similar to the previous one but return a set of polygon with holes.
     */
    void operator()(const Array<Polygon>& polygons, double threshold,
                    Array<Polygon>* results) const {
        CHECK(results);
        CHECK(results != &polygons);

        results->clear();

        Polygon poly;
        this->operator()(polygons, threshold, &poly);

        for (const auto& b : poly.boundaries()) {
            if (b.is_outer) {
                results->emplace_back(b.polygon);
            } else {
                CHECK(!results->empty());
                results->back().Insert(b.polygon, false);
            }
        }
    }

protected:
    /**
     * Return true if the arrangement region is a inner region.
     */
    static bool IsInner(const Arrangement2D<T>& arrangement,
                        const Region& region,
                        const EdgePropertyInt& color) {
        for (const TriFace* face : region) {
            Halfedge* e = face->halfedge;
            if (arrangement.is_constraint(e))
                return color[e] == 0;
            if (arrangement.is_constraint(e->next()))
                return color[e->next()] == 0;
            if (arrangement.is_constraint(e->prev()))
                return color[e->prev()] == 0;
        }
        return false;
    }

    /**
     * Main process. It unions the subset of polygons with snapping.
     */
    void MainProcess(const Array<Polygon>& polygons, double threshold,
                     Polygon* result) const {
        if (polygons.empty()) return;
        if (polygons.size() == 1) {
            *result = polygons.front();
            return;
        }

        Array<Polygon> results;
        for (int i = 0; i < polygons.size(); i += BLOCK_SIZE) {
            int end = std::min(i + BLOCK_SIZE, polygons.size());
            Array<Polygon> sub_polygons(polygons.begin() + i,
                                        polygons.begin() + end);
            Polygon t;
            SubProcess(sub_polygons, threshold, &t);
            if (!t.empty()) results.push_back(t);
        }

        // Recursive.
        MainProcess(results, threshold, result);
    }

    /**
     * Group the intersected polygons into the same cluster.
     */
    void GroupClusters(const Array<Polygon>& polygons, double threshold,
                       Array<Array<Polygon>>* clusters) const {
        CHECK(clusters);

        Array<Box2D<T>> boxes;
        for (int i = 0; i < polygons.size(); ++i) {
            const Box2D<T>& b = polygons[i].bounding_box();
            if (b.empty()) {
                boxes.push_back(b);
            } else {
                boxes.emplace_back(b.x_min() - threshold,
                                   b.x_max() + threshold,
                                   b.y_min() - threshold,
                                   b.y_max() + threshold);
            }
        }

        RTree2D<T> rtree;
        rtree.Construct(boxes);
        DisjointSet set(polygons.size());
        Array<int> entries;
        for (int i = 0; i < polygons.size(); ++i) {
            rtree.Find(boxes[i], &entries);
            for (int j : entries) {
                if (Intersect(boxes[i], boxes[j])) {
                    set.Union(i, j);
                }
            }
        }

        Array<Array<int>> cs;
        set.ToClusters(&cs);
        clusters->resize(cs.size());
        for (int i = 0; i < cs.size(); ++i) {
            for (int v : cs[i]) {
                (*clusters)[i].push_back(polygons[v]);
            }
        }
    }

    /**
     * Compute the arrangement of polygons with snapping.
     */
    void GetArrangement(const Array<Polygon>& polygons, double threshold,
                        Arrangement2D<T>* arrangement) const {
        CHECK(polygons.size() <= BLOCK_SIZE);
        CHECK(arrangement);

        using Line = typename Arrangement2D<T>::Line;
        Array<Line> lines;
        int id = 0;
        for (const Polygon& poly : polygons) {
            for (const auto& b : poly.boundaries()) {
                bool is_clockwise = b.polygon.IsClockwise();
                for (int j = 0; j < b.polygon.size(); ++j) {
                    if (b.is_outer != is_clockwise) {
                        lines.emplace_back(b.polygon.vertex(j),
                                           b.polygon.next_vertex(j),
                                           1 << id, 0);
                    } else {
                        lines.emplace_back(b.polygon.next_vertex(j),
                                           b.polygon.vertex(j),
                                           1 << id, 0);
                    }
                }
            }
            ++id;
        }
        arrangement->set_threshold(threshold);
        arrangement->Reset(lines);

        // Some polygon edges may duplicate due to the snap. These edges need to
        // be filtered out.
        for (Halfedge* e : arrangement->mesh()) {
            int t = arrangement->color(e) & arrangement->color(e->twin());
            int c1 = arrangement->color(e) - t;
            int c2 = arrangement->color(e->twin()) - t;
            if (c1 == 0 && c2 == 0)
                arrangement->UnsetConstraintEdge(e);
            else {
                arrangement->set_color(e, c1);
                arrangement->set_color(e->twin(), c2);
            }
        }
    }

    /**
     * Sub process. It unions the subset of polygons with snapping.
     */
    void SubProcess(const Array<Polygon>& polygons, double threshold,
                    Polygon* result) const {
        if (polygons.size() == 1) {
            *result = polygons[0];
            return;
        }

        Arrangement2D<T> arrangement;
        GetArrangement(polygons, threshold, &arrangement);

        const DCEL& list = arrangement.mesh();
        if (list.empty()) return;

        // Step 1. Initialize colors of each halfedge of arrangement regions.
        arrangement.Arrange();
        const auto& regions = arrangement.regions();

        auto color = list.AddHalfedgeProperty(0);
        for (Halfedge* e : list) {
            color[e] = arrangement.color(e);
        }

        // Step 2. Propagate color between regions.
        Array<int> region_colors(regions.size(), 0);
        for (int i = 0; i < regions.size(); ++i) {
            const Region& region = regions[i];
            for (const TriFace* face : region) {
                Halfedge* e = face->halfedge;
                region_colors[i] |= (color[e] | color[e->next()] |
                                     color[e->prev()]);
            }
        }

        std::queue<int> q;
        for (int i = 0; i < regions.size(); ++i) {
            q.push(i);
        }
        Array<bool> in_queue(regions.size(), true);
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            in_queue[cur] = false;
            const Region& region = regions[cur];
            int c = region_colors[cur];

            for (TriFace* face : region) {
                Halfedge* edges[3];
                edges[0] = face->halfedge;
                edges[1] = edges[0]->next();
                edges[2] = edges[0]->prev();
                for (Halfedge* e : edges) {
                    if ((color[e] & c) != color[e]) continue;

                    int r1 = arrangement.region_id(e->twin());
                    if (r1 == -1) continue;

                    int new_color = (region_colors[r1] | (c - color[e]));
                    if (region_colors[r1] != new_color) {
                        region_colors[r1] = new_color;
                        if (!in_queue[r1]) {
                            q.push(r1);
                            in_queue[r1] = true;
                        }
                    }
                }
            }
        }

        // Step 3. Remove useless halfedges.
        for (Halfedge* e : list) {
            if (arrangement.is_constraint(e)) {
                int r1 = arrangement.region_id(e);
                color[e] = (r1 != -1) ? region_colors[r1] : 0;
                int r2 = arrangement.region_id(e->twin());
                color[e->twin()] = (r2 != -1) ? region_colors[r2] : 0;
                if (color[e] != 0 && color[e->twin()] != 0) {
                    arrangement.UnsetConstraintEdge(e);
                }
            }
        }

        // Step 4. Get result.
        arrangement.Arrange();

        Polygon poly;
        for (int i = 0; i < arrangement.regions().size(); ++i) {
            // Filter inner region.
            if (IsInner(arrangement, arrangement.region(i), color)) continue;

            arrangement.GetRegion(i, &poly);
            for (const auto& b : poly) {
                result->Insert(b.polygon, b.is_outer);
            }
        }
    }
};

/**
 * Compute the union of multiple 2D polygons and retains only their outer
 * contours, removing all inner holes. The resulting multi-polygon consists of
 * disjoint outer contours with no inner holes.
 *
 * Compared to PolygonUnion2D, this function is faster, due to the simplified
 * handling of inner boundaries.
 */
template <typename T>
void ExternalPolygonUnion2D(const Array<MultiPolygon2D<T>>& polygons,
                            double threshold,
                            MultiPolygon2D<T>* result) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(result);
    CHECK(threshold >= 0.0);

    using Line = typename Arrangement2D<T>::Line;
    Array<Line> lines;
    for (const auto& poly : polygons) {
        for (const auto& b : poly.boundaries()) {
            for (int j = 0; j < b.polygon.size(); ++j) {
                lines.emplace_back(b.polygon.vertex(j),
                                   b.polygon.next_vertex(j),
                                   1,
                                   1);
            }
        }
    }

    Arrangement2D<T> arrangement(threshold);
    arrangement.Reset(lines);
    arrangement.Arrange();

    // Remove useless halfedges.
    for (auto e : arrangement.mesh()) {
        if (arrangement.is_constraint(e)) {
            int r1 = arrangement.region_id(e);
            int r2 = arrangement.region_id(e->twin());
            if (r1 != -1 && r2 != -1) {
                arrangement.UnsetConstraintEdge(e);
            }
        }
    }

    // Get result.
    arrangement.Arrange();

    MultiPolygon2D<T> poly;
    for (int i = 0; i < arrangement.regions().size(); ++i) {
        arrangement.GetRegion(i, &poly);
        for (const auto& b : poly) {
            CHECK(b.is_outer);
            result->Insert(b.polygon, b.is_outer);
        }
    }
}
template <typename T>
void ExternalPolygonUnion2D(const Array<MultiPolygon2D<T>>& polygons,
                            MultiPolygon2D<T>* result) {
    ExternalPolygonUnion2D(polygons, 0.0, result);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_

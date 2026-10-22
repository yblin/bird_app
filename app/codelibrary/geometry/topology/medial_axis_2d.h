//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_MEDIAL_AXIS_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_MEDIAL_AXIS_2D_H_

#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/mesh/polygon_triangulation_2d.h"
#include "codelibrary/geometry/topology/geo_graph.h"
#include "codelibrary/geometry/transform_2d.h"

namespace cl {
namespace geometry {

/**
 * Interface for the 2D Medial Axis.
 *
 * The medial axis of a 2D object is defined as the set of all points within the
 * object that have more than one closest point on the object's boundary.
 */
template <typename T>
class MedialAxis2D {
    using Point = Point2D<T>;
    using Branch = Polyline<Point>;

public:
    MedialAxis2D() {
        axis_.set_insert_two_way_edge(true);
    }

    void clear() {
        axis_.clear();
        support_boundary_vertices_.clear();
        non_prunable_vertices_.clear();
    }

    /**
     * Prune branches from the medial axis that have fewer edges than the
     * specified threshold.
     * This operation modifies the internal medial axis structure.
     */
    void PruneByEdgeCount(int min_edge_count) {
        CHECK(min_edge_count >= 0);

        if (min_edge_count <= 1) return;

        std::queue<std::pair<int, int>> queue;
        Array<bool> in_queue(axis_.n_allocated_vertices(), false);
        for (auto v : axis_.vertices()) {
            if (axis_.edges_from(v.id).size() == 1) {
                queue.emplace(v.id, 0);
                in_queue[v.id] = true;
            }
        }

        Array<int> removed_branches;
        while (!queue.empty()) {
            std::pair<int, int> cur = queue.front();
            queue.pop();
            if (cur.second >= min_edge_count) continue;

            CHECK(axis_.edges_from(cur.first).size() <= 2);
            for (int e : axis_.edges_from(cur.first)) {
                int v = axis_.target(e);
                if (in_queue[v]) continue;
                if (axis_.edges_from(v).size() > 2) {
                    removed_branches.push_back(axis_.TwinEdge(e));
                    break;
                }
                in_queue[v] = true;
                queue.emplace(v, cur.second + 1);
            }
        }

        for (auto e : removed_branches) {
            // Remove branch starting from 'e'.
            Array<int> edges;
            std::queue<int> q;
            q.push(e);
            edges.push_back(e);

            while (!q.empty()) {
                int e1 = q.front();
                q.pop();

                int s = axis_.source(e1);
                int t = axis_.target(e1);

                CHECK(axis_.edges_from(t).size() <= 2);
                for (auto e2 : axis_.edges_from(t)) {
                    if (axis_.target(e2) == s) continue;
                    edges.push_back(e2);
                    q.push(e2);
                }
            }
            std::reverse(edges.begin(), edges.end());
            for (auto e : edges) {
                axis_.EraseVertex(axis_.target(e));
            }
        }
    }

    /**
     * Prune vertices from the medial axis by modified bending potential ratio
     * (BPR).
     *
     * Reference:
     *   Shen, Wei, et al. "Skeleton growing and pruning with bending potential
     *   ratio." Pattern Recognition 44.2 (2011): 196-209.
     */
    void PruneBPR(double bpr, double min_cut_distance = 0.0) {
        CHECK(bpr > 0.0);
        CHECK(min_cut_distance >= 0.0);

        Array<Array<double>> prev_length(polygon_.n_boundaries());
        for (int i = 0; i < polygon_.n_boundaries(); ++i) {
            auto& b = polygon_.boundaries()[i].polygon;
            prev_length[i].push_back(0.0);
            for (int j = 1; j < b.size(); ++j) {
                double l = Distance(b[j], b[j - 1]) + prev_length[i].back();
                prev_length[i].push_back(l);
            }
            double l = Distance(b.vertices().front(), b.vertices().back());
            prev_length[i].push_back(prev_length[i].back() + l);
        }

        Array<bool> can_prune(axis_.n_allocated_vertices(), false);
        for (auto v : axis_.vertices()) {
            if (non_prunable_vertices_.find(v.id) !=
                non_prunable_vertices_.end()) {
                continue;
            }

            auto& support = support_boundary_vertices_[v.id];
            if (support.size() < 2) {
                can_prune[v.id] = true;
                continue;
            }

            int boundary_id = support.front().first;
            const RPolygon2D& polygon =
                    polygon_.boundaries()[boundary_id].polygon;
            bool fixed = false;
            for (auto& p : support) {
                if (p.first != boundary_id) {
                    fixed = true;
                    break;
                }
            }
            if (fixed) continue;

            std::sort(support.begin(), support.end());
            double max_length = 0.0;
            double cut_length = 0.0;
            for (int i = 0; i < support.size(); ++i) {
                int a = support[i].second;
                int b = i + 1 == support.size() ? support[0].second
                                                : support[i + 1].second;
                double l = prev_length[boundary_id][b] -
                           prev_length[boundary_id][a];
                l = std::min(prev_length[boundary_id].back() - l, l);
                if (l > max_length) {
                    max_length = l;
                    cut_length = Distance(polygon[a], polygon[b]);
                }
            }

            if (cut_length <= min_cut_distance ||
                max_length / cut_length < bpr) {
                can_prune[v.id] = true;
            }
        }

        std::unordered_set<int> availables;
        for (auto v : axis_.vertices()) {
           if (can_prune[v.id] && axis_.edges_from(v.id).size() == 1)
               availables.insert(v.id);
        }

        while (!availables.empty()) {
            int v = *availables.begin();
            availables.erase(availables.begin());
            if (axis_.edges_from(v).empty()) continue;

            int e = axis_.edges_from(v).front();
            int v1 = axis_.target(e);
            if (can_prune[v1] && axis_.edges_from(v1).size() == 2) {
                availables.insert(v1);
            }
            axis_.EraseVertex(v);
        }
    }

    /**
     * Decompose the medial axis graph into multiple branches, where each branch
     * stores the vertex indices within the medial axis.
     */
    void ToBranches(Array<Array<int>>* branches) const {
        CHECK(branches);

        branches->clear();

        Array<Array<int>> chains;
        axis_.GetChains(&chains);
        Array<bool> visited(axis_.n_allocated_edges(), false);
        for (Array<int>& c : chains) {
            if (visited[c.front()]) continue;

            Array<int> branch;
            for (int c1 : c) {
                visited[c1] = true;
                visited[axis_.TwinEdge(c1)] = true;
                if (branch.empty()) {
                    branch.push_back(axis_.source(c1));
                }
                branch.push_back(axis_.target(c1));
            }
            branches->push_back(branch);
        }
    }

    /**
     * Get both branch vertices and branch polylines.
     */
    void ToBranches(Array<Array<int>>* branch_vertices,
                    Array<Branch>* branches) const {
        CHECK(branch_vertices);
        CHECK(branches);

        this->ToBranches(branch_vertices);
        branches->resize(branch_vertices->size());
        for (int i = 0; i < branch_vertices->size(); ++i) {
            auto& vertices = (*branch_vertices)[i];
            (*branches)[i].resize(vertices.size());
            auto& branch = (*branches)[i];
            for (int j = 0; j < vertices.size(); ++j) {
                branch[j] = axis_.point(vertices[j]);
            }
        }
    }

    /**
     * Decompose medial axis graph into polyline branches.
     */
    void ToBranches(Array<Branch>* branches) const {
        CHECK(branches);

        Array<Array<int>> branch_vertices;
        ToBranches(&branch_vertices, branches);
    }

    /**
     * Extract the primary centerline branch from medial axis, merging other
     * branches that meet the angle threshold.
     *
     * For many applications (e.g., road branch centerlines), only a single
     * primary centerline branch is required. This function identifies the most
     * significant branch as the primary one, then merges other branches into it
     * if they form an angle within the specified threshold relative to the
     * primary branch's direction. The result is a unified primary branch.
     *
     * Return false if we cannot found such primary branch.
     */
    bool GetPrimaryBranch(double angle_threshold,
                          Array<int>* main_branch_vertices,
                          Branch* main_branch) const {
        CHECK(angle_threshold > 0.0);

        Array<Array<int>> branch_vertices;
        Array<Branch> branches;
        ToBranches(&branch_vertices, &branches);

        int main_index = -1;
        double max_area = 0.0;
        for (int i = 0; i < branch_vertices.size(); ++i) {
            auto& b = branches[i];
            Array<double> widths(b.size());
            for (int j = 0; j < b.size(); ++j) {
                widths[j] = 2.0 * this->radius(branch_vertices[i][j]);
            }
            double total_area = 0.0;
            for (int j = 0; j + 1 < b.size(); ++j) {
                double area = (widths[j] + widths[j + 1]) * 0.5 *
                              Distance(b[j], b[j + 1]);
                total_area += area;
            }
            if (total_area > max_area) {
                max_area = total_area;
                main_index = i;
            }
        }
        if (main_index == -1) return false;

        *main_branch = branches[main_index];
        *main_branch_vertices = branch_vertices[main_index];
        CHECK(main_branch->size() == main_branch_vertices->size());
        if (main_branch->front() == main_branch->back()) {
            // The main branch is a ring.
            return true;
        }
        if (branches.size() == 1) return true;

        branches.erase(branches.begin() + main_index);
        branch_vertices.erase(branch_vertices.begin() + main_index);
        bool finish = false;
        while (!finish) {
            finish = true;
            for (int i = 0; i < branches.size(); ++i) {
                Branch& b = branches[i];
                Array<int>& b_vertices = branch_vertices[i];

                if (b.front() == main_branch->front()) {
                    Vector2D<T> v1 = b[0] - b[1];
                    Vector2D<T> v2 = (*main_branch)[1] - (*main_branch)[0];
                    if (Degree(v1, v2) < angle_threshold) {
                        main_branch->insert(main_branch->begin(),
                                            b.rbegin() + 1,
                                            b.rend());
                        main_branch_vertices->insert(
                                    main_branch_vertices->begin(),
                                    b_vertices.rbegin() + 1,
                                    b_vertices.rend());
                        finish = false;
                    }
                } else if (b.front() == main_branch->back()) {
                    Vector2D<T> v1 = b[1] - b[0];
                    Vector2D<T> v2 = main_branch->back() -
                                     (*main_branch)[main_branch->size() - 2];
                    if (Degree(v1, v2) < angle_threshold) {
                        main_branch->insert(main_branch->end(), b.begin() + 1,
                                            b.end());
                        main_branch_vertices->insert(
                                    main_branch_vertices->end(),
                                    b_vertices.begin() + 1,
                                    b_vertices.end());
                        finish = false;
                    }
                } else if (b.back() == main_branch->front()) {
                    Vector2D<T> v1 = b.back() - b[b.size() - 2];
                    Vector2D<T> v2 = (*main_branch)[1] - (*main_branch)[0];
                    if (Degree(v1, v2) < angle_threshold) {
                        main_branch->insert(main_branch->begin(), b.begin() + 1,
                                            b.end());
                        main_branch_vertices->insert(
                                    main_branch_vertices->begin(),
                                    b_vertices.begin() + 1,
                                    b_vertices.end());
                        finish = false;
                    }
                } else if (b.back() == main_branch->back()) {
                    Vector2D<T> v1 = b.back() - b[b.size() - 2];
                    Vector2D<T> v2 = (*main_branch)[main_branch->size() - 2] -
                                     main_branch->back();
                    if (Degree(v1, v2) < angle_threshold) {
                        main_branch->insert(main_branch->end(), b.rbegin() + 1,
                                            b.rend());
                        main_branch_vertices->insert(
                                    main_branch_vertices->end(),
                                    b_vertices.rbegin() + 1,
                                    b_vertices.rend());
                        finish = false;
                    }
                }

                if (!finish) {
                    CHECK(main_branch->size() == main_branch_vertices->size());
                    branches.erase(branches.begin() + i);
                    branch_vertices.erase(branch_vertices.begin() + i);
                    break;
                }
            }
        }

        return true;
    }

    /**
     * Get the radius of vertex 'v' according to the
     * 'support_boundary_vertices'.
     */
    double radius(int v) const {
        CHECK(axis_.contain_vertex(v));

        double w = DBL_MAX;
        Point p = axis_.point(v);
        for (const auto& s : support_boundary_vertices_[v]) {
            const Point& q = polygon_.boundaries()[s.first].polygon[s.second];
            double d = Distance(p, q);
            w = std::min(w, d);
        }
        return w;
    }

    /**
     * Return the boundary vertices that support the vertex 'v'.
     */
    void get_support_vertices(int v, Array<Point>* vertices) const {
        CHECK(axis_.contain_vertex(v));
        CHECK(vertices);

        vertices->clear();
        for (const auto& s : support_boundary_vertices_[v]) {
            vertices->push_back(polygon_.boundaries()[s.first].
                                polygon[s.second]);
        }
    }

    /**
     * The computed 2D medial axis, represented as a geometric graph (GeoGraph)
     * where nodes and edges consist of 2D points.
     */
    const GeoGraph<Point>& axis() const {
        return axis_;
    }

    /**
     * Return the reference 2D polygon from which the medial axis is extracted.
     */
    const MultiPolygon2D<T>& polygon() const {
        return polygon_;
    }

    /**
     * Return the boundary vertices that support each vertex of the medial axis.
     */
    const Array<Array<std::pair<int, int>>>& support_boundary_vertices() const {
        return support_boundary_vertices_;
    }

    /**
     * Return the set of non prunable vertices.
     */
    const std::unordered_set<int>& non_prunable_vertices() const {
        return non_prunable_vertices_;
    }

protected:
    // The geometric graph representation of the computed 2D medial axis.
    GeoGraph<Point> axis_;

    // Vertices that cannot be pruned.
    std::unordered_set<int> non_prunable_vertices_;

    // The reference 2D polygon from which the medial axis is extracted.
    MultiPolygon2D<T> polygon_;

    // The boundary vertices that support each vertex of the medial axis, i.e.,
    // a set of points on the polygon contour that are the closest to i-th axis'
    // vertex.
    Array<Array<std::pair<int, int>>> support_boundary_vertices_;
};

/**
 * Triangulation axis is an alternative skeletal structure for a simple polygon
 * 'P'. This axis is a straight-line graph that can be interpreted as an
 * anisotropic medial axis of P, where inscribed disks are line segments or
 * triangles.
 *
 * It is particularly useful for some special applications, such as the center
 * line of road calculation.
 *
 * Reference:
 *   Aigner, Wolfgang, Franz Aurenhammer, and Bert Jüttler. "On triangulation
 *   axes of polygons." Information Processing Letters 115.1 (2015): 45-51.
 */
template <typename T>
class TriangulationAxis2D : public MedialAxis2D<T> {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;
    using Segment = Segment2D<T>;

public:
    TriangulationAxis2D() = default;

    void clear() {
        MedialAxis2D<T>::clear();
        point_to_boundary_.clear();
    }

    /**
     * Compute the triangulation axis of the given polygon.
     *
     * Parametes:
     *  polygon  - The target polygon for triangulation axis calculation.
     *  cutlines - The edge set of the current polygon that act as cutlines
     *             (edges connecting to other polygons). When computing the
     *             medial axis, the endpoints of the axis will be placed on
     *             these cutlines to achieve seamless connection with the medial
     *             axes of adjacent polygons.
     *
     * Return true if triangulation axis construction succeeds, false otherwise.
     */
    bool Construct(const MultiPolygon2D<T>& in_polygon,
                   const std::set<Segment>& in_cutlines) {
        // Replace every run of consecutive cutlines by a single straight
        // cutline from the run's first endpoint to its last one. This removes
        // the comb-like forks that a chain of cutlines otherwise creates in the
        // medial axis: each run now contributes exactly one connection node.
        MultiPolygon2D<T> polygon;
        std::set<Segment> cutlines;
        MergeConsecutiveCutlines(in_polygon, in_cutlines, &polygon, &cutlines);

        this->polygon_ = polygon;
        this->support_boundary_vertices_.clear();
        point_to_boundary_.clear();
        this->axis_.clear();
        this->axis_.set_insert_two_way_edge(true);
        PolygonTriangulation2D<T> triangulation;
        if (!triangulation.ResetPolygon(polygon))
            return false;

        // Map from boundary point to the index.
        point_to_boundary_.reserve(triangulation.mesh().n_vertices());
        for (int i = 0; i < polygon.n_boundaries(); ++i) {
            const auto& b = polygon.boundaries()[i].polygon;
            for (int j = 0; j < b.size(); ++j) {
                point_to_boundary_[b[j]] = std::pair<int, int>(i, j);
            }
        }

        // Compute the center point of each edge.
        // We do not process ear triangle here.
        using Halfedge = typename PolygonTriangulation2D<T>::Halfedge;

        std::unordered_set<const Halfedge*> border_set;
        for (auto e : triangulation.mesh()) {
            if (triangulation.is_outer(e)) continue;

            Segment edge(e->source_point(), e->target_point());
            if (triangulation.is_constraint(e) &&
                cutlines.find(edge) == cutlines.end()) border_set.insert(e);
        }

        std::unordered_map<const Halfedge*, Point2D<T>> edge_center;
        std::unordered_set<const Halfedge*> is_junction_edge;
        for (const Halfedge* e : triangulation.mesh()) {
            if (triangulation.is_constraint(e)) continue;
            if (edge_center.find(e) != edge_center.end()) continue;
            const Halfedge* e1 = e->next();
            if (edge_center.find(e1) != edge_center.end()) continue;
            const Halfedge* e2 = e->prev();
            if (edge_center.find(e2) != edge_center.end()) continue;

            int c0 = border_set.find(e) != border_set.end();
            int c1 = border_set.find(e1) != border_set.end();
            int c2 = border_set.find(e2) != border_set.end();
            if (c0 + c1 + c2 >= 2) continue;

            if (c0 + c1 + c2 == 0) {
                Point p = Circumcenter(e->source_point(), e->target_point(),
                                       e1->target_point());
                Triangle2D<T> triangle(e->source_point(), e->target_point(),
                                       e1->target_point());
                if (!Intersect(p, triangle)) {
                    p = Project(p, triangle);
                }

                edge_center[e] = p;
                edge_center[e1] = p;
                edge_center[e2] = p;
                is_junction_edge.insert(e);
                is_junction_edge.insert(e1);
                is_junction_edge.insert(e2);

                Array<Point> qs = {e->source_point(), e->target_point(),
                                   e1->target_point()};
                PartitionBoundary(p, qs);
            } else if (c0) {
                Point p1 = Center(e1->source_point(), e1->target_point());
                Point p2 = Center(e2->source_point(), e2->target_point());
                if (edge_center.find(e1) == edge_center.end()) {
                    edge_center[e1] = p1;
                    Array<Point> qs1 = {e1->source_point(), e1->target_point()};
                    PartitionBoundary(p1, qs1);
                }
                if (edge_center.find(e2) == edge_center.end()) {
                    edge_center[e2] = p2;
                    Array<Point> qs2 = {e2->source_point(), e2->target_point()};
                    PartitionBoundary(p2, qs2);
                }

                if (p1 != p2) this->axis_.InsertEdge(p1, p2);
            } else if (c1) {
                Point p = Center(e->source_point(), e->target_point());
                Point p2 = Center(e2->source_point(), e2->target_point());
                edge_center[e] = p;
                Array<Point> qs1 = {e->source_point(), e->target_point()};
                PartitionBoundary(p, qs1);

                if (edge_center.find(e2) == edge_center.end()) {
                    edge_center[e2] = p2;
                    Array<Point> qs2 = {e2->source_point(), e2->target_point()};
                    PartitionBoundary(p2, qs2);
                }

                if (p != p2) this->axis_.InsertEdge(p, p2);
            } else if (c2) {
                Point p = Center(e->source_point(), e->target_point());
                Point p1 = Center(e1->source_point(), e1->target_point());
                edge_center[e] = p;
                Array<Point> qs1 = {e->source_point(), e->target_point()};
                PartitionBoundary(p, qs1);

                if (edge_center.find(e1) == edge_center.end()) {
                    edge_center[e1] = p1;
                    Array<Point> qs2 = {e1->source_point(), e1->target_point()};
                    PartitionBoundary(p1, qs2);
                }

                if (p != p1) this->axis_.InsertEdge(p, p1);
            }
        }

        // Connect each junction point to its neighbor triangles.
        for (auto e : is_junction_edge) {
            if (edge_center.find(e->twin()) != edge_center.end()) {
                if (edge_center[e->twin()] == edge_center[e]) continue;
                this->axis_.InsertEdge(edge_center[e->twin()], edge_center[e]);
            }
        }

        for (auto q : edge_center) {
            Segment edge(q.first->source_point(), q.first->target_point());
            if (cutlines.find(edge) != cutlines.end()) {
                int v = this->axis_.FindVertex(q.second);
                if (v != -1) this->non_prunable_vertices_.insert(v);
            }
        }

        return true;
    }

    /**
     * Compute the triangulation axis of the given polygon.
     *
     * Return true if triangulation axis construction succeeds, false otherwise.
     */
    bool Construct(const MultiPolygon2D<T>& polygon) {
        std::set<Segment> cutlines;
        return Construct(polygon, cutlines);
    }

private:
    /**
     * Replace every run of consecutive cutlines by a single straight cutline.
     *
     * A maximal chain of cutline edges along a boundary (which typically spans
     * a shared border between two neighbour polygons) is collapsed: all its
     * interior vertices are dropped and one new cutline is added from the
     * chain's first endpoint to its last endpoint. Non-cutline edges are left
     * untouched. This guarantees each shared border contributes a single
     * connection node to the medial axis instead of a comb of forks.
     *
     * The collapse is only performed when the merged chord does not cross the
     * ring's own remaining boundary edges. For non-convex rings such a chord
     * would otherwise introduce a self-intersection, and the original cutline
     * edges of that run are kept instead.
     */
    void MergeConsecutiveCutlines(const MultiPolygon2D<T>& in_polygon,
                                  const std::set<Segment>& in_cutlines,
                                  MultiPolygon2D<T>* out_polygon,
                                  std::set<Segment>* out_cutlines) const {
        out_polygon->clear();
        out_cutlines->clear();

        for (const auto& boundary : in_polygon.boundaries()) {
            const Polygon2D<T>& ring = boundary.polygon;
            const int m = ring.size();
            if (m == 0) continue;

            // Classify each boundary edge (ring[i] -> ring[i+1]) as cutline.
            Array<bool> is_cut(m, false);
            int n_cut = 0;
            for (int i = 0; i < m; ++i) {
                Segment e(ring[i], ring.next_vertex(i));
                if (in_cutlines.find(e) != in_cutlines.end()) {
                    is_cut[i] = true;
                    ++n_cut;
                }
            }

            if (n_cut == 0 || n_cut == m) {
                // Nothing to merge, or a degenerate all-cutline ring: keep it.
                Array<Point> vertices;
                for (int i = 0; i < m; ++i) {
                    vertices.push_back(ring[i]);
                    if (is_cut[i]) {
                        out_cutlines->insert(Segment(ring[i],
                                                     ring.next_vertex(i)));
                    }
                }
                Polygon2D<T> poly(vertices);
                if (!poly.empty()) out_polygon->Insert(poly, boundary.is_outer);
                continue;
            }

            // Start on a non-cutline edge so no run wraps across the seam.
            int start = 0;
            while (is_cut[start]) ++start;

            Array<Point> vertices;
            for (int k = 0; k < m; ) {
                if (!is_cut[(start + k) % m]) {
                    vertices.push_back(ring[(start + k) % m]);
                    ++k;
                } else {
                    // Maximal run of consecutive cutline edges: k .. j-1.
                    int j = k;
                    while (j < m && is_cut[(start + j) % m]) ++j;

                    const int run_front = (start + k) % m;
                    const int run_back  = (start + j) % m;
                    const Segment chord(ring[run_front], ring[run_back]);

                    // Merging a run into a single chord cuts across the ring
                    // interior. For a non-convex ring the chord may cross the
                    // ring's own remaining boundary edges, which would make the
                    // ring self-intersecting. Only merge when the chord does
                    // not properly cross any non-run edge; otherwise keep the
                    // original cutline edges so the ring stays simple.
                    bool can_merge = true;
                    if (j > k + 1) {
                        for (int t = 0; t < m; ++t) {
                            // Skip the run's own edges.
                            const bool in_run =
                                    (run_front < run_back) ?
                                        (t >= run_front && t < run_back) :
                                        (t >= run_front || t < run_back);
                            if (in_run) continue;
                            if (Cross(chord, Segment(ring[t],
                                                     ring.next_vertex(t)))) {
                                can_merge = false;
                                break;
                            }
                        }
                    }

                    if (can_merge) {
                        vertices.push_back(ring[run_front]);
                        out_cutlines->insert(chord);
                    } else {
                        // Preserve the original cutline edges of this run.
                        for (int e = k; e < j; ++e) {
                            const int idx = (start + e) % m;
                            vertices.push_back(ring[idx]);
                            out_cutlines->insert(Segment(ring[idx],
                                                         ring.next_vertex(idx)));
                        }
                    }
                    k = j;   // run_back is emitted by the next iteration.
                }
            }

            Polygon2D<T> poly(vertices);
            if (!poly.empty()) out_polygon->Insert(poly, boundary.is_outer);
        }
    }

    /**
     * Assign boundary vertices to 'p'.
     */
    void PartitionBoundary(const Point& p,
                           const Array<Point>& boundary_points) {
        int v = this->axis_.InsertVertex(p);
        if (v >= this->support_boundary_vertices_.size()) {
            CHECK(this->support_boundary_vertices_.size() == v);

            Array<std::pair<int, int>> support_boundaries;
            for (const Point& q : boundary_points) {
                support_boundaries.push_back(point_to_boundary_.at(q));
            }
            this->support_boundary_vertices_.push_back(support_boundaries);
        }
    }

    // Map from boundary point to the index.
    std::unordered_map<Point, std::pair<int, int>> point_to_boundary_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_MEDIAL_AXIS_2D_H_

//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_H_

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/point_compare_2d.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/topology/medial_axis_2d.h"
#include "codelibrary/gis/road_system/road_branch_connector.h"
#include "codelibrary/gis/road_system/road_branch_segmenter.h"
#include "codelibrary/graph/dijkstra_shortest_paths.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace gis {

/**
 * Extract centerline from road polygons.
 */
class RoadCenterline {
    using SingleConnection = RoadBranchConnector::SingleConnection;
    using PairConnection = RoadBranchConnector::PairConnection;
    using BranchWithJunctions = RoadBranchConnector::BranchWithJunctions;

    /**
     * The array of junctions located in the same intersection.
     */
    struct JunctionGroup {
        // Stable junctions.
        Array<int> junctions;

        // Unstable junctions.
        Array<int> unstable_junctions;

        // Topology valid or not.
        bool topology_valid = true;
    };

    /**
     * Graph that organizes centerline connections related to junctions,
     * containing three types of edges: junction-junction, junction-anchor, and
     * anchor-anchor.
     */
    struct JunctionGraph {
        Array<std::pair<int, int>> junction_pairs;
        Array<std::pair<int, int>> junction_anchor_pairs;
        Array<std::pair<int, int>> anchor_pairs;

        JunctionGraph() = default;

        void clear() {
            junction_pairs.clear();
            junction_anchor_pairs.clear();
            anchor_pairs.clear();
        }

        /**
         * Insert a junction-junction edge into graph.
         */
        void AddJunctionPair(int junction1, int junction2) {
            if (junction1 > junction2) {
                std::swap(junction1, junction2);
            }
            junction_pairs.emplace_back(junction1, junction2);
        }

        /**
         * Insert an anchor-anchor edge into graph.
         */
        void AddAnchorPair(int anchor1, int anchor2) {
            if (anchor1 > anchor2) {
                std::swap(anchor1, anchor2);
            }
            anchor_pairs.emplace_back(anchor1, anchor2);
        }

        /**
         * Insert a junction-anchor edge into graph.
         */
        void AddJunctionAnchorPair(int junction, int anchor) {
            junction_anchor_pairs.emplace_back(junction, anchor);
        }
    };

public:
    /**
     * Anchor vertex is the important vertex in the centerline graph.
     */
    struct AnchorVertex {
        // Index of this anchor in the raw medial_axis.
        int id;

        // Position of the anchor.
        RPoint2D point;

        // Width of this vertex.
        double width;

        // Check if this anchor is valid or not.
        bool invalid = false;

        // This anchor is fixed or not.
        bool fixed = false;

        // Score of this anchor.
        double score = 1e100;

        // Adjacent junctions.
        std::set<int> adjacent_junctions;

        // Adjacent anchors.
        std::set<int> adjacent_anchors;

        // True if this anchor point is a cross point of two pair-connections.
        bool is_cross_point = false;

        bool operator <(const AnchorVertex& rhs) const {
            return score < rhs.score;
        }
    };

    /**
     * Decompose the centerline into branch structures to optimize the
     * centerline at road intersections.
     */
    struct Branch {
        // Indices of the branch's vertices in the raw axis graph.
        Array<int> raw_vertices;

        // 2D polyline representing the raw branch, which is identical to
        // the raw medial axis of the road polygon.
        RPolyline2D raw_polyline;

        // Width for each vertex of raw_polyline.
        Array<double> raw_width;

        // Simplified version of the branch's raw polyline.
        RPolyline2D simplified_raw_polyline;

        // Core polyline of the branch, excluding backoff-related parts.
        RPolyline2D polyline;

        // Non-junction part of core polyline.
        // When combined with connection lines, it forms the complete road
        // centerline after OptimizeIntersection() is called.
        RPolyline2D non_junction_polyline;

        // Front backoff polyline, connecting the front end of the core polyline
        // to the front end of the raw_polyline.
        RPolyline2D front_backoff_polyline;

        // Back backoff polyline, connecting the back end of the core polyline
        // to the back end of the raw_polyline.
        RPolyline2D back_backoff_polyline;

        // The branch width at the front of branch polyline.
        double front_width = 0.0;

        // The branch width at the back of branch polyline.
        double back_width = 0.0;

        // Backoff scale to shrink branch for road intersection optimization.
        double backoff_scale = 1.5;

        // If true, indicates this branch cannot be ignored during road
        // intersection optimization.
        bool can_not_ignore = false;

        /**
         * Check if this branch is a stable or not.
         */
        bool is_stable() const {
            return polyline.size() >= 2;
        }
    };

    /**
     * Connection lines at the road intersection.
     */
    struct Connection {       
        RSegment2D line;
        double width;

        Connection() = default;

        Connection(const RSegment2D& l, double w)
            : line(l), width(w) {}

        bool operator== (const Connection& connection) const {
            return line == connection.line;
        }

        bool operator <(const Connection& connection) const {
            return line < connection.line;
        }
    };

    /**
     * Initialize the road centerline extractor with a specified spatial
     * resolution, which controls the densification of road polygon boundaries
     * and influences the detail level of the resulting medial axis.
     */
    explicit RoadCenterline(double resolution = 1.0)
        : resolution_(resolution), branch_connector_(resolution) {
        CHECK(resolution_ > 0.0);
    }

    void clear() {
        medial_axis_.clear();
        branches_.clear();
        junction_groups_.clear();
        anchors_.clear();
        anchor_paths_.clear();
        connections_.clear();
        branch_connector_.clear();
        bad_connections_.clear();
    }

    /**
     * Extract the raw centerline from the given road surface.
     */
    void ExtractRaw(const RMultiPolygon2D& road) {
        this->clear();

        original_road_surface_ = road;

        // Densify the input road polygon.
        road_surface_.clear();
        for (auto& b : road.boundaries()) {
            RPolyline2D poly(b.polygon.vertices());
            poly.push_back(poly.front());
            RPolyline2D densified;
            geometry::polyline_util::Densify(poly, resolution_, &densified);
            RPolygon2D polygon(densified);
            CHECK(!polygon.empty());
            road_surface_.Insert(polygon, b.is_outer);
        }

        medial_axis_.Construct(road_surface_);
        medial_axis_.PruneBPR(3.0, 2.0 * resolution_);

        GetRawBranches();
    }

    /**
     * Optimize the centerlines at road intersections.
     *
     * Raw centerlines typically perform poorly at road intersections. This
     * function optimizes the topology of centerlines at intersections based on
     * the raw centerline, enhancing the quality of centerline results at these
     * junction areas.
     *
     * The final optimized results are primarily stored in the 'polyline' of
     * Branch and the connection lines used to link these polylines.
     */
    void OptimizeIntersection() {
        if (branches_.empty()) return;

        branch_connector_.clear();
        connections_.clear();
        bad_connections_.clear();

        ComputeBackoffScale();

        ShrinkBranches();
        GroupJunctions();
        ConstructAnchorGraph();

        Array<RPolyline2D> branches(branches_.size());
        for (int i = 0; i < branches.size(); ++i) {
            branches[i] = branches_[i].polyline;
        }
        branch_connector_.ResetBranches(branches);
        for (auto& group : junction_groups_) {
            ConnectJunctions(group.junctions);
        }

        // Save the non-junction polyline results into branch.
        CHECK(branches_.size() == branch_connector_.branches().size());
        for (int i = 0; i < branches_.size(); ++i) {
            auto& branch = branches_[i];
            if (branch.polyline.empty()) continue;
            auto& b = branch_connector_.branches()[i];
            b.GetNonJunctionPolyline(&branch.non_junction_polyline);
        }
    }

    /**
     * Densify the non-junction polyline of a road branch, while generating
     * corresponding width values for each densified vertex.
     */
    void DensifyNonJunctionPolyline(const Branch& branch,
                                    RPolyline2D* densified_polyline,
                                    Array<double>* widths) const {
        CHECK(densified_polyline);
        CHECK(widths);

        const RPolyline2D& polyline = branch.non_junction_polyline;

        if (polyline.size() <= 1) {
            *densified_polyline = polyline;
            widths->assign(polyline.size(), 0.0);
            return;
        }
        CHECK(branch.raw_polyline.size() >= 2);

        densified_polyline->clear();
        widths->clear();

        int start = 0;
        Array<int> indices;
        for (int i = 1; i + 1 < polyline.size(); ++i) {
            int found = -1;
            for (int j = start; j < branch.raw_polyline.size(); ++j) {
                if (polyline[i] == branch.raw_polyline[j]) {
                    found = j;
                    start = j;
                    break;
                }
            }
            CHECK(found != -1);
            indices.push_back(found);
        }

        double dis = DBL_MAX;
        int found_first = -1;
        int left = indices.empty() ? branch.raw_polyline.size() - 1
                                   : indices.front();
        for (int i = 0; i <= left; ++i) {
            double d = Distance(branch.raw_polyline[i], polyline.front());
            if (d < dis) {
                dis = d;
                found_first = i;
            }
        }
        CHECK(found_first != -1);
        indices.insert(indices.begin(), found_first);

        dis = DBL_MAX;
        int found_last = -1;
        for (int i = branch.raw_polyline.size() - 1; i >= indices.back(); --i) {
            double d = Distance(branch.raw_polyline[i], polyline.back());
            if (d < dis) {
                dis = d;
                found_last = i;
            }
        }
        CHECK(found_last != -1);
        indices.push_back(found_last);

        CHECK(indices.size() == polyline.size());

        densified_polyline->push_back(polyline.front());
        widths->push_back(branch.raw_width[indices.front()]);
        for (int i = 0; i + 1 < indices.size(); ++i) {
            int l = indices[i], r = indices[i + 1];
            CHECK(l <= r);

            Array<RPoint2D> segs;
            geometry::EvenlyPointsOnSegment2D(polyline[i], polyline[i + 1],
                                              std::max(2, r - l + 1),
                                              &segs);
            for (int j = 1; j < segs.size(); ++j) {
                densified_polyline->push_back(segs[j]);
                widths->push_back(branch.raw_width[std::min(l + j, r)]);
            }
        }
        if (widths->size() == 1) {
            densified_polyline->push_back(polyline.back());
            widths->push_back(branch.raw_width[indices.back()]);
        }

        CHECK(densified_polyline->size() == widths->size());
    }

    /**
     * Return the width of the junction.
     */
    double GetJunctionWidth(int j) const {
        CHECK(j >= 0 && j / 2 < branches_.size());

        double w = 0.0;

        const Branch& branch = branches_[j / 2];
        if (branch.is_stable()) {
            RPoint2D q = j % 2 == 0 ? branch.polyline.front()
                                    : branch.polyline.back();
            int v = geometry::polyline_util::FindNearestVertex(
                        branch.raw_polyline, q);
            CHECK(v != -1);
            w = branch.raw_width[v];
        }
        return w;
    }

    /**
     * Decompose the road centerline into branch-segments, where each sub-array
     * corresponds to a complete road branch.
     */
    void ToBranchSegments(Array<Array<RoadBranchSegment>>* segments) const {
        CHECK(segments);

        segments->clear();

        geometry::GeoGraph<RPoint2D> graph;
        graph.set_insert_two_way_edge(true);

        std::unordered_map<int, double> w;
        for (const auto& c : connections_) {
            int v1 = graph.InsertVertex(c.line.lower_point());
            w[v1] = c.width;
            int v2 = graph.InsertVertex(c.line.upper_point());
            w[v2] = c.width;
            if (v1 != v2) {
                graph.InsertEdge(v1, v2);
            }
        }

        for (const auto& branch : branches_) {
            if (branch.non_junction_polyline.size() <= 1) continue;

            RPolyline2D polyline;
            Array<double> widths;
            DensifyNonJunctionPolyline(branch, &polyline, &widths);
            CHECK(polyline.size() == widths.size());

            for (auto [index, q] : enumerate(polyline)) {
                int v = graph.InsertVertex(q);
                w[v] = widths[index];
            }
            graph.InsertPath(polyline);
        }

        Array<Array<int>> chains;
        graph.GetChains(&chains);
        Array<bool> visited(graph.n_allocated_edges(), false);

        RoadBranchSegmenter segmenter;

        Array<RoadBranchSegment> segs;
        for (Array<int>& c : chains) {
            if (visited[c.front()]) continue;
            RoadBranch road_branch;
            for (int c1 : c) {
                visited[c1] = true;
                visited[graph.TwinEdge(c1)] = true;
                if (road_branch.polyline.empty()) {
                    road_branch.polyline.push_back(graph.source_point(c1));
                    road_branch.widths.push_back(w[graph.source(c1)]);
                }
                road_branch.polyline.push_back(graph.target_point(c1));
                road_branch.widths.push_back(w[graph.target(c1)]);
            }

            segmenter.Split(road_branch, resolution_, &segs);
            // Donot forget to simplify the branch polyline.
            for (auto& s : segs) {
                geometry::polyline_util::DouglasPeuckerSimplify(s.polyline,
                                                                0.01,
                                                                &s.polyline);
            }
            segments->push_back(segs);
        }
    }

    /**
     * Return the j-th junction's connection point.
     */
    const RPoint2D& junction_point(int j) const {
        CHECK(j >= 0 && j / 2 < branch_connector_.branches().size());

        const auto& branch = branch_connector_.branches()[j / 2];
        CHECK(!branch.polyline.empty());

        return j % 2 == 0 ? branch.junction_point1() : branch.junction_point2();
    }

    /**
     * Return raw medial axis of the input polygon.
     */
    const geometry::TriangulationAxis2D<double>& raw_medial_axis() const {
        return medial_axis_;
    }

    /**
     * Return centerline branches.
     */
    const Array<Branch>& branches() const {
        return branches_;
    }

    /**
     * Return the i-th branch of the centerline.
     */
    const Branch& branch(int i) const {
        CHECK(i >= 0 && i < branches_.size());

        return branches_[i];
    }

    /**
     * Return the optimized connection lines computed by OptimizeIntersection().
     *
     * These connection lines are used to link branch's (non-junction) polylines
     * at road intersections. Together with the branch's polylines, they form
     * the complete road centerline after OptimizeIntersection() has been
     * executed.
     */
    const Array<Connection>& connections() const {
        return connections_;
    }

    const Array<RSegment2D>& bad_connections() const {
        return bad_connections_;
    }

    double resolution() const {
        return resolution_;
    }

    /**
     * Return the anchors in the road centerline.
     */
    const Array<AnchorVertex>& anchors() const {
        return anchors_;
    }

private:
    /**
     * Get raw branches from the medial axis graph.
     */
    void GetRawBranches() {
        branches_.clear();

        const auto& axis = medial_axis_.axis();
        Array<Array<int>> chains;
        axis.GetChains(&chains);
        Array<bool> visited(axis.n_allocated_edges(), false);
        for (Array<int>& c : chains) {
            if (visited[c.front()]) continue;

            Branch branch;
            for (int c1 : c) {
                visited[c1] = true;
                visited[axis.TwinEdge(c1)] = true;
                if (branch.raw_polyline.empty()) {
                    branch.raw_polyline.push_back(axis.source_point(c1));
                    branch.raw_vertices.push_back(axis.source(c1));
                }
                branch.raw_polyline.push_back(axis.target_point(c1));
                branch.raw_vertices.push_back(axis.target(c1));
            }
            branches_.push_back(branch);
        }
        Array<double> width(medial_axis_.axis().n_allocated_vertices(), 0.0);
        for (auto v : medial_axis_.axis().vertices()) {
            width[v.id] = 2.0 * medial_axis_.radius(v.id);
        }

        // Compute the simplified raw branch.
        for (auto& branch : branches_) {
            for (auto v : branch.raw_vertices) {
                branch.raw_width.push_back(width[v]);
            }
            Simplify(branch.raw_polyline, branch.raw_width,
                     &branch.simplified_raw_polyline);
        }

        branch_connector_.ResetRoadPolygon(original_road_surface_);

        // Extend branches from the terminal endpoints.
        ExtendBranches();
    }

    /**
     * Use width-adaptive Douglas-Peucker algorithm to simplify the polyline.
     */
    void Simplify(const RPolyline2D& polyline, Array<double>& widths,
                  RPolyline2D* result) const {
        CHECK(result);
        CHECK(polyline.size() == widths.size());

        if (polyline.size() <= 2) {
            *result = polyline;
            return;
        }

        Array<bool> is_remain(polyline.size(), false);
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(0, polyline.size() - 1));
        while (!q.empty()) {
            std::pair<int, int> p = q.front();
            q.pop();

            is_remain[p.first] = true;
            is_remain[p.second] = true;

            typename RPolyline2D::Segment seg(polyline[p.first],
                                              polyline[p.second]);
            int split = -1;
            double dis = 0.0;
            for (int i = p.first + 1; i < p.second; ++i) {
                double d = Distance(polyline[i], seg);
                if (d > dis) {
                    dis = d;
                    split = i;
                }
            }
            if (split == -1) continue;
            double w = widths[split];
            w = std::min(0.2 * w, resolution_);
            if (dis < w) continue;

            if (p.first < split) q.push(std::make_pair(p.first, split));
            if (split < p.second) q.push(std::make_pair(split, p.second));
        }

        result->clear();
        for (int i = 0; i < polyline.size(); ++i) {
            if (is_remain[i])
                result->push_back(polyline[i]);
        }
    }


    /**
     * Extends the branches from their terminal endpoints.
     */
    void ExtendBranches() {
        std::unordered_map<RPoint2D, int> degrees;
        for (const auto& branch : branches_) {
            if (branch.simplified_raw_polyline.size() <= 1) continue;
            ++degrees[branch.simplified_raw_polyline.front()];
            ++degrees[branch.simplified_raw_polyline.back()];
        }
        for (auto& branch : branches_) {
            if (branch.simplified_raw_polyline.size() <= 1) continue;

            RPoint2D& p1 = branch.simplified_raw_polyline.front();
            RPoint2D& p2 = branch.simplified_raw_polyline.back();
            RPoint2D q;
            if (degrees[p1] == 1) {
                double w = branch.raw_width.front();
                RVector2D v = branch.simplified_raw_polyline[0] -
                              branch.simplified_raw_polyline[1];
                if (w > resolution_) {
                    if (branch_connector_.ConnectBranchToBoundary(p1, v, w,
                                                                  &q)) {
                        if (Distance(q, p1) > resolution_) {
                            p1 = q - resolution_ * v.normalized();
                        }
                    } else {
                        p1 += v.normalized() * w;
                    }
                }
            }
            if (degrees[p2] == 1) {
                double w = branch.raw_width.back();
                int n = branch.simplified_raw_polyline.size();
                RVector2D v = branch.simplified_raw_polyline.back() -
                              branch.simplified_raw_polyline[n - 2];
                if (w > resolution_) {
                    if (branch_connector_.ConnectBranchToBoundary(p2, v, w,
                                                                  &q)) {
                        if (Distance(q, p2) > resolution_) {
                            p2 = q -  resolution_ * v.normalized();
                        }
                    } else {
                        p2 += v.normalized() * w;
                    }
                }
            }
        }
    }

    /**
     * Compute the backoff scale for each branch.
     */
    void ComputeBackoffScale() {
        for (auto& branch : branches_) {
            double backoff = 0.5 * branch.backoff_scale *
                             (branch.raw_width.front() +
                              branch.raw_width.back());
            if (backoff > branch.simplified_raw_polyline.Length()) {
                RSegment2D line(branch.raw_polyline.front(),
                                branch.raw_polyline.back());
                if (!branch_connector_.IsValid(line)) {
                    branch.can_not_ignore = true;
                    branch.backoff_scale = 1.0;
                    branch.backoff_scale = EvaluateBackoffScale(branch);
                }
            }
        }

        ShrinkBranches();
        GroupJunctions();
        ConstructAnchorGraph();

        Array<RPolyline2D> branches(branches_.size());
        for (int i = 0; i < branches.size(); ++i) {
            branches[i] = branches_[i].polyline;
        }
        branch_connector_.ResetBranches(branches);

        for (auto& group : junction_groups_) {
            const Array<int>& junctions = group.junctions;
            const int n = junctions.size();
            CHECK(n >= 2);

            Array<PairConnection> connections;
            if (group.topology_valid) {
                if (!PairConnect(junctions, &connections)) {
                    group.topology_valid = false;
                }
            }

            if (!group.topology_valid) {
                // Shrink backoff scale so that (almost) no branch will be set
                // to instable.
                for (int j : group.junctions) {
                    if (!branches_[j / 2].can_not_ignore)
                        branches_[j / 2].backoff_scale = 1.0;
                }
                for (int j : group.unstable_junctions) {
                    if (!branches_[j / 2].can_not_ignore) {
                        branches_[j / 2].backoff_scale = 1.0;
                        branches_[j / 2].backoff_scale =
                                EvaluateBackoffScale(branches_[j / 2]);
                    }
                }
            } else {
                std::set<int> junction_hash;
                for (auto& c : connections) {
                    junction_hash.insert(c.junction1);
                    junction_hash.insert(c.junction2);
                }
                for (int j : junctions) {
                    if (junction_hash.find(j) == junction_hash.end())
                        branches_[j / 2].backoff_scale = 1.0;
                }
            }
        }
    }

    /**
     * Shrink branches at the intersection.
     */
    void ShrinkBranches() {
        const auto& axis = medial_axis_.axis();

        for (int i = 0; i < branches_.size(); ++i) {
            Branch& branch = branches_[i];
            branch.polyline = branch.simplified_raw_polyline;
            if (axis.edges_from(branch.raw_vertices.front()).size() > 2) {
                double backoff = branch.backoff_scale *
                                 0.5 * branch.raw_width.front();
                RPolyline2D poly2;
                geometry::polyline_util::Split(branch.polyline, backoff,
                                               &branch.front_backoff_polyline,
                                               &poly2);
                UpdateBranch(i, poly2);
                if (branch.polyline.empty()) {
                    continue;
                }
            }

            if (axis.edges_from(branch.raw_vertices.back()).size() > 2) {
                double backoff = branch.backoff_scale *
                                 0.5 * branch.raw_width.back();
                double l = branch.polyline.Length();
                if (l < backoff) {
                    branch.polyline.clear();
                    continue;
                }

                RPolyline2D poly1;
                geometry::polyline_util::Split(branch.polyline, l - backoff,
                                               &poly1,
                                               &branch.back_backoff_polyline);
                UpdateBranch(i, poly1);
            }
        }
    }

    /**
     * Evaluate the backoff scale of the branch to make it difficult to set to
     * unstable.
     */
    double EvaluateBackoffScale(const Branch& branch) const {
        const auto& axis = medial_axis_.axis();

        const double min_backoff_scale = 0.1;

        double backoff_scale = branch.backoff_scale;
        while (backoff_scale > min_backoff_scale) {
            RPolyline2D branch_polyline = branch.simplified_raw_polyline;
            if (axis.edges_from(branch.raw_vertices.front()).size() > 2) {
                double backoff = backoff_scale * 0.5 * branch.raw_width.front();
                RPolyline2D poly1, poly2;
                geometry::polyline_util::Split(branch_polyline, backoff,
                                               &poly1, &poly2);
                branch_polyline = poly2;
                if (branch_polyline.size() < 2) {
                    backoff_scale *= 0.5;
                    continue;
                }
            }

            if (axis.edges_from(branch.raw_vertices.back()).size() > 2) {
                double backoff = backoff_scale * 0.5 * branch.raw_width.back();
                double l = branch_polyline.Length();
                if (l < backoff) {
                    backoff_scale *= 0.5;
                    continue;
                }

                RPolyline2D poly1, poly2;
                geometry::polyline_util::Split(branch_polyline, l - backoff,
                                               &poly1, &poly2);
                if (poly1.size() >= 2) {
                    return backoff_scale;
                }
                backoff_scale *= 0.5;
                continue;
            }
            return backoff_scale;
        }
        return std::max(min_backoff_scale, backoff_scale);
    }

    /**
     * Group junctions that belong to the same road intersection.
     */
    void GroupJunctions() {
        int n_junctions = 2 * branches_.size();
        DisjointSet set(n_junctions);

        std::unordered_map<int, Array<int>> junction_group;
        for (auto&& [index, branch] : enumerate(branches_)) {
            junction_group[branch.raw_vertices.front()].push_back(2 * index);
            junction_group[branch.raw_vertices.back()].push_back(2 * index + 1);
            if (!branch.is_stable()) {
                set.Union(2 * index, 2 * index + 1);
            }
        }

        for (auto& p : junction_group) {
            for (auto j : p.second) {
                set.Union(j, p.second.front());
            }
        }

        Array<Array<int>> clusters;
        set.ToClusters(&clusters);
        junction_groups_.clear();
        for (auto& cluster : clusters) {
            JunctionGroup group;
            if (cluster.size() >= 2) {
                for (int i : cluster) {
                    if (branches_[i / 2].is_stable()) {
                        group.junctions.push_back(i);
                    } else {
                        group.unstable_junctions.push_back(i);
                    }
                }
                if (group.junctions.size() > 2 ||
                    (group.junctions.size() == 2 && cluster.size() > 2)) {
                    junction_groups_.push_back(group);
                }
            }
        }
    }

    /**
     * Find anchor path for every pair of junctions in the same connection
     * group and then construct the anchor graph.
     */
    void ConstructAnchorGraph() {
        anchors_.clear();
        anchor_paths_.clear();

        const auto& axis = medial_axis_.axis();
        Array<int> axis_vertex_to_anchor(axis.n_allocated_vertices(), -1);
        for (auto v : axis.vertices()) {
            if (axis.edges_from(v.id).size() >= 3) {
                AnchorVertex anchor;
                anchor.id = v.id;
                anchor.point = v.point;
                anchor.width = 2.0 * medial_axis_.radius(v.id);
                axis_vertex_to_anchor[v.id] = anchors_.size();
                anchors_.push_back(anchor);
            }
        }

        int n_junctions = branches_.size() * 2;
        Array<bool> valid(n_junctions, true);

        // Graph for anchor points and junctions.
        Graph anchor_graph(n_junctions + anchors_.size());
        auto weight = anchor_graph.AddEdgeProperty<double>("weight", 0.0);
        for (int i = 0; i < branches_.size(); ++i) {
            int junction1 = i + i;
            int junction2 = i + i + 1;

            const Branch& branch = branches_[i];
            if (!branch.is_stable()) {
                int v1 = axis_vertex_to_anchor[branch.raw_vertices.front()];
                if (v1 == -1) continue;
                v1 += n_junctions;
                int v2 = axis_vertex_to_anchor[branch.raw_vertices.back()];
                if (v2 == -1) continue;
                v2 += n_junctions;

                if (anchor_graph.FindEdge(v1, v2)) {
                    valid[junction1] = false;
                    valid[junction2] = false;
                } else {
                    auto e = anchor_graph.InsertTwoWayEdge(v1, v2);
                    weight[e] = branch.raw_polyline.Length();
                    weight[e->twin()] = branch.raw_polyline.Length();
                }
                continue;
            }

            if (!branch.front_backoff_polyline.empty()) {
                double l = branch.front_backoff_polyline.Length();
                int v = axis_vertex_to_anchor[branch.raw_vertices.front()];
                CHECK(v != -1);
                v += n_junctions;
                auto e = anchor_graph.InsertTwoWayEdge(junction1, v);
                weight[e] = weight[e->twin()] = l;
            }

            if (!branch.back_backoff_polyline.empty()) {
                double l = branch.back_backoff_polyline.Length();
                int v = axis_vertex_to_anchor[branch.raw_vertices.back()];
                CHECK(v != -1);
                v += n_junctions;
                auto e = anchor_graph.InsertTwoWayEdge(junction2, v);
                weight[e] = weight[e->twin()] = l;
            }
        }

        Array<int> prev;
        Array<double> dis;
        for (const auto& group : junction_groups_) {
            for (auto j1 : group.junctions) {
                graph::DijkstraShortestPaths(anchor_graph, weight, j1, &prev,
                                             &dis);

                for (int j2 : group.junctions) {
                    if (j1 == j2) continue;
                    if (prev[j2] == -1) continue;

                    Array<int> path;
                    path.clear();
                    for (int i = j2; i != -1; i = prev[i]) {
                        path.push_back(i);
                    }
                    std::reverse(path.begin(), path.end());

                    auto& edge = anchor_paths_[std::make_pair(j1, j2)];
                    for (int i = 1; i + 1 < path.size(); ++i) {
                        CHECK(path[i] >= n_junctions);
                        edge.push_back(path[i] - n_junctions);
                    }
                }
            }
        }

        for (auto& group : junction_groups_) {
            for (int j : group.unstable_junctions) {
                if (!valid[j]) group.topology_valid = false;
            }
        }
    }

    /**
     * Return the shortest anchor path from junction1 to junction2.
     */
    bool GetAnchorPath(int junction1, int junction2,
                       Array<RPoint2D>* path) const {
        CHECK(junction1 >= 0 && junction1 < 2 * branches_.size());
        CHECK(junction2 >= 0 && junction2 < 2 * branches_.size());
        CHECK(path);

        path->clear();
        auto key = std::make_pair(junction1, junction2);
        auto it = anchor_paths_.find(key);
        if (it == anchor_paths_.end()) return false;

        path->push_back(this->junction_point(junction1));
        for (auto a : it->second) {
            path->push_back(anchors_[a].point);
        }
        path->push_back(this->junction_point(junction2));
        if (path->size() <= 2) path->clear();

        return !path->empty();
    }

    /**
     * Get ordered anchors on the given pair-connection.
     */
    void GetOrderedAnchors(const PairConnection& connection,
                           const std::set<int>& anchors_on_connection,
                           Array<int>* ordered_anchors) const {
        CHECK(ordered_anchors);

        geometry::PointDotCompare2D<double> compare(connection.junction_point1,
                                                    connection.junction_point2);
        Array<int> anchors;
        Array<RPoint2D> anchor_points;
        for (auto a : anchors_on_connection) {
            const auto& v = anchors_[a];
            if (v.invalid) continue;

            anchors.push_back(a);
            anchor_points.push_back(v.point);
        }

        Array<int> seq;
        IndexSort(anchor_points.begin(), anchor_points.end(), compare, &seq);
        ordered_anchors->clear();
        ordered_anchors->reserve(seq.size());
        for (int i : seq) {
            ordered_anchors->push_back(anchors[i]);
        }
    }

    /**
     * Connect the given junctions.
     */
    void ConnectJunctions(const Array<int>& junctions) {
        const int n = junctions.size();
        CHECK(n >= 2);

        Array<PairConnection> connections;
        PairConnect(junctions, &connections);

        // Assign anchors to the corresponding pair-connections.
        Array<std::set<int>> anchors_on_connection(connections.size());
        for (int i = 0; i < connections.size(); ++i) {
            PairConnection& pc = connections[i];
            for (auto a : anchor_paths_[std::make_pair(pc.junction1,
                                                       pc.junction2)]) {
                anchors_on_connection[i].insert(a);
            }
        }

        std::set<int> anchors;
        UpdateAnchors(junctions, connections, anchors_on_connection, &anchors);

        // Add extra anchors.
        std::unordered_map<RPoint2D, int> has_anchor_points;
        for (int a : anchors) {
            has_anchor_points[anchors_[a].point] = a;
        }
        for (int i = 0; i < connections.size(); ++i) {
            for (int j = i + 1; j < connections.size(); ++j) {
                RPoint2D q;
                if (geometry::Cross(connections[i].line, connections[j].line,
                                    &q)) {
                    if (has_anchor_points.count(q)) {
                        AnchorVertex& v = anchors_[has_anchor_points[q]];
                        v.is_cross_point = true;
                        continue;
                    }
                    AnchorVertex v;
                    v.id = -1;
                    v.point = q;
                    v.fixed = true;
                    v.score = 0.0;
                    v.width = 0.25 * (connections[i].width1 +
                                      connections[i].width2 +
                                      connections[j].width1 +
                                      connections[j].width2);
                    v.is_cross_point = true;
                    anchors_on_connection[i].insert(anchors_.size());
                    anchors_on_connection[j].insert(anchors_.size());
                    anchors.insert(anchors_.size());
                    anchors_.push_back(v);
                }
            }

            // For each invalid anchor, insert a virtual anchor into the related
            // connections.
            for (auto a : anchors_on_connection[i]) {
                if (!anchors_[a].invalid) continue;

                RPoint2D p = geometry::Project(anchors_[a].point,
                                               connections[i].line);
                AnchorVertex v;
                v.id = -1;
                v.point = p;
                v.width = 0.5 * (connections[i].width1 + connections[i].width2);
                anchors_on_connection[i].insert(anchors_.size());
                anchors.insert(anchors_.size());
                anchors_.push_back(v);
            }
        }

        // Merge anchors.
        const double threshold = merge_anchor_threshold_ * resolution_;
        for (int i = 0; i < connections.size(); ++i) {
            MergeAnchors(anchors_on_connection[i], threshold);
        }

        ConnectJunctionsToPairConnection(connections, anchors_on_connection);

        // Merge anchors.
        for (int i = 0; i < connections.size(); ++i) {
            MergeAnchors(anchors_on_connection[i], threshold);
        }

        JunctionGraph junction_graph;
        ConstructJunctionGraph(junctions, connections, anchors,
                               anchors_on_connection, &junction_graph);
        AddConnections(junction_graph);
    }

    /**
     * Construct complete junction graph.
     */
    void ConstructJunctionGraph(
            const Array<int>& junctions,
            const Array<PairConnection>& connections,
            const std::set<int>& anchors,
            const Array<std::set<int>>& anchors_on_connection,
            JunctionGraph* junction_graph) {
        DisjointSet set(junctions.size() + anchors.size());
        std::unordered_map<int, int> junction_index, anchor_index;
        for (int i = 0; i < junctions.size(); ++i) {
            junction_index[junctions[i]] = i;
        }
        int n = junctions.size();
        for (int a : anchors) {
            anchor_index[a] = n++;
        }

        for (int i = 0; i < connections.size(); ++i) {
            Array<int> anchors;
            GetOrderedAnchors(connections[i], anchors_on_connection[i],
                              &anchors);
            if (anchors.empty()) {
                set.Union(junction_index[connections[i].junction1],
                          junction_index[connections[i].junction2]);
                junction_graph->AddJunctionPair(connections[i].junction1,
                                                connections[i].junction2);
            } else {
                set.Union(junction_index[connections[i].junction1],
                          anchor_index[anchors.front()]);
                set.Union(junction_index[connections[i].junction2],
                          anchor_index[anchors.back()]);
                junction_graph->AddJunctionAnchorPair(connections[i].junction1,
                                                      anchors.front());
                junction_graph->AddJunctionAnchorPair(connections[i].junction2,
                                                      anchors.back());
            }
            for (int j = 0; j + 1 < anchors.size(); ++j) {
                set.Union(anchor_index[anchors[j]],
                          anchor_index[anchors[j + 1]]);
                junction_graph->AddAnchorPair(anchors[j], anchors[j + 1]);
            }
        }

        Array<RSegment2D> connection_lines;
        GetConnectionLines(*junction_graph, &connection_lines);

        for (int a : anchors) {
            auto& v = anchors_[a];
            for (int j : v.adjacent_junctions) {
                bool need_connect = true;
                for (int i = 0; i < connections.size(); ++i) {
                    if (!anchors_on_connection[i].count(a)) continue;
                    if (connections[i].junction1 == j ||
                        connections[i].junction2 == j) {
                        need_connect = false;
                        break;
                    }
                }
                if (!need_connect) continue;
                RSegment2D l(this->junction_point(j), anchors_[a].point);
                if (AddConnection(l, &connection_lines)) {
                    set.Union(junction_index[j], anchor_index[a]);
                    junction_graph->AddJunctionAnchorPair(j, a);
                }
            }
            for (int a1 : v.adjacent_anchors) {
                bool on_same_connection = false;
                for (const auto& on_connection : anchors_on_connection) {
                    if (on_connection.count(a) && on_connection.count(a1)) {
                        on_same_connection = true;
                    }
                }
                if (!on_same_connection) {
                    RSegment2D l(anchors_[a].point, anchors_[a1].point);
                    if (AddConnection(l, &connection_lines)) {
                        set.Union(anchor_index[a1], anchor_index[a]);
                        junction_graph->AddAnchorPair(a1, a);
                    }
                }
            }
        }

        std::unordered_map<RPoint2D, int> anchor_map;
        for (auto s : anchors) {
            if (anchor_map.find(anchors_[s].point) != anchor_map.end()) {
                set.Union(anchor_map[anchors_[s].point], anchor_index[s]);
            } else {
                anchor_map[anchors_[s].point] = anchor_index[s];
            }
        }

        Array<Array<RPoint2D>> anchor_points(junctions.size());
        Array<Array<int>> junction_anchors(junctions.size());
        for (int i = 0; i < junctions.size(); ++i) {
            anchor_points[i].push_back(this->junction_point(junctions[i]));
            junction_anchors[i].push_back(-1);
            for (int j : anchors) {
                if (set.Find(i) == set.Find(anchor_index[j])) {
                    anchor_points[i].push_back(anchors_[j].point);
                    junction_anchors[i].push_back(j);
                }
            }
        }

        for (int i = 0; i < junctions.size(); ++i) {
            for (int j = i + 1; j < junctions.size(); ++j) {
                if (set.Find(i) == set.Find(j)) continue;

                RSegment2D best_l;
                std::pair<int, int> best;
                double dis = DBL_MAX;
                for (int k1 = 0; k1 < anchor_points[i].size(); ++k1) {
                    for (int k2 = 0; k2 < anchor_points[j].size(); ++k2) {
                        RPoint2D p = anchor_points[i][k1];
                        RPoint2D q = anchor_points[j][k2];
                        RSegment2D l(p, q);
                        double d = Distance(p, q);
                        if (d >= dis) continue;
                        if (!branch_connector_.IsValid(l)) continue;
                        bool invalid = false;
                        for (auto& c : connection_lines) {
                            if (geometry::Cross(c, l)) {
                                invalid = true;
                                break;
                            }
                        }
                        if (!invalid) {
                            dis = d;
                            best = std::pair<int, int>(k1, k2);
                            best_l = l;
                        }
                    }
                }

                if (dis < DBL_MAX) {
                    set.Union(i, j);
                    connection_lines.push_back(best_l);
                    if (best.first == 0 && best.second == 0) {
                        junction_graph->AddJunctionPair(junctions[i],
                                                        junctions[j]);
                    } else if (best.first == 0) {
                        junction_graph->AddJunctionAnchorPair(
                                    junctions[i],
                                    junction_anchors[j][best.second]);
                    } else if (best.second == 0) {
                        junction_graph->AddJunctionAnchorPair(
                                    junctions[j],
                                    junction_anchors[i][best.first]);
                    } else {
                        junction_graph->AddAnchorPair(
                                    junction_anchors[i][best.first],
                                    junction_anchors[j][best.second]);
                    }
                }
            }
        }

        for (int i = 0; i < junctions.size(); ++i) {
            for (int j = i + 1; j < junctions.size(); ++j) {
                if (set.Find(i) != set.Find(j)) {
                    bad_connections_.emplace_back(
                                this->junction_point(junctions[i]),
                                this->junction_point(junctions[j]));
                    set.Union(i, j);
                }
            }
        }
    }

    /**
     * Update anchors according to the pair connections.
     */
    void UpdateAnchors(const Array<int>& junctions,
                       const Array<PairConnection>& connections,
                       const Array<std::set<int>>& anchors_on_connection,
                       std::set<int>* anchors) {
        CHECK(connections.size() == anchors_on_connection.size());
        CHECK(anchors);

        // Assign pair-conenctions to the corresponding anchors.
        std::map<int, Array<int>> connections_of_anchor;
        const double epsilon = 1e-3;
        for (int i = 0; i < connections.size(); ++i) {
            const PairConnection& pc = connections[i];
            for (int a : anchors_on_connection[i]) {
                auto& ac = connections_of_anchor[a];
                ac.push_back(i);

                auto& v = anchors_[a];
                if (v.invalid) continue;

                if (ac.size() == 1) {
                    v.point = geometry::Project(v.point, pc.line);
                } else {
                    auto line = connections[ac.front()].line;
                    if (!geometry::Cross(line, pc.line, &v.point)) {
                        v.invalid = true;
                        continue;
                    }

                    for (int i : ac) {
                        if (Distance(v.point, connections[i].line) > epsilon) {
                            v.invalid = true;
                            break;
                        }
                    }

                    if (!v.invalid) {
                        v.fixed = true;
                        v.score = 0.0;
                    }
                }
            }
        }

        anchors->clear();
        for (int i = 0; i < junctions.size(); ++i) {
            for (int j = i + 1; j < junctions.size(); ++j) {
                Array<RPoint2D> path;
                auto key = std::make_pair(junctions[i], junctions[j]);
                auto it = anchor_paths_.find(key);
                if (it == anchor_paths_.end()) continue;
                if (it->second.empty()) continue;

                int i1 = it->second.front(), i2 = it->second.back();
                auto& v1 = anchors_[i1];
                auto& v2 = anchors_[i2];
                if (!v1.invalid) {
                    v1.adjacent_junctions.insert(junctions[i]);
                    anchors->insert(i1);
                }
                if (!v2.invalid) {
                    v2.adjacent_junctions.insert(junctions[j]);
                    anchors->insert(i2);
                }
                for (int k = 0; k + 1 < it->second.size(); ++k) {
                    int i1 = it->second[k], i2 = it->second[k + 1];
                    auto& v1 = anchors_[i1];
                    auto& v2 = anchors_[i2];
                    if (!v2.invalid) anchors->insert(i2);
                    if (v1.invalid || v2.invalid) continue;
                    v1.adjacent_anchors.insert(i2);
                    v2.adjacent_anchors.insert(i1);
                }
            }
        }
    }

    /**
     * Connect junctions to the pair-connections and returns the set of
     * junctions that have been successfully connected.
     */
    void ConnectJunctionsToPairConnection(
            const Array<PairConnection>& connections,
            const Array<std::set<int>>& anchors_on_connection) {
        CHECK(connections.size() == anchors_on_connection.size());

        if (connections.empty()) return;

        // The junction is already connected on the pari-connections.
        std::unordered_map<int, bool> connected;

        for (int i = 0; i < connections.size(); ++i) {
            RPoint2D s = connections[i].junction_point1;
            RPoint2D t = connections[i].junction_point2;
            for (int o = -1; o <= 1; o += 2) {
                Array<int> anchors;
                GetOrderedAnchors(connections[i], anchors_on_connection[i],
                                  &anchors);
                RPoint2D prev = s;

                Array<int> anchor_junctions;
                Array<int> supported_anchors;
                for (int anchor_id : anchors) {
                    auto& v = anchors_[anchor_id];

                    bool fixed = v.fixed;
                    if (!fixed) {
                        for (int k : v.adjacent_anchors) {
                            if (anchors_on_connection[i].count(k)) continue;

                            const auto& v1 = anchors_[k];
                            if (geometry::Orientation(s, t, v1.point) == o) {
                                fixed = true;
                                break;
                            }
                        }
                    }
                    if (!fixed) {
                        for (int j : v.adjacent_junctions) {
                            if (j == connections[i].junction1) continue;
                            if (j == connections[i].junction2) continue;

                            int o1 = geometry::Orientation(s, t,
                                                           junction_point(j));
                            if (o1 == o) {
                                anchor_junctions.push_back(j);
                                supported_anchors.push_back(anchor_id);
                            }
                        }
                    } else {
                        ConnectJunctionsToLine(anchor_junctions,
                                               supported_anchors,
                                               RSegment2D(prev, v.point));
                        anchor_junctions.clear();
                        supported_anchors.clear();
                        prev = v.point;
                        continue;
                    }
                }
                ConnectJunctionsToLine(anchor_junctions, supported_anchors,
                                       RSegment2D(prev, t));
            }
        }
    }

    /**
     * Connect junctions to line segment.
     */
    bool ConnectJunctionsToLine(const Array<int>& junctions,
                                const Array<int>& anchors,
                                const RSegment2D& line) {
        CHECK(anchors.size() == junctions.size());

        if (junctions.empty()) return false;
        if (line.lower_point() == line.upper_point()) return false;

        Array<double> junction_width;
        for (int j : junctions) {
            if (j % 2 == 0)
                junction_width.push_back(branches_[j / 2].front_width);
            else
                junction_width.push_back(branches_[j / 2].back_width);
        }

        Array<SingleConnection> connections;
        if (branch_connector_.ConnectJunctionsToLine(junctions, junction_width,
                                                     line, &connections)) {
            for (auto&& [index, c] : enumerate(connections)) {
                AnchorVertex& v = anchors_[anchors[index]];
                v.point = c.connection_point;
                v.score = std::min(v.score, c.score);
            }
            return true;
        }
        return false;
    }

    /**
     * Update the i-th branch.
     */
    void UpdateBranch(int i, const RPolyline2D& polyline) {
        Branch& branch = branches_[i];
        branch.polyline = polyline;
        branch.front_width = GetJunctionWidth(i * 2);
        branch.back_width = GetJunctionWidth(i * 2 + 1);
    }

    /**
     * Merge anchors on the given connection.
     */
    void MergeAnchors(const std::set<int> anchors_on_connection,
                      double distance_threshold) {
        Array<int> anchors(anchors_on_connection.begin(),
                           anchors_on_connection.end());
        Array<double> scores;
        for (auto i : anchors) {
            scores.push_back(anchors_[i].score);
        }
        Array<int> seq;
        IndexSort(scores.begin(), scores.end(), &seq);

        Array<int> remained_anchors;
        for (int i = 0; i < seq.size(); ++i) {
            int v = anchors[seq[i]];

            double min_dis = DBL_MAX;
            int best = -1;
            for (int v1 : remained_anchors) {
                double dis = Distance(anchors_[v1].point,
                                      anchors_[v].point);
                if (dis < min_dis) {
                    min_dis = dis;
                    best = v1;
                }
            }

            if (min_dis < distance_threshold) {
                Array<int> intersection;
                std::set_intersection(anchors_[v].adjacent_junctions.begin(),
                                      anchors_[v].adjacent_junctions.end(),
                                      anchors_[best].adjacent_junctions.begin(),
                                      anchors_[best].adjacent_junctions.end(),
                                      std::back_inserter(intersection));
                if (intersection.empty()) {
                    anchors_[v].point = anchors_[best].point;
                    continue;
                }
            }
            remained_anchors.push_back(v);
        }
    }

    /**
     * Get connection lines from the junction graph.
     */
    void GetConnectionLines(const JunctionGraph& junction_graph,
                            Array<RSegment2D>* connections) const {
        CHECK(connections);

        connections->clear();

        for (auto s : junction_graph.junction_pairs) {
            connections->emplace_back(this->junction_point(s.first),
                                      this->junction_point(s.second));
        }
        for (auto s : junction_graph.anchor_pairs) {
            connections->emplace_back(anchors_[s.first].point,
                                      anchors_[s.second].point);
        }
        for (auto s : junction_graph.junction_anchor_pairs) {
            connections->emplace_back(this->junction_point(s.first),
                                      anchors_[s.second].point);
        }
    }

    /**
     * Add connections from the junction graph with validation.
     */
    void AddConnections(const JunctionGraph& junction_graph) {
        Array<Connection> connections;
        for (auto s : junction_graph.junction_pairs) {
            RSegment2D line(this->junction_point(s.first),
                            this->junction_point(s.second));
            double w = 0.5 * (this->GetJunctionWidth(s.first) +
                              this->GetJunctionWidth(s.second));
            AddConnection(line, w, &connections);
        }
        for (auto s : junction_graph.anchor_pairs) {
            RSegment2D line(anchors_[s.first].point, anchors_[s.second].point);
            double w = 0.5 * (anchors_[s.first].width +
                              anchors_[s.second].width);
            AddConnection(line, w, &connections);
        }
        for (auto s : junction_graph.junction_anchor_pairs) {
            RSegment2D line(this->junction_point(s.first),
                            anchors_[s.second].point);
            double w = 0.5 * (this->GetJunctionWidth(s.first) +
                              anchors_[s.second].width);
            AddConnection(line, w, &connections);
        }
        std::sort(connections.begin(), connections.end());
        connections.erase(std::unique(connections.begin(), connections.end()),
                          connections.end());
        connections_.insert(connections);
    }

    /**
     * Add a connection line into connections.
     */
    bool AddConnection(const RSegment2D& line, double width,
                       Array<Connection>* connections) {
        CHECK(connections);

        bool invalid = false;
        for (auto& c : *connections) {
            if (geometry::Cross(line, c.line)) {
                invalid = true;
                break;
            }
        }
        if (invalid || !branch_connector_.IsValid(line)) {
            bad_connections_.push_back(line);
            return false;
        }

        connections->emplace_back(line, width);
        return true;
    }

    /**
     * Validate a connection line and classifies it into valid or invalid
     * connection sets.
     */
    bool AddConnection(const RSegment2D& line, Array<RSegment2D>* connections) {
        CHECK(connections);

        bool invalid = false;;
        for (auto& c : *connections) {
            if (geometry::Cross(line, c)) {
                invalid = true;
                break;
            }
        }
        if (invalid || !branch_connector_.IsValid(line)) {
            bad_connections_.push_back(line);
            return false;
        }

        connections->emplace_back(line);
        return true;
    }

    /**
     * Connect junction pairs.
     */
    bool PairConnect(const Array<int>& junctions,
                     Array<PairConnection>* connections) {
        CHECK(connections);

        Array<double> junction_width;
        for (int j : junctions) {
            if (j % 2 == 0)
                junction_width.push_back(branches_[j / 2].front_width);
            else
                junction_width.push_back(branches_[j / 2].back_width);
        }
        return branch_connector_.PairConnect(junctions, junction_width,
                                             pair_score_threshold_,
                                             connections);
    }

    /**
     * The spatial resolution (in linear units, e.g., meters) controlling the
     * densification of road polygon boundaries.
     *
     * This critical parameter defines the maximum distance between consecutive
     * vertices when discretizing the input road polygon edges. A smaller value
     * results in more densely spaced vertices along the boundaries, capturing
     * finer geometric details but increasing computational load.
     *
     * It directly influences the density of vertices in the resulting medial
     * axis: denser boundary vertices (from smaller resolution) lead to a more
     * detailed medial axis with more vertices.
     *
     * Typical values depend on road scale and required detail (e.g., 0.5-2.0
     * meters for urban road networks).
     */
    double resolution_;

    // Score threshold for pairing.
    double pair_score_threshold_ = 30.0;

    // The distance threshold to merge neighbor anchors.
    double merge_anchor_threshold_ = 5.0;

    // Original road surface.
    RMultiPolygon2D original_road_surface_;

    // Densified road surface.
    RMultiPolygon2D road_surface_;

    // Extracted raw medial axis.
    geometry::TriangulationAxis2D<double> medial_axis_;

    // Centerline branches.
    Array<Branch> branches_;

    // Junctions in the same connection.
    Array<JunctionGroup> junction_groups_;

    // Anchors for intersection optimization.
    Array<AnchorVertex> anchors_;

    // The shortest anchor path between two junctions.
    std::map<std::pair<int, int>, Array<int>> anchor_paths_;

    // Connection lines in the road intersections.
    Array<Connection> connections_;

    // Road branch connector.
    RoadBranchConnector branch_connector_;

    // Failed to find a correct connection.
    Array<RSegment2D> bad_connections_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_H_

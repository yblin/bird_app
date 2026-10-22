//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_OPTIMIZER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_OPTIMIZER_H_

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/topology/medial_axis_2d.h"
#include "codelibrary/gis/carto_polyline.h"
#include "codelibrary/gis/carto_polyline_util.h"
#include "codelibrary/gis/road_system/road_branch_connector.h"
#include "codelibrary/gis/simplification/douglas_peucker_simplifier.h"
#include "codelibrary/graph/dijkstra_shortest_paths.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace gis {

/**
 * Road centerline optimizer.
 */
class RoadCenterlineOptimizer {
    using Road = CartoPolyline2D;
    using RoadKernel = CartoPolylineKernel2D;
    using MedialAxis = geometry::MedialAxis2D<double>;
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
        // Index of this anchor in the raw medial axis.
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
        // Indices of front and back endpoints in the raw medial axis.
        int front_vertex = -1, back_vertex = -1;

        // 2D polyline representing the raw branch, which is identical to
        // the raw medial axis of the road polygon.
        RPolyline2D raw_polyline;

        // Width for each vertex of raw_polyline.
        Array<double> raw_widths;

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
     * Constructor for RoadCenterlineOptimizer.
     *
     * Parameters:
     *  aoi_polygon   - AOI polygon defining the target region for centerline
     *                  extraction.
     *  resolution    - See RoadCenterline class.
     *  max_tolerance - See RoadBranchSimplifier class.
     */
    RoadCenterlineOptimizer(const RMultiPolygon2D& aoi_polygon,
                            double resolution,
                            double max_tolerance)
        : resolution_(resolution), max_tolerance_(max_tolerance),
          branch_connector_(resolution),
          aoi_polygon_(aoi_polygon) {
        CHECK(resolution_ > 0.0);
        CHECK(max_tolerance_ > 0.0);
    }

    void clear() {
        branches_.clear();
        cutline_extra_branches_.clear();
        junction_groups_.clear();
        anchors_.clear();
        anchor_paths_.clear();
        connections_.clear();
        branch_connector_.clear();
        bad_connections_.clear();
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
    void Optimize(const RMultiPolygon2D& road, const MedialAxis& medial_axis) {
        this->clear();

        Array<Array<int>> branch_vertices;
        Array<RPolyline2D> branches;
        medial_axis.ToBranches(&branch_vertices, &branches);
        if (branches.empty()) return;

        std::unordered_set<RPoint2D> fixed_endpoints;
        const auto& npv = medial_axis.non_prunable_vertices();

        const auto& axis = medial_axis.axis();
        for (auto v : axis.vertices()) {
            if (npv.count(v.id)) fixed_endpoints.insert(v.point);
        }

        Array<double> vertex_width(axis.n_allocated_vertices(), 0.0);
        for (auto v : axis.vertices()) {
            vertex_width[v.id] = 2.0 * medial_axis.radius(v.id);
        }

        // Compute the simplified and exended raw branch.
        branch_connector_.ResetRoadPolygon(road);

        DouglasPeuckerSimplifier simplifer(max_tolerance_);
        for (int i = 0; i < branches.size(); ++i) {
            Array<double> widths(branches[i].size());
            for (auto&& [index, v] : enumerate(branch_vertices[i])) {
                widths[index] = vertex_width[v];
            }
            if (branches[i].size() <= 1) continue;

            RPolyline2D split_branch;
            Array<double> split_widths;
            Array<bool> fixed;
            SplitByAOI(branches[i], widths, &split_branch, &split_widths,
                       &fixed);

            // Fix bug: If a centerline intersects the AOI boundary more than
            // twice, only keep the first fixed point. This is to eliminate the
            // bug where the simplified centerline sticks to the AOI border.
            int fixed_point_count = 0;
            int first_fixed_index = -1;
            for (int idx = 0; idx < fixed.size(); ++idx) {
                if (fixed[idx]) {
                    fixed_point_count++;
                    // Record the index of the FIRST fixed point only
                    if (first_fixed_index == -1) {
                        first_fixed_index = idx;
                    }
                }
            }
            if (fixed_point_count > 1) {
                for (int idx = 0; idx < fixed.size(); ++idx) {
                    fixed[idx] = (idx == first_fixed_index);
                }
            }

            branches[i] = split_branch;
            widths = split_widths;

            Road simplified_branch;
            Road road_branch;
            road_branch.polyline = branches[i];
            road_branch.widths = widths;
            simplifer.Simplify(road_branch, fixed, &simplified_branch);

            int v1 = branch_vertices[i].front();
            int v2 = branch_vertices[i].back();

            if (axis.edges_from(v1).size() == 1 &&
                !fixed_endpoints.count(branches[i].front())) {
                double w = widths.front();
                ExtendBranchFront(w, &simplified_branch.polyline);
            }
            if (axis.edges_from(v2).size() == 1 &&
                !fixed_endpoints.count(branches[i].back())) {
                double w = widths.back();
                ExtendBranchBack(w, &simplified_branch.polyline);
            }

            Branch branch;
            branch.raw_widths = widths;
            branch.raw_polyline = branches[i];
            branch.simplified_raw_polyline = simplified_branch.polyline;
            branch.front_vertex = v1;
            branch.back_vertex = v2;
            if (fixed_endpoints.count(branch.raw_polyline.front()) ||
                fixed_endpoints.count(branch.raw_polyline.back())) {
                branch.can_not_ignore = true;
            }

            branches_.push_back(branch);
        }

        ComputeBackoffScale(medial_axis);
        ShrinkBranches(medial_axis);
        GroupJunctions();
        ConstructAnchorGraph(medial_axis);

        Array<RPolyline2D> polylines(branches_.size());
        for (int i = 0; i < polylines.size(); ++i) {
            polylines[i] = branches_[i].polyline;
        }
        branch_connector_.ResetBranches(polylines);
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

        // Treat cutline (fixed) points as fixed endpoints: shrink the wiggly
        // centerline near them and reconnect with straight lines.
        ShrinkConnectFixedSegments(fixed_endpoints);
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
            w = branch.raw_widths[v];
        }
        return w;
    }

    /**
     * Decompose the road centerline into multiple road kernels, where each
     * sub-array corresponds to a complete road.
     */
    void ToRoadKernels(Array<Array<RoadKernel>>* kernels) const {
        CHECK(kernels);

        kernels->clear();

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

        // Extra cores produced when a branch was split at an interior cutline
        // point by ShrinkConnectFixedSegments().
        for (const auto& branch : cutline_extra_branches_) {
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

        Array<RoadKernel> segs;
        for (Array<int>& c : chains) {
            if (visited[c.front()]) continue;
            Road road;
            for (int c1 : c) {
                visited[c1] = true;
                visited[graph.TwinEdge(c1)] = true;
                if (road.polyline.empty()) {
                    road.polyline.push_back(graph.source_point(c1));
                    road.widths.push_back(w[graph.source(c1)]);
                }
                road.polyline.push_back(graph.target_point(c1));
                road.widths.push_back(w[graph.target(c1)]);
            }

            carto_polyline_util::DecomposeToKernels(road, resolution_, &segs);
            // Donot forget to simplify the branch polyline.
            for (auto& s : segs) {
                geometry::polyline_util::DouglasPeuckerSimplify(s.polyline,
                                                                0.01,
                                                                &s.polyline);
            }
            kernels->push_back(segs);
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
     * Split road branch at the AOI boundary and mark intersection vertices as
     * fixed (for subsequent simplification).
     */
    void SplitByAOI(const RPolyline2D& polyline,
                    const Array<double>& widths,
                    RPolyline2D* split_polyline,
                    Array<double>* split_widths,
                    Array<bool>* is_fixed) const {
        CHECK(split_polyline);
        CHECK(split_widths);
        CHECK(polyline.size() == widths.size());

        if (polyline.size() < 2 || aoi_polygon_.empty()) {
            *split_polyline = polyline;
            *split_widths = widths;
            is_fixed->assign(polyline.size(), false);
            return;
        }

        split_polyline->clear();
        split_widths->clear();
        is_fixed->clear();
        split_polyline->push_back(polyline.front());
        split_widths->push_back(widths.front());
        is_fixed->push_back(true);
        for (int i = 0; i + 1 < polyline.size(); ++i) {
            RSegment2D seg(polyline[i], polyline[i + 1]);
            double w = (widths[i] + widths[i + 1]) * 0.5;
            Array<RPoint2D> inters;
            for (const auto& boundary : aoi_polygon_.boundaries()) {
                RPolyline2D poly(boundary.polygon.vertices());
                poly.push_back(poly.front());

                Array<RPoint2D> intersections;
                geometry::Intersect(seg, poly, &intersections);
                inters.insert(intersections);
            }
            geometry::PointDotCompare2D<double> compare(polyline[i],
                                                        polyline[i + 1]);
            std::sort(inters.begin(), inters.end(), compare);
            for (auto p : inters) {
                if (p != split_polyline->back()) {
                    split_polyline->push_back(p);
                    split_widths->push_back(w);
                    is_fixed->push_back(true);
                }
            }
            if (polyline[i + 1] != split_polyline->back()) {
                split_polyline->push_back(polyline[i + 1]);
                split_widths->push_back(w);
                is_fixed->push_back(false);
            }
        }
    }

    /**
     * Extend the branch from its front endpoint.
     */
    void ExtendBranchFront(double w, RPolyline2D* branch) const {
        CHECK(branch);

        if (branch->size() <= 1) return;
        if (w < resolution_) return;

        RPoint2D& p = branch->front();
        RPoint2D q;
        RVector2D v = (*branch)[0] - (*branch)[1];

        if (branch_connector_.ConnectBranchToBoundary(
                    p, v, boundary_extend_scale_ * w, &q)) {
            // Snap the endpoint onto the polygon boundary so that the
            // centerline reaches the border (no reserved gap).
            p = q;
        } else {
            p += v.normalized() * w;
        }
    }

    /**
     * Extend the branch from its back endpoint.
     */
    void ExtendBranchBack(double w, RPolyline2D* branch) const {
        CHECK(branch);

        if (branch->size() <= 1) return;
        if (w < resolution_) return;

        RPoint2D& p = branch->back();
        RPoint2D q;
        RVector2D v = branch->back() - (*branch)[branch->size() - 2];
        if (branch_connector_.ConnectBranchToBoundary(
                    p, v, boundary_extend_scale_ * w, &q)) {
            // Snap the endpoint onto the polygon boundary so that the
            // centerline reaches the border (no reserved gap).
            p = q;
        } else {
            p += v.normalized() * w;
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
        widths->push_back(branch.raw_widths[indices.front()]);
        for (int i = 0; i + 1 < indices.size(); ++i) {
            int l = indices[i], r = indices[i + 1];
            CHECK(l <= r);

            Array<RPoint2D> segs;
            geometry::EvenlyPointsOnSegment2D(polyline[i], polyline[i + 1],
                                              std::max(2, r - l + 1),
                                              &segs);
            for (int j = 1; j < segs.size(); ++j) {
                densified_polyline->push_back(segs[j]);
                widths->push_back(branch.raw_widths[std::min(l + j, r)]);
            }
        }
        if (widths->size() == 1) {
            densified_polyline->push_back(polyline.back());
            widths->push_back(branch.raw_widths[indices.back()]);
        }

        CHECK(densified_polyline->size() == widths->size());
    }

    /**
     * Per-vertex width of a branch's non_junction_polyline (no densification),
     * by matching each vertex back to the raw polyline.
     */
    void NonJunctionVertexWidths(const Branch& branch,
                                 Array<double>* widths) const {
        const RPolyline2D& P = branch.non_junction_polyline;
        widths->assign(P.size(), 0.0);
        if (P.empty() || branch.raw_polyline.empty()) return;

        int start = 0;
        for (int i = 0; i < P.size(); ++i) {
            int found = -1;
            for (int j = start; j < branch.raw_polyline.size(); ++j) {
                if (P[i] == branch.raw_polyline[j]) {
                    found = j;
                    start = j;
                    break;
                }
            }
            if (found == -1) {
                double best = DBL_MAX;
                for (int j = 0; j < branch.raw_polyline.size(); ++j) {
                    double d = Distance(P[i], branch.raw_polyline[j]);
                    if (d < best) { best = d; found = j; }
                }
            }
            (*widths)[i] = branch.raw_widths[found];
        }
    }

    /**
     * Validate a connection line whose endpoint may lie ON the road boundary
     * (a cutline point). branch_connector_.IsValid() treats an endpoint that
     * touches the boundary as a crossing, so shorten the segment slightly at
     * both ends before testing; a real interior crossing is still detected.
     */
    bool ValidCutlineLine(const RSegment2D& line) const {
        const RPoint2D a = line.lower_point(), b = line.upper_point();
        const double len = Distance(a, b);
        if (len < 1e-9) return false;
        const double t = std::min(0.01, 0.25 * len) / len;
        RPoint2D na(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        RPoint2D nb(b.x + (a.x - b.x) * t, b.y + (a.y - b.y) * t);
        return branch_connector_.IsValid(RSegment2D(na, nb));
    }

    /**
     * Treat each cutline endpoint (a non-prunable / fixed point) as a fixed
     * point on the centerline. A branch is split by the fixed points into
     * segments; each segment is shrunk back from its fixed end(s) by a generous
     * backoff, and the trimmed core is reconnected to the fixed point(s) with a
     * straight line -- so the wiggly medial axis near a cutline becomes a clean
     * straight approach to the (fixed) cutline center. A segment whose stable
     * core vanishes collapses to a single straight line between its fixed ends.
     *
     * A connection (or collapse) that would leave the road (IsValid == false)
     * is rejected and that segment is kept unchanged. Trimming keeps the core's
     * interior vertices (raw-polyline vertices, required by
     * DensifyNonJunctionPolyline); only the trimmed endpoints are interpolated.
     * Cores beyond the first (from interior splits) go to
     * cutline_extra_branches_.
     */
    void ShrinkConnectFixedSegments(
            const std::unordered_set<RPoint2D>& fixed_endpoints) {
        // Backoff near a cutline is more aggressive than at a junction
        // (junction uses ~0.75 * width); tune here if needed.
        const double backoff_scale = 2.0;

        Array<Connection> new_connections;
        const int n0 = branches_.size();
        for (int bi = 0; bi < n0; ++bi) {
            const RPolyline2D polyline = branches_[bi].non_junction_polyline;
            if (polyline.size() < 2) continue;

            Array<int> fixed_idx;
            for (int i = 0; i < polyline.size(); ++i) {
                if (fixed_endpoints.count(polyline[i])) fixed_idx.push_back(i);
            }
            if (fixed_idx.empty()) continue;

            Array<double> w;
            NonJunctionVertexWidths(branches_[bi], &w);

            std::set<int> bset;
            bset.insert(0);
            bset.insert(polyline.size() - 1);
            for (int f : fixed_idx) bset.insert(f);
            Array<int> bounds(bset.begin(), bset.end());
            std::unordered_set<int> fixed_set(fixed_idx.begin(),
                                              fixed_idx.end());

            Array<RPolyline2D> cores;
            for (int s = 0; s + 1 < bounds.size(); ++s) {
                const int a = bounds[s], b = bounds[s + 1];
                const bool af = fixed_set.count(a) > 0;
                const bool bf = fixed_set.count(b) > 0;

                // Shrink the segment back from its fixed end(s) by the backoff
                // distance using polyline_util::Split (the trimmed point becomes
                // a core endpoint, matched by nearest in DensifyNonJunctionPolyline).
                RPolyline2D core(polyline.begin() + a, polyline.begin() + b + 1);
                if (af && w[a] > 0.0) {
                    RPolyline2D removed, rest;
                    geometry::polyline_util::Split(core, backoff_scale * w[a],
                                                   &removed, &rest);
                    core = rest;
                }
                if (bf && w[b] > 0.0 && core.size() >= 2) {
                    const double keep = core.Length() - backoff_scale * w[b];
                    if (keep > 0.0) {
                        RPolyline2D kept, removed;
                        geometry::polyline_util::Split(core, keep, &kept,
                                                       &removed);
                        core = kept;
                    } else {
                        core.clear();
                    }
                }

                const bool stable =
                        core.size() >= 2 && core.Length() >= resolution_;

                if (stable) {
                    Array<Connection> seg_conns;
                    bool ok = true;
                    if (af && polyline[a] != core.front()) {
                        RSegment2D line(polyline[a], core.front());
                        if (ValidCutlineLine(line)) {
                            seg_conns.emplace_back(line, w[a]);
                        } else {
                            ok = false;
                        }
                    }
                    if (ok && bf && core.back() != polyline[b]) {
                        RSegment2D line(core.back(), polyline[b]);
                        if (ValidCutlineLine(line)) {
                            seg_conns.emplace_back(line, w[b]);
                        } else {
                            ok = false;
                        }
                    }
                    if (ok) {
                        for (auto& c : seg_conns)
                            new_connections.push_back(c);
                        cores.push_back(core);
                        continue;
                    }
                } else if (polyline[a] != polyline[b]) {
                    RSegment2D line(polyline[a], polyline[b]);
                    if (ValidCutlineLine(line)) {
                        new_connections.emplace_back(line, 0.5 * (w[a] + w[b]));
                        continue;
                    }
                }

                // Fallback: keep the original segment (it still reaches the
                // fixed point(s) via its own polyline).
                cores.push_back(RPolyline2D(polyline.begin() + a,
                                            polyline.begin() + b + 1));
            }

            if (cores.empty()) {
                branches_[bi].non_junction_polyline.clear();
            } else {
                branches_[bi].non_junction_polyline = cores[0];
                for (int k = 1; k < cores.size(); ++k) {
                    Branch extra;
                    extra.raw_polyline = branches_[bi].raw_polyline;
                    extra.raw_widths = branches_[bi].raw_widths;
                    extra.non_junction_polyline = cores[k];
                    cutline_extra_branches_.push_back(extra);
                }
            }
        }

        for (auto& c : new_connections)
            connections_.push_back(c);
    }

    /**
     * Compute the backoff scale for each branch.
     */
    void ComputeBackoffScale(const MedialAxis& medial_axis) {
        for (auto& branch : branches_) {
            double backoff = 0.5 * branch.backoff_scale *
                             (branch.raw_widths.front() +
                              branch.raw_widths.back());
            if (backoff > branch.simplified_raw_polyline.Length()) {
                RSegment2D line(branch.raw_polyline.front(),
                                branch.raw_polyline.back());
                if (branch.can_not_ignore || !branch_connector_.IsValid(line)) {
                    branch.can_not_ignore = true;
                    branch.backoff_scale = 1.0;
                    branch.backoff_scale = EvaluateBackoffScale(medial_axis,
                                                                branch);
                }
            }
        }

        ShrinkBranches(medial_axis);
        GroupJunctions();
        ConstructAnchorGraph(medial_axis);

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
                                EvaluateBackoffScale(medial_axis,
                                                     branches_[j / 2]);
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
    void ShrinkBranches(const MedialAxis& medial_axis) {
        const auto& axis = medial_axis.axis();

        for (int i = 0; i < branches_.size(); ++i) {
            Branch& branch = branches_[i];
            branch.polyline = branch.simplified_raw_polyline;
            if (axis.edges_from(branch.front_vertex).size() > 2) {
                double backoff = branch.backoff_scale *
                                 0.5 * branch.raw_widths.front();
                RPolyline2D poly2;
                geometry::polyline_util::Split(branch.polyline, backoff,
                                               &branch.front_backoff_polyline,
                                               &poly2);
                UpdateBranch(i, poly2);
                if (branch.polyline.empty()) {
                    continue;
                }
            }

            if (axis.edges_from(branch.back_vertex).size() > 2) {
                double backoff = branch.backoff_scale *
                                 0.5 * branch.raw_widths.back();
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
    double EvaluateBackoffScale(const MedialAxis& medial_axis,
                                const Branch& branch) const {
        const auto& axis = medial_axis.axis();

        const double min_backoff_scale = 0.1;

        double backoff_scale = branch.backoff_scale;
        while (backoff_scale > min_backoff_scale) {
            RPolyline2D branch_polyline = branch.simplified_raw_polyline;
            if (axis.edges_from(branch.front_vertex).size() > 2) {
                double backoff = backoff_scale * 0.5 *
                                 branch.raw_widths.front();
                RPolyline2D poly1, poly2;
                geometry::polyline_util::Split(branch_polyline, backoff,
                                               &poly1, &poly2);
                branch_polyline = poly2;
                if (branch_polyline.size() < 2) {
                    backoff_scale *= 0.5;
                    continue;
                }
            }

            if (axis.edges_from(branch.back_vertex).size() > 2) {
                double backoff = backoff_scale * 0.5 * branch.raw_widths.back();
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
            junction_group[branch.front_vertex].push_back(2 * index);
            junction_group[branch.back_vertex].push_back(2 * index + 1);
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
    void ConstructAnchorGraph(const MedialAxis& medial_axis) {
        anchors_.clear();
        anchor_paths_.clear();

        const auto& axis = medial_axis.axis();
        Array<int> axis_vertex_to_anchor(axis.n_allocated_vertices(), -1);
        for (auto v : axis.vertices()) {
            if (axis.edges_from(v.id).size() >= 3) {
                AnchorVertex anchor;
                anchor.id = v.id;
                anchor.point = v.point;
                anchor.width = 2.0 * medial_axis.radius(v.id);
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
                int v1 = axis_vertex_to_anchor[branch.front_vertex];
                if (v1 == -1) continue;
                v1 += n_junctions;
                int v2 = axis_vertex_to_anchor[branch.back_vertex];
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
                int v = axis_vertex_to_anchor[branch.front_vertex];
                CHECK(v != -1);
                v += n_junctions;
                auto e = anchor_graph.InsertTwoWayEdge(junction1, v);
                weight[e] = weight[e->twin()] = l;
            }

            if (!branch.back_backoff_polyline.empty()) {
                double l = branch.back_backoff_polyline.Length();
                int v = axis_vertex_to_anchor[branch.back_vertex];
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

    // The spatial resolution (in linear units, e.g., meters) controlling the
    // densification of road polygon boundaries.
    //
    // This critical parameter defines the maximum distance between consecutive
    // vertices when discretizing the input road polygon edges. A smaller value
    // results in more densely spaced vertices along the boundaries, capturing
    // finer geometric details but increasing computational load.
    //
    // It directly influences the density of vertices in the resulting medial
    // axis: denser boundary vertices (from smaller resolution) lead to a more
    // detailed medial axis with more vertices.
    //
    // Typical values depend on road scale and required detail (e.g., 0.5-2.0
    // meters for urban road networks).
    double resolution_;

    // See RoadBranchSimplifier.
    double max_tolerance_;

    // Scale applied to the branch width when searching for the polygon
    // boundary in ExtendBranchFront()/ExtendBranchBack(). A value > 1 lengthens
    // the directional search ray so the endpoint can still reach the boundary
    // when the medial-axis leaf was pruned back from the true road end.
    double boundary_extend_scale_ = 3.0;

    // Score threshold for pairing.
    double pair_score_threshold_ = 30.0;

    // The distance threshold to merge neighbor anchors.
    double merge_anchor_threshold_ = 5.0;

    // Centerline branches.
    Array<Branch> branches_;

    // Extra centerline cores produced when ShrinkConnectFixedSegments() splits
    // a branch at an interior cutline (fixed) point. Only ToRoadKernels() reads
    // them; they are kept out of branches_ so the raw-centerline output is not
    // duplicated.
    Array<Branch> cutline_extra_branches_;

    // Junctions in the same connection.
    Array<JunctionGroup> junction_groups_;

    // Anchors for intersection optimization.
    Array<AnchorVertex> anchors_;

    // The shortest anchor path between two junctions.
    std::map<std::pair<int, int>, Array<int>> anchor_paths_;

    // Connection lines in the road intersections.
    Array<Connection> connections_;

    // Failed to find a correct connection.
    Array<RSegment2D> bad_connections_;

    // Road branch connector.
    RoadBranchConnector branch_connector_;

    // AOI (Area of Interest) polygon: target region for centerline final
    // storage.
    // Note: Centerlines are extracted from a larger area first (clipped to AOI
    //       later), and this class does NOT handle clipping.
    RMultiPolygon2D aoi_polygon_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_OPTIMIZER_H_

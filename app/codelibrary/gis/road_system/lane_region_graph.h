//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_LANE_GRAPH_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_LANE_GRAPH_H_

#include <set>

#include "codelibrary/geometry/topology/geo_graph.h"

namespace cl {
namespace gis {

/**
 * A graph structure specialized for modeling topological relationships between
 * lane regions, built on the general GeoGraph framework to leverage its support
 * for complex spatial relationships.
 *
 * It also capture lane-specific semantics, including:
 * - Classification of edges/vertices as cutlines or cutpoints (critical for
 *   lane continuity).
 * - Explicit adjacency relationships between lane regions via a dedicated
 *   adjacency graph.
 *
 * Typically extracted from LaneArrangement (see lane_arrangement.h for
 * details).
 */
class LaneRegionGraph {
    using GeoGraph = geometry::GeoGraph<RPoint2D>;
    using VertexIndex = GeoGraph::VertexIndex;
    using EdgeIndex = GeoGraph::EdgeIndex;

public:
    /**
     * A continuous polyline composed of consecutive edges, forming part of a
     * lane region's boundary.
     */
    struct Polyedge {
        bool is_cutline;

        // Indices of consecutive edges (from base_graph) that form this
        // continuous polyedge.
        Array<int> edges;

        // Polyline version of the edges.
        RPolyline2D polyline;
    };

    /**
     * Lane region.
     */
    struct Region {
        // Unique ID for the lane region.
        // This ID is consistent with the face ID of the region in the
        // base_graph.
        int id;

        // Collection of boundary polyedges that form the closed boundary of the
        // lane region. It supports regions with holes.
        Array<Array<Polyedge>> rings;

        // Cutpoints on the region.
        Array<int> cutpoints;
    };

    /**
     * A struct representing an edge in the region adjacency graph, capturing
     * two adjacent regions and the boundary edges connecting them.
     */
    struct RegionAdjacency {
        int region;           // Current region ID.
        int adjacency_region; // Adjacent region ID.

        // Store indices of edges in the base_graph that form the boundary
        // between two regions.
        Array<EdgeIndex> boundary_edges;

        // Store the index of the vertex in the base_graph that acts as
        // the degenerate boundary (cutpoint) between two regions.
        VertexIndex cutpoint_vertex = -1;
    };

    LaneRegionGraph() = default;

    /**
     * Reset lane-region graph.
     */
    void Reset(const GeoGraph& geo_graph,
               const std::unordered_map<int, bool>& is_cutline) {
        this->clear();

        is_cutline_ = is_cutline;
        base_graph_ = geo_graph;
        Initialize();
    }

    void clear() {
        is_cutline_.clear();
        is_cutpoint_.clear();
        base_graph_.clear();
        region_adjacencies_.clear();
        regions_.clear();
    }

    /**
     * Extracts cutline polyedges that can be safely ignored (and removed) when
     * merging lane regions.
     *
     * These edges are part of a shared cutline between two regions, and their
     * removal won't break the core property of the merged region (i.e.,
     * retaining two valid cutlines).
     */
    bool ExtractIgnoreableCutlines(Array<int>* ignoreable_cutlines) const {
        CHECK(ignoreable_cutlines);
        CHECK(regions_.size() == region_adjacencies_.size());

        ignoreable_cutlines->clear();

        int n_regions = regions_.size();
        Array<bool> is_dual_cutlines_lane(n_regions);
        for (int i = 0; i < n_regions; ++i) {
            is_dual_cutlines_lane[i] = IsDualCutlinesLane(i);
        }

        for (int i = 0; i < n_regions; ++i) {
            if (!is_dual_cutlines_lane[i]) continue;

            for (const auto& e : region_adjacencies_[i]) {
                int j = e.adjacency_region;
                if (j <= i) continue;
                if (!is_dual_cutlines_lane[j]) continue;
                if (e.boundary_edges.empty()) continue;

                bool can_be_ignored = true;
                for (int e : e.boundary_edges) {
                    if (!is_cutline(e) || is_cutpoint(base_graph_.source(e)) ||
                        is_cutpoint(base_graph_.target(e))) {
                        can_be_ignored = false;
                        break;
                    }
                }
                if (!can_be_ignored) continue;

                std::set<int> set1(e.boundary_edges.begin(),
                                   e.boundary_edges.end());
                std::set<int> set2;
                for (int e : e.boundary_edges) {
                    int e1 = base_graph_.TwinEdge(e);
                    CHECK(e1 != -1);
                    set2.insert(e1);
                }

                Array<std::set<int>> cutlines1, cutlines2;
                for (auto e : regions_[i].rings.front()) {
                    if (e.is_cutline) {
                        cutlines1.emplace_back(e.edges.begin(), e.edges.end());
                    }
                }
                for (auto e : regions_[j].rings.front()) {
                    if (e.is_cutline) {
                        cutlines2.emplace_back(e.edges.begin(), e.edges.end());
                    }
                }
                CHECK(cutlines1.size() <= 2);
                CHECK(cutlines2.size() <= 2);

                if (cutlines1.find(set1) != cutlines1.end() &&
                    cutlines2.find(set2) != cutlines2.end()) {

                    ignoreable_cutlines->insert(e.boundary_edges);
                }
            }
        }
    }

    /**
     * Check if the specified lane region is a "dual-cutline lane region" — a
     * lane region bounded by exactly two cutlines (including both regular
     * cutlines and degenerate cutpoints).
     */
    bool IsDualCutlinesLane(int i) const {
        CHECK(i >= 0 && i < regions_.size());

        int n_cutlines = 0;
        if (regions_[i].rings.size() >= 2) return false;

        CHECK(!regions_[i].rings.empty());

        for (auto& polyedge : regions_[i].rings.front()) {
            if (polyedge.is_cutline) ++n_cutlines;
        }

        return n_cutlines + regions_[i].cutpoints.size() == 2;
    }

    int n_lane_regions() const {
        return regions_.size();
    }

    /**
     * Return true if the given edge is a cutline.
     */
    bool is_cutline(int e) const {
        auto it = is_cutline_.find(e);
        return (it != is_cutline_.end()) ? it->second : false;
    }

    /**
     * Return true if the given vertex is a cutpoint.
     */
    bool is_cutpoint(int v) const {
        auto it = is_cutpoint_.find(v);
        return (it != is_cutpoint_.end()) ? it->second : false;
    }

    const GeoGraph& base_graph() const {
        return base_graph_;
    }

    const Array<Array<RegionAdjacency>>& region_adjacencies() const {
        return region_adjacencies_;
    }

    const Array<Region>& regions() const {
        return regions_;
    }

private:
    /**
     * Extract boundary polyedges from the input edge sequence, grouping
     * consecutive edges of the same type (cutline or normal boundary) into
     * ordered Polyedge instances.
     */
    void ExtractRegionBoundary(const Array<int>& edges,
                               Array<Polyedge>* polyedges) const {
        CHECK(polyedges);

        polyedges->clear();

        const int n = edges.size();
        if (n == 0) return;

        // Mark which edges are cutlines
        Array<bool> is_cutline_edge(n);
        for (int i = 0; i < edges.size(); ++i) {
            is_cutline_edge[i] = this->is_cutline(edges[i]);
        }

        Polyedge polyedge;
        for (int i = 0; i < n; ++i) {
            if (polyedge.edges.empty()) {
                polyedge.is_cutline = is_cutline_edge[i];
            } else if (polyedge.is_cutline != is_cutline_edge[i]) {
                polyedges->push_back(polyedge);

                polyedge = Polyedge{};
                polyedge.is_cutline = is_cutline_edge[i];
            }
            polyedge.edges.push_back(edges[i]);
        }
        if (!polyedge.edges.empty()) {
            polyedges->push_back(polyedge);
        }

        if (polyedges->size() <= 1) return;

        // Handle cyclic continuity: merge first and last if same type.
        if (polyedges->front().is_cutline == polyedges->back().is_cutline) {
            polyedges->front().edges.insert(polyedges->front().edges.begin(),
                                            polyedges->back().edges.begin(),
                                            polyedges->back().edges.end());
            polyedges->pop_back();
        }

        for (auto& polyedge : *polyedges) {
            for (int e : polyedge.edges) {
                if (polyedge.polyline.empty()) {
                    polyedge.polyline.push_back(base_graph_.source_point(e));
                }
                polyedge.polyline.push_back(base_graph_.target_point(e));
            }
        }
    }

    /**
     * Initialize the lane-region graph.
     */
    void Initialize() {
        if (is_cutline_.empty() || base_graph_.empty()) return;

        // Compute region adjancecies.
        region_adjacencies_.resize(base_graph_.n_faces());

        for (auto v : base_graph_.vertices()) {
            const auto& edges = base_graph_.edges_from(v.id);
            if (edges.size() < 2) continue;

            int e1 = edges[0];
            const auto& rings1 = base_graph_.rings_of_edge(e1);
            for (int i = 1; i < edges.size(); ++i) {
                int e2 = edges[i];
                const auto& rings2 = base_graph_.rings_of_edge(e2);

                if (rings1 == rings2) continue;

                for (auto r1 : rings1) {
                    for (auto r2 : rings2) {
                        int f1 = base_graph_.ring(r1).face;
                        int f2 = base_graph_.ring(r2).face;
                        if (f1 == f2) continue;

                        int e3 = base_graph_.PreviousEdgeOnRing(r1, e1);
                        if (e3 == -1) continue;

                        int e4 = base_graph_.PreviousEdgeOnRing(r2, e2);
                        if (e4 == -1) continue;

                        if (base_graph_.TwinEdge(e1) == e2) continue;
                        if (base_graph_.TwinEdge(e1) == e4) continue;
                        if (base_graph_.TwinEdge(e3) == e2) continue;
                        if (base_graph_.TwinEdge(e3) == e4) continue;

                        int cnt = is_cutline_[e1] + is_cutline_[e2] +
                                  is_cutline_[e3] + is_cutline_[e4];
                        if (cnt != 1) continue;

                        InsertCutpointAdjacency(f1, f2, v.id);
                        InsertCutpointAdjacency(f2, f1, v.id);
                        is_cutpoint_[v.id] = true;
                    }
                }
            }
        }

        for (auto e : base_graph_.edges()) {
            int e1 = base_graph_.TwinEdge(e.id);
            if (e1 == -1) continue;

            Array<int> regions1, regions2;
            base_graph_.GetFacesOfEdge(e.id, &regions1);
            base_graph_.GetFacesOfEdge(e1, &regions2);
            if (regions1.empty() || regions2.empty()) continue;

            for (auto r1 : regions1) {
                for (auto r2 : regions2) {
                    if (r1 == r2) continue;

                    InsertEdgeAdjacency(r1, r2, e.id);
                }
            }
        }

        for (auto f : base_graph_.faces()) {
            Region region;
            region.id = f.id;
            Array<Polyedge> boundary;
            ExtractRegionBoundary(base_graph_.ring(f.outer_ring).edges,
                                  &boundary);
            region.rings.push_back(boundary);
            for (auto ring : f.inner_rings) {
                ExtractRegionBoundary(base_graph_.ring(ring).edges, &boundary);
                region.rings.push_back(boundary);
            }

            for (auto& ring : region.rings) {
                for (auto& polyedge : ring) {
                    for (int e : polyedge.edges) {
                        if (is_cutpoint(base_graph_.source(e))) {
                            region.cutpoints.push_back(base_graph_.source(e));
                        }
                    }
                }
            }
            regions_.push_back(region);
        }
    }

    /**
     * Insert a boundary edge-based adjacency into region adjacencies.
     */
    void InsertEdgeAdjacency(int r1, int r2, int edge) {
        CHECK(r1 != r2);

        bool found = false;
        for (auto& r : region_adjacencies_[r1]) {
            if (r.adjacency_region == r2) {
                found = true;
                r.boundary_edges.push_back(edge);
                break;
            }
        }
        if (!found) {
            RegionAdjacency r;
            r.region = r1;
            r.adjacency_region = r2;
            r.boundary_edges.push_back(edge);
            region_adjacencies_[r1].push_back(r);
        }
    }

    /**
     * Insert a cutpoint-based adjacency into region adjacencies.
     */
    void InsertCutpointAdjacency(int r1, int r2, int v) {
        CHECK(r1 != r2);

        bool found = false;
        for (auto& r : region_adjacencies_[r1]) {
            if (r.adjacency_region == r2) {
                found = true;
                r.cutpoint_vertex = v;
                break;
            }
        }
        if (!found) {
            RegionAdjacency r;
            r.region = r1;
            r.adjacency_region = r2;
            r.cutpoint_vertex = v;
            region_adjacencies_[r1].push_back(r);
        }
    }

    // Indicate whether the edge of the graph is a "cut line" — a specialized
    // edge that marks the longitudinal endpoint of a LaneRegion.
    std::unordered_map<VertexIndex, bool> is_cutline_;

    // Identify vertices that act as "cut points".
    // A cut point is a degenerate form of a cut line: it arises when a cut line
    // (originally an edge) collapses into a single point.
    std::unordered_map<EdgeIndex, bool> is_cutpoint_;

    // Category for each lane region.
    //Array<std::string> categories_;

    // Underlying geometric graph storing the base topology for lane regions:
    // - Vertices: Geometric points (e.g., vertices of lane polygons).
    // - Edges: Line segments (e.g., edge of lane polygons).
    // - Faces: Polygonal faces, each representing a distinct lane region.
    GeoGraph base_graph_;

    // Adjacencies explicitly modeling relationships between lane regions.
    Array<Array<RegionAdjacency>> region_adjacencies_;

    // Lane regions.
    Array<Region> regions_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_LANE_GRAPH_H_

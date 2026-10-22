//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_REGION_GRAPH_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_REGION_GRAPH_H_

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>

#include "codelibrary/graph/graph.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/topology/geo_graph.h"
#include "codelibrary/geometry/topology/geo_graph_util.h"
#include "codelibrary/geometry/util/snap_2d.h"
#include "codelibrary/gis/lane_system/lane_arrangement.h"
#include "codelibrary/util/set/disjoint_set.h"

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
 */
class LaneRegionGraph {
    using GeoGraph = geometry::GeoGraph<RPoint2D>;
    using VertexIndex = GeoGraph::VertexIndex;
    using EdgeIndex = GeoGraph::EdgeIndex;

public:
    /**
     * Type enumeration for lane region boundaries.
     */
    enum BoundaryType {
        // Unknown type.
        UNKNOWN = 0,
        // Functional boundary line used to divide lane regions, specifically
        // for separating front and rear adjacent lanes.
        CUTLINE = 1,
        // Non-cutline lane line that serves as an auxiliary boundary,
        // specifically for separating left and right adjacent lanes.
        ASSIST_LINE = 2,
        // Outermost edge of the road area, with no adjacent lane/land region on
        // one side.
        OUTERMOST_ROAD = 3,
        // Boundary of a gore area, used for special processing to identify
        // shoulder cutlines.
        GORE_AREA_BORDER = 4,
        // Boundary of a raised or vegetation median, used for special
        // processing to identify protected bike lanes.
        RAISED_MEDIAN_BORDER = 5,
        // Boundary of painted median, used for special processing to identify
        // buffered bike lanes.
        PAINTED_MEDIAN_BORDER = 6,
        // Boundary of a land region that does not fall into above categories.
        LAND_REGION_BORDER = 7,
    };

    /**
     * Return the boundary type info.
     */
    static std::string GetBoundaryTypeName(BoundaryType type) {
        switch (type) {
        case UNKNOWN: return "unknown";
        case CUTLINE: return "cutline";
        case ASSIST_LINE: return "assist_line";
        case OUTERMOST_ROAD: return "outermost_road";
        case GORE_AREA_BORDER: return "gore_area_border";
        case LAND_REGION_BORDER: return "land_region_border";
        case RAISED_MEDIAN_BORDER: return "raised_median_border";
        case PAINTED_MEDIAN_BORDER: return "painted_median_border";
        }
        return "unknown";
    }

    /**
     * A continuous polyline composed of consecutive edges, forming part of a
     * lane region's boundary.
     */
    struct Polyedge {
        // Type identifier of this polyedge.
        // This type may differ from the type of individual edges in the
        // 'edges' array, as it accounts for special handling of multi-level
        // regions.
        BoundaryType type = UNKNOWN;

        // Indices of consecutive edges (from base_graph) that form this
        // continuous polyedge.
        Array<int> edges;

        // Polyline version of the edges.
        RPolyline2D polyline;

        Polyedge() = default;

        bool empty() const {
            return edges.empty();
        }

        /**
         * This polyedge is a cutline.
         */
        bool is_cutline() const {
            return type == CUTLINE;
        }

        /**
         * Return true if this polyedge is a border of road.
         */
        bool is_road_border() const {
            return type == OUTERMOST_ROAD ||
                   type == GORE_AREA_BORDER ||
                   type == LAND_REGION_BORDER ||
                   type == RAISED_MEDIAN_BORDER ||
                   type == PAINTED_MEDIAN_BORDER;
        }
    };

    /**
     * Lane region.
     */
    struct Region {
        // Unique ID for the lane region.
        // This ID is consistent with the face ID of the region in the
        // base_graph.
        int id;

        // Category title of this region.
        std::string category;

        // Vertical level of this region.
        int level = 0;

        // True if this region is a lane region.
        bool is_lane;

        // Collection of boundary polyedges that form the closed boundary of the
        // lane region. It supports regions with holes.
        Array<Array<Polyedge>> rings;

        // Cutpoints on the region.
        Array<int> cutpoints;

        // Region polygon.
        RMultiPolygon2D polygon;
    };

    /**
     * The adjacent lane regions can be further grouped if they share a same
     * cutline.
     */
    struct RegionGroup {
        int id;

        // Region's level.
        int level;

        // Region group type.
        std::string category;

        // Grouped regions.
        Array<int> regions;

        // Collection of boundary polyedges that form the closed boundary of the
        // lane region group.
        Array<Array<Polyedge>> rings;

        // Region polygon.
        RMultiPolygon2D polygon;

        // Special treat for virtual intersection.
        bool is_virtual_intersection = false;

        /**
         * Retrun true if this region group is a lane region.
         */
        bool is_lane() const {
            return category != "non_lane";
        }
    };

    /**
     * A struct representing an edge in the region adjacency graph, capturing
     * two adjacent regions and the boundary edges connecting them.
     */
    struct RegionAdjacency {
        // Current region ID.
        int region;

        // Adjacent region ID.
        int adjacency_region;

        // Store indices of edges in the base_graph that form the boundary
        // between two regions. The edges are unordered.
        Array<EdgeIndex> boundary_edges;

        // Decomposed polyedges from unordered boundary edges.
        Array<Polyedge> boundary_polyedges;

        // Store the index of the vertex in the base_graph that acts as
        // the degenerate boundary (cutpoint) between two regions.
        VertexIndex cutpoint_vertex = -1;
    };

    explicit LaneRegionGraph(double snap_threshold = 1e-2)
        : snap_threshold_(snap_threshold) {}

    /**
     * Construct the lane-region graph from road and lane regions.
     */
    void Construct(const Array<LandRegionOverlay>& region_overlays,
                   const Array<LaneArrangement::LaneRegion>& lane_regions) {
        this->clear();
        raw_region_overlays_ = region_overlays;
        raw_lane_regions_ = lane_regions;
        ConstructGeoGraph();
        ConstructRegionAdjacencies();
        ConstructRegionPolyedges();
        FindSpecialCutlines();
        FindCutpoints();
        ComputeAdjacencyBoundaries();
    }

    void clear() {
        edge_type_.clear();
        is_cutpoint_.clear();
        base_graph_.clear();
        raw_edge_type_.clear();
        region_adjacencies_.clear();
        regions_.clear();
        raw_region_overlays_.clear();
        raw_lane_regions_.clear();
    }

    /**
     * The adjacent lane regions can be further grouped if they are lane region
     * and share only one cutline.
     */
    void GroupRegions(Array<RegionGroup>* region_groups) const {
        CHECK(region_groups);

        region_groups->clear();

        Array<int> ignorable_edges;
        Array<Array<int>> groups;
        GroupRegions(&groups, &ignorable_edges);

        std::unordered_set<int> ignored_edges(ignorable_edges.begin(),
                                              ignorable_edges.end());
        for (int e : ignorable_edges) {
            int e1 = base_graph_.TwinEdge(e);
            if (e1 != -1) ignored_edges.insert(e1);
        }

        for (auto& group : groups) {
            bool do_not_group = group.size() == 1;
            if (group.size() > 1) {
                // Merge regions' polyedges.
                Array<int> edges;
                std::unordered_map<int, BoundaryType> type_map;
                for (int g : group) {
                    CHECK(regions_[g].rings.size() == 1);

                    for (auto& polyedge : regions_[g].rings.front()) {
                        for (auto e : polyedge.edges) {
                            if (ignored_edges.count(e)) continue;
                            edges.push_back(e);
                            type_map[e] = polyedge.type;
                        }
                    }
                }

                Array<Array<int>> chains;
                if (geometry::geo_graph_util::GetEdgeChains(base_graph_, edges,
                                                            &chains) &&
                    chains.size() == 1) {
                    Array<BoundaryType> types;
                    for (auto e : chains.front()) {
                        types.push_back(type_map[e]);
                    }
                    Array<Polyedge> ring;
                    ExtractRegionBoundary(chains.front(), types, &ring);

                    RegionGroup region_group;
                    region_group.id = region_groups->size();
                    region_group.level = regions_[group.front()].level;
                    region_group.regions = group;
                    region_group.rings.push_back(ring);
                    for (const auto& ring : region_group.rings) {
                        Array<RPoint2D> vertices;
                        for (const auto& polyedge : ring) {
                            vertices.insert(polyedge.polyline);
                        }
                        RPolygon2D poly(vertices);
                        if (poly.empty()) {
                            if (region_group.polygon.empty()) {
                                do_not_group = true;
                                break;
                            }
                            continue;
                        }

                        if (region_group.polygon.empty()) {
                            region_group.polygon.Insert(poly, true);
                        } else {
                            region_group.polygon.Insert(poly, false);
                        }
                    }
                    if (region_group.polygon.empty()) {
                        do_not_group = true;
                    } else if (!do_not_group) {
                        region_groups->push_back(region_group);
                    }
                } else {
                    do_not_group = true;
                }
            }

            // Do not group.
            if (do_not_group) {
                for (int g : group) {
                    RegionGroup region_group;
                    region_group.id = region_groups->size();
                    region_group.level = regions_[g].level;
                    region_group.regions.push_back(g);
                    region_group.rings = regions_[g].rings;
                    region_group.polygon = regions_[g].polygon;
                    region_groups->push_back(region_group);
                }
            }
        }

        // Set category.
        for (auto& group : *region_groups) {
            group.category = ComputeCategory(group);
            group.is_virtual_intersection =
                    (group.regions.size() == 1 &&
                     regions_[group.regions.front()].category ==
                     "virtual_intersection");
        }
    }

    /**
     * Return number of regions in the region graph.
     */
    int n_regions() const {
        return regions_.size();
    }

    /**
     * Return true if the given edge is a cutline.
     */
    bool is_cutline(int e) const {
        auto it = edge_type_.find(e);
        return (it != edge_type_.end()) ? it->second == CUTLINE : false;
    }

    /**
     * Return the given edge's type.
     */
    BoundaryType edge_type(int e) const {
        auto it = edge_type_.find(e);
        return (it != edge_type_.end()) ? it->second : UNKNOWN;
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

    double cutline_threshold() const {
        return cutline_threshold_;
    }

private:
    /**
     * Check if there is a cutline between two categories.
     */
    static bool HasCutline(const std::string& category1,
                           const std::string& category2) {
        if (road_util::IsRoadSegment(category1)) {
            return road_util::IsRoadIntersection(category2) ||
                   road_util::IsRoadOverlap(category2);
        }

        if (road_util::IsRoadSegment(category2)) {
            return road_util::IsRoadIntersection(category1) ||
                   road_util::IsRoadOverlap(category1);
        }

        return false;
    }

    /**
     * Return the number of cutlines of the given region.
     */
    int CountCutlines(const Region& region) const {
        int n_cutlines = 0;
        for (const auto& ring : region.rings) {
            for (auto& polyedge : ring) {
                if (polyedge.is_cutline()) {
                    ++n_cutlines;
                }
            }
        }
        return n_cutlines;
    }

    /**
     * Check if two adjacent regions can be merged.
     */
    bool CanMerge(const RegionAdjacency& adjacency) const {
        const Region& r1 = regions_[adjacency.region];
        const Region& r2 = regions_[adjacency.adjacency_region];
        if (!r1.is_lane || !r2.is_lane) return false;
        // Do not merge regions on different vertical levels (e.g. an overpass
        // and the road beneath it that touch along a cross-level cutline) into
        // one region group. A group is given a single level, so merging across
        // levels would produce one medial axis spanning both and stamp part of
        // it (the portion inside the higher-level road) with the wrong level.
        // They stay separate regions and are still joined by a connection at
        // the shared cutline.
        if (r1.level != r2.level) return false;
        if (r1.polygon.n_boundaries() > 1 || r2.polygon.n_boundaries() > 1)
            return false;

        if (adjacency.boundary_polyedges.size() != 1) return false;
        auto boundary = adjacency.boundary_polyedges.front();
        if (!boundary.is_cutline()) return false;

        int n_cutlines1 = CountCutlines(r1) + r1.cutpoints.size();
        int n_cutlines2 = CountCutlines(r2) + r2.cutpoints.size();
        if (n_cutlines1 != 2 || n_cutlines2 != 2) return false;

        bool can_be_ignored = true;
        for (int e1 : adjacency.boundary_edges) {
            if (!is_cutline(e1) ||
                is_cutpoint(base_graph_.source(e1)) ||
                is_cutpoint(base_graph_.target(e1))) {
                can_be_ignored = false;
                break;
            }
        }
        if (!can_be_ignored) return false;

        std::set<int> set1(adjacency.boundary_edges.begin(),
                           adjacency.boundary_edges.end());
        std::set<int> set2;
        for (int e : adjacency.boundary_edges) {
            int e1 = base_graph_.TwinEdge(e);
            CHECK(e1 != -1);
            set2.insert(e1);
        }

        Array<std::set<int>> cutlines1, cutlines2;
        for (auto e : r1.rings.front()) {
            if (e.is_cutline()) {
                cutlines1.emplace_back(e.edges.begin(), e.edges.end());
            }
        }
        for (auto e : r2.rings.front()) {
            if (e.is_cutline()) {
                cutlines2.emplace_back(e.edges.begin(), e.edges.end());
            }
        }

        return cutlines1.find(set1) != cutlines1.end() &&
               cutlines2.find(set2) != cutlines2.end();
    }

    /**
     * The adjacent lane regions can be further grouped if they are lane region
     * and share only one cutline.
     * This function groups the above lane regions and also outputs the ignored
     * edges.
     */
    void GroupRegions(Array<Array<int>>* groups,
                      Array<int>* ignoreable_edges) const {
        CHECK(groups);
        CHECK(ignoreable_edges);
        CHECK(regions_.size() == region_adjacencies_.size());

        ignoreable_edges->clear();

        int n_regions = regions_.size();

        Graph graph(n_regions);
        auto connects = graph.AddEdgeProperty<int>("connects", 0);
        for (int i = 0; i < n_regions; ++i) {
            for (const auto& e : region_adjacencies_[i]) {
                int j = e.adjacency_region;
                if (j <= i) continue;

                auto e1 = graph.InsertTwoWayEdge(i, j);
                for (auto e2 : e.boundary_polyedges) {
                    if (e2.is_cutline()) {
                        connects[e1]++;
                        connects[e1->twin()]++;
                    }
                }
            }
        }

        DisjointSet disjoint_set(n_regions);

        for (int i = 0; i < n_regions; ++i) {
            for (const auto& e : region_adjacencies_[i]) {
                int j = e.adjacency_region;
                if (j <= i) continue;
                auto e1 = graph.FindEdge(i, j);
                if (!e1) continue;
                if (connects[e1] != 1) continue;

                if (!CanMerge(e)) continue;

                // Merge regions i and j, update their connects.
                ignoreable_edges->insert(e.boundary_edges.begin(),
                                         e.boundary_edges.end());

                disjoint_set.Union(i, j);

                Array<int> neighbor_i, neighbor_j;
                for (auto e : graph.edges_from(i)) {
                    neighbor_i.push_back(e->target());
                }
                for (auto e : graph.edges_from(j)) {
                    neighbor_j.push_back(e->target());
                }
                for (int k : neighbor_i) {
                    if (k == j) continue;

                    auto e1 = graph.FindEdge(k, j);
                    if (!e1) e1 = graph.InsertTwoWayEdge(k, j);
                    connects[e1]++;
                    connects[e1->twin()]++;
                }
                for (int k : neighbor_j) {
                    if (k == i) continue;

                    auto e1 = graph.FindEdge(k, i);
                    if (!e1) e1 = graph.InsertTwoWayEdge(k, i);
                    connects[e1]++;
                    connects[e1->twin()]++;
                }
            }
        }
        disjoint_set.ToClusters(groups);
    }

    /**
     * Check if the i-th region is a lane region.
     */
    bool IsLaneRegion(int i) const {
        CHECK(i >= 0 && i < regions_.size());

        std::string category = regions_[i].category;
        if (category == "middle_turn_median") return true;

        if (!road_util::IsRoadSegment(category) &&
            !road_util::IsRoadIntersection(category) /*&&
            !road_util::IsRoadOverlap(category)*/) {
            return false;
        }

        int n_cutlines = CountCutlines(regions_[i]);
        int n_cutpoints = regions_[i].cutpoints.size();

        if (road_util::IsRoadIntersection(category)) {
            if (n_cutlines + n_cutpoints != 2) return false;
        } else if (road_util::IsRoadSegment(category)) {
            if (n_cutlines == 0) return false;
        }

        return true;
    }

    /**
     * Construct geo-graph.
     */
    void ConstructGeoGraph() {
        Array<RPoint2D> points;
        for (const auto& r : raw_region_overlays_) {
            for (const auto& b : r.polygon.boundaries()) {
                points.insert(b.polygon.vertices());
            }
        }
        for (const auto& r : raw_lane_regions_) {
            for (const auto& b : r.polygon.boundaries()) {
                points.insert(b.polygon.vertices());
            }
        }

        geometry::Snap2D<double> snap(points, snap_threshold_);
        for (const auto& r : raw_lane_regions_) {
            RMultiPolygon2D region_polygon;
            Array<RPoint2D> outer_face;
            Array<Array<RPoint2D>> inner_faces;
            for (const auto& raw_ring : r.rings) {
                Array<RPoint2D> ring;
                for (auto e : raw_ring) {
                    RPolyline2D polyline;
                    snap.GetSnapPolyline(e.source, e.target, &polyline);
                    for (int i = 0; i + 1 < polyline.size(); ++i) {
                        int eid = base_graph_.InsertEdge(polyline[i],
                                                         polyline[i + 1]);
                        raw_edge_type_[eid] |= e.type;
                        ring.push_back(polyline[i]);
                    }
                }
                RPolygon2D polygon(ring);
                polygon.Trim();
                ring = polygon.vertices();
                if (polygon.IsClockwise()) {
                    if (!polygon.empty()) {
                        region_polygon.Insert(polygon, false);
                        inner_faces.push_back(ring);
                    }
                } else {
                    if (polygon.empty()) break;
                    outer_face = ring;
                    region_polygon.Insert(polygon, true);
                }
            }
            if (region_polygon.empty()) continue;

            inner_faces.insert(inner_faces.begin(), outer_face);
            int face = base_graph_.InsertFace(inner_faces);
            if (face != -1) {
                Region region;
                region.id = face;
                region.polygon = region_polygon;
                region.level = r.level;
                region.category = r.category;
                regions_.push_back(region);
            }
        }
    }

    /**
     * Compute the adjancecies between the regions.
     */
    void ConstructRegionAdjacencies() {
        int n_regions = regions_.size();
        region_adjacencies_.resize(n_regions);

        edge_type_.reserve(base_graph_.n_edges());
        for (auto e : base_graph_.edges()) {
            int e1 = base_graph_.TwinEdge(e.id);
            int t = raw_edge_type_[e.id];
            int t1 = 0;
            if (e1 != -1) t1 = raw_edge_type_[e1];
            if ((t & (1 << LaneArrangement::CUTLINE)) != 0 ||
                (t & (1 << LaneArrangement::AOI_BORDER)) != 0) {
                edge_type_[e.id] = CUTLINE;
            } else if (LaneArrangement::is_raised_median_edge(t) ||
                       LaneArrangement::is_raised_median_edge(t1)) {
                edge_type_[e.id] = RAISED_MEDIAN_BORDER;
            } else if (LaneArrangement::is_painted_median_edge(t) ||
                       LaneArrangement::is_painted_median_edge(t1)) {
                edge_type_[e.id] = PAINTED_MEDIAN_BORDER;
            } else if (LaneArrangement::is_lane_edge(t)) {
                edge_type_[e.id] = ASSIST_LINE;
            } else if (LaneArrangement::is_region_edge(t) ||
                       LaneArrangement::is_region_edge(t1)) {
                edge_type_[e.id] = LAND_REGION_BORDER;
            }
        }

        // Add common adjacency.
        for (auto e : base_graph_.edges()) {
            int e1 = e.id;
            int e2 = base_graph_.TwinEdge(e1);
            if (e2 == -1) {
                if (edge_type_[e.id] != CUTLINE) {
                    edge_type_[e.id] = OUTERMOST_ROAD;
                }
                continue;
            }

            Array<int> regions1, regions2;
            base_graph_.GetFacesOfEdge(e1, &regions1);
            base_graph_.GetFacesOfEdge(e2, &regions2);

            if (regions1.empty() || regions2.empty()) continue;

            if (regions1.size() == 1 && regions2.size() == 1) {
                Region& r1 = regions_[regions1.front()];
                Region& r2 = regions_[regions2.front()];
                if (r1.id == r2.id) continue;

                if (r1.level != r2.level &&
                    road_util::IsRoadSegment(r1.category) &&
                    road_util::IsRoadSegment(r2.category)) {
                    edge_type_[e.id] = CUTLINE;
                }
                if (edge_type_[e.id] != CUTLINE) {
                    if (road_util::IsGore(r1.category) ||
                        road_util::IsGore(r2.category)) {
                        edge_type_[e.id] = GORE_AREA_BORDER;
                    } else if (HasCutline(r1.category, r2.category)) {
                        edge_type_[e.id] = CUTLINE;
                    }
                }

                InsertEdgeAdjacency(r1.id, r2.id, e.id);
                continue;
            }

            for (int r1 : regions1) {
                for (int r2 : regions2) {
                    if (r1 == r2) continue;

                    Region& region1 = regions_[r1];
                    Region& region2 = regions_[r2];

                    if (region1.level != region2.level) continue;
//                    edge_type_[e.id] = CUTLINE;
//                    InsertEdgeAdjacency(r1, r2, e.id);
                }
            }
        }

        // Do not connect two regions whose shared CUTLINE boundary is too short
        // in total: accumulate the CUTLINE-edge length per region pair and
        // demote that pair's CUTLINE edges to LAND_REGION_BORDER (so they no
        // longer connect) when the total is below cutline_threshold_. Mirrors
        // the road pipeline's shared-boundary length gate.
        std::map<std::pair<int, int>, double> cutline_pair_length;
        std::map<std::pair<int, int>, Array<int>> cutline_pair_edges;
        for (auto e : base_graph_.edges()) {
            if (edge_type(e.id) != CUTLINE) continue;
            int twin = base_graph_.TwinEdge(e.id);
            if (twin == -1) continue;

            Array<int> rs1, rs2;
            base_graph_.GetFacesOfEdge(e.id, &rs1);
            base_graph_.GetFacesOfEdge(twin, &rs2);
            if (rs1.empty() || rs2.empty()) continue;
            int r1 = rs1.front(), r2 = rs2.front();
            if (r1 == r2) continue;

            auto key = std::make_pair(std::min(r1, r2), std::max(r1, r2));
            // Count each undirected edge's length once.
            if (e.id < twin) {
                cutline_pair_length[key] +=
                        Distance(base_graph_.source_point(e.id),
                                 base_graph_.target_point(e.id));
            }
            cutline_pair_edges[key].push_back(e.id);
        }
        for (const auto& p : cutline_pair_length) {
            if (p.second >= cutline_threshold_) continue;
            for (int eid : cutline_pair_edges[p.first]) {
                edge_type_[eid] = LAND_REGION_BORDER;
            }
        }
    }

    /**
     * Compute polyedge boundary for each region.
     */
    void ConstructRegionPolyedges() {
        for (auto& region : regions_) {
            CHECK(region.rings.empty());

            auto face = base_graph_.face(region.id);
            Array<int> rings;
            rings.push_back(face.outer_ring);
            rings.insert(face.inner_rings);

            for (int ring : rings) {
                auto& edges = base_graph_.ring(ring).edges;
                Array<BoundaryType> types(edges.size());
                for (int i = 0; i < edges.size(); ++i) {
                    types[i] = this->edge_type(edges[i]);
                    if (types[i] != CUTLINE) continue;

                    int twin = base_graph_.TwinEdge(edges[i]);
                    if (twin != -1) {
                        Array<int> regions;
                        base_graph_.GetFacesOfEdge(twin, &regions);
                        bool has_adjacent = false;
                        std::string adjacent_category;
                        if (regions.size() == 1) continue;
                        for (auto r : regions) {
                            if (regions_[r].level == region.level) {
                                has_adjacent = true;
                                adjacent_category = regions_[r].category;
                                break;
                            }
                        }
                        if (!has_adjacent)
                            types[i] = OUTERMOST_ROAD;
                        else if (!road_util::IsRoadSegment(adjacent_category) &&
                                 !road_util::IsRoadIntersection(
                                     adjacent_category))
                            types[i] = LAND_REGION_BORDER;
                    }
                }
                Array<Polyedge> boundary;
                ExtractRegionBoundary(edges, types, &boundary);
                region.rings.push_back(boundary);
            }
        }
    }

    /**
     * Find the special lane-region cutines.
     */
    void FindSpecialCutlines() {
        for (Region& region : regions_) {
            if (road_util::IsRoadSegment(region.category)) {
                for (auto& ring : region.rings) {
                    for (int i = 0; i < ring.size(); ++i) {
                        int prev = i == 0 ? ring.size() - 1 : i - 1;
                        int next = i + 1 == ring.size() ? 0 : i + 1;
                        if (ring[i].type == GORE_AREA_BORDER &&
                            (ring[prev].type == OUTERMOST_ROAD ||
                             ring[next].type == OUTERMOST_ROAD ||
                             ring[prev].type == LAND_REGION_BORDER ||
                             ring[next].type == LAND_REGION_BORDER)) {
                            ring[i].type = CUTLINE;
                            for (int e : ring[i].edges) {
                                edge_type_[e] = CUTLINE;
                            }
                        }
                    }
                }
            }

            // Filter too short cutlines.
            const double threshold = 0.2;
            for (auto& ring : region.rings) {
                for (auto& e : ring) {
                    if (e.is_cutline() && e.polyline.Length() < threshold) {
                        e.type = LAND_REGION_BORDER;
                    }
                }
            }
        }
    }

    /**
     * Add cutpoint adjacency.
     */
    void FindCutpoints() {
        for (auto& r : regions_) {
            r.is_lane = IsLaneRegion(r.id);
        }

        for (auto v : base_graph_.vertices()) {
            const auto& edges = base_graph_.edges_from(v.id);
            if (edges.size() < 2) continue;

            for (int i = 0; i < edges.size(); ++i) {
                int e1 = edges[i];
                const auto& rings1 = base_graph_.rings_of_edge(e1);
                for (int j = i + 1; j < edges.size(); ++j) {
                    int e2 = edges[j];
                    const auto& rings2 = base_graph_.rings_of_edge(e2);

                    for (auto r1 : rings1) {
                        for (auto r2 : rings2) {
                            int f1 = base_graph_.ring(r1).face;
                            int f2 = base_graph_.ring(r2).face;
                            if (f1 == f2) continue;
                            if (!regions_[f1].is_lane || !regions_[f2].is_lane)
                                continue;
                            if (road_util::IsMedian(regions_[f1].category))
                                continue;
                            if (road_util::IsMedian(regions_[f2].category))
                                continue;

                            int e3 = base_graph_.PreviousEdgeOnRing(r1, e1);
                            if (e3 == -1) continue;

                            int e4 = base_graph_.PreviousEdgeOnRing(r2, e2);
                            if (e4 == -1) continue;

                            if (base_graph_.TwinEdge(e1) == e2) continue;
                            if (base_graph_.TwinEdge(e1) == e4) continue;
                            if (base_graph_.TwinEdge(e3) == e2) continue;
                            if (base_graph_.TwinEdge(e3) == e4) continue;

                            int cnt = is_cutline(e1) + is_cutline(e2) +
                                      is_cutline(e3) + is_cutline(e4);
                            if (cnt != 1) continue;

                            if (!is_cutline(e1) && !is_cutline(e3)) {
                                RVector2D v1 = base_graph_.target_point(e1) -
                                               base_graph_.source_point(e1);
                                RVector2D v2 = base_graph_.source_point(e3) -
                                               base_graph_.target_point(e3);
                                if (Degree(v1, v2) > 90.0) continue;
                            }
                            if (!is_cutline(e2) && !is_cutline(e4)) {
                                RVector2D v1 = base_graph_.target_point(e2) -
                                               base_graph_.source_point(e2);
                                RVector2D v2 = base_graph_.source_point(e4) -
                                               base_graph_.target_point(e4);
                                if (Degree(v1, v2) > 90.0) continue;
                            }

                            InsertCutpointAdjacency(f1, f2, v.id);
                            InsertCutpointAdjacency(f2, f1, v.id);
                            is_cutpoint_[v.id] = true;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < region_adjacencies_.size(); ++i) {
            std::unordered_set<int> hash;
            for (auto& ring : regions_[i].rings) {
                for (auto& polyedges : ring) {
                    if (!polyedges.is_cutline()) continue;
                    for (int e : polyedges.edges) {
                        hash.insert(base_graph_.source(e));
                        hash.insert(base_graph_.target(e));
                    }
                }
            }
            for (auto& j : region_adjacencies()[i]) {
                if (j.cutpoint_vertex != -1) {
                    if (!hash.count(j.cutpoint_vertex)) {
                        regions_[i].cutpoints.push_back(j.cutpoint_vertex);
                    }
                }
            }
        }
    }

    /**
     * Extract boundary polyedges from the input edge sequence, grouping
     * consecutive edges of the same type into ordered Polyedge instances.
     */
    void ExtractRegionBoundary(const Array<int>& edges,
                               Array<Polyedge>* polyedges) const {
        CHECK(polyedges);

        polyedges->clear();

        const int n = edges.size();
        if (n == 0) return;

        Array<BoundaryType> types(n);
        for (int i = 0; i < edges.size(); ++i) {
            types[i] = this->edge_type(edges[i]);
        }
        ExtractRegionBoundary(edges, types, polyedges);
    }

    /**
     * Extract boundary polyedges from the input edge sequence with types,
     * grouping consecutive edges of the same type into ordered Polyedge
     * instances.
     */
    void ExtractRegionBoundary(const Array<int>& edges,
                               const Array<BoundaryType>& types,
                               Array<Polyedge>* polyedges) const {
        CHECK(edges.size() == types.size());
        CHECK(polyedges);

        polyedges->clear();

        const int n = edges.size();
        if (n == 0) return;

        Polyedge polyedge;
        for (int i = 0; i < n; ++i) {
            if (polyedge.edges.empty()) {
                polyedge.type = types[i];
            } else if (polyedge.type != types[i]) {
                polyedges->push_back(polyedge);

                polyedge = Polyedge{};
                polyedge.type = types[i];
            }
            polyedge.edges.push_back(edges[i]);
        }
        if (!polyedge.edges.empty()) {
            polyedges->push_back(polyedge);
        }

        bool is_cyclic = base_graph_.source(edges.front()) ==
                         base_graph_.target(edges.back());

        // Handle cyclic continuity: merge first and last if same type.
        if (is_cyclic && polyedges->size() > 1 &&
            polyedges->front().type == polyedges->back().type) {
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
     * Extract multiple polyedges from unordered boundary edges.
     */
    void BoundaryToPolyedges(const Array<int>& edges,
                             Array<Polyedge>* polyedges) const {
        CHECK(polyedges);

        Array<Array<int>> chains;
        geometry::geo_graph_util::GetEdgeChains(base_graph_, edges, &chains);

        Array<Polyedge> t;
        for (auto& c : chains) {
            ExtractRegionBoundary(c, &t);
            polyedges->insert(t);
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

    /**
     * Set boundary polyedges.
     */
    void ComputeAdjacencyBoundaries() {
        for (int i = 0; i < regions_.size(); ++i) {
            for (auto& adj : region_adjacencies_[i]) {
                BoundaryToPolyedges(adj.boundary_edges,
                                    &adj.boundary_polyedges);
            }
        }
    }

    /**
     * Get region group's category.
     */
    std::string ComputeCategory(const RegionGroup& group) const {
        if (group.regions.size() == 1 &&
            !regions_[group.regions.front()].is_lane) {
            return "non_lane";
        }

        for (int r : group.regions) {
            if (regions_[r].category == "middle_turn_median") {
                return "middle_turn_median";
            }
        }

        for (int r : group.regions) {
            for (auto& adj : region_adjacencies_[r]) {
                if (adj.boundary_polyedges.size() != 1) continue;
                if (!adj.boundary_polyedges.front().is_cutline()) continue;
                int j = adj.adjacency_region;
                if (road_util::IsGore(regions_[j].category)) {
                    return "shoulder";
                }
            }
        }

        for (int r : group.regions) {
            for (auto& ring : regions_[r].rings) {
                for (auto& polyedge : ring) {
                    if (polyedge.is_road_border()) {
                        return "shoulder_candidate";
                    }
                }
            }
        }

        return "lane";
    }

    // The snap distance for snapping.
    double snap_threshold_ = 1e-2;

    // Threshold for filtering short cutlines.
    double cutline_threshold_ = 0.2;

    // Boundary type for each edge in the base_graph.
    std::unordered_map<EdgeIndex, BoundaryType> edge_type_;

    // Identify vertices that act as "cut points".
    // A cut point is a degenerate form of a cut line: it arises when a cut line
    // (originally an edge) collapses into a single point.
    std::unordered_map<VertexIndex, bool> is_cutpoint_;

    // Underlying geometric graph storing the base topology for lane regions:
    // - Vertices: Geometric points (e.g., vertices of lane polygons).
    // - Edges: Line segments (e.g., edge of lane polygons).
    // - Faces: Polygonal faces, each representing a distinct lane region.
    GeoGraph base_graph_;

    // Raw edge type from the input.
    std::unordered_map<EdgeIndex, int> raw_edge_type_;

    // Adjacencies explicitly modeling relationships between lane regions.
    Array<Array<RegionAdjacency>> region_adjacencies_;

    // Lane regions.
    Array<Region> regions_;

    // Input road region overlays.
    Array<LandRegionOverlay> raw_region_overlays_;

    // Input lane regions.
    Array<LaneArrangement::LaneRegion> raw_lane_regions_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_REGION_GRAPH_H_

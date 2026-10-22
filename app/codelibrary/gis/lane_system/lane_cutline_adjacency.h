//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_CUTLINE_ADJACENCY_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_CUTLINE_ADJACENCY_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/gis/lane_system/lane_region_graph.h"

namespace cl {
namespace gis {

/**
 * Establishe cutline-based adjacency relationships between lane regions.
 * This adjacency model provides critical support for applications such as lane
 * centerline generation, lane connectivity analysis, and lane region topology
 * construction.
 */
class LaneCutlineAdjacency {
    using Region          = LaneRegionGraph::Region;
    using RegionGroup     = LaneRegionGraph::RegionGroup;
    using Polyedge        = LaneRegionGraph::Polyedge;
    using RegionAdjacency = LaneRegionGraph::RegionAdjacency;

public:
    /**
     * The intersection result of two adjacent cutlines.
     */
    struct CutlineIntersection {
        // Whether the intersection degenerates into a single point.
        bool is_point;
        // Intersection line (valid when is_point is false).
        RPolyline2D line;
        // Intersection point (valid when is_point is false).
        RPoint2D point;
        // Connection point for centerline.
        RPoint2D connection;

        CutlineIntersection() = default;

        explicit CutlineIntersection(const RPoint2D& p)
            : is_point(true), point(p) {
            connection = p;
        }

        explicit CutlineIntersection(const RPolyline2D& edge)
            : is_point(false), line(edge) {
            connection = Center(edge);
        }

        CutlineIntersection(const RPoint2D& p, const RPoint2D& c)
            : is_point(true), point(p), connection(c) {}
    };

    /**
     * Represent a shared boundary (cut edge) between two lane regions.
     */
    struct Cutline {
        // Unique ID for the cutline
        int id;

        // Index of the lane region that owns this cutline.
        int region;

        // IDs of cutlines adjacent to this one.
        // The intersection part is represented by polyline.
        Array<std::pair<int, CutlineIntersection>> adjacent_cutlines;

        // Geometric representing of this cutline.
        Polyedge polyedge;

        // Cutline may degenerate into a cutpoint.
        RPoint2D cutpoint;

        // True if this cutline is shared by two regions that sit on different
        // vertical levels (e.g. the boundary between an overpass and the road
        // underneath it). Computed once during Construct(); always false for a
        // degenerate cutpoint cutline.
        bool is_multilevel = false;

        /**
         * This cutline is degenerated into a cutpoint.
         */
        bool is_cutpoint() const {
            return polyedge.empty();
        }

        /**
         * Return true if this cutline lies between two regions of different
         * vertical levels (an overpass/underpass boundary).
         */
        bool is_multilevel_cutline() const {
            return is_multilevel;
        }
    };

    LaneCutlineAdjacency() = default;

    /**
     * Construct lane cutline adjacency between grouped lane regions.
     */
    void Construct(const LaneRegionGraph& lane_graph,
                   const Array<RegionGroup>& region_groups) {
        if (region_groups.empty()) return;

        cutlines_.clear();
        region_cutlines_.clear();
        cutpoint_to_cutline_.clear();

        const auto& graph = lane_graph.base_graph();
        const Array<Region>& regions = lane_graph.regions();

        int n_region_groups = region_groups.size();
        region_cutlines_.resize(n_region_groups);

        cutline_threshold_ = lane_graph.cutline_threshold();

        // Step1: Extract cutlines.
        for (int i = 0; i < n_region_groups; ++i) {
            const auto& region_group = region_groups[i];
            for (const auto& ring : region_group.rings) {
                for (const auto& polyedge : ring) {
                    if (!polyedge.is_cutline()) continue;

                    Cutline cutline;
                    cutline.polyedge = polyedge;
                    cutline.region = i;
                    cutline.id = cutlines_.size();
                    // A cutline edge is shared by two faces (the owning region
                    // on this side, and the neighbour across the twin edge). If
                    // any neighbour across the cutline sits on a different
                    // level, this is a multilevel (overpass/underpass) cutline.
                    cutline.is_multilevel = IsMultilevelPolyedge(
                            polyedge, region_group.level, graph, regions);
                    region_cutlines_[i].push_back(cutline.id);
                    cutlines_.push_back(cutline);
                }
            }

            int n_cutlines = region_cutlines_[i].size();
            for (int r : region_group.regions) {
                std::unordered_set<int> cutpoints;
                for (int v : regions[r].cutpoints) {
                    cutpoints.insert(v);
                }
                for (int v : cutpoints) {
                    Cutline cutline;
                    cutline.region = i;
                    cutline.id = cutlines_.size();
                    cutline.cutpoint = graph.point(v);
                    if (n_cutlines != 2) {
                        region_cutlines_[i].push_back(cutline.id);
                    }
                    cutlines_.push_back(cutline);
                    cutpoint_to_cutline_[cutline.cutpoint] = cutline.id;
                }
            }
        }

        // Step2: Compute cutline adjacencies.
        Array<int> set(regions.size(), -1);
        for (auto& r : region_groups) {
            for (int i : r.regions) {
                set[i] = r.id;
            }
        }
        for (int i = 0; i < n_region_groups; ++i) {
            const auto& region_group = region_groups[i];
            for (int r1 : region_group.regions) {
                for (const auto& adj : lane_graph.region_adjacencies()[r1]) {
                    if (adj.cutpoint_vertex == -1) {
                        if (adj.boundary_polyedges.size() != 1 ||
                            !adj.boundary_polyedges.front().is_cutline())
                            continue;
                    }

                    int r2 = adj.adjacency_region;
                    if (set[r1] == set[r2]) continue;
                    if (set[r1] == -1 || set[r2] == -1) continue;
                    if (set[r2] <= set[r1]) continue;

                    AddCutlineAdjacency(set[r1], set[r2], adj, graph);
                }
            }
        }
    }

    /**
     * Return cutlines.
     */
    const Array<Cutline>& cutlines() const {
        return cutlines_;
    }

    /**
     * Return indices of cutlines for each region.
     */
    const Array<Array<int>>& region_cutlines() const {
        return region_cutlines_;
    }

private:
    /**
     * Return true if the cutline polyedge is shared with a region on a
     * different vertical level.
     *
     * Each edge of the polyedge belongs to the owning region on this side; the
     * region(s) on the other side are obtained from the twin edge's faces. The
     * polyedge is multilevel if any such neighbour's level differs from the
     * owning region group's level. Note that a single edge may border more than
     * one face when regions are vertically stacked, so all neighbour faces are
     * checked.
     */
    static bool IsMultilevelPolyedge(
            const Polyedge& polyedge, int level,
            const geometry::GeoGraph<RPoint2D>& graph,
            const Array<Region>& regions) {
        for (int e : polyedge.edges) {
            int twin = graph.TwinEdge(e);
            if (twin == -1) continue;

            Array<int> faces;
            graph.GetFacesOfEdge(twin, &faces);
            for (int f : faces) {
                if (f >= 0 && f < regions.size() && regions[f].level != level) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Compute cutline adjacencies.
     */
    void AddCutlineAdjacency(int i, int j,
                             const RegionAdjacency& adjacency,
                             const geometry::GeoGraph<RPoint2D>& graph) {
        if (adjacency.cutpoint_vertex != -1) {
            RPoint2D p = graph.point(adjacency.cutpoint_vertex);
            if (!cutpoint_to_cutline_.count(p)) return;

            int k2 = cutpoint_to_cutline_[p];
            auto& c2 = cutlines_[k2];

            for (int k1 : region_cutlines_[i]) {
                auto& c1 = cutlines_[k1];
                for (auto v : c1.polyedge.polyline) {
                    if (v == p) {
                        CutlineIntersection i(p, Center(c1.polyedge.polyline));
                        c1.adjacent_cutlines.emplace_back(k2, i);
                        c2.adjacent_cutlines.emplace_back(k1, i);
                    }
                }
            }
            for (int k1 : region_cutlines_[j]) {
                auto& c1 = cutlines_[k1];
                for (auto v : c1.polyedge.polyline) {
                    if (v == p) {
                        CutlineIntersection i(p, Center(c1.polyedge.polyline));
                        c1.adjacent_cutlines.emplace_back(k2, i);
                        c2.adjacent_cutlines.emplace_back(k1, i);
                    }
                }
            }

            return;
        }

        for (auto e : adjacency.boundary_polyedges) {
            if (!e.is_cutline()) continue;

            std::unordered_set<int> hash(e.edges.begin(), e.edges.end());
            for (int e1 : e.edges) {
                int e2 = graph.TwinEdge(e1);
                if (e2 != -1) hash.insert(e2);
            }

            for (int k1 : region_cutlines_[i]) {
                auto& c1 = cutlines_[k1];
                bool found = false;
                for (int e : c1.polyedge.edges) {
                    if (hash.count(e)) {
                        found = true;
                        break;
                    }
                }
                if (!found) continue;

                for (int k2 : region_cutlines_[j]) {
                    auto& c2 = cutlines_[k2];
                    bool found = false;
                    for (int e : c2.polyedge.edges) {
                        if (hash.count(e)) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) continue;

                    // Found a cutline adjacency.
                    std::unordered_map<int, int> hash_count;
                    for (int e : c1.polyedge.edges) {
                        if (hash.count(e)) {
                            int e1 = graph.TwinEdge(e);
                            if (e1 != -1)
                                hash_count[e1]++;
                        }
                    }
                    for (int e : c2.polyedge.edges) {
                        if (hash.count(e)) {
                            hash_count[e]++;
                        }
                    }

                    Array<int> edges;
                    for (auto p : hash_count) {
                        if (p.second == 2) {
                            edges.push_back(p.first);
                        }
                    }

                    Array<RPolyline2D> chains;
                    if (!geometry::geo_graph_util::GetEdgeChains(graph, edges,
                                                                 &chains)) {
                        continue;
                    }

                    RPolyline2D longest_polyline;
                    double length = 0.0;
                    for (auto& polyline : chains) {
                        double l = polyline.Length();
                        if (l > length) {
                            length = l;
                            longest_polyline = polyline;
                        }
                    }
                    if (longest_polyline.Length() < cutline_threshold_)
                        continue;

                    c1.adjacent_cutlines.emplace_back(k2, longest_polyline);
                    c2.adjacent_cutlines.emplace_back(k1, longest_polyline);
                }
            }
        }
    }

    // Threshold for filtering short cutlines.
    double cutline_threshold_ = 0.0;

    // Cutlines for connection.
    Array<Cutline> cutlines_;

    // Indices of cutlines for each region.
    Array<Array<int>> region_cutlines_;

    // Map cutpoint to cutline.
    std::unordered_map<RPoint2D, int> cutpoint_to_cutline_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_CUTLINE_ADJACENCY_H_

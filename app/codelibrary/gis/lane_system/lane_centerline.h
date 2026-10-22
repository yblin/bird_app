//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_CENTERLINE_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_CENTERLINE_H_

#include <set>

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/range_ray_2d.h"
#include "codelibrary/geometry/topology/medial_axis_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/carto_polyline.h"
#include "codelibrary/gis/carto_polyline_util.h"
#include "codelibrary/gis/lane_system/lane_categorizer.h"
#include "codelibrary/gis/lane_system/lane_cutline_adjacency.h"
#include "codelibrary/gis/lane_system/lane_region_graph.h"
#include "codelibrary/gis/simplification/douglas_peucker_simplifier.h"
#include "codelibrary/statistics/mean.h"

namespace cl {
namespace gis {

/**
 * Extract centerlines for lane regions from a lane region graph.
 */
class LaneCenterline {
    using Lane = CartoPolyline2D;
    using LaneKernel = CartoPolylineKernel2D;
    using Region = LaneRegionGraph::Region;
    using RegionGroup = LaneRegionGraph::RegionGroup;
    using Cutline = LaneCutlineAdjacency::Cutline;
    using CutlineIntersection = LaneCutlineAdjacency::CutlineIntersection;

    /**
     * Attributes of a cutline.
     */
    struct CutlineAttributes {
        // Cutline id.
        int cutline_id;
        // Corresponding region.
        int region_id;
        // Width of this cutline.
        double width;
        // Whether this cutline has the branch connection point.
        bool has_branch_connection;
        // Point in the region for connection.
        RPoint2D branch_connection;
        // Whether this cutline has the cutline connection point.
        bool has_cutline_connection;
        // Point on cutline for connection.
        RPoint2D cutline_connection;
        // Indicate which endpoint of the owning region's centerline (branch) is
        // associated with this cutline.
        // 0 = front endpoint of the branch;
        // 1 = back endpoint of the branch.
        int connected_branch_endpoint;
        // Center point of the cutline.
        RPoint2D center;
    };

public:
    /**
     * Error type of lane.
     */
    enum LaneErrorType {
        LANE_VALID = 0,             // No error.
        LANE_TOO_SHORT = 1,         // Lane is too short.
        LANE_TOO_NARROW = 2,        // Lane is too narrow.
        LANE_NOT_STABLE = 3,        // Lane is not stable.
        LANE_WITHOUT_CENTERLINE = 4 // Lane without centerline.
    };

    /**
     * Constructor for LaneCenterline.
     *
     * Parameters:
     *  resolution - Spatial resolution in meters, which controls the
     *               densification of lane polygon boundaries and influences the
     *               detail level of the resulting medial axis.
     */
    explicit LaneCenterline(double resolution = 0.3)
        : resolution_(resolution) {
        CHECK(resolution_ > 0.0);
    }

    /**
     * Extracts lane centerlines and their connections from a lane region graph.
     *
     * Processe each lane region in the input graph to generate a centerline
     * branch, then identify connections between these branches based on shared
     * cutlines.
     */
    void Extract(const LaneRegionGraph& lane_graph,
                 const Array<LaneMarking>& lane_markings) {
        lanes_.clear();
        cutline_attributes_.clear();

        // Step1. get region group and construct the cutline adjacency.
        lane_graph.GroupRegions(&regions_);
        cutline_adjacency_.Construct(lane_graph, regions_);

        cutline_attributes_.resize(cutline_adjacency_.cutlines().size());
        for (int i = 0; i < cutline_attributes_.size(); ++i) {
            const auto& cutline = cutline_adjacency_.cutlines()[i];
            cutline_attributes_[i].cutline_id = i;
            cutline_attributes_[i].region_id = cutline.region;
            cutline_attributes_[i].center =
                    cutline.is_cutpoint() ? cutline.cutpoint
                                          : Center(cutline.polyedge.polyline);
        }
        need_centerline_.assign(regions_.size(), true);
        lanes_.resize(regions_.size());

        // Step2: Extract centerline for each lane region.
        for (const auto& region : regions_) {
            if (region.category == "non_lane" ||
                cutline_adjacency_.region_cutlines()[region.id].size() > 2) {
                need_centerline_[region.id] = false;
                continue;
            }

            ExtractBranch(region);
        }

        // Step3: assign category to each region.
        LaneCategorizer categorizer;
        Array<std::string> categories;
        categorizer.AssignCategories(regions_, lanes_, lane_markings,
                                     &categories);
        CHECK(lanes_.size() == categories.size());

        for (int i = 0; i < categories.size(); ++i) {
            std::string s = categorizer.to_lane_branch_category(categories[i]);
            if (regions_[i].category == "shoulder_candidate") {
                if (s != "shoulder") {
                    regions_[i].category = "lane";
                }
            }
            lanes_[i].category = s;
        }
        FixShoulderRegions();

        for (int i = 0; i < regions_.size(); ++i) {
            if (regions_[i].category == "shoulder" ||
                regions_[i].category == "middle_turn_median") {
                auto error_type = IsLaneWidthValid(lanes_[i]);
                if (error_type == LANE_TOO_NARROW) {
                    regions_[i].category = "lane_too_narrow";
                    lanes_[i].clear();
                    need_centerline_[i] = false;
                }
            }
        }

        // Step4: connect the branches at cutlines.
        ConnectLanes();

        // Step5: extend remain lanes.
        ExtendLanes();

        // Step6: special treat for virtual intersections.
        ConnectVirtualIntersection();

        // Step7: check the results.
        for (auto& r : regions_) {
            if (r.category == "non_lane" || r.category == "lane_too_narrow")
                continue;

            if (r.category == "lane") {
                if (lanes_[r.id].empty()) {
                    r.category = "lane_without_centerline";
                    continue;
                }
                if (lanes_[r.id].category != "through" &&
                    lanes_[r.id].category != "shoulder") continue;
            }

            auto error_type = IsLaneWidthValid(lanes_[r.id]);
            switch (error_type) {
            case LANE_TOO_SHORT:
                r.category = "lane_too_short";
                break;
            case LANE_TOO_NARROW:
                r.category = "lane_too_narrow";
                break;
            case LANE_NOT_STABLE:
                r.category = "lane_not_stable";
                break;
            case LANE_WITHOUT_CENTERLINE:
                r.category = "lane_without_centerline";
                break;
            default:
                r.category = "lane";
            }
        }
    }

    /**
     * Decompose the lane centerline into lane-kernels, where each sub-array
     * corresponds to a complete lane branch.
     */
    void ToLaneKernels(Array<Array<LaneKernel>>* kernels) const {
        CHECK(kernels);

        kernels->clear();

        Array<Lane> all_lanes = lanes_;
        all_lanes.insert(connections_);

        Array<bool> connected(all_lanes.size(), false);
        std::unordered_map<RPoint2D, int> degrees;
        std::unordered_map<RPoint2D, Array<int>> graph;
        for (auto&& [index, branch] : enumerate(all_lanes)) {
            if (branch.empty()) continue;
            degrees[branch.polyline.front()]++;
            degrees[branch.polyline.back()]++;
            graph[branch.polyline.front()].push_back(index);
            graph[branch.polyline.back()].push_back(-index);
        }
        for (auto p : graph) {
            if (p.second.size() == 2) {
                if (p.second[0] >= lanes_.size() &&
                    p.second[1] >= lanes_.size()) {
                    // Forced split at p.
                    degrees[p.first]++;
                }
            }
        }

        Array<Array<int>> groups;
        for (int i = 0; i < all_lanes.size(); ++i) {
            if (all_lanes[i].empty()) continue;
            if (connected[i]) continue;
            Array<int> group;
            group.push_back(i);
            connected[i] = true;
            std::string category = all_lanes[i].category;

            while (true) {
                int front = group.front();
                RPoint2D c = front < 0 ? all_lanes[-front].polyline.back()
                                       : all_lanes[front].polyline.front();
                if (degrees[c] == 2) {
                    auto& adj = graph[c];
                    CHECK(adj.size() == 2);
                    int t = std::abs(adj[0]) == std::abs(front) ? adj[1]
                                                                : adj[0];
                    if (connected[std::abs(t)]) break;
                    if (all_lanes[std::abs(t)].category != category) break;
                    group.insert(group.begin(), -t);
                    connected[std::abs(t)] = true;
                } else {
                    break;
                }
            }

            while (true) {
                int back = group.back();
                RPoint2D c = back >= 0 ? all_lanes[back].polyline.back()
                                       : all_lanes[-back].polyline.front();
                if (degrees[c] == 2) {
                    auto& adj = graph[c];
                    CHECK(adj.size() == 2);
                    int t = std::abs(adj[0]) == std::abs(back) ? adj[1]
                                                               : adj[0];

                    if (connected[std::abs(t)]) break;
                    if (all_lanes[std::abs(t)].category != category) break;
                    group.push_back(t);
                    connected[std::abs(t)] = true;
                } else {
                    break;
                }
            }

            groups.push_back(group);
        }

        for (auto& group : groups) {
            Lane lane;
            lane.category = all_lanes[std::abs(group.front())].category;
            // Propagate the vertical level (z-index) to the merged lane.
            // Connection segments carry no level, so take it from a constituent
            // lane region (indices < lanes_.size() are lanes, the rest are
            // connections).
            for (int g : group) {
                int idx = std::abs(g);
                if (idx < lanes_.size()) {
                    lane.level = all_lanes[idx].level;
                    break;
                }
            }
            for (int g : group) {
                if (!lane.polyline.empty()) {
                    lane.polyline.pop_back();
                    lane.widths.pop_back();
                }

                if (g < 0) {
                    lane.polyline.insert(lane.polyline.end(),
                                         all_lanes[-g].polyline.rbegin(),
                                         all_lanes[-g].polyline.rend());
                    lane.widths.insert(lane.widths.end(),
                                       all_lanes[-g].widths.rbegin(),
                                       all_lanes[-g].widths.rend());
                } else {
                    lane.polyline.insert(lane.polyline.end(),
                                         all_lanes[g].polyline.begin(),
                                         all_lanes[g].polyline.end());
                    lane.widths.insert(lane.widths.end(),
                                       all_lanes[g].widths.begin(),
                                       all_lanes[g].widths.end());
                }
            }

            Array<LaneKernel> segs;
            carto_polyline_util::DecomposeToKernels(lane, resolution_, &segs);
            kernels->push_back(segs);
        }
    }

    /**
     * Return the extracted lanes.
     *
     * The output lanes have a one-to-one correspondence with lane regions.
     */
    const Array<Lane>& lanes() const {
        return lanes_;
    }

    /**
     * Return the connections between lanes.
     */
    const Array<Lane>& connections() const {
        return connections_;
    }

    /**
     * Return lane regions.
     */
    const Array<RegionGroup>& regions() const {
        return regions_;
    }

    /**
     * Indicate if the i-th region needs centerline extraction.
     */
    bool need_centerline(int i) const {
        CHECK(i >= 0 && i < regions_.size());

        return need_centerline_[i];
    }

    const LaneCutlineAdjacency& cutline_adjacency() const {
        return cutline_adjacency_;
    }

private:
    /**
     * Return true if a specific category lane region do not need to connect.
     */
    static bool DonotConnect(const std::string& category1,
                             const std::string& category2) {
        if (category1 == category2) return false;

        return category1 == "shoulder" || category1 == "middle_turn_median" ||
               category1 == "non_lane" ||
               category2 == "shoulder" || category2 == "middle_turn_median" ||
               category2 == "non_lane";
    }

    /**
     * Return true if the given lane category is a dedicated bike lane, which
     * must not be connected to motor-vehicle lanes.
     *
     * Note: "shared_bl" is intentionally excluded. A shared bike lane region is
     * effectively a through lane (its centerline coincides with the through
     * lane, and is relabeled as "through" downstream), so it should connect
     * like a vehicle lane.
     */
    static bool IsBikeLane(const std::string& category) {
        return category == "bike_lane" || category == "protected_bl" ||
               category == "buffered_bl";
    }

    /**
     * Return true if the two lane regions are allowed to be connected.
     *
     * Applies both compatibility rules consistently across all connection
     * sites: the region-category rule (no shoulder / middle_turn_median /
     * non_lane mixing) and the bike-lane rule (no bike / motor-vehicle mixing).
     */
    bool CanConnect(int region_a, int region_b) const {
        if (DonotConnect(regions_[region_a].category,
                         regions_[region_b].category)) {
            return false;
        }
        if (IsBikeLane(lanes_[region_a].category) !=
            IsBikeLane(lanes_[region_b].category)) {
            return false;
        }
        return true;
    }

    /**
     * Check if the given line segment is valid (not cross with the region's
     * non-cutline edges.
     */
    static bool IsValid(const RegionGroup& region, const RSegment2D& seg) {
        for (auto& ring : region.rings) {
            for (const auto& polyedge : ring) {
                if (polyedge.is_cutline()) continue;

                if (geometry::Cross(polyedge.polyline, seg)) return false;
            }
        }
        return true;
    }

    /**
     * Extract a single main centerline branch (with width information) from the
     * i-th lane region.
     */
    bool ExtractBranch(const RegionGroup& region) {
        int rid = region.id;

        Lane& lane = lanes_[rid];

        lane.clear();
        lane.category = region.category;
        geometry::TriangulationAxis2D<double> medial_axis;

        RMultiPolygon2D polygon;
        for (const auto& ring : region.rings) {
            Array<RPoint2D> vertices;
            for (const auto& polyedge : ring) {
                if (polyedge.is_cutline()) {
                    vertices.insert(polyedge.polyline);
                } else {
                    RPolyline2D densified;
                    geometry::polyline_util::Subdivide(polyedge.polyline,
                                                       resolution_,
                                                       &densified);
                    vertices.insert(densified);
                }
            }
            RPolygon2D poly(vertices);
            if (poly.empty()) {
                if (polygon.empty()) break;
                continue;
            }

            if (polygon.empty()) {
                polygon.Insert(poly, true);
            } else {
                polygon.Insert(poly, false);
            }
        }
        if (polygon.empty()) return false;

        if (!medial_axis.Construct(polygon)) {
            LOG(ERROR) << "Lane region: " << rid
                       << " is invalid after densifing.";
            return false;
        }

        Array<int> main_branch_vertices;
        RPolyline2D main_branch;
        medial_axis.PruneBPR(2.0, resolution_);
        if (!medial_axis.GetPrimaryBranch(45.0, &main_branch_vertices,
                                          &main_branch)) {
            if (cutline_adjacency_.region_cutlines()[rid].size() != 2) {
                need_centerline_[rid] = false;
            }
            return false;
        }

        lane.level = region.level;
        Array<double> widths;
        for (auto v : main_branch_vertices) {
            widths.push_back(medial_axis.radius(v) * 2.0);
        }

        Lane main_lane;
        main_lane.polyline = main_branch;
        main_lane.widths = widths;
        DouglasPeuckerSimplifier simplifier(resolution_);
        simplifier.Simplify(main_lane, &main_lane);
        lane.polyline = main_lane.polyline;
        if (lane.polyline.size() < 2) {
            lane.clear();
            return false;
        }

        return OptimizeRawLane(region, &lane);
    }

    /**
     * Special treat for virtual intersections.
     */
    void ConnectVirtualIntersection() {
        geometry::RTree2D<double> rtree;
        Array<RBox2D> boxes;
        for (auto l : lanes_) {
            boxes.push_back(RBox2D(l.polyline.begin(), l.polyline.end()));
        }
        rtree.Construct(boxes);

        for (auto& r : regions_) {
            if (!r.is_virtual_intersection) continue;
            auto& cutlines = cutline_adjacency_.region_cutlines()[r.id];
            if (cutlines.size() != 3 && cutlines.size() != 1) continue;

            struct ConnectPair {
                RVector2D v;
                RPoint2D p;
                double w;
                // Region of the connected neighbor branch, used to forbid
                // incompatible (shoulder / median / non-lane / bike-vs-vehicle)
                // connections.
                int region;
            };
            Array<ConnectPair> connection;
            Array<ConnectPair> single_connection;

            auto& all_cutlines = cutline_adjacency_.cutlines();
            for (int c : cutlines) {
                auto adj = all_cutlines[c].adjacent_cutlines;
                if (adj.size() == 1) {
                    int j = adj.front().first;
                    int neighbor_region = cutline_attributes_[j].region_id;
                    Lane lane = lanes_[neighbor_region];
                    lane.polyline.Trim();
                    if (lane.empty()) continue;
                    ConnectPair p;
                    p.region = neighbor_region;
                    if (cutline_attributes_[j].connected_branch_endpoint == 1) {
                        p.v = lane.polyline.back() -
                              lane.polyline[lane.polyline.size() - 2];
                        p.p = lane.polyline.back();
                        p.w = lane.widths.back();
                    } else {
                        p.v = lane.polyline[0] - lane.polyline[1];
                        p.p = lane.polyline.front();
                        p.w = lane.widths.front();
                    }
                    connection.push_back(p);
                }
            }
            CHECK(connection.size() <= 3);
            if (connection.empty()) continue;
            if (connection.size() == 1) {
                // DISABLED: the single-neighbor case cast a ray from the
                // dangling lane endpoint and connected it to whatever nearby
                // lane the ray happened to hit, which could attach to an
                // unrelated region at a mid-lane point (wrong connection).
                // For now we simply do not connect in this case. Re-enable (or
                // replace with an adjacency-based connection) later.
                /*
                auto ring = regions_[r.id].rings.front();
                int prev = ring.size() - 1;
                bool found = false;
                for (int i = 0; i < ring.size(); ++i) {
                    if (ring[i].is_cutline()) {
                        found = true;
                        break;
                    }
                    prev = i;
                }
                if (!found) continue;

                if (ring[prev].type == LaneRegionGraph::ASSIST_LINE) {
                    RPoint2D c = Center(ring[prev].polyline);
                    double dis = Distance(connection.front().p, c);
                    RBox2D box(c.x - dis * 2.0, c.x + dis * 2.0,
                               c.y - dis * 2.0, c.y + dis * 2.0);
                    Array<int> entries;
                    rtree.Find(box, &entries);

                    RRangeRay2D ray(RRay2D(connection.front().p, c));

                    for (int j : entries) {
                        if (!CanConnect(connection.front().region, j)) continue;
                        RPolyline2D& poly = lanes_[j].polyline;
                        if (poly.empty()) continue;
                        if (poly.back() == ray.origin()) continue;
                        if (poly.front() == ray.origin()) continue;
                        ray.Intercept(poly);
                    }

                    if (ray.upper() < DBL_MAX &&
                        ray.lower_point() != ray.upper_point()) {
                        Lane c;
                        c.category = "connection";
                        c.level = lanes_[connection[0].region].level;
                        c.polyline = RPolyline2D(ray.lower_point(),
                                                 ray.upper_point());
                        c.widths = {connection[0].w, connection[0].w};
                        connections_.push_back(c);
                    }
                }
                */
            } else if (connection.size() == 2) {
                if (CanConnect(connection[0].region, connection[1].region)) {
                    Lane c;
                    c.category = "connection";
                    c.level = lanes_[connection[0].region].level;
                    c.polyline = RPolyline2D(connection[0].p, connection[1].p);
                    c.widths = {connection[0].w, connection[1].w};
                    connections_.push_back(c);
                }
            } else {
                Array<Array<int>> paths = {{0,1,2}, {0,2,1}, {1,0,2}};
                double min_total_angle = DBL_MAX;
                Array<int> best_path = paths[0];

                for (const auto& path : paths) {
                   int a = path[0], b = path[1], c = path[2];
                   RVector2D vec1 = connection[b].p - connection[a].p;
                   RVector2D vec2 = connection[c].p - connection[b].p;

                   double angle1 = Degree(vec1, connection[a].v);
                   double angle2 = Degree(vec2, connection[b].v);
                   double angle3 = Degree(-vec1, connection[b].v);
                   double angle4 = Degree(-vec2, connection[c].v);
                   double sum = angle1 + angle2 + angle3 + angle4;

                   if (sum < min_total_angle) {
                       min_total_angle = sum;
                       best_path = path;
                    }
                }
                Lane lane;
                lane.category = "connection";
                int a = best_path[0], b = best_path[1], c = best_path[2];
                if (CanConnect(connection[a].region, connection[b].region)) {
                    lane.level = lanes_[connection[a].region].level;
                    lane.polyline = RPolyline2D(connection[a].p, connection[b].p);
                    lane.widths = {connection[a].w, connection[b].w};
                    connections_.push_back(lane);
                }
                if (CanConnect(connection[b].region, connection[c].region)) {
                    lane.level = lanes_[connection[b].region].level;
                    lane.polyline = RPolyline2D(connection[b].p, connection[c].p);
                    lane.widths = {connection[b].w, connection[c].w};
                    connections_.push_back(lane);
                }
            }
        }
    }

    /**
     * Compute left and right width values for each vertex of a lane polyline.
     *
     * Widths are measured as the distance from each centerline vertex to the
     * lane's left/right boundaries. This implementation uses a more precise
     * method than RoadCenterline.
     */
    void EvaluateWidth(const RMultiPolygon2D& region,
                       const RPolyline2D& lane,
                       Array<double>* left_widths,
                       Array<double>* right_widths) const {
        left_widths->clear();
        right_widths->clear();

        if (lane.size() == 1) return;

        for (int i = 0; i < lane.size(); ++i) {
            RVector2D v = i + 1 < lane.size() ? lane[i + 1] - lane[i]
                                              : lane.back() -
                                                lane[lane.size() - 2];
            RVector2D v1(-v.y, v.x), v2(v.y, -v.x);
            RRangeRay2D ray1(RRay2D(lane[i], v1));
            RRangeRay2D ray2(RRay2D(lane[i], v2));

            for (int i = 0; i < region.n_boundaries(); ++i) {
                auto& b = region.boundaries()[i].polygon;
                for (int j = 0; j < b.size(); ++j) {
                    ray1.Intercept(b.edge(j));
                    ray2.Intercept(b.edge(j));
                }
            }
            if (ray1.upper() == DBL_MAX) {
                left_widths->push_back(0.0);
            } else {
                left_widths->push_back(Distance(ray1.upper_point(),
                                                ray1.lower_point()));
            }
            if (ray2.upper() == DBL_MAX) {
                right_widths->push_back(0.0);
            } else {
                right_widths->push_back(Distance(ray2.upper_point(),
                                                 ray2.lower_point()));
            }
        }
    }

    /**
     * Densify the lane and evaluate the width values.
     */
    void EvaluateWidth(const RMultiPolygon2D& region,
                       const RPolyline2D& lane,
                       RPolyline2D* densified_lane,
                       Array<double>* widths) const {
        widths->clear();

        if (lane.size() <= 1) return;

        geometry::polyline_util::Densify(lane, resolution_, densified_lane);
        Array<double> left_widths, right_widths;
        EvaluateWidth(region, *densified_lane, &left_widths, &right_widths);

        for (int i = 0; i < left_widths.size(); ++i) {
            widths->push_back(left_widths[i] + right_widths[i]);
        }
    }

    /**
     * Determine if a region is truly a shoulder, if a shoulder cadindate
     * region shares a cutline with another non-shoulder region, then that
     * region cannot be a shoulder.
     */
    void FixShoulderRegions() {
        Array<Cutline> cutlines;
        for (auto c : cutline_adjacency_.cutlines()) {
            if (regions_[c.region].category == "shoulder_candidate") {
                cutlines.push_back(c);
            }
        }

        const auto& all_cutlines = cutline_adjacency_.cutlines();

        bool finish = false;
        while (!finish) {
            finish = true;
            for (auto& c1 : cutlines) {
                if (regions_[c1.region].category != "shoulder_candidate")
                    continue;

                std::set<std::string> found_types;
                for (auto& ajacent_cutline : c1.adjacent_cutlines) {
                    Cutline c2 = all_cutlines[ajacent_cutline.first];
                    if (all_cutlines[c2.id].adjacent_cutlines.size() != 1) {
                        found_types.clear();
                        break;
                    }
                    std::string type = regions_[c2.region].category;
                    if (type == "lane") {
                        if (!lanes_[c2.region].category.empty()) {
                            found_types.insert(lanes_[c2.region].category);
                        }
                    }
                }
                if (found_types.size() == 1) {
                    finish = false;
                    regions_[c1.region].category = "lane";
                    lanes_[c1.region].category = *found_types.begin();
                } else if (found_types.size() > 1) {
                    finish = false;
                    regions_[c1.region].category = "lane";
                    lanes_[c1.region].category = "through";
                }
            }
        }
        for (auto& r : regions_) {
            if (r.category == "shoulder_candidate") {
                r.category = "shoulder";
                lanes_[r.id].category = "shoulder";
            }
        }
    }

    /**
     * Optimize the given centerline lane branch.
     */
    bool OptimizeRawLane(const RegionGroup& region, Lane* lane) {
        if (lane->polyline.size() <= 1) {
            lane->clear();
            return false;
        }

        RPolyline2D polyline;
        geometry::polyline_util::Subdivide(lane->polyline, 2.0 * resolution_,
                                           &polyline);

        Array<double> w1, w2;
        EvaluateWidth(region.polygon, polyline, &w1, &w2);
        CHECK_EQ(w1.size(), polyline.size());
        CHECK_EQ(w2.size(), polyline.size());

        const double ratio_threshold = 1.3;
        double threshold1 = (w1.front() + w2.front()) * 2.0;
        double threshold2 = (w1.back() + w2.back()) * 2.0;

        int start = 0;
        double l1 = 0.0;
        while (start < polyline.size()) {
            // Check and remove unstable start endpoint.
            double w_left = w1[start];
            double w_right = w2[start];
            double ratio = ratio_threshold;
            if (start + 1 < polyline.size()) {
                l1 += Distance(polyline[start], polyline[start + 1]);
                if (l1 > threshold1) break;
            }
            if (w_left > 0.0 && w_right > 0.0) {
                ratio = std::max(w_left, w_right) / std::min(w_left, w_right);
            }
            if (ratio >= ratio_threshold) {
                ++start;
            } else {
                break;
            }
        }

        if (start + 1 >= polyline.size()) {
            lane->clear();
            return false;
        }

        double l2 = 0.0;
        while (!polyline.empty()) {
            if (polyline.size() > 2) {
                l2 += Distance(polyline[polyline.size() - 2], polyline.back());
                if (l2 > threshold2) break;
            }
            // Check and remove unstable end endpoint.
            double w_left = w1.back();
            double w_right = w2.back();
            double ratio = ratio_threshold;
            if (w_left > 0.0 && w_right > 0.0) {
                ratio = std::max(w_left, w_right) / std::min(w_left, w_right);
            }
            if (ratio >= ratio_threshold) {
                polyline.pop_back();
                w1.pop_back();
                w2.pop_back();
            } else {
                break;
            }
        }

        if (start + 1 >= polyline.size()) {
            lane->clear();
            return false;
        }

        polyline = RPolyline2D(polyline.begin() + start, polyline.end());
        w1 = Array<double>(w1.begin() + start, w1.end());
        w2 = Array<double>(w2.begin() + start, w2.end());
        CHECK(polyline.size() >= 2);
        CHECK(polyline.size() == w1.size());
        CHECK(polyline.size() == w2.size());

        lane->polyline = polyline;
        lane->widths.resize(polyline.size());
        for (int i = 0; i < polyline.size(); ++i) {
            lane->widths[i] = w1[i] + w2[i];
        }

        // Update cutlines and lanes.
        double w_front = 2.0 * (w1[start] + w2[start]);
        double w_back = 2.0 * (w1.back() + w2.back());

        Array<CutlineAttributes*> cutline_attributes;

        const auto& region_cutlines =
                cutline_adjacency_.region_cutlines()[region.id];
        if (region_cutlines.size() == 1) {
            auto& cutline_attri = cutline_attributes_[region_cutlines[0]];
            UpdateCutlineAttributes(lane, &cutline_attri);
            cutline_attributes.push_back(&cutline_attri);
        } else if (region_cutlines.size() == 2) {
            auto& cutline_attri1 = cutline_attributes_[region_cutlines[0]];
            auto& cutline_attri2 = cutline_attributes_[region_cutlines[1]];
            if (UpdateCutlineAttributes(lane,
                                        &cutline_attri1,
                                        &cutline_attri2)) {
                cutline_attributes.push_back(&cutline_attri1);
                cutline_attributes.push_back(&cutline_attri2);
            }
        }

        for (auto cutline_attri : cutline_attributes) {
            if (!cutline_attri->has_branch_connection) continue;

            if (cutline_attri->connected_branch_endpoint == 0) {
                BackoffFront(cutline_attri->center, w_front, lane);
                cutline_attri->branch_connection = lane->polyline.front();
            } else {
                BackoffBack(cutline_attri->center, w_back, lane);
                cutline_attri->branch_connection = lane->polyline.back();
            }
        }

        return true;
    }

    /**
     * Connect lane to the specific cutline and update the cutline attributes.
     */
    void ConnectLane(const Cutline& cutline,
                     const RegionGroup& region,
                     CutlineAttributes* cutline_attri,
                     Lane* lane) {
        CHECK(lane);
        CHECK(cutline_attri);
        CHECK(cutline_attri->has_branch_connection);

        if (cutline.is_cutpoint()) return;

        RPolyline2D& polyline = lane->polyline;
        Array<double>& widths = lane->widths;
        CHECK(polyline.size() >= 2) << polyline.size();

        CHECK(cutline_attri->branch_connection == polyline.front() ||
              cutline_attri->branch_connection == polyline.back());

        if (polyline.front() == polyline.back()) {
            // The lane is a cycle.
            return;
        }

        if (Distance(cutline_attri->branch_connection,
                     cutline.polyedge.polyline) < resolution_) return;

        RPoint2D c = Center(cutline.polyedge.polyline);
        // Only search for a splice point within a short window near the
        // connected endpoint, so the connector stays close to the medial axis
        // (the region's middle) instead of cutting across a bend.
        double limit = std::max(2.0 * cutline_attri->width, 2.0 * resolution_);
        if (polyline.front() == cutline_attri->branch_connection) {
            double score = 360.0;
            int best = -1;
            double l = 0.0;
            for (int i = 0; i + 1 < polyline.size(); ++i) {
                if (l > limit) break;
                RVector2D v1 = polyline[i] - c;
                RVector2D v2 = polyline[i + 1] - polyline[i];
                double d = Degree(v1, v2);
                if (d < score) {
                    RSegment2D line(polyline[i], c);
                    if (IsValid(region, line)) {
                        score = d;
                        best = i;
                    }
                }
                l += Distance(polyline[i], polyline[i + 1]);
            }
            if (best != -1) {
                double w = widths.front();
                polyline = RPolyline2D(polyline.begin() + best, polyline.end());
                widths = Array<double>(widths.begin() + best, widths.end());

                RPolyline2D poly(c, polyline.front());
                RPolyline2D poly1;
                Array<double> ws;
                EvaluateWidth(region.polygon, poly, &poly1, &ws);
                ws.front() = w;

                polyline.insert(polyline.begin(), poly1.begin(),
                                poly1.end() - 1);
                widths.insert(widths.begin(), ws.begin(), ws.end() - 1);
            }
        } else {
            double score = 360.0;
            int best = -1;
            double l = 0.0;
            for (int i = polyline.size() - 1; i > 0; --i) {
                if (l > limit) break;
                RVector2D v1 = polyline[i] - c;
                RVector2D v2 = polyline[i - 1] - polyline[i];
                double d = Degree(v1, v2);
                if (d < score) {
                    RSegment2D line(polyline[i], c);
                    if (IsValid(region, line)) {
                        score = d;
                        best = i;
                    }
                }
                l += Distance(polyline[i], polyline[i - 1]);
            }
            if (best != -1) {
                double w = widths.back();
                polyline = RPolyline2D(polyline.begin(),
                                       polyline.begin() + best + 1);
                widths = Array<double>(widths.begin(),
                                       widths.begin() + best + 1);

                RPolyline2D poly(polyline.back(), c);
                RPolyline2D poly1;
                Array<double> ws;
                EvaluateWidth(region.polygon, poly, &poly1, &ws);
                ws.back() = w;

                polyline.insert(polyline.end(), poly1.begin() + 1, poly1.end());
                widths.insert(widths.end(), ws.begin() + 1, ws.end());
            }
        }

        cutline_attri->branch_connection = c;
    }

    /**
     * Connect lane to the specific cutline with the specific connection
     * point.
     */
    void ConnectLane(const RPoint2D& connection,
                     CutlineAttributes* cutline_attri,
                     Lane* lane) {
        CHECK(lane);
        CHECK(cutline_attri);
        CHECK(cutline_attri->has_branch_connection);

        RPolyline2D& polyline = lane->polyline;
        Array<double>& widths = lane->widths;
        if (polyline.empty()) return;

        CHECK(cutline_attri->branch_connection == polyline.front() ||
              cutline_attri->branch_connection == polyline.back());

        if (polyline.front() == polyline.back()) {
            // The lane is a cycle.
            return;
        }

        if (cutline_attri->connected_branch_endpoint == 0) {
            if (polyline.front() != connection) {
                polyline.insert(polyline.begin(), connection);
                widths.insert(widths.begin(), widths.front());
            }
        } else {
            if (polyline.back() != connection) {
                polyline.push_back(connection);
                widths.push_back(widths.back());
            }
        }
        cutline_attri->branch_connection = connection;
        cutline_attri->has_cutline_connection = true;
        cutline_attri->cutline_connection = connection;
    }

    /**
     * Extend lanes to touch the boundary.
     */
    void ExtendLanes() {
        const auto& region_cutlines = cutline_adjacency_.region_cutlines();

        std::unordered_set<RPoint2D> has_connection;
        for (auto& c : connections_) {
            if (c.empty()) continue;
            has_connection.insert(c.polyline.front());
            has_connection.insert(c.polyline.back());
        }

        for (int i = 0; i < regions_.size(); ++i) {
            if (lanes_[i].empty()) continue;
            RPolyline2D& polyline = lanes_[i].polyline;
            Array<double>& widths = lanes_[i].widths;

            Array<int> donot_need_extend;
            for (auto j : region_cutlines[i]) {
                CutlineAttributes& cutline = cutline_attributes_[j];
                if (!cutline.has_branch_connection) continue;
                if (cutline.has_cutline_connection) continue;
                if (cutline.connected_branch_endpoint == 0) {
                    if (has_connection.find(polyline.front()) ==
                        has_connection.end()) continue;
                } else {
                    if (has_connection.find(polyline.back()) ==
                        has_connection.end()) continue;
                }
                donot_need_extend.push_back(cutline.connected_branch_endpoint);
            }

            if (donot_need_extend.find(0) == donot_need_extend.end()) {
                RRangeRay2D ray(polyline.front(), polyline[0] - polyline[1]);
                RPoint2D p;
                bool hit = false;
                for (auto b : regions_[i].polygon) {
                    RPolyline2D poly(b.polygon.vertices());
                    poly.push_back(poly.front());
                    hit |= ray.Intercept(poly, &p);
                }
                if (hit) {
                    polyline.insert(polyline.begin(), p);
                    widths.insert(widths.begin(), widths.front());
                }
            }

            if (donot_need_extend.find(1) == donot_need_extend.end()) {
                RRangeRay2D ray(polyline.back(), polyline.back() -
                                                 polyline[polyline.size() - 2]);
                RPoint2D p;
                bool hit = false;
                for (auto b : regions_[i].polygon) {
                    RPolyline2D poly(b.polygon.vertices());
                    poly.push_back(poly.front());
                    hit |= ray.Intercept(poly, &p);
                }
                if (hit) {
                    polyline.push_back(p);
                    widths.push_back(widths.back());
                }
            }
        }
    }

    /**
     * Update the given cutline's attributes after its centerline lane is
     * computed, and adjust the lane accordingly.
     */
    void UpdateCutlineAttributes(Lane* lane,
                                 CutlineAttributes* cutline_attri) {
        CHECK(lane);
        CHECK(cutline_attri);
        CHECK(lane->widths.size() == lane->polyline.size());

        RPolyline2D& polyline = lane->polyline;
        Array<double>& widths = lane->widths;
        CHECK(polyline.size() == widths.size());

        RPoint2D c = cutline_attri->center;

        // Find the closest point on the lane to the cutline.
        int closest = geometry::polyline_util::FindNearestVertex(polyline, c);
        if (closest == -1) return;

        cutline_attri->has_branch_connection = true;
        if (closest == 0) {
            cutline_attri->connected_branch_endpoint = 0;
        } else if (closest == polyline.size() - 1) {
            cutline_attri->connected_branch_endpoint = 1;
        } else {
            double d1 = Distance(polyline.front(), c);
            double d2 = Distance(polyline.back(), c);
            cutline_attri->connected_branch_endpoint = d1 < d2 ? 0 : 1;
        }

        if (cutline_attri->connected_branch_endpoint == 0) {
            polyline = RPolyline2D(polyline.begin() + closest, polyline.end());
            widths = Array<double>(widths.begin() + closest, widths.end());
            cutline_attri->branch_connection = polyline.front();
            cutline_attri->width = widths.front();
        } else {
            polyline = RPolyline2D(polyline.begin(),
                                   polyline.begin() + closest + 1);
            widths = Array<double>(widths.begin(),
                                   widths.begin() + closest + 1);
            cutline_attri->branch_connection = polyline.back();
            cutline_attri->width = widths.back();
        }
        CHECK(lane->widths.size() == lane->polyline.size());
        CHECK(polyline.size() >= 2);
        CHECK(cutline_attri->branch_connection == polyline.front() ||
              cutline_attri->branch_connection == polyline.back());
    }

    /**
     * Update the cutlines on the same region after the centerline lane is
     * computed.
     */
    bool UpdateCutlineAttributes(Lane* lane,
                                 CutlineAttributes* cutline_attri1,
                                 CutlineAttributes* cutline_attri2) {
        CHECK(lane);

        RPolyline2D& polyline = lane->polyline;
        Array<double>& widths = lane->widths;
        CHECK(polyline.size() == widths.size());

        RPoint2D c1 = cutline_attri1->center;
        RPoint2D c2 = cutline_attri2->center;

        double d1 = Distance(polyline.front(), c1);
        double d2 = Distance(polyline.front(), c2);
        double d3 = Distance(polyline.back(), c1);
        double d4 = Distance(polyline.back(), c2);
        cutline_attri1->has_branch_connection = true;
        cutline_attri2->has_branch_connection = true;
        if (d1 + d4 < d2 + d3) {
            cutline_attri1->branch_connection = polyline.front();
            cutline_attri1->connected_branch_endpoint = 0;
            cutline_attri2->branch_connection = polyline.back();
            cutline_attri2->connected_branch_endpoint = 1;
        } else {
            cutline_attri1->branch_connection = polyline.back();
            cutline_attri1->connected_branch_endpoint = 1;
            cutline_attri2->branch_connection = polyline.front();
            cutline_attri2->connected_branch_endpoint = 0;
        }

        for (int i = 0; i < 2; ++i) {
            RPoint2D c = i == 0 ? c1 : c2;
            CutlineAttributes& cutline_attri = i == 0 ? *cutline_attri1
                                                      : *cutline_attri2;
            int closest = geometry::polyline_util::FindNearestVertex(polyline,
                                                                     c);

            if (cutline_attri.connected_branch_endpoint == 0) {
                if (closest == polyline.size() - 1) {
                    lane->clear();
                    cutline_attri1->has_branch_connection = false;
                    cutline_attri2->has_branch_connection = false;
                    return false;
                }
                polyline = RPolyline2D(polyline.begin() + closest,
                                       polyline.end());
                widths = Array<double>(widths.begin() + closest, widths.end());

                cutline_attri.width = widths.front();
                cutline_attri.branch_connection = polyline.front();
            } else {
                if (closest == 0) {
                    lane->clear();
                    cutline_attri1->has_branch_connection = false;
                    cutline_attri2->has_branch_connection = false;
                    return false;
                }
                polyline = RPolyline2D(polyline.begin(),
                                       polyline.begin() + closest + 1);
                widths = Array<double>(widths.begin(),
                                       widths.begin() + closest + 1);
                cutline_attri.width = widths.back();
                cutline_attri.branch_connection = polyline.back();
            }
        }

        CHECK(cutline_attri1->branch_connection == polyline.front() ||
              cutline_attri1->branch_connection == polyline.back());
        CHECK(cutline_attri2->branch_connection == polyline.front() ||
              cutline_attri2->branch_connection == polyline.back());
        CHECK(polyline.size() == widths.size());

        return true;
    }

    /**
     * Backoff the front endpoint of the lane so that it can be smoothly
     * connected to point p.
     */
    void BackoffFront(const RPoint2D& p, double length, Lane* lane) {
        int best = -1;
        double min_angle = DBL_MAX;
        double l = 0.0;
        for (int i = 0; i + 1 < lane->polyline.size(); ++i) {
            if (l > length) break;
            RVector2D v1 = lane->polyline[i] - lane->polyline[i + 1];
            RVector2D v2 = p - lane->polyline[i];
            double angle = Degree(v1, v2);
            if (angle < min_angle) {
                min_angle = angle;
                best = i;
            }
            l += v1.norm();
        }
        if (best != -1) {
            lane->polyline = RPolyline2D(lane->polyline.begin() + best,
                                         lane->polyline.end());
            lane->widths = Array<double>(lane->widths.begin() + best,
                                         lane->widths.end());
        }
    }

    /**
     * Backoff the back endpoint of the lane so that it can be smoothly
     * connected to point p.
     */
    void BackoffBack(const RPoint2D& p, double length, Lane* lane) {
        int best = -1;
        double min_angle = DBL_MAX;
        double l = 0.0;
        for (int i = lane->polyline.size() - 1; i > 0; --i) {
            if (l > length) break;
            RVector2D v1 = lane->polyline[i] - lane->polyline[i - 1];
            RVector2D v2 = p - lane->polyline[i];
            double angle = Degree(v1, v2);
            if (angle < min_angle) {
                min_angle = angle;
                best = i;
            }
            l += v1.norm();
        }
        if (best != -1) {
            lane->polyline = RPolyline2D(lane->polyline.begin(),
                                         lane->polyline.begin() + best + 1);
            lane->widths = Array<double>(lane->widths.begin(),
                                         lane->widths.begin() + best + 1);
        }
    }

    /**
     * Filter adjacent cutlines from cutline_adjacency.
     */
    void FilterAdjacentCutlines(int region_id,
                                Array<std::pair<int, CutlineIntersection>>*
                                adjacent_cutlines) const {
        CHECK(adjacent_cutlines);

        const auto& cutlines = cutline_adjacency_.cutlines();
        Array<std::pair<int, CutlineIntersection>> results;
        for (const auto& c : *adjacent_cutlines) {
            Cutline cutline = cutlines[c.first];
            if (!need_centerline_[cutline.region]) continue;
            if (!CanConnect(region_id, cutline.region)) continue;
            results.push_back(c);
        }
        *adjacent_cutlines = results;
    }

    /**
     * Connect lanes at cutline.
     */
    void ConnectLanes() {
        connections_.clear();

        const auto& cutlines = cutline_adjacency_.cutlines();
        const auto& region_cutlines = cutline_adjacency_.region_cutlines();

        Array<Lane> all_connections;
        for (auto& c1 : cutline_attributes_) {
            if (!c1.has_branch_connection) continue;
            if (!need_centerline_[c1.region_id]) continue;

            const auto& cutline1 = cutlines[c1.cutline_id];
            auto& cutline_attribute1 = cutline_attributes_[c1.cutline_id];
            const auto& region = regions_[cutline1.region];
            Lane& lane = lanes_[cutline1.region];

            auto adjacent_cutlines1 = cutline1.adjacent_cutlines;
            FilterAdjacentCutlines(cutline1.region, &adjacent_cutlines1);

            bool connected = false;
            for (auto j : adjacent_cutlines1) {
                auto& c2 = cutline_attributes_[j.first];
                if (!need_centerline_[c2.region_id]) continue;

                const auto& cutline2 = cutlines[c2.cutline_id];
                auto& cutline_attribute2 = cutline_attributes_[c2.cutline_id];
                Lane& lane2 = lanes_[cutline2.region];

                if (!c2.has_branch_connection) {
                    if (cutline1.is_cutpoint()) {
                        if (adjacent_cutlines1.size() != 1) continue;
                        CHECK(!cutline2.is_cutpoint());
                        Lane c;
                        c.category = "connection";
                        c.level = lanes_[c1.region_id].level;
                        c.polyline = RPolyline2D(c1.branch_connection,
                                                 j.second.connection);
                        c.widths = {c1.width, c1.width};
                        all_connections.push_back(c);

                        cutline_attribute1.has_cutline_connection = true;
                        cutline_attribute1.cutline_connection =
                                j.second.connection;
                        connected = true;
                    } else if (adjacent_cutlines1.size() == 1) {
                        ConnectLane(cutline1, region, &cutline_attribute1,
                                    &lane);
                        connected = true;
                    }
                    continue;
                }

                Lane c;
                c.level = lanes_[c1.region_id].level;
                if (cutline1.is_cutpoint()) {
                    c.category = "connection";
                    c.polyline = RPolyline2D(c1.branch_connection,
                                             j.second.connection);
                    c.widths = {c1.width, c2.width};
                    all_connections.push_back(c);
                    connected = true;
                } else if (cutline2.is_cutpoint()) {
                    c.category = "connection";
                    c.polyline = RPolyline2D(c2.branch_connection,
                                             j.second.connection);
                    c.widths = {c2.width, c1.width};
                    all_connections.push_back(c);

                    ConnectLane(j.second.connection, &cutline_attribute1,
                                &lane);
                    connected = true;
                } else {
                    auto adjacent_cutlines2 = cutline2.adjacent_cutlines;
                    FilterAdjacentCutlines(cutline2.region,
                                           &adjacent_cutlines2);

                    int n_adjacent_cutpoints1 = 0;
                    int n_adjacent_cutpoints2 = 0;
                    for (auto j : adjacent_cutlines1) {
                        if (cutlines[j.first].is_cutpoint())
                            ++n_adjacent_cutpoints1;
                    }
                    for (auto j : adjacent_cutlines2) {
                        if (cutlines[j.first].is_cutpoint())
                            ++n_adjacent_cutpoints2;
                    }

                    if (adjacent_cutlines1.size() >= 1 &&
                        adjacent_cutlines2.size() >= 1) {
                        if (adjacent_cutlines1.size() == 1 ||
                            n_adjacent_cutpoints1 + 1 ==
                            adjacent_cutlines1.size()) {
                            ConnectLane(j.second.connection,
                                        &cutline_attribute1,
                                        &lane);
                        } else {
                            c.category = "connection";
                            c.polyline = RPolyline2D(c1.branch_connection,
                                                     j.second.connection);
                            c.widths = {c1.width, c2.width};
                            all_connections.push_back(c);
                        }

                        if (adjacent_cutlines2.size() == 1 ||
                            n_adjacent_cutpoints2 + 1 ==
                            adjacent_cutlines2.size()) {
                            ConnectLane(j.second.connection,
                                        &cutline_attribute2,
                                        &lane2);
                        } else {
                            c.category = "connection";
                            c.polyline = RPolyline2D(c2.branch_connection,
                                                     j.second.connection);
                            c.widths = {c2.width, c1.width};
                            all_connections.push_back(c);
                        }
                        connected = true;
                    }
                }
            }

            if (!connected) {
                ConnectLane(cutline1, region, &c1, &lane);
                continue;
            }
        }

        for (int i = 0; i < regions_.size(); ++i) {
            if (region_cutlines[i].size() != 2) continue;
            if (!need_centerline_[i]) continue;
            if (!lanes_[i].empty()) continue;
            Array<CutlineAttributes> adjacents[2];

            for (int j = 0; j < region_cutlines[i].size(); ++j) {
                int c = region_cutlines[i][j];
                const auto& cutline1 = cutlines[c];

                auto adjacent_cutlines = cutline1.adjacent_cutlines;
                FilterAdjacentCutlines(i, &adjacent_cutlines);
                for (auto k : adjacent_cutlines) {
                    adjacents[j].push_back(cutline_attributes_[k.first]);
                }
            }

            if (adjacents[0].size() == 1 &&
                adjacents[0].begin()->has_cutline_connection) {
                auto& p = *adjacents[0].begin();

                for (auto& q : adjacents[1]) {
                    if (!q.has_cutline_connection) continue;

                    Lane c;
                    c.category = lanes_[p.region_id].category;
                    c.level = lanes_[p.region_id].level;
                    c.polyline = RPolyline2D(p.cutline_connection,
                                             q.cutline_connection);
                    c.widths = {p.width, q.width};
                    all_connections.push_back(c);
                }
            }
            if (adjacents[1].size() == 1 &&
                adjacents[1].begin()->has_cutline_connection) {
                auto& p = *adjacents[1].begin();

                for (auto& q : adjacents[0]) {
                    if (!q.has_cutline_connection) continue;

                    Lane c;
                    c.category = lanes_[p.region_id].category;
                    c.level = lanes_[p.region_id].level;
                    c.polyline = RPolyline2D(p.cutline_connection,
                                             q.cutline_connection);
                    c.widths = {p.width, q.width};
                    all_connections.push_back(c);
                }
            }
        }

        if (all_connections.empty()) return;

        std::set<RSegment2D> connection_hash;
        for (const auto& c : all_connections) {
            if (c.polyline.front() == c.polyline.back()) continue;
            RSegment2D line(c.polyline.front(), c.polyline.back());
            if (connection_hash.find(line) == connection_hash.end()) {
                connections_.push_back(c);
                connection_hash.insert(line);
            }
        }
    }

    /**
     * Check if the given lane width is valid.
     *
     * Parameters:
     *  lane               - The given lane.
     *  min_width          - Min allowed width.
     *  min_length         - Min allowed length.
     *  max_adjacent_ratio - Ratio to compute max allowed adjacent width
     *                       difference (prevent jump).
     */
    LaneErrorType IsLaneWidthValid(const Lane& lane,
                                   double min_width = 0.35,
                                   double min_length = 0.35,
                                   double max_adjacent_ratio = 0.2) const {
        int n = lane.polyline.size();
        if (n < 2) return LANE_WITHOUT_CENTERLINE;

        // Step 1: Validate the length.
        if (lane.polyline.Length() < min_length) return LANE_TOO_SHORT;

        // Step 2: Validate width of lane.
        double mean_w = Mean(lane.widths.begin(), lane.widths.end());
        if (mean_w < min_width) return LANE_TOO_NARROW;

        // Step 3: Validate the stability.
        double max_adjacent_diff = max_adjacent_ratio * mean_w;
        for (int i = 0; i + 1 < n; ++i) {
            double diff = std::abs(lane.widths[i] - lane.widths[i + 1]);
            if (diff > max_adjacent_diff) {
                return LANE_NOT_STABLE;
            }
        }

        // All validation passed.
        return LANE_VALID;
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

    // Input lane regions.
    Array<RegionGroup> regions_;

    // Check if the i-th region need to extract centerline.
    Array<bool> need_centerline_;

    // Centerline lanes.
    Array<Lane> lanes_;

    // Lane connections.
    Array<Lane> connections_;

    // Cutline attributes.
    Array<CutlineAttributes> cutline_attributes_;

    // Cutline adjacency.
    LaneCutlineAdjacency cutline_adjacency_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_CENTERLINE_H_

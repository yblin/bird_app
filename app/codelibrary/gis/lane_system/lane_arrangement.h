//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_ARRANGEMENT_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_ARRANGEMENT_H_

#include "codelibrary/base/enumerate.h"
#include "codelibrary/gis/lane_system/lane_marking.h"
#include "codelibrary/gis/lane_system/lane_util.h"
#include "codelibrary/gis/land_region.h"
#include "codelibrary/gis/road_system/road_util.h"
#include "codelibrary/gis/topology/land_region_arrangement.h"

namespace cl {
namespace gis {

/**
 * A class responsible for constructing the arrangement of road and lane
 * regions, integrating road surfaces and road markings to generate structured
 * geographic partitions.
 */
class LaneArrangement {
    using Vertex = geometry::Arrangement2D<double>::Vertex;
    using Halfedge = geometry::Arrangement2D<double>::Halfedge;
    using Region = geometry::Arrangement2D<double>::Region;

public:
    /**
     * Basic types for edges. Each halfedge in the arrangement may be a
     * combination of these basic types.
     */
    enum EdgeType {
        AOI_BORDER = 0,
        ROAD = 1,
        ROAD_INTERSECTION = 2,
        ROAD_OVERLAP = 3,
        VIRTUAL_INTERSECTION = 4,
        RAISED_MEDIAN = 5,
        PAINTED_MEDIAN = 6,
        PARKING_RELATED = 7,
        GORE_AREA = 8,
        ASSIST_LINE = 9,
        CUTLINE = 10,
        SOLID_DASH_LINE = 11,
        WHITE_LINE = 12,
        YELLOW_LINE = 13
    };

    /**
     * Represent a lane region as output by LaneArrangement.
     */
    struct LaneRegion {
        struct Edge {
            // Source point and target point of the edge.
            RPoint2D source, target;
            // Edge type. It can be composed of multiple EdgeTypes.
            int type;
        };

        // ID of this lane region.
        int id;
        // Vertical level of this lane region, which is identical to the level
        // of its parent road region.
        int level;
        // Lane region category.
        std::string category;
        // Index of the source region-overlay this piece came from.
        int overlay = -1;
        // Ring edges of the lane region.
        Array<Array<Edge>> rings;
        // Polygon of the region.
        RMultiPolygon2D polygon;
    };

    explicit LaneArrangement(double snap_threshold = 1e-2)
        : snap_threshold_(snap_threshold) {
        CHECK(snap_threshold_ > 0.0);
    }

    LaneArrangement(const LaneArrangement&) = delete;

    LaneArrangement& operator=(const LaneArrangement&) = delete;

    /**
     * Construct arrangements of lane regions from input road surfaces and
     * road markings.
     */
    void Arrange(const Array<RMultiPolygon2D>& working_region,
                 const Array<LandRegion>& road_regions,
                 const Array<LaneMarking>& lane_markings,
                 Array<LandRegionOverlay>* region_overlays,
                 Array<LaneRegion>* lane_regions)  {
        CHECK(region_overlays);
        CHECK(lane_regions);

        region_overlays->clear();
        lane_regions->clear();

        if (road_regions.empty()) return;

        // Get first level arrangement.
        Array<LandRegion> filtered_road_regions;
        for (const auto& r : road_regions) {
            if (GetRoadCategory(r.category) != -1) {
                filtered_road_regions.push_back(r);
            }
        }
        LandRegionArrangement arrangement(snap_threshold_);
        arrangement.Arrange(filtered_road_regions);
        *region_overlays = arrangement.region_overlays();
        if (region_overlays->empty()) return;

        // Get second level arrangement.
        Array<RPolyline2D> polylines;
        Array<int> polyline_types;
        Array<int> polyline_levels;

        // Add lanes.
        for (const auto& lane_marking : lane_markings) {
            int index = GetLaneCategory(lane_marking.category);
            if (index == -1) continue;

            polylines.push_back(lane_marking.polyline);
            polyline_types.push_back(index);
            polyline_levels.push_back(lane_marking.level);
        }
        // Add working region.
        // Assume that inner boundaries are clockwise and outer boundary is
        // anit-clockwise.
        for (const auto& region : working_region) {
            for (const auto& b : region.boundaries()) {
                RPolyline2D polyline(b.polygon.vertices());
                CHECK(!polyline.empty());

                polyline.push_back(polyline.front());
                polylines.push_back(polyline);
                polyline_types.push_back(AOI_BORDER);
                polyline_levels.push_back(-1);
            }
        }

        Array<RBox2D> boxes;
        for (const auto& polyline : polylines) {
            boxes.emplace_back(polyline.begin(), polyline.end());
        }
        geometry::RTree2D<double> line_rtree;
        line_rtree.Construct(boxes);

        // Endpoints of all level-0 markings. A higher-level marking that shares
        // one of these endpoints inside a level-0 region continues a level-0
        // marking, and is pulled into the level-0 arrangement even in a
        // vertically-overlapping area (see the level-0 exception below).
        std::unordered_set<RPoint2D> level0_endpoints;
        for (int k = 0; k < polylines.size(); ++k) {
            if (polyline_levels[k] == 0 && !polylines[k].empty()) {
                level0_endpoints.insert(polylines[k].front());
                level0_endpoints.insert(polylines[k].back());
            }
        }

        // Get per-overlays partition.
        cl::Array<int> entries;
        for (int overlay_index = 0; overlay_index < region_overlays->size();
             ++overlay_index) {
            auto& r = (*region_overlays)[overlay_index];
            for (auto& p : r.level_categories) {
                std::string category = GetCategory(p.second);
                p.second.clear();
                p.second.insert(category);

                int type = GetRoadCategory(category);
                CHECK(type != -1);
                int level = p.first;

                using Line = geometry::Arrangement2D<double>::Line;
                Array<Line> lines;

                // Add region's border.
                for (const auto& b : r.polygon.boundaries()) {
                    for (int i = 0; i < b.polygon.size(); ++i) {
                        lines.emplace_back(b.polygon.vertex(i),
                                           b.polygon.next_vertex(i),
                                           1 << type,
                                           0);
                    }
                }

                line_rtree.Find(r.polygon.bounding_box(), &entries);
                for (int j : entries) {
                    if (!geometry::Intersect(polylines[j], r.polygon)) {
                        continue;
                    }

                    if (r.level_categories.size() > 1) {
                        // In overlap region, we can add all road markings.
                        if (polyline_levels[j] != level &&
                            polyline_levels[j] != -1) {
                            continue;
                        }
                    }

                    int type = polyline_types[j];
                    if (type == -1) type = 0;

                    for (int i = 0; i + 1 < polylines[j].size(); ++i) {
                        lines.emplace_back(polylines[j][i],
                                           polylines[j][i + 1],
                                           1 << type,
                                           1 << type);
                    }
                }
                GetLaneRegions(r, overlay_index, lines, level, category,
                               lane_regions);
            }
        }

        // Merge the per-overlay lane regions of the same (level, category):
        // dissolve the overlay-border seams, keep the lane markings, so a lane
        // split across overlays becomes one region again.
        MergeLaneRegions(lane_regions);

        FindMiddleTurnMedian(lane_markings, lane_regions);
    }

    /**
     * Return true if the halfedge 'e' is a 'solid-dash' line.
     */
    static bool is_solid_dash_line(int e) {
        const int mask = 1 << EdgeType::SOLID_DASH_LINE;
        return (e & mask) != 0;
    }

    /**
     * Return true if the given edge type is a region edge.
     */
    static bool is_region_edge(int type) {
        int mask = (1 << ROAD)                 |
                   (1 << ROAD_INTERSECTION)    |
                   (1 << ROAD_OVERLAP)         |
                   (1 << VIRTUAL_INTERSECTION) |
                   (1 << RAISED_MEDIAN)        |
                   (1 << PAINTED_MEDIAN)       |
                   (1 << PARKING_RELATED)      |
                   (1 << GORE_AREA);
        return (type & mask) != 0;
    }

    /**
     * Return true if the given edge type is a region edge.
     */
    static bool is_lane_edge(int type) {
        int mask = (1 << ASSIST_LINE)     |
                   (1 << CUTLINE)         |
                   (1 << SOLID_DASH_LINE) |
                   (1 << WHITE_LINE)      |
                   (1 << YELLOW_LINE);
        return (type & mask) != 0;
    }

    /**
     * Return true if the given edge type is a raised median edge.
     */
    static bool is_raised_median_edge(int type) {
        int mask = (1 << RAISED_MEDIAN);
        return (type & mask) != 0;
    }

    /**
     * Return true if the given edge type is a painted median edge.
     */
    static bool is_painted_median_edge(int type) {
        int mask = (1 << PAINTED_MEDIAN);
        return (type & mask) != 0;
    }

    /**
     * Return true if the given edge type is working region's border.
     */
    static bool is_working_region_border(int type) {
        int mask = (1 << AOI_BORDER);
        return (type & mask) != 0;
    }

private:
    /**
     * Get the index of the specified road category.
     */
    static int GetRoadCategory(const std::string& title) {
        if (road_util::IsRoadSegment(title))
            return EdgeType::ROAD;
        if (title == "intersection_virtual")
            return EdgeType::VIRTUAL_INTERSECTION;
        if (road_util::IsRoadIntersection(title))
            return EdgeType::ROAD_INTERSECTION;
        if (road_util::IsRoadOverlap(title))
            return EdgeType::ROAD_OVERLAP;
        if (title == "raised_median" || title == "vegetation_median")
            return EdgeType::RAISED_MEDIAN;
        if (title == "painted_median")
            return EdgeType::PAINTED_MEDIAN;
        if (road_util::IsParkingRelated(title))
            return EdgeType::PARKING_RELATED;
        if (road_util::IsGore(title))
            return EdgeType::GORE_AREA;
        return -1;
    }

    /**
     * Get the type of the specified lane category.
     */
    static int GetLaneCategory(const std::string& title) {
        if (title == "assist_line")
            return EdgeType::ASSIST_LINE;
        if (title == "cut_line")
            return EdgeType::CUTLINE;
        if (lane_util::IsSolidDashLine(title))
            return EdgeType::SOLID_DASH_LINE;
        if (lane_util::IsWhiteLine(title))
            return EdgeType::WHITE_LINE;
        if (lane_util::IsYellowLine(title))
            return EdgeType::YELLOW_LINE;
        return -1;
    }

    /**
     * Return the unique category from overlay categories.
     */
    static std::string GetCategory(const std::set<std::string>& categories) {
        if (categories.size() == 1) return *categories.begin();

        std::string result;
        int rank = -1;
        for (auto category : categories) {
            int r = GetRoadCategory(category);
            if (r > rank) {
                rank = r;
                result = category;
            }
        }

        CHECK(rank != -1);
        return result;
    }

    /**
     * Get the title of the specified road category.
     */
    static std::string GetRoadCategoryTitle(int type) {
        switch (type) {
        case EdgeType::ROAD:
            return "road";
        case EdgeType::ROAD_INTERSECTION:
            return "road-intersection";
        case EdgeType::ROAD_OVERLAP:
            return "road_overlap";
        case EdgeType::VIRTUAL_INTERSECTION:
            return "virtual_intersection";
        case EdgeType::RAISED_MEDIAN:
            return "raised_median";
        case EdgeType::PAINTED_MEDIAN:
            return "painted_median";
        case EdgeType::PARKING_RELATED:
            return "parking";
        case EdgeType::GORE_AREA:
            return "gore_area";
        default:
            return "";
        }

        return "";
    }

    /**
     * Return true if higher-level marking continues a level-0 marking: one
     * of its endpoints coincides with a level-0 marking's endpoint and lies
     * inside one of the given level-0 regions. Such a marking is kept for the
     * level-0 arrangement even in vertically-overlapping areas.
     */
    bool ConnectsToLevel0Inside(
            const RPolyline2D& polyline,
            const std::unordered_set<RPoint2D>& level0_endpoints,
            const LandRegionOverlay& region_overlay) const {
        if (polyline.empty()) return false;

        const RPoint2D ends[2] = {polyline.front(), polyline.back()};
        for (const RPoint2D& p : ends) {
            if (level0_endpoints.find(p) == level0_endpoints.end()) continue;
            if (geometry::Intersect(p, region_overlay.polygon) ||
                Distance(p, region_overlay.polygon) < snap_threshold_) {
                return true;
            }
        }
        return false;
    }

    /**
     * Check if the arrangement's region is inside the road overlay.
     */
    bool IsInside(const LandRegionOverlay& region_overlay,
                  const Region& region) const {
        double total_area = 0.0;
        double inside_area = 0.0;
        for (auto tri : region) {
            RTriangle2D t(tri->halfedge->source_point(),
                          tri->halfedge->target_point(),
                          tri->halfedge->next()->target_point());
            double area = t.Area();
            total_area += area;
            RPoint2D c = Center(t.vertices());
            if (geometry::Intersect(region_overlay.polygon, c)) {
                inside_area += area;
            }
        }

        return inside_area * 2.0 > total_area;
    }

    /**
     * Get lane regions from the arrangement of region-overlays and polylines.
     */
    void GetLaneRegions(
            const LandRegionOverlay& region_overlay,
            int overlay_index,
            const Array<geometry::Arrangement2D<double>::Line>& lines,
            int level,
            const std::string& category,
            Array<LaneRegion>* lane_regions) const {
        geometry::Arrangement2D<double> arrangement(snap_threshold_);
        arrangement.Reset(lines);
        arrangement.Arrange();
        arrangement.RemoveCoincideEdges();
        for (auto e : arrangement.mesh()) {
            if (arrangement.is_constraint(e)) {
                const int c1 = arrangement.color(e);
                const int c2 = arrangement.color(e->twin());
                if (c1 != c2) continue;
                if (is_working_region_border(c1) ||
                    is_working_region_border(c2)) continue;

                if (is_region_edge(c1) && !is_lane_edge(c1)) {
                    arrangement.UnsetConstraintEdge(e);
                }
            }
        }
        arrangement.Arrange();

        using Edge = LaneRegion::Edge;
        Array<Array<Halfedge*>> edge_lists;
        for (auto [index, region] : enumerate(arrangement.regions())) {
            if (!IsInside(region_overlay, region))
                continue;

            LaneRegion lane_region;
            lane_region.id = lane_regions->size();
            lane_region.level = level;
            lane_region.category = category;
            lane_region.overlay = overlay_index;

            arrangement.GetRegion(index, &edge_lists);
            for (auto& list : edge_lists) {
                Array<Edge> ring;
                for (auto e : list) {
                    Edge e1;
                    e1.source = e->source_point();
                    e1.target = e->target_point();
                    e1.type = arrangement.color(e);
                    ring.push_back(e1);
                }
                lane_region.rings.push_back(ring);
            }
            arrangement.GetRegion(index, &lane_region.polygon);
            lane_regions->push_back(lane_region);
        }
    }

    /**
     * Merge the per-overlay lane regions of the same (level, category) by
     * running a second arrangement over their boundary edges -- mirrors
     * GetLaneRegions(). The overlay-border seams (region edges with the same
     * region on both sides) are dissolved while lane markings are kept, so a
     * lane that was split across overlays becomes a single region again.
     * Output format (per-lane LaneRegion) is unchanged.
     */
    void MergeLaneRegions(Array<LaneRegion>* lane_regions) const {
        std::map<std::pair<int, std::string>, Array<int>> groups;
        for (int i = 0; i < lane_regions->size(); ++i) {
            groups[std::make_pair((*lane_regions)[i].level,
                                  (*lane_regions)[i].category)].push_back(i);
        }

        using Line = geometry::Arrangement2D<double>::Line;
        using Edge = LaneRegion::Edge;
        Array<LaneRegion> merged;
        for (auto& group : groups) {
            const int level = group.first.first;
            const std::string& category = group.first.second;

            Array<Line> lines;
            Array<RBox2D> boxes;
            Array<const RMultiPolygon2D*> members;
            for (int idx : group.second) {
                const LaneRegion& lr = (*lane_regions)[idx];
                std::set<RSegment2D> s;
                for (const auto& ring : lr.rings) {
                    for (const auto& e : ring) {
                        int c2 = is_lane_edge(e.type) ? e.type : 0;
                        s.insert(RSegment2D(e.source, e.target));
                        lines.emplace_back(e.source, e.target, e.type, c2);
                    }
                }
                members.push_back(&lr.polygon);
                boxes.push_back(lr.polygon.bounding_box());
            }
            geometry::RTree2D<double> rtree;
            rtree.Construct(boxes);

            geometry::Arrangement2D<double> arrangement;
            arrangement.ResetWithoutCross(lines);
            arrangement.Arrange();
            arrangement.RemoveCoincideEdges();
            for (auto e : arrangement.mesh()) {
                if (!arrangement.is_constraint(e)) continue;
                const int c1 = arrangement.color(e);
                const int c2 = arrangement.color(e->twin());
                if (c1 != c2) continue;
                if (is_working_region_border(c1) ||
                    is_working_region_border(c2)) continue;
                if (is_region_edge(c1) && !is_lane_edge(c1)) {
                    arrangement.UnsetConstraintEdge(e);
                }
            }
            arrangement.Arrange();

            Array<int> entries;
            Array<Array<Halfedge*>> edge_lists;
            for (auto [index, region] : enumerate(arrangement.regions())) {
                // Keep only faces inside the merged road area (drop the outer
                // face and any holes).
                double total = 0.0, inside = 0.0;
                for (auto tri : region) {
                    RTriangle2D t(tri->halfedge->source_point(),
                                  tri->halfedge->target_point(),
                                  tri->halfedge->next()->target_point());
                    double area = t.Area();
                    total += area;
                    RPoint2D c = Center(t.vertices());
                    rtree.Find(c.bounding_box(), &entries);
                    for (int k : entries) {
                        if (geometry::Intersect(*members[k], c)) {
                            inside += area;
                            break;
                        }
                    }
                }
                if (inside * 2.0 <= total) continue;

                LaneRegion lane_region;
                lane_region.id = merged.size();
                lane_region.level = level;
                lane_region.category = category;

                arrangement.GetRegion(index, &edge_lists);
                for (auto& list : edge_lists) {
                    Array<Edge> ring;
                    for (auto e : list) {
                        Edge e1;
                        e1.source = e->source_point();
                        e1.target = e->target_point();
                        e1.type = arrangement.color(e);
                        ring.push_back(e1);
                    }
                    lane_region.rings.push_back(ring);
                }
                arrangement.GetRegion(index, &lane_region.polygon);
                merged.push_back(lane_region);
            }
        }

        *lane_regions = merged;
    }

    /**
     * Special treat for "middle_turn_median".
     */
    void FindMiddleTurnMedian(const Array<LaneMarking>& lane_markings,
                              Array<LaneRegion>* lane_regions) const {
        Array<RPoint2D> left_turns;
        for (const auto& marking : lane_markings) {
            if (lane_util::IsLeftTurn(marking.category)) {
                Array<RPoint2D> points(marking.polyline.begin(),
                                       marking.polyline.end());
                left_turns.push_back(Center(points));
            }
        }

        geometry::RTree2D<double> rtree;
        Array<RBox2D> left_turn_boxes(left_turns.size());
        for (int i = 0; i < left_turn_boxes.size(); ++i) {
            left_turn_boxes[i] = left_turns[i].bounding_box();
        }
        rtree.Construct(left_turn_boxes);

        Array<int> entries;
        for (auto& r : *lane_regions) {
            std::string& category = r.category;
            if (!road_util::IsMedian(category)) continue;

            rtree.Find(r.polygon.bounding_box(), &entries);
            int n_turns = 0;
            for (int j : entries) {
                if (geometry::Intersect(left_turns[j], r.polygon)) {
                    ++n_turns;
                }
            }
            if (n_turns >= 1) {
                category = "middle_turn_median";
                continue;
            }

            for (auto& ring : r.rings) {
                for (auto e : ring) {
                    if (is_solid_dash_line(e.type)) {
                        category = "middle_turn_median";
                        break;
                    }
                }
                if (category == "middle_turn_median") break;
            }
        }
    }

    // The snap distance for snapping.
    double snap_threshold_ = 1e-3;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_ARRANGEMENT_H_

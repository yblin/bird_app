//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ARRANGEMENT_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ARRANGEMENT_H_

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/envelope/min_enclosing_rectangle_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/geo_graph.h"
#include "codelibrary/geometry/topology/polyline_regularization_2d.h"
#include "codelibrary/geometry/transform_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/land_cover.h"
#include "codelibrary/gis/road_system/lane_region_graph.h"
#include "codelibrary/gis/road_system/lane_util.h"
#include "codelibrary/gis/road_system/road_util.h"

namespace cl {
namespace gis {

/**
 * Arrangement for constructing closed lane polygons of a road.
 * The primary goal is to convert the lane polylines into enclosed polygonal
 * areas, leveraging road landcover polygons to determine boundary constraints
 * for completing the closure of each lane.
 */
class LaneArrangement {
    using Vertex = geometry::Arrangement2D<double>::Vertex;
    using Halfedge = geometry::Arrangement2D<double>::Halfedge;
    using Region = geometry::Arrangement2D<double>::Region;

public:
    enum class RoadCategory {
        ROAD = 0,
        ROAD_OVERLAP = 1,
        ROAD_INTERSECTION = 2,
        MEDIAN = 3,
        PARKING_RELATED = 4,
        COUNT
    };

    enum class LaneCategory {
        ASSIST_LINE = 5,
        CUT_LINE = 6,
        WHITE_LINE = 7,
        YELLOW_LINE = 8
    };

    explicit LaneArrangement(double snap_threshold = 1.0)
        : snap_threshold_(snap_threshold) {
        CHECK(snap_threshold_ > 0.0);
    }

    LaneArrangement(const LaneArrangement&) = delete;

    LaneArrangement& operator=(const LaneArrangement&) = delete;

    /**
     * Construct closed regions from road land cover and lane polylines.
     */
    void Arrange(const LandCover& road_surface,
                 const Array<RPolyline2D>& lanes,
                 const Array<std::string>& lane_titles) {
        CHECK(lanes.size() == lane_titles.size());

        if (road_surface.empty() || lanes.empty()) return;

        road_surface.ToPolygonCategoryPairs(&road_regions_, &road_categories_);

        // Categories need to snap.
        static const Array<std::string> snap_categories = {
            "painted_median",
            "gore_area",
            "parking",
            "inside_parking",
            "no_parking_area"
        };

        Array<RPolyline2D> polylines, snap_polylines;
        Array<int> category_indices, snap_category_indices;

        // Add road boundaries.
        for (const auto& geos : road_surface.geometries()) {
            int index = road_category(geos.first);
            if (index == -1) continue;

            bool need_snap = snap_categories.find(geos.first) !=
                             snap_categories.end();

            for (const auto& polygon : geos.second) {
                for (const auto& b : polygon.boundaries()) {
                    auto vertices = b.polygon.vertices();
                    vertices.push_back(vertices.front());
                    if (need_snap) {
                        snap_polylines.emplace_back(vertices);
                        snap_category_indices.push_back(index);
                    } else {
                        polylines.emplace_back(vertices);
                        category_indices.push_back(index);
                    }
                }
            }
        }

        // Add lanes.
        for (auto&& [i, lane] : enumerate(lanes)) {
            int index = lane_category(lane_titles[i]);
            if (index == -1) continue;

            polylines.emplace_back(lane);
            category_indices.push_back(index);
        }

        SnapPolylines(polylines, &snap_polylines);
        polylines.insert(snap_polylines);
        category_indices.insert(snap_category_indices);
        geometry::SnapPolylines(epsilon_, &polylines);

        const int n_road_categories = static_cast<int>(RoadCategory::COUNT);
        using Line = geometry::Arrangement2D<double>::Line;
        Array<Line> lines;
        for (auto&& [index, polyline] : enumerate(polylines)) {
            if (category_indices[index] < n_road_categories) {
                for (int i = 0; i + 1 < polyline.size(); ++i) {
                    lines.emplace_back(polyline[i], polyline[i + 1],
                                       1 << category_indices[index], 0);
                }
            } else {
                for (int i = 0; i + 1 < polyline.size(); ++i) {
                    lines.emplace_back(polyline[i], polyline[i + 1],
                                       1 << category_indices[index],
                                       1 << category_indices[index]);
                }
            }
        }
        arrangement_.set_threshold(epsilon_);
        arrangement_.Reset(lines);
        arrangement_.Arrange();

        RemoveSmallRegions();
        AssignCategories();
    }

    /**
     * Extract land cover from the arrangement results.
     */
    void ExtractLandCover(LandCover* land_cover) const {
        CHECK(land_cover);

        land_cover->clear();
        if (arrangement_.empty()) return;

        RMultiPolygon2D region_polygon;
        for (int i = 0; i < arrangement_.regions().size(); ++i) {
            std::string category = categories_[i];
            if (category.empty()) continue;

            arrangement_.GetRegion(i, &region_polygon);
            land_cover->AddGeometry(category, region_polygon);
        }
    }

    /**
     * Check if the i-th region is a lane region.
     */
    bool IsLaneRegion(int i) const {
        CHECK(i >= 0 && i < arrangement_.regions().size());

        std::string category = categories_[i];
        if (!road_util::IsRoadSegment(category) &&
            !road_util::IsRoadIntersection(category) &&
            !road_util::IsRoadOverlap(category)) {
            return false;
        }

        Array<Array<Halfedge*>> lane_halfedges;
        arrangement_.GetRegion(i, &lane_halfedges);

        if (road_util::IsRoadIntersection(category)) {
            if (lane_halfedges.size() >= 2) return false;
            auto& edges = lane_halfedges.front();
            if (CountContinuousCutlines(edges) > 2) return false;
        }

        return true;
    }

    /**
     * Extract the lane region graph from the arrangement results.
     */
    void ExtractLaneRegionGraph(LaneRegionGraph* lane_graph) const {
        lane_graph->clear();
        if (arrangement_.empty()) return;

        geometry::GeoGraph<RPoint2D> graph;
        std::unordered_map<int, bool> is_cutline;
        is_cutline.reserve(arrangement_.mesh().n_halfedges());
        Array<Array<Halfedge*>> edges;
        for (int i = 0; i < arrangement_.regions().size(); ++i) {
            if (!IsLaneRegion(i)) continue;

            arrangement_.GetRegion(i, &edges);

            Array<Array<RPoint2D>> face;
            for (auto es : edges) {
                Array<RPoint2D> ring;
                for (auto e : es) {
                    int eid = graph.InsertEdge(e->source_point(),
                                               e->target_point());
                    is_cutline[eid] = IsCutline(e);
                    ring.push_back(e->source_point());
                }
                face.push_back(ring);
            }
            graph.InsertFace(face);
        }

        lane_graph->Reset(graph, is_cutline);
    }

    /**
     * Get the index of the specified road category in the 'road_categories'.
     */
    int road_category(const std::string& title) const {
        if (road_util::IsRoadSegment(title))
            return static_cast<int>(RoadCategory::ROAD);
        if (road_util::IsRoadOverlap(title))
            return static_cast<int>(RoadCategory::ROAD_OVERLAP);
        if (road_util::IsRoadIntersection(title))
            return static_cast<int>(RoadCategory::ROAD_INTERSECTION);
        if (road_util::IsMedian(title))
            return static_cast<int>(RoadCategory::MEDIAN);
        if (road_util::IsParkingRelated(title))
            return static_cast<int>(RoadCategory::PARKING_RELATED);
        return -1;
    }

    /**
     * Get the index of the specified lane category in the 'lane_categories'.
     */
    int lane_category(const std::string& title) const {
        if (title == "assist_line")
            return static_cast<int>(LaneCategory::ASSIST_LINE);
        if (title == "cut_line")
            return static_cast<int>(LaneCategory::CUT_LINE);
        if (lane_util::IsWhiteLine(title))
            return static_cast<int>(LaneCategory::WHITE_LINE);
        if (lane_util::IsYellowLine(title))
            return static_cast<int>(LaneCategory::YELLOW_LINE);
        return -1;
    }

    /**
     * Return arrangement results.
     */
    const geometry::Arrangement2D<double>& arrangement() const {
        return arrangement_;
    }

private:
    /**
     * Determine whether the given region should be ignored.
     * A region is ignored if the minimum width of its minimal enclosing
     * rectangle is smaller than the snap threshold.
     */
    bool ShouldIgnoreRegion(const Region& region) const {
        Array<RPoint2D> vertices;
        for (auto t : region) {
            vertices.push_back(t->halfedge->source_point());
            vertices.push_back(t->halfedge->target_point());
            vertices.push_back(t->halfedge->next()->target_point());
        }

        Array<RPoint2D> rectangle;
        if (!geometry::MinEnclosingRectangle2D(vertices, &rectangle)) {
            return true;
        }

        double w = std::min(Distance(rectangle[0], rectangle[1]),
                            Distance(rectangle[0], rectangle[3]));
        return w < snap_threshold_;
    }

    /**
     * Assign a category to the given arragement's region according to the input
     * road land cover.
     */
    std::string GetCategory(const Region& region,
                            const geometry::RTree2D<double>& rtree) const {
        Array<int> entries;
        RBox2D box;
        for (auto tri : region) {
            Halfedge* e = tri->halfedge;
            box.Union(e->source_point().bounding_box());
            box.Union(e->target_point().bounding_box());
            box.Union(e->next()->target_point().bounding_box());
        }
        rtree.Find(box, &entries);

        if (entries.empty()) return "";

        std::map<std::string, double> category_score;
        double total_area = 0.0;
        for (auto tri : region) {
            RTriangle2D t(tri->halfedge->source_point(),
                          tri->halfedge->target_point(),
                          tri->halfedge->next()->target_point());
            double area = t.Area();
            total_area += area;
            RPoint2D c = Center(t.vertices());
            bool has = false;
            for (int j : entries) {
                if (geometry::Intersect(road_regions_[j], c)) {
                    category_score[road_categories_[j]] += area;
                    has = true;
                }
            }
            if (!has) {
                category_score[""] += t.Area();
            }
        }

        Array<std::string> categroies;
        for (auto score : category_score) {
            if (score.second * 2.0 > total_area)
                categroies.push_back(score.first);
        }
        if (categroies.empty()) return "";

        std::sort(categroies.begin(), categroies.end(),
                  [this](const std::string& a, const std::string& b) {
            return road_category(a) > road_category(b);
        });

        if (road_category(categroies.front()) == -1) return "";
        return categroies.front();
    }

    /**
     * Check if a halfedge is a cutline.
     */
    bool IsCutline(const Halfedge* e) const {
        if (!arrangement_.is_constraint(e)) return false;

        const int color = arrangement_.color(e) | arrangement_.color(e->twin());

        int t1 = (1 << static_cast<int>(RoadCategory::ROAD_INTERSECTION)) |
                 (1 << static_cast<int>(RoadCategory::ROAD));
        int t2 = (1 << static_cast<int>(RoadCategory::ROAD_OVERLAP)) |
                 (1 << static_cast<int>(RoadCategory::ROAD));
        return (color & t1) == t1 || (color & t2) == t2 ||
               (color & (1 << static_cast<int>(LaneCategory::CUT_LINE)));
    }

    /**
     * Check if a halfedge is a lane line.
     */
    bool IsLane(const Halfedge* e) const {
        if (!arrangement_.is_constraint(e)) return false;

        const int color = arrangement_.color(e);
        return color >= (1 << (static_cast<int>(RoadCategory::COUNT)));
    }

    /**
     * Count the number of continuous cut-lines in the given region.
     */
    int CountContinuousCutlines(const Array<Halfedge*>& edges) const {
        int n = edges.size();
        if (n == 0) return 0;

        int count = 0;
        bool in_cut = false;

        for (int i = 0; i < n; ++i) {
            bool is_current_cut = IsCutline(edges[i]);

            if (is_current_cut && !in_cut) {
                ++count;
                in_cut = true;
            } else if (!is_current_cut) {
                in_cut = false;
            }
        }

        bool is_first_cut = IsCutline(edges.front());
        bool is_last_cut = IsCutline(edges.back());
        if (is_first_cut && is_last_cut && count > 1) {
            --count;
        }

        return count;
    }

    /**
     * Snap the polylines in 'snap_polylines' to the nearest segments of the
     * reference 'polylines'. This operation adjusts the vertices of
     * 'snap_polylines' to ensure they lie precisely on or near the reference
     * polylines.
     */
    void SnapPolylines(const Array<RPolyline2D>& polylines,
                       Array<RPolyline2D>* snap_polylines) const {
        CHECK(snap_polylines);

        if (polylines.empty() || snap_polylines->empty()) return;

        Array<RSegment2D> ref_segments;
        Array<RPoint2D> ref_vertices;
        for (int i = 0; i < polylines.size(); ++i) {
            ref_vertices.insert(polylines[i]);
            for (int j = 0; j + 1 < polylines[i].size(); ++j) {
                ref_segments.emplace_back(polylines[i][j], polylines[i][j + 1]);
            }
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(ref_segments);

        Array<int> entries;
        for (auto& polyline : *snap_polylines) {
            RPolyline2D densified_polyline;
            geometry::polyline_util::Densify(polyline, snap_threshold_,
                                             &densified_polyline);
            for (auto& p : densified_polyline) {
                RBox2D box(p.x - snap_threshold_, p.x + snap_threshold_,
                           p.y - snap_threshold_, p.y + snap_threshold_);
                rtree.Find(box, &entries);

                RSegment2D nearest;
                double dis = DBL_MAX;
                for (int j : entries) {
                    double d = Distance(p, ref_segments[j]);
                    if (d < dis) {
                        dis = d;
                        nearest = ref_segments[j];
                    }
                }

                if (dis <= snap_threshold_) {
                    p = geometry::Project(p, nearest);
                }
            }
            geometry::polyline_util::DouglasPeuckerSimplify(densified_polyline,
                                                            epsilon_,
                                                            &polyline);
        }
    }

    /**
     * Remove the small regions from the arrangement.
     */
    void RemoveSmallRegions() {
        bool finish = false;
        int n_steps = 30;
        while (!finish && n_steps--) {
            finish = true;
            // Make sure each region only be modified once.
            std::unordered_set<int> modified_regions;
            Array<Halfedge*> removed_edges;

            for (int i = 0; i < arrangement_.regions().size(); ++i) {
                if (modified_regions.find(i) != modified_regions.end())
                    continue;
                if (!ShouldIgnoreRegion(arrangement_.regions()[i])) continue;

                Array<Array<Halfedge*>> edges;
                arrangement_.GetRegion(i, &edges);

                // Remove the longest edge of the region.
                for (auto l : edges) {
                    double length = 0.0;
                    Halfedge* longest_e = nullptr;
                    for (auto e : l) {
                        double d = cl::Distance(e->source_point(),
                                                e->target_point());
                        if (d > length) {
                            longest_e = e;
                            length = d;
                        }
                    }
                    if (longest_e) {
                        finish = false;
                        removed_edges.push_back(longest_e);

                        auto face = arrangement_.face(longest_e->twin());
                        if (face) {
                            int r = face->region_id;
                            if (r != -1) {
                                modified_regions.insert(r);
                            }
                        }
                        modified_regions.insert(i);
                    }
                }
            }

            for (auto e : removed_edges) {
                arrangement_.UnsetConstraintEdge(e);
            }
            arrangement_.Arrange();
            arrangement_.RemoveCoincideEdges();
        }
    }

    /**
     * Set category title for each region in arrangement.
     */
    void AssignCategories() {
        categories_.clear();

        Array<RBox2D> boxes;
        for (const auto& r : road_regions_) {
            boxes.push_back(r.bounding_box());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        for (int i = 0; i < arrangement_.regions().size(); ++i) {
            auto& region = arrangement_.regions()[i];
            std::string category = GetCategory(region, rtree);
            categories_.push_back(category);
        }
    }

    // Snap distance for painted_median, gore_area, parking, etc.
    double snap_threshold_ = 1.0;

    // The snap distance for micro snapping.
    double epsilon_ = 1e-2;

    // Lane arrangement result.
    geometry::Arrangement2D<double> arrangement_;

    // Corresponding categories for each region in the arrangement.
    Array<std::string> categories_;

    // Land cover for the arrangement.
    LandCover land_cover_;

    // The input road regions.
    Array<RMultiPolygon2D> road_regions_;

    // The input road categories.
    Array<std::string> road_categories_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ARRANGEMENT_H_

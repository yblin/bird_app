//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_TOPOLOGY_LAND_REGION_ARRANGEMENT_H_
#define CODELIBRARY_GIS_TOPOLOGY_LAND_REGION_ARRANGEMENT_H_

#include <map>
#include <set>

#include "codelibrary/gis/land_region.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"

namespace cl {
namespace gis {

/**
 * Arrange a collection of LandRegions (each with a vertical level and category)
 * into a set of non-overlapping polygon regions.
 *
 * The core functionality is to process overlapping input LandRegions—whether
 * overlapping due to different vertical levels or different categories, and
 * split/merge them into unified, non-overlapping regions. Each resulting region
 * retains all levels and categories from the original overlapping LandRegions
 * that cover it.
 *
 * For example:
 * - Overlapping road regions with levels 1 and 2 are merged into a single
 *   non-overlapping polygon, which stores both levels 1 and 2.
 * - Overlapping regions with categories "road" and "median" are merged into a
 *   single non-overlapping polygon, which stores both categories.
 */
class LandRegionArrangement {
    using Vertex   = geometry::Arrangement2D<double>::Vertex;
    using Halfedge = geometry::Arrangement2D<double>::Halfedge;
    using Region   = geometry::Arrangement2D<double>::Region;

public:
    explicit LandRegionArrangement(double snap_threshold = 0.01)
        : snap_threshold_(snap_threshold) {
        CHECK(snap_threshold_ > 0.0);
    }

    LandRegionArrangement(const LandRegionArrangement&) = delete;

    LandRegionArrangement& operator=(const LandRegionArrangement&) = delete;

    /**
     * Arrange the land regions into land region overlays, including split and
     * merge.
     */
    void Arrange(const Array<LandRegion>& regions) {
        region_overlays_.clear();

        if (regions.empty()) return;

        input_regions_ = regions;

        Array<RBox2D> boxes;
        for (const auto& r : regions) {
            boxes.push_back(r.polygon.bounding_box());
        }
        road_rtree_.Construct(boxes);

        using Line = geometry::Arrangement2D<double>::Line;
        Array<Line> lines;
        for (const auto& region : regions) {
            for (const auto& b : region.polygon.boundaries()) {
                if (b.is_outer != b.polygon.IsClockwise()) {
                    for (int i = 0; i < b.polygon.size(); ++i) {
                        lines.emplace_back(b.polygon.vertex(i),
                                           b.polygon.next_vertex(i),
                                           1,
                                           0);
                    }
                } else {
                    for (int i = 0; i < b.polygon.size(); ++i) {
                        lines.emplace_back(b.polygon.next_vertex(i),
                                           b.polygon.vertex(i),
                                           1,
                                           0);
                    }
                }
            }
        }

        arrangement_.set_threshold(snap_threshold_);
        arrangement_.Reset(lines);
        arrangement_.Arrange();

        GetRegionOverlays();
        MergeRegionOverlays();
    }

    /**
     * Merge region overlays according to their level_categories.
     */
    void MergeRegionOverlays() {
        if (region_overlays_.empty()) return;

        // Merge regions with the same type.
        Array<Halfedge*> removed_edges;
        for (auto e : arrangement_.mesh()) {
            if (!arrangement_.is_constraint(e)) continue;
            int r1 = arrangement_.region_id(e);
            int r2 = arrangement_.region_id(e->twin());
            if (r1 != -1 && r2 != -1 && r1 != r2) {
                int o1 = region_map_[r1], o2 = region_map_[r2];
                if (o1 == -1 || o2 == -1) continue;
                if (region_overlays_[o1].level_categories ==
                    region_overlays_[o2].level_categories) {
                    removed_edges.push_back(e);
                }
            }
        }
        if (removed_edges.empty()) return;
        for (auto e : removed_edges) {
            arrangement_.UnsetConstraintEdge(e);
        }

        arrangement_.Arrange();
        GetRegionOverlays();
    }

    /**
     * Merge adjacent regions at the same level (ignoring categories) to
     * generate land regions of different levels.
     */
    void MergeSameLevelRegions(Array<LandRegion>* regions) const {
        CHECK(regions);

        regions->clear();
        if (region_overlays_.empty()) return;

        std::map<int, Array<int>> level_regions;
        for (const auto& r : region_overlays_) {
            for (const auto& p : r.level_categories) {
                level_regions[p.first].push_back(r.id);
            }
        }

        Array<int> entries;
        for (auto& group : level_regions) {
            int level = group.first;
            Array<int>& overlay_indices = group.second;

            using Line = geometry::Arrangement2D<double>::Line;
            Array<Line> lines;
            for (int o : overlay_indices) {
                const auto& r = region_overlays_[o];

                // Add region's border.
                for (const auto& b : r.polygon.boundaries()) {
                    for (int i = 0; i < b.polygon.size(); ++i) {
                        lines.emplace_back(b.polygon.vertex(i),
                                           b.polygon.next_vertex(i),
                                           1,
                                           0);
                    }
                }
            }

            geometry::Arrangement2D<double> arrangement;
            arrangement.Reset(lines);
            arrangement.Arrange();
            arrangement.RemoveCoincideEdges();
            for (auto e : arrangement.mesh()) {
                if (arrangement.is_constraint(e)) {
                    const int c1 = arrangement.color(e);
                    const int c2 = arrangement.color(e->twin());
                    if (c1 != c2) continue;

                    if (c1 == c2 && c1 == 1) {
                        arrangement.UnsetConstraintEdge(e);
                    }
                }
            }
            arrangement.Arrange();

            using Halfedge = geometry::Arrangement2D<double>::Halfedge;
            Array<Array<Halfedge*>> edge_lists;
            for (int i = 0; i < arrangement.regions().size(); ++i) {
                arrangement.GetRegion(i, &edge_lists);
                bool outside = false;
                for (auto& list : edge_lists) {
                    for (auto e : list) {
                        if (arrangement.color(e) == 0) {
                            outside = true;
                            break;
                        }
                    }
                }
                if (outside) continue;

                LandRegion land_region;
                land_region.id = regions->size();
                land_region.level = level;
                arrangement.GetRegion(i, &land_region.polygon);
                regions->push_back(land_region);
            }
        }
    }

    /**
     * Return arrangement results.
     */
    const geometry::Arrangement2D<double>& arrangement() const {
        return arrangement_;
    }

    /**
     * Return the land region overlays results.
     */
    const Array<LandRegionOverlay>& region_overlays() const {
        return region_overlays_;
    }

    /**
     * Allow users to modify the category set of a specific level in a target
     * region overlay and the arrangement results can be further adjusted by the
     * MergeRegionOverlays() function.
     */
    void set_level_categories(int i, int level,
                              const std::set<std::string>& categories) {
        CHECK(i >= 0 && i < region_overlays_.size());

        auto& level_categories = region_overlays_[i].level_categories;
        level_categories[level] = categories;
    }

    /**
     * Similar to the set_level_categories(), but assigns a single uniform
     * category instead of a category set to the specific level of the i-th
     * region overlay.
     */
    void set_level_category(int i, int level, const std::string& category) {
        CHECK(i >= 0 && i < region_overlays_.size());

        auto& level_categories = region_overlays_[i].level_categories;
        level_categories[level].clear();
        level_categories[level].insert(category);
    }

private:
    /**
     * Get land region overlay covered by the i-th arragement's region.
     *
     * Return false if it is a outer region (ignored).
     */
    bool GetLandRegionOverlay(int i, LandRegionOverlay* region_overlay) {
        CHECK(i >= 0 && i < arrangement_.regions().size());
        CHECK(region_overlay);

        region_overlay->level_categories.clear();

        Array<int> entries;
        RBox2D box;
        for (auto tri : arrangement_.region(i)) {
            Halfedge* e = tri->halfedge;
            box.Union(e->source_point().bounding_box());
            box.Union(e->target_point().bounding_box());
            box.Union(e->next()->target_point().bounding_box());
        }
        road_rtree_.Find(box, &entries);
        if (entries.empty()) return false;

        std::map<std::pair<int, std::string>, double> category_area;
        double total_area = 0.0;
        for (auto tri : arrangement_.region(i)) {
            RTriangle2D t(tri->halfedge->source_point(),
                          tri->halfedge->target_point(),
                          tri->halfedge->next()->target_point());
            double area = t.Area();
            total_area += area;
            RPoint2D c = Center(t.vertices());
            bool has = false;
            for (int j : entries) {
                if (geometry::Intersect(input_regions_[j].polygon, c)) {
                    auto p = std::make_pair(input_regions_[j].level,
                                            input_regions_[j].category);
                    category_area[p] += area;
                    has = true;
                }
            }
            if (!has) {
                auto p = std::make_pair(-1, "");
                category_area[p] += t.Area();
            }
        }

        for (auto score : category_area) {
            if (score.second * 2.0 > total_area) {
                if (score.first.second == "") {
                    return false;
                }
                region_overlay->level_categories[score.first.first].insert(
                            score.first.second);
            }
        }
        if (region_overlay->level_categories.empty()) return false;

        arrangement_.GetRegion(i, &region_overlay->polygon);
        region_overlay->id = region_overlays_.size();
        region_overlays_.push_back(*region_overlay);

        return true;
    }

    /**
     * Get overlay results from the arrangement.
     */
    void GetRegionOverlays() {
        region_overlays_.clear();
        region_map_.clear();

        LandRegionOverlay overlay;
        for (int i = 0; i < arrangement_.regions().size(); ++i) {
            if (GetLandRegionOverlay(i, &overlay)) {
                region_map_[i] = overlay.id;
            } else {
                region_map_[i] = -1;
            }
        }
    }

    // Snap distance threshold.
    double snap_threshold_;

    // Lane arrangement result.
    geometry::Arrangement2D<double> arrangement_;

    // The input land regions.
    Array<LandRegion> input_regions_;

    // Rtree for road regions.
    geometry::RTree2D<double> road_rtree_;

    // Map arrangement region to region_overlay.
    std::unordered_map<int, int> region_map_;

    // The output land region overlays.
    Array<LandRegionOverlay> region_overlays_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_TOPOLOGY_LAND_REGION_ARRANGEMENT_H_

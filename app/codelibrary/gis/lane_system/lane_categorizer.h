//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_CATEGORIZER_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_CATEGORIZER_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/lane_system/lane_marking_query.h"
#include "codelibrary/gis/lane_system/lane_region_graph.h"
#include "codelibrary/gis/lane_system/lane_util.h"
#include "codelibrary/statistics/mean.h"

namespace cl {
namespace gis {

/**
 * Assigning categories to lanes.
 */
class LaneCategorizer {
    using Region = LaneRegionGraph::Region;
    using RegionGroup = LaneRegionGraph::RegionGroup;
    using Lane = CartoPolyline2D;

public:
    LaneCategorizer() {
        aoi_markings_ = {
            "bike_sign",
            "share_bike_sign",
            "left_turn",
            "right_turn",
            "straight_left_turn",
            "straight_right_turn",
            "triple_turn",
            "merge_arrow_left",
            "merge_arrow_right",
            "u_turn",
        };
        marking_to_category_ = {
            {"bike_sign",           "bike_lane"},
            {"share_bike_sign",     "shared_bl"},
            {"protected_bl",        "protected_bl"},
            {"buffered_bl",         "buffered_bl"},
            {"left_turn",           "left_turn"},
            {"right_turn",          "right_turn"},
            {"through",             "through"},
            {"straight_left_turn",  "left_through"},
            {"straight_right_turn", "right_through"},
            {"triple_turn",         "triple_turn"},
            {"merge_arrow_left",    "merge_arrow_left"},
            {"merge_arrow_right",   "merge_arrow_right"},
            {"u_turn",              "u_turn"},
            {"shoulder",            "shoulder"},
            {"non_lane",            "non_lane"},
            {"middle_turn_median",  "middle_turn"},
        };
    }

    LaneCategorizer(const LaneCategorizer&) = delete;

    LaneCategorizer& operator=(const LaneCategorizer&) = delete;

    /**
     * Assign categories for each region in the lane region graph.
     */
    void AssignCategories(const Array<RegionGroup>& regions,
                          const Array<Lane>& lanes,
                          const Array<LaneMarking>& lane_markings,
                          Array<std::string>* categories) const {
        CHECK(regions.size() == lanes.size());
        CHECK(categories);

        categories->assign(regions.size(), "unknown");
        for (int i = 0; i < regions.size(); ++i) {
            if (regions[i].category == "shoulder" ||
                regions[i].category == "middle_turn_median" ||
                regions[i].category == "non_lane") {
                (*categories)[i] = regions[i].category;
            }
        }

        Array<LaneMarking> aoi_markings;
        for (int i = 0; i < lane_markings.size(); ++i) {
            if (aoi_markings_.find(lane_markings[i].category) ==
                aoi_markings_.end()) continue;
            aoi_markings.push_back(lane_markings[i]);
        }
        LaneMarkingQuery query(aoi_markings);

        Array<LaneMarking> entries;
        for (int i = 0; i < regions.size(); ++i) {
            std::string& category = (*categories)[i];
            if (category != "unknown") continue;

            const auto& region = regions[i];
            query.Query(region.polygon, &entries);
            for (LaneMarking& marking : entries) {
                if (marking.level == region.level) {
                    AssignCategory(marking.category, &category);
                }
            }
        }

        // Find shoulders.
        for (int i = 0; i < regions.size(); ++i) {
            std::string& category = (*categories)[i];
            if (category != "unknown") continue;
            if (regions[i].category != "shoulder_candidate") continue;

            if (IsShoulder(lanes[i])) {
                category = "shoulder";
            }
        }

        for (int i = 0; i < regions.size(); ++i) {
            std::string& category = (*categories)[i];
            if (category == "unknown") {
                category = "through";
            }
        }
    }

    /**
     * Convert region category to lane branch category.
     */
    std::string to_lane_branch_category(const std::string& category) const {
        auto it = marking_to_category_.find(category);
        if (it != marking_to_category_.end()) {
            return it->second;
        }
        return "through";
    }

private:
    /**
     * Assign a specified category to 'dst', with special handling when 'dst'
     *  already has a non-empty category.
     */
    void AssignCategory(const std::string& category, std::string* dst) const {
        CHECK(dst);

        if (dst->empty()) {
            *dst = category;
            return;
        }

        auto i1 = aoi_markings_.find(*dst);
        auto i2 = aoi_markings_.find(category);
        CHECK(i2 != aoi_markings_.end());

        if (i1 == aoi_markings_.end() || i2 < i1) {
            *dst = category;
        }
    }

    /**
     * Return true if the given lane is a shoulder.
     */
    bool IsShoulder(const Lane& lane) const {
        if (lane.empty()) return false;

        double mean_w = Mean(lane.widths.begin(), lane.widths.end());
        return mean_w < shoulder_width_threshold_;
    }

    // Width threshold to determine if a lane region is a shoulder.
    double shoulder_width_threshold_ = 3.0;

    // AOI road markings.
    Array<std::string> aoi_markings_;

    // Covert road marking to lane category.
    std::map<std::string, std::string> marking_to_category_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_CATEGORIZER_H_

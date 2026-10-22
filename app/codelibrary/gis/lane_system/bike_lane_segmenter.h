//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_BRANCH_SEGMENTER_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_BRANCH_SEGMENTER_H_

#include "codelibrary/gis/carto_polyline_util.h"
#include "codelibrary/gis/lane_system/lane_region_graph.h"

namespace cl {
namespace gis {

/**
 * Split a bike lane into traditional bike, protected bike and buffered bike
 * lanes based on the border type of the bike lane region.
 *
 * Protected bike lane (also known as separated bike lane): Physically
 * separated from motor vehicle lanes/parking areas by
 * RAISED_MEDIAN_BORDER.
 *
 * Buffered Bike Lane: Features additional buffer space delineated by
 * PAINTED_MEDIAN_BORDER markings, separating the bike lane from adjacent
 * motor vehicle lanes/parking areas.
 *
 * Parameters:
 *  region        - The region to which the bike lane belongs.
 *  branch        - The bike lane branch to be split.
 *  resolution    - Resolution of the lane branch (see LaneCenterline).
 *  min_length    - The minimal length of a bike branch.
 *  bike_branches - array storing the split bike lanes.
 */
class BikeLaneSegmenter {
    using Lane = CartoPolyline2D;
    using RegionGroup = LaneRegionGraph::RegionGroup;

public:
    BikeLaneSegmenter() = default;

    BikeLaneSegmenter(const BikeLaneSegmenter&) = delete;

    BikeLaneSegmenter& operator=(const BikeLaneSegmenter&) = delete;

    /**
     * Split a bike lane into normal bike, protected bike and buffered bike
     * lanes based on the border type of the bike lane region.
     *
     * Parameters:
     *  region           - The region to which the bike lane belongs.
     *  bike_lane        - The bike lane to be split.
     *  resolution       - Resolution of the lane (see LaneCenterline).
     *  min_length       - The minimal length of a bike lane.
     *  split_bike_lanes - the split bike lane.
     */
    void Split(const RegionGroup& region, const Lane& bike_lane,
               double resolution, double min_length,
               Array<Lane>* split_bike_lanes) const {
        CHECK(bike_lane.category == "bike_lane");
        CHECK(resolution > 0.0);
        CHECK(min_length > resolution);
        CHECK(split_bike_lanes);

        split_bike_lanes->clear();
        if (bike_lane.polyline.size() < 2) return;

        Lane new_lane;
        carto_polyline_util::Subdivide(bike_lane, resolution, &new_lane);
        if (new_lane.polyline.size() < 2) {
            split_bike_lanes->push_back(bike_lane);
            return;
        }
        Array<int> types(new_lane.polyline.size(), 0);

        for (int i = 0; i < new_lane.polyline.size(); ++i) {
            int type = 0;
            double distance = DBL_MAX;
            for (auto& ring : region.rings) {
                for (auto& e : ring) {
                    double d = Distance(e.polyline, new_lane.polyline[i]);
                    if (e.type != LaneRegionGraph::RAISED_MEDIAN_BORDER &&
                        e.type != LaneRegionGraph::PAINTED_MEDIAN_BORDER) {
                        d *= 1.5;
                    }
                    if (d < distance) {
                        if (e.type == LaneRegionGraph::RAISED_MEDIAN_BORDER) {
                            type = 2;
                        } else if (e.type ==
                                   LaneRegionGraph::PAINTED_MEDIAN_BORDER) {
                            type = 1;
                        } else {
                            type = 0;
                        }
                        distance = d;
                    }
                }
            }
            types[i] = type;
        }

        // Step 2: Group consecutive points with the same type and calculate
        // segment lengths.
        struct Segment {
            int type;
            int start_idx; // Start index in polyline
            int end_idx;   // End index in polyline
            double length; // Total length of the segment
        };
        Array<Segment> segments;

        Segment current_seg;
        current_seg.type = types[0];
        current_seg.start_idx = 0;
        current_seg.length = 0.0;
        for (int i = 1; i < new_lane.polyline.size(); ++i) {
            // Calculate distance between current and previous point
            double seg_len = Distance(new_lane.polyline[i - 1],
                                      new_lane.polyline[i]);
            if (types[i] == current_seg.type) {
                current_seg.length += seg_len;
                current_seg.end_idx = i;
            } else {
                current_seg.end_idx = i - 1;
                segments.push_back(current_seg);

                current_seg.type = types[i];
                current_seg.start_idx = i;
                current_seg.length = 0.0;
            }
        }
        // Add the last segment
        current_seg.end_idx = new_lane.polyline.size() - 1;
        segments.push_back(current_seg);

        // Step 3: Merge short segments (< min_length) into adjacent segments.
        Array<Segment> merged_segments = segments;
        bool has_short_segments = true;
        while (has_short_segments) {
            has_short_segments = false;
            Array<Segment> temp_merged;

            for (int i = 0; i < merged_segments.size(); ++i) {
                Segment current_seg = merged_segments[i];

                if (current_seg.length >= min_length) {
                    temp_merged.push_back(current_seg);
                    continue;
                }

                has_short_segments = true;

                if (i == 0) {
                    if (i + 1 < merged_segments.size()) {
                        merged_segments[i + 1].start_idx =
                                current_seg.start_idx;
                        merged_segments[i + 1].length += current_seg.length;
                    }
                } else if (i == merged_segments.size() - 1) {
                    if (!temp_merged.empty()) {
                        temp_merged.back().end_idx = current_seg.end_idx;
                        temp_merged.back().length += current_seg.length;
                    }
                } else {
                   if (!temp_merged.empty()) {
                       temp_merged.back().end_idx = current_seg.end_idx;
                       temp_merged.back().length += current_seg.length;
                   } else {
                       merged_segments[i + 1].start_idx = current_seg.start_idx;
                       merged_segments[i + 1].length += current_seg.length;
                   }
               }
           }

           merged_segments = temp_merged;
           if (merged_segments.empty()) break;
        }

        // Step 4: Generate RoadBranch from merged segments and add to output.
        for (const Segment& seg : merged_segments) {
            if (seg.start_idx >= seg.end_idx) continue;
            Lane r;
            r.category = bike_lane.category;
            if (seg.type == 2) {
                r.category = "protected_bl";
            } else if (seg.type == 1) {
                r.category = "buffered_bl";
            } else {
                r.category = "traditional_bl";
            }

            for (int i = seg.start_idx; i <= seg.end_idx; ++i) {
                r.polyline.push_back(new_lane.polyline[i]);
                r.widths.push_back(new_lane.widths[i]);
            }
            split_bike_lanes->push_back(r);
        }
    }
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_BRANCH_SEGMENTER_H_

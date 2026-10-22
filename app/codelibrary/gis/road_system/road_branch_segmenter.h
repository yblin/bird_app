//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_SEGMENTER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_SEGMENTER_H_

#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/gis/road_system/road_branch.h"
#include "codelibrary/statistics/mean.h"

namespace cl {
namespace gis {

/**
 * Split road branch into segments based on width, ensuring each segment
 * maintains a consistent width.
 */
class RoadBranchSegmenter {
public:
    RoadBranchSegmenter() = default;

    RoadBranchSegmenter(const RoadBranchSegmenter&) = delete;

    RoadBranchSegmenter& operator=(const RoadBranchSegmenter&) = delete;

    /**
     * Split the given road branch into multiple BranchSegments, where each
     * segment is defined such that the width variation (difference between
     * maximum and minimum width) within the segment does not exceed the
     * specified width threshold.
     */
    void Split(const RoadBranch& branch, double width_threshold,
               Array<RoadBranchSegment>* branch_segments) const {
        CHECK(branch_segments);
        CHECK(width_threshold > 0.0);

        branch_segments->clear();
        if (branch.empty()) return;

        const RPolyline2D& polyline = branch.polyline;
        const Array<double>& widths = branch.widths;
        CHECK(polyline.size() == widths.size());

        RoadBranchSegment s;
        double min_w = widths[0];
        double max_w = widths[0];
        double total_w = 0.0;
        double total_l = 0.0;
        double sum_w = 0.0;

        s.polyline.push_back(polyline[0]);
        for (int i = 1; i < polyline.size(); ++i) {
            double l = Distance(polyline[i - 1], polyline[i]);

            min_w = std::min(widths[i], min_w);
            max_w = std::max(widths[i], max_w);
            if (max_w - min_w > width_threshold) {
                if (s.polyline.size() > 1) {
                    if (total_l == 0.0) {
                        s.width = sum_w / s.polyline.size();
                    } else {
                        s.width = total_w / total_l;
                    }
                    branch_segments->push_back(s);
                }

                if (s.polyline.size() == 1 ||
                    std::abs(widths[i - 1] - widths[i]) > width_threshold) {
                    s.polyline = RPolyline2D(polyline[i - 1], polyline[i]);
                    s.width = 0.5 * (widths[i - 1] + widths[i]);
                    branch_segments->push_back(s);

                    s.polyline.clear();
                    max_w = min_w = widths[i];
                    total_w = 0.0;
                    total_l = 0.0;
                } else {
                    s.polyline.clear();
                    s.polyline.push_back(polyline[i - 1]);
                    max_w = std::max(widths[i], widths[i - 1]);
                    min_w = std::min(widths[i], widths[i - 1]);
                    total_w = 0.0;
                    total_l = 0.0;
                    sum_w = widths[i - 1];
                }
            }

            s.polyline.push_back(polyline[i]);
            total_l += l;
            double mean_w = 0.5 * (widths[i - 1] + widths[i]);
            sum_w += widths[i];
            total_w += mean_w * l;
        }

        if (s.polyline.size() > 1) {
            if (total_l == 0.0) {
                s.width = total_w / s.polyline.size();
            } else {
                s.width = total_w / total_l;
            }
            branch_segments->push_back(s);
        }
    }
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_SEGMENTER_H_

//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_MERGER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_MERGER_H_

#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/gis/road_system/road_branch.h"
#include "codelibrary/gis/road_system/road_util.h"

namespace cl {
namespace gis {

/**
 * Merge road branch segments.
 */
class RoadBranchMerger {
public:
    RoadBranchMerger() = default;

    RoadBranchMerger(const RoadBranchMerger&) = delete;

    RoadBranchMerger& operator=(const RoadBranchMerger&) = delete;

    /**
     * Merge adjacent BranchSegments under the following conditions:
     * 1) Their width difference is within the specified width threshold;
     * 2) The previous segment's length is shorter than the specified length
     *    threshold.
     * 3) Two categories are identical or one of the category is empty.
     */
    void Merge(double width_threshold, double length_threshold,
               Array<RoadBranchSegment>* branch_segments) const {
        CHECK(branch_segments);

        MergeSemgents(width_threshold, length_threshold, branch_segments);
        MergeShortSemgents(length_threshold, branch_segments);
    }

private:
    /**
     * Merge a branch-segment into another one.
     */
    static void MergeSegments(const RoadBranchSegment& s,
                              RoadBranchSegment* s1) {
        double l1 = s.polyline.Length(), l2 = s1->polyline.Length();
        double total_length = l1 + l2;

        s1->width = (s1->width * l2 + s.width * l1) / total_length;
        if (!s.polyline.empty() &&
            s1->polyline.back() != s.polyline.front()) {
            s1->polyline.insert(s.polyline);
        } else if (s.polyline.size() > 1) {
            s1->polyline.insert(s1->polyline.end(), s.polyline.begin() + 1,
                                s.polyline.end());
        }
    }

    /**
     * Merge segments with the (nearly) same widths and categories.
     */
    void MergeSemgents(double width_threshold, double length_threshold,
                       Array<RoadBranchSegment>* branch_segments) const {
        CHECK(branch_segments);
        CHECK(width_threshold > 0.0);
        CHECK(length_threshold >= 0.0);

        if (branch_segments->empty()) return;

        Array<RoadBranchSegment> merged_segments;
        merged_segments.push_back((*branch_segments)[0]);

        for (int i = 1; i < branch_segments->size(); ++i) {
            const RoadBranchSegment& current = (*branch_segments)[i];
            RoadBranchSegment& last = merged_segments.back();
            double last_length = last.polyline.Length();
            double current_length = current.polyline.Length();

            if (!last.category.empty() && !current.category.empty() &&
                last.category != current.category) {
                merged_segments.push_back(current);
                continue;
            }

            if (std::abs(current.width - last.width) <= width_threshold ||
                last_length <= length_threshold ||
                current_length <= length_threshold) {
                MergeSegments(current, &last);
            } else {
                merged_segments.push_back(current);
            }
        }

        *branch_segments = merged_segments;
    }

    /**
     * Merge short road segments whose length is less than the specified
     * threshold.
     *
     * Unlike MergeSegments(), this function does not require the segments to
     * have the same category.
     */
    void MergeShortSemgents(double length_threshold,
                            Array<RoadBranchSegment>* branch_segments) const {
        if (branch_segments->empty()) return;

        Array<RoadBranchSegment> merged_segments;
        merged_segments.push_back((*branch_segments)[0]);

        for (int i = 1; i < branch_segments->size(); ++i) {
            const RoadBranchSegment& current = (*branch_segments)[i];
            RoadBranchSegment& last = merged_segments.back();
            double last_length = last.polyline.Length();
            double current_length = current.polyline.Length();

            if (last_length <= length_threshold ||
                current_length <= length_threshold) {
                std::string current_category = current.category;
                std::string last_category = last.category;

                // Special treat for crosswalks.
                bool donot_merge = false;
                if (road_util::IsCrosswalk(current_category)) {
                    if (last_length <= length_threshold) {
                        MergeSegments(current, &last);
                        last.category = current_category;
                        continue;
                    }
                    donot_merge = true;
                }
                if (road_util::IsCrosswalk(last_category)) {
                    if (current_length <= length_threshold) {
                        MergeSegments(current, &last);
                        last.category = last_category;
                        continue;
                    }
                    donot_merge = true;
                }
                if (!donot_merge) {
                    MergeSegments(current, &last);
                    last.category = last_length < current_length
                            ? current_category : last_category;
                    continue;
                }
            }
            merged_segments.push_back(current);
        }

        *branch_segments = merged_segments;
    }
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_MERGER_H_

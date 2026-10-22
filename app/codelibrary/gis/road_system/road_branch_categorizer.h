//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_

#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/land_cover.h"
#include "codelibrary/gis/road_system/road_branch.h"

namespace cl {
namespace gis {

/**
 * Assigning categories to road branch polylines based on associated landcover
 * data.
 */
class RoadBranchCategorizer {
public:
    RoadBranchCategorizer() = default;

    explicit RoadBranchCategorizer(const LandCover& land_cover) {
        ResetLandCover(land_cover);
    }

    RoadBranchCategorizer(const RoadBranchCategorizer&) = delete;

    RoadBranchCategorizer& operator=(const RoadBranchCategorizer&) = delete;

    /**
     * Reset land cover for category assigning.
     */
    void ResetLandCover(const LandCover& land_cover) {
        this->clear();

        land_cover.ToPolygonCategoryPairs(&polygons_, &categories_);

        Array<RBox2D> boxes;
        for (const auto& p : polygons_) {
            boxes.push_back(p.bounding_box());
        }
        rtree_.Construct(boxes);
    }

    /**
     * Check whether the given road branch intersects with the polygon
     * associated with the specified category.
     */
    bool HasCategory(const RPolyline2D& branch,
                     const std::string& category) const {
        RBox2D box(branch.begin(), branch.end());
        Array<int> entries;
        rtree_.Find(box, &entries);
        for (int i : entries) {
            if (categories_[i] != category) continue;

            if (geometry::Intersect(branch, polygons_[i])) return true;
        }
        return false;
    }

    /**
     * Split a road branch (composed of multiple branch segments) into multiple
     * road branch segments, where each segment is assigned a category
     * corresponding to the landcover polygon that covers that segment.
     */
    void AssignCategory(
            const Array<RoadBranchSegment>& segments,
            Array<RoadBranchSegment>* segments_with_category) const {
        CHECK(segments_with_category);

        if (segments.empty() || polygons_.empty()) {
            segments_with_category->clear();
            return;
        }

        RBox2D box;
        for (const auto& s : segments) {
            RBox2D box1(s.polyline.begin(), s.polyline.end());
            box.Union(box1);
        }
        *segments_with_category = segments;

        Array<int> entries;
        rtree_.Find(box, &entries);

        for (int i : entries) {
            Array<RoadBranchSegment> segs, new_segments;
            for (const auto& s : *segments_with_category) {
                AssignCategory(s, i, &segs);
                new_segments.insert(segs);
            }
            *segments_with_category = new_segments;
        }
    }

    void clear() {
        categories_.clear();
        polygons_.clear();
        rtree_.clear();
    }

private:
    /**
     * Split a single road branch segment into multiple segments based on the
     * i-th landcover polygon, and assign the polygon's corresponding category
     * to the overlapping parts.
     */
    void AssignCategory(const RoadBranchSegment& segment, int i,
                     Array<RoadBranchSegment>* segments) const {
        CHECK(i >= 0 && i < polygons_.size());
        CHECK(segments);

        segments->clear();

        if (segment.polyline.empty()) return;

        const auto& polygon = polygons_[i];
        const std::string& category = categories_[i];
        if (segment.category == category) {
            segments->push_back(segment);
            return;
        }

        using Segment = typename RPolyline2D::Segment;
        const int n = segment.polyline.size();

        RPolyline2D poly1, poly2;
        RoadBranchSegment branch_segment;
        for (int i = 0; i + 1 < n; ++i) {
            if (segment.polyline[i] == segment.polyline[i + 1]) continue;

            const Segment& seg = segment.polyline.segment(i);
            Array<RPoint2D> intersection;
            geometry::Intersect(seg, polygon, &intersection);
            CHECK(intersection.size() % 2 == 0);

            const RPoint2D& prev = segment.polyline[i];
            const RPoint2D& next = segment.polyline[i + 1];

            if (!intersection.empty()) {
                if (prev != intersection.front()) {
                    if (poly2.empty()) poly2.push_back(prev);
                    poly2.push_back(intersection.front());

                    if (!poly1.empty()) {
                        segments->emplace_back(poly1, segment.width, category);
                        poly1.clear();
                    }
                }

                for (int j = 0; j < intersection.size(); ++j) {
                    if (j % 2 == 0) {
                        if (!poly2.empty()) {
                            segments->emplace_back(poly2, segment.width,
                                                   segment.category);
                            poly2.clear();
                        }

                        if (intersection[j] != intersection[j + 1]) {
                            if (poly1.empty()) {
                                poly1.push_back(intersection[j]);
                            }
                            poly1.push_back(intersection[j + 1]);
                        }
                    } else {
                        if (j + 1 == intersection.size()) break;
                        if (!poly1.empty()) {
                            segments->emplace_back(poly1, segment.width,
                                                   category);
                            poly1.clear();
                        }

                        if (intersection[j] != intersection[j + 1]) {
                            if (poly2.empty()) {
                                poly2.push_back(intersection[j]);
                            }
                            poly2.push_back(intersection[j + 1]);
                        }
                    }
                }

                if (next != intersection.back()) {
                    if (poly2.empty()) poly2.push_back(intersection.back());
                    poly2.push_back(next);

                    if (!poly1.empty()) {
                        segments->emplace_back(poly1, segment.width, category);
                        poly1.clear();
                    }
                }
            } else {
                if (poly2.empty()) poly2.push_back(prev);
                poly2.push_back(next);
                if (!poly1.empty()) {
                    segments->emplace_back(poly1, segment.width, category);
                    poly1.clear();
                }
            }
        }

        CHECK(poly1.empty() || poly2.empty());
        if (!poly1.empty()) {
            segments->emplace_back(poly1, segment.width, category);
        }
        if (!poly2.empty()) {
            segments->emplace_back(poly2, segment.width, segment.category);
        }

        int n_segments = 0;
        for (RoadBranchSegment& segment : *segments) {
            segment.polyline.RemoveDuplicateVertices();
            if (segment.polyline.size() >= 2) {
                (*segments)[n_segments++] = segment;
            }
        }
        segments->resize(n_segments);

        // In case where the polyline degenerates into a point.
        if (segments->empty()) {
            if (geometry::Intersect(segment.polyline.front(), polygon)) {
                segments->push_back(segment);
                segments->back().category = category;
            }
        }
    }

    Array<std::string> categories_;
    Array<RMultiPolygon2D> polygons_;
    geometry::RTree2D<double> rtree_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_

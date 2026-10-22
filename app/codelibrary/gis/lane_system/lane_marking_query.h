//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_QUERY_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_QUERY_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/lane_system/lane_marking.h"

namespace cl {
namespace gis {

/**
 * Spatial range query for lane markings, supporting spatial range retrieval.
 */
class LaneMarkingQuery {
public:
    LaneMarkingQuery() = default;

    LaneMarkingQuery(const Array<LaneMarking>& lane_markings) {
        Reset(lane_markings);
    }

    /**
     * Reset the lane markings and rebuild spatial index.
     *
     * Note that, the spatial index is built based on the center points of each
     * road marking, not the full road marking geometry.
     */
    void Reset(const Array<LaneMarking>& lane_markings) {
        lane_markings_ = lane_markings;

        road_marking_centers_.clear();
        for (int i = 0; i < lane_markings_.size(); ++i) {
            Array<RPoint2D> points(lane_markings_[i].polyline.begin(),
                                   lane_markings_[i].polyline.end());
            road_marking_centers_.push_back(Center(points));
        }

        Array<RBox2D> boxes(lane_markings_.size());
        for (int i = 0; i < lane_markings_.size(); ++i) {
            boxes[i] = road_marking_centers_[i].bounding_box();
        }
        rtree_.Construct(boxes);
    }

    /**
     * Range query to find the intersected road markings.
     */
    void Query(const RMultiPolygon2D& range,
               Array<LaneMarking>* lane_markings) const {
        CHECK(lane_markings);

        lane_markings->clear();
        Array<int> entries;
        rtree_.Find(range.bounding_box(), &entries);
        for (int j : entries) {
            if (geometry::Intersect(road_marking_centers_[j], range)) {
                lane_markings->push_back(lane_markings_[j]);
            }
        }
    }

private:
    // Stored lane markings.
    Array<LaneMarking> lane_markings_;

    // Center point of each road marking.
    Array<RPoint2D> road_marking_centers_;

    // Spatial index.
    geometry::RTree2D<double> rtree_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_QUERY_H_

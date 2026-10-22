//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ORDER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ORDER_H_

#include <map>
#include <set>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace gis {

/**
 * Determine the order of lane groups that intersect with a given border
 * polyline, sorted by the positions of their intersection points along the
 * border's direction.
 *
 * For each lane group (composed of connected segments), this function
 * identifies all intersection points, sorts these points along the border's
 * path, and records the range of their positions as a [start, end] index
 * pair. These ranges provide both the order of lane groups and their
 * potential nesting relationships. e.g., lane group A with the order [1,9]
 * contains lane group B with the order [2,8].
 */
template <typename T>
void GetLaneGroupOrder(const Array<Point2D<T>>& border,
                       const Array<Array<RSegment2D>>& lane_segments,
                       Array<std::pair<int, int>>* orders) {
    CHECK(orders);

    orders->resize(lane_segments.size());
    for (int i = 0; i < lane_segments.size(); ++i) {
        (*orders)[i] = std::make_pair(INT_MAX, -INT_MAX);
    }
    if (border.empty()) return;

    Array<Array<std::pair<double, int>>> seqs(border.size() - 1);
    Array<bool> intersected(lane_segments.size(), false);

    if (border.back() == border.front()) return;

    int n_orders = 0;
    for (int i = 0; i + 1 < border.size(); ++i) {
        RSegment2D s(border[i], border[i + 1]);
        std::map<double, Array<int>> bucket_lanes;
        for (int j = 0; j < lane_segments.size(); ++j) {
            std::set<RPoint2D> all_inters;
            Array<RPoint2D> inters;
            for (auto s1 : lane_segments[j]) {
                geometry::Intersect(s1, s, &inters);
                for (auto p : inters) {
                    if (i != 0 && p == border.front()) {
                        // Treat the case when border is a ring.
                        continue;
                    }
                    all_inters.insert(p);
                }
            }
            if (all_inters.empty()) continue;

            for (auto p : all_inters) {
                bucket_lanes[Distance(p, border[i])].push_back(j);
            }
        }
        for (auto s1 : bucket_lanes) {
            for (int l : s1.second) {
                auto& order = (*orders)[l];

                order.first = std::min(order.first, n_orders);
                order.second = std::max(order.second, n_orders);
            }
            ++n_orders;
        }
    }
}

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ORDER_H_

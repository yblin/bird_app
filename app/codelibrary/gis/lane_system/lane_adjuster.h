//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_ADJUSTER_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_ADJUSTER_H_

#include <cfloat>
#include <unordered_map>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace gis {

/**
 * A class that handles lane extension operations to connect the lane with
 * borders, supporting separate extension logic for the lane's front and back
 * endpoints.
 */
class LaneExtender {
public:
    explicit LaneExtender(double max_extend_distance)
        : max_extend_distance_(max_extend_distance) {
        CHECK(max_extend_distance > 0.0);
    }

    /**
     * Extend the front endpoint of the lane until it touches a border.
     */
    bool ExtendFront(const RPolyline2D& lane, const Array<RPolyline2D>& borders,
                     RPolyline2D* extended_lane) const {
        CHECK(extended_lane);

        *extended_lane = lane;
        if (lane.size() <= 1) return false;

        RVector2D v = (lane[0] - lane[1]).normalized();
        RSegment2D s(lane.front(), lane.front() + v * max_extend_distance_);

        double dis = DBL_MAX;
        RPoint2D nearest;
        Array<RPoint2D> candidates;
        for (const auto& border : borders) {
            for (auto p : border) {
                if (p != lane.front() && geometry::Intersect(p, s)) {
                    candidates.push_back(p);
                }
            }
            for (int i = 0; i + 1 < border.size(); ++i) {
                RSegment2D s1(border[i], border[i + 1]);
                RPoint2D p;
                if (geometry::Cross(s1, s, &p) && p != lane.front()) {
                    candidates.push_back(p);
                }
            }
        }
        for (auto p : candidates) {
            double d = Distance(p, lane.front());
            if (d < dis) {
                dis = d;
                nearest = p;
            }
        }

        if (dis == DBL_MAX) return false;

        extended_lane->insert(extended_lane->begin(), nearest);
        return true;
    }

    /**
     * Extend the back endpoint of the lane until it touches a border.
     */
    bool ExtendBack(const RPolyline2D& lane, const Array<RPolyline2D>& borders,
                    RPolyline2D* extended_lane) const {
        CHECK(extended_lane);

        *extended_lane = lane;
        if (lane.size() <= 1) return false;

        RVector2D v = (lane.back() - lane[lane.size() - 2]).normalized();
        RSegment2D s(lane.back(), lane.back() + v * max_extend_distance_);

        Array<RPoint2D> candidates;
        for (const auto& border : borders) {
            for (auto p : border) {
                if (p != lane.back() && geometry::Intersect(p, s)) {
                    candidates.push_back(p);
                }
            }
            for (int i = 0; i + 1 < border.size(); ++i) {
                RSegment2D s1(border[i], border[i + 1]);
                RPoint2D p;
                if (geometry::Cross(s1, s, &p) && p != lane.back()) {
                    candidates.push_back(p);
                }
            }
        }

        double dis = DBL_MAX;
        RPoint2D nearest;
        for (auto p : candidates) {
            double d = Distance(p, lane.back());
            if (d < dis) {
                dis = d;
                nearest = p;
            }
        }

        if (dis == DBL_MAX) return false;

        extended_lane->push_back(nearest);
        return true;
    }

private:
    double max_extend_distance_;
};

/**
 * A class that handles lane shrinking operations to connect the lane with
 * borders, supporting separate shrinking logic for the lane's front and back
 * endpoints.
 */
class LaneShrinker {
public:
    explicit LaneShrinker(double max_shrink_distance)
        : max_shrink_distance_(max_shrink_distance) {
        CHECK(max_shrink_distance > 0.0);
    }

    /**
     * Shrink the front endpoint of the lane until it touches a border.
     *
     * The input lane is assumed to be contained in the borders.
     */
    bool ShrinkFront(const RPolyline2D& lane, const Array<RPolyline2D>& borders,
                     RPolyline2D* shrinked_lane) const {
        CHECK(shrinked_lane);

        *shrinked_lane = lane;
        if (lane.size() <= 1) return false;

        std::unordered_map<RPoint2D, int> degrees;
        for (const auto& border : borders) {
            for (auto p : border) {
                ++degrees[p];
            }
        }

        Array<RPoint2D> front_poly, tmp_poly;
        geometry::polyline_util::Split(lane, max_shrink_distance_,
                                       &front_poly, &tmp_poly);
        int best = -1;
        double best_dis = DBL_MAX;
        RPoint2D best_p;
        for (int i = 0; i + 1 < front_poly.size(); ++i) {
            if (degrees[front_poly[i]] >= 2) {
                double dis = Distance(front_poly[i], lane.front());
                if (dis < best_dis) {
                    best = i;
                    best_dis = dis;
                    best_p = front_poly[i];
                    break;
                }
            }

            RSegment2D s(front_poly[i], front_poly[i + 1]);
            for (auto b : borders) {
                for (int j = 0; j + 1 < b.size(); ++j) {
                    RSegment2D s1(b[j], b[j + 1]);
                    RPoint2D p;
                    if (geometry::Cross(s, s1, &p)) {
                        double dis = Distance(p, lane.front());
                        if (dis < best_dis) {
                            best = i;
                            best_dis = dis;
                            best_p = p;
                        }
                    }
                }
            }
            if (best != -1) break;
        }

        if (best_dis == DBL_MAX || best_p == lane.front()) return false;

        shrinked_lane->erase(shrinked_lane->begin(),
                             shrinked_lane->begin() + best);
        CHECK(!shrinked_lane->empty());
        shrinked_lane->front() = best_p;
        return true;
    }

    /**
     * Shrink the back endpoint of the lane until it touches a border.
     *
     * The input lane is assumed to be contained in the borders.
     */
    bool ShrinkBack(const RPolyline2D& lane, const Array<RPolyline2D>& borders,
                    RPolyline2D* shrinked_lane) const {
        CHECK(shrinked_lane);

        *shrinked_lane = lane;
        if (lane.size() <= 1) return false;

        std::unordered_map<RPoint2D, int> degrees;
        for (const auto& border : borders) {
            for (auto p : border) {
                ++degrees[p];
            }
        }

        Array<RPoint2D> back_poly, tmp_poly;
        double length = lane.Length() - max_shrink_distance_;
        length = std::max(0.0, length);
        geometry::polyline_util::Split(lane, length, &tmp_poly, &back_poly);

        double best_dis = DBL_MAX;
        int best = -1;
        RPoint2D best_p;
        for (int i = 0; i + 1 < back_poly.size(); ++i) {
            if (degrees[back_poly[i]] >= 2) {
                double dis = Distance(back_poly[i], lane.back());
                if (dis < best_dis) {
                    best = i;
                    best_p = back_poly[i];
                    best_dis = dis;
                    break;
                }
            }

            RSegment2D s(back_poly[i], back_poly[i + 1]);
            for (auto b : borders) {
                for (int j = 0; j + 1 < b.size(); ++j) {
                    RSegment2D s1(b[j], b[j + 1]);
                    RPoint2D p;
                    if (geometry::Cross(s, s1, &p)) {
                        double dis = Distance(p, lane.back());
                        if (dis < best_dis) {
                            best = i;
                            best_dis = dis;
                            best_p = p;
                        }
                    }
                }
            }
            if (best != -1) break;
        }

        if (best_dis == DBL_MAX || best_p == lane.back()) return false;

        shrinked_lane->resize(shrinked_lane->size() - best);
        shrinked_lane->back() = best_p;
        return true;
    }

private:
    double max_shrink_distance_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_ADJUSTER_H_

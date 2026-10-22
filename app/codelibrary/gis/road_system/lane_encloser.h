//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ENCLOSER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ENCLOSER_H_

#include "codelibrary/base/index_sort.h"
#include "codelibrary/base/progress_bar.h"
#include "codelibrary/geometry/transform_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/geometry/util/snap_2d.h"
#include "codelibrary/geometry/topology/geo_graph.h"
#include "codelibrary/geometry/topology/polyline_regularization_2d.h"
#include "codelibrary/gis/land_cover.h"
#include "codelibrary/gis/road_system/lane_adjuster.h"
#include "codelibrary/gis/road_system/lane_order.h"
#include "codelibrary/gis/road_system/lane_util.h"
#include "codelibrary/gis/road_system/road_util.h"
#include "codelibrary/util/set/disjoint_set.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace gis {

/**
 * Helper class for LaneEncloser to connect the lane segment array.
 */
class LaneSegmentConnector {
    struct LaneSegment {
        Array<RSegment2D> segs;
        std::unordered_map<RPoint2D, int> connection;
    };

public:
    explicit LaneSegmentConnector(double max_connection_distance,
                                  double angle_threshold = 20.0)
        : max_connection_distance_(max_connection_distance),
          angle_threshold_(angle_threshold) {
        CHECK(max_connection_distance_ > 0.0);
        CHECK(angle_threshold_ > 0.0 && angle_threshold < 90.0);
    }

    /**
     * Connect the given lane segments in order.
     */
    void Connect(const Array<Array<RSegment2D>>& lanes,
                 const Array<RPoint2D>& connection_points,
                 Array<RSegment2D>* connection_lines) {
        CHECK(lanes.size() >= 2);
        CHECK(connection_lines);

        connection_lines->clear();
        if (connection_points.empty()) return;

        std::unordered_set<RPoint2D> is_connection;
        for (auto p : connection_points) {
            is_connection.insert(p);
        }

        Array<LaneSegment> lane_segments(lanes.size());
        for (int i = 0; i < lanes.size(); ++i) {
            lane_segments[i].segs = lanes[i];
            auto& connection = lane_segments[i].connection;
            for (int j = 0; j < lane_segments[i].segs.size(); ++j) {
                RSegment2D s = lane_segments[i].segs[j];
                if (is_connection.find(s.lower_point()) !=
                    is_connection.end()) {
                    connection[s.lower_point()] = j;
                }
                if (is_connection.find(s.upper_point()) !=
                    is_connection.end()) {
                    connection[s.upper_point()] = j;
                }
            }
        }

        for (int i = 1; i + 1 < lanes.size(); ++i) {
            LaneSegment& lane = lane_segments[i];
            if (lane.connection.size() >= 2) {
                // Minimum spanning tree to connect dangling points inside
                // lane-segment.
                DisjointSet set(lane.connection.size());
                std::unordered_map<RPoint2D, int> indices;
                int n_dangling = 0;
                for (auto p : lane.connection) {
                    indices[p.first] = n_dangling++;
                }

                Array<std::pair<double, RSegment2D>> edges;
                for (auto p : lane.connection) {
                    RSegment2D s1 = lane.segs[p.second];
                    for (auto q : lane.connection) {
                        RSegment2D s2 = lane.segs[q.second];
                        RSegment2D e(p.first, q.first);
                        if (p.first != q.first && ValidConnection(s1, s2, e)) {
                            edges.emplace_back(e.length(), e);
                        }
                    }
                }

                std::sort(edges.begin(), edges.end());
                for (auto e : edges) {
                    int a = indices[e.second.lower_point()];
                    int b = indices[e.second.upper_point()];
                    if (set.Find(a) != set.Find(b)) {
                        set.Union(a, b);
                        connection_lines->push_back(e.second);
                    }
                }
            }
        }

        Array<bool> right_conneceted(lanes.size(), false);
        for (int i = 1; i + 1 < lanes.size(); ++i) {
            if (lane_segments[i].connection.empty()) continue;

            for (int j = i - 1; j >= 0; --j) {
                RSegment2D c;
                int s = GetConnection(lane_segments[i], lane_segments[j], &c);
                if (s != 0) {
                    if (s == 1) right_conneceted[j] = true;
                    connection_lines->push_back(c);
                    break;
                }
            }
        }
        for (int i = 1; i + 1 < lanes.size(); ++i) {
            if (lane_segments[i].connection.empty()) continue;
            if (right_conneceted[i]) continue;

            for (int j = i + 1; j < lanes.size(); ++j) {
                RSegment2D c;
                if (GetConnection(lane_segments[i], lane_segments[j], &c)) {
                    connection_lines->push_back(c);
                    break;
                }
            }
        }
    }

private:
    /**
     * Check if the given connection is valid.
     */
    bool ValidConnection(const RSegment2D& s1, const RSegment2D& s2,
                         const RSegment2D& connection) const {
        bool s1_lower = connection.lower_point() == s1.lower_point() ||
                        connection.upper_point() == s1.lower_point();
        bool s2_lower = connection.lower_point() == s2.lower_point() ||
                        connection.upper_point() == s2.lower_point();

        RPoint2D p1 = s1_lower ? s1.lower_point() : s1.upper_point();
        RPoint2D p2 = s1_lower ? s1.upper_point() : s1.lower_point();
        RPoint2D p3 = s2_lower ? s2.lower_point() : s2.upper_point();
        RPoint2D p4 = s2_lower ? s2.upper_point() : s2.lower_point();

        double dis = Distance(p1, p3);
        if (dis > max_connection_distance_) return false;

        double d1 = Degree(p2 - p1, p3 - p1);
        if (d1 < angle_threshold_) return false;

        double d2 = Degree(p4 - p3, p1 - p3);
        if (d2 < angle_threshold_) return false;

        return true;
    }

    /**
     * Get the connection from segs1's dangling points to segs2.
     */
    int GetConnection(const LaneSegment& segs1, const LaneSegment& segs2,
                      RSegment2D* connection) const {
        CHECK(connection);

        double min_distance = DBL_MAX;
        for (auto p : segs1.connection) {
            RSegment2D s1 = segs1.segs[p.second];
            for (auto q : segs2.connection) {
                RSegment2D s2 = segs2.segs[q.second];

                RSegment2D c(p.first, q.first);
                if (ValidConnection(s1, s2, c) && c.length() < min_distance) {
                    min_distance = c.length();
                    *connection = c;
                }
            }
        }

        if (min_distance < DBL_MAX) return 1;

        for (auto p : segs1.connection) {
            RSegment2D s1 = segs1.segs[p.second];

            double d = DBL_MAX;
            RSegment2D best_s;
            for (auto s2 : segs2.segs) {
                double dis = Distance(p.first, s2);
                if (dis > max_connection_distance_) continue;
                if (dis < d) {
                    d = dis;
                    RPoint2D q = geometry::Project(p.first, s2);
                    double degree = s1.lower_point() == p.first
                            ? Degree(q - p.first, s1.upper_point() - p.first)
                            : Degree(q - p.first, s1.lower_point() - p.first);
                    if (degree < angle_threshold_) continue;
                    best_s = RSegment2D(p.first, q);
                }
            }
            if (d < min_distance) {
                min_distance = d;
                *connection = best_s;
            }
        }

        return min_distance < DBL_MAX ? 2 : 0;
    }

    double max_connection_distance_ = 10.0;
    double angle_threshold_ = 20.0;
};

/**
 * This class resolves discontinuities in lane polylines to create enclosed lane
 * regions that are spatially consistent with the parent road surface.
 */
class LaneEncloser {
public:
    LaneEncloser() = default;

    LaneEncloser(const LandCover& road_surface,
                 const Array<RPolyline2D>& lanes,
                 const Array<std::string>& lane_titles,
                 double snap_threshold = 1e-2)
        : snap_threshold_(snap_threshold),
          lanes_(lanes),
          lane_titles_(lane_titles) {
        CHECK(lanes_.size() == lane_titles_.size());
        CHECK(snap_threshold_ > 0.0);

        ResetRoadBorders(road_surface);
    }

    /**
     * Reset road borders from road polygons.
     */
    void ResetRoadBorders(const LandCover& road_surface) {
        road_borders_.clear();
        road_segments_.clear();
        road_intersections_.clear();

        for (const auto& p : road_surface) {
            if (road_util::IsRoadSegment(p.first)) {
                road_segments_.insert(p.second);
            }

            bool is_road_intersection = IsRoadIntersection(p.first);

            for (const auto& poly : p.second) {
                if (poly.empty()) continue;

                if (is_road_intersection) {
                    road_intersection_out_borders_.push_back(
                                road_borders_.size());
                    road_intersections_.push_back(poly);
                }

                for (const auto& b : poly.boundaries()) {
                    RPolyline2D polyline(b.polygon.vertices());
                    if (!polyline.empty()) {
                        polyline.push_back(polyline.front());
                    }
                    road_border_titles_.push_back(p.first);
                    road_borders_.push_back(polyline);
                }
            }
        }
    }

    /**
     * Snap the road borders and lanes.
     */
    void SnapRoadFeatures() {
        if (lanes_.empty()) return;

        Array<RPolyline2D> polylines = road_borders_;
        polylines.insert(lanes_);

        geometry::SnapPolylines(snap_threshold_, &polylines);
        road_borders_.assign(polylines.begin(),
                             polylines.begin() + road_borders_.size());
        lanes_.assign(polylines.begin() + road_borders_.size(),
                      polylines.end());
    }

    /**
     * Filter out assist lines that are not closed.
     */
    void FilterUnclosedAssistLines() {
        Array<bool> is_filtered(lanes_.size(), false);
        std::unordered_map<RPoint2D, int> degrees;
        std::unordered_map<RPoint2D, Array<int>> graph;
        for (auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (int i = 0; i < lanes_.size(); ++i) {
            auto& lane = lanes_[i];
            if (lane.size() <= 1) {
                is_filtered[i] = true;
            } else {
                for (auto p : lane)
                    ++degrees[p];

                graph[lane.front()].push_back(i);
                graph[lane.back()].push_back(i);
            }
        }

        std::queue<int> q;
        for (int i = 0; i < lanes_.size(); ++i) {
            if (lanes_[i].size() <= 1) continue;
            if (lane_titles_[i] == "assist_line" &&
                (degrees[lanes_[i].front()] == 1 ||
                 degrees[lanes_[i].back()] == 1)) {
                q.push(i);
            }
        }

        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            is_filtered[cur] = true;
            --degrees[lanes_[cur].front()];
            --degrees[lanes_[cur].back()];

            if (degrees[lanes_[cur].front()] == 1) {
                for (int v : graph[lanes_[cur].front()]) {
                    if (lane_titles_[v] == "assist_line") {
                        q.push(v);
                    }
                }
            }
            if (degrees[lanes_[cur].back()] == 1) {
                for (int v : graph[lanes_[cur].back()]) {
                    if (lane_titles_[v] == "assist_line") {
                        q.push(v);
                    }
                }
            }
        }

        int n_lanes = 0;
        for (int i = 0; i < lanes_.size(); ++i) {
            if (!is_filtered[i]) {
                lane_titles_[n_lanes] = lane_titles_[i];
                lanes_[n_lanes++] = lanes_[i];
            }
        }
        lane_titles_.resize(n_lanes);
        lanes_.resize(n_lanes);
    }

    /**
     * Connect dangling endpoints of lanes by extending and shrinking them.
     */
    void ExtendShrinkLanes(double extend_distance, double shrink_distance) {
        CHECK(extend_distance > 0.0);
        CHECK(shrink_distance > 0.0);

        if (lanes_.empty()) return;

        Array<RPolyline2D> polylines = lanes_;
        polylines.insert(road_borders_);
        geometry::ClosePolylines(extend_distance, shrink_distance, &polylines);
        lanes_ = Array<RPolyline2D>(polylines.begin(),
                                    polylines.begin() + lanes_.size());
    }

    /**
     * Adjust the front endpoint of the given lane by either extending or
     * shrinking it to connect with the specified borders.
     *
     * If the required extension length exceeds the provided shrink distance t
     * hreshold, an assist line will be added instead of over-extending the
     * lane.
     */
    bool ExtendShrinkLaneFront(const RPolyline2D& lane,
                               const Array<RPolyline2D>& borders,
                               double extend_distance,
                               double shrink_distance,
                               RPolyline2D* adjusted_lane) {
        CHECK(extend_distance > 0.0);
        CHECK(shrink_distance > 0.0);
        CHECK(adjusted_lane);

        if (lane.size() <= 1) return false;

        LaneShrinker shrinker(shrink_distance);
        LaneExtender extender(extend_distance);

        if (shrinker.ShrinkFront(lane, borders, adjusted_lane)) {
            return true;
        }

        RPoint2D p = lane.front();
        if (extender.ExtendFront(lane, borders, adjusted_lane)) {
            if (Distance(p, adjusted_lane->front()) > shrink_distance) {
                lanes_.emplace_back(p, adjusted_lane->front());
                lane_titles_.push_back("assist_line");
            }
            return true;
        }

        return false;
    }

    /**
     * Adjust the back endpoint of the given lane by either extending or
     * shrinking it to connect with the specified borders.
     *
     * If the required extension length exceeds the provided shrink distance t
     * hreshold, an assist line will be added instead of over-extending the
     * lane.
     */
    bool ExtendShrinkLaneBack(const RPolyline2D& lane,
                              const Array<RPolyline2D>& borders,
                              double extend_distance,
                              double shrink_distance,
                              RPolyline2D* adjusted_lane) {
        CHECK(extend_distance > 0.0);
        CHECK(shrink_distance > 0.0);
        CHECK(adjusted_lane);

        if (lane.size() <= 1) return false;

        LaneShrinker shrinker(shrink_distance);
        LaneExtender extender(extend_distance);

        if (shrinker.ShrinkBack(lane, borders, adjusted_lane)) {
            return true;
        }

        RPoint2D p = lane.back();
        if (extender.ExtendBack(lane, borders, adjusted_lane)) {
            if (Distance(p, adjusted_lane->back()) > shrink_distance) {
                lanes_.emplace_back(p, adjusted_lane->back());
                lane_titles_.push_back("assist_line");
            }
            return true;
        }

        return false;
    }

    /**
     * This function identifies yellow and white lanes with dangling endpoints
     * and extends them by a specified length (defined by extend_distance)
     * along their natural direction. The extended assist lines are generated to
     * bridge the gap between the lanes' dangling endpoints and the target road
     * intersections, aiming to establish a continuous connection between lanes
     * and road intersections.
     */
    void ExtendLanesToRoadIntersection(double extend_distance) {
        CHECK(extend_distance > 0.0);

        std::unordered_map<RPoint2D, int> degrees;
        for (auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (auto& lane : lanes_) {
            for (auto p : lane)
                ++degrees[p];
        }

        Array<RPolyline2D> polylines = lanes_;
        std::map<RSegment2D, bool> is_intersection;
        for (int i = 0; i < road_borders_.size(); ++i) {
            if (IsRoadIntersection(road_border_titles_[i])) {
                for (int j = 0; j + 1 < road_borders_[i].size(); ++j) {
                    RSegment2D seg(road_borders_[i][j],
                                   road_borders_[i][j + 1]);
                    is_intersection[seg] = true;
                }
            }
        }
        for (int i = 0; i < road_borders_.size(); ++i) {
            if (!IsRoadIntersection(road_border_titles_[i])) {
                for (int j = 0; j + 1 < road_borders_[i].size(); ++j) {
                    RSegment2D seg(road_borders_[i][j],
                                   road_borders_[i][j + 1]);
                    if (!is_intersection[seg]) {
                        polylines.emplace_back(seg.lower_point(),
                                               seg.upper_point());
                    }
                }
            }
        }

        Array<RBox2D> boxes(polylines.size());
        for (int i = 0; i < polylines.size(); ++i) {
            boxes[i] = RBox2D(polylines[i].begin(), polylines[i].end());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        Array<RPolyline2D> extend_lanes;
        for (int i = 0; i < lanes_.size(); ++i) {
            if (lanes_[i].size() <= 1) continue;
            if (IsProcessLane(lane_titles_[i])) {
                if (degrees[lanes_[i].front()] == 1 ||
                    degrees[lanes_[i].back()] == 1) {
                    extend_lanes.push_back(lanes_[i]);
                }
            }
        }

        Array<RBox2D> extend_lane_boxes(extend_lanes.size());
        for (int i = 0; i < extend_lanes.size(); ++i) {
            extend_lane_boxes[i] = RBox2D(extend_lanes[i].begin(),
                                          extend_lanes[i].end());
        }
        geometry::RTree2D<double> lane_rtree;
        lane_rtree.Construct(extend_lane_boxes);

        Array<int> entries;
        for (int i = 0; i < road_borders_.size(); ++i) {
            if (!road_util::IsRoadSegment(road_border_titles_[i])) continue;

            Array<RSegment2D> borders;
            for (int j = 0; j + 1 < road_borders_[i].size(); ++j) {
                RSegment2D seg(road_borders_[i][j],
                               road_borders_[i][j + 1]);
                if (is_intersection[seg]) {
                    borders.push_back(seg);
                }
            }

            RPolygon2D poly(road_borders_[i]);
            lane_rtree.Find(poly.bounding_box(), &entries);
            for (int j : entries) {
                auto& lane = extend_lanes[j];
                if (degrees[lane.front()] == 1 &&
                    geometry::Cross(poly, lane.front())) {
                    ++degrees[lane.front()];

                    RVector2D v = (lane[0] - lane[1]).normalized() *
                                  extend_distance;
                    RPoint2D p;
                    if (FindNearestIntersection(lane.front(), lane.front() + v,
                                                borders, &p)) {
                        RSegment2D s(lane.front(), p);
                        if (!CheckCross(s, polylines, rtree)) {
                            lanes_.emplace_back(lane.front(), p);
                            lane_titles_.push_back("assist_line");
                        }
                    }
                }

                if (degrees[lane.back()] == 1 &&
                    geometry::Cross(poly, lane.back())) {
                    ++degrees[lane.back()];

                    RVector2D v = lane.back() - lane[lane.size() - 2];
                    v = v.normalized() * extend_distance;
                    RPoint2D p;
                    if (FindNearestIntersection(lane.back(), lane.back() + v,
                                                borders, &p)) {
                        RSegment2D s(lane.back(), p);
                        if (!CheckCross(s, polylines, rtree)) {
                            lanes_.emplace_back(lane.back(), p);
                            lane_titles_.push_back("assist_line");
                        }
                    }
                }
            }
        }
    }

    /**
     * This function processes each yellow/white lane in the road segment
     * individually:
     * 1. For each lane, identifies all its dangling endpoints (unconnected
     *    endpoints not linked to other road features).
     * 2. Generates a perpendicular cut line at each dangling endpoint, oriented
     *    relative to the lane's polyline direction at that point.
     * 3. Adjusts other relevant lane lines (by extending or shrinking them) to
     *    terminate exactly at the corresponding cut lines, ensuring consistent
     *    alignment across all lanes in the road segment.
     */
    void AddCutlinesForRoadSegments(double max_cutline_length,
                                    double extend_distance,
                                    double shrink_distance) {
        CHECK(max_cutline_length > 0.0);
        CHECK(extend_distance > 0.0);
        CHECK(shrink_distance > 0.0);

        Array<RPolyline2D> polylines = lanes_;
        polylines.insert(road_borders_);

        Array<RBox2D> boxes(polylines.size());
        for (int i = 0; i < polylines.size(); ++i) {
            boxes[i] = RBox2D(polylines[i].begin(), polylines[i].end());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        LaneExtender cutline_extender(max_cutline_length);
        const double threshold = snap_threshold_;

        Array<int> entries;
        for (auto& road : ProgressBar(road_segments_)) {
            RBox2D box(road.bounding_box().x_min() - snap_threshold_,
                       road.bounding_box().x_max() + snap_threshold_,
                       road.bounding_box().y_min() - snap_threshold_,
                       road.bounding_box().y_max() + snap_threshold_);
            rtree.Find(box, &entries);

            Array<RPolyline2D> lanes;
            Array<int> lane_indices;
            Array<RPolyline2D> borders;
            int n_lanes = lanes_.size();
            for (int i : entries) {
                if (i < n_lanes) {
                    if (lanes_[i].size() > 1) {
                        lane_indices.push_back(i);
                        lanes.push_back(lanes_[i]);
                    }
                } else {
                    if (polylines[i].size() > 1)
                        borders.push_back(polylines[i]);
                }
            }
            borders.insert(borders.begin(), lanes.begin(), lanes.end());

            std::unordered_map<RPoint2D, int> degrees;
            for (auto& line : borders) {
                for (auto p : line)
                    ++degrees[p];
            }
            int n_dangling = 0;
            for (int i = 0; i < lanes.size(); ++i) {
                const std::string title = lane_titles_[lane_indices[i]];
                if (!IsProcessLane(title)) continue;

                if (degrees[lanes[i].front()] == 1) {
                    if (!geometry::Cross(road, lanes[i].front())) {
                        degrees[lanes[i].front()] = 0;
                    } else {
                        ++n_dangling;
                    }
                }
                if (degrees[lanes[i].back()] == 1) {
                    if (!geometry::Cross(road, lanes[i].back())) {
                        degrees[lanes[i].back()] = 0;
                    } else {
                        ++n_dangling;
                    }
                }
            }

            while (n_dangling--) {
                // Extend and shrink yellow/white lanes.
                for (int j = 0; j < lanes.size(); ++j) {
                    const std::string title = lane_titles_[lane_indices[j]];
                    if (!IsProcessLane(title)) continue;
                    RPolyline2D& lane = lanes_[lane_indices[j]];

                    if (degrees[lanes[j].front()] == 1) {
                        if (ExtendShrinkLaneFront(borders[j], borders,
                                                  extend_distance,
                                                  shrink_distance,
                                                  &borders[j])) {
                            degrees[lanes[j].front()] = 0;
                            lane = borders[j];
                        }
                    }
                    if (degrees[lanes[j].back()] == 1) {
                        if (ExtendShrinkLaneBack(borders[j], borders,
                                                 extend_distance,
                                                 shrink_distance,
                                                 &borders[j])) {
                            degrees[lanes[j].back()] = 0;
                            lane = borders[j];
                        }
                    }
                }

                Array<RPolyline2D> candidates;
                for (int j = 0; j < lanes.size(); ++j) {
                    Array<RPolyline2D> cutlines;
                    const std::string title = lane_titles_[lane_indices[j]];
                    if (!IsProcessLane(title)) continue;

                    const RPolyline2D& lane = lanes[j];
                    if (degrees[lane.front()] == 1) {
                        RVector2D v = lane[1] - lane[0];
                        v = RVector2D(v.y, -v.x);
                        v = v.normalized() * 0.5 * snap_threshold_;
                        candidates.emplace_back(lane.front() + v,
                                                lane.front() - v);
                    }
                    if (degrees[lane.back()] == 1) {
                        RVector2D v = lane[lane.size() - 2] - lane.back();
                        v = RVector2D(v.y, -v.x);
                        v = v.normalized() * 0.5 * snap_threshold_;
                        candidates.emplace_back(lane.back() + v,
                                                lane.back() - v);
                    }
                }
                if (candidates.empty()) break;
                Array<RPolyline2D> cutlines;
                for (auto c : candidates) {
                    if (cutline_extender.ExtendFront(c, borders, &c) &&
                        cutline_extender.ExtendBack(c, borders, &c)) {
                        cutlines.push_back(c);
                    }
                }
                if (cutlines.empty()) break;

                // Found a longest one.
                double len = -1;
                RPolyline2D best_cutline;
                for (auto c : cutlines) {
                    if (c.Length() > len) {
                        len = c.Length();
                        best_cutline = c;
                    }
                }

                lanes_.push_back(best_cutline);
                lane_titles_.push_back("cut_line");
                borders.push_back(best_cutline);

                for (int j = 0; j < lanes.size(); ++j) {
                    auto& lane = lanes[j];

                    if (degrees[lane.front()] == 1) {
                        if (Distance(lane.front(), best_cutline) < threshold) {
                            degrees[lane.front()] = 2;
                        }
                    }
                    if (degrees[lane.back()] == 1) {
                        if (Distance(lane.back(), best_cutline) < threshold) {
                            degrees[lane.back()] = 2;
                        }
                    }
                }
            }
        }
    }

    /**
     * This function processes each yellow/white lane in the road segment
     * individually:
     * 1. For each lane, identifies all its dangling endpoints (unconnected
     *    endpoints not linked to other road features).
     * 2. Generates a perpendicular cut line at each dangling endpoint, oriented
     *    relative to the lane's polyline direction at that point.
     * 3. Check if the cutline is valid.
     */
    void AddCutlinesForGeneralCase(double max_cutline_length) {
        CHECK(max_cutline_length > 0.0);

        Array<RPolyline2D> polylines = lanes_;
        polylines.insert(road_borders_);

        std::unordered_map<RPoint2D, int> degrees;
        for (auto& line : polylines) {
            for (auto p : line)
                ++degrees[p];
        }

        Array<RBox2D> boxes;
        for (int i = 0; i < polylines.size(); ++i) {
            boxes.emplace_back(polylines[i].begin(), polylines[i].end());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        LaneExtender cutline_extender(max_cutline_length);

        Array<int> entries;
        Array<RPolyline2D> cutlines;
        for (int i = 0; i < lanes_.size(); ++i) {
            const RPolyline2D& lane = lanes_[i];
            if (lane.size() <= 1) continue;
            for (int j = 0; j < 2; ++j) {
                RPoint2D q = j == 0 ? lane.front() : lane.back();
                if (degrees[q] != 1) continue;

                RBox2D box(boxes[i].x_min() - max_cutline_length,
                           boxes[i].x_max() + max_cutline_length,
                           boxes[i].y_min() - max_cutline_length,
                           boxes[i].y_max() + max_cutline_length);
                rtree.Find(box, &entries);

                Array<RPolyline2D> borders;
                for (int k : entries) {
                    borders.push_back(polylines[k]);
                }

                RPolyline2D c;
                if (j == 0) {
                    RVector2D v = lane[1] - lane[0];
                    v = RVector2D(v.y, -v.x);
                    v = v.normalized() * 0.5 * snap_threshold_;
                    c = RPolyline2D(lane.front() + v, lane.front() - v);
                } else {
                    RVector2D v = lane[lane.size() - 2] - lane.back();
                    v = RVector2D(v.y, -v.x);
                    v = v.normalized() * 0.5 * snap_threshold_;
                    c = RPolyline2D(lane.back() + v, lane.back() - v);
                }

                if (cutline_extender.ExtendFront(c, borders, &c) &&
                    cutline_extender.ExtendBack(c, borders, &c)) {
                    cutlines.push_back(c);
                }
            }
        }
        for (auto c : cutlines) {
            lanes_.push_back(c);
            lane_titles_.push_back("cut_line");
        }
    }

    /**
     * Connect endpoints of disjoint lanes that intersect with the road
     * intersection border, focusing exclusively on endpoints inside the road
     * intersection.
     *
     * Parameters:
     *  max_cutline_length - Maximum allowed length for the connecting segments
     *                      (cutlines).
     */
    void ConnectLanesAtRoadIntersectionBorder(double max_cutline_length) {
        CHECK(max_cutline_length > 0.0);

        Array<RBox2D> boxes(lanes_.size());
        for (int i = 0; i < lanes_.size(); ++i) {
            boxes[i] = RBox2D(lanes_[i].begin(), lanes_[i].end());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        std::unordered_map<RPoint2D, int> degrees;
        for (auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (auto& line : lanes_) {
            for (auto p : line)
                ++degrees[p];
        }

        std::map<RSegment2D, bool> is_road_edge;
        for (int i = 0; i < road_border_titles_.size(); ++i) {
            if (!road_util::IsRoadSegment(road_border_titles_[i])) continue;

            auto& border = road_borders_[i];
            for (int j = 0; j + 1 < border.size(); ++j) {
                RSegment2D s(border[j], border[j + 1]);
                is_road_edge[s] = true;
            }
        }

        int n_road_intersections = 0;
        for (auto& road_intersection : ProgressBar(road_intersections_)) {
            int id = road_intersection_out_borders_[n_road_intersections++];
            const auto& border = road_borders_[id];

            RBox2D box(border.begin(), border.end());
            Array<int> entries;
            rtree.Find(box, &entries);

            Array<RPolyline2D> lanes;
            for (int i : entries) {
                if (lanes_[i].size() <= 1) continue;
                if (IsProcessLane(lane_titles_[i])) {
                    lanes.push_back(lanes_[i]);
                }
            }

            // Divid road intersection into road and non-road polylines.
            Array<std::pair<RPolyline2D, bool>> divided_border;
            RPolyline2D road_border, non_road_border;
            for (int i = 0; i + 1 < border.size(); ++i) {
                RSegment2D s(border[i], border[i + 1]);
                if (is_road_edge[s]) {
                    if (!non_road_border.empty()) {
                        divided_border.emplace_back(non_road_border, false);
                        non_road_border.clear();
                    }
                    if (road_border.empty()) {
                        road_border.push_back(border[i]);
                    }
                    road_border.push_back(border[i + 1]);
                } else {
                    if (!road_border.empty()) {
                        divided_border.emplace_back(road_border, true);
                        road_border.clear();
                    }
                    if (non_road_border.empty()) {
                        non_road_border.push_back(border[i]);
                    }
                    non_road_border.push_back(border[i + 1]);
                }
            }
            if (!non_road_border.empty())
                divided_border.emplace_back(non_road_border, false);
            else
                divided_border.emplace_back(road_border, true);

            if (divided_border.size() >= 2) {
                if (divided_border.front().second ==
                    divided_border.back().second) {
                    RPolyline2D polyline = divided_border.back().first;
                    polyline.insert(polyline.end(),
                                    divided_border.front().first.begin() + 1,
                                    divided_border.front().first.end());
                    divided_border.front().first = polyline;
                    divided_border.pop_back();
                }
            }

            Array<RPoint2D> dangling_points;
            for (auto& lane : lanes) {
                if (degrees[lane.front()] == 1 &&
                    geometry::Intersect(road_intersection, lane.front())) {
                    dangling_points.push_back(lane.front());
                }
                if (degrees[lane.back()] == 1 &&
                    geometry::Intersect(road_intersection, lane.back())) {
                    dangling_points.push_back(lane.back());
                }
            }

            for (int i = 0; i < divided_border.size(); ++i) {
                if (!divided_border[i].second) continue;
                const RPolyline2D& border = divided_border[i].first;

                RPolyline2D prev_border, next_border;
                if (divided_border.size() > 1) {
                    prev_border = i - 1 < 0 ? divided_border.back().first
                                            : divided_border[i - 1].first;
                    next_border = i + 1 == divided_border.size() ?
                                  divided_border.front().first :
                                  divided_border[i + 1].first;
                }

                Array<RSegment2D> cutlines;
                AddCutlines(road_intersection, border, prev_border, next_border,
                            lanes, dangling_points, max_cutline_length,
                            &cutlines);
                for (auto c : cutlines) {
                    if (geometry::Cross(c, border)) continue;
                    lanes_.emplace_back(c.lower_point(), c.upper_point());
                    lane_titles_.push_back("cut_line");
                }
            }
        }
    }

    /**
     * Connect disjoint lanes to median.
     */
    void ConnectLanesToMedian(double max_connect_distance,
                              double angle_threshold) {
        CHECK(max_connect_distance > 0.0);
        CHECK(angle_threshold > 0.0 && angle_threshold < 180.0);

        Array<RPolyline2D> medians;
        Array<RBox2D> median_boxes;
        for (int i = 0; i < road_borders_.size(); ++i) {
            if (road_util::IsMedian(road_border_titles_[i])) {
                medians.push_back(road_borders_[i]);
                median_boxes.emplace_back(road_borders_[i].begin(),
                                          road_borders_[i].end());
            }
        }
        geometry::RTree2D<double> median_rtree;
        median_rtree.Construct(median_boxes);

        std::unordered_map<RPoint2D, int> degrees;
        for (auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (auto& line : lanes_) {
            for (auto p : line)
                ++degrees[p];
        }

        Array<RPolyline2D> cutlines;
        Array<int> entries;
        for (auto& lane : lanes_) {
            if (lane.size() <= 1) continue;
            for (int i = 0; i < 2; ++i) {
                RPoint2D p = i == 0 ? lane.front() : lane.back();
                RPoint2D p1 = i == 0 ? lane[1] : lane[lane.size() - 2];
                if (degrees[p] != 1) continue;
                RBox2D box(p.x - max_connect_distance,
                           p.x + max_connect_distance,
                           p.y - max_connect_distance,
                           p.y + max_connect_distance);
                median_rtree.Find(box, &entries);
                for (auto j : entries) {
                    RPolyline2D& median = medians[j];
                    auto q = geometry::Project(p, median);
                    double degree = Degree(p1 - p, q - p);
                    if (degree < angle_threshold) continue;
                    if (Distance(p, q) < max_connect_distance) {
                        cutlines.emplace_back(p, q);
                        break;
                    }
                }
            }
        }

        for (auto& c : cutlines) {
            lanes_.push_back(c);
            lane_titles_.push_back("cut_line");
        }
    }

    /**
     * Connect disjoint lanes by adding assit lines.
     */
    void ConnectLanesByAssistLines(double max_connection_distance,
                                   double angle_threshold) {
        CHECK(angle_threshold > 0.0 && angle_threshold < 90.0);

        Array<RSegment2D> assist_lines;
        Array<RPoint2D> incidents;
        std::unordered_map<RPoint2D, int> degrees;
        for (const auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (const auto& line : lanes_) {
            if (line.size() <= 1) continue;

            for (auto p : line)
                ++degrees[p];
            incidents.push_back(line.front());
            incidents.push_back(line.back());
        }

        KDTree<RPoint2D> kdtree(incidents);
        double threshold = max_connection_distance * max_connection_distance;

        for (const auto& lane : lanes_) {
            if (lane.size() < 2) continue;

            for (int j = 0; j < 2; ++j) {
                auto p = j == 0 ? lane.front() : lane.back();
                if (degrees[p] != 1) continue;

                Array<RPoint2D> neighbors;
                kdtree.FindRadiusNeighbors(p, threshold, &neighbors);

                double dis = DBL_MAX;
                RPoint2D best;
                for (auto q : neighbors) {
                    if (q == p) continue;

                    double degree = 0.0;
                    if (j == 0) {
                        degree = Degree(lane[1] - p, q - p);
                    } else {
                        degree = Degree(lane[lane.size() - 2] - p, q - p);
                    }
                    if (degree < 180.0 - angle_threshold) continue;

                    double d = Distance(p, q);
                    if (d < dis) {
                        dis = d;
                        best = q;
                    }
                }
                if (dis < DBL_MAX) {
                    assist_lines.emplace_back(p, best);
                    ++degrees[p];
                    ++degrees[best];
                }
            }
        }

        for (auto c : assist_lines) {
            lanes_.emplace_back(c.lower_point(), c.upper_point());
            lane_titles_.push_back("assist_line");
        }
    }

    /**
     * Connect disjoint lanes by adding cutline.
     */
    void ConnectLanesByCutlines(double max_connection_distance,
                                double angle_threshold) {
        CHECK(max_connection_distance > 0.0);
        CHECK(angle_threshold > 0.0 && angle_threshold < 90.0);

        Array<RSegment2D> cutlines;
        std::unordered_map<RPoint2D, int> degrees;
        for (const auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (const auto& line : lanes_) {
            if (line.size() <= 1) continue;

            for (auto p : line)
                ++degrees[p];
        }
        Array<RPoint2D> incidents;
        std::unordered_map<RPoint2D, int> indices;
        for (int i = 0; i < lanes_.size(); ++i) {
            const RPolyline2D& lane = lanes_[i];
            if (lane.size() < 1) continue;
            if (degrees[lane.front()] == 1) {
                indices[lane.front()] = i;
                incidents.push_back(lane.front());
            }
            if (degrees[lane.back()] == 1) {
                indices[lane.back()] = i;
                incidents.push_back(lane.back());
            }
        }

        KDTree kd_tree(incidents);
        double threshold = max_connection_distance * max_connection_distance;
        Array<RPoint2D> neighbors;
        for (auto p : incidents) {
            if (degrees[p] != 1) continue;
            kd_tree.FindRadiusNeighbors(p, threshold, &neighbors);
            double dis = DBL_MAX;
            RPoint2D best;
            for (auto q : neighbors) {
                if (p == q) continue;
                int i = indices[p];
                int j = indices[q];
                if (i == j) continue;

                RPoint2D p1 = p == lanes_[i].front()
                        ? lanes_[i][1]
                        : lanes_[i][lanes_[i].size() - 2];
                RPoint2D q1 = q == lanes_[j].front()
                        ? lanes_[j][1]
                        : lanes_[j][lanes_[j].size() - 2];

                double d1 = Degree(q1 - q, p - q);
                if (d1 < angle_threshold) continue;

                double d2 = Degree(p1 - p, q - p);
                if (d2 < angle_threshold) continue;

                double d = Distance(p, q);
                if (d < dis) {
                    dis = d;
                    best = q;
                }
            }
            if (dis < DBL_MAX) {
                cutlines.emplace_back(p, best);
                ++degrees[p];
                ++degrees[best];
            }
        }

        for (auto c : cutlines) {
            lanes_.emplace_back(c.lower_point(), c.upper_point());
            lane_titles_.push_back("cut_line");
        }
    }

    /**
     * Add cutlines where lanes change.
     */
    void AddCutlinesWhereLanesChange(double max_cutline_length) {
        geometry::GeoGraph<RPoint2D> graph;

        Array<RPolyline2D> polylines = lanes_;
        polylines.insert(road_borders_);

        Array<RBox2D> boxes(polylines.size());
        for (int i = 0; i < polylines.size(); ++i) {
            if (polylines.size() < 2) continue;
            boxes[i] = RBox2D(polylines[i].begin(), polylines[i].end());
        }
        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);

        LaneExtender cutline_extender(max_cutline_length);

        Array<int> entries;
        for (auto& road : ProgressBar(road_segments_)) {
            RBox2D box(road.bounding_box().x_min() - snap_threshold_,
                       road.bounding_box().x_max() + snap_threshold_,
                       road.bounding_box().y_min() - snap_threshold_,
                       road.bounding_box().y_max() + snap_threshold_);
            rtree.Find(box, &entries);

            Array<RPolyline2D> lanes;
            Array<int> lane_indices;
            Array<RPolyline2D> borders;
            int n_lanes = lanes_.size();
            for (int i : entries) {
                if (i < n_lanes) {
                    if (IsProcessLane(lane_titles_[i])) {
                        lane_indices.push_back(i);
                        lanes.push_back(lanes_[i]);
                    }
                } else {
                    borders.push_back(polylines[i]);
                }
            }
            borders.insert(borders.begin(), lanes.begin(), lanes.end());

            geometry::GeoGraph<RPoint2D> graph;
            for (auto& lane : lanes)
                graph.InsertPath(lane);
            Array<Array<RPoint2D>> polylines;
            graph.GetChains(&polylines);

            std::unordered_map<RPoint2D, int> degrees;
            for (auto& line : polylines) {
                CHECK(line.size() > 1);
                ++degrees[line.front()];
                ++degrees[line.back()];
            }

            for (auto p : degrees) {
                const int n = 3;
                if (p.second != n || !geometry::Cross(road, p.first)) continue;
                int v = graph.FindVertex(p.first);
                CHECK(v != -1);
                Array<int> neighbors;
                graph.GetNeighborVertices(v, &neighbors);
                if (neighbors.size() != n) continue;

                Array<RVector2D> vs(n);
                Array<RVector2D> perpendicular_vs(n);
                for (int j = 0; j < n; ++j) {
                    vs[j] = graph.point(neighbors[j]) - p.first;
                    perpendicular_vs[j] = RVector2D(vs[j].y, -vs[j].x);
                }
                Array<double> angle(n, DBL_MAX);
                for (int j = 0; j < n; ++j) {
                    for (int i = 0; i < n; ++i) {
                        double degree = Degree(perpendicular_vs[j], vs[i]);
                        degree = std::min(degree, 180.0 - degree);
                        angle[j] = std::min(degree, angle[j]);
                    }
                }
                int max_index = std::max_element(angle.begin(), angle.end()) -
                                angle.begin();
                RVector2D v_best = perpendicular_vs[max_index].normalized() *
                                   0.5 * snap_threshold_;
                RPolyline2D c(p.first - v_best, p.first + v_best);
                if (cutline_extender.ExtendFront(c, borders, &c) &&
                    cutline_extender.ExtendBack(c, borders, &c)) {
                    lanes_.push_back(c);
                    lane_titles_.push_back("cut_line");
                }
            }
        }
    }

    /**
     * Detect dangling points in lanes.
     */
    void DetectDanglingPoints(Array<RPoint2D>* dangling_points) const {
        CHECK(dangling_points);

        dangling_points->clear();
        std::unordered_map<RPoint2D, int> degrees;
        for (auto& line : road_borders_) {
            for (auto p : line)
                ++degrees[p];
        }
        for (auto& lane : lanes_) {
            for (auto p : lane)
                ++degrees[p];
        }

        for (auto& lane : lanes_) {
            if (lane.empty()) continue;
            if (degrees[lane.front()] == 1) {
                dangling_points->push_back(lane.front());
            }
            if (degrees[lane.back()] == 1) {
                dangling_points->push_back(lane.back());
            }
        }
    }

    const Array<RPolyline2D>& lanes() const {
        return lanes_;
    }

    const Array<std::string>& lane_titles() const {
        return lane_titles_;
    }

private:
    /**
     * Check if a road category is a road-intersection.
     */
    static bool IsRoadIntersection(const std::string& title) {
        return road_util::IsRoadOverlap(title) ||
               road_util::IsRoadIntersection(title);
    }

    /**
     * Check if the given segment cross with any polyline in the provided
     * collection.
     */
    static bool CheckCross(const RSegment2D& s,
                           const Array<RPolyline2D>& polylines,
                           const geometry::RTree2D<double>& rtree) {
        Array<int> entries;
        rtree.Find(s.bounding_box(), &entries);
        for (int k : entries) {
            if (geometry::Cross(polylines[k], s)) return true;
        }
        return false;
    }

    /**
     * Find the nearest intersection between segment and borders.
     */
    static bool FindNearestIntersection(const RPoint2D& s, const RPoint2D& t,
                                        const Array<RSegment2D>& borders,
                                        RPoint2D* nearest) {
        RSegment2D seg(s, t);
        double dis = DBL_MAX;
        for (auto& b : borders) {
            RPoint2D p;
            if (!geometry::Cross(b, seg, &p)) continue;
            double d = Distance(p, s);
            if (d < dis) {
                *nearest = p;
                dis = d;
            }
        }
        return dis < DBL_MAX;
    }

    /**
     * Return true if the given title is a lane line for lane processing.
     */
    static bool IsProcessLane(const std::string& title) {
        return lane_util::IsWhiteLine(title) ||
               lane_util::IsYellowLine(title) ||
               title == "lane" ||
               title == "assist_line";
    }

    /**
     * Add cutlines to connect dangling points of the specified lanes.
     *
     * Assume that the input lanes are not crossed with others.
     */
    void AddCutlines(const RMultiPolygon2D& road_intersection,
                     const RPolyline2D& border,
                     const RPolyline2D& prev_border,
                     const RPolyline2D& next_border,
                     const Array<RPolyline2D>& lanes,
                     const Array<RPoint2D>& dangling_points,
                     double max_cutline_length,
                     Array<RSegment2D>* cutlines) {
        CHECK(!border.empty());
        CHECK(cutlines);

        cutlines->clear();

        LaneSegmentConnector connector(max_cutline_length);

        std::unordered_set<RPoint2D> on_border;
        for (auto p : border) {
            on_border.insert(p);
        }

        geometry::GeoGraph<RPoint2D> lane_graph;
        lane_graph.set_insert_two_way_edge(true);
        for (const auto& lane : lanes) {
            lane_graph.InsertPath(lane);
        }

        DisjointSet set(lane_graph.n_allocated_edges());
        for (auto v : lane_graph.vertices()) {
            if (!geometry::Intersect(road_intersection, v.point) &&
                on_border.find(v.point) == on_border.end()) continue;

            const auto& edges = lane_graph.edges_from(v.id);
            if (edges.size() >= 2) {
                for (int i = 1; i < edges.size(); ++i) {
                    set.Union(edges.front(), edges[i]);
                }
            }
        }
        for (auto e : lane_graph.edges()) {
            set.Union(e.id, e.id ^ 1);
        }
        Array<Array<int>> components;
        set.ToClusters(&components);

        Array<Array<RSegment2D>> lane_segments;
        for (int i = 0; i < components.size(); ++i) {
            Array<RSegment2D> lane_segs;
            for (auto e : components[i]) {
                if (e % 2 == 0) continue;
                lane_segs.emplace_back(lane_graph.source_point(e),
                                       lane_graph.target_point(e));
            }
            lane_segments.push_back(lane_segs);
        }

        Array<std::pair<int, int>> orders;
        GetLaneGroupOrder(border, lane_segments, &orders);

        // Some lane segments may be contained by others, we need to identify
        // them.
        Array<bool> skip(lane_segments.size(), false);
        for (int i = 0; i < orders.size(); ++i) {
            if (orders[i].first > orders[i].second) {
                skip[i] = true;
                continue;
            }
            for (int j = 0; j < orders.size(); ++j) {
                if (orders[j].first > orders[j].second || skip[j]) {
                    skip[j] = true;
                    continue;
                }
                if (i == j) continue;

                if (orders[i].first >= orders[j].first &&
                    orders[i].second <= orders[j].second) {
                    skip[i] = true;
                    break;
                }
            }
        }

        Array<int> seq;
        IndexSort(orders.begin(), orders.end(), &seq);
        Array<RSegment2D> first, last;
        for (int i = 0; i + 1 < prev_border.size(); ++i) {
            first.emplace_back(prev_border[i], prev_border[i + 1]);
        }
        for (int i = 0; i + 1 < next_border.size(); ++i) {
            last.emplace_back(next_border[i], next_border[i + 1]);
        }

        Array<Array<RSegment2D>> segments;
        segments.push_back(first);
        for (int i : seq) {
            if (skip[i]) continue;
            segments.push_back(lane_segments[i]);
        }
        segments.push_back(last);
        if (segments.size() == 2) return;

        connector.Connect(segments, dangling_points, cutlines);
    }

    // Snap distance.
    double snap_threshold_ = 1e-2;

    // The input road segments.
    Array<RMultiPolygon2D> road_segments_;

    // The input road intersections.
    Array<RMultiPolygon2D> road_intersections_;

    // The outside border of each road intersections.
    Array<int> road_intersection_out_borders_;

    // Polyline borders of road surface and corresponding titles.
    // Road borders may be adjusted by snapping them to the lanes.
    Array<RPolyline2D> road_borders_;
    Array<std::string> road_border_titles_;

    // Lane markings.
    Array<RPolyline2D> lanes_;
    Array<std::string> lane_titles_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_LANE_ENCLOSER_H_

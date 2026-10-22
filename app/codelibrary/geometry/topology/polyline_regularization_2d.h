//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_REGULARIZATION_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_REGULARIZATION_2D_H_

#include <unordered_map>

#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/util/snap_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"

namespace cl {
namespace geometry {

/**
 * Smooth polyline corner by Chaikin's algorithm.
 *
 * Parameters:
 *  polyline          - the input polyline.
 *  corner_angle      - angle in degree to define the corner.
 *  n_iterations      - number of Chaikin's iterations to apply.
 *  smoothed_polyline - the output smoothed polyline.
 */
template <typename T>
void SmoothPolylineCorner(const Array<Point2D<T>>& polyline,
                          double corner_angle,
                          int n_iterations,
                          Array<Point2D<T>>* smoothed_polyline) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(n_iterations > 0);
    CHECK(smoothed_polyline);
    CHECK(corner_angle >= 0.0);
    CHECK(corner_angle <= 180.0);

    *smoothed_polyline = polyline;
    if (polyline.size() <= 2) return;

    bool is_loop = (polyline.front() == polyline.back());

    Array<Point2D<T>> new_polyline;
    while (n_iterations--) {
        new_polyline.clear();

        bool add_last = false;
        Point2D<T> last;
        if (is_loop) {
            const Point2D<T>& prev =
                    (*smoothed_polyline)[smoothed_polyline->size() - 2];
            const Point2D<T>& cur  = (*smoothed_polyline)[0];
            const Point2D<T>& next  = (*smoothed_polyline)[1];
            double angle = Degree(prev - cur, next - cur);
            if (angle <= corner_angle) {
                new_polyline.push_back(0.75 * cur + 0.25 * next.ToVector());
                last = 0.75 * cur + 0.25 * prev.ToVector();
                add_last = true;
            }
        }

        if (!add_last) {
            new_polyline.push_back(smoothed_polyline->front());
        }

        for (int i = 1; i + 1 < smoothed_polyline->size(); ++i) {
            const Point2D<T>& prev = (*smoothed_polyline)[i - 1];
            const Point2D<T>& cur  = (*smoothed_polyline)[i];
            const Point2D<T>& next = (*smoothed_polyline)[i + 1];
            double angle = Degree(prev - cur, next - cur);
            if (angle <= corner_angle) {
                new_polyline.push_back(0.75 * cur + 0.25 * prev.ToVector());
                new_polyline.push_back(0.75 * cur + 0.25 * next.ToVector());
            } else {
                new_polyline.push_back(cur);
            }
        }
        if (is_loop) {
            if (add_last) {
                new_polyline.push_back(last);
            }
            new_polyline.push_back(new_polyline.front());
        } else {
            new_polyline.push_back(smoothed_polyline->back());
        }
        *smoothed_polyline = new_polyline;
    }
}

/**
 * Snap the given polylines within a specified threshold distance.
 */
template <typename T>
void SnapPolylines(double threshold, Array<Polyline<Point2D<T>>>* polylines) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(threshold > 0.0);
    CHECK(polylines);

    Array<Point2D<T>> snap_points;
    for (auto& polyline : *polylines) {
        snap_points.insert(polyline.begin(), polyline.end());
    }
    Snap2D<T> snap(snap_points, threshold);
    for (auto& polyline : *polylines) {
        snap.GetSnapPolyline(polyline, &polyline);
    }
}

/**
 * Connect dangling endpoints of multiple 2D polylines by extending and
 * shrinking them.
 *
 * We assume there is no duplicate points in the polyline. Therefore it is
 * better to call SnapPolylines() before calling this function.
 *
 * Parameter:
 *  extend_distance  - the distance to extend the polylines.
 *  shrink_distance  - the distance to shrink the polylines.
 *  polylines        - an array of Polyline2D, which will be modified in place.
 */
template <typename T>
void ClosePolylines(double extend_distance,
                    double shrink_distance,
                    Array<Polyline<Point2D<T>>>* polylines) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(extend_distance >= 0.0);
    CHECK(shrink_distance >= 0.0);
    CHECK(polylines);

    // Find the dangling endpoints of the polyline.
    std::unordered_map<Point2D<T>, int> end_point_degree;
    Array<Segment2D<T>> segs;
    for (auto& polyline : *polylines) {
        for (auto& p : polyline) {
            ++end_point_degree[p];
        }
    }

    // Extend process.
    Array<bool> is_dangling(polylines->size() * 2, false);
    for (int i = 0; i < polylines->size(); ++i) {
        auto& polyline = (*polylines)[i];
        if (polyline.size() <= 1) continue;

        if (end_point_degree[polyline.front()] == 1) {
            is_dangling[2 * i] = true;
            Vector2D<T> v = (polyline[0] - polyline[1]).normalized();
            polyline.insert(polyline.begin(),
                            polyline[0] + v * extend_distance);
        }
        if (end_point_degree[polyline.back()] == 1) {
            is_dangling[2 * i + 1] = true;
            Vector2D<T> v = (polyline.back() -
                             polyline[polyline.size() - 2]).normalized();
            polyline.push_back(polyline.back() + v * extend_distance);
        }
    }

    for (auto& polyline : *polylines) {
        for (int i = 0; i + 1 < polyline.size(); ++i) {
            segs.emplace_back(polyline[i], polyline[i + 1]);
        }
    }
    RTree2D<T> rtree;
    rtree.Construct(segs);

    // Shrink process.
    Array<int> entries;
    for (int i = 0; i < polylines->size(); ++i) {
        auto& l = (*polylines)[i];
        if (l.size() <= 1) continue;

        if (is_dangling[2 * i]) {
            Array<Point2D<T>> front_poly, tmp_poly;
            polyline_util::Split(l, extend_distance + shrink_distance,
                                 &front_poly, &tmp_poly);

            int best = -1;
            bool found = false;
            Point2D<T> best_p;
            for (int i = 0; i + 1 < front_poly.size(); ++i) {
                if (end_point_degree[front_poly[i]] >= 2) {
                    best = i;
                    best_p = front_poly[i];
                    found = true;
                    break;
                }
                Segment2D<T> s(front_poly[i], front_poly[i + 1]);
                rtree.Find(s.bounding_box(), &entries);

                double best_dis = DBL_MAX;
                for (int j : entries) {
                    Point2D<T> p;
                    if (Cross(s, segs[j], &p)) {
                        double dis = Distance(p, front_poly[i]);
                        if (dis < best_dis) {
                            best = i;
                            best_dis = dis;
                            best_p = p;
                            found = true;
                        }
                    }
                }
                if (found) break;
            }

            if (!found) {
                l.erase(l.begin());
            } else {
                l.erase(l.begin(), l.begin() + best);
                l.front() = best_p;
            }
        }

        if (is_dangling[2 * i + 1]) {
            double length = l.Length() - extend_distance - shrink_distance;
            length = std::max(0.0, length);
            Array<Point2D<T>> back_poly, tmp_poly;
            polyline_util::Split(l, length, &tmp_poly, &back_poly);
            std::reverse(back_poly.begin(), back_poly.end());

            int best = -1;
            bool found = false;
            Point2D<T> best_p;
            for (int i = 0; i + 1 < back_poly.size(); ++i) {
                if (end_point_degree[back_poly[i]] >= 2) {
                    best = i;
                    best_p = back_poly[i];
                    found = true;
                    break;
                }

                Segment2D<T> s(back_poly[i], back_poly[i + 1]);
                rtree.Find(s.bounding_box(), &entries);

                double best_dis = DBL_MAX;
                for (int j : entries) {
                    Point2D<T> p;
                    if (Cross(s, segs[j], &p)) {
                       double dis = Distance(p, back_poly[i]);
                        if (dis < best_dis) {
                            best = i;
                            best_dis = dis;
                            best_p = p;
                            found = true;
                        }
                    }

                }
                if (found) break;
            }

            if (!found) {
                l.pop_back();
            } else {
                l.resize(l.size() - best);
                l.back() = best_p;
            }
        }
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_POLYLINE_REGULARIZATION_2D_H_

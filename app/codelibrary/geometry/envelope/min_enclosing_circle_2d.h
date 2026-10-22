//
// Copyright 2023-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_2D_H_
#define CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_2D_H_

#include <algorithm>
#include <list>
#include <limits>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/distance_2d.h"

namespace cl {
namespace geometry {

/**
 * Computing the smallest circle of p1.
 */
template <typename T>
void MinEnclosingCircle2D(const Point2D<T>& p1, Circle2D<T>* circle) {
    CHECK(circle);

    *circle = Circle2D<T>(p1, 0);
}

/**
 * Computing the smallest circle of p1 and p2.
 */
template <typename T>
void MinEnclosingCircle2D(const Point2D<T>& p1, const Point2D<T>& p2,
                          Circle2D<T>* circle) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(circle);

    *circle = Circle2D<T>(Center(p1, p2),
                          static_cast<T>(Distance(p1, p2) * 0.5));
}

/**
 * Computing the smallest circle of p1, p2 and p3.
 */
template <typename T>
void MinEnclosingCircle2D(const Point2D<T>& p1, const Point2D<T>& p2,
                          const Point2D<T>& p3, Circle2D<T>* circle) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(circle);

    Point2D<T> c = Circumcenter(p1, p2, p3);
    double radius = std::max(std::max(Distance(c, p1), Distance(c, p2)),
                             Distance(c, p3));
    *circle = Circle2D<T>(c, radius);
}

/**
 * Computing the smallest circle of a set of points.
 */
template <typename T>
void MinEnclosingCircle2D(const Array<Point2D<T>>& points, int n_supports,
                          Circle2D<T>* circle) {
    CHECK(n_supports >= 1 && n_supports <= 3) <<
        "The number of supporting points for a circle must be between 1 and 3.";

    CHECK(circle);
    CHECK(points.size() >= n_supports);

    switch (n_supports) {
    case 1:
        MinEnclosingCircle2D(points[0], circle);
        break;
    case 2:
        MinEnclosingCircle2D(points[0], points[1], circle);
        break;
    case 3:
        MinEnclosingCircle2D(points[0], points[1], points[2], circle);
        break;
    default:
        CHECK(false);
    }
}

/**
 * Emo Welzl's move-to-front heuristic to compute the smallest enclosing circle
 * that contains 'points' while ensuring that all points of 'support_set' lie on
 * its boundary.
 *
 * If such a circle does not exist, it will compute a circle that encloses as
 * many points as possible.
 *
 * Parameter:
 *  points      - the input set of points.
 *  support_set - the set of points required to lie on the circle's boundary.
 *  sequence    - the input and output sequence of the points. This function
 *                will rearrange the sequence to ensure that the support set
 *                points are placed at the front.
 *  circle      - the computed enclosing circle.
 *
 * Return the number of support points stored in the front of sequence.
 */
template <typename T>
int MinEnclosingCircle2D(const Array<Point2D<T>>& points,
                          const Array<Point2D<T>>& support_set,
                          std::list<int>* sequence,
                          Circle2D<T>* circle) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(support_set.size() >= 1 && support_set.size() <= 3) <<
        "The number of supporting points for a circle must be between 1 and 3.";
    CHECK(sequence);
    CHECK(circle);

    using Point = Point2D<T>;

    MinEnclosingCircle2D(support_set, support_set.size(), circle);
    if (sequence->empty() || support_set.size() == 3) return 0;

    int n_supports = 0;
    Array<Point2D<T>> supports = support_set;
    std::list<int> seq;

    double cx = circle->center().x, cy = circle->center().y;
    double sqr_r = static_cast<double>(circle->radius()) * circle->radius();
    for (int i : *sequence) {
        Point p = points[i];

        // Check if p is outside the current circle.
        if ((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy) > sqr_r) {
            if (Distance(p, circle->center()) - circle->radius() >
                std::numeric_limits<T>::epsilon() * circle->radius()) {
                supports.push_back(p);
                MinEnclosingCircle2D(points, supports, &seq, circle);
                supports.pop_back();

                seq.push_front(i);
                ++n_supports;
            } else {
                seq.push_back(i);
            }
        } else {
            seq.push_back(i);
        }
    }

    *sequence = seq;
    return n_supports;
}

/**
 * Computing the smallest circle that contain the given points.
 */
template <typename T>
void MinEnclosingCircle2D(const Array<Point2D<T>>& points,
                          Circle2D<T>* circle) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(circle);

    using Point = Point2D<T>;

    if (points.empty()) return;

    constexpr int n_supports = 3;

    double squared_radius = -1.0;
    double cx = circle->center().x, cy = circle->center().y;
    Array<Point> support_points;

    std::list<int> seq;
    int loops_without_progress = 0;
    double best_sqr_r = squared_radius;
    do {
        double sqr_r = squared_radius;

        // Find the farthest point.
        int pivot = 0;
        double max_e = 0.0;
        for (int k = 0; k < points.size(); ++k) {
            const Point& p = points[k];
            double e = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy) -
                       sqr_r;
            if (e > max_e) {
                max_e = e;
                pivot = k;
            }
        }

        if (sqr_r < 0.0 || max_e > 0.0) {
            // Check if the pivot is already contained in the support set.
            if (std::find(seq.begin(), seq.end(), pivot) == seq.end()) {
                Point q = points[pivot];
                if (seq.empty() ||
                    Distance(q, circle->center()) - circle->radius() >
                    std::numeric_limits<T>::epsilon() * circle->radius()) {
                    support_points.push_back(q);
                    MinEnclosingCircle2D(points, support_points, &seq, circle);
                    support_points.pop_back();

                    cx = circle->center().x;
                    cy = circle->center().y;
                    double r = circle->radius();
                    squared_radius = r * r;

                    seq.push_front(pivot);
                    if (seq.size() == n_supports + 1) {
                        seq.pop_back();
                    }
                }
            }
        }

        if (best_sqr_r < squared_radius) {
            best_sqr_r = squared_radius;
            loops_without_progress = 0;
        } else {
            ++loops_without_progress;
        }
    } while (loops_without_progress < 2);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_CIRCLE_2D_H_

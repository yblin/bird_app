//
// Copyright 2019-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// 2D geometric objects generators.
//

#ifndef CODELIBRARY_GEOMETRY_GENERATOR_2D_H_
#define CODELIBRARY_GEOMETRY_GENERATOR_2D_H_

#include <random>

#include "codelibrary/base/array.h"
#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/geometry/vector_2d.h"

namespace cl {
namespace geometry {

/**
 * Randomly generate 2D points in the box.
 */
template <typename T>
class RandomPointInBox2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInBox2D(const Box2D<T>& box)
        : uniform_x_(box.x_min(), box.x_max()),
          uniform_y_(box.y_min(), box.y_max()) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        return Point2D<T>(uniform_x_(engine), uniform_y_(engine));
    }

private:
    // For generating uniform 2D points.
    std::uniform_real_distribution<T> uniform_x_, uniform_y_;
};

/**
 * Randomly generate real point in 2D circle.
 */
template <typename T>
class RandomPointInCircle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInCircle2D(const Circle2D<T>& circle)
        : uniform_angle_(0, 2 * numbers::pi_v<T>),
          uniform_r_(0, T(1)),
          radius_(circle.radius()),
          center_x_(circle.center().x),
          center_y_(circle.center().y) {}

    /**
     * Generate a 2D point in the circle.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T angle = uniform_angle_(engine);
        T sqrt_r = std::sqrt(uniform_r_(engine));
        T x = sqrt_r * std::cos(angle) * radius_;
        T y = sqrt_r * std::sin(angle) * radius_;

        return Point2D<T>(x + center_x_, y + center_y_);
    }

private:
    // For generating uniform angles in range [0, 2 * PI).
    std::uniform_real_distribution<T> uniform_angle_;

    // For generating uniform value in range [0, 1].
    std::uniform_real_distribution<T> uniform_r_;

    // The circle that is used to generate the random points in it.
    T radius_, center_x_, center_y_;
};

/**
 * Randomly generate 2D points (almost) on a 2D circle.
 */
template <typename T>
class RandomPointOnCircle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointOnCircle2D(const Circle2D<T>& circle)
        : uniform_angle_(0, 2 * numbers::pi_v<T>),
          radius_(circle.radius()),
          center_x_(circle.center().x),
          center_y_(circle.center().y) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        // Generate the random point in the unite circle.
        T angle = uniform_angle_(engine);
        T x = std::cos(angle) * radius_ + center_x_;
        T y = std::sin(angle) * radius_ + center_y_;
        return Point2D<T>(x, y);
    }

private:
    // For generating uniform angles in range [0, 2PI).
    std::uniform_real_distribution<T> uniform_angle_;

    // The circle that is used to generate the random points in it.
    T radius_, center_x_, center_y_;
};

/**
 * Randomly generate 2D point in the 2D triangle.
 *
 * According to [1], a random point, P, uniformly from within triangle ABC,
 * can be computed by the following convex combination of the vertices:
 *
 *   P = (1 - \sqrt(r1)) A + (\sqrt(r1)(1-r2)) B + (r2 \sqrt(r1)) C
 *
 * where r1, r2 ~ U[0, 1]
 *
 * Note that the generated points just almost on the triangle, not exact.
 *
 * Reference:
 * [1] Osada R, Funkhouser T, Chazelle B, et al. Shape distributions[J]. ACM
 *     Transactions on Graphics, 2002, 21(4):807-832.
 */
template <typename T>
class RandomPointInTriangle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInTriangle2D(const Triangle2D<T>& triangle)
        : uniform_(0, 1) {
        CHECK(!triangle.empty());

        x1_ = triangle.vertices()[0].x;
        x2_ = triangle.vertices()[1].x;
        x3_ = triangle.vertices()[2].x;
        y1_ = triangle.vertices()[0].y;
        y2_ = triangle.vertices()[1].y;
        y3_ = triangle.vertices()[2].y;
    }

    /**
     * Random generate point on the given triangle.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T r1 = uniform_(engine);
        T r2 = uniform_(engine);
        T r1_sqrt = std::sqrt(r1);
        T t1 = 1 - r1_sqrt;
        T t2 = r1_sqrt * (1 - r2);
        T t3 = r2 * r1_sqrt;

        return Point2D<T>(t1 * x1_ + t2 * x2_ + t3 * x3_,
                          t1 * y1_ + t2 * y2_ + t3 * y3_);
    }

private:
    T x1_, x2_, x3_, y1_, y2_, y3_;
    std::uniform_real_distribution<T> uniform_;
};

/**
 * Generate random point (almost) on a 2D line segment.
 */
template <typename T>
class RandomPointOnSegment2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointOnSegment2D(const Segment2D<T>& line)
        : uniform_(0, 1),
          direction_(line.direction().x, line.direction().y),
          x_(line.lower_point().x),
          y_(line.lower_point().y) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T t = uniform_(engine);
        return Point2D<T>(t * direction_.x + x_, t * direction_.y + y_);
    }

private:
    // Uniform generator.
    std::uniform_real_distribution<T> uniform_;

    // The input segment.
    Vector2D<T> direction_;
    T x_, y_;
};

/**
 * Evenly generate N points (almost) on a 2D segment.
 */
template <typename T>
void EvenlyPointsOnSegment2D(const Point2D<T>& start, const Point2D<T>& end,
                             int n, Array<Point2D<T>>* points) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(n >= 0);
    CHECK(points);

    points->clear();
    if (n == 0) return;

    if (start == end) {
        points->resize(n, start);
        return;
    }

    Vector2D<T> v = (end - start) * (1.0 / (n - 1.0));
    for (int i = 0; i + 1 < n; ++i) {
        points->push_back(start + v * i);
    }
    points->push_back(end);
}

/**
 * Evenly generate N points (almost) on a 2D circle (in anticlockwise
 * orientation).
 */
template <typename T>
void EvenlyPointsOnCircle2D(const Circle2D<T>& circle, int n,
                            Array<Point2D<T>>* points) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(n >= 0);
    CHECK(points);

    points->clear();
    if (n == 0) return;

    T step = (2 * numbers::pi_v<T>) / n;
    T angle = 0;
    for (int i = 0; i < n; ++i) {
        points->push_back(circle.GetPoint(angle));
        angle += step;
        if (angle >= 2 * numbers::pi_v<T>) angle = 0;
    }
}

/**
 * Evenly generate N points (almost) on a 2D circular arc.
 */
template <typename T>
void EvenlyPointsOnArc2D(const Arc2D<T>& arc, int n,
                         Array<Point2D<T>>* points) {
    CHECK(n >= 0);
    CHECK(points);

    points->clear();
    if (arc.empty()) {
        points->resize(n, arc.start_point());
        return;
    }

    if (n == 0) return;
    if (n == 1) {
        points->push_back(arc.start_point());
        return;
    }

    if (arc.orientation() == 0) {
        EvenlyPointsOnSegment2D(arc.start_point(), arc.end_point(), n, points);
        return;
    }

    points->push_back(arc.start_point());

    T start = arc.start_angle();
    T end = arc.end_angle();
    T angle = start;
    int first = 1;

    if (arc.orientation() > 0) {
        if (start < end) end -= 2 * numbers::pi_v<T>;
        T step = (start - end) / (n - 1);
        if (arc.is_circle()) {
            step = (2 * numbers::pi_v<T>) / (n - 1);
        }
        Circle2D<T> circle = arc.GetCircle();

        for (int i = 1; i < n - 1; ++i) {
            angle -= step;
            Point2D<T> q = circle.GetPoint(angle);
            T t = Radian(q - arc.center());
            if (arc.IsAngleBetween(t)) {
                points->push_back(q);
            } else {
                if (i == first) {
                    ++first;
                    points->push_back(arc.start_point());
                } else {
                    points->push_back(arc.end_point());
                }
            }
        }
    } else {
        if (start > end) end += 2 * numbers::pi_v<T>;
        T step = (end - start) / (n - 1);
        if (arc.is_circle()) {
            step = (2 * numbers::pi_v<T>) / (n - 1);
        }
        Circle2D<T> circle = arc.GetCircle();

        for (int i = 1; i < n - 1; ++i) {
            angle += step;
            Point2D<T> q = circle.GetPoint(angle);
            T t = Radian(q - arc.center());
            if (arc.IsAngleBetween(t)) {
                points->push_back(q);
            } else {
                if (i == first) {
                    ++first;
                    points->push_back(arc.start_point());
                } else {
                    points->push_back(arc.end_point());
                }
            }
        }
    }
    points->push_back(arc.end_point());
}

/**
 * Evenly generate N points (almost) on a 2D polyline.
 */
template <typename T>
void EvenlyPointsOnPolyline2D(const Array<Point2D<T>>& polyline, int n,
                              Array<Point2D<T>>* points) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(n >= 0);
    CHECK(points);
    CHECK(points != &polyline);

    points->clear();
    if (n == 0 || polyline.empty()) return;

    if (n == 1) {
        points->push_back(polyline.front());
        return;
    }

    double length = 0.0;
    Array<double> seg_lengths;
    for (int i = 0; i + 1 < polyline.size(); ++i) {
        double seg_len = Distance(polyline[i], polyline[i + 1]);
        seg_lengths.push_back(seg_len);
        length += seg_len;
    }
    if (length == 0.0) {
        while (points->size() < n)
            points->push_back(polyline[0]);
        return;
    }

    double step = length / (n - 1);
    double current_length = 0.0;
    int s = 0;

    points->push_back(polyline[0]);
    for (int i = 1; i + 1 < n; ++i) {
        current_length += step;

        while (s < seg_lengths.size() && current_length > seg_lengths[s]) {
            current_length -= seg_lengths[s];
            ++s;
        }
        if (s >= seg_lengths.size()) break;

        double t = current_length / seg_lengths[s];
        points->emplace_back(polyline[s].x * (1 - t) + polyline[s + 1].x * t,
                             polyline[s].y * (1 - t) + polyline[s + 1].y * t);
    }
    while (points->size() < n)
        points->push_back(polyline.back());
}

/**
 * Evenly generate N points with tangent vectors (almost) on a 2D polyline.
 */
template <typename T>
void EvenlyPointsWithTangentOnPolyline2D(const Array<Point2D<T>>& polyline,
                                         int n,
                                         Array<Point2D<T>>* points,
                                         Array<Vector2D<T>>* tangents) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(n >= 0);
    CHECK(points);
    CHECK(tangents);
    CHECK(points != &polyline);

    points->clear();
    tangents->clear();
    if (n == 0 || polyline.empty()) return;

    if (polyline.size() == 1) {
        for (int i = 0; i < n; ++i) {
            points->push_back(polyline.front());
            tangents->emplace_back(0, 0);
        }
        return;
    }

    if (n == 1) {
        points->push_back(polyline.front());
        tangents->push_back((polyline[1] - polyline[0]).normalized());
        return;
    }

    double length = 0.0;
    Array<double> seg_lengths;

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        double seg_len = Distance(polyline[i], polyline[i + 1]);
        seg_lengths.push_back(seg_len);
        length += seg_len;
    }

    double step = length / (n - 1);
    double current_length = 0.0;
    int s = 0;

    points->push_back(polyline[0]);
    tangents->push_back((polyline[1] - polyline[0]).normalized());
    for (int i = 1; i + 1 < n; ++i) {
        current_length += step;

        while (s < seg_lengths.size() && current_length > seg_lengths[s]) {
            current_length -= seg_lengths[s];
            ++s;
        }
        if (s >= seg_lengths.size()) break;

        double t = current_length / seg_lengths[s];
        points->emplace_back(polyline[s].x * (1 - t) + polyline[s + 1].x * t,
                             polyline[s].y * (1 - t) + polyline[s + 1].y * t);
        tangents->push_back((polyline[s + 1] - polyline[s]).normalized());
    }
    while (points->size() < n) {
        points->push_back(polyline.back());
        tangents->push_back((polyline.back() -
                             polyline[polyline.size() - 2]).normalized());
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_GENERATOR_2D_H_

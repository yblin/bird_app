//
// Copyright 2013-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_FITTING_LEAST_MEDIAN_SQUARES_H_
#define CODELIBRARY_POINT_CLOUD_FITTING_LEAST_MEDIAN_SQUARES_H_

#include <algorithm>
#include <cfloat>
#include <limits>
#include <random>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/point_compare_3d.h"
#include "codelibrary/statistics/median.h"

namespace cl {
namespace point_cloud {

/**
 * Least median square fitting for 3D plane of 3D points.
 *
 * Return the median distance from points to the plane.
 */
template <typename T>
double LeastMedianSquares(const Array<Point3D<T>>& points, Plane3D<T>* plane) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(points.size() > 0);
    CHECK(plane);

    // A default horizontal plane that goes through the first input point.
    *plane = Plane3D<T>(points[0], Vector3D<T>(0, 0, 1));

    // The iterations number m can be determined by following equation:
    //   P = 1 - (1 - e^k)^m,
    // where e is the fraction of inliers, k is the least number of points to
    // define the model and P is the probability that at least one of the m
    // sub-samples is good.
    // Impose e = 0.5, k = 3 and m = 50, we have P = 99.87%.
    const int n_iterations = 50;

    double min_distance = DBL_MAX;

    std::mt19937 random_engine;
    std::uniform_int_distribution<int> uniform(0, points.size() - 1);

    for (int itr = 0; itr < n_iterations; ++itr) {
        int a = uniform(random_engine);
        int b = uniform(random_engine);
        int c = uniform(random_engine);
        if (a == b || b == c || a == c) continue;

        Point3D<T> pa = points[a], pb = points[b], pc = points[c];
        if (pa == pb || pa == pc || pb == pc) continue;

        Plane3D<T> plane_of_3_points(pa, pb, pc);

        Array<double> distances(points.size());
        for (int i = 0; i < points.size(); ++i) {
            distances[i] = Distance(points[i], plane_of_3_points);
        }

        double distance = Median(distances.begin(), distances.end());
        if (distance < min_distance) {
            min_distance = distance;
            *plane = plane_of_3_points;
        }
    }

    return min_distance;
}

/**
 * Least median square fitting for 2D line.
 *
 * Return the median distance from points to the line.
 */
template <typename T>
double LeastMedianSquares(const Array<Point2D<T>>& points, Line2D<T>* line) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(points.size() > 0);
    CHECK(line);

    // A default line along X axis.
    *line = Line2D<T>(points[0], Vector2D<T>(1, 0));

    // The iterations number m can be determined by following equation:
    //   P = 1 - (1 - e^k)^m,
    // where e is the fraction of inliers, k is the least number of points to
    // define the model and P is the probability that at least one of the m
    // sub-samples is good.
    // Impose e = 0.5, k = 2 and m = 30, we have P = 99.98%.
    const int n_iterations = 30;

    double min_distance = DBL_MAX;

    int size = points.size();
    std::mt19937 random_engine;
    std::uniform_int_distribution<int> uniform(0, size - 1);

    for (int itr = 0; itr < n_iterations; ++itr) {
        const Point2D<T>& pa = points[uniform(random_engine)];
        const Point2D<T>& pb = points[uniform(random_engine)];
        if (pa == pb) continue;

        Line2D<T> l(pa, pb);

        Array<double> distances(size);
        for (int i = 0; i < size; ++i) {
            distances[i] = Distance(points[i], l);
        }

        double distance = Median(distances.begin(), distances.end());
        if (distance < min_distance) {
            min_distance = distance;
            *line = l;
        }
    }

    return min_distance;
}

/**
 * Least median square fitting for 3D line.
 *
 * Return the median distance from points to the line.
 */
template <typename T>
double LeastMedianSquares(const Array<Point3D<T>>& points, Line3D<T>* line) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(points.size() > 0);
    CHECK(line);

    // A default line along Z axis.
    *line = Line3D<T>(points[0], Vector3D<T>(0, 0, 1));

    // The iterations number m can be determined by following equation:
    //   P = 1 - (1 - e^k)^m,
    // where e is the fraction of outliers, k is the least number of points to
    // define the model and P is the probability that at least one of the m
    // sub-samples is good.
    // Impose e = 0.5, k = 2 and m = 30, we have P = 99.98%.
    const int n_iterations = 30;

    double min_distance = std::numeric_limits<double>::max();

    int size = points.size();
    std::mt19937 random_engine;
    std::uniform_int_distribution<int> uniform(0, size - 1);

    for (int itr = 0; itr < n_iterations; ++itr) {
        const Point3D<T>& pa = points[uniform(random_engine)];
        const Point3D<T>& pb = points[uniform(random_engine)];
        if (pa == pb) continue;

        Line3D<T> l(pa, pb);

        Array<double> distances(size);
        for (int i = 0; i < size; ++i) {
            distances[i] = Distance(points[i], l);
        }

        double distance = Median(distances.begin(), distances.end());
        if (distance < min_distance) {
            min_distance = distance;
            *line = l;
        }
    }

    return min_distance;
}

/**
 * See previous one.
 */
template <typename T>
double LeastMedianSquares(const Array<Point3D<T>>& points, Segment3D<T>* seg) {
    CHECK(seg);

    Line3D<T> line;
    double dis = LeastMedianSquares(points, &line);

    geometry::PointDotCompare3D<T> compare(line.point1(), line.point2());
    auto start = *std::min_element(points.begin(), points.end(), compare);
    auto end = *std::max_element(points.begin(), points.end(), compare);

    *seg = Segment3D<T>(start, end);
    return dis;
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_FITTING_LEAST_MEDIAN_SQUARES_H_

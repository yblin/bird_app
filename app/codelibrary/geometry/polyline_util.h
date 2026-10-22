//
// Copyright 2020-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Implement common polyline functions.
//

#ifndef CODELIBRARY_GEOMETRY_POLYLINE_UTIL_H_
#define CODELIBRARY_GEOMETRY_POLYLINE_UTIL_H_

#include <algorithm>
#include <queue>
#include <utility>

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace geometry {
namespace polyline_util {

/**
 * Find the index of the nearest segment of the polyline to p.
 */
template <typename Point>
int FindNearestSegment(const Array<Point>& polyline, const Point& p) {
    using Segment = typename Polyline<Point>::Segment;

    double dis = DBL_MAX;
    int best = -1;
    for (int i = 0; i + 1 < polyline.size(); ++i) {
        double d = Distance(p, Segment(polyline[i], polyline[i + 1]));
        if (d < dis) {
            dis = d;
            best = i;
        }
    }
    return best;
}

/**
 * Find the index of the nearest vertex of the polyline to p.
 */
template <typename Point>
int FindNearestVertex(const Array<Point>& polyline, const Point& p) {
    double dis = DBL_MAX;
    int best = -1;
    for (int i = 0; i < polyline.size(); ++i) {
        double d = Distance(p, polyline[i]);
        if (d < dis) {
            dis = d;
            best = i;
        }
    }
    return best;
}

/**
 * Get the unit tangent vector of a 2D polyline at the specified vertex index.
 */
template <typename T>
Vector2D<T> GetTangent(const Array<Point2D<T>>& polyline, int index) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(index >= 0 && index < polyline.size());

    if (polyline.size() == 1) return Vector2D<T>(0, 0);

    int prev = index == 0 ? 0 : index - 1;
    int next = index + 1 == polyline.size() ? polyline.size() - 1 : index + 1;
    while (prev > 0 && polyline[prev] == polyline[index]) {
        --prev;
    }
    while (next + 1 < polyline.size() && polyline[next] == polyline[index]) {
        ++next;
    }

    Vector2D<T> dir1 = (polyline[index] - polyline[prev]).normalized();
    Vector2D<T> dir2 = (polyline[next] - polyline[index]).normalized();
    return (dir1 + dir2).normalized();
}

/**
 * Get the normal vector of polyline at the index-th vertex.
 */
template <typename T>
Vector2D<T> GetNormal(const Array<Point2D<T>>& polyline, int index) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(index >= 0 && index < polyline.size());

    Vector2D<T> v = GetTangent(polyline, index);
    return Vector2D<T>(-v.y, v.x);
}

/**
 * Split a polyline at a specified point 'p'. If the point 'p' is not the vertex
 * of the polyline, it will be inserted into the polyline at the nearest segment
 * position determined by FindNearestSegment() and then split.
 */
template <typename Point>
void Split(const Array<Point>& polyline, const Point& p,
           Array<Point>* polyline1, Array<Point>* polyline2) {
    CHECK(polyline.size() > 1);
    CHECK(polyline1);
    CHECK(polyline2);

    int index = FindNearestSegment(polyline, p);
    CHECK(index != -1);

    polyline1->assign(polyline.begin(), polyline.begin() + index + 1);
    if (polyline1->back() != p) polyline1->push_back(p);

    polyline2->clear();
    polyline2->push_back(p);
    if (p != polyline[index + 1]) {
        polyline2->insert(polyline.begin() + index + 1, polyline.end());
    } else {
        polyline2->insert(polyline.begin() + index + 2, polyline.end());
    }
}

/**
 * Split a polyline at the specified length.
 */
template <typename Point>
void Split(const Array<Point>& polyline, double length,
           Array<Point>* polyline1, Array<Point>* polyline2) {
    CHECK(length >= 0.0);
    CHECK(polyline1);
    CHECK(polyline2);
    CHECK(polyline1 != &polyline);
    CHECK(polyline2 != &polyline);
    CHECK(polyline1 != polyline2);

    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value);

    polyline1->clear();
    polyline2->clear();

    if (polyline.size() <= 1 || length == 0.0) {
        *polyline1 = polyline;
        return;
    }

    double l = 0.0;
    polyline1->push_back(polyline[0]);
    for (int i = 1; i < polyline.size(); ++i) {
        double d = Distance(polyline[i], polyline[i - 1]);
        if (d + l > length) {
            double r = (length - l) / d;
            Vector2D<T> v = (polyline[i] - polyline[i - 1]) * r;
            polyline1->push_back(polyline[i - 1] + v);
            polyline2->push_back(polyline[i - 1] + v);
            for (int j = i; j < polyline.size(); ++j) {
                polyline2->push_back(polyline[j]);
            }

            break;
        } else {
            polyline1->push_back(polyline[i]);
            l += d;
        }
    }
}

/**
 * The N-th point routine is a naive O(n) algorithm polyline simplification.
 * It keeps only the first, last, and each nth point. All other points are
 * removed.
 */
template <typename Point>
void NthPointSimplify(const Array<Point>& polyline, int n_th,
                      Array<Point>* result) {
    CHECK(n_th > 0);
    CHECK(result);

    result->resize(polyline.size());
    int size = 0;
    for (int i = 0; i < polyline.size(); i += n_th) {
        (*result)[size++] = polyline[i];
    }
    result->resize(size);
}

/**
 * The Douglas-Peucker algorithm to simplify the given polyline.
 *
 * The algorithm defines 'dissimilar' based on the maximum distance between the
 * original curve and the simplified curve (i.e., the Hausdorff distance between
 * the curves). The simplified curve consists of a subset of the points that
 * defined the original curve.
 */
template <typename Point>
void DouglasPeuckerSimplify(const Array<Point>& polyline, double threshold,
                            Array<int>* result) {
    CHECK(threshold > 0.0);
    CHECK(result);

    result->clear();
    if (polyline.size() < 2) {
        for (int i = 0; i < polyline.size(); ++i) {
            result->push_back(i);
        }
        return;
    }

    Array<bool> is_remain(polyline.size(), false);
    std::queue<std::pair<int, int>> q;
    q.push(std::make_pair(0, polyline.size() - 1));
    while (!q.empty()) {
        std::pair<int, int> p = q.front();
        q.pop();

        is_remain[p.first] = true;
        is_remain[p.second] = true;

        typename Polyline<Point>::Segment seg(polyline[p.first],
                                              polyline[p.second]);
        int split = -1;
        double dis = 0.0;
        for (int i = p.first + 1; i < p.second; ++i) {
            double d = Distance(polyline[i], seg);
            if (d > dis) {
                dis = d;
                split = i;
            }
        }

        if (split == -1 || dis < threshold) continue;

        if (p.first < split) q.push(std::make_pair(p.first, split));
        if (split < p.second) q.push(std::make_pair(split, p.second));
    }

    for (int i = 0; i < polyline.size(); ++i) {
        if (is_remain[i]) result->push_back(i);
    }
}
template <typename Point>
void DouglasPeuckerSimplify(const Array<Point>& polyline, double threshold,
                            Array<Point>* result) {
    CHECK(result);

    Array<int> indices;
    DouglasPeuckerSimplify(polyline, threshold, &indices);
    Array<Point> temp;
    temp.reserve(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        temp.push_back(polyline[indices[i]]);
    }
    *result = std::move(temp);
}

/**
 * Densify the input polyline so that no segment exceeds the 'step'.
 *
 * This function preserves all unique original vertices and geometric features
 * (e.g., sharp corners) of the input polyline, and the overall shape of the
 * polyline remains unchanged - only additional vertices are inserted into
 * segments longer than 'step'.
 */
template <typename Point>
void Densify(const Array<Point>& polyline, double step,
             Array<Point>* output) {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value);

    CHECK(output);
    CHECK(&polyline != output);

    output->clear();
    if (polyline.empty()) return;

    output->push_back(polyline.front());
    for (int i = 0; i + 1 < polyline.size(); ++i) {
        if (polyline[i] == polyline[i + 1]) continue;
        auto v = (polyline[i + 1] - polyline[i]).normalized();
        double dis = Distance(polyline[i], polyline[i + 1]);

        int n = std::max(1, static_cast<int>(std::ceil(dis / step)));
        double s = dis / n;
        for (int j = 1; j < n; ++j) {
            output->push_back(polyline[i] + v * s * j);
        }
        output->push_back(polyline[i + 1]);
    }
}

/**
 * Subdivide the input polyline into exactly N vertices by sampling uniformly
 * along the total length of the polyline (resulting in N-1 uniform segments).
 *
 * Unlike Densify (which preserves the original polyline shape by only
 * subdividing overlong segments), this function samples vertices uniformly
 * along the total length of the polyline to form exactly N segments, and does
 * not guarantee the preservation of original geometric features such as sharp
 * corners.
 */
template <typename Point>
bool Subdivide(const Array<Point>& polyline, int n,
               Array<Point>* output_polyline) {
    CHECK(n >= 2);
    CHECK(output_polyline);

    *output_polyline = polyline;
    if (polyline.size() <= 1) return false;

    Array<Point> output_poly;

    double length = 0.0;
    Array<double> seg_lengths;
    for (int i = 0; i + 1 < polyline.size(); ++i) {
        double seg_len = Distance(polyline[i], polyline[i + 1]);
        seg_lengths.push_back(seg_len);
        length += seg_len;
    }
    if (length == 0.0) return false;

    double step = length / (n - 1);
    CHECK(step > 0.0);

    double current_length = 0.0;
    int s = 0;

    output_poly.push_back(polyline[0]);
    for (int i = 1; i + 1 < n; ++i) {
        current_length += step;

        while (s < seg_lengths.size() && current_length > seg_lengths[s]) {
            current_length -= seg_lengths[s];
            ++s;
        }
        if (s >= seg_lengths.size()) break;

        CHECK(seg_lengths[s] > 0.0);
        double t = current_length / seg_lengths[s];
        Point p = polyline[s] * (1 - t) + polyline[s + 1].ToVector() * t;
        output_poly.push_back(p);
    }
    while (output_poly.size() < n) {
        output_poly.push_back(polyline.back());
    }

    *output_polyline = output_poly;
    return true;
}

/**
 * Subdivide the input polyline into vertices uniformly spaced at approximately
 * 'resolution' distance, by estimating the number of vertices (N) from the
 * total length and resolution.
 */
template <typename Point>
bool Subdivide(const Array<Point>& polyline, double resolution,
               Array<Point>* output_polyline) {
    CHECK(resolution >= DBL_EPSILON);
    CHECK(output_polyline);

    *output_polyline = polyline;

    double length = 0.0;
    Array<double> seg_lengths;
    for (int i = 0; i + 1 < polyline.size(); ++i) {
        length += Distance(polyline[i], polyline[i + 1]);
    }
    if (length == 0.0) return false;

    double t = std::ceil(length / resolution);
    CHECK(t <= INT_MAX);
    int n = static_cast<int>(t);
    n = std::max(2, n + 1);
    return Subdivide(polyline, n, output_polyline);
}

} // namespace polyline_util
} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POLYLINE_UTIL_H_

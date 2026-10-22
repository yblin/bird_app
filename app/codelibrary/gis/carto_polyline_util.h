//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Low-level utility functions for CartoPolyline and CartoPolylineKernel data
// structures.
//

#ifndef CODELIBRARY_GIS_CARTO_POLYLINE_UTIL_H_
#define CODELIBRARY_GIS_CARTO_POLYLINE_UTIL_H_

#include <cfloat>
#include <unordered_map>

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/gis/carto_polyline.h"

namespace cl {
namespace gis {
namespace carto_polyline_util {

/**
 * Decompose a CartoPolyline into CartoPolylineKernels by width threshold.
 *
 * It ensure the width variation (max width - min width) within each Kernel does
 * not exceed the specified width threshold.
 */
template <typename Point>
void DecomposeToKernels(const CartoPolyline<Point>& carto_polyline,
                        double width_threshold,
                        Array<CartoPolylineKernel<Point>>* kernels) {
    CHECK(kernels);
    CHECK(width_threshold > 0.0);

    kernels->clear();
    if (carto_polyline.empty()) return;

    const Polyline<Point>& polyline = carto_polyline.polyline;
    const Array<double>& widths = carto_polyline.widths;
    CHECK(polyline.size() == widths.size());

    CartoPolylineKernel<Point> s;
    s.category = carto_polyline.category;
    s.level = carto_polyline.level;
    double min_w = widths[0];
    double max_w = widths[0];
    double total_w = 0.0;
    double total_l = 0.0;
    double sum_w = 0.0;

    s.polyline.push_back(polyline[0]);
    for (int i = 1; i < polyline.size(); ++i) {
        double l = Distance(polyline[i - 1], polyline[i]);

        min_w = std::min(widths[i], min_w);
        max_w = std::max(widths[i], max_w);
        if (max_w - min_w > width_threshold) {
            if (s.polyline.size() > 1) {
                if (total_l == 0.0) {
                    s.width = sum_w / s.polyline.size();
                } else {
                    s.width = total_w / total_l;
                }
                kernels->push_back(s);
            }

            if (s.polyline.size() == 1 ||
                std::abs(widths[i - 1] - widths[i]) > width_threshold) {
                s.polyline = Polyline<Point>(polyline[i - 1], polyline[i]);
                s.width = 0.5 * (widths[i - 1] + widths[i]);
                kernels->push_back(s);

                s.polyline.clear();
                max_w = min_w = widths[i];
                total_w = 0.0;
                total_l = 0.0;
            } else {
                s.polyline.clear();
                s.polyline.push_back(polyline[i - 1]);
                max_w = std::max(widths[i], widths[i - 1]);
                min_w = std::min(widths[i], widths[i - 1]);
                total_w = 0.0;
                total_l = 0.0;
                sum_w = widths[i - 1];
            }
        }

        s.polyline.push_back(polyline[i]);
        total_l += l;
        double mean_w = 0.5 * (widths[i - 1] + widths[i]);
        sum_w += widths[i];
        total_w += mean_w * l;
    }

    if (s.polyline.size() > 1) {
        if (total_l == 0.0) {
            s.width = total_w / s.polyline.size();
        } else {
            s.width = total_w / total_l;
        }
        kernels->push_back(s);
    }
}

/**
 * Merge a kernel into another one. Required two kernels are adjacent.
 */
template <typename Point>
void MergeTwoAdjacentKernels(const CartoPolylineKernel<Point>& right,
                             CartoPolylineKernel<Point>* left) {
    CHECK(left);

    if (left->empty()) {
        *left = right;
        return;
    }

    double l1 = right.polyline.Length(), l2 = left->polyline.Length();
    double total_length = l1 + l2;

    if (total_length == 0.0) {
        left->width = 0.0;
    } else {
        left->width = (right.width * l1 + left->width * l2) / total_length;
    }
    if (!right.polyline.empty() &&
        left->polyline.back() != right.polyline.front()) {
        left->polyline.insert(right.polyline);
    } else if (right.polyline.size() > 1) {
        left->polyline.insert(left->polyline.end(), right.polyline.begin() + 1,
                              right.polyline.end());
    }
}

/**
 * Merge contiguous (ordered) CartoPolylineKernels under the following
 * conditions:
 * 1. Width difference between adjacent kernels <= specified width threshold;
 * 2. Previous kernel's length <= specified length threshold;
 * 3. Categories are identical OR one category is empty.
 */
template <typename Point>
void MergeAdjacentKernels(double width_threshold, double length_threshold,
                          Array<CartoPolylineKernel<Point>>* kernels) {
    CHECK(kernels);
    CHECK(width_threshold > 0.0) << "Width threshold must be positive.";
    CHECK(length_threshold >= 0.0) << "Length threshold must be non-negative.";

    if (kernels->empty()) return;

    Array<CartoPolylineKernel<Point>> merged_kernels;
    merged_kernels.push_back((*kernels)[0]);

    for (int i = 1; i < kernels->size(); ++i) {
        const CartoPolylineKernel<Point>& current = (*kernels)[i];
        CartoPolylineKernel<Point>& last = merged_kernels.back();
        double last_length = last.polyline.Length();
        double current_length = current.polyline.Length();
        CHECK(last.level == current.level);

        if (!last.category.empty() && !current.category.empty() &&
            last.category != current.category) {
            merged_kernels.push_back(current);
            continue;
        }

        if (std::abs(current.width - last.width) <= width_threshold ||
            last_length <= length_threshold ||
            current_length <= length_threshold) {
            MergeTwoAdjacentKernels(current, &last);
        } else {
            merged_kernels.push_back(current);
        }
    }

    *kernels = merged_kernels;
}

/**
 * Merge adjacent short (ordered) kernels without category/width consistency
 * checks.
 */
template <typename Point>
void MergeShortAdjacentKernels(double length_threshold,
                               Array<CartoPolylineKernel<Point>>* kernels) {
    CHECK(length_threshold > 0.0);
    CHECK(kernels);

    if (kernels->empty()) return;

    Array<CartoPolylineKernel<Point>> merged_kernels;
    merged_kernels.push_back((*kernels)[0]);

    for (int i = 1; i < kernels->size(); ++i) {
        const CartoPolylineKernel<Point>& current = (*kernels)[i];
        CartoPolylineKernel<Point>& last = merged_kernels.back();
        double last_length = last.polyline.Length();
        double current_length = current.polyline.Length();

        if (last_length <= length_threshold ||
            current_length <= length_threshold) {
            std::string current_category = current.category;
            std::string last_category = last.category;

            MergeTwoAdjacentKernels(current, &last);
            last.category = last_length < current_length ? current_category
                                                         : last_category;
            continue;
        }
        merged_kernels.push_back(current);
    }

    *kernels = merged_kernels;
}

/**
 * Merge CartoPolylineKernels with identical endpoints based on width threshold.
 */
template <typename Point>
void MergeKernels(double width_threshold,
                  Array<CartoPolylineKernel<Point>>* kernels) {
    CHECK(kernels);
    CHECK(width_threshold > 0.0);

    if (kernels->empty()) return;

    bool merged;
    do {
        merged = false;

        std::unordered_map<Point, Array<int>> endpoint_to_polylines;
        for (auto&& [index, s] : enumerate(*kernels)) {
            if (s.polyline.empty()) continue;
            endpoint_to_polylines[s.polyline.front()].push_back(index);
            endpoint_to_polylines[s.polyline.back()].push_back(index);
        }

        Array<Point> degree2_endpoints;
        for (const auto& [p, indices] : endpoint_to_polylines) {
            if (indices.size() == 2) {
                degree2_endpoints.push_back(p);
            }
        }

        for (const Point& p : degree2_endpoints) {
            const auto& indices = endpoint_to_polylines[p];
            CHECK(indices.size() == 2);

            int idx1 = indices[0];
            int idx2 = indices[1];
            if (idx1 == idx2) continue;

            auto& s1 = (*kernels)[idx1];
            auto& s2 = (*kernels)[idx2];

            if (s1.category != s2.category) continue;
            if (s1.level != s2.level) continue;
            if (std::abs(s1.width - s2.width) > width_threshold) continue;

            bool can_merge1 = (s1.polyline.back() == p) &&
                              (s2.polyline.front() == p);
            bool can_merge2 = (s1.polyline.back() == p) &&
                              (s2.polyline.back() == p);
            bool can_merge3 = (s1.polyline.front() == p) &&
                              (s2.polyline.back() == p);
            bool can_merge4 = (s1.polyline.front() == p) &&
                              (s2.polyline.front() == p);

            Polyline<Point> merged_poly;
            if (can_merge1) {
                merged_poly = s1.polyline;
                merged_poly.insert(s2.polyline);
            } else if (can_merge2) {
                Polyline<Point> s2_rev = s2.polyline;
                s2_rev.Reverse();
                merged_poly = s1.polyline;
                merged_poly.insert(s2_rev);
            } else if (can_merge3) {
                merged_poly = s2.polyline;
                merged_poly.insert(s1.polyline);
            } else if (can_merge4) {
                Polyline<Point> s1_rev = s1.polyline;
                s1_rev.Reverse();
                merged_poly = s1_rev;
                merged_poly.insert(s2.polyline);
            } else {
                continue;
            }

            geometry::polyline_util::DouglasPeuckerSimplify(merged_poly,
                                                            1e-3,
                                                            &merged_poly);

            CartoPolylineKernel<Point> s = s1;
            s.polyline = merged_poly;
            s.width = (s.width + s2.width) / 2.0;

            int del_idx1 = std::max(idx1, idx2);
            int del_idx2 = std::min(idx1, idx2);
            kernels->erase(kernels->begin() + del_idx1);
            kernels->erase(kernels->begin() + del_idx2);
            kernels->push_back(s);

            merged = true;
            break;
        }
    } while (merged);
}

/**
 * Subdivide a CartoPolyline by inserting uniformly spaced vertices along its
 * path.
 *
 * The first and last vertices of the original polyline are always preserved.
 * Corresponding width values are linearly interpolated for all new vertices to
 * maintain continuous width distribution along the polyline.
 */
template <typename Point>
void Subdivide(const CartoPolyline<Point>& polyline, int n,
               CartoPolyline<Point>* output_polyline) {
    CHECK(n >= 2);
    CHECK(output_polyline);
    CHECK(polyline.polyline.size() == polyline.widths.size());

    *output_polyline = polyline;
    const Polyline<Point>& poly = polyline.polyline;
    const Array<double>& widths = polyline.widths;
    if (poly.size() < 2) return;

    Polyline<Point> output_poly;
    Array<double> output_widths;

    double length = 0.0;
    Array<double> seg_lengths;
    for (int i = 0; i + 1 < poly.size(); ++i) {
        double seg_len = Distance(poly[i], poly[i + 1]);
        seg_lengths.push_back(seg_len);
        length += seg_len;
    }
    if (length == 0.0) return;

    double step = length / (n - 1);
    CHECK(step > 0.0);

    double current_length = 0.0;
    int s = 0;

    output_poly.push_back(poly[0]);
    output_widths.push_back(widths[0]);
    for (int i = 1; i + 1 < n; ++i) {
        current_length += step;

        while (s < seg_lengths.size() && current_length > seg_lengths[s]) {
            current_length -= seg_lengths[s];
            ++s;
        }
        if (s >= seg_lengths.size()) break;

        CHECK(seg_lengths[s] > 0.0);
        double t = current_length / seg_lengths[s];
        Point p = poly[s] * (1 - t) + poly[s + 1].ToVector() * t;
        double w = widths[s] * (1 - t) + widths[s + 1] * t;
        output_poly.push_back(p);
        output_widths.push_back(w);
    }
    while (output_poly.size() < n) {
        output_poly.push_back(poly.back());
        output_widths.push_back(widths.back());
    }
    output_polyline->polyline = output_poly;
    output_polyline->widths = output_widths;
}

/**
 * See previous function.
 */
template <typename Point>
void Subdivide(const CartoPolyline<Point>& polyline, double resolution,
               CartoPolyline<Point>* output_polyline) {
    CHECK(resolution >= DBL_EPSILON);
    CHECK(polyline.polyline.size() == polyline.widths.size());
    CHECK(output_polyline);

    *output_polyline = polyline;

    double length = 0.0;
    Array<double> seg_lengths;
    for (int i = 0; i + 1 < polyline.polyline.size(); ++i) {
        length += Distance(polyline.polyline[i], polyline.polyline[i + 1]);
    }
    if (length == 0.0) return;

    double t = std::ceil(length / resolution);
    CHECK(t <= INT_MAX);
    int n = static_cast<int>(t);
    n = std::max(2, n + 1);
    Subdivide(polyline, n, output_polyline);
}

} // namespace carto_polyline_util
} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_CARTO_POLYLINE_UTIL_H_

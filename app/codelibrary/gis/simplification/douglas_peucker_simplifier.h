//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SIMPLIFICATION_DOUGLAS_PEUCKER_SIMPLIFIER_H_
#define CODELIBRARY_GIS_SIMPLIFICATION_DOUGLAS_PEUCKER_SIMPLIFIER_H_

#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/gis/carto_polyline.h"

namespace cl {
namespace gis {

/**
 * Douglas-Peucker based polyline simplifier.
 *
 * It simplifies the polyline with width-aware adaptive tolerance
 * (Douglas-Peucker algorithm variant).
 *
 * The simplification uses an adaptive threshold calculated from vertex width
 * (threshold = width_factor * vertex_width), capped by max_tolerance to avoid
 * excessive simplification for wide vertices.
 */
class DouglasPeuckerSimplifier {
public:
    /**
     * Constructor for DouglasPeuckerSimplifier.
     *
     * Paramters:
     *  max_tolerance - Maximum allowed simplification tolerance (upper limit
     *                  for adaptive threshold). Prevents over-simplification
     *                  for extremely wide vertices.
     *  width_factor  - Scaling factor for width-based adaptive tolerance
     *                  (typically 0.2).
     */
    explicit DouglasPeuckerSimplifier(double max_tolerance,
                                      double width_factor = 0.2)
        : max_tolerance_(max_tolerance), width_factor_(width_factor) {
        CHECK(max_tolerance_ > 0.0);
        CHECK(width_factor_ > 0.0 && width_factor_ < 1.0);
    }

    /**
     * Simplify the polyline while enforcing fixed vertex constraints: vertices
     * marked as fixed (is_fixed[i] = true) will retain their original positions
     * (not moved or removed) throughout the simplification process.
     */
    void Simplify(const CartoPolyline2D& polyline,
                  const Array<bool>& is_fixed,
                  CartoPolyline2D* result) const {
        CHECK(result);

        const RPolyline2D& poly = polyline.polyline;
        const Array<double>& widths = polyline.widths;
        CHECK(poly.size() == widths.size());
        CHECK(is_fixed.size() == poly.size());

        *result = polyline;

        if (poly.size() <= 2) return;

        Array<bool> is_remain(poly.size(), false);
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(0, poly.size() - 1));
        while (!q.empty()) {
            std::pair<int, int> p = q.front();
            q.pop();

            is_remain[p.first] = true;
            is_remain[p.second] = true;

            typename RPolyline2D::Segment seg(poly[p.first], poly[p.second]);
            int split = -1;
            double dis = 0.0;
            for (int i = p.first + 1; i < p.second; ++i) {
                if (is_fixed[i])  {
                    split = i;
                    break;
                }
                double d = Distance(poly[i], seg);
                if (d > dis) {
                    dis = d;
                    split = i;
                }
            }
            if (split == -1) continue;
            double w = widths[split];

            // Calculate adaptive threshold.
            w = std::min(width_factor_ * w, max_tolerance_);
            if (!is_fixed[split] && dis < w) continue;

            if (p.first < split) q.push(std::make_pair(p.first, split));
            if (split < p.second) q.push(std::make_pair(split, p.second));
        }

        RPolyline2D simplified_poly;
        Array<double> simplified_widths;
        for (int i = 0; i < poly.size(); ++i) {
            if (is_remain[i]) {
                simplified_poly.push_back(poly[i]);
                simplified_widths.push_back(widths[i]);
            }
        }
        result->polyline = simplified_poly;
        result->widths = simplified_widths;
    }

    /**
     * See previous one.
     */
    void Simplify(const CartoPolyline2D& polyline,
                  CartoPolyline2D* result) const {
        CHECK(result);

        Array<bool> is_fixed(polyline.polyline.size(), false);
        Simplify(polyline, is_fixed, result);
    }

private:
    // Maximum allowed simplification tolerance (upper limit for adaptive
    // threshold). Prevents over-simplification for extremely wide vertices.
    double max_tolerance_;

    // Scaling factor for width-based adaptive tolerance (typically 0.2).
    double width_factor_ = 0.2;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SIMPLIFICATION_DOUGLAS_PEUCKER_SIMPLIFIER_H_

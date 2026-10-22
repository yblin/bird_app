//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SMOOTHING_BIARC_SMOOTHER_H_
#define CODELIBRARY_GIS_SMOOTHING_BIARC_SMOOTHER_H_

#include <cfloat>
#include <random>

#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/gis/smoothing/smoothing_scorer.h"

namespace cl {
namespace gis {

/**
 * Use two circular arcs (or line segment) to approximate a polyline.
 *
 * This class uses geometric optimization to find the best-fitting biarc that
 * approximates a given polyline while respecting tangent constraints at both
 * endpoints.
 */
class BiarcSmoother {
public:
    using Arc = RArc2D;
    using ScoreType = std::pair<int, double>;

    /**
     * Construct an arc smoother with specified parameters
     *
     * Parameters:
     *   angle_threshold - Maximum allowed angular deviation (degrees) between
     *                     the fitted arc and the tangents.
     *   max_residual    - Maximum allowed perpendicular distance from polyline
     *                     points to the fitted arc.
     *   max_radius      - Maximum arc radius threshold. Arcs with radii
     *                     exceeding this value will be approximated as lines.
     */
    BiarcSmoother(double angle_threshold, double max_residual,
                  double max_radius)
        : angle_threshold_(angle_threshold), max_residual_(max_residual),
          max_radius_(max_radius) {
        CHECK(angle_threshold_ > 0.0);
        CHECK(max_residual_ > 0.0);
        CHECK(max_radius_ > 0.0);
    }

    /**
     * Fit a biarc to approximate the polyline[first, last) with tangent
     * constraints.
     *
     * Return the fitting score and smoothing score.
     */
    template <typename SmoothingScorer>
    ScoreType Smooth(const Array<RPoint2D>& polyline,
                     const RVector2D& start_tangent,
                     const RVector2D& end_tangent,
                     const SmoothingScorer& smoothing_scorer,
                     Arc* best_arc1,
                     Arc* best_arc2,
                     int* split_index) const {
        CHECK(best_arc1);
        CHECK(best_arc2);
        CHECK(split_index);

        ScoreType best_score(INT_MAX, DBL_MAX);
        if (polyline.size() < 3) return best_score;

        const int n_iterations = std::min(50, polyline.size());

        std::mt19937 random_engine;
        std::uniform_int_distribution<int> uniform(0, polyline.size() - 1);

        const RPoint2D& s = polyline.front();
        const RPoint2D& t = polyline.back();

        for (int itr = 0; itr < n_iterations; ++itr) {
            int a = uniform(random_engine);
            int b = uniform(random_engine);
            int c = uniform(random_engine);
            if (a == b || b == c || a == c) continue;
            int indices[3] = {a, b, c};
            std::sort(indices + 0, indices + 3);
            a = indices[0], b = indices[1], c = indices[2];
            const RPoint2D& pa = polyline[a];
            const RPoint2D& pb = polyline[b];
            const RPoint2D& pc = polyline[c];
            if (pb == pa || pb == pc) continue;

            Arc arc1(s, pa, pb), arc2(pb, pc, t);
            if (arc1.radius() > max_radius_) {
                arc1 = Arc(s, pb);
            }
            if (arc2.radius() > max_radius_) {
                arc2 = Arc(pb, t);
            }
            if (arc1.empty() || arc2.empty()) continue;

            if (!IsSmooth(start_tangent, arc1.start_tangent())) continue;
            if (!IsSmooth(end_tangent, arc2.end_tangent())) continue;
            if (!IsSmooth(arc1.end_tangent(), arc2.start_tangent())) continue;

            double f1 = GetFittingScore(arc1, polyline.begin(),
                                        polyline.begin() + b);
            if (f1 == DBL_MAX) continue;
            double f2 = GetFittingScore(arc2, polyline.begin() + b,
                                        polyline.end());
            if (f2 == DBL_MAX) continue;

            int s = smoothing_scorer(arc1.start_tangent(), start_tangent) +
                    smoothing_scorer(arc1.end_tangent(), arc2.start_tangent()) +
                    smoothing_scorer(arc2.end_tangent(), end_tangent);
            double f = f1 + f2;

            if (std::make_pair(s, f) < best_score) {
                best_score = {s, f};
                *best_arc1 = arc1;
                *best_arc2 = arc2;
                *split_index = b;
            }
        }

        return best_score;
    }

private:
    /**
     * Check if two vector is smooth.
     */
    bool IsSmooth(const RVector2D& v1, const RVector2D& v2) const {
        return Degree(v1, v2) <= angle_threshold_;
    }

    /**
     * Compute the fitting score between arc and a set of points.
     */
    template <typename Iterator>
    double GetFittingScore(const Arc& arc, Iterator first,
                           Iterator last) const {
        double dis = 0.0;
        for (auto p = first; p != last; ++p) {
            double d = Distance(arc, *p);
            if (d > max_residual_) return DBL_MAX;
            dis += d;
        }
        return dis;
    }

    double angle_threshold_;
    double max_residual_;
    double max_radius_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SMOOTHING_BIARC_SMOOTHER_H_

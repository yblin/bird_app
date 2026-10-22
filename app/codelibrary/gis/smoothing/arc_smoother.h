//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SMOOTHING_ARC_SMOOTHER_H_
#define CODELIBRARY_GIS_SMOOTHING_ARC_SMOOTHER_H_

#include <cfloat>
#include <random>

#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/gis/smoothing/smoothing_scorer.h"

namespace cl {
namespace gis {

/**
 * Use a single circular arc (or line segment) to approximate a polyline.
 *
 * This class uses geometric optimization to find the best-fitting arc that
 * approximates a given polyline while respecting tangent constraints at both
 * endpoints.
 */
class ArcSmoother {
public:
    using Point = RPoint2D;
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
    ArcSmoother(double angle_threshold, double max_residual, double max_radius)
        : angle_threshold_(angle_threshold), max_residual_(max_residual),
          max_radius_(max_radius) {
        CHECK(angle_threshold_ > 0.0);
        CHECK(max_residual_ > 0.0);
        CHECK(max_radius_ > 0.0);
    }

    /**
     * Fit an arc to approximate the polyline[first, last) with tangent
     * constraints.
     *
     * Return the fitting score and smoothing score.
     */
    template <typename SmoothingScorer>
    ScoreType Smooth(const Array<Point>& polyline,
                     const RVector2D& start_tangent,
                     const RVector2D& end_tangent,
                     const SmoothingScorer& smoothing_scorer,
                     Arc* best_arc) const {
        CHECK(best_arc);

        ScoreType best_score(INT_MAX, DBL_MAX);
        if (polyline.size() <= 1) return best_score;

        const int n_iterations = std::min(30, polyline.size());

        std::mt19937 random_engine;
        std::uniform_int_distribution<int> uniform(0, polyline.size() - 1);

        const Point& s = polyline.front();
        const Point& t = polyline.back();

        if (t != s) {
            if (IsSmooth(t - s, end_tangent) &&
                IsSmooth(t - s, start_tangent)) {
                ScoreType score;
                score.first = smoothing_scorer(t - s, start_tangent) +
                              smoothing_scorer(t - s, end_tangent);
                Arc line(s, t);
                score.second = GetFittingScore(line, polyline);
                if (score < best_score && score.second < DBL_MAX) {
                    best_score = score;
                    *best_arc = line;
                }
            }

            Arc arc1(s, start_tangent, t), arc2(s, t, end_tangent);
            if (arc1.radius() <= max_radius_ && IsSmooth(arc1.end_tangent(),
                                                         end_tangent)) {
                std::pair<int, double> score;
                score.first = smoothing_scorer(arc1.end_tangent(),
                                               end_tangent) +
                              smoothing_scorer(arc1.start_tangent(),
                                               start_tangent);
                score.second = GetFittingScore(arc1, polyline);
                if (score < best_score && score.second < DBL_MAX) {
                    best_score = score;
                    *best_arc = arc1;
                }
            }
            if (arc2.radius() <= max_radius_ && IsSmooth(arc2.start_tangent(),
                                                         start_tangent)) {
                std::pair<int, double> score;
                score.first = smoothing_scorer(arc2.start_tangent(),
                                               start_tangent) +
                              smoothing_scorer(arc2.end_tangent(), end_tangent);
                score.second = GetFittingScore(arc2, polyline);
                if (score < best_score && score.second < DBL_MAX) {
                    best_score = score;
                    *best_arc = arc2;
                }
            }
        }

        for (int itr = 0; itr < n_iterations; ++itr) {
            const Point& p = polyline[uniform(random_engine)];

            Arc arc(s, p, t);
            if (arc.empty()) continue;
            double fitting_score = GetFittingScore(arc, polyline);
            if (fitting_score == DBL_MAX) continue;

            RVector2D v1 = arc.start_tangent();
            RVector2D v2 = arc.end_tangent();
            if (arc.radius() > max_radius_) continue;

            if (!IsSmooth(v1, start_tangent) || !IsSmooth(v2, end_tangent))
                continue;

            int smooth_score = smoothing_scorer(v1, start_tangent) +
                               smoothing_scorer(v2, end_tangent);
            if (std::make_pair(smooth_score, fitting_score) < best_score) {
                best_score = {smooth_score, fitting_score};
                *best_arc = arc;
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
    double GetFittingScore(const Arc& arc, const Array<Point>& points) const {
        double dis = 0.0;
        for (auto p : points) {
            double d = Distance(arc, p);
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

#endif // CODELIBRARY_GIS_SMOOTHING_ARC_SMOOTHER_H_

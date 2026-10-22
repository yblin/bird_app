//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_SMOOTHER_H_
#define CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_SMOOTHER_H_

#include <queue>
#include <random>
#include <unordered_set>

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/generator_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/transform_2d.h"
#include "codelibrary/gis/smoothing/arc_smoother.h"
#include "codelibrary/gis/smoothing/biarc_smoother.h"
#include "codelibrary/gis/smoothing/piecewise_arc.h"
#include "codelibrary/point_cloud/fitting/least_median_squares.h"
#include "codelibrary/util/interval/interval_set.h"

namespace cl {
namespace gis {

/**
 * This class is designed to process a given polyline, typically a sequence of
 * points, and generate a piecewise arc that approximates the original shape.
 *
 * This class converts a polyline (sequence of points) into a piecewise arc
 * representation, while minimizing curvature discontinuities and ensuring the
 * approximation error stays within a specified tolerance.
 *
 * Commonly used for smoothing linear features like road centerlines and rivers
 * in GIS, navigation systems, cartography generalization, and CAD/CAM
 * applications.
 */
class PiecewiseArcSmoother {
    struct ConstraintPolyline {
        Array<RPoint2D> polyline;
        RVector2D start_tangent, end_tangent;
    };

public:
    using Arc = RArc2D;
    using Point = RPoint2D;
    using Piece = PiecewiseArc::Piece;
    using Vector = RVector2D;

    struct Parameter {
        // Distance threshold for filtering redundant polyline vertices.
        double distance_epsilon = 1e-6;

        // Smoothing angle threshold (degrees).
        // Determines the angular threshold between any two adjacent arcs:
        // - Smaller values enforce smoother transitions (more arcs)
        // - Larger values allow fewer arcs but may introduce sharper corners.
        double angle_threshold = 9.0;

        // Maximum residual distance.
        // Defines the maximum allowable deviation between the original polyline
        // and the fitted arcs:
        //  - Residual is calculated as the perpendicular distance from a point
        //    to the arc
        //  - Smaller values result in higher fidelity to the original shape.
        double max_residual = 0.3;

        // Maximum arc radius threshold.
        // Arcs with radii exceeding this value will be approximated as straight
        // line segments.
        double max_radius = 1000.0;

        // The input polyline will be densified so that no segment exceeds this
        // length.
        // This ensures smoother approximation and better arc fitting.
        double step = 0.3;

        // The distance threshold used to determine whether a line segment
        // should be smoothed.
        double no_smooth_line_length = 13.0;

        // The angle threshold (in degrees) used to determine whether sharp
        // corners are preserved.
        double no_smooth_angle = 120.0;

        // The maximum steps to backtrack when connecting adjacent arc pieces.
        int backoff_steps = 10;

        // The minimum number of steps (points) to form a initial piece.
        int min_initial_piece_steps = 5;
    };

    explicit PiecewiseArcSmoother(Parameter parameter)
        : parameter_(parameter),
          arc_smoother_(parameter.angle_threshold,
                        parameter.max_residual,
                        parameter.max_radius),
          biarc_smoother_(parameter.angle_threshold,
                          parameter.max_residual,
                          parameter.max_radius) {
        CHECK(parameter_.max_radius > 0.0);
        CHECK(parameter_.max_residual > 0.0);
        CHECK(parameter_.angle_threshold > 0.0);
        CHECK(parameter_.angle_threshold <= 90.0);
        CHECK(parameter_.max_radius > parameter_.max_residual);
        CHECK(parameter_.step > 0.0);
        CHECK(parameter_.backoff_steps > 0);
        CHECK(parameter_.no_smooth_line_length >
              2.0 * parameter_.step * parameter_.backoff_steps);
        CHECK(parameter_.min_initial_piece_steps >= 3);
    }

    /**
     * Extract initial arc pieces from polyline using RANSAC algorithm.
     * It serves as a robust initialization step for subsequent steps.
     *
     * Parameters:
     *  polyline       - the input point sequence to be approximated.
     *  start_tangent  - required tangent direction at the starting point.
     *  end_tangent    - required tangent direction at the ending point.
     *  piecewise_arc  - the output result.
     */
    void ExtractInitialPieces(const Array<Point>& polyline,
                              const Vector& start_tangent,
                              const Vector& end_tangent,
                              PiecewiseArc* piecewise_arc) const {
        CHECK(piecewise_arc);

        Initialize(polyline, piecewise_arc);
        piecewise_arc->start_tangent = start_tangent;
        piecewise_arc->end_tangent = end_tangent;

        IntervalSet<int> intervals;
        intervals.Insert(Interval<int>::Closed(0, piecewise_arc->points.size() -
                                                  1));
        for (auto piece : piecewise_arc->pieces) {
            intervals.Erase(Interval<int>::Closed(piece.start, piece.end));
        }

        std::queue<std::pair<int, int>> q;
        for (auto interval : intervals) {
            int left = interval.is_left_open() ? interval.lower_bound() + 1
                                               : interval.lower_bound();
            int right = interval.is_right_open() ? interval.upper_bound()
                                                 : interval.upper_bound() + 1;
            q.emplace(left, right);
        }
        while (!q.empty()) {
            auto p = q.front();
            q.pop();

            Piece piece;
            if (ExtractPiece(piecewise_arc->points, p.first, p.second,
                             &piece)) {
                piecewise_arc->pieces.push_back(piece);
                q.emplace(p.first, piece.start);
                q.emplace(piece.end + 1, p.second);
            }
        }
        std::sort(piecewise_arc->pieces.begin(), piecewise_arc->pieces.end());
    }

    /**
     * Extract initial arc pieces without tangent constraints.
     */
    void ExtractInitialPieces(const Array<Point>& polyline,
                              PiecewiseArc* piecewise_arc) const {
        Vector v;
        ExtractInitialPieces(polyline, v, v, piecewise_arc);
    }

   /**
    * Smooth a given polyline into a piecewise arc representation with specified
    * tangent constraints.
    *
    * This function processes the input polyline by:
    * 1. Splitting it into segments at vertices where the angle exceeds
    *    'no_smooth_angle'.
    * 2. Applying smoothing to each segment individually while respecting
    *    tangent constraints
    * 3. Merging the smoothed segments back into a single piecewise arc.
    *
    * Return true if all segments were successfully smoothed. Even if false is
    * returned, successfully smoothed segments will be included in the result
    * and unsuccessfully smoothed segments will be preserved as the original
    * polyline. The output piecewise_arc will always be valid but may not be
    * fully smoothed.
    */
    bool Smooth(const Array<Point>& polyline,
                const Vector& start_tangent,
                const Vector& end_tangent,
                PiecewiseArc* piecewise_arc) const {
        CHECK(piecewise_arc);

        piecewise_arc->clear();
        if (polyline.empty()) return false;

        RPolyline2D trimed_polyline;
        trimed_polyline.push_back(polyline.front());
        for (int i = 1; i < polyline.size(); ++i) {
            double dis = Distance(trimed_polyline.back(), polyline[i]);
            if (dis > parameter_.distance_epsilon) {
               trimed_polyline.push_back(polyline[i]);
            }
        }
        if (trimed_polyline.size() <= 1) return false;

        // Split the input polyline into pieces at vertices with sharp angles.
        Array<ConstraintPolyline> constraint_polylines;

        int start = 0;
        Vector vs = start_tangent;
        for (int i = 1; i + 1 < trimed_polyline.size(); ++i) {
            Vector v = trimed_polyline[i + 1] - trimed_polyline[i];
            Vector v_prev = trimed_polyline[i - 1] - trimed_polyline[i];
            if (Degree(v, v_prev) <= parameter_.no_smooth_angle) {
                ConstraintPolyline poly;
                poly.polyline.assign(trimed_polyline.begin() + start,
                                     trimed_polyline.begin() + i + 1);
                poly.start_tangent = vs;
                poly.end_tangent = -v_prev;
                constraint_polylines.push_back(poly);
                vs = v;
                start = i;
            }
        }
        ConstraintPolyline poly;
        poly.polyline = RPolyline2D(trimed_polyline.begin() + start,
                                    trimed_polyline.end());
        poly.start_tangent = vs;
        poly.end_tangent = end_tangent;
        constraint_polylines.push_back(poly);

        piecewise_arc->start_tangent = start_tangent;
        piecewise_arc->end_tangent = end_tangent;
        bool success = true;
        for (auto poly : constraint_polylines) {
            PiecewiseArc arc;
            if (!SmoothImplement(poly.polyline, poly.start_tangent,
                                 poly.end_tangent, &arc)) {
                // Fallback to original line segments if smoothing fails.
                arc.clear();
                arc.points = poly.polyline;
                for (int i = 0; i + 1 < arc.points.size(); ++i) {
                    Piece piece(arc.points[i], arc.points[i + 1]);
                    piece.start = i;
                    piece.end = i + 1;
                    arc.pieces.push_back(piece);
                }
                success = false;
            }

            piecewise_arc->Append(arc);
        }
        return success;
    }

    /**
     * Similar to the previous one, but uses the start and end tangents of the
     * polyline itself as constraints.
     */
    bool Smooth(const Array<Point>& polyline,
                PiecewiseArc* piecewise_arc) const {
        if (polyline.size() < 2) return false;
        RVector2D s = polyline[1] - polyline[0];
        RVector2D t = polyline.back() - polyline[polyline.size() - 2];
        return Smooth(polyline, s, t, piecewise_arc);
    }

    /**
     * Replace arcs in a piecewise-arc with straight lines if they meet
     * geometric error and smoothness constraints.
     *
     * Intended as a post-processing step after Smooth() function.
     */
    void ReplaceArcsWithLines(PiecewiseArc* arc) const {
        for (int i = 0; i < arc->pieces.size(); ++i) {
            auto& piece = arc->pieces[i];
            if (piece.orientation() == 0 || piece.fixed) continue;

            RVector2D vs, vt;
            if (i == 0)
                vs = arc->start_tangent;
            else
                vs = arc->pieces[i - 1].end_tangent();
            if (i + 1 == arc->pieces.size())
                vt = arc->end_tangent;
            else
                vt = arc->pieces[i + 1].start_tangent();

            RLine2D line(piece.start_point(), piece.end_point());
            if (IsSmooth(line.direction(), vs) &&
                IsSmooth(line.direction(), vt)) {
                double dis = 0.0;
                for (int i = piece.start; i <= piece.end; ++i) {
                    dis = std::max(dis, Distance(arc->points[i], line));
                }
                if (dis > parameter_.max_residual) continue;
                Piece new_piece(piece.start_point(), piece.end_point());
                new_piece.start = piece.start;
                new_piece.end = piece.end;
                piece = new_piece;
            }
        }
    }

    /**
     * Check if all the piece of piecewise-arc is smooth.
     */
    bool IsSmooth(const PiecewiseArc& arc) const {
        if (arc.pieces.empty()) return false;

        if (!is_left_smooth(arc, 0)) return false;
        if (!is_right_smooth(arc, arc.pieces.size() - 1)) return false;

        for (int i = 0; i + 1 < arc.pieces.size(); ++i) {
            if (!is_right_smooth(arc, i)) return false;
        }
        return true;
    }

    /**
     * Check if the left side of the i-th piece is smooth.
     */
    bool is_left_smooth(const PiecewiseArc& arc, int i) const {
        CHECK(i >= 0 && i < arc.pieces.size());

        if (i == 0) {
            return arc.pieces[i].start_point() == arc.points.front() &&
                   IsSmooth(arc.pieces[i].start_tangent(), arc.start_tangent);
        }

        return arc.pieces[i].start_point() == arc.pieces[i - 1].end_point() &&
               IsSmooth(arc.pieces[i - 1].end_tangent(),
                        arc.pieces[i].start_tangent());
    }

    /**
     * Check if the right side of the i-th piece is smooth.
     */
    bool is_right_smooth(const PiecewiseArc& arc, int i) const {
        CHECK(i >= 0 && i < arc.pieces.size());

        if (i + 1 == arc.pieces.size()) {
            return arc.pieces[i].end_point() == arc.points.back() &&
                   IsSmooth(arc.pieces[i].end_tangent(), arc.end_tangent);
        }

        return arc.pieces[i].end_point() == arc.pieces[i + 1].start_point() &&
               IsSmooth(arc.pieces[i].end_tangent(),
                        arc.pieces[i + 1].start_tangent());
    }

private:
    /**
     * Compute the fitting score between arc and a set of points.
     */
    template <class Model, typename Iterator>
    static double GetFittingScore(const Model& model, Iterator first,
                                  Iterator last) {
        double dis = 0.0;
        for (auto p = first; p != last; ++p) {
            dis += Distance(model, *p);
        }
        return dis;
    }

    /**
     * Check if two vector is smooth.
     */
    bool IsSmooth(const Vector& v1, const Vector& v2) const {
        return Degree(v1, v2) <= parameter_.angle_threshold;
    }

    /**
     * Smooth the given polyline.
     */
    bool SmoothImplement(const Array<Point>& polyline,
                         const Vector& start_tangent,
                         const Vector& end_tangent,
                         PiecewiseArc* piecewise_arc) const {
        CHECK(piecewise_arc);

        if (polyline.size() <= 1) return false;

        if (polyline.size() == 2) {
            Vector v = polyline[1] - polyline[0];
            if (IsSmooth(v, start_tangent) && IsSmooth(v, end_tangent)) {
                piecewise_arc->clear();
                piecewise_arc->start_tangent = start_tangent;
                piecewise_arc->end_tangent = end_tangent;
                piecewise_arc->points = polyline;
                Piece piece(polyline[0], polyline[1]);
                piece.start = 0;
                piece.end = 1;
                piecewise_arc->pieces.push_back(piece);
                return true;
            }
        }

        ExtractInitialPieces(polyline, start_tangent, end_tangent,
                             piecewise_arc);

        Array<Piece>& pieces = piecewise_arc->pieces;
        bool has_fixed_pieces = false;
        for (auto& piece : pieces) {
            if (piece.fixed) {
                has_fixed_pieces = true;
                break;
            }
        }
        if (!has_fixed_pieces && pieces.size() <= 2) {
            if (SmoothOnePiece(piecewise_arc)) return true;
            if (SmoothTwoPieces(piecewise_arc)) return true;
        }
        if (pieces.empty()) return false;

        // Solve line-arc, arc-line, line-line patterns.
        ConnectPiecesByExtending(piecewise_arc);

        // Solve line-arc-line and line-arc-arc-line patterns.
        for (int i = 0; i < pieces.size(); ++i) {
            if (pieces[i].orientation() != 0) continue;

            bool all_optimized = is_right_smooth(*piecewise_arc, i);
            int left = i;
            int right = -1;
            for (int j = i + 1; j < pieces.size(); ++j) {
                if (!is_left_smooth(*piecewise_arc, j) ||
                    !is_right_smooth(*piecewise_arc, j))
                    all_optimized = false;

                if (pieces[j].orientation() == 0) {
                    right = j;
                    break;
                }
            }
            if (right != -1 && !all_optimized) {
                if (SmoothLineArcLinePieces(left, right, 0, piecewise_arc)) {
                    i = left + 1;
                } else if (SmoothLineArcLinePieces(left, right, 1,
                                                   piecewise_arc)) {
                    i = left + 2;
                }
            }
        }
        piecewise_arc->Trim();
        if (piecewise_arc->pieces.empty()) return false;

        // Connect the leftmost piece to the start point.
        SmoothLeftPiece(piecewise_arc);
        // Connect the rightmost piece to the end point.
        SmoothRightPiece(piecewise_arc);
        piecewise_arc->Trim();

        // Solve genernal pattern.
        Array<Piece> new_pieces;
        for (int i = 0; i + 1 < pieces.size(); ++i) {
            if (is_right_smooth(*piecewise_arc, i)) continue;

            Array<Piece> ps;
            if (ConnectPieces(i, i + 1, 0, piecewise_arc, &ps)) {
                new_pieces.insert(ps);
            } else if (ConnectPieces(i, i + 1, 1, piecewise_arc, &ps)) {
                new_pieces.insert(ps);
            }
        }
        pieces.insert(new_pieces);
        std::sort(pieces.begin(), pieces.end());
        piecewise_arc->Trim();

        return IsSmooth(*piecewise_arc);
    }

    /**
     * Densify the input polyline so that no segment exceeds the 'step'.
     */
    void Initialize(const Array<Point>& polyline,
                    PiecewiseArc* piecewise_arc) const {
        piecewise_arc->clear();

        Array<Point>& points = piecewise_arc->points;
        for (int i = 0; i + 1 < polyline.size(); ++i) {
            Vector v = (polyline[i + 1] - polyline[i]).normalized();
            double dis = Distance(polyline[i], polyline[i + 1]);
            if (dis >= parameter_.no_smooth_line_length) {
                int start = points.size();
                for (int j = (i != 0); j < parameter_.backoff_steps; ++j) {
                    points.push_back(polyline[i] + v * parameter_.step * j);
                }
                for (int j = parameter_.backoff_steps - 1; j >= 0; --j) {
                    points.push_back(polyline[i + 1] - v * parameter_.step * j);
                }
                int end = points.size() - 1;
                Piece piece(points[start], points[end]);
                piece.start = start;
                piece.end = end;
                piece.fixed = true;
                piecewise_arc->pieces.push_back(piece);
            } else {
                int n = std::max(1, static_cast<int>(dis / parameter_.step));
                double s = dis / (n + 1);
                for (int j = (i != 0); j < n; ++j) {
                    points.push_back(polyline[i] + v * s * j);
                }
                points.push_back(polyline[i + 1]);
            }
        }
        if (polyline.size() == 1) {
            points.push_back(polyline.front());
        }
    }

    /**
     * RANSAC to find a arc piece that can fit most continuous points in
     * [start, end).
     */
    bool ExtractPiece(const Array<Point>& points, int start, int end,
                      Piece* piece) const {
        CHECK(piece);

        const int n_iterations = 50;

        int size = end - start;
        if (size < parameter_.min_initial_piece_steps) return 0;

        std::mt19937 random_engine;
        std::uniform_int_distribution<int> uniform(0, size - 1);

        int max_length = parameter_.min_initial_piece_steps - 1;

        for (int itr = 0; itr < n_iterations; ++itr) {
            int a = uniform(random_engine) + start;
            int b = uniform(random_engine) + start;
            int c = uniform(random_engine) + start;
            if (a == b || b == c || a == c) continue;
            const Point& pa = points[a];
            const Point& pb = points[b];
            const Point& pc = points[c];
            if (pa == pb || pb == pc || pa == pc) continue;

            for (int k = 0; k < 2; ++k) {
                if (k == 0) {
                    Point c = Circumcenter(pa, pb, pc);
                    double r1 = Distance(c, pa);
                    double r2 = Distance(c, pb);
                    double r3 = Distance(c, pc);
                    double radius = std::max(r1, std::max(r2, r3));
                    if (radius > parameter_.max_radius) continue;
                    RCircle2D circle(c, radius);
                    FindLongestPiece(circle, points, start, end, true,
                                     &max_length, piece);
                } else {
                    RLine2D line(pa, pc);
                    FindLongestPiece(line, points, start, end, false,
                                     &max_length, piece);
                }
            }
        }

        return max_length >= parameter_.min_initial_piece_steps;
    }

    /**
     * Compute the piece to fit the circle.
     */
    void GetPiece(const RCircle2D& circle, const Array<Point>& points,
                  int start, int end,
                  Piece* piece) const {
        int i = start, j = end - 1;
        Point p1 = geometry::Project(points[i], circle);
        if (i == 0) {
            p1 = points.front();
        }

        Point p2 = geometry::Project(points[(i + j) / 2], circle);
        Point p3 = geometry::Project(points[j], circle);
        if (j == points.size() - 1) {
            p3 = points.back();
        }

        *piece = Arc(p1, p2, p3);
        piece->start = i;
        piece->end = j;
    }

    /**
     * Compute the piece to fit the line.
     */
    void GetPiece(const RLine2D& line, const Array<Point>& points,
                  int start, int end, Piece* piece) const {
        int i = start, j = end - 1;
        Point p1 = geometry::Project(points[i], line);
        if (i == 0) {
            p1 = points.front();
        }

        Point p2 = geometry::Project(points[j], line);
        if (j == points.size() - 1) {
            p2 = points.back();
        }

        *piece = Arc(p1, p2);
        piece->start = i;
        piece->end = j;
    }

    /**
     * Find the longest range covered by the model.
     */
    template <class Model>
    void FindLongestPiece(const Model& model, const Array<Point>& points,
                          int start, int end,
                          bool refined,
                          int* max_length,
                          Piece* best_piece) const {
        const double residual = parameter_.max_residual * 0.5;
        const double eps = std::sqrt(DBL_EPSILON);

        int size = end - start;
        Array<bool> on_model(size, false);

        for (int i = 0; i < size; ++i) {
            on_model[i] = Distance(points[i + start], model) <= residual;
        }
        if (start == 0) {
            on_model[0] = Distance(points[0], model) <= eps;
        }
        if (end == points.size()) {
            on_model.back() = Distance(points.back(), model) <= eps;
        }

        int s = -1;
        int length = 0;
        for (int i = start; i < end; ++i) {
            if (on_model[i - start]) {
                if (s == -1) {
                    s = i;
                }
                ++length;
            } else {
                if (length > *max_length) {
                    if (!refined) {
                        // Important! Lines fitted by RANSAC may not well. It
                        // should be refined by LeastMedianSquares.
                        RLine2D line;
                        Array<Point> ps(points.begin() + s,
                                        points.begin() + s + length);
                        point_cloud::LeastMedianSquares(ps, &line);
                        FindLongestPiece(line, points, start, end, true,
                                         max_length, best_piece);
                    } else {
                        Piece piece;
                        GetPiece(model, points, s, s + length, &piece);
                        *max_length = length;
                        *best_piece = piece;
                    }
                }
                length = 0;
                s = -1;
            }
        }
        if (length > *max_length) {
            Piece piece;
            GetPiece(model, points, s, s + length, &piece);
            *max_length = length;
            *best_piece = piece;
        }
    }

    /**
     * Smooth one piece.
     */
    bool SmoothOnePiece(PiecewiseArc* piecewise_arc) const {
        CHECK(piecewise_arc);

        Piece piece;
        auto score = arc_smoother_.Smooth(piecewise_arc->points,
                                          piecewise_arc->start_tangent,
                                          piecewise_arc->end_tangent,
                                          smoothing_scorer_,
                                          &piece);
        if (score.first == INT_MAX) {
            return false;
        }

        piece.start = 0;
        piece.end = piecewise_arc->points.size() - 1;

        piecewise_arc->pieces.clear();
        piecewise_arc->pieces.push_back(piece);
        return true;
    }

    /**
     * Smooth two piece.
     */
    bool SmoothTwoPieces(PiecewiseArc* piecewise_arc) const {
        CHECK(piecewise_arc);

        int split;
        Piece piece1, piece2;
        auto score = biarc_smoother_.Smooth(piecewise_arc->points,
                                            piecewise_arc->start_tangent,
                                            piecewise_arc->end_tangent,
                                            smoothing_scorer_,
                                            &piece1,
                                            &piece2,
                                            &split);
        if (score.first == INT_MAX) {
            return false;
        }

        piece1.start = 0;
        piece1.end = split;
        piece2.start = split;
        piece2.end = piecewise_arc->points.size() - 1;

        piecewise_arc->pieces.clear();
        piecewise_arc->pieces.push_back(piece1);
        piecewise_arc->pieces.push_back(piece2);
        return true;
    }

    /**
     * Connect adjacent pieces by extending them.
     */
    void ConnectPiecesByExtending(PiecewiseArc* piecewise_arc) const {
        for (int i = 0; i + 1 < piecewise_arc->pieces.size(); ++i) {
            if (is_right_smooth(*piecewise_arc, i)) continue;

            const Array<Point>& points = piecewise_arc->points;
            Piece& piece1 = piecewise_arc->pieces[i];
            Piece& piece2 = piecewise_arc->pieces[i + 1];
            int start = piece1.start, end = piece2.end;
            Arc arc1, arc2;

            if (piece1.orientation() == 0 && piece2.orientation() == 0) {
                RLine2D line1(piece1.start_point(), piece1.end_point());
                RLine2D line2(piece2.start_point(), piece2.end_point());
                if (!IsSmooth(line1.direction(), line2.direction()))
                    continue;

                Point p;
                if (!geometry::Cross(line1, line2, &p)) continue;

                arc1 = Arc(piece1.start_point(), p);
                arc2 = Arc(p, piece2.end_point());
            } else if (piece1.orientation() == 0 && piece2.orientation() != 0) {
                RLine2D line1(piece1.start_point(), piece1.end_point());
                RCircle2D circle2 = piece2.GetCircle();
                RSegment2D res;
                if (!geometry::Cross(line1, circle2, &res)) continue;

                double dis1 = Distance(piece1.end_point(), res.lower_point());
                double dis2 = Distance(piece1.end_point(), res.upper_point());
                Point p = dis1 < dis2 ? res.lower_point() : res.upper_point();

                double radiance = Radian(p - circle2.center());
                Vector tangent = circle2.tangent(radiance);
                if (piece2.orientation() < 0) {
                    tangent = -tangent;
                }

                arc1 = Arc(piece1.start_point(), p);
                arc2 = Arc(p, tangent, piece2.end_point());
            } else if (piece1.orientation() != 0 && piece2.orientation() == 0) {
                RCircle2D circle1 = piece1.GetCircle();
                RLine2D line2(piece2.start_point(), piece2.end_point());
                RSegment2D res;
                if (!geometry::Cross(circle1, line2, &res)) continue;

                double dis1 = Distance(piece1.end_point(), res.lower_point());
                double dis2 = Distance(piece1.end_point(), res.upper_point());
                Point p = dis1 < dis2 ? res.lower_point() : res.upper_point();

                double radiance = Radian(p - circle1.center());
                Vector tangent = circle1.tangent(radiance);
                if (piece1.orientation() < 0) {
                    tangent = -tangent;
                }

                arc1 = Arc(piece1.start_point(), p, tangent);
                arc2 = Arc(p, piece2.end_point());
            } else {
                RCircle2D circle1 = piece1.GetCircle();
                RCircle2D circle2 = piece2.GetCircle();
                RSegment2D res;
                if (!geometry::Cross(circle1, circle2, &res)) continue;

                double dis1 = Distance(piece1.end_point(), res.lower_point());
                double dis2 = Distance(piece1.end_point(), res.upper_point());
                Point p = dis1 < dis2 ? res.lower_point() : res.upper_point();

                double radiance1 = Radian(p - circle1.center());
                double radiance2 = Radian(p - circle2.center());
                Vector tangent1 = circle1.tangent(radiance1);
                Vector tangent2 = circle2.tangent(radiance2);
                if (piece1.orientation() < 0) {
                    tangent1 = -tangent1;
                }
                if (piece2.orientation() < 0) {
                    tangent2 = -tangent2;
                }

                arc1 = Arc(piece1.start_point(), p, tangent1);
                arc2 = Arc(p, tangent2, piece2.end_point());
            }

            if (arc1.empty() || arc2.empty()) continue;
            if (!IsSmooth(arc1.start_tangent(), piece1.start_tangent()))
                continue;
            if (!IsSmooth(arc2.end_tangent(), piece2.end_tangent()))
                continue;
            if (!IsSmooth(arc1.end_tangent(), arc2.start_tangent()))
                continue;

            Point p = arc1.end_point();
            int best = -1;
            double min_dis = DBL_MAX;
            for (int i = start + 1; i < end; ++i) {
                double dis = Distance(points[i], p);
                if (dis < min_dis) {
                    min_dis = dis;
                    best = i;
                }
            }
            if (min_dis > parameter_.max_residual) continue;
            bool valid = true;
            for (int i = start; i <= best; ++i) {
                if (Distance(points[i], arc1) > parameter_.max_residual) {
                    valid = false;
                    break;
                }
            }
            for (int i = best; i <= end; ++i) {
                if (Distance(points[i], arc2) > parameter_.max_residual) {
                    valid = false;
                    break;
                }
            }
            if (!valid) continue;

            piece1 = arc1;
            piece1.start = start;
            piece1.end = best;

            piece2 = arc2;
            piece2.start = best;
            piece2.end = end;
        }

        piecewise_arc->Trim();
    }

    /**
     * Try connecting two line pieces using one or two arc.
     */
    bool SmoothLineArcLinePieces(int left, int right, int mode,
                                 PiecewiseArc* piecewise_arc) const {
        CHECK(left < right);
        CHECK(piecewise_arc);

        Piece& piece1 = piecewise_arc->pieces[left];
        Piece& piece2 = piecewise_arc->pieces[right];
        CHECK(piece1.orientation() == 0);
        CHECK(piece2.orientation() == 0);

        const Array<Point>& points = piecewise_arc->points;

        int start = piece1.start;
        int end = piece2.end;

        const int backoff = parameter_.backoff_steps;
        int left_start = std::max(start, piece1.end - backoff);
        int right_start = std::min(end, piece2.start + backoff);

        int left_extend = std::min(piece1.end + backoff, piece2.end);
        int right_extend = std::max(piece2.start - backoff, piece1.start);

        ArcSmoother::ScoreType best_score = {INT_MAX, DBL_MAX};
        Array<Piece> best_pieces;

        RLine2D left_line(piece1.start_point(), piece1.end_point());
        RLine2D right_line(piece2.start_point(), piece2.end_point());
        Vector vs = left_line.direction(), vt = right_line.direction();

        double f1 = GetFittingScore(left_line, points.begin() + start,
                                    points.begin() + left_start);
        double f = GetFittingScore(right_line, points.begin() + right_start,
                                   points.begin() + end + 1);
        for (int i = left_start; i <= left_extend; ++i) {
            double dis = Distance(points[i], left_line);
            if (dis > parameter_.max_residual) break;
            f1 += dis;

            Point q1 = geometry::Project(points[i], left_line);
            double f2 = f;

            Array<Point> ps;
            for (int j = i; j <= right_start; ++j) {
                ps.push_back(points[j]);
            }
            if (!ps.empty()) ps.front() = q1;

            for (int j = right_start; j >= right_extend; --j) {
                if (j <= i) break;

                double dis = Distance(points[j], right_line);
                if (dis > parameter_.max_residual) break;
                f2 += dis;

                Point q2 = geometry::Project(points[j], right_line);
                ps.back() = q2;

                Array<Piece> pieces;
                auto score = SmoothKernel(ps, i, j, vs, vt, mode, &pieces);
                ps.pop_back();

                if (score.first < INT_MAX) {
                    score.second += (f1 + f2);
                    if (score < best_score) {
                        best_pieces = pieces;
                        best_score = score;
                    }
                }
            }
        }

        if (best_pieces.empty()) {
            return false;
        }

        piece1 = Arc(piece1.start_point(), best_pieces.front().start_point());
        piece1.start = start;
        piece1.end = best_pieces.front().start;

        piece2 = Arc(best_pieces.back().end_point(), piece2.end_point());
        piece2.start = best_pieces.back().end;
        piece2.end = end;

        auto it_start = piecewise_arc->pieces.begin() + left + 1;
        auto it_end = piecewise_arc->pieces.begin() + right;
        piecewise_arc->pieces.erase(it_start, it_end);
        piecewise_arc->pieces.insert(piecewise_arc->pieces.begin() + left + 1,
                                     best_pieces.begin(), best_pieces.end());

        return true;
    }

    /**
     * Connect the left-most piece to the start point.
     */
    bool SmoothLeftPiece(PiecewiseArc* piecewise_arc) const {
        CHECK(!piecewise_arc->pieces.empty());

        if (is_left_smooth(*piecewise_arc, 0)) return true;

        Piece& piece = piecewise_arc->pieces.front();
        int left = std::max(1, piece.start - parameter_.backoff_steps);
        int right = std::min(piece.start + parameter_.backoff_steps, piece.end);

        const Array<Point>& points = piecewise_arc->points;

        Piece best_piece;
        ArcSmoother::ScoreType best_score = {INT_MAX, DBL_MAX};
        for (int i = left; i <= right; ++i) {
            Point q = geometry::Project(points[i], piece);

            Arc arc;
            Array<Point> ps(points.begin(), points.begin() + i + 1);
            ps.back() = q;
            auto s = arc_smoother_.Smooth(ps, piecewise_arc->start_tangent,
                                          piece.tangent(q),
                                          smoothing_scorer_,
                                          &arc);

            if (s < best_score) {
                best_piece = arc;
                best_piece.start = 0;
                best_piece.end = i;
                best_score = s;
            }
        }

        if (best_score.first < INT_MAX) {
            int end = piece.end;
            if (piece.orientation() == 0) {
                piece = Arc(best_piece.end_point(), piece.end_point());
            } else {
                piece = Arc(best_piece.end_point(), piece.end_point(),
                            piece.end_tangent());
            }
            piece.start = best_piece.end;
            piece.end = end;

            piecewise_arc->pieces.insert(piecewise_arc->pieces.begin(),
                                         best_piece);
            return true;
        }

        // Try biarc.
        Piece best_piece1, best_piece2;
        best_score = {INT_MAX, DBL_MAX};
        for (int i = left; i <= right; ++i) {
            Point q = geometry::Project(points[i], piece);

            Arc arc1, arc2;
            int split;
            Array<Point> ps(points.begin(), points.begin() + i + 1);
            ps.back() = q;
            auto s = biarc_smoother_.Smooth(ps, piecewise_arc->start_tangent,
                                            piece.tangent(q),
                                            smoothing_scorer_,
                                            &arc1,
                                            &arc2,
                                            &split);
            if (s < best_score) {
                best_score = s;

                best_piece1 = arc1;
                best_piece1.start = 0;
                best_piece1.end = split;

                best_piece2 = arc2;
                best_piece2.start = split;
                best_piece2.end = i;
            }
        }

        if (best_score.first < INT_MAX) {
            int end = piece.end;
            if (piece.orientation() == 0) {
                piece = Arc(best_piece2.end_point(), piece.end_point());
            } else {
                piece = Arc(best_piece2.end_point(), piece.end_point(),
                            piece.end_tangent());
            }
            piece.start = best_piece2.end;
            piece.end = end;

            piecewise_arc->pieces.insert(piecewise_arc->pieces.begin(),
                                         best_piece2);
            piecewise_arc->pieces.insert(piecewise_arc->pieces.begin(),
                                         best_piece1);
            return true;
        }
        return false;
    }

    /**
     * Connect the right-most piece to the end point.
     */
    bool SmoothRightPiece(PiecewiseArc* piecewise_arc) const {
        if (is_right_smooth(*piecewise_arc, piecewise_arc->pieces.size() - 1))
            return true;

        const Array<Point>& points = piecewise_arc->points;

        Piece& piece = piecewise_arc->pieces.back();
        int left = std::max(piece.start, piece.end - parameter_.backoff_steps);
        int right = std::min(piece.end + parameter_.backoff_steps,
                             points.size() - 2);

        Piece best_piece;
        ArcSmoother::ScoreType best_score = {INT_MAX, DBL_MAX};
        for (int i = left; i <= right; ++i) {
            Point q = geometry::Project(points[i], piece);

            Arc arc;
            Array<Point> ps(points.begin() + i, points.end());
            ps.front() = q;
            auto s = arc_smoother_.Smooth(ps, piece.tangent(q),
                                          piecewise_arc->end_tangent,
                                          smoothing_scorer_,
                                          &arc);

            if (s < best_score) {
                best_piece = arc;
                best_piece.start = i;
                best_piece.end = points.size() - 1;
                best_score = s;
            }
        }

        if (best_score.first < INT_MAX) {
            int start = piece.start;
            if (piece.orientation() == 0) {
                piece = Arc(piece.start_point(), best_piece.start_point());
            } else {
                piece = Arc(piece.start_point(), piece.start_tangent(),
                            best_piece.start_point());
            }
            piece.start = start;
            piece.end = best_piece.start;

            piecewise_arc->pieces.push_back(best_piece);
            return true;
        }

        // Try biarc.
        Piece best_piece1, best_piece2;
        best_score = {INT_MAX, DBL_MAX};
        for (int i = left; i <= right; ++i) {
            Point q = geometry::Project(points[i], piece);

            Arc arc1, arc2;
            int split;
            Array<Point> ps(points.begin() + i, points.end());
            ps.front() = q;
            auto s = biarc_smoother_.Smooth(ps, piece.tangent(q),
                                            piecewise_arc->end_tangent,
                                            smoothing_scorer_,
                                            &arc1,
                                            &arc2,
                                            &split);

            if (s < best_score) {
                best_score = s;

                best_piece1 = arc1;
                best_piece1.start = i;
                best_piece1.end = split + i;

                best_piece2 = arc2;
                best_piece2.start = split + i;
                best_piece2.end = points.size() - 1;
            }
        }

        if (best_score.first < INT_MAX) {
            int start = piece.start;
            if (piece.orientation() == 0) {
                piece = Arc(piece.start_point(), best_piece1.start_point());
            } else {
                piece = Arc(piece.start_point(), piece.start_tangent(),
                            best_piece1.start_point());
            }
            piece.start = start;
            piece.end = best_piece1.start;

            piecewise_arc->pieces.push_back(best_piece1);
            piecewise_arc->pieces.push_back(best_piece2);
            return true;
        }
        return false;
    }

    /**
     * Connect two pieces by one or two arcs.
     */
    bool ConnectPieces(int left, int right, int mode,
                       PiecewiseArc* piecewise_arc,
                       Array<Piece>* new_pieces) const {
        CHECK(left < right);
        CHECK(piecewise_arc);
        CHECK(new_pieces);

        new_pieces->clear();

        const Array<Point>& points = piecewise_arc->points;
        Piece& piece1 = piecewise_arc->pieces[left];
        Piece& piece2 = piecewise_arc->pieces[right];

        int start = piece1.start;
        int end = piece2.end;

        const int backoff = parameter_.backoff_steps;
        int left_start = std::max(start + 1, piece1.end - backoff);
        int right_start = std::min(end - 1, piece2.start + backoff);

        int left_extend = std::min(piece1.end + backoff, piece2.end - 1);
        int right_extend = std::max(piece2.start - backoff, piece1.start + 1);

        ArcSmoother::ScoreType best_score = {INT_MAX, DBL_MAX};

        double f1 = GetFittingScore(piece1, points.begin() + start,
                                    points.begin() + left_start);
        double f = GetFittingScore(piece2, points.begin() + right_start,
                                   points.begin() + end + 1);
        for (int i = left_start; i <= left_extend; ++i) {
            double dis = Distance(points[i], piece1);
            if (dis > parameter_.max_residual) break;
            f1 += dis;

            Point q1 = geometry::Project(points[i], piece1);
            Vector vs = piece1.tangent(q1);

            double f2 = f;

            Array<Point> ps;
            for (int j = i; j <= right_start; ++j) {
                ps.push_back(points[j]);
            }
            if (!ps.empty()) ps.front() = q1;

            for (int j = right_start; j >= right_extend; --j) {
                if (j <= i) break;

                double dis = Distance(points[j], piece2);
                if (dis > parameter_.max_residual) break;
                f2 += dis;

                Point q2 = geometry::Project(points[j], piece2);
                Vector vt = piece2.tangent(q2);
                ps.back() = q2;

                Array<Piece> pieces;
                auto score = SmoothKernel(ps, i, j, vs, vt, mode, &pieces);
                ps.pop_back();

                if (score.first < INT_MAX) {
                    score.second += (f1 + f2);
                    if (score < best_score) {
                        *new_pieces = pieces;
                        best_score = score;
                    }
                }
            }
        }

        if (new_pieces->empty()) return false;

        if (piece1.orientation() == 0) {
            piece1 = Arc(piece1.start_point(),
                         new_pieces->front().start_point());
        } else {
            piece1 = Arc(piece1.start_point(), piece1.start_tangent(),
                         new_pieces->front().start_point());
        }
        piece1.start = start;
        piece1.end = new_pieces->front().start;

        if (piece2.orientation() == 0) {
            piece2 = Arc(new_pieces->back().end_point(), piece2.end_point());
        } else {
            piece2 = Arc(new_pieces->back().end_point(), piece2.end_point(),
                         piece2.end_tangent());
        }
        piece2.start = new_pieces->back().end;
        piece2.end = end;

        return true;
    }

    /**
     * Use arc-smoother (mode = 0) or biarc-smoother (mode = 1) to get new
     * pieces from points.
     */
    ArcSmoother::ScoreType SmoothKernel(const Array<Point>& points,
                                        int start,
                                        int end,
                                        const Vector& start_tangent,
                                        const Vector& end_tangent,
                                        int mode,
                                        Array<Piece>* pieces) const {
        CHECK(end == start + points.size() - 1);

        pieces->clear();

        if (mode == 0) {
            Piece piece;
            auto score = arc_smoother_.Smooth(points,
                                              start_tangent,
                                              end_tangent,
                                              smoothing_scorer_,
                                              &piece);
            piece.start = start;
            piece.end = start + points.size() - 1;
            pieces->push_back(piece);
            return score;
        } else {
            Piece piece1, piece2;
            int split;
            auto score = biarc_smoother_.Smooth(points,
                                                start_tangent,
                                                end_tangent,
                                                smoothing_scorer_,
                                                &piece1,
                                                &piece2,
                                                &split);
            piece1.start = start;
            piece1.end = start + split;
            piece2.start = start + split;
            piece2.end = start + points.size() - 1;
            pieces->push_back(piece1);
            pieces->push_back(piece2);

            return score;
        }
    }

    Parameter parameter_;

    // Used to compute smoothing score.
    SmoothingScorer smoothing_scorer_;

    // Arc smoother for connection.
    ArcSmoother arc_smoother_;

    // Biarc smoother for connection.
    BiarcSmoother biarc_smoother_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_SMOOTHER_H_

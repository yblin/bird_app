//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_H_
#define CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_H_

#include <cfloat>

#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/arc_2d.h"

namespace cl {
namespace gis {

/*
 * The PiecewiseArc class represents a smooth, continuous approximation of a
 * polyline formed by a series of connected points in 2D space.
 *
 * The class stores the original polyline points and a collection of arcs
 * (pieces) that together form the piecewise arc, making it suitable for tasks
 * requiring curve fitting and smoothing.
 */
struct PiecewiseArc {
    /**
     * Arc piece.
     */
    struct Piece : public RArc2D {
        Piece() = default;

        Piece(const RArc2D& arc)
            : RArc2D(arc) {}

        /**
         * Construct a line from s to t.
         */
        Piece(const RPoint2D& s, const RPoint2D& t)
            : RArc2D(s, t) {}

        /**
         * Constructs an arc passing through three points.
         */
        Piece(const RPoint2D& s, const RPoint2D& m, const RPoint2D& t)
            : RArc2D(s, m, t) {}

        bool empty() const {
            return start >= end || RArc2D::empty();
        }

        bool operator <(const Piece& rhs) const {
            return start < rhs.start;
        }

        // Indices representing the start and end point of this piece in the
        // original polyline.
        int start = -1, end = -1;
        // Used in PiecewiseArcSmoother.
        // When set to true, the type of this piece (line or arc) will NOT be
        // changed by smoothing algorithms.
        bool fixed = false;
    };

    // Points constituting the original polyline that is being approximated.
    Array<RPoint2D> points;

    // Collection of arcs, each arc piece approximating a part of the polyline.
    Array<Piece> pieces;

    // Start and end control vectors.
    RVector2D start_tangent, end_tangent;

    PiecewiseArc() = default;

    /**
     * Trim the empty pieces.
     */
    void Trim() {
        int n_pieces = 0;
        for (int i = 0; i < pieces.size(); ++i) {
            if (!pieces[i].empty()) {
                pieces[n_pieces++] = pieces[i];
            }
        }
        pieces.resize(n_pieces);
    }

    /**
     * Append another PiecewiseArc to the end of this arc.
     */
    void Append(const PiecewiseArc& arc) {
        if (arc.empty()) return;
        if (this->empty()) {
            *this = arc;
            return;
        }

        int offset = points.size();
        if (points.back() == arc.points.front()) {
            points.insert(points.end(), arc.points.begin() + 1,
                          arc.points.end());
            --offset;
        } else {
            points.insert(arc.points);
        }

        int old_pieces = pieces.size();
        pieces.insert(arc.pieces);
        for (int i = old_pieces; i < pieces.size(); ++i) {
            pieces[i].start += offset;
            pieces[i].end += offset;
        }

        end_tangent = arc.end_tangent;
    }

    void clear() {
        points.clear();
        pieces.clear();
    }

    bool empty() const {
        return points.empty();
    }
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SMOOTHING_PIECEWISE_ARC_H_

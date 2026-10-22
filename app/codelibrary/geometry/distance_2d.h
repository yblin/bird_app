//
// Copyright 2012-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Distance between 2D geometric kernel objects.
//

#ifndef CODELIBRARY_GEOMETRY_DISTANCE_2D_H_
#define CODELIBRARY_GEOMETRY_DISTANCE_2D_H_

#include <algorithm>
#include <cfloat>
#include <cmath>

#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/arc_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/triangle_2d.h"

namespace cl {

/**
 * Return the squared Euclidean distance between two points.
 */
template <typename T>
double SquaredDistance(const Point2D<T>& p1, const Point2D<T>& p2) {
    double t1 = p1.x - p2.x;
    double t2 = p1.y - p2.y;
    return t1 * t1 + t2 * t2;
}

/**
 * Return the Euclidean distance between two points.
 */
template <typename T>
double Distance(const Point2D<T>& p1, const Point2D<T>& p2) {
    return std::sqrt(SquaredDistance(p1, p2));
}

/**
 * Return the signed Euclidean distance from point to line.
 */
template <typename T>
double SignedDistance(const Point2D<T>& p, const Line2D<T>& line) {
    double x1 = line.direction().x;
    double y1 = line.direction().y;
    double x2 = p.x - line.point().x;
    double y2 = p.y - line.point().y;
    double norm = std::sqrt(x1 * x1 + y1 * y1);

    if (norm == 0.0) return Distance(p, line.point());
    return (x1 * y2 - x2 * y1) / norm;
}
template <typename T>
double SignedDistance(const Line2D<T>& line, const Point2D<T>& p) {
    return SignedDistance(p, line);
}

/**
 * Return the Euclidean distance from point to line.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Line2D<T>& line) {
    return std::fabs(SignedDistance(p, line));
}
template <typename T>
double Distance(const Line2D<T>& line, const Point2D<T>& p) {
    return Distance(p, line);
}

/**
 * Return the Euclidean distance from point to segment.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Segment2D<T>& segment) {
    if (segment.lower_point() == segment.upper_point()) {
        return Distance(p, segment.lower_point());
    }

    RVector2D v0(segment.upper_point().x - segment.lower_point().x,
                 segment.upper_point().y - segment.lower_point().y);
    RVector2D v1(p.x - segment.lower_point().x,
                 p.y - segment.lower_point().y);
    double t1 = v0.dot(v1);
    if (t1 < 0.0) return Distance(p, segment.lower_point());

    double t2 = v0.dot(v0);
    if (t2 <= t1) return Distance(p, segment.upper_point());

    double b = t1 / t2;
    RPoint2D pb(b * v0.x + segment.lower_point().x,
                b * v0.y + segment.lower_point().y);
    RPoint2D pt(p.x, p.y);

    return Distance(pt, pb);
}
template <typename T>
double Distance(const Segment2D<T>& segment, const Point2D<T>& p) {
    return Distance(p, segment);
}

/**
 * Return the Euclidean distance from point to box.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Box2D<T>& box) {
    Array<Segment2D<T>> segs = {
        Segment2D<T>(Point2D<T>(box.x_min(), box.y_min()),
                     Point2D<T>(box.x_min(), box.y_max())),
        Segment2D<T>(Point2D<T>(box.x_max(), box.y_min()),
                     Point2D<T>(box.x_max(), box.y_max())),
        Segment2D<T>(Point2D<T>(box.x_min(), box.y_min()),
                     Point2D<T>(box.x_max(), box.y_min())),
        Segment2D<T>(Point2D<T>(box.x_min(), box.y_max()),
                     Point2D<T>(box.x_max(), box.y_max())),
    };
    double dis = DBL_MAX;
    for (auto s : segs) {
        double d = Distance(p, s);
        if (d < dis) {
            dis = d;
        }
    }
    return dis;
}
template <typename T>
double Distance(const Box2D<T>& box, const Point2D<T>& p) {
    return Distance(p, box);
}

/**
 * Return the Euclidean distance from point to triangle's boundary.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Triangle2D<T>& triangle) {
    CHECK(!triangle.empty());

    double dis = DBL_MAX;
    for (int i = 0; i < 3; ++i) {
        dis = std::min(dis, Distance(p, triangle.edge(i)));
    }
    return dis;
}
template <typename T>
double Distance(const Triangle2D<T>& triangle, const Point2D<T>& p) {
    return Distance(p, triangle);
}

/**
 * Return the Euclidean distance from point to polygon's boundary.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Polygon2D<T>& polygon) {
    double dis = DBL_MAX;

    for (int i = 0; i < polygon.size(); ++i) {
        dis = std::min(dis, Distance(p, polygon.edge(i)));
    }
    return dis;
}
template <typename T>
double Distance(const Polygon2D<T>& polygon, const Point2D<T>& p) {
    return Distance(p, polygon);
}

/**
 * Return the Euclidean distance from point to multi-polygon's boundary.
 */
template <typename T>
double Distance(const Point2D<T>& p, const MultiPolygon2D<T>& polygon) {
    double dis = DBL_MAX;

    for (const auto& boundary : polygon.boundaries()) {
        dis = std::min(dis, Distance(p, boundary.polygon));
    }
    return dis;
}
template <typename T>
double Distance(const MultiPolygon2D<T>& polygon, const Point2D<T>& p) {
    return Distance(p, polygon);
}

/**
 * Return the Euclidean distance from point to circle.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Circle2D<T>& circle) {
    double distance_to_center = Distance(p, circle.center());
    return std::fabs(distance_to_center - circle.radius());
}
template <typename T>
double Distance(const Circle2D<T>& circle, const Point2D<T>& p) {
    return Distance(p, circle);
}

/**
 * Return the Euclidean distance from point to circular arc.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Arc2D<T>& arc) {
    if (arc.orientation() == 0) {
        return Distance(p, Segment2D<T>(arc.start_point(), arc.end_point()));
    }

    double distance_to_center = Distance(p, arc.center());
    double radian = Radian(p - arc.center());
    if (arc.IsAngleBetween(radian)) {
        return std::fabs(distance_to_center - arc.radius());
    } else {
        double distance_to_start = Distance(p, arc.start_point());
        double distance_to_end = Distance(p, arc.end_point());
        return std::min(distance_to_start, distance_to_end);
    }
}
template <typename T>
double Distance(const Arc2D<T>& arc, const Point2D<T>& p) {
    return Distance(p, arc);
}

/**
 * Return the Euclidean Distance from 'p' to polyline segment.
*/
template <typename T>
double Distance(const Point2D<T>& p,
                const typename Polyline<Point2D<T>>::Segment& seg) {
    Segment2D<T> s(seg.source, seg.target);
    return Distance(p, s);
}
template <typename T>
double Distance(const typename Polyline<Point2D<T>>::Segment& seg,
                const Point2D<T>& p) {
    return Distance(p, seg);
}

/**
 * Return the Euclidean distance from point to polyline.
 */
template <typename T>
double Distance(const Point2D<T>& p, const Polyline<Point2D<T>>& polyline) {
    double dis = DBL_MAX;

    if (polyline.size() == 1) {
        return Distance(p, polyline.front());
    }

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        dis = std::min(dis, Distance(p, polyline.segment(i)));
    }
    return dis;
}
template <typename T>
double Distance(const Polyline<Point2D<T>>& polyline, const Point2D<T>& p) {
    return Distance(p, polyline);
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_DISTANCE_2D_H_

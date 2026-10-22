//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Distance between 3D geometric kernel objects.
//

#ifndef CODELIBRARY_GEOMETRY_DISTANCE_3D_H_
#define CODELIBRARY_GEOMETRY_DISTANCE_3D_H_

#include <cfloat>
#include <cmath>

#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/segment_3d.h"

namespace cl {

/**
 * Return the squared Euclidean distance between two 3D points.
 */
template <typename T>
double SquaredDistance(const Point3D<T>& p1, const Point3D<T>& p2) {
    double t1 = p1.x - p2.x;
    double t2 = p1.y - p2.y;
    double t3 = p1.z - p2.z;
    return t1 * t1 + t2 * t2 + t3 * t3;
}

/**
 * Return the Euclidean distance between two 3D points.
 */
template <typename T>
double Distance(const Point3D<T>& p1, const Point3D<T>& p2) {
    return std::sqrt(SquaredDistance(p1, p2));
}

/**
 * Return the signed Euclidean distance from point to plane.
 */
template <typename T>
double SignedDistance(const Point3D<T>& p, const Plane3D<T>& plane) {
    double a = plane.normal().x;
    double b = plane.normal().y;
    double c = plane.normal().z;
    double d = -a * plane.point().x - b * plane.point().y - c * plane.point().z;

    double norm = std::sqrt(a * a + b * b + c * c);
    if (norm == 0.0) return Distance(p, plane.point());

    return (a * p.x + b * p.y + c * p.z + d) / norm;
}
template <typename T>
double SignedDistance(const Plane3D<T>& plane, const Point3D<T>& p) {
    return SignedDistance(p, plane);
}

/**
 * Return the Euclidean distance from point to plane.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Plane3D<T>& plane) {
    return std::fabs(SignedDistance(p, plane));
}
template <typename T>
double Distance(const Plane3D<T>& plane, const Point3D<T>& p) {
    return Distance(p, plane);
}

/**
 * Return the Euclidean distance from point to segment.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Segment3D<T>& segment) {
    if (segment.lower_point() == segment.upper_point()) {
        return Distance(p, segment.lower_point());
    }

    RVector3D v0(segment.upper_point().x - segment.lower_point().x,
                 segment.upper_point().y - segment.lower_point().y,
                 segment.upper_point().z - segment.lower_point().z);
    RVector3D v1(p.x - segment.lower_point().x,
                 p.y - segment.lower_point().y,
                 p.z - segment.lower_point().z);

    double t1 = v0.dot(v1);
    if (t1 < 0.0) return Distance(p, segment.lower_point());

    double t2 = v0.dot(v0);
    if (t2 <= t1) return Distance(p, segment.upper_point());

    double b = t1 / t2;
    RPoint3D pb(b * v0.x + segment.lower_point().x,
                b * v0.y + segment.lower_point().y,
                b * v0.z + segment.lower_point().z);
    RPoint3D pt(p.x, p.y, p.z);

    return Distance(pt, pb);
}
template <typename T>
double Distance(const Segment3D<T>& seg, const Point3D<T>& p) {
    return Distance(p, seg);
}

/**
 * Return the Euclidean distance from point to line.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Line3D<T>& l) {
    RVector3D v0(l.direction().x, l.direction().y, l.direction().z);
    RVector3D v1(p.x - l.point().x, p.y - l.point().y, p.z - l.point().z);

    double t1 = v0.dot(v1);
    double t2 = v0.dot(v0);
    if (t2 == 0.0) return Distance(p, l.point());

    double b = t1 / t2;
    RPoint3D pb(b * v0.x + l.point().x, b * v0.y + l.point().y,
                b * v0.z + l.point().z);

    return Distance(RPoint3D(p.x, p.y, p.z), pb);
}
template <typename T>
double Distance(const Line3D<T>& line, const Point3D<T>& p) {
    return Distance(p, line);
}

/**
 * Return the Euclidean Distance from 'p' to polyline segment.
*/
template <typename T>
double Distance(const Point3D<T>& p,
                const typename Polyline<Point3D<T>>::Segment& seg) {
    Segment3D<T> s(seg.source, seg.target);
    return Distance(p, s);
}
template <typename T>
double Distance(const typename Polyline<Point3D<T>>::Segment& seg,
                const Point3D<T>& p) {
    return Distance(p, seg);
}

/**
 * Return the Euclidean distance from point to polyline.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Polyline<Point3D<T>>& polyline) {
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
double Distance(const Polyline<Point3D<T>>& polyline, const Point3D<T>& p) {
    return Distance(p, polyline);
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_DISTANCE_3D_H_

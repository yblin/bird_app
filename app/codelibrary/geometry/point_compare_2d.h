//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_
#define CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_

#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/predicate_2d.h"

namespace cl {
namespace geometry {

/**
 * Exact dot comparison for points.
 *
 * lhs < rhs means lhs is before rhs along line (p1, p2).
 */
template <typename T>
class PointDotCompare2D {
public:
    PointDotCompare2D(const Point2D<T>& p1, const Point2D<T>& p2)
        : point1_(p1), point2_(p2) {}

    explicit PointDotCompare2D(const Line2D<T>& line)
        : point1_(line.point1()), point2_(line.point2()) {}

    explicit PointDotCompare2D(const Segment2D<T>& seg)
        : point1_(seg.lower_point()), point2_(seg.upper_point()) {}

    bool operator()(const Point2D<T>& lhs, const Point2D<T>& rhs) const {
        return DotProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Compute the dot product value: (q - p)(s - p).
     */
    template <typename T1>
    static T1 DotProduct(const T1& px, const T1& py, const T1& qx, const T1& qy,
                         const T1& sx, const T1& sy);

    /**
     * Exact compare (q - p)(s - p) with (q - p)(r - p).
     */
    static int DotProductCompare(const Point2D<T>& p, const Point2D<T>& q,
                                 const Point2D<T>& s, const Point2D<T>& r) {
        IntervalFloat det1 = DotProductCompareCompute<IntervalFloat>(
            p.x, p.y, q.x, q.y, s.x, s.y, r.x, r.y);

        if (det1.lower() > 0.0)
            return +1;
        if (det1.upper() < 0.0)
            return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0)
            return 0;

        ExactFloat det2 = DotProductCompareCompute<ExactFloat>(
            p.x, p.y, q.x, q.y, s.x, s.y, r.x, r.y);
        return det2.sign();
    }

    template <typename Tfp>
    static Tfp DotProductCompareCompute(T px, T py, T qx, T qy, T sx, T sy,
                                        T rx, T ry);

    Point2D<T> point1_, point2_;
};

/**
 * Exact cross compare for points.
 *
 * lhs < rhs means lhs is before rhs along the normal vector of line (p1, p2).
 */
template <typename T>
class PointCrossCompare2D {
public:
    PointCrossCompare2D(const Point2D<T>& p1, const Point2D<T>& p2)
        : point1_(p1), point2_(p2) {}

    bool operator()(const Point2D<T>& lhs, const Point2D<T>& rhs) const {
        return CrossProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Exact compare (q - p) x (s - p) with (q - p) x (r - p).
     */
    static int CrossProductCompare(const Point2D<T>& p, const Point2D<T>& q,
                                   const Point2D<T>& s, const Point2D<T>& r) {
        IntervalFloat det1 = CrossProductCompareCompute<IntervalFloat>(
            p.x, p.y, q.x, q.y, s.x, s.y, r.x, r.y);

        if (det1.lower() > 0.0)
            return +1;
        if (det1.upper() < 0.0)
            return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0)
            return 0;

        ExactFloat det2 = CrossProductCompareCompute<ExactFloat>(
            p.x, p.y, q.x, q.y, s.x, s.y, r.x, r.y);
        return det2.sign();
    }

    template <typename Tfp>
    static Tfp CrossProductCompareCompute(T px, T py, T qx, T qy, T sx, T sy,
                                          T rx, T ry);

    Point2D<T> point1_, point2_;
};

}  // namespace geometry
}  // namespace cl

CL_INTERVAL_FLOAT_BEGIN
template <typename T>
template <typename T1>
T1 cl::geometry::PointDotCompare2D<T>::DotProduct(const T1& px, const T1& py,
                                                  const T1& qx, const T1& qy,
                                                  const T1& sx, const T1& sy) {
    return (qx - px) * (sx - px) + (qy - py) * (sy - py);
}

template <typename T>
template <typename Tfp>
Tfp cl::geometry::PointDotCompare2D<T>::DotProductCompareCompute(T px, T py,
                                                                 T qx, T qy,
                                                                 T sx, T sy,
                                                                 T rx, T ry) {
    ScopedRounding<Tfp> rounding;
    (void) rounding;

    Tfp det1 =
        DotProduct<Tfp>(Tfp(px), Tfp(py), Tfp(qx), Tfp(qy), Tfp(sx), Tfp(sy));
    Tfp det2 =
        DotProduct<Tfp>(Tfp(px), Tfp(py), Tfp(qx), Tfp(qy), Tfp(rx), Tfp(ry));
    return det1 - det2;
}

template <typename T>
template <typename Tfp>
Tfp cl::geometry::PointCrossCompare2D<T>::CrossProductCompareCompute(
    T px, T py, T qx, T qy, T sx, T sy, T rx, T ry) {
    ScopedRounding<Tfp> rounding;
    (void) rounding;

    Tfp det1 = OrientationDeterminant(Tfp(px), Tfp(py), Tfp(qx), Tfp(qy),
                                      Tfp(sx), Tfp(sy));
    Tfp det2 = OrientationDeterminant(Tfp(px), Tfp(py), Tfp(qx), Tfp(qy),
                                      Tfp(rx), Tfp(ry));
    return det1 - det2;
}
CL_INTERVAL_FLOAT_END

#endif  // CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_

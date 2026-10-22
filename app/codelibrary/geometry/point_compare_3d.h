//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_
#define CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/math/number/exact_float.h"
#include "codelibrary/math/number/interval_float.h"

namespace cl {
namespace geometry {

/**
 * Exact dot comparison for points.
 *
 * lhs < rhs means lhs is before rhs along line (p1, p2).
 */
template <typename T>
class PointDotCompare3D {
public:
    PointDotCompare3D(const Point3D<T>& p1, const Point3D<T>& p2)
        : point1_(p1), point2_(p2) {}

    bool operator()(const Point3D<T>& lhs, const Point3D<T>& rhs) const {
        return DotProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Compute the dot product value: (q - p)(s - p).
     */
    template <typename T1>
    static T1 DotProduct(const T1& px, const T1& py, const T1& pz, const T1& qx,
                         const T1& qy, const T1& qz, const T1& sx, const T1& sy,
                         const T1& sz);

    /**
     * Exact compare (q - p)(s - p) with (q - p)(r - p).
     */
    static int DotProductCompare(const Point3D<T>& p, const Point3D<T>& q,
                                 const Point3D<T>& s, const Point3D<T>& r) {
        IntervalFloat det1 = DotProductCompareCompute<IntervalFloat>(
            p.x, p.y, p.z, q.x, q.y, q.z, s.x, s.y, s.z, r.x, r.y, r.z);

        if (det1.lower() > 0.0)
            return +1;
        if (det1.upper() < 0.0)
            return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0)
            return 0;

        ExactFloat det2 = DotProductCompareCompute<ExactFloat>(
            p.x, p.y, p.z, q.x, q.y, q.z, s.x, s.y, s.z, r.x, r.y, r.z);

        return det2.sign();
    }

    template <typename Tfp>
    static Tfp DotProductCompareCompute(T px, T py, T pz, T qx, T qy, T qz,
                                        T sx, T sy, T sz, T rx, T ry, T rz);

    Point3D<T> point1_, point2_;
};

}  // namespace geometry
}  // namespace cl

CL_INTERVAL_FLOAT_BEGIN
template <typename T>
template <typename T1>
T1 cl::geometry::PointDotCompare3D<T>::DotProduct(const T1& px, const T1& py,
                                                  const T1& pz, const T1& qx,
                                                  const T1& qy, const T1& qz,
                                                  const T1& sx, const T1& sy,
                                                  const T1& sz) {
    return (qx - px) * (sx - px) + (qy - py) * (sy - py) +
           (qz - pz) * (sz - pz);
}

template <typename T>
template <typename Tfp>
Tfp cl::geometry::PointDotCompare3D<T>::DotProductCompareCompute(
    T px, T py, T pz, T qx, T qy, T qz, T sx, T sy, T sz, T rx, T ry, T rz) {
    ScopedRounding<Tfp> rounding;
    (void) rounding;

    Tfp det1 = DotProduct(Tfp(px), Tfp(py), Tfp(pz), Tfp(qx), Tfp(qy), Tfp(qz),
                          Tfp(sx), Tfp(sy), Tfp(sz));
    Tfp det2 = DotProduct(Tfp(px), Tfp(py), Tfp(pz), Tfp(qx), Tfp(qy), Tfp(qz),
                          Tfp(rx), Tfp(ry), Tfp(rz));
    return det1 - det2;
}

CL_INTERVAL_FLOAT_END

#endif  // CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_

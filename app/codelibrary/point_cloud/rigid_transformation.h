//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_RIGID_TRANSFORMATION_H_
#define CODELIBRARY_POINT_CLOUD_RIGID_TRANSFORMATION_H_

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/math/matrix/matrix.h"
#include "codelibrary/math/matrix/vector.h"

namespace cl {
namespace point_cloud {

/**
 * This function calculates the rigid transformation (rotation and translation)
 * that aligns the source point set to the target point set, given matching
 * pairs of points.
 *
 * It uses the Umeyama algorithm for point set alignment.
 */
template <typename T>
bool RigidTransformation(const Array<Point3D<T>>& source,
                         const Array<Point3D<T>>& target,
                         RMatrix3* rotation,
                         RVector3D* translation) {
    CHECK(source.size() == target.size());
    CHECK(rotation);
    CHECK(translation);

    if (source.size() < 3) return false;

    const int n = source.size();

    RMatrix source_m(3, n), target_m(3, n);
    for (int i = 0; i < n; ++i) {
        source_m(0, i) = source[i].x;
        source_m(1, i) = source[i].y;
        source_m(2, i) = source[i].z;
        target_m(0, i) = target[i].x;
        target_m(1, i) = target[i].y;
        target_m(2, i) = target[i].z;
    }
    RMatrix4 t = Eigen::umeyama(source_m, target_m);
    *rotation = t.block<3, 3>(0, 0);
    translation->x = t(0, 3);
    translation->y = t(1, 3);
    translation->z = t(2, 3);
    return true;
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_RIGID_TRANSFORMATION_H_

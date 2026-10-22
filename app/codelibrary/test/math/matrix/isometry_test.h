//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_MATRIX_ISOMETRY_TEST_H_
#define CODELIBRARY_TEST_MATH_MATRIX_ISOMETRY_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/matrix/isometry.h"

namespace cl {
namespace test {

TEST(Isometry3Test, SE3Map) {
    RVector twist = make_vector(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
    RMatrix4 s = se3::Exp(twist);
    RVector res = se3::Log(s);
    ASSERT_EQ_RANGE(twist.begin(), twist.end(), res.begin(), res.end());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_MATRIX_ISOMETRY_TEST_H_

//
// Copyright 2025 Yangbin Lin & Kai Jia. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_INTERVAL_FLOAT_TEST_H_
#define CODELIBRARY_TEST_MATH_INTERVAL_FLOAT_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/predicate_2d.h"
#include "codelibrary/math/number/interval_float.h"
#include "codelibrary/test/math/number/interval_float_test_data.h"

namespace cl {
namespace test {

namespace {

using IntervalFloatCheckCb =
    std::function<void(int, int, int, const IntervalFloat&)>;

CL_INTERVAL_FLOAT_BEGIN;
inline static void IntervalFloatTestRounding(
        const IntervalFloatCheckCb& fn_check) {
    ScopedRounding<IntervalFloat> rounding;
    (void)rounding;

    IntervalFloat v0 =
        (IntervalFloat(0.1) + IntervalFloat(0.1) + IntervalFloat(0.1)) *
            IntervalFloat(10.0 / 3.0) -
        IntervalFloat(1.0);

    fn_check(-1, v0.sign(), 0, v0);
    fn_check(-2, v0.lower() <= 0, 1, v0);
    fn_check(-3, v0.upper() >= 0, 1, v0);
}

/**
 * Run the interval float test cases
 * fn_check a callback for result checking
 *
 * It is recommended to run this testcase in your program at startup since
 * interval float correctness is not always guaranteed on all platforms.
 */
inline static void RunIntervalFloatTest(const IntervalFloatCheckCb& fn_check) {
    IntervalFloatTestRounding(fn_check);

    using interval_float_data::TEST_CASES;
    using interval_float_data::TestCase;
    using geometry::OrientationDeterminant;

    // run cases extracted from a real-world usecase
    for (int i = 0; i < TEST_CASES.size(); ++i) {
        const TestCase& tc = TEST_CASES[i];

        // Compute orientation determinant using IntervalFloat
        IntervalFloat det_interval =
            OrientationDeterminant(IntervalFloat(tc.p[0]),
                                   IntervalFloat(tc.p[1]),
                                   IntervalFloat(tc.q[0]),
                                   IntervalFloat(tc.q[1]),
                                   IntervalFloat(tc.r[0]),
                                   IntervalFloat(tc.r[1]));

        // Compute orientation determinant using ExactFloat
        ExactFloat det_exact = OrientationDeterminant(ExactFloat(tc.p[0]),
                                                      ExactFloat(tc.p[1]),
                                                      ExactFloat(tc.q[0]),
                                                      ExactFloat(tc.q[1]),
                                                      ExactFloat(tc.r[0]),
                                                      ExactFloat(tc.r[1]));

        int interval_sign = det_interval.sign();
        int exact_sign = det_exact.sign();

        // Check that interval sign is consistent with exact sign.
        if (interval_sign != 0) {
            fn_check(i, interval_sign, exact_sign, det_interval);
        }
    }
}

CL_INTERVAL_FLOAT_END;

} // namespace

TEST(IntervalFloatTest, CompareWithExactOnData) {
    auto fn_check = [](int case_num,
                       int interval_sign,
                       int exact_sign,
                       const IntervalFloat& interval_val) {
        ASSERT_EQ(interval_sign, exact_sign)
            << "Test case " << case_num << ": interval sign " << interval_sign
            << " inconsistent with exact sign " << exact_sign << " (interval: ["
            << interval_val.lower() << ", " << interval_val.upper() << "])";
    };

    RunIntervalFloatTest(fn_check);
}

}  // namespace test
}  // namespace cl

#endif  // CODELIBRARY_TEST_MATH_INTERVAL_FLOAT_TEST_H_

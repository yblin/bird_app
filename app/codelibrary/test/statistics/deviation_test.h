//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_STATISTICS_DEVIATION_TEST_H_
#define CODELIBRARY_TEST_STATISTICS_DEVIATION_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/statistics/deviation.h"

namespace cl {
namespace test {

TEST(DeviationTest, StandardDeviation) {
    double a[] = { 2, 4, 4, 4, 5, 5, 7, 9 };

    CHECK_EQ(StandardDeviation(a + 0, a + 8), 2);
}

TEST(DeviationTest, MedianAbsoluteDeviation) {
    int a[] = { 2, 2, 3, 4, 14 };

    CHECK_EQ(MedianAbsoluteDeviation(a + 0, a + 5), 1);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_STATISTICS_DEVIATION_TEST_H_

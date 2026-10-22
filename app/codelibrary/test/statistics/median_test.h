//
// Copyright 2015-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_STATISTICS_MEDIAN_TEST_H_
#define CODELIBRARY_TEST_STATISTICS_MEDIAN_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/statistics/median.h"

namespace cl {
namespace test {

TEST(MedianTest, Median) {
    int a[] = {1, 2, 3, 4, 5, 6, 7};

    CHECK_EQ(Median(a + 0, a + 7), 4);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_STATISTICS_MEDIAN_TEST_H_

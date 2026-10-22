//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_STATISTICS_RANKING_TEST_H_
#define CODELIBRARY_TEST_STATISTICS_RANKING_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/statistics/ranking.h"

namespace cl {
namespace test {

TEST(RankingTest, CompetitionRanking) {
    double a[] = { 1, 4, 2, 2 };

    Array<int> rank;

    CompetitionRanking(a + 0, a + 4, &rank);

    CHECK_EQ(rank[0], 0);
    CHECK_EQ(rank[1], 3);
    CHECK_EQ(rank[2], 1);
    CHECK_EQ(rank[3], 1);
}

TEST(RankingTest, DenseRanking) {
    double a[] = { 1, 4, 2, 2 };

    Array<int> rank;

    DenseRanking(a + 0, a + 4, &rank);

    CHECK_EQ(rank[0], 0);
    CHECK_EQ(rank[1], 2);
    CHECK_EQ(rank[2], 1);
    CHECK_EQ(rank[3], 1);
}

TEST(RankingTest, OrdinalRanking) {
    double a[] = { 1, 4, 2, 2 };

    Array<int> rank;

    OrdinalRanking(a + 0, a + 4, &rank);

    CHECK_EQ(rank[0], 0);
    CHECK_EQ(rank[1], 3);
    CHECK_EQ(rank[2], 1);
    CHECK_EQ(rank[3], 2);
}

TEST(RankingTest, FractionalRanking) {
    double a[] = { 1, 4, 2, 2 };

    Array<double> rank;

    FractionalRanking(a + 0, a + 4, &rank);

    CHECK_EQ(rank[0], 0);
    CHECK_EQ(rank[1], 3);
    CHECK_EQ(rank[2], 1.5);
    CHECK_EQ(rank[3], 1.5);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_STATISTICS_RANKING_TEST_H_

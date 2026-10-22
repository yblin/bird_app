//
// Copyright 2014-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_COVARIANCE_H_
#define CODELIBRARY_STATISTICS_COVARIANCE_H_

#include <iterator>

namespace cl {

/**
 * Get the covariance between two random variables X [first1, last1) and Y
 * [first2, last2).
 *
 * Covariance is a measure of how much two random variables change together.
 */
template <typename Iterator>
double Covariance(Iterator first1, Iterator last1,
                  Iterator first2, Iterator last2) {
    auto n1 = std::distance(first1, last1);
    auto n2 = std::distance(first2, last2);
    CHECK(n1 == n2);
    CHECK(n1 > 0);
    CHECK(n2 > 0);

    double sum1 = 0.0, sum2 = 0.0, sum3 = 0.0;
    Iterator i = first1, j = first2;
    for (; i != last1; ++i, ++j) {
        sum1 += *i;
        sum2 += *j;
        sum3 += (*i) * (*j);
    }

    return sum3 / n1 - sum1 / n1 * sum2 / n1;
}

} // namespace cl

#endif // CODELIBRARY_STATISTICS_COVARIANCE_H_

//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_DEVIATION_H_
#define CODELIBRARY_STATISTICS_DEVIATION_H_

#include <cmath>
#include <vector>

#include "codelibrary/statistics/mean.h"
#include "codelibrary/statistics/median.h"

namespace cl {

/**
 * Get the population standard deviation in range [first, last).
 *
 * Uses the divisor n (not n-1); treats the input as the full population
 * rather than a sample.
 */
template <typename Iterator>
double StandardDeviation(Iterator first, Iterator last) {
    auto n = std::distance(first, last);
    CHECK(n > 0);

    double mean = Mean(first, last);
    double sum = 0.0;
    for (Iterator i = first; i != last; ++i) {
        sum += (*i - mean) * (*i - mean);
    }

    return std::sqrt(sum / n);
}

/**
 * Get the median absolute deviation in range [first, last).
 *
 * The median absolute deviation is the median of the absolute deviation from
 * the median. It is a robust estimator of dispersion.
 */
template <typename Iterator>
double MedianAbsoluteDeviation(Iterator first, Iterator last) {
    auto n = std::distance(first, last);
    CHECK(n > 0);

    using T = typename std::iterator_traits<Iterator>::value_type;
    T median = Median(first, last);
    std::vector<T> absolute_deviation(n, 0);

    int i = 0;
    for (Iterator iter = first; iter != last; ++iter) {
        absolute_deviation[i++] += std::abs(*iter - median);
    }

    return Median(absolute_deviation.begin(), absolute_deviation.end());
}

/**
 * the root-mean-square deviation (RMSD) or root-mean-square error (RMSE) is a
 * frequently used measure of the differences between values predicted by a
 * model and the values observed.
 *
 *   RMSD = \sqrt(\sum(y1 - y)^2 / n)
 */
template <typename Iterator>
double RootMeanSquareDeviation(Iterator first1, Iterator last1,
                               Iterator first2, Iterator last2) {
    auto n = std::distance(first1, last1);
    CHECK(n > 0);
    auto n1 = std::distance(first2, last2);
    CHECK(n1 == n);

    double s = 0.0;
    for (Iterator i1 = first1, i2 = first2; i1 != last1; ++i1, ++i2) {
        s += (*i1 - *i2) * (*i1 - *i2);
    }

    return std::sqrt(s / n);
}

} // namespace cl

#endif // CODELIBRARY_STATISTICS_DEVIATION_H_

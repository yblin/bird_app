//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_MEDIAN_H_
#define CODELIBRARY_STATISTICS_MEDIAN_H_

#include <algorithm>
#include <vector>

namespace cl {

/**
 * Get the median in range [first, last).
 */
template <typename Iterator>
typename std::iterator_traits<Iterator>::value_type
Median(Iterator first, Iterator last) {
    CHECK(first != last);

    using T = typename std::iterator_traits<Iterator>::value_type;

    std::vector<T> values(first, last);
    std::nth_element(values.begin(), values.begin() + values.size() / 2,
                     values.end());
    return values[values.size() / 2];
}

} // namespace cl

#endif // CODELIBRARY_STATISTICS_MEDIAN_H_

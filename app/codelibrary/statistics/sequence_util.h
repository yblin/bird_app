//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STATISTICS_SEQUENCE_UTIL_H_
#define CODELIBRARY_STATISTICS_SEQUENCE_UTIL_H_

#include <deque>

#include "codelibrary/base/array.h"

namespace cl {
namespace sequence_util {

/**
 * Function to find contiguous segments in a sequence where the maximum
 * difference between values in the segment does not exceed a specified
 * threshold (max_difference).
 *
 * The function assigns a cluster label to each element in the input sequence
 * and returns the total number of clusters formed.
 *
 * The time cost of this function is O(N).
 *
 * Parameters:
 *   - sequence:       The input sequential data.
 *   - max_difference: The maximum allowed difference between values for them
 *                     to be considered in the same cluster.
 *   - min_elements:   The minimum number of elements required in a cluster
 *                     for it to be considered valid.
 *   - labels:         hold the cluster labels for each element.
 *
 * Returns:
 *   - The number of clusters formed.
 */
template <typename T>
int ClusterByDifference(const Array<T>& sequence, T max_difference,
                        int min_elements, Array<int>* labels,
                        Array<std::pair<int, int>>* clusters = nullptr) {
    CHECK(labels);

    labels->assign(sequence.size(), -1);

    std::vector<std::pair<int, int>> segments;
    int n = sequence.size();
    Array<int> prev(n, -1);

    std::deque<int> min_deque, max_deque;
    int start = 0;
    for (int end = 0; end < n; ++end) {
        while (!min_deque.empty() &&
               sequence[min_deque.back()] >= sequence[end]) {
            min_deque.pop_back();
        }
        min_deque.push_back(end);
        while (!max_deque.empty() &&
               sequence[max_deque.back()] <= sequence[end]) {
            max_deque.pop_back();
        }
        max_deque.push_back(end);

        while (sequence[max_deque.front()] - sequence[min_deque.front()] >
               max_difference) {
            ++start;
            if (min_deque.front() < start) {
                min_deque.pop_front();
            }
            if (max_deque.front() < start) {
                max_deque.pop_front();
            }
        }

        if (sequence[max_deque.front()] - sequence[min_deque.front()] <=
            max_difference) {
            if (end - start + 1 >= min_elements) {
                prev[end] = start;
            }
        }
    }

    int n_clusters = 0;
    for (int end = n - 1; end >= 0; --end) {
        if (prev[end] != -1) {
            for (int i = prev[end]; i <= end; ++i) {
                (*labels)[i] = n_clusters;
            }
            if (clusters) {
                clusters->emplace_back(prev[end], end);
            }
            ++n_clusters;
            end = prev[end];
        }
    }

    return n_clusters;
}

} // namespace sequence_util
} // namespace cl

#endif // CODELIBRARY_STATISTICS_SEQUENCE_UTIL_H_

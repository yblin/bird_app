//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_CONCURRENCY_PARALLEL_FOR_TEST_H_
#define CODELIBRARY_TEST_CONCURRENCY_PARALLEL_FOR_TEST_H_

#include <cmath>
#include <random>

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/testing.h"
#include "codelibrary/base/timer.h"
#include "codelibrary/concurrency/parallel_for.h"

namespace cl {

/**
 * Tests the parallel for loop computes the correct result for various number of
 * threads.
 */
TEST(ParallelFor, NumThreads) {
    int max_threads = ParallelInvoke::GetInstance()->max_threads();
    const int size = max_threads * 4;
    Array<double> expected_results(size, 0);
    for (int i = 0; i < size; ++i) {
        expected_results[i] = std::sqrt(i);
    }

    for (int n_threads = max_threads; n_threads < 2 * max_threads;
         ++n_threads) {
        Array<double> values(size, 0);
        ParallelFor(0, size, n_threads, [&values](int i) {
            values[i] = std::sqrt(i);
        });
        ASSERT_EQ_RANGE(values.begin(), values.end(),
                        expected_results.begin(), expected_results.end());
    }
}

/**
 * Tests nested for loops do not result in a deadlock.
 */
TEST(ParallelFor, NestedParallelForDeadlock) {
    int max_threads = ParallelInvoke::GetInstance()->max_threads();
    const int size = max_threads;
    ArrayND<double> expected_results(size, size);
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            expected_results(i, j) = std::sqrt(i + j);
        }
    }

    ArrayND<double> values(size, size);
    ParallelFor(0, size, [&values, size](int i) {
        ParallelFor(0, size, [&values, i](int j) {
            values(i, j) = std::sqrt(i + j);
        });
    });
    ASSERT_EQ_RANGE(values.begin(), values.end(),
                    expected_results.begin(), expected_results.end());
}

TEST(ParallelFor, CheckDeadlock) {
    const int n_tests = 100;

    std::mt19937 r;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    const int n = 1024;
    Array<double> x(n);
    for (int i = 0; i < n; ++i)
        x[i] = uniform(r);

    for (int t = 0; t < n_tests; ++t) {
        ParallelFor(0, n, 16, [&x](int id) { x[id] = 0.0; });
    }
    double sum = 0.0;
    for (double v : x) {
        sum += v;
    }
    ASSERT_EQ(sum, 0.0);
}

TEST(ParallelFor, Performance) {
    int sizes[]   = { 128, 256, 1024, 4096 };
    int n_threads[] = { 1, 2, 4, 8, 16 };
    const int n_tests = 100;

    std::mt19937 r;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    Print("\n--------------------------------------------------------\n");

    Print("Size");
    for (int n_threads : n_threads) {
        Print("{:7}", n_threads);
    }
    Print("\n");
    for (int n : sizes) {
        Array<double> x(n);
        for (int i = 0; i < n; ++i)
            x[i] = uniform(r);
        Print("{:4}", n);
        for (int n_threads : n_threads) {
            Timer timer;
            timer.Start();
            for (int t = 0; t < 100; ++t) {
                ParallelFor(0, n, n_threads, [&x](int id) { x[id] = 0.0; });
            }
            timer.Stop();
            Print("{:>7}", timer.average_time(n_tests));

            double sum = 0.0;
            for (double v : x) {
                sum += v;
            }
            ASSERT_EQ(sum, 0.0);
        }
        Print("\n");
    }
    Print("-------------------------------------------------------\n\n");
}

/**
 * Tests parallel for loop with ranges.
 */
TEST(ParallelForWithRange, NumThreads) {
    int max_threads = ParallelInvoke::GetInstance()->max_threads();
    const int size = max_threads * 4;
    Array<double> expected_results(size, 0);
    for (int i = 0; i < size; ++i) {
        expected_results[i] = std::sqrt(i);
    }

    for (int num_threads = max_threads; num_threads < 2 * max_threads;
         ++num_threads) {
        Array<double> values(size, 0);
        ParallelFor(0, size, num_threads, [&values](int start, int end) {
            for (int i = start; i < end; ++i)
                values[i] = std::sqrt(i);
        });
        ASSERT_EQ_RANGE(values.begin(), values.end(),
                        expected_results.begin(), expected_results.end());
    }
}

/**
 * Tests parallel for loop with ranges and lower bound on minimal range size.
 */
TEST(ParallelForWithRange, MinimalSize) {
    const int n_threads = 4;
    const int min_block_size = 5;

    for (int size = min_block_size; size <= 25; ++size) {
        std::atomic<bool> failed(false);
        ParallelFor(0, size, n_threads,
                    [&failed](int start, int end) {
                        if (end - start < min_block_size) failed = true;
                    },
                    min_block_size);
        ASSERT_EQ(failed.load(), false);
    }
}

}  // namespace cl

#endif // CODELIBRARY_TEST_CONCURRENCY_PARALLEL_FOR_TEST_H_

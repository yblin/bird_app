//
// Copyright 2018-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_PRIME_WHEEL_SIEVE_TEST_H_
#define CODELIBRARY_TEST_MATH_PRIME_WHEEL_SIEVE_TEST_H_

#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/base/timer.h"
#include "codelibrary/math/prime/wheel_sieve.h"

namespace cl {
namespace test {

TEST(WheelSieveTest, Performance) {
    const int n_test = 7;
    int p[]     = { 2, 3, 4, 5, 6, 7, 8 };
    int tests[] = { 100000, 100000, 10000, 1000, 100, 10, 1 };

    Print("\n");
    Print(" n          Time      Number of primes\n");
    Print("--------------------------------------\n");

    for (int i = 0; i < n_test; ++i) {
        Timer timer;
        timer.Start();
        int n = static_cast<int>(pow(10.0, p[i]));
        for (int j = 0; j < tests[i]; ++j) {
           prime::WheelSieve30 s(n);
        }
        timer.Stop();
        std::string time = Timer::ReadableTime(timer.elapsed_seconds() /
                                                     tests[i]);

        prime::WheelSieve30 s(n);
        int n_primes = 0;
        for (auto i = s.begin(); i != s.end(); ++i) {
            if (s.IsPrime(i)) ++n_primes;
        }

        Print("10^{} {:>11}s {:13}\n", p[i], time.c_str(), n_primes + 3);
    }
    Print("--------------------------------------\n");
    Print("\n");
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_PRIME_WHEEL_SIEVE_TEST_H_

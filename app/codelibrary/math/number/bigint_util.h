//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_NUMBER_BIGINT_UTIL_H_
#define CODELIBRARY_MATH_NUMBER_BIGINT_UTIL_H_

#include <random>

#include "codelibrary/math/number/bigint.h"

namespace cl {
namespace bigint_util {

/**
 * Generate a random bigint in the range [0, n].
 */
template <typename RandomEngine>
BigInt Random(const BigInt& n, RandomEngine& random) {
    CHECK(n >= 0);

    if (n == 0) return BigInt(0);

    uint32_t back = n.highest_bit32();
    std::uniform_int_distribution<uint32_t> uniform(0, back);

    BigInt res;
    while (true) {
        uint32_t r = uniform(random);
        bool flag = (r == back);
        res = r;

        int i = 0;
        for (i = n.size() - 2; i >= 0; --i) {
            uint32_t tmp = random();
            if (flag) {
                if (tmp > n.data(i)) break;
                if (tmp < n.data(i)) flag = false;
            }

            res <<= 32;
            res += tmp;
        }
        if (i < 0) break;
    }

    return res;
}

} // namespace bigint_util
} // namespace cl

#endif // CODELIBRARY_MATH_NUMBER_BIGINT_UTIL_H_

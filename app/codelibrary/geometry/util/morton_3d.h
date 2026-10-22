//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_MORTON_3D_H_
#define CODELIBRARY_GEOMETRY_UTIL_MORTON_3D_H_

#include <cstdint>

#include "codelibrary/base/log.h"

namespace cl {
namespace geometry {
namespace morton {

/**
 * Method to seperate bits from a given integer 3 positions apart.
 */
inline uint64_t SplitBy3Bits(int a) {
    uint64_t x = a    & 0x00000000001fffffLL;
    x = (x | x << 32) & 0x001f00000000ffffLL;
    x = (x | x << 16) & 0x001f0000ff0000ffLL;
    x = (x | x << 8)  & 0x100f00f00f00f00fLL;
    x = (x | x << 4)  & 0x10c30c30c30c30c3LL;
    x = (x | x << 2)  & 0x1249249249249249LL;
    return x;
}

/**
 * The inverse operation of SplitBy3Bits().
 */
inline int GetThirdBits(uint64_t x) {
    x = x & 0x1249249249249249ULL;
    x = (x ^ (x >> 2))  & 0x10c30c30c30c30c3ULL;
    x = (x ^ (x >> 4))  & 0x100f00f00f00f00fULL;
    x = (x ^ (x >> 8))  & 0x001f0000ff0000ffULL;
    x = (x ^ (x >> 16)) & 0x001f00000000ffffULL;
    x = (x ^ (x >> 32)) & 0x00000000001fffffULL;
    return static_cast<int>(x);
}

} // namespace morton

/**
 * 3D Morton encode.
 */
inline uint64_t MortonEncode(int x, int y, int z) {
    return morton::SplitBy3Bits(x) | (morton::SplitBy3Bits(y) << 1) |
           (morton::SplitBy3Bits(z) << 2);
}

/**
 * 3D Morton decode.
 */
inline void MortonDecode(uint64_t c, int* x, int* y, int* z) {
    CHECK(x && y && z);

    *x = morton::GetThirdBits(c);
    *y = morton::GetThirdBits(c >> 1);
    *z = morton::GetThirdBits(c >> 2);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_MORTON_3D_H_

//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_ENDIAN_UTIL_H_
#define CODELIBRARY_UTIL_IO_ENDIAN_UTIL_H_

#include <cstdint>

namespace cl {
namespace endian_util {

enum class Endian {
#if defined(_MSC_VER) && !defined(__clang__)
    LITTLE = 0,
    BIG    = 1,
    NATIVE = LITTLE
#else
    LITTLE = __ORDER_LITTLE_ENDIAN__,
    BIG    = __ORDER_BIG_ENDIAN__,
    NATIVE = __BYTE_ORDER__
#endif
};

/**
 * Get the endian type of the current compiler.
 */
inline Endian GetEndian() {
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1 ? Endian::BIG : Endian::LITTLE;
}

/**
 * Swap bytes for 16-bit integer.
 */
inline uint16_t Swap2(uint16_t val) {
    return (val << 8) | (val >> 8);
}

/**
 * Inplace endian swap.
 */
inline void Swap2(uint8_t* data) {
    uint16_t tmp = *reinterpret_cast<uint16_t*>(data);
    tmp = static_cast<uint16_t>((tmp >> 8) | (tmp << 8));
    *reinterpret_cast<uint16_t*>(data) = tmp;
}

/**
 * Swap bytes for 32-bit integer.
 */
inline uint32_t Swap4(uint32_t val) {
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0x00FF00FF);
    return (val << 16) | (val >> 16);
}

/**
 * Inplace endian swap.
 */
inline void Swap4(uint8_t* data) {
    uint32_t tmp = *reinterpret_cast<uint32_t*>(data);
    tmp = (tmp >> 16) | (tmp << 16);
    tmp = ((tmp & 0xFF00FF00) >> 8) | ((tmp & 0x00FF00FF) << 8);
    *reinterpret_cast<uint32_t*>(data) = tmp;
}

/**
 * Swap bytes for 64-bit integer.
 */
inline uint64_t Swap8(uint64_t val) {
    val = ((val << 8)  & 0xFF00FF00FF00FF00ULL) |
          ((val >> 8)  & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) |
          ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | (val >> 32);
}

/**
 * Inplace endian swap.
 */
inline void Swap8(uint8_t* data) {
    uint64_t tmp = *reinterpret_cast<uint64_t*>(data);
    tmp = (tmp >> 32) | (tmp << 32);
    tmp = ((tmp & 0xFFFF0000FFFF0000) >> 16) |
          ((tmp & 0x0000FFFF0000FFFF) << 16);
    tmp = ((tmp & 0xFF00FF00FF00FF00) >> 8) | ((tmp & 0x00FF00FF00FF00FF) << 8);
    *reinterpret_cast<uint64_t*>(data) = tmp;
}

} // namespace endian_util
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_ENDIAN_UTIL_H_

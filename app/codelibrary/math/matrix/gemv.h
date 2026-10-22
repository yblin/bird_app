//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_GEMV_H_
#define CODELIBRARY_MATH_MATRIX_GEMV_H_

#include <cstring>
#include <type_traits>

namespace cl {
namespace blas {

/**
 * Compute a matrix-vector product using a general matrix, which is defined as:
 *
 *   c += Ab
 *
 * Input:
 *  m - The number of rows of the matrix a.
 *  n - The number of columns of the matrix a.
 *  a - Array, size m * n.
 *  b - Array, size at least n.
 *  c - Array, size at least m.
 *
 * Output:
 *  c - Update vector c.
 *
 * Performance:
 *                       Baseline    Eigen
 * m=4,     n=4          4ns         58ns
 * m=10,    n=10         42ns        80ns
 * m=100,   n=100        3us         1us
 * m=1000,  n=1000       333us       206us
 * m=10000, n=10000      49ms        41ms
 * m=1000,  n=100000     59ms        45ms
 * m=100,   n=1000000    64ms        55ms
 * m=5,     n=10000000   40ms        28ms
 */
template <typename T>
void GEMV(int m, int n, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0:
        return;
    case 1:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i] * b[0];
        }
        return;
    case 2:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 2]     * b[0] +
                    a[i * 2 + 1] * b[1];
        }
        return;
    case 3:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 3]     * b[0] +
                    a[i * 3 + 1] * b[1] +
                    a[i * 3 + 2] * b[2];
        }
        return;
    case 4:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 4]     * b[0] +
                    a[i * 4 + 1] * b[1] +
                    a[i * 4 + 2] * b[2] +
                    a[i * 4 + 3] * b[3];
        }
        return;
    }

    for (int i = 0; i < m; ++i) {
        // Using cache size 4 to accelerate.
        int j = 0;
        for (; j + 3 < n; j += 4) {
            c[i] += a[i * n + j]     * b[j] +
                    a[i * n + j + 1] * b[j + 1] +
                    a[i * n + j + 2] * b[j + 2] +
                    a[i * n + j + 3] * b[j + 3];
        }
        for (; j < n; ++j) {
            c[i] += a[i * n + j] * b[j];
        }
    }
}

/**
 * Compute a matrix-vector product using a general matrix, which is defined as:
 *
 *   c += A'b
 *
 * Input:
 *  m - The number of rows of the matrix a.
 *  n - The number of columns of the matrix a.
 *  a - Array, size m * n.
 *  b - Array, size at least m.
 *  c - Array, size at least n.
 *
 * Output:
 *  c - Update vector c.
 */
template <typename T>
void GEMVTrans(int m, int n, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value);

    switch (n) {
    case 0:
        return;
    case 1:
        memcpy(c, a, sizeof(T) * m);
        return;
    case 2:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 2]     * b[0] +
                    a[i * 2 + 1] * b[1];
        }
        return;
    case 3:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 3]     * b[0] +
                    a[i * 3 + 1] * b[1] +
                    a[i * 3 + 2] * b[2];
        }
        return;
    case 4:
        for (int i = 0; i < m; ++i) {
            c[i] += a[i * 4]     * b[0] +
                    a[i * 4 + 1] * b[1] +
                    a[i * 4 + 2] * b[2] +
                    a[i * 4 + 3] * b[3];
        }
        return;
    }

    for (int i = 0; i < m; ++i) {
        // Using cache size 4 to accelerate.
        int j = 0;
        for (; j + 3 < n; j += 4) {
            c[j]     += a[i * n + j]     * b[i];
            c[j + 1] += a[i * n + j + 1] * b[i];
            c[j + 2] += a[i * n + j + 2] * b[i];
            c[j + 3] += a[i * n + j + 3] * b[i];
        }
        for (; j < n; ++j) {
            c[j] += a[i * n + j] * b[i];
        }
    }
}

} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_GEMV_H_

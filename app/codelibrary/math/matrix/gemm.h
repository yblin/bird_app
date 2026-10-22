//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_GEMM_H_
#define CODELIBRARY_MATH_MATRIX_GEMM_H_

#include "codelibrary/base/log.h"
#include "codelibrary/math/basic_linear_algebra.h"
#include "codelibrary/math/matrix/dgemm.h"
#include "codelibrary/math/matrix/gemv.h"
#include "codelibrary/math/matrix/sgemm.h"

namespace cl {
namespace blas {

/**
 * Compute a matrix-matrix product for small matrices.
 * See GEMM() for more details.
 */
template <typename T>
void GEMMSmall(int m, int n, int k, const T* a, const T* b, T* c) {
    int p;
    T sum;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < k; ++j) {
            sum = T(0);
            for (p = 0; p + 3 < n; p += 4) {
                sum += a[i * n + p]     * b[ p      * k + j];
                sum += a[i * n + p + 1] * b[(p + 1) * k + j];
                sum += a[i * n + p + 2] * b[(p + 2) * k + j];
                sum += a[i * n + p + 3] * b[(p + 3) * k + j];
            }
            for (; p < n; ++p) {
                sum += a[i * n + p] * b[p * k + j];
            }
            c[i * k + j] += sum;
        }
    }
}

/**
 * Computes a matrix-matrix product with general matrices, which is defined as:
 *
 *   c += a * b
 *
 * Input:
 *  m - The number of rows of the matrix a.
 *  n - The number of columns of the matrix a.
 *  k - The number of columns of the matrix b.
 *  a - Array, size m * n.
 *  b - Array, size n * k.
 *  c - Array, size m * k.
 *
 * Output:
 *  c - Update matrix c.
 */
template <typename T>
void GEMMGeneric(int m, int n, int k, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(c != a && c != b);

    if (m == 1) {
        GEMVTrans(n, k, b, a, c);
        return;
    } else if (n == 1) {
        #pragma omp parallel for
        for (int i = 0; i < m; ++i) {
            AXPY(k, a[i], b, c + m * i);
        }
        return;
    } else if (k == 1) {
        GEMV(m, n, a, b, c);
        return;
    }

    const int block_size = 128;

    if (m <= block_size && n <= block_size && k <= block_size) {
        GEMMSmall(m, n, k, a, b, c);
        return;
    }

#pragma omp parallel for
    for (int ii = 0; ii < m; ii += block_size) {
        for (int jj = 0; jj < k; jj += block_size) {
            for (int pp = 0; pp < n; pp += block_size) {
                int block_m = (ii + block_size <= m) ? block_size : (m - ii);
                int block_n = (pp + block_size <= n) ? block_size : (n - pp);
                int block_k = (jj + block_size <= k) ? block_size : (k - jj);
                int i, j, p;
                for (i = ii; i < ii + block_m; ++i) {
                    for (j = jj; j < jj + block_k; ++j) {
                        T sum = T(0);
                        for (p = pp; p + 3 < pp + block_n; p += 4) {
                            sum += a[i * n + p]     * b[ p      * k + j];
                            sum += a[i * n + p + 1] * b[(p + 1) * k + j];
                            sum += a[i * n + p + 2] * b[(p + 2) * k + j];
                            sum += a[i * n + p + 3] * b[(p + 3) * k + j];
                        }
                        for (; p < pp + block_n; ++p) {
                            sum += a[i * n + p] * b[p * k + j];
                        }
                        c[i * k + j] += sum;
                    }
                }
            }
        }
    }
}

template <typename T>
void GEMM(int m, int n, int k, const T* a, const T* b, T* c) {
    return GEMMGeneric(m, n, k, a, b, c);
}

inline void GEMM(int m, int n, int k, const float* a, const float* b,
                 float* c) {
    CHECK(c != a && c != b);

    if (m == 1 || n == 1 || k == 1) {
        GEMMGeneric(m, n, k, a, b, c);
    } else if (m < sgemm::ROW_PACK && n < sgemm::ROW_PACK &&
               k < sgemm::ROW_PACK) {
        GEMMSmall(m, n, k, a, b, c);
    } else {
        sgemm::SGEMM(m, n, k, a, b, c);
    }
}

inline void GEMM(int m, int n, int k, const double* a, const double* b,
                 double *c) {
    CHECK(c != a && c != b);

    if (m == 1 || n == 1 || k == 1) {
        GEMMGeneric(m, n, k, a, b, c);
    } else if (m < dgemm::ROW_PACK && n < dgemm::ROW_PACK &&
               k < dgemm::ROW_PACK) {
        GEMMSmall(m, n, k, a, b, c);
    } else {
        dgemm::DGEMM(m, n, k, a, b, c);
    }
}

} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_GEMM_H_

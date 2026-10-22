//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Code from ugemm.
//

#ifndef CODELIBRARY_MATH_MATRIX_SGEMM_H_
#define CODELIBRARY_MATH_MATRIX_SGEMM_H_

#include <algorithm>
#include <cstring>
#include <immintrin.h>

namespace cl {
namespace blas {
namespace sgemm {

static const int ROW_PACK = 8;
static const int COL_PACK = 8;

static const int N_BUFFER_M = 384;
static const int N_BUFFER_K = 384;
static const int N_BUFFER_N = 4096;

// Local buffers for storing panels from A and B.
alignas(32) static float BUFFER_A[N_BUFFER_M * N_BUFFER_K];
alignas(32) static float BUFFER_B[N_BUFFER_K * N_BUFFER_N];

/**
 * Packing complete panels from A (i.e. without padding).
 */
static void PackMRxk(int k, const float* a, int inc_row_a, float* buffer) {
    for (int j = 0; j < k; ++j) {
        for (int i = 0; i < ROW_PACK; ++i) {
            buffer[i] = a[i * inc_row_a];
        }
        buffer += ROW_PACK;
        ++a;
    }
}

/**
 * Packing panels from A with padding if required.
 */
static void PackA(int mc, int kc, const float* a, int inc_row_a,
                  float *buffer) {
    int mp = mc / ROW_PACK;
    int mr = mc % ROW_PACK;

    int i, j;
    for (i = 0; i < mp; ++i) {
        PackMRxk(kc, a, inc_row_a, buffer);
        buffer += kc * ROW_PACK;
        a      += ROW_PACK * inc_row_a;
    }
    if (mr > 0) {
        for (j = 0; j < kc; ++j) {
            for (i = 0; i < mr; ++i) {
                buffer[i] = a[i * inc_row_a];
            }
            for (i = mr; i < ROW_PACK; ++i) {
                buffer[i] = 0.0f;
            }
            buffer += ROW_PACK;
            ++a;
        }
    }
}

/**
 * Packing complete panels from B (i.e. without padding).
 */
static void PackkxNR(int k, const float* b, int inc_row_b, float* buffer) {
    int i, j;

    for (i = 0; i < k; ++i) {
        for (j = 0; j < COL_PACK; ++j) {
            buffer[j] = b[j];
        }
        buffer += COL_PACK;
        b      += inc_row_b;
    }
}

/**
 * Packing panels from B with padding if required.
 */
static void PackB(int kc, int nc, const float* b, int inc_row_b,
                  float* buffer) {
    int np  = nc / COL_PACK;
    int _nr = nc % COL_PACK;

    int i, j;

    for (j=0; j<np; ++j) {
        PackkxNR(kc, b, inc_row_b, buffer);
        buffer += kc * COL_PACK;
        b      += COL_PACK;
    }
    if (_nr > 0) {
        for (i = 0; i < kc; ++i) {
            for (j = 0; j < _nr; ++j) {
                buffer[j] = b[j];
            }
            for (j = _nr; j < COL_PACK; ++j) {
                buffer[j] = 0.0;
            }
            buffer += COL_PACK;
            b      += inc_row_b;
        }
    }
}

static inline void Dot8x8AVX(const float *a, const float *b, const int kc,
                             float *c) {
    __m256 a0, b0, b1, b2, b3, b4, b5, b6, b7;
    __m256 c0, c1, c2, c3, c4, c5, c6, c7;

    __asm__ volatile("prefetcht2 0(%0)          \n\t" : :"r"(b));
    __asm__ volatile("prefetcht0 0(%0)          \n\t" : :"r"(a));
    c0 = _mm256_setzero_ps();
    c1 = _mm256_setzero_ps();
    c2 = _mm256_setzero_ps();
    c3 = _mm256_setzero_ps();
    c4 = _mm256_setzero_ps();
    c5 = _mm256_setzero_ps();
    c6 = _mm256_setzero_ps();
    c7 = _mm256_setzero_ps();

    for (int i = 0; i < kc; ++i) {
        b0 = _mm256_broadcast_ss(b + 0);
        b1 = _mm256_broadcast_ss(b + 1);
        b2 = _mm256_broadcast_ss(b + 2);
        b3 = _mm256_broadcast_ss(b + 3);
        b4 = _mm256_broadcast_ss(b + 4);
        b5 = _mm256_broadcast_ss(b + 5);
        b6 = _mm256_broadcast_ss(b + 6);
        b7 = _mm256_broadcast_ss(b + 7);
        a0 = _mm256_load_ps(a);

        a += 8;
        b += 8;
        __asm__ volatile("prefetcht2 0(%0)          \n\t" : :"r"(b));
        __asm__ volatile("prefetcht0 0(%0)          \n\t" : :"r"(a));

        c0 = _mm256_add_ps(c0, _mm256_mul_ps(a0, b0));
        c1 = _mm256_add_ps(c1, _mm256_mul_ps(a0, b1));
        c2 = _mm256_add_ps(c2, _mm256_mul_ps(a0, b2));
        c3 = _mm256_add_ps(c3, _mm256_mul_ps(a0, b3));
        c4 = _mm256_add_ps(c4, _mm256_mul_ps(a0, b4));
        c5 = _mm256_add_ps(c5, _mm256_mul_ps(a0, b5));
        c6 = _mm256_add_ps(c6, _mm256_mul_ps(a0, b6));
        c7 = _mm256_add_ps(c7, _mm256_mul_ps(a0, b7));
    }

    _mm256_store_ps(c +  0, c0);
    _mm256_store_ps(c +  8, c1);
    _mm256_store_ps(c + 16, c2);
    _mm256_store_ps(c + 24, c3);
    _mm256_store_ps(c + 32, c4);
    _mm256_store_ps(c + 40, c5);
    _mm256_store_ps(c + 48, c6);
    _mm256_store_ps(c + 56, c7);
}

/**
 * Micro kernel for multiplying panels from A and B.
 */
static void GEMMMicroKernel(int kc, const float* a, const float* b,
                            int inc_row_c, int inc_col_c,
                            float* c) {
    float ab[ROW_PACK * COL_PACK] __attribute__ ((aligned (32)));
    int i, j;

    // Compute AB = A * B
    Dot8x8AVX(a, b, kc, ab);

    // Update C += AB.
    for (j = 0; j < COL_PACK; ++j) {
        for (i = 0; i < ROW_PACK; ++i) {
            c[i * inc_row_c + j * inc_col_c] += ab[i + j * ROW_PACK];
        }
    }
}

/**
 * Kernel for the multiplication of blocks of A and B.
 * We assume that these blocks were previously packed to buffers A and B.
 */
static void GEMMKernel(int mc, int nc, int kc, int inc_row_c, float* c) {
    int mp = (mc + ROW_PACK - 1) / ROW_PACK;
    int np = (nc + COL_PACK - 1) / COL_PACK;

    int _mr = mc % ROW_PACK;
    int _nr = nc % COL_PACK;

    #pragma omp parallel for
    for (int j = 0; j < np; ++j) {
        int nr = (j != np - 1 || _nr == 0) ? COL_PACK : _nr;

        for (int i = 0; i < mp; ++i) {
            int mr = (i != mp - 1 || _mr == 0) ? ROW_PACK : _mr;
            if (mr == ROW_PACK && nr == COL_PACK) {
                GEMMMicroKernel(kc,
                                &BUFFER_A[i * kc * ROW_PACK],
                                &BUFFER_B[j * kc * COL_PACK],
                                inc_row_c,
                                1,
                                &c[i * ROW_PACK * inc_row_c + j * COL_PACK]);
            } else {
                float buffer_c[ROW_PACK * COL_PACK]
                        __attribute__ ((aligned (32)));
                std::memset(buffer_c, 0, sizeof(buffer_c));
                GEMMMicroKernel(kc,
                                &BUFFER_A[i * kc * ROW_PACK],
                                &BUFFER_B[j * kc * COL_PACK],
                                1,
                                ROW_PACK,
                                buffer_c);
                int k = i * ROW_PACK * inc_row_c + j * COL_PACK;
                for (int tj = 0; tj < nr; ++tj) {
                    for (int ti = 0; ti < mr; ++ti) {
                        c[k + ti * inc_row_c + tj] +=
                                buffer_c[ti + tj * ROW_PACK];
                    }
                }
            }
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
inline void SGEMM(int m, int n, int k, const float* a, const float* b,
                  float* c) {
    std::swap(n, k);

    int mb = (m + N_BUFFER_M - 1) / N_BUFFER_M;
    int nb = (n + N_BUFFER_N - 1) / N_BUFFER_N;
    int kb = (k + N_BUFFER_K - 1) / N_BUFFER_K;

    int _mc = m % N_BUFFER_M;
    int _nc = n % N_BUFFER_N;
    int _kc = k % N_BUFFER_K;

    int mc, nc, kc;
    int i, j, l;

    for (j = 0; j < nb; ++j) {
        nc = (j != nb - 1 || _nc == 0) ? N_BUFFER_N : _nc;

        for (l = 0; l < kb; ++l) {
            kc = (l != kb - 1 || _kc == 0) ? N_BUFFER_K : _kc;
            PackB(kc, nc, &b[l * N_BUFFER_K * n + j * N_BUFFER_N], n, BUFFER_B);

            for (i = 0; i < mb; ++i) {
                mc = (i != mb - 1 || _mc == 0) ? N_BUFFER_M : _mc;

                PackA(mc, kc, &a[i * N_BUFFER_M * k + l * N_BUFFER_K], k,
                      BUFFER_A);
                GEMMKernel(mc, nc, kc, n,
                           &c[i * N_BUFFER_M * n + j * N_BUFFER_N]);
            }
        }
    }
}

} // namespace sgemm
} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_SGEMM_H_

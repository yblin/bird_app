//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Code from ugemm.
//

#ifndef CODELIBRARY_MATH_MATRIX_DGEMM_H_
#define CODELIBRARY_MATH_MATRIX_DGEMM_H_

#include <algorithm>
#include <cstring>
#include <immintrin.h>

namespace cl {
namespace blas {
namespace dgemm {

static const int ROW_PACK = 8;
static const int COL_PACK = 4;

static const int N_BUFFER_M = 384;
static const int N_BUFFER_K = 384;
static const int N_BUFFER_N = 4096;

// Local buffers for storing panels from A and B.
alignas(32) static double BUFFER_A[N_BUFFER_M * N_BUFFER_K];
alignas(32) static double BUFFER_B[N_BUFFER_K * N_BUFFER_N];

/**
 * Packing complete panels from A (i.e. without padding).
 */
static void PackMRxk(int k, const double* a, int inc_row_a, double* buffer) {
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
static void PackA(int mc, int kc, const double* a, int inc_row_a,
                  double *buffer) {
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
                buffer[i] = 0.0;
            }
            buffer += ROW_PACK;
            ++a;
        }
    }
}

/**
 * Packing complete panels from B (i.e. without padding).
 */
static void PackkxNR(int k, const double* b, int inc_row_b, double* buffer) {
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
static void PackB(int kc, int nc, const double* b, int inc_row_b,
                  double* buffer) {
    int np = nc / COL_PACK;
    int nr = nc % COL_PACK;

    int i, j;

    for (j = 0; j < np; ++j) {
        PackkxNR(kc, b, inc_row_b, buffer);
        buffer += kc * COL_PACK;
        b      += COL_PACK;
    }
    if (nr > 0) {
        for (i = 0; i < kc; ++i) {
            for (j = 0; j < nr; ++j) {
                buffer[j] = b[j];
            }
            for (j = nr; j < COL_PACK; ++j) {
                buffer[j] = 0.0;
            }
            buffer += COL_PACK;
            b      += inc_row_b;
        }
    }
}

/**
 * Micro kernel for multiplying panels from A and B.
 */
static void GEMMMicroKernel(int kc, const double *a, const double *b,
                            int rs_c, int cs_c, const double* b_next,
                            double* c) {
    int k_iter  = kc / 2;
    int k_left  = kc % 2;

    int i;

    double *c00, *c01, *c02, *c03;
    double *c40, *c41, *c42, *c43;

    // Quads.
    __m256d va0_3, va4_7;
    __m256d vA0_3, vA4_7;
    __m256d vb0, vb1, vb2, vb3;
    __m256d vb;
    __m256d vB0;

    __m256d va0_3b_0, va4_7b_0;
    __m256d va0_3b_1, va4_7b_1;
    __m256d va0_3b_2, va4_7b_2;
    __m256d va0_3b_3, va4_7b_3;

    __m256d va0_3b0, va4_7b0;
    __m256d va0_3b1, va4_7b1;
    __m256d va0_3b2, va4_7b2;
    __m256d va0_3b3, va4_7b3;

    __m256d vtmp;
    __m256d vc0_3_0, vc0_3_1, vc0_3_2, vc0_3_3;
    __m256d vc4_7_0, vc4_7_1, vc4_7_2, vc4_7_3;

    __m128d aa, bb;

    __asm__ volatile("prefetcht0 0(%0)          \n\t" : :"r"(a));
    __asm__ volatile("prefetcht2 0(%0)          \n\t" : :"r"(b_next));
    __asm__ volatile("prefetcht0 0(%0)          \n\t" : :"r"(c));

    va0_3b_0 = _mm256_setzero_pd();
    va0_3b_1 = _mm256_setzero_pd();
    va0_3b_2 = _mm256_setzero_pd();
    va0_3b_3 = _mm256_setzero_pd();

    va4_7b_0 = _mm256_setzero_pd();
    va4_7b_1 = _mm256_setzero_pd();
    va4_7b_2 = _mm256_setzero_pd();
    va4_7b_3 = _mm256_setzero_pd();

    // Load va0_3.
    va0_3 = _mm256_load_pd(a);
    // Load va4_7.
    va4_7 = _mm256_load_pd(a + 4);

    // Load vb (b0,b1,b2,b3).
    vb0 = _mm256_load_pd(b);

    for (i = 0; i < k_iter; ++i) {
        __asm__ volatile("prefetcht0 192(%0)          \n\t" : :"r"(a) );

        // Load va0_3 (Prefetch).
        vA0_3 = _mm256_load_pd(a + 8);

        // Iteration 0.
        vtmp = _mm256_mul_pd(va0_3, vb0);
        va0_3b_0 = _mm256_add_pd(va0_3b_0, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb0);
        va4_7b_0 = _mm256_add_pd(va4_7b_0, vtmp);

        // Load va4_7 (Prefetch).
        vA4_7 = _mm256_load_pd(a + 12);

        // Shuffle vb (b1,b0,b3,b2).
        vb1 = _mm256_shuffle_pd(vb0, vb0, 0x5);

        vtmp = _mm256_mul_pd(va0_3, vb1);
        va0_3b_1 = _mm256_add_pd(va0_3b_1, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb1);
        va4_7b_1 = _mm256_add_pd(va4_7b_1, vtmp);

        // Permute vb (b3,b2,b1,b0).
        vb2 = _mm256_permute2f128_pd(vb1, vb1, 0x1);

        // Load vb (b0,b1,b2,b3) (Prefetch).
        vB0 = _mm256_load_pd(b + 4);

        vtmp = _mm256_mul_pd(va0_3, vb2);
        va0_3b_2 = _mm256_add_pd(va0_3b_2, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb2);
        va4_7b_2 = _mm256_add_pd(va4_7b_2, vtmp);

        // Shuffle vb (b3,b2,b1,b0).
        vb3 = _mm256_shuffle_pd(vb2, vb2, 0x5);

        vtmp = _mm256_mul_pd(va0_3, vb3);
        va0_3b_3 = _mm256_add_pd(va0_3b_3, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb3);
        va4_7b_3 = _mm256_add_pd(va4_7b_3, vtmp);

        // Iteration 1.

        __asm__ volatile("prefetcht0 512(%0)          \n\t" : :"r"(a) );

        // Load va0_3 (Next iteration).
        va0_3 = _mm256_load_pd(a + 16);

        vtmp = _mm256_mul_pd(vA0_3, vB0);
        va0_3b_0 = _mm256_add_pd(va0_3b_0, vtmp);

        vb1 = _mm256_shuffle_pd(vB0, vB0, 0x5);

        vtmp = _mm256_mul_pd(vA4_7, vB0);
        va4_7b_0 = _mm256_add_pd(va4_7b_0, vtmp);

        vtmp = _mm256_mul_pd(vA0_3, vb1);
        va0_3b_1 = _mm256_add_pd(va0_3b_1, vtmp);

        // Load va4_7 (Next iteration).
        va4_7 = _mm256_load_pd(a + 20);

        vb2 = _mm256_permute2f128_pd(vb1, vb1, 0x1);

        vtmp = _mm256_mul_pd(vA4_7, vb1);
        va4_7b_1 = _mm256_add_pd(va4_7b_1, vtmp);

        vtmp = _mm256_mul_pd(vA0_3, vb2);
        va0_3b_2 = _mm256_add_pd(va0_3b_2, vtmp);

        vb3 = _mm256_shuffle_pd(vb2, vb2, 0x5);

        vtmp = _mm256_mul_pd(vA4_7, vb2);
        va4_7b_2 = _mm256_add_pd(va4_7b_2, vtmp);

        // Load vb0(Next iteration).
        vb0 = _mm256_load_pd(b + 8);

        vtmp = _mm256_mul_pd(vA0_3, vb3);
        va0_3b_3 = _mm256_add_pd(va0_3b_3, vtmp);

        vtmp = _mm256_mul_pd(vA4_7, vb3);
        va4_7b_3 = _mm256_add_pd(va4_7b_3, vtmp);

        a += 16;
        b += 8;
    }

    for (i = 0; i < k_left; ++i) {
        // Iteration 0.

        // Load va0_3.
        va0_3 = _mm256_load_pd(a);
        // Load va4_7.
        va4_7 = _mm256_load_pd(a + 4);

        // Load vb (b0,b1,b2,b3).
        vb = _mm256_load_pd(b);

        vtmp = _mm256_mul_pd(va0_3, vb);
        va0_3b_0 = _mm256_add_pd(va0_3b_0, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb);
        va4_7b_0 = _mm256_add_pd(va4_7b_0, vtmp);

        // Shuffle vb (b1,b0,b3,b2).
        vb = _mm256_shuffle_pd(vb, vb, 0x5);

        vtmp = _mm256_mul_pd(va0_3, vb);
        va0_3b_1 = _mm256_add_pd(va0_3b_1, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb);
        va4_7b_1 = _mm256_add_pd(va4_7b_1, vtmp);

        // Permute vb (b3,b2,b1,b0).
        vb = _mm256_permute2f128_pd(vb, vb, 0x1);

        vtmp = _mm256_mul_pd(va0_3, vb);
        va0_3b_2 = _mm256_add_pd(va0_3b_2, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb);
        va4_7b_2 = _mm256_add_pd(va4_7b_2, vtmp);

        // Shuffle vb (b3,b2,b1,b0).
        vb = _mm256_shuffle_pd(vb, vb, 0x5);

        vtmp = _mm256_mul_pd(va0_3, vb);
        va0_3b_3 = _mm256_add_pd(va0_3b_3, vtmp);

        vtmp = _mm256_mul_pd(va4_7, vb);
        va4_7b_3 = _mm256_add_pd(va4_7b_3, vtmp);

        a += 8;
        b += 4;

    }

    __m256d vtmpa_0_3b_0 = _mm256_blend_pd(va0_3b_0, va0_3b_1, 0x6);
    __m256d vtmpa_0_3b_1 = _mm256_blend_pd(va0_3b_1, va0_3b_0, 0x6);

    __m256d vtmpa_0_3b_2 = _mm256_blend_pd(va0_3b_2, va0_3b_3, 0x6);
    __m256d vtmpa_0_3b_3 = _mm256_blend_pd(va0_3b_3, va0_3b_2, 0x6);

    __m256d vtmpa_4_7b_0 = _mm256_blend_pd(va4_7b_0, va4_7b_1, 0x6);
    __m256d vtmpa_4_7b_1 = _mm256_blend_pd(va4_7b_1, va4_7b_0, 0x6);

    __m256d vtmpa_4_7b_2 = _mm256_blend_pd(va4_7b_2, va4_7b_3, 0x6);
    __m256d vtmpa_4_7b_3 = _mm256_blend_pd(va4_7b_3, va4_7b_2, 0x6);

    va0_3b0 = _mm256_permute2f128_pd(vtmpa_0_3b_0, vtmpa_0_3b_2, 0x30);
    va0_3b3 = _mm256_permute2f128_pd(vtmpa_0_3b_2, vtmpa_0_3b_0, 0x30);

    va0_3b1 = _mm256_permute2f128_pd(vtmpa_0_3b_1, vtmpa_0_3b_3, 0x30);
    va0_3b2 = _mm256_permute2f128_pd(vtmpa_0_3b_3, vtmpa_0_3b_1, 0x30);

    va4_7b0 = _mm256_permute2f128_pd(vtmpa_4_7b_0, vtmpa_4_7b_2, 0x30);
    va4_7b3 = _mm256_permute2f128_pd(vtmpa_4_7b_2, vtmpa_4_7b_0, 0x30);

    va4_7b1 = _mm256_permute2f128_pd(vtmpa_4_7b_1, vtmpa_4_7b_3, 0x30);
    va4_7b2 = _mm256_permute2f128_pd(vtmpa_4_7b_3, vtmpa_4_7b_1, 0x30);

    if (rs_c == 1) {
        // Calculate address.
        c00 = (c + 0 * rs_c + 0 * cs_c);

        // Load.
        vc0_3_0 = _mm256_load_pd(c00);
        // Add gemm result.
        vc0_3_0 = _mm256_add_pd(vc0_3_0, va0_3b0);
        // Store back to memory.
        _mm256_store_pd(c00, vc0_3_0);

        // Calculate address.
        c40 = (c + 4 * rs_c + 0 * cs_c);

        // Load.
        vc4_7_0 = _mm256_load_pd(c40);
        // Add gemm result.
        vc4_7_0 = _mm256_add_pd(vc4_7_0, va4_7b0);
        // Store back to memory.
        _mm256_store_pd(c40, vc4_7_0);

        // Calculate address.
        c01 = (c + 0 * rs_c + 1 * cs_c);

        // Load.
        vc0_3_1 = _mm256_load_pd(c01);
        // Add gemm result.
        vc0_3_1 = _mm256_add_pd(vc0_3_1, va0_3b1);
        // Store back to memory.
        _mm256_store_pd(c01, vc0_3_1);

        // Calculate address.
        c41 = (c + 4 * rs_c + 1 * cs_c);

        // Load.
        vc4_7_1 = _mm256_load_pd(c41);
        // Add gemm result.
        vc4_7_1 = _mm256_add_pd(vc4_7_1, va4_7b1);
        // Store back to memory.
        _mm256_store_pd(c41, vc4_7_1);

        // Calculate address.
        c02 = (c + 0 * rs_c + 2 * cs_c);

        // Load.
        vc0_3_2 = _mm256_load_pd(c02);
        // Add gemm result.
        vc0_3_2 = _mm256_add_pd(vc0_3_2, va0_3b2);
        // Store back to memory.
        _mm256_store_pd(c02, vc0_3_2);

        // Calculate address.
        c42 = (c + 4 * rs_c + 2 * cs_c);

        // Load.
        vc4_7_2 = _mm256_load_pd(c42);
        // Add gemm result.
        vc4_7_2 = _mm256_add_pd(vc4_7_2, va4_7b2);
        // Store back to memory.
        _mm256_store_pd(c42, vc4_7_2);

        // Calculate address.
        c03 = (c + 0 * rs_c + 3 * cs_c);

        // Load.
        vc0_3_3 = _mm256_load_pd(c03);
        // Add gemm result.
        vc0_3_3 = _mm256_add_pd(vc0_3_3, va0_3b3);
        // Store back to memory.
        _mm256_store_pd(c03, vc0_3_3);

        // Calculate address.
        c43 = (c + 4 * rs_c + 3 * cs_c);

        // Load.
        vc4_7_3 = _mm256_load_pd(c43);
        // Add gemm result.
        vc4_7_3 = _mm256_add_pd(vc4_7_3, va4_7b3);
        // Store back to memory.
        _mm256_store_pd(c43, vc4_7_3);
    } else {
        // Load.
        // vc0_3_0 = _mm256_load_pd(c + 0 * rs_c + 0 * cs_c);
        vc0_3_0 = _mm256_set_pd(*(c + 3 * rs_c + 0 * cs_c),
                                *(c + 2 * rs_c + 0 * cs_c),
                                *(c + 1 * rs_c + 0 * cs_c),
                                *(c + 0 * rs_c + 0 * cs_c));
        // Add gemm result.
        vc0_3_0 = _mm256_add_pd(vc0_3_0, va0_3b0);
        // Store back to memory.
        // _mm256_store_pd(c00, vc0_3_0);
        aa = _mm256_extractf128_pd(vc0_3_0, 0);
        bb = _mm256_extractf128_pd(vc0_3_0, 1);

        _mm_storel_pd(c + 0 * rs_c + 0 * cs_c, aa);
        _mm_storeh_pd(c + 1 * rs_c + 0 * cs_c, aa);
        _mm_storel_pd(c + 2 * rs_c + 0 * cs_c, bb);
        _mm_storeh_pd(c + 3 * rs_c + 0 * cs_c, bb);

        // Load.
        // vc4_7_0 = _mm256_load_pd(c + 4 * rs_c + 0 * cs_c);
        vc4_7_0 = _mm256_set_pd(*(c + 7 * rs_c + 0 * cs_c),
                                *(c + 6 * rs_c + 0 * cs_c),
                                *(c + 5 * rs_c + 0 * cs_c),
                                *(c + 4 * rs_c + 0 * cs_c));
        // Add gemm result.
        vc4_7_0 = _mm256_add_pd(vc4_7_0, va4_7b0);
        // Store back to memory.
        // _mm256_store_pd(c40, vc4_7_0);

        aa = _mm256_extractf128_pd(vc4_7_0, 0) ;
        bb = _mm256_extractf128_pd(vc4_7_0, 1) ;

        _mm_storel_pd(c + 4 * rs_c + 0 * cs_c, aa);
        _mm_storeh_pd(c + 5 * rs_c + 0 * cs_c, aa);
        _mm_storel_pd(c + 6 * rs_c + 0 * cs_c, bb);
        _mm_storeh_pd(c + 7 * rs_c + 0 * cs_c, bb);

        // Load.
        // vc0_3_1 = _mm256_load_pd(c + 0 * rs_c + 1 * cs_c);
        vc0_3_1 = _mm256_set_pd(*(c + 3 * rs_c + 1 * cs_c),
                                *(c + 2 * rs_c + 1 * cs_c),
                                *(c + 1 * rs_c + 1 * cs_c),
                                *(c + 0 * rs_c + 1 * cs_c));
        // Add gemm result.
        vc0_3_1 = _mm256_add_pd(vc0_3_1, va0_3b1);
        // Store back to memory.
        //_mm256_store_pd(c01, vc0_3_1);

        aa = _mm256_extractf128_pd(vc0_3_1, 0) ;
        bb = _mm256_extractf128_pd(vc0_3_1, 1) ;

        _mm_storel_pd(c + 0 * rs_c + 1 * cs_c, aa);
        _mm_storeh_pd(c + 1 * rs_c + 1 * cs_c, aa);
        _mm_storel_pd(c + 2 * rs_c + 1 * cs_c, bb);
        _mm_storeh_pd(c + 3 * rs_c + 1 * cs_c, bb);

        // Load.
        // vc4_7_1 = _mm256_load_pd(c + 4 * rs_c + 1 * cs_c);
        vc4_7_1 = _mm256_set_pd(*(c + 7 * rs_c + 1 * cs_c),
                                *(c + 6 * rs_c + 1 * cs_c),
                                *(c + 5 * rs_c + 1 * cs_c),
                                *(c + 4 * rs_c + 1 * cs_c));
        // Add gemm result.
        vc4_7_1 = _mm256_add_pd(vc4_7_1, va4_7b1);
        // Store back to memory.
        // _mm256_store_pd(c41, vc4_7_1);

        aa = _mm256_extractf128_pd(vc4_7_1, 0) ;
        bb = _mm256_extractf128_pd(vc4_7_1, 1) ;

        _mm_storel_pd(c + 4 * rs_c + 1 * cs_c, aa);
        _mm_storeh_pd(c + 5 * rs_c + 1 * cs_c, aa);
        _mm_storel_pd(c + 6 * rs_c + 1 * cs_c, bb);
        _mm_storeh_pd(c + 7 * rs_c + 1 * cs_c, bb);

        // Load.
        // vc0_3_2 = _mm256_load_pd(c + 0 * rs_c + 2 * cs_c);
        vc0_3_2 = _mm256_set_pd(*(c + 3 * rs_c + 2 * cs_c),
                                *(c + 2 * rs_c + 2 * cs_c),
                                *(c + 1 * rs_c + 2 * cs_c),
                                *(c + 0 * rs_c + 2 * cs_c));
        // Add gemm result.
        vc0_3_2 = _mm256_add_pd(vc0_3_2, va0_3b2);
        // Store back to memory.
        // _mm256_store_pd(c02, vc0_3_2);

        aa = _mm256_extractf128_pd(vc0_3_2, 0) ;
        bb = _mm256_extractf128_pd(vc0_3_2, 1) ;

        _mm_storel_pd(c + 0 * rs_c + 2 * cs_c, aa);
        _mm_storeh_pd(c + 1 * rs_c + 2 * cs_c, aa);
        _mm_storel_pd(c + 2 * rs_c + 2 * cs_c, bb);
        _mm_storeh_pd(c + 3 * rs_c + 2 * cs_c, bb);

        // Load.
        // vc4_7_2 = _mm256_load_pd(c + 4 * rs_c + 2 * cs_c);
        vc4_7_2 = _mm256_set_pd(*(c + 7 * rs_c + 2 * cs_c),
                                *(c + 6 * rs_c + 2 * cs_c),
                                *(c + 5 * rs_c + 2 * cs_c),
                                *(c + 4 * rs_c + 2 * cs_c));
        // Add gemm result.
        vc4_7_2 = _mm256_add_pd(vc4_7_2, va4_7b2);
        // Store back to memory.
        // _mm256_store_pd(c42, vc4_7_2);

        aa = _mm256_extractf128_pd(vc4_7_2, 0);
        bb = _mm256_extractf128_pd(vc4_7_2, 1);

        _mm_storel_pd(c + 4 * rs_c + 2 * cs_c, aa);
        _mm_storeh_pd(c + 5 * rs_c + 2 * cs_c, aa);
        _mm_storel_pd(c + 6 * rs_c + 2 * cs_c, bb);
        _mm_storeh_pd(c + 7 * rs_c + 2 * cs_c, bb);

        // Load.
        // vc0_3_3 = _mm256_load_pd(c + 0 * rs_c + 3 * cs_c);
        vc0_3_3 = _mm256_set_pd(*(c + 3 * rs_c + 3 * cs_c),
                                *(c + 2 * rs_c + 3 * cs_c),
                                *(c + 1 * rs_c + 3 * cs_c),
                                *(c + 0 * rs_c + 3 * cs_c));
        // Add gemm result.
        vc0_3_3 = _mm256_add_pd(vc0_3_3, va0_3b3);
        // Store back to memory.
        // _mm256_store_pd(c03, vc0_3_3);

        aa = _mm256_extractf128_pd(vc0_3_3, 0);
        bb = _mm256_extractf128_pd(vc0_3_3, 1);

        _mm_storel_pd(c + 0 * rs_c + 3 * cs_c, aa);
        _mm_storeh_pd(c + 1 * rs_c + 3 * cs_c, aa);
        _mm_storel_pd(c + 2 * rs_c + 3 * cs_c, bb);
        _mm_storeh_pd(c + 3 * rs_c + 3 * cs_c, bb);

        // Load.
        // vc4_7_3 = _mm256_load_pd(c + 4 * rs_c + 3 * cs_c);
        vc4_7_3 = _mm256_set_pd(*(c + 7 * rs_c + 3 * cs_c),
                                *(c + 6 * rs_c + 3 * cs_c),
                                *(c + 5 * rs_c + 3 * cs_c),
                                *(c + 4 * rs_c + 3 * cs_c));
        // Add gemm result.
        vc4_7_3 = _mm256_add_pd(vc4_7_3, va4_7b3);
        // Store back to memory.
        //_mm256_store_pd(c43, vc4_7_3);

        aa = _mm256_extractf128_pd(vc4_7_3, 0);
        bb = _mm256_extractf128_pd(vc4_7_3, 1);

        _mm_storel_pd(c + 4 * rs_c + 3 * cs_c, aa);
        _mm_storeh_pd(c + 5 * rs_c + 3 * cs_c, aa);
        _mm_storel_pd(c + 6 * rs_c + 3 * cs_c, bb);
        _mm_storeh_pd(c + 7 * rs_c + 3 * cs_c, bb);
    }
}

/**
 * Kernel for the multiplication of blocks of A and B.
 * We assume that these blocks were previously packed to buffers A and B.
 */
static void GEMMKernel(int mc, int nc, int kc, int inc_row_c, double* c) {
    int mp = (mc + ROW_PACK - 1) / ROW_PACK;
    int np = (nc + COL_PACK - 1) / COL_PACK;

    int _mr = mc % ROW_PACK;
    int _nr = nc % COL_PACK;

    #pragma omp parallel for
    for (int j = 0; j < np; ++j) {
        int nr = (j != np - 1 || _nr == 0) ? COL_PACK : _nr;
        const double* next_b = &BUFFER_B[j * kc * COL_PACK];

        for (int i = 0; i < mp; ++i) {
            int mr = (i != mp - 1 || _mr == 0) ? ROW_PACK : _mr;
            if (i == mp - 1) {
                next_b = &BUFFER_B[(j + 1) * kc * COL_PACK];
                if (j == np - 1) {
                    next_b = BUFFER_B;
                }
            }

            if (mr == ROW_PACK && nr == COL_PACK) {
                GEMMMicroKernel(kc,
                                &BUFFER_A[i * kc * ROW_PACK],
                                &BUFFER_B[j * kc * COL_PACK],
                                inc_row_c,
                                1,
                                next_b,
                                &c[i * ROW_PACK * inc_row_c + j * COL_PACK]);
            } else {
                double buffer_c[ROW_PACK * COL_PACK]
                        __attribute__ ((aligned (32)));
                std::memset(buffer_c, 0, sizeof(buffer_c));
                GEMMMicroKernel(kc,
                                &BUFFER_A[i * kc * ROW_PACK],
                                &BUFFER_B[j * kc * COL_PACK],
                                1,
                                ROW_PACK,
                                next_b,
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
 *   c = a * b
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
inline void DGEMM(int m, int n, int k, const double* a, const double* b,
                  double* c) {
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

} // namespace dgemm
} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_DGEMM_H_

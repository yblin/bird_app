//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_CHECK_H_
#define CODELIBRARY_CUDA_CHECK_H_

#include "codelibrary/base/log.h"

/**
 * Check the result of a cudaXXXXXX call.
 */
#define CUDA_CHECK(x)                                               \
    do {                                                            \
        cudaError_t result = x;                                     \
        CHECK(result == cudaSuccess) << cudaGetErrorString(result); \
    } while (0)

#endif // CODELIBRARY_CUDA_CHECK_H_

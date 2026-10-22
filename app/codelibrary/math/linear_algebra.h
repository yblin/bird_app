//
// Copyright 2023-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Implement Basic Linear Algebra Subroutines (BLAS) routines:
//   [1]. Scale, x = a*x
//   [2]. AXPY, y = a*x + y
//   [3]. Dot product
//   [4]. Euclidean norm
//   [5]. ASum, sum of absolute values
//   [6]. Matrix-vector product.
//
// And BLAS-like routines:
//   [1]. Unary negate for vector
//   [2]. Add two vectors.
//   [3]. Subtract two vectors.
//   [4]. Element multiplication of two vectors.
//

#ifndef CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_
#define CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <memory>

#include "codelibrary/base/log.h"

namespace cl {
namespace blas {

/**
 * Computes the product of a vector by a scalar, which is defined as:
 *
 *   b *= a.
 *
 * Input:
 *  n - Specify the number of elements in vector b.
 *  a - Specify the scalar a.
 *  b - Array, size at least n.
 *
 * Output:
 *  b - Update vector b.
 */
template <typename T>
void Scale(int n, T a, T* b) {
    switch (n) {
    case 0: return;
    case 1: b[0] *= a;
            return;
    case 2: b[0] *= a;
            b[1] *= a;
            return;
    case 3: b[0] *= a;
            b[1] *= a;
            b[2] *= a;
            return;
    case 4: b[0] *= a;
            b[1] *= a;
            b[2] *= a;
            b[3] *= a;
            return;
    }

    T* p = b;

    int i = 0;
    for (; i + 3 < n; p += 4, i += 4) {
        *(p)     *= a;
        *(p + 1) *= a;
        *(p + 2) *= a;
        *(p + 3) *= a;
    }
    for (; i < n; ++p, ++i)
        *(p) *= a;
}

/**
 * Constant times a vector plus a vector, which is defined as:
 *
 *    c += a * b.
 *
 * Input:
 *  n - Specify the number of elements in vector b.
 *  a - Specify the scalar a.
 *  b - Array, size at least n.
 *
 * Output:
 *  c - Update vector b.
 */
template <typename T>
void AXPY(int n, T a, const T* b, T* c) {
    switch (n) {
    case 0: return;
    case 1: c[0] += a * b[0];
            return;
    case 2: c[0] += a * b[0];
            c[1] += a * b[1];
            return;
    case 3: c[0] += a * b[0];
            c[1] += a * b[1];
            c[2] += a * b[2];
            return;
    case 4: c[0] += a * b[0];
            c[1] += a * b[1];
            c[2] += a * b[2];
            c[3] += a * b[3];
            return;
    }

    const T* p1 = b;

    T* p = c;
    int i = 0;
    for (; i + 3 < n; p1 += 4, p += 4, i += 4) {
        *(p)     += a * *(p1);
        *(p + 1) += a * *(p1 + 1);
        *(p + 2) += a * *(p1 + 2);
        *(p + 3) += a * *(p1 + 3);
    }
    for (; i < n; ++p1, ++p, ++i)
        *(p) += a * *(p1);
}

/**
 * Compute a vector-vector dot product.
 *
 * Input:
 *  n - Specify the number of elements in vectors x and y.
 *  a - Array, size at least n.
 *  b - Array, size at least n.
 *
 * Return:
 *  The result of the dot product of a and b, if n is positive.
 *  Otherwise, return 0.
 */
template <typename T>
T DotProduct(int n, const T* a, const T* b) {
    if (n <= 0) return 0;

    switch (n) {
    case 1: return a[0] * b[0];
    case 2: return a[0] * b[0] + a[1] * b[1];
    case 3: return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    case 4: return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    T s = 0;
    const T* p1 = a;
    const T* p2 = b;

    int i = 0;
    for (; i + 3 < n; p1 += 4, p2 += 4, i += 4) {
        s += *(p1)     * *(p2);
        s += *(p1 + 1) * *(p2 + 1);
        s += *(p1 + 2) * *(p2 + 2);
        s += *(p1 + 3) * *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++i)
        s += *(p1) * *(p2);
    return s;
}

/**
 * Compute the Euclidean norm of a vector.
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Array, size at least n.
 *
 * Return:
 *  The Euclidean norm of the vector a.
 */
template <typename T>
T EuclideanNorm(int n, const T* a) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return 0;
    case 1: return std::sqrt(a[0] * a[0]);
    case 2: return std::sqrt(a[0] * a[0] + a[1] * a[1]);
    case 3: return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    case 4: return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] +
                             a[3] * a[3]);
    }

    T sum = 0;
    const T* p = a;

    int i = 0;
    for (; i + 3 < n; p += 4, i += 4) {
        sum += *(p)     * *(p);
        sum += *(p + 1) * *(p + 1);
        sum += *(p + 2) * *(p + 2);
        sum += *(p + 3) * *(p + 3);
    }
    for (; i < n; ++p, ++i)
        sum += *(p) * *(p);

    return std::sqrt(sum);
}

/**
 * Compute the sum of absolute values of a vector.
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Array, size at least n.
 *
 * Return:
 *  The sum of absolute values of the vector a.
 */
template <typename T>
T ASum(int n, const T* a) {
    switch (n) {
    case 0: return 0;
    case 1: return std::abs(a[0]);
    case 2: return std::abs(a[0]) + std::abs(a[1]);
    case 3: return std::abs(a[0]) + std::abs(a[1]) + std::abs(a[2]);
    case 4: return std::abs(a[0]) + std::abs(a[1]) + std::abs(a[2]) +
                   std::abs(a[3]);
    }

    T sum = 0;
    const T* p = a;

    int i = 0;
    for (; i + 3 < n; p += 4, i += 4) {
        sum += std::abs(*(p)) + std::abs(*(p + 1)) +
               std::abs(*(p + 2)) + std::abs(*(p + 3));
    }
    for (; i < n; ++p, ++i)
        sum += std::abs(*p);

    return sum;
}

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
        for (int i = 0; i < m; ++i) {
            c[0] += a[i] * b[i];
        }
        return;
    case 2:
        for (int i = 0; i < m; ++i) {
            c[0] += a[i * 2]     * b[i];
            c[1] += a[i * 2 + 1] * b[i];
        }
        return;
    case 3:
        for (int i = 0; i < m; ++i) {
            c[0] += a[i * 3]     * b[i];
            c[1] += a[i * 3 + 1] * b[i];
            c[2] += a[i * 3 + 2] * b[i];
        }
        return;
    case 4:
        for (int i = 0; i < m; ++i) {
            c[0] += a[i * 4]     * b[i];
            c[1] += a[i * 4 + 1] * b[i];
            c[2] += a[i * 4 + 2] * b[i];
            c[3] += a[i * 4 + 3] * b[i];
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

/**
 * In-place negate, which is defined as:
 *
 *  a = -a
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Pointer to array that contains the input vector a.
 *
 * Output:
 *  a - Update vector a.
 */
template <typename T>
void Negate(int n, T* a) {
    switch (n) {
    case 0: return;
    case 1: a[0] = -a[0];
            return;
    case 2: a[0] = -a[0];
            a[1] = -a[1];
            return;
    case 3: a[0] = -a[0];
            a[1] = -a[1];
            a[2] = -a[2];
            return;
    case 4: a[0] = -a[0];
            a[1] = -a[1];
            a[2] = -a[2];
            a[3] = -a[3];
            return;
    }

    T* p = a;

    int i = 0;
    for (; i + 3 < n; p += 4, i += 4) {
        *(p)     = -*(p);
        *(p + 1) = -*(p + 1);
        *(p + 2) = -*(p + 2);
        *(p + 3) = -*(p + 3);
    }
    for (; i < n; ++p, ++i)
        *(p) = -*(p);
}

/**
 * Perform element by element addition of two vectors, which is defined as:
 *
 *  c = a + b
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Add(int n, const T* a, const T* b, T* c) {
    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] + b[0];
            return;
    case 2: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            return;
    case 3: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            c[2] = a[2] + b[2];
            return;
    case 4: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            c[2] = a[2] + b[2];
            c[3] = a[3] + b[3];
            return;
    }

    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 3 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     + *(p2);
        *(p + 1) = *(p1 + 1) + *(p2 + 1);
        *(p + 2) = *(p1 + 2) + *(p2 + 2);
        *(p + 3) = *(p1 + 3) + *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) + *(p2);
}

/**
 * Perform element by element subtraction of vector a and vector b.
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Subtract(int n, const T* a, const T* b, T* c) {
    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] - b[0];
            return;
    case 2: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            return;
    case 3: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            c[2] = a[2] - b[2];
            return;
    case 4: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            c[2] = a[2] - b[2];
            c[3] = a[3] - b[3];
            return;
    }
    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 3 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     - *(p2);
        *(p + 1) = *(p1 + 1) - *(p2 + 1);
        *(p + 2) = *(p1 + 2) - *(p2 + 2);
        *(p + 3) = *(p1 + 3) - *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) - *(p2);
}

/**
 * Perform element by element multiplication of vector a and vector b.
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Multiply(int n, const T* a, const T* b, T* c) {
    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] * b[0];
            return;
    case 2: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            return;
    case 3: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            c[2] = a[2] * b[2];
            return;
    case 4: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            c[2] = a[2] * b[2];
            c[3] = a[3] * b[3];
            return;
    }

    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 3 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     * *(p2);
        *(p + 1) = *(p1 + 1) * *(p2 + 1);
        *(p + 2) = *(p1 + 2) * *(p2 + 2);
        *(p + 3) = *(p1 + 3) * *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) * *(p2);
}

} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_

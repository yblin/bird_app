//
// Copyright 2019-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// I don't want to implement a complete sparse matrix library, but just provide
// some simple data sturctures as the interface for more complicate third party
// libraries.
//

#ifndef CODELIBRARY_MATH_MATRIX_SPARSE_MATRIX_H_
#define CODELIBRARY_MATH_MATRIX_SPARSE_MATRIX_H_

#include "codelibrary/base/array.h"
#include "codelibrary/math/vector.h"

namespace cl {

/**
 * Sparse matrix in coordinate storage format.
 *
 * The coordinate format is the most flexible and simplest format for the sparse
 * matrix representation. Only non-zero elements are stored, and the coordinates
 * of each non-zero element are given explicitly.
 *
 * The coordinate format is specified by three arrays: values, rows, and
 * columns.
 *
 * All three arrays have the same dimension, nnz, representing the number of
 * non-zero elements in the matrix. The following list describes the arrays in
 * terms of the values, row, and column positions of the non-zero elements.
 *
 * values:  A real array that contains the non-zero elements of A in any order.
 * rows:    Element i of the integer array rows is the number of the row in A
 *          that contains the i-th value in the values array.
 * columns: Element i of the integer array columns is the number of the columns
 *          in A that contains the i-th value in the values array.
 *
 * Note that the coordinate format is zero-based indexing.
 *
 * For example, the sparse matrix C:
 *         1 -1 -3  0  0
 *        -2  5  0  0  0
 *  C = [  0  0  4  6  4  ]
 *        -4  0  2  7  0
 *         0  8  0  0 -5
 *
 * can be represented in the coordinate format as follows:
 *
 * values  = (1 -1 -3 -2  5  4  6  4 -4  2  7  8 -5)
 * rows    = (0  0  0  1  1  2  2  2  3  3  3  4  4)
 * columns = (0  1  2  0  1  2  3  4  0  2  3  1  4)
 */
template <typename T>
struct CoordinateSparseMatrix {
    int n_rows = 0;
    int n_columns = 0;
    int n_elements = 0;
    Array<T> values;
    Array<int> rows;
    Array<int> columns;

    /**
     * Check if the matrix is valid.
     */
    void Check() const {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_elements >= 0);
        CHECK(values.size()  == n_elements);
        CHECK(rows.size()    == n_elements);
        CHECK(columns.size() == n_elements);
        for (int i = 0; i < n_elements; ++i) {
            CHECK(rows[i] >= 0);
            CHECK(rows[i] < n_rows);
            CHECK(columns[i] >= 0);
            CHECK(columns[i] < n_columns);
        }
    }

    /**
     * Clear the data.
     */
    void clear() {
        n_rows = 0;
        n_columns = 0;
        n_elements = 0;
        values.clear();
        rows.clear();
        columns.clear();
    }
};

/**
 * Sparse matrix in compressed sparse row (CSR) format.
 *
 * For example, the sparse matrix C:
 *         1 -1 -3  0  0
 *        -2  5  0  0  0
 *  C = [  0  0  4  6  4  ]
 *        -4  0  2  7  0
 *         0  8  0  0 -5
 *
 * can be represented in the coordinate format as follows:
 *
 * values    = (1 -1 -3 -2  5  4  6  4 -4  2  7  8 -5)
 * columns   = (0  1  2  0  1  2  3  4  0  2  3  1  4)
 * row_start = (0  3  5  8  11)
 * row_end   = (3  5  8 11  13)
 *
 * where, columns[i] stores the column of the i-th element, row_start[i] is the
 * first element in i-th row, and row_end[i] is the last element in i-th row..
 */
template <typename T>
struct CSRSparseMatrix {
    int n_rows = 0;
    int n_columns = 0;
    int n_elements = 0;
    Array<T> values;
    Array<int> columns;
    Array<int> row_start;
    Array<int> row_end;

    /**
     * Check if the matrix is valid.
     */
    void Check() const {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_elements >= 0);
        CHECK(values.size() == n_elements);
        CHECK(row_start.size() == n_rows);
        CHECK(row_end.size() == n_rows);
        CHECK(columns.size() == n_elements);

        if (n_rows > 0) {
            CHECK(row_start.front() == 0);
            CHECK(row_end.back() == n_elements);
        }
        for (int i = 0; i < n_rows; ++i) {
            CHECK(row_start[i] >= 0);
            CHECK(row_start[i] <= row_end[i]);
            if (i > 0) {
                CHECK(row_start[i] >= row_start[i - 1]);
                CHECK(row_end[i] >= row_end[i - 1]);
            }
        }

        for (int i = 0; i < n_columns; ++i) {
            CHECK(columns[i] >= 0 && columns[i] < n_columns);
        }
    }

    /**
     * Clear the data.
     */
    void clear() {
        n_rows = 0;
        n_columns = 0;
        n_elements = 0;
        values.clear();
        row_start.clear();
        row_end.clear();
        columns.clear();
    }
};

/**
 * Three array variation of CSR format.
 *
 * It uses rows to replace row_start and row_end. We have:
 *   rows[0, n_rows) = row_start[0, n_rows) and
 *   rows[1, n_rows] = row_end(0, n_rows)
 */
template <typename T>
struct CSR3SparseMatrix {
    int n_rows = 0;
    int n_columns = 0;
    int n_elements = 0;
    Array<T> values;
    Array<int> columns;
    Array<int> rows;

    /**
     * Check if the matrix is valid.
     */
    void Check() const {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(values.size() == n_elements);
        CHECK(columns.size() == n_elements);

        if (n_rows > 0) {
            CHECK(rows.size() == n_rows + 1);
            CHECK(rows.front() == 0);
            CHECK(rows.back() == n_elements);
        }
        for (int i = 0; i <= n_rows; ++i) {
            CHECK(rows[i] >= 0);
            CHECK(rows[i] <= n_elements);
            if (i > 0) {
                CHECK(rows[i] >= rows[i - 1]);
            }
        }

        for (int i = 0; i < n_columns; ++i) {
            CHECK(columns[i] >= 0 && columns[i] < n_columns);
        }
    }

    /**
     * Clear the data.
     */
    void clear() {
        n_rows = 0;
        n_columns = 0;
        n_elements = 0;
        values.clear();
        rows.clear();
        columns.clear();
    }
};

/**
 * Return the product of matrix and vector.
 */
template <typename T>
Vector<T> operator*(const CSR3SparseMatrix<T>& a, const Vector<T>& b) {
    CHECK(b.size() == a.n_columns);

    Vector<T> res(a.n_rows, 0);
    if (a.n_elements) {
        for (int r = 0; r < a.n_rows; ++r) {
            for (int j = a.rows[r]; j < a.rows[r + 1]; ++j) {
                int c = a.columns[j];
                res[r] += b[c] * a.values[j];
            }
        }
    }
    return res;
}
template <typename T>
Vector<T> operator *(const Vector<T>& a, const CSR3SparseMatrix<T>& b) {
    CHECK(a.size() == b.n_rows);

    Vector<T> res(b.n_columns, 0);
    if (b.n_elements) {
        for (int r = 0; r < b.n_rows; ++r) {
            for (int j = b.rows[r]; j < b.rows[r + 1]; ++j) {
                int c = b.columns[j];
                res[c] += a[r] * b.values[j];
            }
        }
    }
    return res;
}

namespace sparse_matrix {

/**
 * Convert Coordinate-Sparse-Matrix to CSR3 format.
 */
template <typename T>
void Convert(const CoordinateSparseMatrix<T>& coord,
             CSR3SparseMatrix<T>* csr3) {
    CHECK(csr3);

    int n_rows = coord.n_rows;
    int n_cols = coord.n_columns;
    CHECK(n_rows >= 0);
    CHECK(n_cols >= 0);

    csr3->n_rows = n_rows;
    csr3->n_columns = n_cols;

    // Sort the coordinates according to their rows.
    Array<int> bucket(n_rows, 0);
    for (int i = 0; i < coord.n_elements; ++i) {
        ++bucket[coord.rows[i]];
    }

    csr3->rows.resize(n_rows + 1);
    csr3->rows[0] = 0;
    for (int i = 0; i < n_rows; ++i) {
        csr3->rows[i + 1] = csr3->rows[i] + bucket[i];
        bucket[i] = csr3->rows[i];
    }

    csr3->columns.resize(coord.n_elements);
    csr3->values.resize(coord.n_elements);
    for (int i = 0; i < coord.n_elements; ++i) {
        int p = bucket[coord.rows[i]]++;
        csr3->columns[p] = coord.columns[i];
        csr3->values[p] = coord.values[i];
    }

    // Note that the CoordinateSparseMatrix format may have duplicate entries.
    // They can be compressed by the following code.
    Array<int> w(n_cols, -1);
    csr3->n_elements = 0;
    for (int r = 0; r < n_rows; ++r) {
        int q = csr3->n_elements;
        for (int p = csr3->rows[r]; p < csr3->rows[r + 1]; ++p) {
            int c = csr3->columns[p];
            if (w[c] >= q) {
                csr3->values[w[c]] += csr3->values[p];
            } else {
                w[c] = csr3->n_elements;
                csr3->columns[csr3->n_elements] = c;
                csr3->values[csr3->n_elements++] = csr3->values[p];
            }
        }
        csr3->rows[r] = q;
    }
    csr3->rows[n_rows] = csr3->n_elements;
    csr3->columns.resize(csr3->n_elements);
    csr3->values.resize(csr3->n_elements);
}

/**
 * Convert CSR format to CSR3 format.
 */
template <typename T>
void Convert(const CSRSparseMatrix<T>& csr, CSR3SparseMatrix<T>* csr3) {
    CHECK(csr3);

    int n_rows = csr.n_rows;
    int n_cols = csr.n_columns;
    CHECK(n_rows >= 0);
    CHECK(n_cols >= 0);

    csr3->n_rows = n_rows;
    csr3->n_columns = n_cols;

    if (n_rows > 0 && n_cols > 0) {
        csr3->n_elements = csr.n_elements;
        csr3->values = csr.values;
        csr3->columns = csr.columns;
        csr3->rows.resize(n_rows + 1);
        for (int i = 0; i < n_rows; ++i) {
            csr3->rows[i] = csr.rows[i];
        }
        csr3->rows[n_rows] = csr3->n_elements;
    } else {
        csr3->n_elements = 0;
    }
}

/**
 * Convert CSR3 format to CSR format.
 */
template <typename T>
void Convert(const CSR3SparseMatrix<T>& csr3, CSRSparseMatrix<T>* csr) {
    CHECK(csr);

    int n_rows = csr3.n_rows;
    int n_cols = csr3.n_columns;
    CHECK(n_rows >= 0);
    CHECK(n_cols >= 0);

    csr->n_rows = n_rows;
    csr->n_columns = n_cols;
    csr->n_elements = csr3.n_elements;
    csr->values = csr3.values;
    csr->columns = csr3.columns;
    csr->row_start.resize(n_rows);
    csr->row_end.resize(n_rows);
    for (int i = 0; i < n_rows; ++i) {
        csr->row_start[i] = csr3.rows[i];
        csr->row_end[i] = csr3.rows[i + 1];
    }
}

/**
 * Computes the squared norm for each column, returning a vector where the J-th
 * element is the squared norm of the J-th column.
 */
template <typename T>
void SquaredColumnNorm(const CSR3SparseMatrix<T>& csr3, Vector<T>* norm) {
    CHECK(norm);

    norm->assign(csr3.n_columns, 0.0);

    for (int row = 0; row < csr3.n_rows; ++row) {
        int start = csr3.rows[row];
        int end = csr3.rows[row + 1];

        for (int i = start; i < end; ++i) {
            int col = csr3.columns[i];
            (*norm)[col] += csr3.values[i] * csr3.values[i];
        }
    }
}

} // namespace sparse_matrix
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_SPARSE_MATRIX_H_

//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_LINEAR_PRECONDITIONER_H_
#define CODELIBRARY_OPTIMIZATION_LINEAR_PRECONDITIONER_H_

#include "codelibrary/math/matrix/sparse_matrix.h"
#include "codelibrary/math/vector.h"

namespace cl {
namespace optimization {

/**
 * Type of Preconditioner.
 */
enum PreconditionerType {
    // Trivial preconditioner - the identity matrix.
    IDENTITY,

    // Block diagonal of the Gauss-Newton Hessian.
    JACOBI
};

/**
 * In linear solver, a preconditioner P of a matrix A is a matrix such that P*A
 * has a smaller condition number than A. Here, preconditioning is performed in
 * a matrix-free fashion.
 *
 * The default preconditioner is an identity preconditioner.
 */
class Preconditioner {
public:
    Preconditioner() = default;

    virtual ~Preconditioner() = default;

    /**
     * Solve Mz = r.
     */
    virtual RVector Solve(const RVector& r) {
        return r;
    }
};

/**
 * Jacobi preconditioner for ConjugateGradient method.
 *
 * This class allows to approximately solve for (A'A + D'D) x = A'b problems
 * assuming A'A + D'D is a diagonal matrix.
 *
 * In other words, this preconditioner neglects all off diagonal entries.
 */
class JacobiPreconditioner : public Preconditioner {
public:
    explicit JacobiPreconditioner(const CSR3SparseMatrix<double>& a,
                                  const RVector* d = nullptr) {
        if (d) CHECK(d->size() == a.n_columns);
        Factorize(a, d);
    }

    /**
     * Solve Mz = r.
     */
    RVector Solve(const RVector& r) override {
        return inverse_diagonal_ * r;
    }

protected:
    /**
     * Compute the inverse squared-norm of each column of the matrix.
     */
    void Factorize(const CSR3SparseMatrix<double>& a, const RVector* d) {
        if (d) CHECK(a.n_columns == d->size());

        sparse_matrix::SquaredColumnNorm(a, &inverse_diagonal_);
        for (int j = 0; j < inverse_diagonal_.size(); ++j) {
            double sum = inverse_diagonal_[j];
            if (d) {
                sum += (*d)[j] * (*d)[j];
            }

            if (sum > 0.0)
                inverse_diagonal_[j] = 1.0 / sum;
            else
                inverse_diagonal_[j] = 1.0;
        }
    }

    // The diagonal entries are pre-inverted and stored into a dense vector.
    RVector inverse_diagonal_;
};


} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_LINEAR_PRECONDITIONER_H_

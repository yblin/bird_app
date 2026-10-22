//
// Copyright 2017-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_LINEAR_CGNR_SOLVER_H_
#define CODELIBRARY_OPTIMIZATION_LINEAR_CGNR_SOLVER_H_

#include "codelibrary/optimization/linear/conjugate_gradient_solver.h"

namespace cl {
namespace optimization {

/**
 * Since classical conjugate gradient solver can be only applied to the matrix
 * which is symmetric and positive-semdefinite. CNGR (Conjugate Gradient on the
 * Normal Equations) extends CG method to the general matrix. Given a general
 * matrix A, CGNR solves the following linear system:
 *
 *    A'A x = A'b
 *
 * A'A is a symmetric positive-semidefinite matrix for any A.
 *
 * CGNRSolver can also take a extra diagonal matrix D to solve the following
 * liner system:
 *
 *   (A'A + D'D) x = A'b
 *
 * It is the solution of the following least squares problem:
 *
 *   min ||Ax - b||^2 + ||Dx||^2
 *    x
 */
class CGNRSolver : public SparseLinearSolver {
    /**
     * Directly compute A'A is time consuming. Thus, we replace it to compute
     * y = Ax and z = A'y.
     */
    struct LinearOperator {
        LinearOperator(const CSR3SparseMatrix<double>& a, const RVector* d)
            : a_(a), d_(d) {}

        int n_rows() const {
            return a_.n_columns;
        }

        int n_columns() const {
            return a_.n_columns;
        }

        /**
         * Return (A'A + D'D) x.
         */
        friend RVector operator *(const LinearOperator& l, const RVector& x) {
            CHECK(l.a_.n_columns == x.size());

            // z = Ax.
            RVector z = l.a_ * x;

            // z = A'Ax
            z = z * l.a_;

            if (l.d_ != nullptr) {
                CHECK(l.d_->size() == z.size());
                for (int i = 0; i < z.size(); ++i) {
                    z[i] += (*l.d_)[i] * (*l.d_)[i] * x[i];
                }
            }

            return z;
        }

    private:
        CSR3SparseMatrix<double> a_;
        const RVector* d_;
    };

public:
    explicit CGNRSolver(const Options& options = Options())
        : SparseLinearSolver(options) {}

    /**
     * Solve the linear system: A'Ax = A'b or (A'A + D'D)x = A'b
     */
    Result Solve(const CSR3SparseMatrix<double>& a, const RVector& b,
                 RVector* x) const override {
        LinearOperator l(a, options_.diagonal);
        ConjugateGradientSolver solver(options_);
        return solver.Solve(l, b * a, x);
    }
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_LINEAR_CGNR_SOLVER_H_

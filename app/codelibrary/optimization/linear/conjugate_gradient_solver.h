//
// Copyright 2015-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_H_
#define CODELIBRARY_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_H_

#include "codelibrary/math/matrix/matrix.h"
#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/linear/linear_solver.h"

namespace cl {
namespace optimization {

/**
 * The conjugate gradient method is an iterative method for solving a linear
 * system of equations:
 *
 *    Ax = b
 *
 * where A is an n × n symmetric positive definite matrix.
 *
 * The performance of conjugate gradient solver is proportional to the square
 * root of the condition number of the matrix A.
 *
 * Optionally it can use a preconditioner to reduce the condition number of the
 * linear system and improve the convergence rate. Given a linear system, and a
 * preconditioner the preconditioned system is given by
 *
 *    M^-1 Ax = M^-1 b
 *
 * The resulting algorithm is known as Preconditioned Conjugate Gradients
 * algorithm (PCG) and its worst case complexity now depends on the condition
 * number of the preconditioned matrix.
 *
 * The preconditioner matrix has to be symmetric positive-definite.
 *
 * Reference:
 *   Nash S G. A survey of truncated-Newton methods[J]. Journal of
 *   Computational & Applied Mathematics, 2000, 124:45-59.
 */
class ConjugateGradientSolver : public LinearSolver {
public:
    explicit ConjugateGradientSolver(const Options& options = Options())
        : LinearSolver(options) {}

    /**
     * Solve the linear system Ax=b by conjugate gradient method.
     */
    template <class Matrix>
    Result Solve(const Matrix& a, const RVector& b, RVector* x) const {
        CHECK(a.n_rows() == a.n_columns());
        CHECK(a.n_rows() == b.size());
        CHECK(x);

        Result result;
        result.num_iterations = 0;
        int n_columns = a.n_columns();
        if (x->size() != n_columns) x->assign(n_columns, 0.0);

        const double norm_b = b.norm();
        if (norm_b == 0.0) {
            result.termination_type = SUCCESS;
            result.message = "Convergence. |b| = 0.";
            return result;
        }

        const double tol_r = options_.r_tolerance * norm_b;
        RVector tmp = a * (*x); // t = Ax
        RVector r = b - tmp;    // r0 = b - Ax.

        double norm_r = r.norm();
        if (options_.min_num_iterations == 0 && norm_r <= tol_r) {
            result.termination_type = SUCCESS;
            result.message = Format("Convergence. |r| = {:e} <= {:e}.",
                                    norm_r, tol_r);
            return result;
        }

        double rho = 1.0;

        // Initial value of the quadratic model Q = x'Ax - 2 * b'x.
        double q0 = -(x->dot(b + r));

        RVector p(n_columns), z(n_columns);

        for (result.num_iterations = 1; ; ++result.num_iterations) {
            // Apply precondition to solve Mz = r.
            if (options_.preconditioner) {
                z = options_.preconditioner->Solve(r);
            } else {
                z = r;
            }

            double last_rho = rho;
            rho = r.dot(z);
            if (rho == 0.0 || std::isinf(rho)) {
                result.termination_type = FAILURE;
                result.message = Format("Numerical failure. rho = r'z = {:e}.",
                                        rho);
                break;
            }

            if (result.num_iterations == 1) {
                p = z;
            } else {
                double beta = rho / last_rho;
                if (beta == 0.0 || std::isinf(beta)) {
                    result.termination_type = FAILURE;
                    result.message = Format(
                            "Numerical failure. beta = rho_n / rho_{{n-1}} = "
                            "{:e}, rho_n = {:e}, rho_{{n-1}} = {:e}",
                            beta, rho, last_rho);
                    break;
                }

                p = z + beta * p;
            }

            RVector& q = z;
            q = a * p;
            const double pq = p.dot(q);
            if (pq <= 0.0 || std::isinf(pq)) {
                result.termination_type = NO_CONVERGENCE;
                result.message = Format(
                    "Matrix is indefinite, no more progress can be made. "
                    "p'q = {:e}. |p| = {:e}, |q| = {:e}",
                    pq, p.norm(), q.norm());
                break;
            }

            const double alpha = rho / pq;
            if (alpha == 0.0 || std::isinf(alpha)) {
                result.termination_type = FAILURE;
                result.message = Format(
                      "Numerical failure. alpha = rho / pq = {:e}.", alpha);
                break;
            }

            *x += alpha * p;

            // Ideally we would just use the update r = r - alpha*q to keep
            // track of the residual vector. However this estimate tends to
            // drift over time due to round off errors. Thus every
            // residual_reset_period iterations, we calculate the residual as
            // r = b'b - A'Ax. We do not do this every iteration because this
            // requires an additional matrix vector multiply which would
            // double the complexity of the algorithm.
            if (result.num_iterations % options_.residual_reset_period == 0) {
                tmp = a * (*x);
                r = b - tmp;
            } else {
                r -= alpha * q;
            }

            // Quadratic model based termination.
            //   Q1 = x'Ax - 2 * b' x.
            const double q1 = -(x->dot(b + r));
            const double zeta = result.num_iterations * (q1 - q0) / q1;
            if (zeta < options_.q_tolerance &&
                result.num_iterations >= options_.min_num_iterations) {
                result.termination_type = SUCCESS;
                result.message = Format("Iteration {} Convergence: "
                                        "zeta = {:e} < {:e}. |r| = {:e}",
                                        result.num_iterations,
                                        zeta,
                                        options_.q_tolerance,
                                        r.norm());
                break;
            }
            q0 = q1;

            // Residual based termination.
            norm_r = r.norm();
            if (norm_r <= tol_r &&
                result.num_iterations >= options_.min_num_iterations) {
                result.termination_type = SUCCESS;
                result.message =
                        Format("Iteration {} Convergence: |r| = {:e} <= {:e}.",
                               result.num_iterations,
                               norm_r,
                               tol_r);
                break;
            }

            if (result.num_iterations >= options_.max_num_iterations) {
                result.termination_type = NO_CONVERGENCE;
                result.message = "Maximum number of iterations reached.";
                break;
            }
        }

        return result;
    }
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_H_

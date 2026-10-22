//
// Copyright 2015-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.

#ifndef CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_H_
#define CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_H_

#include <string>

#include "codelibrary/base/log.h"
#include "codelibrary/math/matrix/sparse_matrix.h"
#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/linear/preconditioner.h"

namespace cl {
namespace optimization {

/**
 * Enum type of linear solver.
 */
enum LinearSolverType {
    // Conjugate gradients on the normal equations.
    CGNR
};

/**
 * Abstract base class for objects that implement algorithms for solving linear
 * systems:
 *
 *   Ax = b.
 */
class LinearSolver {
public:
    /**
     * Termination type for Linear Solver.
     */
    enum TerminationType {
        // Termination criterion was met.
        SUCCESS,

        // Solver ran for max_num_iterations and terminated before the
        // termination tolerance could be satisfied.
        NO_CONVERGENCE,

        // Solver was terminated due to numerical problems, generally due to the
        // linear system being poorly conditioned.
        FAILURE,

        // Solver failed with a fatal error that cannot be recovered from.
        FATAL_ERROR
    };

    /**
     * Result of a call to the Solve method. We should move away from the
     * true/false method for determining solver success.
     * We should let the result object do the talking.
     */
    struct Result {
        int num_iterations = -1;
        TerminationType termination_type = FAILURE;
        std::string message;
    };

    /**
     * Options for LinearSolver.
     */
    struct Options {
        // Minimum number of internal iterations that the solver uses.
        int min_num_iterations = 1;

        // Maximum number of internal iterations that the solver uses.
        int max_num_iterations = 100;

        // Solver terminates when
        //
        //   |Ax - b| <= r_tolerance * |b|.
        //
        // This is the most commonly used termination criterion for
        // iterative solvers.
        double r_tolerance = 1e-6;

        // For PSD matrices A, let
        //
        //   Q(x) = x'Ax - 2b'x
        //
        // be the cost of the quadratic function defined by A and b. Then,
        // the solver terminates at iteration i if
        //
        //   i * (Q(x_i) - Q(x_i-1)) / Q(x_i) < q_tolerance.
        //
        // This termination criterion is more useful when using CG to solve the
        // Newton step.
        //
        // Reference:
        //  [1] Stephen G. Nash & Ariela Sofer, Assessing A Search Direction
        //      Within A Truncated Newton Method,
        //      Operation Research Letters 9(1990) 219-221.
        //
        //  [2] Stephen G. Nash, A Survey of Truncated Newton Methods,
        //      Journal of Computational and Applied Mathematics, 124(1-2),
        //      45-59, 2000.
        double q_tolerance = 1e-6;

        // Conjugate Gradient method maintains a cheap estimate of the residual
        // which may become inaccurate over time. Thus for non-zero values of
        // this parameter, the solver can be told to recalculate the value of
        // the residual using a |b - Ax| evaluation.
        int residual_reset_period = 10;

        // This option only makes sense for unsymmetric linear solvers that can
        // solve rectangular linear systems.
        //
        // Given a matrix A, an optional diagonal matrix D as a vector,
        // and a vector b, the linear solver will solve for
        //
        //   | A | x = | b |
        //   | D |     | 0 |
        //
        // If D is null, then it is treated as zero, and the solver returns
        // the solution to
        //
        //   Ax = b
        //
        // In either case, x is the vector that solves the following
        // optimization problem.
        //
        //   arg min_x ||Ax - b||^2 + ||Dx||^2
        //
        // Here A is a matrix of size m x n, with full column rank. If A
        // does not have full column rank, the results returned by the
        // solver cannot be relied on. D, if it is not null is an array of
        // size n.  b is an array of size m and x is an array of size n.
        RVector* diagonal = nullptr;

        // This option only makes sense for iterative solvers.
        //
        // In general the performance of an iterative linear solver
        // depends on the condition number of the matrix A. For example
        // the convergence rate of the conjugate gradients algorithm
        // is proportional to the square root of the condition number.
        //
        // One particularly useful technique for improving the
        // conditioning of a linear system is to precondition it. In its
        // simplest form a preconditioner is a matrix M such that instead
        // of solving Ax = b, we solve the linear system AM^{-1} y = b
        // instead, where M is such that the condition number k(AM^{-1})
        // is smaller than the conditioner k(A). Given the solution to
        // this system, x = M^{-1} y. The iterative solver takes care of
        // the mechanics of solving the preconditioned system and
        // returning the corrected solution x. The user only needs to
        // supply a linear operator.
        //
        // A null preconditioner is equivalent to an identity matrix being
        // used a preconditioner.
        Preconditioner* preconditioner = nullptr;
    };

    explicit LinearSolver(const Options& options)
        : options_(options) {
        CHECK(options_.min_num_iterations >= 0);
        CHECK(options_.min_num_iterations < options_.max_num_iterations);
        CHECK(options_.r_tolerance >= 0.0);
        CHECK(options_.q_tolerance > 0.0);
        CHECK(options_.residual_reset_period > 0);
    }

    LinearSolver(const LinearSolver&) = delete;

    virtual ~LinearSolver() = default;

    LinearSolver& operator=(const LinearSolver&) = delete;

protected:
    Options options_;
};

class SparseLinearSolver : public LinearSolver {
public:
    explicit SparseLinearSolver(const Options& options = Options())
        : LinearSolver(options) {}

    /**
     * Solve the linear system: Ax = b.
     */
    virtual Result Solve(const CSR3SparseMatrix<double>& a, const RVector& b,
                         RVector* x) const = 0;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_H_

//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEVENBERG_MARQUARDT_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEVENBERG_MARQUARDT_H_

#include <algorithm>

#include "codelibrary/base/clamp.h"
#include "codelibrary/optimization/linear/linear_solver_factory.h"
#include "codelibrary/optimization/unconstrained/trust_region_minimizer.h"

namespace cl {
namespace optimization {

/**
 * Levenberg-Marquardt method for non-linear least square problem.
 */
class LevenbergMarquardt : public TrustRegionMinimizer {
public:
    explicit LevenbergMarquardt(const Options& options = Options())
        : TrustRegionMinimizer(options)  {}

    bool ComputeTrustRegionStep(const CSR3SparseMatrix<double>& jacobian,
                                const RVector& residuals,
                                RVector* step) override {
        CHECK(step);

        const int num_parameters = jacobian.n_columns;
        if (!reuse_diagonal_) {
            sparse_matrix::SquaredColumnNorm(jacobian, &diagonal_);
            for (int i = 0; i < num_parameters; ++i) {
                diagonal_[i] = Clamp(diagonal_[i], options_.min_lm_diagonal,
                                     options_.max_lm_diagonal);
            }
        }

        lm_diagonal_ = diagonal_ * (1.0 / radius_);
        for (double& v : lm_diagonal_) {
            v = std::sqrt(v);
        }

        LinearSolver::Options linear_solve_options;
        linear_solve_options.q_tolerance = options_.eta;
        // Disable r_tolerance checking. Since we only care about
        // termination via the q_tolerance. As Nash and Sofer show,
        // r_tolerance based termination is essentially useless in
        // Truncated Newton methods.
        linear_solve_options.r_tolerance = 0.0;
        linear_solve_options.diagonal = &lm_diagonal_;
        std::unique_ptr<Preconditioner> preconditioner;
        if (options_.preconditioner_type == PreconditionerType::JACOBI) {
            preconditioner.reset(new JacobiPreconditioner(jacobian,
                                                          &lm_diagonal_));
        }
        linear_solve_options.preconditioner = preconditioner.get();

        // Instead of solving Jx = -r, solve Jy = r.
        // Then x can be found as x = -y, but the inputs jacobian and residuals
        // do not need to be modified.
        auto linear_solver = LinearSolverFactory::GetInstance()->Create(
              options_.linear_solver_type, linear_solve_options);
        LinearSolver::Result ls_result = linear_solver->Solve(jacobian,
                                                              residuals,
                                                              step);

        if (ls_result.termination_type == LinearSolver::FATAL_ERROR) {
            result_.termination_type = FATAL_ERROR;
            result_.message = "Linear solver fatal error: ";
            result_.message += ls_result.message;
            return false;
        }

        if (ls_result.termination_type == LinearSolver::FAILURE) {
            result_.termination_type = FAILURE;
            result_.message = "Linear solver failure: ";
            result_.message += ls_result.message;
            return false;
        }

        *step = -(*step);
        reuse_diagonal_ = true;
        return true;
    }

    void StepAccepted(double step_quality) override {
        CHECK(step_quality > 0.0);

        radius_ = radius_ / std::max(1.0 / 3.0,
                                     1.0 - pow(2.0 * step_quality - 1.0, 3.0));
        radius_ = std::min(options_.max_radius, radius_);
        decrease_factor_ = 2.0;
        reuse_diagonal_ = false;
    }

    void StepRejected() override {
        radius_ = radius_ / decrease_factor_;
        decrease_factor_ *= 2.0;
        reuse_diagonal_ = true;
    }

private:
    double decrease_factor_ = 2.0;

    bool reuse_diagonal_ = false;

    // diagonal = diag(J'J)
    RVector diagonal_;

    // Scaled copy of diagonal. Stored here as optimization to prevent
    // allocations in every iteration and reuse when a step fails and
    // ComputeTrustRegionStep() is called again.
    //
    // lm_diagonal = diagonal / radius
    RVector lm_diagonal_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEVENBERG_MARQUARDT_H_

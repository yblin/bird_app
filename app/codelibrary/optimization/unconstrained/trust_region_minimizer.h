//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TRUST_REGION_MINIMIZER_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TRUST_REGION_MINIMIZER_H_

#include <algorithm>
#include <limits>

#include "codelibrary/optimization/linear/linear_solver.h"
#include "codelibrary/optimization/unconstrained/least_squares_problem.h"
#include "codelibrary/optimization/unconstrained/minimizer.h"

namespace cl {
namespace optimization {

/**
 * Generic trust region minimization algorithm for unconstrained least sqaure
 * problems.
 */
class TrustRegionMinimizer : public Minimizer {
    /**
     * Evaluate the quality of a step.
     */
    class StepEvaluator {
    public:
        StepEvaluator(double initial_cost,
                      int max_consecutive_nonmonotonic_steps)
            : minimum_cost_(initial_cost),
              current_cost_(initial_cost),
              reference_cost_(initial_cost),
              candidate_cost_(initial_cost),
              max_consecutive_nonmonotonic_steps_(
                  max_consecutive_nonmonotonic_steps) {}

        /**
         * Return the quality of the step given its cost and the decrease in
         * the cost of the model. model_cost_change has to be positive.
         */
        double StepQuality(double cost, double model_cost_change) const {
            // If the function evaluation for this step was a failure, in which
            // case the TrustRegionMinimizer would have set the cost to
            // std::numeric_limits<double>::max(). In this case, the division by
            // model_cost_change can result in an overflow. To prevent that from
            // happening, we will deal with this case explicitly.
            if (cost >= std::numeric_limits<double>::max()) {
                return std::numeric_limits<double>::lowest();
            }

            double relative_decrease = (current_cost_ - cost) /
                                        model_cost_change;
            double historical_relative_decrease = (reference_cost_ - cost) /
                (accumulated_reference_model_cost_change_ + model_cost_change);
            return std::max(relative_decrease, historical_relative_decrease);
        }

        /**
         * Inform the step evaluator that a step with the given cost and
         * model_cost_change has been accepted by the trust region minimizer.
         */
        void StepAccepted(double cost, double model_cost_change) {
            // Algorithm 10.1.2 from Trust Region Methods by Conn, Gould &
            // Toint.
            //
            // Step 3a.
            current_cost_ = cost;
            accumulated_candidate_model_cost_change_ += model_cost_change;
            accumulated_reference_model_cost_change_ += model_cost_change;

            // Step 3b.
            if (current_cost_ < minimum_cost_) {
                minimum_cost_ = current_cost_;
                n_consecutive_nonmonotonic_steps_ = 0;
                candidate_cost_ = current_cost_;
                accumulated_candidate_model_cost_change_ = 0.0;
            } else {
                // Step 3c.
                ++n_consecutive_nonmonotonic_steps_;
                if (current_cost_ > candidate_cost_) {
                    candidate_cost_ = current_cost_;
                    accumulated_candidate_model_cost_change_ = 0.0;
                }
            }

            // Step 3d.
            //
            // At this point we have made too many non-monotonic steps and
            // we are going to reset the value of the reference iterate so
            // as to force the algorithm to descend.
            //
            // Note: In the original algorithm by Toint, this step was only
            // executed if the step was non-monotonic, but that would not handle
            // the case of max_consecutive_nonmonotonic_steps = 0. The small
            // modification of doing this always handles that corner case
            // correctly.
            if (n_consecutive_nonmonotonic_steps_ ==
                max_consecutive_nonmonotonic_steps_) {
                reference_cost_ = candidate_cost_;
                accumulated_reference_model_cost_change_ =
                    accumulated_candidate_model_cost_change_;
            }
        }

     private:
        // The minimum cost encountered up till now.
        double minimum_cost_;

        // The current cost of the trust region minimizer as informed by the
        // last call to StepAccepted.
        double current_cost_;
        double reference_cost_;
        double candidate_cost_;

        // Accumulated model cost since the last time the reference model
        // cost was updated, i.e., when a step with cost less than the
        // current known minimum cost is accepted.
        double accumulated_reference_model_cost_change_ = 0.0;

        // Accumulated model cost since the last time the candidate model
        // cost was updated, i.e., a non-monotonic step was taken with a
        // cost that was greater than the current candidate cost.
        double accumulated_candidate_model_cost_change_ = 0.0;

        // Number of steps taken since the last time minimum_cost was updated.
        int n_consecutive_nonmonotonic_steps_ = 0;

        // Maximum number of consecutive nonmonotonic steps.
        int max_consecutive_nonmonotonic_steps_;
    };

public:
    /**
     * Options for trust region minimier.
     *
     * The defaults should be suitable for a wide range of problems; however,
     * better performance is often obtainable with tweaking.
     */
    struct Options {
        // Minimizer terminates when
        //
        //   |new_cost - old_cost| < function_tolerance * |old_cost|;
        //
        double function_tolerance = 1e-6;

        // Minimizer terminates when
        //
        //   max_i |x - Project(x - g(x))| < gradient_tolerance
        //
        // This value should typically be 1e-4 * function_tolerance.
        double gradient_tolerance = 1e-10;

        // Minimizer terminates when
        //
        //   |step|_2 <= parameter_tolerance * (|x|_2 +  parameter_tolerance)
        //
        double parameter_tolerance = 1e-8;

        // Maximum number of iterations for the minimizer to run.
        int max_iterations = 50;

        // Initial radius of trust region.
        double initial_radius = 1e4;

        // Minimizer terminates when the trust region radius becomes smaller
        // than this value.
        double min_radius = 1e-32;

        // Maximum radius of trust region.
        double max_radius = 1e32;

        // Minimum and maximum values of the diagonal damping matrix.
        double min_lm_diagonal = 1e-6, max_lm_diagonal = 1e32;

        // Lower bound for the relative decrease before a step is
        // accepted.
        double min_relative_decrease = 1e-3;

        // Sometimes due to numerical conditioning problems or linear solver
        // flakiness, the trust region method may return a numerically invalid
        // step that can be fixed by reducing the trust region size. So the
        // TrustRegionMinimizer allows for a few successive invalid steps before
        // it declares NO_CONVERGENCE.
        int max_consecutive_invalid_steps = 5;

        // The window size used by the step selection algorithm to accept
        // non-monotonic steps. Setting this parameter to zero, recovers the
        // classic montonic descent algorithm.
        int max_consecutive_nonmonotonic_steps = 0;

        // Forcing sequence parameter. The truncated Newton solver uses
        // this number to control the relative accuracy with which the
        // Newton step is computed.
        //
        // This constant is passed to ConjugateGradientsSolver which uses
        // it to terminate the iterations when
        //
        //  i * (Q_i - Q_{i-1})/Q_i < eta
        double eta = 1e-1;

        // Normalize the jacobian using Jacobi scaling before calling
        // the linear least squares solver.
        bool jacobi_scaling = true;

        ////////////////////////////////////////////////////////////////////////
        // Linear least squares solver options.
        LinearSolverType linear_solver_type = CGNR;
        PreconditionerType preconditioner_type = PreconditionerType::JACOBI;
    };

    /**
     * Callback for each step.
     */
    class Callback {
    public:
        Callback() = default;
        virtual ~Callback() = default;

        /**
         * This function will be called at each step.
         * The default one only print the state.
         */
        virtual void operator() (int n_iteration, double radius,
                                 double x_cost, double candidate_cost,
                                 double gradient_max_norm) {
            Print("{:4} r={:3.2e} f={:3.2e} f_change={:3.2e} g={:3.2e}\n",
                  n_iteration,
                  radius,
                  x_cost,
                  x_cost - candidate_cost,
                  gradient_max_norm);
        }
    };

    explicit TrustRegionMinimizer(const Options& options)
        : options_(options) {
        CHECK(options_.function_tolerance > 0.0);
        CHECK(options_.gradient_tolerance > 0.0);
        CHECK(options_.parameter_tolerance > 0.0);
        CHECK(options_.max_iterations > 0);
        CHECK(options_.initial_radius > 0.0);
        CHECK(options_.initial_radius <= options_.max_radius);
        CHECK(options_.min_radius > 0.0);
        CHECK(options_.max_radius > 0.0);
        CHECK(options_.min_radius < options_.max_radius);
        CHECK(options_.max_lm_diagonal > 0.0);
        CHECK(options_.min_lm_diagonal > 0.0);
        CHECK(options_.min_lm_diagonal < options_.max_lm_diagonal);
        CHECK(options_.max_consecutive_invalid_steps >= 0);
        CHECK(options_.max_consecutive_nonmonotonic_steps >= 0);
    }

    /**
     * Minimize the least square problem, and output the corresponding
     * parameters.
     */
    Result Minimize(LeastSquaresProblem* problem, RVector* parameters) {
        result_.Initialize();

        RVector& x = *parameters;
        x.resize(problem->n_parameters(), 0.0);

        double x_cost;
        RVector gradient, residuals, jacobian_scaling(problem->n_parameters());
        CSR3SparseMatrix<double> jacobian;

        ++result_.n_function_evaluations;
        ++result_.n_gradient_evaluations;
        x_cost = problem->Evaluate(x, &gradient, &residuals, &jacobian);

        double gradient_max_norm = gradient.infinty_norm();
        double x_norm = x.norm();

        StepEvaluator step_evaluator(x_cost, options_.
                                     max_consecutive_nonmonotonic_steps);

        if (options_.jacobi_scaling) {
            // Compute a scaling vector that is used to improve the
            // conditioning of the Jacobian.
            //
            // jacobian_scaling = diag(J'J)^{-1}
            sparse_matrix::SquaredColumnNorm(jacobian, &jacobian_scaling);
            for (double& v : jacobian_scaling) {
                // Add one to the denominator to prevent division by zero.
                v = 1.0 / (1.0 + std::sqrt(v));
            }
        }

        // Scale jacobian columns.
        for (int r = 0; r < jacobian.n_rows; ++r) {
            for (int j = jacobian.rows[r]; j < jacobian.rows[r + 1]; ++j) {
                int c = jacobian.columns[j];
                jacobian.values[j] *= jacobian_scaling[c];
            }
        }

        radius_ = options_.initial_radius;

        // Number of consecutive steps where the minimizer loop computed a
        // numerically invalid step.
        int n_consecutive_invalid_steps = 0;

        while (true) {
            if (MaxIterationsReached(++result_.n_iterations,
                                     options_.max_iterations))
                return result_;
            if (GradientConvergence(gradient_max_norm,
                                    options_.gradient_tolerance))
                return result_;
            if (MinRadiusReached(radius_)) return result_;

            // Solve the trust region subproblem.
            RVector trust_region_step;
            if (!ComputeTrustRegionStep(jacobian, residuals,
                                        &trust_region_step))
                return result_;

            // new_model_cost
            //  = 1/2 [r + J * step]^2
            //  = 1/2 [ r'r + 2r'J * step + step' * r' * r * step ]
            // model_cost_change
            //  = cost - new_model_cost
            //  = r'r/2  - 1/2 [ r'r + 2r'J * step + step' * J' * J * step]
            //  = -r'J * step - step' * J' * J * step / 2
            //  = -(J * step)'(r + J * step / 2)
            RVector model_residuals = jacobian * trust_region_step;
            const double model_cost_change =
                -model_residuals.dot(0.5 * model_residuals + residuals);

            if (model_cost_change < 0.0) {
                // This step is invalid.
                if (++n_consecutive_invalid_steps >=
                    options_.max_consecutive_invalid_steps) {
                    result_.message =
                            Format("Number of consecutive invalid steps more "
                                   "than max_consecutive_invalid_steps: {}",
                                   options_.max_consecutive_invalid_steps);
                    result_.termination_type = NO_CONVERGENCE;
                    result_.timer.Stop();
                    return result_;
                }

                continue;
            }

            n_consecutive_invalid_steps = 0;
            // Undo the Jacobian column scaling.
            for (int i = 0; i < trust_region_step.size(); ++i) {
                trust_region_step[i] *= jacobian_scaling[i];
            }

            RVector candidate_x = x + trust_region_step;
            double candidate_cost = 0.0;

            ++result_.n_function_evaluations;
            candidate_cost = problem->Evaluate(candidate_x, nullptr);

            if (ParametersConvergence(x_norm, trust_region_step.norm(),
                                      options_.parameter_tolerance))
                return result_;
            if (FunctionConvergence(x_cost, candidate_cost,
                                    options_.function_tolerance))
                return result_;

            double relative_descrease =
                    step_evaluator.StepQuality(candidate_cost,
                                               model_cost_change);

            if (callback_) callback_->operator()(result_.n_iterations,
                                                 radius_,
                                                 x_cost,
                                                 candidate_cost,
                                                 gradient_max_norm);

            if (relative_descrease > options_.min_relative_decrease) {
                // step success.
                x = candidate_x;
                x_norm = x.norm();

                ++result_.n_function_evaluations;
                ++result_.n_gradient_evaluations;
                x_cost = problem->Evaluate(x, &gradient, &residuals, &jacobian);

                gradient_max_norm = gradient.infinty_norm();
                // Scale jacobian columns.
                for (int r = 0; r < jacobian.n_rows; ++r) {
                    for (int j = jacobian.rows[r]; j < jacobian.rows[r + 1];
                         ++j) {
                        int c = jacobian.columns[j];
                        jacobian.values[j] *= jacobian_scaling[c];
                    }
                }
                StepAccepted(relative_descrease);
                step_evaluator.StepAccepted(candidate_cost, model_cost_change);
            } else {
                // step unsuccess.
                StepRejected();
            }
        }
    }

    /**
     * Set callback.
     */
    void set_callback(Callback* callback) {
        callback_ = callback;
    }

protected:
    /**
     * Solve the trust region subproblem by using the current radius.
     *
     * We seek a solution of the subproblem:
     *
     *    min m(p) = 1/2[J * step + r]^2,  s.t. ||p|| <= radius.
     *     p
     */
    virtual bool ComputeTrustRegionStep(
            const CSR3SparseMatrix<double>& jacobian,
            const RVector& residual,
            RVector* step) = 0;

    /**
     * Current step has been accepted.
     */
    virtual void StepAccepted(double step_quality) = 0;

    /**
     * Current step has been rejected.
     */
    virtual void StepRejected() = 0;

    /**
     * Check convergence based the size of the trust region radius.
     */
    bool MinRadiusReached(double radius) {
        if (radius > options_.min_radius) return false;

        result_.message = Format("Minimum trust region radius reached. "
                                 "Trust region radius: {:e} <= {:e}",
                                 radius,
                                 options_.min_radius);
        result_.termination_type = CONVERGENCE;
        return true;
    }

    Options options_;

    // Current radius of trust region.
    double radius_ = 0.0;

    // Callback for each iteration.
    Callback* callback_ = nullptr;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TRUST_REGION_MINIMIZER_H_

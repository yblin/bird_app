//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_MINIMIZER_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_MINIMIZER_H_

#include <string>

#include "codelibrary/base/timer.h"

namespace cl {
namespace optimization {

/**
 * Generic unconstrained minimizer.
 */
class Minimizer {
public:
    // Termination type for minimizer.
    enum TerminationType {
        // Termination criterion was met.
        CONVERGENCE,

        // Solver ran for max_num_iterations and terminated before the
        // termination tolerance could be satisfied.
        NO_CONVERGENCE,

        // Solver was terminated due to numerical problems, generally due to the
        // linear system being poorly conditioned.
        FAILURE,

        // Solver failed with a fatal error that cannot be recovered from.
        FATAL_ERROR
    };

    // Result for Minimizer.
    struct Result {
        void Initialize() {
            termination_type = FATAL_ERROR;
            n_iterations = 0;
            n_function_evaluations = 0;
            n_gradient_evaluations = 0;
            message.clear();
            timer.Reset();
            timer.Start();
        }

        // Minimizer convergence or not.
        TerminationType termination_type = FATAL_ERROR;

        // Number of iterations.
        int n_iterations = 0;

        // Number of function evaluations.
        int n_function_evaluations = 0;

        // Number of gradient evaluations.
        int n_gradient_evaluations = 0;

        // Obtained message.
        std::string message;

        // Record running time.
        Timer timer;

        /**
         * Print result.
         */
        friend std::ostream& operator <<(std::ostream& os, const Result& rhs) {
            switch (rhs.termination_type) {
            case CONVERGENCE:
                os << "CONVERGENCE\n";
                break;
            case NO_CONVERGENCE:
                os << "NO_CONVERGENCE\n";
                break;
            case FAILURE:
                os << "FAILURE\n";
                break;
            case FATAL_ERROR:
                os << "FATAL_ERROR\n";
                break;
            }

            os << Format("n_iterations:           {}\n"
                         "n_function_evaluations: {}\n"
                         "n_gradient_evaluations: {}\n"
                         "Running time:           {}\n"
                         "{}",
                         rhs.n_iterations,
                         rhs.n_function_evaluations,
                         rhs.n_gradient_evaluations,
                         rhs.timer.elapsed_time(),
                         rhs.message);
             return os;
        }
    };

    Minimizer() = default;

    Minimizer(const Minimizer&) = delete;

    Minimizer& operator=(const Minimizer&) = delete;

    virtual ~Minimizer() = default;

    /**
     * Return the result after minimization.
     */
    const Result& result() const { return result_; }

protected:
    /**
     * Check if the max iterations reached.
     */
    bool MaxIterationsReached(int n_iterations, int max_iterations) {
        if (n_iterations >= max_iterations) {
            result_.message = "Maximum number of iterations reached.";
            result_.termination_type = NO_CONVERGENCE;
            result_.timer.Stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the parameters are convergence.
     */
    bool ParametersConvergence(double previous_x_norm,
                               double delta_x_norm,
                               double parameter_tolerance) {
        double t = previous_x_norm + parameter_tolerance;
        double tolerance = parameter_tolerance * t;
        if (delta_x_norm <= tolerance) {
            result_.message = Format("Parameter tolerance reached. "
                                     "Relative step_norm: {:e} <= {:e}.",
                                     delta_x_norm / t,
                                     parameter_tolerance);
            result_.termination_type = CONVERGENCE;
            result_.timer.Stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the function cost is convergence.
     */
    bool FunctionConvergence(double previous_cost,
                             double current_cost,
                             double function_tolerance) {
        double cost_change = std::fabs(current_cost - previous_cost);
        double absolute_cost = std::fabs(previous_cost);
        double tolerance = function_tolerance * absolute_cost;

        if (cost_change <= tolerance) {
            result_.message = Format("Function tolerance reached. "
                                     "|cost_change/cost|: {:e} <= {:e}",
                                     cost_change / absolute_cost,
                                     function_tolerance);
            result_.termination_type = CONVERGENCE;
            result_.timer.Stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the gradient is convergence.
     */
    bool GradientConvergence(double gradient_max_norm,
                             double gradient_tolerance) {
        if (gradient_max_norm <= gradient_tolerance) {
            result_.message = Format("Gradient tolerance reached. "
                                     "Gradient max norm: {:e} <= {:e}",
                                     gradient_max_norm,
                                     gradient_tolerance);
            result_.termination_type = CONVERGENCE;
            result_.timer.Stop();
            return true;
        }
        return false;
    }

    Result result_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_MINIMIZER_H_

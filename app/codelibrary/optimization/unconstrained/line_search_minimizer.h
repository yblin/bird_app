//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_MINIMIZER_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_MINIMIZER_H_

#include <algorithm>

#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/unconstrained/line_search.h"
#include "codelibrary/optimization/unconstrained/minimizer.h"

namespace cl {
namespace optimization {

/**
 * Generic line search minimization algorithm for unconstrained problems.
 */
class LineSearchMinimizer : public Minimizer {
public:
    using Problem = UnconstrainedProblem;

    /**
     * Options for line search minimizer.
     *
     * The defaults should be suitable for a wide range of problems; however,
     * better performance is often obtainable with tweaking.
     */
    struct Options {
        // Line search minimizer terminates when
        //
        //   |new_cost - old_cost| < function_tolerance * |old_cost|;
        //
        double function_tolerance = 1e-6;

        // Line search minimizer terminates when
        //
        //   max_i |x - Project(x - g(x))| < gradient_tolerance
        //
        // This value should typically be 1e-4 * function_tolerance.
        double gradient_tolerance = 1e-10;

        // Line search minimizer terminates when
        //
        //   |step|_2 <= parameter_tolerance * (|x|_2 +  parameter_tolerance)
        //
        double parameter_tolerance = 1e-8;

        // Maximum number of iterations for the line search minimizer to run.
        int max_iterations = 50;

        // Maximum number of restarts of the line search direction before
        // terminating the optimization. Restarts of the line search direction
        // occur when the current algorithm fails to produce a new descent
        // direction. This typically indicates a numerical failure, or a
        // breakdown in the validity of the approximations used.
        int max_direction_restarts = 10;

        // Solving the line search problem exactly is computationally
        // prohibitive. Fortunately, line search based optimization algorithms
        // can still guarantee convergence if instead of an exact solution, the
        // line search algorithm returns a solution which decreases the value of
        // the objective function sufficiently. More precisely, we are looking
        // for a step_size
        // s.t.
        //
        //  f(step_size) <= f(0) + sufficient_decrease * f'(0) * step_size
        double sufficient_decrease = 1e-4;

        // The strong Wolfe conditions consist of the Armijo sufficient decrease
        // condition, and an additional requirement that the step-size be chosen
        // s.t. the _magnitude_ ('strong' Wolfe conditions) of the gradient
        // along the search direction decreases sufficiently. Precisely, this
        // second condition is that we seek a step_size s.t.
        //
        //   |f'(step_size)| <= sufficient_curvature_decrease * |f'(0)|
        //
        // Where f() is the line search objective and f'() is the derivative
        // of f w.r.t step_size (d f / d step_size).
        double sufficient_curvature_decrease = 0.9;

        // If during the line search, the step_size falls below this value,
        // it is truncated to zero.
        double min_step_size = 1e-9;

        // Maximum number of trial step size iterations during each line search,
        // if a step size satisfying the search conditions cannot be found
        // within this number of trials, the line search will terminate.
        int max_line_search_iterations = 20;

        // During the bracketing phase of the Wolfe search, the step size is
        // increased until either a point satisfying the Wolfe conditions is
        // found, or an upper bound for a bracket containing a point satisfying
        // the conditions is found.  Precisely, at each iteration of the
        // expansion:
        //
        //   new_step_size <= max_step_expansion * step_size.
        //
        // By definition for expansion, max_step_expansion > 1.0.
        double max_step_expansion = 10.0;
    };

    /**
     * Iteration state of Line Search Minimizer.
     */
    struct State {
        // Function cost.
        double cost = 0.0;

        // Gradient.
        RVector gradient;

        // gradient.square_norm().
        double gradient_squared_norm = 0.0;

        // gradient.infinity_norm().
        double gradient_max_norm = 0.0;

        // Search direction.
        RVector search_direction;

        // Step size obtained by line search.
        double step_size = 0.0;
    };

    /**
     * Callback for each step.
     */
    class Callback {
    public:
        Callback() = default;
        virtual ~Callback() = default;

        virtual void Initialize() {
            n_calls_ = 0;
        }

        /**
         * This function will be called at each step.
         * The default one only print the state.
         */
        virtual void operator() (const State& previous_state,
                                 const State& current_state,
                                 const RVector&) {
            Print("{:4} f={:3.2e} f_change={:3.2e} g={:3.2e} step={:3.2e}\n",
                  ++n_calls_,
                  current_state.cost,
                  previous_state.cost - current_state.cost,
                  current_state.gradient_max_norm,
                  current_state.step_size);
        }

    protected:
        int n_calls_ = 0;
    };

    explicit LineSearchMinimizer(const Options& options)
        : options_(options) {
        CHECK(options_.function_tolerance > 0.0);
        CHECK(options_.gradient_tolerance > 0.0);
        CHECK(options_.parameter_tolerance > 0.0);
        CHECK(options_.max_iterations > 0);
        CHECK(options_.max_direction_restarts > 0);
        CHECK(options_.sufficient_decrease > 0.0);
        CHECK(options_.sufficient_decrease <
              options_.sufficient_curvature_decrease);
        CHECK(options_.sufficient_curvature_decrease < 1.0);
        CHECK(options_.min_step_size > 0.0);
        CHECK(options_.max_line_search_iterations > 0);
        CHECK(options_.max_step_expansion > 1.0);
    }

    LineSearchMinimizer(const LineSearchMinimizer&) = delete;

    LineSearchMinimizer& operator=(const LineSearchMinimizer&) = delete;

    /**
     * Minimize the problem according to the line search.
     *
     * Parameters:
     *  problem     - the problem to be minimized.
     *  parameters  - the (optional) initial input parameters, and it also
     *                stores the optimal parameters which can minimize the
     *                problem.
     *  line_search - line search processor.
     * 
     * Return:
     *  the summary.
     */
    Result Minimize(Problem* problem, RVector* parameters) {
        CHECK(problem);
        CHECK(parameters);

        RVector& x = *parameters;
        int n_parameters = problem->n_parameters();
        if (parameters->size() != n_parameters) {
            parameters->assign(n_parameters, 0.0);
        }

        result_.Initialize();

        State current_state, previous_state;
        Evaluate(problem, x, &current_state);
        previous_state = current_state;

        if (GradientConvergence(current_state.gradient_max_norm,
                                options_.gradient_tolerance))
            return result_;

        LineSearch::Options o;
        o.max_n_iterations = options_.max_line_search_iterations;
        o.max_step_expansion = options_.max_step_expansion;
        o.min_step_size = options_.min_step_size;
        o.sufficient_curvature_decrease =
                options_.sufficient_curvature_decrease;
        o.sufficient_decrease = options_.sufficient_decrease;
        LineSearch line_search(o);

        if (callback_) callback_->Initialize();

        int n_direction_restarts = 0;
        while (true) {
            if (callback_) callback_->operator()(previous_state,
                                                 current_state,
                                                 x);

            if (MaxIterationsReached(++result_.n_iterations,
                                     options_.max_iterations))
                return result_;

            bool line_search_status = true;
            if (result_.n_iterations == 1) {
                current_state.search_direction = -current_state.gradient;
            } else {
                line_search_status =
                        SearchDirection(previous_state, current_state,
                                        &current_state.search_direction);
            }

            if (!line_search_status &&
                n_direction_restarts >= options_.max_direction_restarts) {
                // Line search direction failed to generate a new direction, and
                // we have already reached our specified maximum number of
                // restarts, terminate optimization.
                result_.message = Format("Line search direction failure: "
                                         "max_direction_restarts: {} "
                                         "reached.",
                                         options_.max_direction_restarts);
                result_.termination_type = FAILURE;
                result_.timer.Stop();
                return result_;
            }

            if (!line_search_status) {
                // Restart line search direction with gradient descent on first
                // iteration as we have not yet reached our maximum number of
                // restarts.
                ++n_direction_restarts;
                RestartSearchDirection();
                current_state.search_direction = -current_state.gradient;
            }

            double directional_derivative =
                    current_state.search_direction.dot(current_state.gradient);

            // Start line search.
            double cost_change = current_state.cost - previous_state.cost;
            double initial_step_size =
                    (result_.n_iterations == 1 || !line_search_status) ?
                    std::min(1.0, 1.0 / current_state.gradient_max_norm) :
                    std::min(1.0, 2.0 * (cost_change) / directional_derivative);
            if (initial_step_size < 0.0) {
                result_.message =
                    Format("Numerical failure in line search, "
                           "initial_step_size is negative: {:.5e}, "
                           "directional_derivative: {:.5e}, "
                           "(current_cost - previous_cost): {:.5e}",
                           initial_step_size,
                           directional_derivative,
                           current_state.cost - previous_state.cost);
                result_.termination_type = FAILURE;
                result_.timer.Stop();
                return result_;
            }
            LineSearchProblem p(problem, x, current_state.search_direction);
            LineSearch::Result r = line_search.Search(&p,
                                                      initial_step_size,
                                                      current_state.cost,
                                                      current_state.gradient,
                                                      directional_derivative);
            result_.n_function_evaluations += r.n_function_evaluations;
            result_.n_gradient_evaluations += r.n_gradient_evaluations;

            if (!r.success) {
                result_.message =
                    Format("Numerical failure in line search, failed to find a "
                           "valid step size, (did not run out of iterations) "
                           "using initial_step_size: {:.5e}. "
                           "The detail is:\n"
                           "{}",
                           initial_step_size,
                           r.message);
                result_.termination_type = FAILURE;
                result_.timer.Stop();
                return result_;
            }

            current_state.step_size = r.optimal_point.x;
            previous_state = current_state;
            current_state.cost = r.optimal_point.f;
            current_state.gradient = r.optimal_point.g;
            current_state.gradient_max_norm = r.optimal_point.g.infinty_norm();
            current_state.gradient_squared_norm =
                    r.optimal_point.g.squared_norm();

            const double x_norm = x.norm();
            RVector delta = r.optimal_point.x * current_state.search_direction;
            x += delta;
            const double delta_norm = delta.norm();

            if (ParametersConvergence(x_norm, delta_norm,
                                      options_.parameter_tolerance))
                return result_;
            if (GradientConvergence(current_state.gradient_max_norm,
                                    options_.gradient_tolerance))
                return result_;
            if (FunctionConvergence(previous_state.cost, current_state.cost,
                                    options_.function_tolerance))
                return result_;
        }
    }

    /**
     * Return the options.
     */
    const Options& options() const {
        return options_;
    }

    /**
     * Set callback.
     */
    void set_callback(Callback* callback) {
        callback_ = callback;
    }

protected:
    /**
     * Evaluate the state according to the given parameters.
     */
    void Evaluate(Problem* problem, const RVector& x, State* state) {
        state->cost = problem->Evaluate(x, &state->gradient);
        state->gradient_max_norm = state->gradient.infinty_norm();
        state->gradient_squared_norm = state->gradient.squared_norm();
        ++result_.n_function_evaluations;
        ++result_.n_gradient_evaluations;
    }

    /**
     * The implementation of SearchDirection is based on different methods.
     *
     * Here, we adopt the simplest one: gradient direction
     */
    virtual bool SearchDirection(const State& previous,
                                 const State& current,
                                 RVector* search_direction) = 0;

    /**
     * Restart search direction.
     */
    virtual void RestartSearchDirection() {}

    Options options_;

    // Callback for each iteration.
    Callback* callback_ = nullptr;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_MINIMIZER_H_

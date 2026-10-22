//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_H_

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>

#include "codelibrary/base/equal.h"
#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/unconstrained/problem.h"

namespace cl {
namespace optimization {

/**
 * Line Search Problem.
 *
 * Line search is an one dimensional optimization algorithm. Given a direction
 * p_k and an objective function f (assumed smooth), line search try to find a
 * step length 'a' which can minimize the following problem:
 *
 *   min F(x_k + a*p_k).
 *   a>0
 */
struct LineSearchProblem {
    // Function sample.
    struct FunctionSample {
        double x = 0.0;      // Step size.
        double f = 0.0;      // Function value.
        RVector g;           // Gradient.
        double g_norm = 0.0; // Gradient norm.
    };

    LineSearchProblem(UnconstrainedProblem* f, const RVector& x,
                      const RVector& p)
        : problem(f), position(x), direction(p) {
        CHECK(position.size() == problem->n_parameters());
        CHECK(direction.size() == problem->n_parameters());
    }

    /**
     * Evaluate the function sample at the given step_size.
     */
    void Evaluate(double x, FunctionSample* sample) const {
        CHECK(sample);

        sample->x = x;
        sample->f = problem->Evaluate(x * direction + position, &sample->g);
        sample->g_norm = sample->g.dot(direction);
    }

    UnconstrainedProblem* problem;
    RVector position;
    RVector direction;
};

/**
 * Line search using strong Wolfe conditions.
 *
 * By solving line search problem exactly, we would derive the maximum benefit
 * from the direction p_k, but an exact minimization is expensive and
 * unnecessary. Instead, the line search algorithm generates a limited number of
 * trial step lengths until it finds one that loosely approximates the minimum
 * of above problem.
 *
 * An elegant and practical criterion for a suitable a_k is to require a_k to
 * satisfy the sufficient decrease condition (also known as Armijo condition):
 *
 * F(x_k + a * p_k) <= F(x_k) + u_0 * a_k * F'(x_k)^T * p_k
 *
 * and the curvature condition:
 *
 * |F'(x_k + a * p_k)^T * p_k| <= u_1 * |F'(x_k)^T * p_k|
 *
 * where u_0 and u_1 are two constants with 0 < u_0 < u_1 < 1.
 *
 * The sufficient decrease condition guarantees, in particular, that
 * F(x_k+1) < F(x_k) while the curvature condition requires that a_k be not too
 * far from a minimizer of F(x_k + a * p_k).
 *
 * The following line search procedure is done in two stages: A bracketing phase
 * finds an interval containing desirable step lengths, and a interpolation
 * phase computes a good step length within this interval.
 *
 * Reference:
 *   Nocedal J, Wright S J. Numerical optimization [M]. Springer, 2006.
 */
class LineSearch {
    using FunctionSample = LineSearchProblem::FunctionSample;

public:
    /**
     * Result of the line search.
     */
    struct Result {
        bool success = false;
        FunctionSample optimal_point;
        int n_function_evaluations = 0;
        int n_gradient_evaluations = 0;
        int n_iterations = 0;
        std::string message;
    };

    /**
     * Options for LineSearch.
     * 
     * Note that, the default value of sufficient_curvature_decrease is 0.9 when
     * the search direction pk is chosen by a Newton or Quasi-Newton method, and
     * 0.1 when pk is obtained from a nonlinear conjugate gradient method.
     */
    struct Options {
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
        int max_n_iterations = 20;

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

    explicit LineSearch(const Options& options)
        : options_(options) {
        CHECK(options_.sufficient_decrease > 0.0);
        CHECK(options_.sufficient_decrease <
              options_.sufficient_curvature_decrease);
        CHECK(options_.sufficient_curvature_decrease < 1.0);
        CHECK(options_.min_step_size > 0.0);
        CHECK(options_.max_n_iterations > 0);
        CHECK(options_.max_step_expansion > 1.0);
    }

    LineSearch(const LineSearch&) = delete;

    LineSearch& operator=(const LineSearch&) = delete;

    /**
     * Perform the line search to minimize the given line search problem.
     *
     * Note that, initial_cost and initial_gradient are evaluated at
     * step_size = 0, not initial_step_size, which is the starting guess.
     *
     * Parameters:
     *  problem               - the line search problem to be minimized.
     *  initial_step_size     - initial step size.
     *  initial_function_cost - initial function cost at step_size = 0.
     *  initial_gradient      - initial gradient at step_size = 0.
     *  initial_gradient_norm - Dot(gradient, direction).
     */
    Result Search(LineSearchProblem* problem,
                  double initial_step_size,
                  double initial_function_cost,
                  const RVector& initial_gradient,
                  double initial_gradient_norm) const {
        CHECK(initial_step_size > 0.0);
        CHECK(initial_gradient_norm < 0.0);

        Result result;
        FunctionSample initial;
        initial.x = 0.0;
        initial.f = initial_function_cost;
        initial.g = initial_gradient;
        initial.g_norm = initial_gradient_norm;

        bool do_zoom_search = false;
        FunctionSample solution, bracket_low, bracket_high;

        if (!BracketingPhase(problem, initial, initial_step_size,
                             &bracket_low, &bracket_high, &do_zoom_search,
                             &result))
            return result;

        if (!do_zoom_search) {
            result.optimal_point = bracket_low;
            result.success = true;
            return result;
        }

        if (!ZoomPhase(problem, initial, bracket_low, bracket_high,
                       &solution, &result))
            return result;

        // Ensure that if we ran out of iterations while zooming the bracket, or
        // shrank the bracket width to < tolerance and failed to find a point
        // which satisfies the strong Wolfe curvature condition, that we return
        // the point amongst those found thus far, which minimizes f() and
        // satisfies the Armijo condition.
        result.optimal_point = solution.f <= bracket_low.f ? solution
                                                           : bracket_low;
        result.success = true;
        return result;
    }

private:
    /**
     * Return step_size in [min_step_size, max_step_size] which minimizes the
     * cubic hermite polynomial that interpolates the two most recent values of
     * step_size.
     *
     * Given two function samples: (x1, f1, g1) and (x2, f2, g2), the cubic
     * Hermite polynomial can be written explicitly as:
     *
     *                   x-x1   x2-x
     *   H(x) = [1 + 2 * -----][-----]^2 * f1 +
     *                   x2-x1  x2-x1
     *
     *                   x2-x   x-x1
     *          [1 + 2 * -----][-----]^2 * f2 +
     *                   x2-x1  x2-x1
     *
     *                          x2-x
     *                 [x - x1][-----]^2 * g1 +
     *                          x2-x1
     *
     *                          x-x1
     *                 [x - x2][-----]^2 * g2
     *                          x2-x1
     *
     * The minimizer of H(x) in [min_step_size, max_step_size] is either at one
     * of the end points, or else in the interior (given by setting H'(x) = 0).
     */
    double CubicHermiteInterpolation(const FunctionSample& s1,
                                     const FunctionSample& s2,
                                     double min_step_size,
                                     double max_step_size) const {
        double x1 = s1.x, f1 = s1.f, g1 = s1.g_norm;
        double x2 = s2.x, f2 = s2.f, g2 = s2.g_norm;

        double delta = (x2 - x1);
        CHECK(delta != 0.0);

        double d1 = g1 + g2 + 3.0 * (f1 - f2) / delta;
        double d2 = std::sqrt(d1 * d1 - g1 * g2);
        if (delta < 0.0) d2 = -d2;

        double x = x2 - delta * ((g2 + d2 - d1) / (g2 - g1 + 2.0 * d2));
        double t1 = (x2 - x) / delta;
        double t2 = (x - x1) / delta;
        double y = (1.0 + 2.0 * t2) * t1 * t1 * f1 +
                   (1.0 + 2.0 * t1) * t2 * t2 * f2 +
                   (x - x1) * t1 * t1 * g1 +
                   (x - x2) * t2 * t2 * g2;

        double step_size = 0.0;
        double optimal_value = DBL_MAX;
        if (x >= min_step_size && x <= max_step_size) {
            optimal_value = y;
            step_size = x;
        }
        if (x1 >= min_step_size && x1 <= max_step_size && f1 < optimal_value) {
            optimal_value = f1;
            step_size = x1;
        }
        if (x2 >= min_step_size && x2 <= max_step_size && f2 < optimal_value) {
            optimal_value = f2;
            step_size = x2;
        }

        return step_size;
    }

    /**
     * Return true if either:
     *
     * A termination condition satisfying the (strong) Wolfe bracketing
     * conditions is found:
     *
     *  - A valid point, defined as a bracket of zero width [zoom not required].
     *  - A valid bracket (of width > tolerance), [zoom required].
     *
     * Or, searching was stopped due to an 'artificial' constraint, i.e. not a
     * condition required by the underlying algorithm, but instead an
     * implementation consideration.
     *
     * If A step size which exceeds the minimum step size and satisfies the
     * Armijo condition, it should be used [zoom not required].
     *
     * Return false if no step size > minimum step size was found which
     * satisfies at least the Armijo condition.
     */
    bool BracketingPhase(LineSearchProblem* problem,
                         const FunctionSample& initial,
                         double initial_step_size,
                         FunctionSample* bracket_low,
                         FunctionSample* bracket_high,
                         bool* do_zoom_search,
                         Result* result) const {
        double descent_direction_max_norm = problem->direction.infinty_norm();
        *do_zoom_search = false;
        *bracket_low = initial;

        FunctionSample previous = initial, current;
        ++result->n_function_evaluations;
        ++result->n_gradient_evaluations;
        problem->Evaluate(initial_step_size, &current);

        while (true) {
            ++result->n_iterations;
            if ((current.f > initial.f + options_.sufficient_decrease *
                             initial.g_norm * current.x) ||
                current.f > previous.f) {
                // Bracket found: current step size violates Armijo sufficient
                // decrease condition, or has stepped past an inflection point
                // of f() relative to the previous step size.
                *do_zoom_search = true;
                *bracket_low = previous;
                *bracket_high = current;
                break;
            }

            if (std::fabs(current.g_norm) <=
                    -options_.sufficient_curvature_decrease * initial.g_norm) {
                // Current step size satisfies the strong Wolfe conditions, and
                // is thus a valid termination point, therefore a Zoom not
                // required.
                *bracket_low = current;
                *bracket_high = current;
                break;
            }

            if (current.g_norm >= 0.0) {
                // Bracket found: current step size has stepped past an
                // inflection point of f(), but Armijo sufficient decrease is
                // still satisfied and f(current) is our best minimum thus far.
                // Remember step size monotonically increases, thus
                // previous_step_size < current_step_size
                // even though f(previous) > f(current).
                *do_zoom_search = true;
                // Note inverse ordering from first bracket case.
                *bracket_low = current;
                *bracket_high = previous;
                break;
            }

            if (std::fabs(current.x - previous.x) *
                descent_direction_max_norm < options_.min_step_size) {
                // We have shrunk the search bracket to a width less than our
                // tolerance, and still not found either a point satisfying the
                // strong Wolfe conditions, or a valid bracket containing such a
                // point. Stop searching and set bracket_low to the size size
                // amongst all those tested which minimizes f() and satisfies
                // the Armijo condition.
                *bracket_low = current;
                break;
            }

            if (result->n_iterations >= options_.max_n_iterations) {
                // Check num iterations bound here so that we always evaluate
                // the max_n_iterations-th iteration against all conditions,
                // and then perform no additional (unused) evaluations.
                result->message =
                        Format("[Line search failed] Wolfe bracketing "
                               "phase failed to find a point satisfying "
                               "strong Wolfe conditions, or a bracket "
                               "containing such a point within specified "
                               "max_n_iterations: {}.",
                               options_.max_n_iterations);
                // Ensure that bracket_low is always set to the step size
                // amongst all those tested which minimizes f() and satisfies
                // the Armijo condition when we terminate due to the
                // 'artificial' max_n_iterations condition.
                *bracket_low = current.f < bracket_low->f ? current
                                                          : *bracket_low;
                break;
            }

            // If f(current) meets no criteria, we expand the search by
            // increasing the step size.
            double max_step_size = current.x * options_.max_step_expansion;
            const double step_size = CubicHermiteInterpolation(previous,
                                                               current,
                                                               previous.x,
                                                               max_step_size);
            if (step_size * descent_direction_max_norm <
                options_.min_step_size) {
                result->message = Format("[Line search failed] "
                                         "step_size too small: {:.5e} with "
                                         "descent_direction_max_norm: {:.5e}",
                                         step_size,
                                         descent_direction_max_norm);
                return false;
            }

            previous = current;
            ++result->n_function_evaluations;
            ++result->n_gradient_evaluations;
            problem->Evaluate(step_size, &current);
        }

        // Ensure that even if a valid bracket was found, we will only mark a
        // zoom as required if the bracket's width is greater than our minimum
        // tolerance.
        if (*do_zoom_search &&
            std::fabs(bracket_high->x - bracket_low->x) *
            descent_direction_max_norm < options_.min_step_size) {
            *do_zoom_search = false;
        }

        return true;
    }

    /**
     * Return true iff solution satisfies the strong Wolfe conditions.
     * Otherwise, on return false, if we stopped searching due to the
     * 'artificial' condition of reaching max_n_iterations, solution is the
     * step size amongst all those tested, which satisfied the Armijo decrease
     * condition and minimized f().
     */
    bool ZoomPhase(LineSearchProblem* problem,
                   const FunctionSample& initial,
                   FunctionSample bracket_low,
                   FunctionSample bracket_high,
                   FunctionSample* solution,
                   Result* result) const {
        if (bracket_low.g_norm * (bracket_high.x - bracket_low.x) >= 0.0) {
            // The third condition for a valid initial bracket:
            //
            //   3. bracket_high is chosen after bracket_low, s.t.
            //      bracket_low.gradient * (bracket_high.x - bracket_low.x) < 0.
            //
            // is not satisfied.  As this can happen when the users' cost
            // function returns inconsistent gradient values relative to the
            // function values, we stop processing and return an invalid value.
            result->message =
                    Format("[Line search failed] "
                           "Wolfe zoom phase passed a bracket which does not "
                           "satisfy: bracket_low.gradient * "
                           "(bracket_high.x - bracket_low.x) < 0 "
                           "[{:.8e} !< 0] with initial_position: "
                           "[{:.8e} {:.8e} {:.8e}], "
                           "bracket_low: [{:.8e} {:.8e} {:.8e}], "
                           "bracket_high: [{:.8e} {:.8e} {:.8e}]. "
                           "The most likely cause of which is the cost "
                           "function returning inconsistent "
                           "gradient & function values.",
                           bracket_low.g_norm * (bracket_high.x -
                                                 bracket_low.x),
                           initial.x,
                           initial.f,
                           initial.g_norm,
                           bracket_low.x,
                           bracket_low.f,
                           bracket_low.g_norm,
                           bracket_high.x,
                           bracket_high.f,
                           bracket_high.g_norm);
            return false;
        }

        const int n_bracketing_iterations = result->n_iterations;
        double descent_direction_max_norm = problem->direction.infinty_norm();

        while (true) {
            // Set solution to bracket_low, as it is our best step size
            // (smallest f()) found thus far and satisfies the Armijo condition,
            // even though it does not satisfy the Wolfe condition.
            *solution = bracket_low;
            if (result->n_iterations >= options_.max_n_iterations) {
                result->message =
                        Format("[Line search failed] Wolfe zoom phase failed to"
                               " find a point satisfying strong Wolfe "
                               "conditions within specified "
                               "max_n_iterations: {}, "
                               "(num iterations taken for bracketing: {}).",
                               options_.max_n_iterations,
                               n_bracketing_iterations);
                return false;
            }

            if (std::fabs(bracket_high.x - bracket_low.x) *
                descent_direction_max_norm < options_.min_step_size) {
                // Bracket width has been reduced below tolerance, and no point
                // satisfying the strong Wolfe conditions has been found.
                result->message =
                        Format("[Line search failed] Wolfe zoom bracket "
                               "width: {:.5e} too small with "
                               "descent_direction_max_norm: {:.5e}.",
                               std::fabs(bracket_high.x - bracket_low.x),
                               descent_direction_max_norm);
                return false;
            }

            // Polynomial interpolation requires inputs ordered according to
            // step size, not f(step size).
            const FunctionSample& lower_bound_step =
                    bracket_low.x < bracket_high.x ? bracket_low : bracket_high;
            const FunctionSample& upper_bound_step =
                    bracket_low.x < bracket_high.x ? bracket_high : bracket_low;

            ++result->n_iterations;
            double step_size = CubicHermiteInterpolation(lower_bound_step,
                                                         upper_bound_step,
                                                         lower_bound_step.x,
                                                         upper_bound_step.x);
            if (Equal(step_size, bracket_low.x)) {
                *solution = bracket_low;
            } else if (Equal(step_size, bracket_high.x)) {
                *solution = bracket_high;
            } else {
                ++result->n_function_evaluations;
                ++result->n_gradient_evaluations;
                problem->Evaluate(step_size, solution);
            }

            if ((solution->f > (initial.f + options_.sufficient_decrease *
                                initial.g_norm * solution->x)) ||
                (solution->f >= bracket_low.f)) {
                // Armijo sufficient decrease not satisfied, or not better
                // than current lowest sample, use as new upper bound.
                bracket_high = *solution;
                continue;
            }

            // Armijo sufficient decrease satisfied, check strong Wolfe
            // condition.
            if (std::fabs(solution->g_norm) <=
                    -options_.sufficient_curvature_decrease * initial.g_norm) {
                break;
            }
            if (solution->g_norm * (bracket_high.x - bracket_low.x) >= 0.0) {
                bracket_high = bracket_low;
            }

            bracket_low = *solution;
        }

        // Solution contains a valid point which satisfies the strong Wolfe
        // conditions.
        return true;
    }

    Options options_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LINE_SEARCH_H_

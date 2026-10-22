//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_NON_LINEAR_CONJUGATE_GRADIENT_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_NON_LINEAR_CONJUGATE_GRADIENT_H_

#include "codelibrary/optimization/unconstrained/line_search_minimizer.h"

namespace cl {
namespace optimization {

/**
 * In general, nonlinear conjugate gradient directions are much more effective
 * than the steepest descent direction and are simple to compute.
 *
 * It does not attain the fast convergence rates of Newton or quasi-Newton
 * methods, but they have the advantage of not requiring storage of matrices.
 *
 * Three non-linear conjugate gradient methods are included. They are
 * FLETCHER–REEVES method, POLAK-RIBIERE method, and HESTENES-STIEFEL method.
 *
 * Reference:
 *   Nocedal J, Wright S J. Numerical optimization [M]. Springer, 2006.
 */
class NonlinearConjugateGradient : public LineSearchMinimizer {
public:
    enum Method {
        FLETCHER_REEVES,
        POLAK_RIBIERE,
        HESTENES_STIEFEL
    };

    explicit NonlinearConjugateGradient(const Options& options = Options(),
                                        const Method& method = FLETCHER_REEVES)
      : LineSearchMinimizer(options),
        method_(method) {}

    NonlinearConjugateGradient(const NonlinearConjugateGradient&) = delete;

    NonlinearConjugateGradient&
    operator=(const NonlinearConjugateGradient&) = delete;

private:
    bool SearchDirection(const LineSearchMinimizer::State& previous,
                         const LineSearchMinimizer::State& current,
                         RVector* search_direction) override {
        double beta = 0.0;
        RVector gradient_change;
        switch (method_) {
        case FLETCHER_REEVES:
            beta = current.gradient_squared_norm /
                   previous.gradient_squared_norm;
            break;
        case POLAK_RIBIERE:
            gradient_change = current.gradient - previous.gradient;
            beta = current.gradient.dot(gradient_change) /
                   previous.gradient_squared_norm;
            break;
        case HESTENES_STIEFEL:
            gradient_change = current.gradient - previous.gradient;
            beta = current.gradient.dot(gradient_change) /
                   previous.search_direction.dot(gradient_change);
            break;
        default:
            LOG(FATAL) << "Unknown method: " << method_;
        }

        *search_direction = -current.gradient +
                            beta * previous.search_direction;
        double directional_derivative = current.gradient.dot(*search_direction);
        if (directional_derivative > -options_.function_tolerance) {
            LOG(WARNING) << "Restarting non-linear conjugate gradients: "
                         << directional_derivative;
            *search_direction = -current.gradient;
        }

        return true;
    }

private:
    Method method_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_NON_LINEAR_CONJUGATE_GRADIENT_H_

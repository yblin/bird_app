//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_H_

#include "codelibrary/optimization/unconstrained/line_search_minimizer.h"

namespace cl {
namespace optimization {

/**
 * Steepest descent is a first-order iterative optimization algorithm.
 * To find a local minimum of a function using gradient descent, one takes steps
 * proportional to the negative of the gradient of the function at the current
 * point.
 */
class SteepestDescent : public LineSearchMinimizer {
public:
    explicit SteepestDescent(const Options& options = Options())
        : LineSearchMinimizer(options) {}

    SteepestDescent(const SteepestDescent&) = delete;

    SteepestDescent& operator =(const SteepestDescent&) = delete;

private:
    bool SearchDirection(const State& /*previous*/,
                         const State& current,
                         RVector* search_direction) override {
        *search_direction = -current.gradient;
        return true;
    }
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_H_

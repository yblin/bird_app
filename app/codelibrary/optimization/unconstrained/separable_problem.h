//
// Copyright 2019-2024 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_SEPARABLE_PROBLEM_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_SEPARABLE_PROBLEM_H_

#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/unconstrained/problem.h"

namespace cl {
namespace optimization {

/**
 * Likely the most important type of function to be optimized in machine
 * learning and some other fields is the (differentiable) separable function.
 *
 * A separable function f(x) can be represented as the sum of many functions:
 *
 *   f(x) = f_0(x) + f_1(x) + ... + f_N(x).
 */
class SeparableProblem : public UnconstrainedProblem {
public:
    SeparableProblem(int n_parameters, int n_functions)
        : UnconstrainedProblem(n_parameters), n_functions_(n_functions) {
        CHECK(n_functions_ > 0);
    }

    virtual ~SeparableProblem() override = default;

    virtual void Evaluate(const RVector &x, double *f, RVector* g) override {
        CHECK(f);

        *f = 0.0;
        double f_i = 0.0;
        RVector g_i;
        if (g) g->Assign(n_parameters_, 0.0);

        for (int i = 0; i < n_functions_; ++i) {
            if (g) {
                Evaluate(x, i, &f_i, &g_i);
                *f += f_i;
                *g += g_i;
            } else {
                Evaluate(x, i, &f_i, nullptr);
                *f += f_i;
            }
        }
        *f /= n_functions_;
        if (g) *g *= 1.0 / n_functions_;
    }

    /**
     * Evaluate the i-th objective function value.
     * f must not be null. The gradient is computed only if g is not null.
     */
    virtual void Evaluate(const RVector& x, int i, double* f_i,
                          RVector* g_i) = 0;

    int n_functions() const {
        return n_functions_;
    }

protected:
    int n_functions_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_SEPARABLE_PROBLEM_H_

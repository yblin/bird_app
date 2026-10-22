//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_PROBLEM_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_PROBLEM_H_

#include "codelibrary/math/derivative.h"
#include "codelibrary/math/vector.h"

namespace cl {
namespace optimization {

/**
 * Unconstrained optimization problem consider the problem of minimizing an
 * objective function that depends on real variables with no restrictions on
 * their values.
 *
 * The mathematical formulation is
 *
 *    min f(x)
 *     x
 *
 * where x in R^n is a real vector with n components and f: R^n -> R is a smooth
 * function (i.e., its second derivative exists and is continuous).
 */
class UnconstrainedProblem {
public:
    // Dual type for automatic gradient evaluation.
    using Dual = DualNumber<double>;

    /**
     * Construct UnconstrainedProblem with cost function and number of
     * parameters.
     */
    explicit UnconstrainedProblem(int n_parameters)
        : n_parameters_(n_parameters) {
        CHECK(n_parameters > 0);
    }

    virtual ~UnconstrainedProblem() = default;

    /**
     * Users should define their specific cost function by overriding the
     * operator() method. The default implementation is incomplete and
     * should not be used directly.
     */
    virtual Dual operator()(const Dual*) {
        CHECK(false) << "You must override operator() for the specific cost "
                        "function!";
        return Dual();
    }

    /**
     * Evaluate the objective function value.
     *
     * The gradient is automatically computed if g is not null.
     * The user can also compute the gradient manually by overriding this
     * function.
     *
     * Parameters:
     *  x - Input vector.
     *  g - Gradient vector (if gradient computation is required).
     *
     * Return the value of the objective function.
     */
    virtual double Evaluate(const RVector& x, RVector* g) {
        CHECK(n_parameters_ == x.size());

        Array<Dual> d(x.begin(), x.end());
        double f = this->operator()(d.data()).value;
        if (g) {
            g->resize(x.size());
            for (int i = 0; i < x.size(); ++i) {
                (*g)[i] = Derivative(this, x)[i];
            }
        }
        return f;
    }

    int n_parameters() const {
        return n_parameters_;
    }

protected:
    // Number of parameters.
    int n_parameters_ = 0;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_PROBLEM_H_

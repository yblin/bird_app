//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Unconstrained problems for testing.
//
// Reference:
//   Jorge J. Moré, Garbow B S, Hillstrom K E. Testing Unconstrained
//   Optimization Software[J]. ACM Transactions on Mathematical Software, 1981,
//   7(1):17-41.

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TEST_PROBLEMS_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TEST_PROBLEMS_H_

#include "codelibrary/optimization/unconstrained/least_squares_problem.h"
#include "codelibrary/optimization/unconstrained/problem.h"

namespace cl {
namespace optimization {

/**
 * f     = (1 - x1)^2 + 10(x2 - x1^2)^2
 *
 * x0    = (-1.2, 1)
 * f_min = 0, at (1, 1)
 */
class RosenbrockFunction : public optimization::UnconstrainedProblem {
public:
    RosenbrockFunction()
        : optimization::UnconstrainedProblem(2) {}

    double Evaluate(const RVector& x, RVector* g) override {
        double f = (1.0 - x[0]) * (1.0 - x[0]) + 10.0 * (x[1] - x[0] * x[0]) *
                                                        (x[1] - x[0] * x[0]);
        g->resize(2);
        (*g)[0] = -2.0 * (1.0 - x[0]) - 40.0 * x[0] * (x[1] - x[0] * x[0]);
        (*g)[1] = 20.0 * (x[1] - x[0] * x[0]);
        return f;
    }
};

/**
 *         n-2
 * f     = sum((1 - x_i)^2 + 100(x_i+1 - x_i^2)^2)
 *         i=0
 *
 * x0    = (-1, -1, ..., -1)
 * f_min = 0, at (1, 1, ..., 1)
 */
class GeneralizedRosenbrockFunction : public UnconstrainedProblem {
public:
    explicit GeneralizedRosenbrockFunction(int n_parameters)
        : UnconstrainedProblem(n_parameters) {
        CHECK(n_parameters_ >= 2);
    }

    Dual operator()(const Dual* x) override {
        Dual f = 0.0;
        for (int i = 0; i < n_parameters_ - 1; ++i) {
            Dual t1 = (1.0 - x[i]);
            Dual t2 = x[i + 1] - x[i] * x[i];
            f += t1 * t1 + 100.0 * t2 * t2;
        }

        return f;
    }
};

/**
 * f      = (-13 + x1 + ((5 - x2)x2 - 2 )x2)^2 +
 *          (-29 + x1 + ((x2 + 1)x2 - 14)x2)^2
 *
 * x0     = (-0.5, -2)
 * f_min  = 0 at (5, 4) and (11.41, -0.8968)
 */
class FreudensteinRothFunction : public UnconstrainedProblem {
public:
    FreudensteinRothFunction() : UnconstrainedProblem(2) {}

    double Evaluate(const RVector& x, RVector* g) override {
        double t1 = (-13.0 + x[0] + ((5.0 - x[1]) * x[1] -  2.0) * x[1]);
        double t2 = (-29.0 + x[0] + ((x[1] + 1.0) * x[1] - 14.0) * x[1]);
        double f = t1 * t1 + t2 * t2;
        g->resize(2);
        (*g)[0] = 2.0 * t1 + 2.0 * t2;
        (*g)[1] = 2.0 * t1 * (10.0 * x[1] - 3.0 * x[1] * x[1] - 2.0) +
                  2.0 * t2 * (3.0 * x[1] * x[1] + 2.0 * x[1] - 14.0);
        return f;
    }
};

/**
 * Least squares form of FreudensteinRothFunction.
 */
class FreudensteinRothLeastSqauresProblem : public LeastSquaresProblem {
    struct Residual1 : public UnconstrainedProblem {
        Residual1() : UnconstrainedProblem(2) {}

        Dual operator()(const Dual* x) override {
            return -13.0 + x[0] + ((5.0 - x[1]) * x[1] -  2.0) * x[1];
        }
    };

    struct Residual2 : public UnconstrainedProblem {
        Residual2() : UnconstrainedProblem(2) {}

        Dual operator()(const Dual* x) override {
            return -29.0 + x[0] + ((x[1] + 1.0) * x[1] - 14.0) * x[1];
        }
    };

public:
    FreudensteinRothLeastSqauresProblem()
        : LeastSquaresProblem(2) {
        this->AddResidual(&residual1_);
        this->AddResidual(&residual2_);
    }

private:
    Residual1 residual1_;
    Residual2 residual2_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_TEST_PROBLEMS_H_

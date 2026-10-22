//
// Copyright 2017-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LBFGS_TEST_H_
#define CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LBFGS_TEST_H_

#include <memory>

#include "codelibrary/base/testing.h"
#include "codelibrary/optimization/unconstrained/lbfgs.h"
#include "codelibrary/optimization/unconstrained/test_problems.h"

namespace cl {
namespace test {

TEST(LBFGSTest, RosenbrockFunctionTest) {
    optimization::RosenbrockFunction problem;

    optimization::LBFGS minimier;
    RVector x(2);
    x[0] = -1.2;
    x[1] = 1.0;

    auto result = minimier.Minimize(&problem, &x);
    ASSERT_EQ(result.termination_type, optimization::Minimizer::CONVERGENCE);
    ASSERT_EQ_NEAR(x[0], 1.0, 1e-4);
    ASSERT_EQ_NEAR(x[1], 1.0, 1e-4);
}

TEST(LBFGSTest, GeneralizedRosenbrockFunctionTest) {
    optimization::GeneralizedRosenbrockFunction problem(100);

    optimization::LineSearch::Options o;
    optimization::LineSearch line_search(o);
    RVector x(problem.n_parameters(), -1.0);

    optimization::LBFGS::Options options;
    options.max_iterations = 10000;
    options.parameter_tolerance = 1e-10;
    optimization::LBFGS minimier(options);
    auto result = minimier.Minimize(&problem, &x);

    ASSERT_EQ(result.termination_type, optimization::Minimizer::CONVERGENCE);
    for (int i = 0; i < problem.n_parameters(); ++i) {
        ASSERT_EQ_NEAR(x[i], 1.0, 1e-4);
    }
}

TEST(LBFGSTest, FreudensteinRothFunctionTest) {
    optimization::FreudensteinRothFunction problem;

    optimization::LBFGS minimier;
    RVector x = make_vector(-0.5, 2.0);
    auto result = minimier.Minimize(&problem, &x);

    ASSERT_EQ(result.termination_type, optimization::Minimizer::CONVERGENCE);
    ASSERT_EQ_NEAR(x[0], 5.0, 1e-4);
    ASSERT_EQ_NEAR(x[1], 4.0, 1e-4);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LBFGS_TEST_H_

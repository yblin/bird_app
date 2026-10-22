//
// Copyright 2017 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_TEST_H_
#define CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_TEST_H_

#include <memory>

#include "codelibrary/base/testing.h"
#include "codelibrary/optimization/unconstrained/steepest_descent.h"
#include "codelibrary/optimization/unconstrained/test_problems.h"

namespace cl {
namespace test {

TEST(SteepestDescentTest, RosenbrockFunctionTest) {
    std::unique_ptr<optimization::UnconstrainedProblem> problem(
                new optimization::RosenbrockFunction());

    optimization::LineSearchMinimizer::Options options;
    options.max_iterations = 10000;
    optimization::SteepestDescent minimier(options);

    RVector x = make_vector(-1.2, 1.0);
    optimization::LineSearchMinimizer::Result result =
            minimier.Minimize(problem.get(), &x);

    ASSERT_EQ(result.termination_type, optimization::Minimizer::CONVERGENCE);
    ASSERT_EQ_NEAR(x[0], 1.0, 1e-4);
    ASSERT_EQ_NEAR(x[1], 1.0, 1e-4);
}

TEST(SteepestDescentTest, FreudensteinRothFunctionTest) {
    std::unique_ptr<optimization::UnconstrainedProblem> problem(
                new optimization::FreudensteinRothFunction());

    optimization::LineSearchMinimizer::Options options;
    options.max_iterations = 10000;
    optimization::SteepestDescent minimier(options);

    RVector x = make_vector(-0.5, 2.0);
    optimization::LineSearchMinimizer::Result result =
            minimier.Minimize(problem.get(), &x);

    ASSERT_EQ(result.termination_type, optimization::Minimizer::CONVERGENCE);
    ASSERT_EQ_NEAR(x[0], 5.0, 1e-3);
    ASSERT_EQ_NEAR(x[1], 4.0, 1e-3);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_STEEPEST_DESCENT_TEST_H_

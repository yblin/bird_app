//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
#ifndef CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_TEST_H_
#define CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/optimization/unconstrained/test_problems.h"

namespace cl {
namespace test {

TEST(LeastSqauresProblem, RosenbrockFunctionTest) {
    optimization::FreudensteinRothLeastSqauresProblem problem;

    optimization::FreudensteinRothFunction func;

    RVector x = make_vector(-0.5, -2.0);
    RVector g;
    problem.Evaluate(x, &g);
    ASSERT_EQ_NEAR(g[0], 26.0, 1e-8);
    ASSERT_EQ_NEAR(g[1], -1192.0, 1e-8);

    RVector r;
    CSR3SparseMatrix<double> j;
    problem.Evaluate(x, &g, &r, &j);
    ASSERT_EQ_NEAR(g[0], 26.0, 1e-8);
    ASSERT_EQ_NEAR(g[1], -1192.0, 1e-8);

    ASSERT_EQ_NEAR(r[0], 18.5, 1e-8);
    ASSERT_EQ_NEAR(r[1], -5.5, 1e-8);

    ASSERT_EQ(j.n_elements, 4);
    ASSERT_EQ_NEAR(j.values[0], 1.0, 1e-8);
    ASSERT_EQ_NEAR(j.values[1], -34.0, 1e-8);
    ASSERT_EQ_NEAR(j.values[2], 1.0, 1e-8);
    ASSERT_EQ_NEAR(j.values[3], -6.0, 1e-8);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_TEST_H_

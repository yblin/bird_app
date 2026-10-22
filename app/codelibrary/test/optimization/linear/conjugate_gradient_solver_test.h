//
// Copyright 2017-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_TEST_H_
#define CODELIBRARY_TEST_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/matrix/matrix.h"
#include "codelibrary/optimization/linear/conjugate_gradient_solver.h"

namespace cl {
namespace test {

TEST(ConjugateGradientTest, Solves2x2) {
    RMatrix a(2, 2);
    a(0, 0) = 4.0;
    a(0, 1) = 1.0;
    a(1, 0) = 1.0;
    a(1, 1) = 3.0;

    RVector b(2);
    b[0] = 1.0;
    b[1] = 2.0;

    optimization::ConjugateGradientSolver::Options options;
    optimization::ConjugateGradientSolver solver(options);
    RVector x;
    optimization::LinearSolver::Result res = solver.Solve(a, b, &x);
    ASSERT_EQ(res.termination_type, optimization::LinearSolver::SUCCESS);

    ASSERT_EQ_NEAR(x[0], 0.0909, 1e-4); // x[0] = 1/11.
    ASSERT_EQ_NEAR(x[1], 0.6364, 1e-4); // x[1] = 7/11.
}

TEST(ConjugateGraientTest, Solves3x3Identity) {
    RMatrix a = RMatrix::Identity(3);

    RVector b(3);
    b[0] = 1.0;
    b[1] = 2.0;
    b[2] = 3.0;

    RVector x(3);
    x[0] = 1;
    x[1] = 1;
    x[2] = 1;

    optimization::ConjugateGradientSolver::Options options;
    options.max_num_iterations = 10;
    options.r_tolerance = 1e-9;

    optimization::ConjugateGradientSolver solver(options);
    optimization::LinearSolver::Result res = solver.Solve(a, b, &x);

    ASSERT_EQ(res.termination_type, optimization::LinearSolver::SUCCESS);
    ASSERT_EQ(res.num_iterations, 1);

    ASSERT_EQ(x[0], 1.0);
    ASSERT_EQ(x[1], 2.0);
    ASSERT_EQ(x[2], 3.0);
}

TEST(ConjugateGradientTest, Solves3x3) {
    optimization::ConjugateGradientSolver::Options o;
    o.q_tolerance = 1e-9;
    o.r_tolerance = 1e-9;
    optimization::ConjugateGradientSolver s(o);

    //      | 2  -1  0|
    //  A = |-1   2 -1| is symmetric positive definite matrix.
    //      | 0  -1  2|
    Array<double> a_data = {2.0,  -1.0,  0.0,
                            -1.0,  2.0, -1.0,
                            0.0,  -1.0,  2.0};
    RMatrix a(3, 3, a_data);
    RVector b(3), x(3, 1.0);
    b[0] = -1.0;
    b[1] =  0.0;
    b[2] =  3.0;

    optimization::LinearSolver::Result res = s.Solve(a, b, &x);
    ASSERT_EQ(res.termination_type, optimization::LinearSolver::SUCCESS);

    ASSERT_EQ(x.size(), 3);
    ASSERT_EQ(x[0], 0.0);
    ASSERT_EQ(x[1], 1.0);
    ASSERT_EQ(x[2], 2.0);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_OPTIMIZATION_LINEAR_CONJUGATE_GRADIENT_SOLVER_TEST_H_

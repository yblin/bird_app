//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_H_

#include "codelibrary/math/matrix/sparse_matrix.h"
#include "codelibrary/optimization/unconstrained/problem.h"

namespace cl {
namespace optimization {

/**
 * An unconstrained (non-linear) least square problem has the form:
 *
 *  f(x) = Sum(||r_i(x)||^2)
 *
 * where, each r_i(.) is a smooth function from R^n to R. We refer to each r_i
 * as a residual.
 */
class LeastSquaresProblem : public UnconstrainedProblem {
public:
    explicit LeastSquaresProblem(int n_parameters)
        : UnconstrainedProblem(n_parameters) {}

    /**
     * Add a residual term into the problem.
     */
    void AddResidual(UnconstrainedProblem* problem) {
        CHECK(problem);
        CHECK(problem->n_parameters() == n_parameters_);

        residuals_.push_back(problem);
    }

    double Evaluate(const RVector& x, RVector* g) override {
        CHECK(n_parameters_ == x.size());

        if (g) {
            g->assign(this->n_parameters_, 0.0);
        }

        double f = 0.0;
        RVector residual_g(n_parameters_);
        for (auto residual : residuals_) {
            double r = residual->Evaluate(x, &residual_g);
            f += r * r;
            if (g) {
                *g += 2.0 * r * residual_g;
            }
        }
        return f;
    }

    /**
     * Evaluate the function cost, gradient, residual and jacobian.
     *
     * Jacobian(i, j) = dr_i / dx_j.
     */
    double Evaluate(const RVector& x, RVector* g, RVector* residuals,
                    CSR3SparseMatrix<double>* jacobian) {
        CHECK(g);
        CHECK(residuals);
        CHECK(jacobian);

        residuals->resize(residuals_.size());
        g->assign(this->n_parameters_, 0.0);

        CoordinateSparseMatrix<double> coord;
        coord.n_rows = this->n_residuals();
        coord.n_columns = this->n_parameters();

        RVector residual_g(n_parameters_);
        double f = 0.0;
        for (int i = 0; i < residuals_.size(); ++i) {
            double r = residuals_[i]->Evaluate(x, &residual_g);
            *g += 2.0 * r * residual_g;
            (*residuals)[i] = r;

            for (int j = 0; j < residual_g.size(); ++j) {
                double v = residual_g[j];
                if (v != 0.0) {
                    coord.rows.push_back(i);
                    coord.columns.push_back(j);
                    coord.values.push_back(v);
                    coord.n_elements++;
                }
            }
            f += r * r;
        }

        sparse_matrix::Convert(coord, jacobian);
        return f;
    }

    /**
     * Return the number of residuals.
     */
    int n_residuals() const {
        return residuals_.size();
    }

protected:
    Array<UnconstrainedProblem*> residuals_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LEAST_SQUARES_PROBLEM_H_

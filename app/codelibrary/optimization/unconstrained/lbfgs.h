//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LBFGS_H_
#define CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LBFGS_H_

#include <deque>

#include "codelibrary/base/array.h"
#include "codelibrary/math/vector.h"
#include "codelibrary/optimization/unconstrained/line_search_minimizer.h"

namespace cl {
namespace optimization {

/**
 * Limited-memory BFGS (Broyden-Fletcher-Goldfarb-Shanno).
 *
 * Forming the n x n dense matrix H_k can be quite expensive for large problems.
 * L-BFGS stores a limited history of the BFGS update vectors delta_x and
 * delta_gradient (which are size n), and use these to "implicitly" form the
 * matrix operations.
 *
 * Reference:
 *  Nocedal J. Updating quasi-Newton matrices with limited storage [J].
 *  Mathematics of Computation, 1980, 35(35):773-782.
 */
class LBFGS : public LineSearchMinimizer {
public:
    struct Options : public LineSearchMinimizer::Options {
        Options() : LineSearchMinimizer::Options() {}

        int max_rank = 8; // Rank of the Hessian approximation.
    };

    explicit LBFGS(const Options& options = Options())
        : LineSearchMinimizer(options) {
        CHECK(options.max_rank > 0);

        max_rank_ = options.max_rank;
    }

private:
    bool SearchDirection(const State& previous, const State& current,
                         RVector* search_direction) override {
        CHECK(is_positive_definite_) <<
            "Inverse Hessian approximation has become indefinite.";

        if (n_parameters_ == 0) {
            n_parameters_ = current.gradient.size();
            InitializeLowRankInverseHessian();
        }
        UpdateInverseHessian(previous.search_direction * previous.step_size,
                             current.gradient - previous.gradient);
        Multiply(current.gradient, search_direction);
        *search_direction = -*search_direction;

        if (current.gradient.dot(*search_direction) >= 0.0) {
            LOG(WARNING) << "Numerical failure in L-BFGS update: inverse "
                         << "Hessian approximation is not positive definite, "
                         << "and thus initial gradient for search direction is "
                         << "positive: "
                         << search_direction->dot(current.gradient);
            is_positive_definite_ = false;
            return false;
        }

        return true;
    }

    void RestartSearchDirection() override {
        InitializeLowRankInverseHessian();
    }

    /**
     * Initialize a positive definite approximation to the Hessian using the
     * limited memory variant of the Broyden-Fletcher-Goldfarb-Shanno(BFGS)
     * secant formula for approximating the Hessian.
     */
    void InitializeLowRankInverseHessian() {
        CHECK(n_parameters_ > 0);

        is_positive_definite_ = true;
        delta_x_history_.resize(max_rank_);
        delta_gradient_history_.resize(max_rank_);
        for (int i = 0; i < max_rank_; ++i) {
            delta_x_history_[i].assign(n_parameters_, 0.0);
            delta_gradient_history_[i].assign(n_parameters_, 0.0);
        }
        delta_x_dot_delta_gradient_.assign(max_rank_, 0.0);

        indices_.clear();
    }

    /**
     * Update the low rank inverse Hessian.
     *
     * Parameters:
     *  delta_x        - the change in the domain of Hessian.
     *  delta_gradient - the change in the gradient.
     */
    bool UpdateInverseHessian(const RVector& delta_x,
                              const RVector& delta_gradient) {
        const double tolerance = 1e-14;
        const double delta_x_dot_delta_gradient = delta_x.dot(delta_gradient);
        if (delta_x_dot_delta_gradient <= tolerance) return false;

        int next = static_cast<int>(indices_.size());

        // Once the size of the list reaches max_rank_, simulate a circular
        // buffer by removing the first element of the list and making it the
        // next position where the LBFGS history is stored.
        if (next == max_rank_) {
            next = indices_.front();
            indices_.pop_front();
        }

        indices_.push_back(next);
        delta_x_history_[next] = delta_x;
        delta_gradient_history_[next] = delta_gradient;
        delta_x_dot_delta_gradient_[next] = delta_x_dot_delta_gradient;
        return true;
    }

    /**
     * Multiply the Hessian with a given vector.
     */
    void Multiply(const RVector& x, RVector* y) {
        *y = x;
        RVector alpha(static_cast<int>(indices_.size()));

        for (auto it = indices_.rbegin(); it != indices_.rend(); ++it) {
            double alpha_i = delta_x_history_[*it].dot(*y) /
                             delta_x_dot_delta_gradient_[*it];
            *y -= alpha_i * delta_gradient_history_[*it];
            alpha[*it] = alpha_i;
        }

        for (int index : indices_) {
            double beta = delta_gradient_history_[index].dot(*y) /
                          delta_x_dot_delta_gradient_[index];
            *y += delta_x_history_[index] * (alpha[index] - beta);
        }
    }

    // Number of n_parameters.
    int n_parameters_ = 0;

    // Set true if the input matrix is positive definite.
    bool is_positive_definite_ = true;

    // The rank of the Hessian approximation.
    int max_rank_;

    // For inverse Hessian approximation.
    Array<RVector> delta_x_history_;
    Array<RVector> delta_gradient_history_;
    RVector delta_x_dot_delta_gradient_;
    std::deque<int> indices_;
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_UNCONSTRAINED_LBFGS_H_

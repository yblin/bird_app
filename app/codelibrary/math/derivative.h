//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_DERIVATIVE_H_
#define CODELIBRARY_MATH_DERIVATIVE_H_

#include "codelibrary/math/vector.h"
#include "codelibrary/math/number/dual_number.h"

namespace cl {

/**
 * Compute derivative of the function on the given paramters.
 *
 * ------------------------------------------------------
 * Sample usage [1]:
 * ------------------------------------------------------
 *
 *  struct Function {
 *  public:
 *      template <typename T>
 *      T operator()(const T& x, const T& y) const {
 *          return x * x + x * y;
 *      }
 *  };
 *
 *  Function function;
 *  LOG(INFO) << Derivative(&function, 1.0, 2.0)[0];
 *
 * ------------------------------------------------------
 * Sample usage [2]:
 * ------------------------------------------------------
 *
 *  struct Function {
 *      template <typename T>
 *      T operator()(const T* x) const {
 *          return x[0] * x[0] + x[0] * x[1];
 *      }
 *  };
 *
 *  Function function;
 *  RVector x = make_vector(1.0, 2.0);
 *  LOG(INFO) << Derivative(&function, x)[0];
 *
 * ------------------------------------------------------
 * Sample usage [3]:
 * ------------------------------------------------------
 *
 *  template <typename T>
 *  T Function(const T& x, const T& y) {
 *      return x * x + x * y;
 *  }
 *
 *  LOG(INFO) << Derivative(&Function<DualNumber<double>>, 1.0, 2.0)[0];
 *
 *
 * ----------------------------------------------------------------------
 * This class can also get the second-order derivatives, for example:
 *
 * struct CostFunction {
 *     template <typename T>
 *     T operator()(const T& x, const T& y) const {
 *         return x * x + x * y;
 *     }
 * };
 *
 * CostFunction function;
 *
 * The following code will get the Partial Derivative of d^2f / dxdy
 * Derivative(&function,
 *            DualNumber<double>(1.0, 1.0),
 *            DualNumber<double>(2.0, 0.0))[1].dual;
 *
 *
 * However, it is not encouraged to use this method to obtain the second
 * derivative, because the efficiency is too low. The best way is to manually
 * obtain the analytical solution.
 */
template <typename T, class Function, int D = 0>
class DerivativeSystem {
    static_assert(D >= 0, "");

    using Dual = DualNumber<T>;

    /**
     * N = 0 for functions with dynamic number of parameters.
     */
    template <class Func, int N>
    struct Evaluator;

    template <class Func>
    struct Evaluator<Func, 0> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x.data());
        }
    };
    template <class Func>
    struct Evaluator<Func, 1> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 2> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 3> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 4> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 5> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 6> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 7> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 8> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]);
        }
    };
    template <class Func>
    struct Evaluator<Func, 9> {
        Dual operator()(Func* f, const Vector<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]);
        }
    };

public:
    DerivativeSystem(Function* function, const Vector<T>& value)
        : function_(function), values_(value) {
        CHECK(function_);
        if (D > 0) CHECK(values_.size() == D);
    }

    /**
     * Return the first-order partial derivative value of the i-th paramter,
     * i.e., df/dx_i
     */
    T operator[] (int i) const {
        CHECK(i >= 0 && i < values_.size());

        Vector<Dual> duals(values_.size());
        for (int j = 0; j < values_.size(); ++j) {
            duals[j] = values_[j];
        }
        duals[i].dual = T(1);
        Evaluator<Function, D> evaluator;
        return evaluator(function_, duals).dual;
    }

private:
    Function* function_ = nullptr;
    Vector<T> values_;
};

template <typename T, class Func>
DerivativeSystem<T, Func, 0> Derivative(Func* func, const Vector<T>& a) {
    return DerivativeSystem<T, Func, 0>(func, a);
}
template <typename T, class Func>
DerivativeSystem<T, Func, 1> Derivative(Func* func, T a) {
    return DerivativeSystem<T, Func, 1>(func, make_vector(a));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 2> Derivative(Func* func, T a, T b) {
    return DerivativeSystem<T, Func, 2>(func, make_vector(a, b));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 3> Derivative(Func* func, T a, T b, T c) {
    return DerivativeSystem<T, Func, 3>(func, make_vector(a, b, c));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 4> Derivative(Func* func, T a, T b, T c, T d) {
    return DerivativeSystem<T, Func, 4>(func, make_vector(a, b, c, d));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 5> Derivative(Func* func, T a, T b, T c, T d, T e) {
    return DerivativeSystem<T, Func, 5>(func, make_vector(a, b, c, d, e));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 6> Derivative(Func* func, T a, T b, T c, T d,
                                        T e, T f) {
    return DerivativeSystem<T, Func, 6>(func, make_vector(a, b, c, d, e, f));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 7> Derivative(Func* func, T a, T b, T c, T d,
                                        T e, T f, T g) {
    return DerivativeSystem<T, Func, 7>(func, make_vector(a, b, c, d, e, f, g));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 8> Derivative(Func* func, T a, T b, T c, T d,
                                        T e, T f, T g, T h) {
    return DerivativeSystem<T, Func, 8>(func,
                                        make_vector(a, b, c, d, e, f, g, h));
}
template <typename T, class Func>
DerivativeSystem<T, Func, 9> Derivative(Func* func, T a, T b, T c, T d,
                                        T e, T f, T g, T h, T i) {
    return DerivativeSystem<T, Func, 9>(func,
                                        make_vector(a, b, c, d, e, f, g, h, i));
}

} // namespace cl

#endif // CODELIBRARY_MATH_DERIVATIVE_H_

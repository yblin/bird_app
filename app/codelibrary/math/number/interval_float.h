//
// Copyright 2017-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_NUMBER_INTERVAL_FLOAT_H_
#define CODELIBRARY_MATH_NUMBER_INTERVAL_FLOAT_H_

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cfenv>
#include <cstdint>
#include <stdexcept>

// Compiler-specific floating-point precision control macros
#if defined(_MSC_VER)
#define CL_INTERVAL_FLOAT_BEGIN                \
    __pragma(float_control(precise, on, push)) \
        __pragma(float_control(except, on)) __pragma(fenv_access(on))
#define CL_INTERVAL_FLOAT_END \
    __pragma(fenv_access(off)) __pragma(float_control(pop))
#elif defined(__clang__)
#if __clang_major__ >= 13
#define CL_INTERVAL_FLOAT_BEGIN _Pragma("STDC FENV_ACCESS ON")
#define CL_INTERVAL_FLOAT_END _Pragma("STDC FENV_ACCESS OFF")
#else
#warning \
    "Clang 13 or higher is required for IntervalFloat support (needs #pragma STDC FENV_ACCESS)"
#define CL_INTERVAL_FLOAT_BEGIN
#define CL_INTERVAL_FLOAT_END
#endif
#elif defined(__GNUC__)

#if (__GNUC__ < 10)
// Before GCC 10: disable -Wattributes. It is a false-positive warning from
// older gcc.  Note that this version cutoff is a guess. If you encounter this
// warning on GCC 10, increase the cutoff to GCC 11.
#define CL_FLOAT_EXTRA_ATTR_BEGIN \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wattributes\"")

#define CL_FLOAT_EXTRA_ATTR_END \
    _Pragma("GCC diagnostic pop")

#else
// no need to disable the warning on higher gcc versions
#define CL_FLOAT_EXTRA_ATTR_BEGIN
#define CL_FLOAT_EXTRA_ATTR_END
#endif

#define CL_INTERVAL_FLOAT_BEGIN \
    _Pragma("GCC push_options") _Pragma("GCC optimize (\"-frounding-math\")") \
    CL_FLOAT_EXTRA_ATTR_BEGIN
#define CL_INTERVAL_FLOAT_END \
    CL_FLOAT_EXTRA_ATTR_END _Pragma("GCC pop_options")
#else
#error "Unsupported compiler for IntervalFloat. Please use MSVC, GCC, or Clang."
#endif

CL_INTERVAL_FLOAT_BEGIN

namespace cl {

// sets up floating point rounding for the given floating point class.
template <typename T>
class ScopedRounding;

/**
 * A floating number type defined in interval form. It allows interval
 * arithmetics which is useful in the geometric predicates.
 *
 * Use ScopedRounding<IntervalFloat> to automatically manage FE_UPWARD rounding
 * mode in a thread-safe manner.
 *
 * Wrap all code using IntervalFloat with CL_INTERVAL_FLOAT_BEGIN and
 * CL_INTERVAL_FLOAT_END macros to ensure proper compiler floating-point
 * precision control.
 *
 * Rule of thumb for safe usage:
 *  1. When arithmetic methods of IntervalFloat are used, ScopedRounding must be
 *     active
 *  2. When ScopedRounding is used, it must be within
 *     CL_INTERVAL_FLOAT_BEGIN/CL_INTERVAL_FLOAT_END scope
 *
 * Due to runtime perfomance considerations, we do not check those requirements.
 */
class IntervalFloat {
public:
    IntervalFloat() = default;
    explicit IntervalFloat(int32_t n) : lower_(-n), upper_(n) {}
    explicit IntervalFloat(float n) : lower_(-n), upper_(n) {}
    explicit IntervalFloat(double n) : lower_(-n), upper_(n) {}

    IntervalFloat(double lower, double upper) : lower_(lower), upper_(upper) {}

    IntervalFloat(const IntervalFloat&) = default;

    IntervalFloat(IntervalFloat&&) = default;

    IntervalFloat& operator=(const IntervalFloat&) = default;

    IntervalFloat& operator=(IntervalFloat&&) = default;

    double lower() const {
        return -lower_;
    }

    double upper() const {
        return upper_;
    }

    /**
     * This = a + b.
     */
    IntervalFloat& Add(const IntervalFloat& a, const IntervalFloat& b) {
        lower_ = a.lower_ + b.lower_;
        upper_ = a.upper_ + b.upper_;

        return *this;
    }

    /**
     * This = a - b.
     */
    IntervalFloat& Subtract(const IntervalFloat& a, const IntervalFloat& b) {
        double l = a.lower_ + b.upper_;  // For the case: this == &b.
        upper_ = a.upper_ + b.lower_;
        lower_ = l;

        return *this;
    }

    /**
     * This = a * b.
     */
    IntervalFloat& Multiply(const IntervalFloat& a, const IntervalFloat& b) {
        double l = 0.0, u = 0.0;  // For the case: this == &a or this == &b.

        int cfg = ((a.lower_ < 0.0) << 3) + ((a.upper() < 0.0) << 2) +
                  ((b.lower_ < 0.0) << 1) + (b.upper_ < 0.0);

        switch (cfg) {
            case 10:
                l = a.lower_ * (-b.lower_);
                u = a.upper_ * b.upper_;
                break;
            case 8:
                l = a.upper_ * b.lower_;
                u = a.upper_ * b.upper_;
                break;
            case 9:
                l = a.upper_ * b.lower_;
                u = (-a.lower_) * b.upper_;
                break;
            case 2:
                l = a.lower_ * b.upper_;
                u = a.upper_ * b.upper_;
                break;
            case 0:
                l = std::max(a.lower_ * b.upper_, a.upper_ * b.lower_);
                u = std::max(a.upper_ * b.upper_, a.lower_ * b.lower_);
                break;
            case 1:
                l = a.upper_ * b.lower_;
                u = a.lower_ * b.lower_;
                break;
            case 6:
                l = a.lower_ * b.upper_;
                u = a.upper_ * (-b.lower_);
                break;
            case 4:
                l = a.lower_ * b.upper_;
                u = a.lower_ * b.lower_;
                break;
            case 5:
                l = (-a.upper_) * b.upper_;
                u = a.lower_ * b.lower_;
                break;
        };

        lower_ = l;
        upper_ = u;

        return *this;
    }

    IntervalFloat& operator+=(const IntervalFloat& a) {
        return Add(*this, a);
    }

    IntervalFloat& operator-=(const IntervalFloat& a) {
        return Subtract(*this, a);
    }

    IntervalFloat& operator*=(const IntervalFloat& a) {
        return Multiply(*this, a);
    }

    int sign() const {
        return (lower() > 0.0) ? 1 : ((upper() < 0.0) ? -1 : 0);
    }

    friend IntervalFloat operator-(const IntervalFloat& a) {
        IntervalFloat r;
        r.lower_ = -a.upper_;
        r.upper_ = -a.lower_;
        return r;
    }

    friend IntervalFloat operator+(const IntervalFloat& a,
                                   const IntervalFloat& b) {
        IntervalFloat r;
        r.Add(a, b);
        return r;
    }

    friend IntervalFloat operator-(const IntervalFloat& a,
                                   const IntervalFloat& b) {
        IntervalFloat r;
        r.Subtract(a, b);
        return r;
    }

    friend IntervalFloat operator*(const IntervalFloat& a,
                                   const IntervalFloat& b) {
        IntervalFloat r;
        r.Multiply(a, b);
        return r;
    }

private:
    double lower_ = 0.0, upper_ = 0.0;  // Two bounds of interval.
};

template <>
class ScopedRounding<IntervalFloat> {
public:
    // fesetround() / fegetround() are PER-THREAD on Linux (FPU control word
    // is per-thread). The previous process-wide ref-counted scheme was a race:
    // only the first thread to enter set FE_UPWARD; subsequent concurrent
    // threads kept their default rounding mode and produced WRONG interval
    // bounds, which silently corrupted Orientation() / InCircle() results
    // and eventually broke Delaunay/Arrangement DCEL.
    //
    // Now each scope manages its own thread's rounding mode independently.
    ScopedRounding() : saved_round_mode_(fegetround()) {
        if (saved_round_mode_ != FE_UPWARD) {
            if (fesetround(FE_UPWARD)) {
                fprintf(stderr, "Error: Failed to set rounding mode!\n");
                std::abort();
            }
        }
    }

    ~ScopedRounding() {
        if (saved_round_mode_ != FE_UPWARD) {
            fesetround(saved_round_mode_);
        }
    }

    ScopedRounding(const ScopedRounding&) = delete;
    ScopedRounding& operator=(const ScopedRounding&) = delete;

private:
    int saved_round_mode_;
};

}  // namespace cl

CL_INTERVAL_FLOAT_END

#endif  // CODELIBRARY_MATH_NUMBER_INTERVAL_FLOAT_H_

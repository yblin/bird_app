//
// Copyright 2014-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_METRIC_ANGULAR_H_
#define CODELIBRARY_UTIL_METRIC_ANGULAR_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/util/metric/cosine.h"

namespace cl {
namespace metric {

/**
 * The metric to measure the angle between two vectors.
 *
 * The range of angular metric is [0, 1].
 */
class Angular {
public:
    Angular() = default;

    Angular(const Angular&) = delete;

    Angular& operator=(const Angular&) = delete;

    template <typename T>
    double operator() (const T& a, const T& b) const {
        return std::acos(cosine_(a, b)) / numbers::pi_v<double>;
    }

private:
    Cosine cosine_;
};

} // namespace metric
} // namespace cl

#endif // CODELIBRARY_UTIL_METRIC_ANGULAR_H_

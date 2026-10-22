//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SMOOTHING_SMOOTHING_SCORER_H_
#define CODELIBRARY_GIS_SMOOTHING_SMOOTHING_SCORER_H_

#include "codelibrary/geometry/angle.h"

namespace cl {
namespace gis {

/**
 * Compute the discrete bin score for the angle between two vectors.
 *
 * This scorer quantizes the angle between two vectors into discrete bins of
 * configurable width (default 2 degrees). Vectors with angular differences that
 * fall into the same bin will receive the same score, effectively treating
 * small angle variations as equivalent for smoothing purposes.
 *
 * Example (bin_width=2.0):
 * - Angles [0-2) -> bin 0 -> score 0
 * - Angles [2-4) -> bin 1 -> score 1
 * - Angles [4-6) -> bin 2 -> score 2
 * - ...
 */
class SmoothingScorer {
public:
    /**
     * Construct a scorer with specified bin width
     *
     * Parameters:
     *  bin_width - Angular width of each bin in degrees (must be > 0).
     */
    explicit SmoothingScorer(int bin_width = 2)
        : bin_width_(bin_width) {
        CHECK(bin_width_ > 0.0) << "Bin width must be positive";
    }

    /**
     * Compute the discrete bin score for the angle between two vectors.
     */
    int operator()(const RVector2D& v1, const RVector2D& v2) const {
        return static_cast<int>(Degree(v1, v2) / bin_width_);
    }

private:
    int bin_width_; // Width of each bin in degrees.
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SMOOTHING_SMOOTHING_SCORER_H_

//
// Copyright 2015-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_AXIS_H_
#define CODELIBRARY_PLOT_AXIS_H_

#include <climits>
#include <cmath>
#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/base/clamp.h"

namespace cl {
namespace plot {

/**
 * Axis for Plot.
 */
class Axis {
public:
    /**
     * Construct a axis by given range and maximal number of ticks.
     * Note that, the data range not necessarily the same as the axis range.
     *
     * Parameters:
     *  min       - the expected minimal value of axis.
     *  max       - the expected maximal value of axis.
     *  max_ticks - the expected maximal number of ticks on the axis.
     */
    explicit Axis(double min = 0.0, double max = 1.0, int max_ticks = 10) {
        Reset(min, max, max_ticks);
    }

    /**
     * Reset this axis.
     */
    void Reset(double min, double max, int max_ticks = 10) {
        CHECK(min <= max);
        CHECK(max_ticks > 0);

        ChooseInterval(min, max, max_ticks);
        GenerateTicks(min, max);
    }

    /**
     * Return the minimum value of tick on the axis.
     */
    double min() const {
        return min_;
    }

    /**
     * Return the maximal value of tick on the axis.
     */
    double max() const {
        return max_;
    }

    /**
     * Return the length of axis.
     */
    double length() const {
        return max_ - min_;
    }

    /**
     * Return the interval between two neighbor ticks.
     */
    double interval() const {
        return interval_;
    }

    /**
     * Manually set an interval.
     */
    void SetInterval(double interval) {
        CHECK(interval > 0.0);
        CHECK(length() < INT_MAX * interval) <<
              "The given interval can not be too small.";

        interval_ = interval;
        GenerateTicks(min(), max());
    }

    /**
     * Return the tick values.
     */
    const Array<double>& ticks() const {
        return ticks_;
    }

    /**
     * Return the tick labels.
     */
    const Array<std::string>& tick_labels() const {
        return tick_labels_;
    }

    /**
     * Reverse the direction of axis.
     */
    void Reverse() {
        is_increasing_ ^= 1;
    }

    /**
     * Return true if the axis value is increasing.
     */
    bool is_increasing() const {
        return is_increasing_;
    }

    /**
     * Fix axes range or not.
     */
    void set_range_fixed(bool fixed) {
        range_fixed_ = fixed;
    }

private:
    /**
     * Automatically choose a usable ticking interval.
     */
    void ChooseInterval(double min, double max, int max_ticks) {
        // Order of magnitude of argument.
        double length = max - min;
        if (length == 0.0) length = 1.0;
        double power = std::pow(10.0, std::floor(std::log10(length)));

        // Approximate number of decades, we expect that 1 <= xnorm <= 10.
        double xnorm = length / power;
        xnorm = Clamp(xnorm, 1.0, 10.0);

        // Approximate number of ticks per decade.
        double n_ticks = max_ticks / xnorm;
        if (n_ticks > 20.0) {
            interval_ = 0.05;         // e.g. 0, 0.05, 0.10, ...
        } else if (n_ticks > 10.0) {
            interval_ = 0.1;          // e.g. 0, 0.1, 0.2, ...
        } else if (n_ticks > 5.0) {
            interval_ = 0.2;          // e.g. 0, 0.2, 0.4, ...
        } else if (n_ticks > 2.0) {
            interval_ = 0.5;          // e.g. 0, 0.5, 1, ...
        } else if (n_ticks > 1.0) {
            interval_ = 1.0;          // e.g. 0, 1, 2, ...
        } else if (n_ticks > 0.5) {
            interval_ = 2.0;          // e.g. 0, 2, 4, ...
        } else {
            interval_ = std::ceil(xnorm);
        }

        interval_ *= power;
    }

    /**
     * Generate tick values and labels.
     */
    void GenerateTicks(double min, double max) {
        if (range_fixed_) {
            min_ = min;
            max_ = max;
        }

        double start = std::floor(min / interval_);
        double end   = std::ceil(max / interval_);
        CHECK(start <= end);

        ticks_.clear();
        for (double i = start; i <= end; i += 1.0) {
            AddTick(i * interval_);
        }
        if (ticks_.size() == 1) {
            AddTick((end + 1) * interval_);
        }

        // Generate tick labels.
        std::string str;
        tick_labels_.clear();
        for (double x : ticks_) {
            str = Format("{:g}", x);
            tick_labels_.push_back(str);
        }

        if (!range_fixed_) {
            min_ = ticks_.front();
            max_ = ticks_.back();
        }
    }

    /**
     * Add a tick value.
     */
    void AddTick(double value) {
        if (range_fixed_) {
            if (value >= min_ && value <= max_) {
                ticks_.push_back(value);
            }
        } else {
            ticks_.push_back(value);
        }
    }

    // The interval between two neighbor ticks.
    double interval_ = 0.0;

    // Position of ticks.
    Array<double> ticks_;

    // Labels for every tick.
    Array<std::string> tick_labels_;

    // True if the values are increasing.
    bool is_increasing_ = true;

    // Are min and max range fixed or not.
    bool range_fixed_ = false;

    // The min and max range of the axes.
    double min_, max_;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_AXIS_H_

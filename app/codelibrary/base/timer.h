//
// Copyright 2018-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_TIMER_H_
#define CODELIBRARY_BASE_TIMER_H_

#include <string>
#include <chrono>

#include "codelibrary/base/log.h"

namespace cl {

/**
 * A timer for calculating user's process time.
 */
class Timer {
public:
    Timer() {
        Reset();
    }

    /**
     * Reset the timer.
     */
    void Reset() {
        elapsed_ = std::chrono::duration<double, std::ratio<1, 1>>::zero();
        if (running_) {
            started_ = std::chrono::steady_clock::now();
        }
    }

    /**
     * Timer start (continue).
     */
    void Start() {
        running_ = true;
        started_ = std::chrono::steady_clock::now();
    }

    /**
     * Resets the timer and starts it from zero.
     */
    void Restart() {
        elapsed_ = std::chrono::duration<double, std::ratio<1, 1>>::zero();
        running_ = true;
        started_ = std::chrono::steady_clock::now();
    }

    /**
     * Stops (pauses) the timer.
     * The timer can be resumed with Start(), and the elapsed time continues to
     * accumulate.
     */
    void Stop() {
        if (running_) {
            running_ = false;
            elapsed_ += (std::chrono::steady_clock::now() - started_);
        }
    }

    /**
     * Record the elapsed time without stopping the timer.
     */
    void Record() {
        if (running_) {
            auto t = std::chrono::steady_clock::now();
            elapsed_ += (t - started_);
            started_ = t;
        }
    }

    /**
     * Return the elapsed time in seconds.
     */
    double elapsed_seconds() const {
        return elapsed_.count() * 1e-9;
    }

    /**
     * Return the human readable elapsed time.
     */
    std::string elapsed_time() const {
        return ReadableTime(elapsed_seconds());
    }

    /**
     * Return the average time: elapsed_time / n_times.
     */
    std::string average_time(int n_times) const {
        CHECK(n_times > 0);

        return ReadableTime(elapsed_seconds() / n_times);
    }

    /**
     * Convert a time in seconds to human readable string.
     */
    static std::string ReadableTime(double time, int n_precision = 2) {
        CHECK(n_precision >= 0);

        if (time < 1e-6) {
            return Format("{}ns", static_cast<int>(time * 1e9));
        }

        if (time < 1e-3) {
            return Format("{}us", static_cast<int>(time * 1e6));
        }

        if (time < 1.0) {
            return Format("{}ms", static_cast<int>(time * 1e3));
        }

        if (time < 60.0) {
            return Format("{:.{}f}s", time, n_precision);
        }

        time /= 60.0;
        if (time < 60.0) {
            return Format("{:.{}f}min", time, n_precision);
        }

        time /= 60.0;
        if (time < 24.0) {
            return Format("{:.{}f}h", time, n_precision);
        }

        time /= 24.0;
        if (time < 30.0) {
            return Format("{:.{}f} days", time, n_precision);
        }

        if (time < 365.2425) {
            return Format("{:.{}f} months", time / 30.43687, n_precision);
        }

        return Format("{:.{}f} years", time / 365.2425, n_precision);
    }

private:
    // True if timer is running.
    bool running_ = false;

    // The elapsed time (in ns).
    std::chrono::duration<double, std::ratio<1, 1000000000>> elapsed_;

    // The time of started.
    std::chrono::steady_clock::time_point started_;
};

} // namespace cl

#endif // CODELIBRARY_BASE_TIMER_H_

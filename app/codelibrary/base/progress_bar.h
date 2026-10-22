//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_PROGRESS_BAR_H_
#define CODELIBRARY_BASE_PROGRESS_BAR_H_

#include "codelibrary/base/arange.h"
#include "codelibrary/base/array.h"
#include "codelibrary/base/format.h"
#include "codelibrary/base/timer.h"

namespace cl {

/**
 * A class to create a progress bar for iterating over a range of elements.
 */
template <typename ContainerIterator>
class ProgressBarRange {
    static const int BAR_WIDTH = 50;

    using ValueType = typename std::iterator_traits<ContainerIterator>::
                               value_type;

    // Iterator for ProgressBarRange.
    class Iterator {
        friend class ProgressBarRange;

    public:
        Iterator(ContainerIterator it, ProgressBarRange* range)
            : it_(it), range_(range) {}

        bool operator == (const Iterator& rhs) const {
            return it_ == rhs.it_;
        }

        bool operator != (const Iterator& rhs) const {
            return !(*this == rhs);
        }

        auto& operator*() { return *it_; }

        Iterator& operator ++() {
            ++it_;
            range_->Increment();

            return *this;
        }

    private:
        ContainerIterator it_;
        ProgressBarRange* range_;
    };

public:
    /**
     * Build ProgressBarRange from data in [first, last).
     */
    ProgressBarRange(ContainerIterator begin, ContainerIterator end)
        : begin_(begin), end_(end) {
        total_ = std::distance(begin, end);
        timer_.Start();
        if (total_ < 1000) {
            period_ = 1;
        } else {
            period_ = total_ / 1000;
        }

        if (begin == end) {
            std::string s = GetBarString(1.0);
            Print("\r{}  [{:>5.1f}% | {}/{} | {}]        \n",
                  s, 100.0, cur_, total_, 0.0);
        }
        fflush(stdout);
    }

    Iterator begin() {
        return Iterator(begin_, this);
    }

    Iterator end() {
        return Iterator(end_, this);
    }

    /**
     * Increment progress by one.
     */
    void Increment() {
        ++cur_;

        if (cur_ >= total_) {
            // Finish.
            std::string s = GetBarString(1.0);
            timer_.Stop();
            Print("\r{}  [{:>5.1f}% | {}/{} | {}]        \n",
                  s, 100.0, cur_, total_, timer_.elapsed_time());
            fflush(stdout);
        } else if (cur_ % period_ == 0) {
            double r = static_cast<double>(cur_) / total_;
            std::string s = GetBarString(r);
            timer_.Record();

            double secs = timer_.elapsed_seconds();
            Print("\r{}  [{:>5.1f}% | {}/{} | {}<{}]    ",
                  s, 100.0 * r, cur_, total_,
                  Timer::ReadableTime(secs, 0),
                  Timer::ReadableTime(secs * (total_ - cur_) / cur_, 0));
            fflush(stdout);
        }
    }

private:
    /**
     * Return progress bar.
     */
    std::string GetBarString(double progress) const {
        static const Array<std::string> bars = {" ", "▏", "▎", "▍", "▌", "▋",
                                                "▊", "▉", "█"};
        // Finish.
        std::string s;
        if (progress >= 1.0) {
            for (int i = 0; i < BAR_WIDTH; ++i) {
                s += bars.back();
            }
        } else {
            int pos = std::max(static_cast<int>(BAR_WIDTH * progress), 0);
            double sub = BAR_WIDTH * progress - pos;

            int i = 0;
            for (i = 0; i < pos; ++i) {
                s += bars.back();
            }
            if (i + 1 != BAR_WIDTH) {
                int sub_pos = sub * (bars.size() - 1);
                if (sub_pos > 0) {
                    ++i;
                    s += bars[std::min(sub_pos, bars.size() - 1)];
                }
            }
            for (; i < BAR_WIDTH; ++i) {
                s += ' ';
            }
        }
        return s;
    }

    ContainerIterator begin_;
    ContainerIterator end_;
    size_t cur_ = 0;
    size_t total_ = 0;
    size_t period_ = 1;
    Timer timer_;
};

template <class Container,
          class Bar = ProgressBarRange<typename Container::const_iterator>>
Bar ProgressBar(const Container& c) {
    return Bar(c.begin(), c.end());
}

template <class Container,
          class Bar = ProgressBarRange<typename Container::iterator>>
Bar ProgressBar(Container& c) {
    return Bar(c.begin(), c.end());
}

/**
 * Create a progress bar for integers [0, total).
 */
inline ProgressBarRange<ArangeIterator<int>::Iterator> ProgressBar(int total) {
    ArangeIterator<int> seq(0, total, 1);
    return ProgressBarRange<ArangeIterator<int>::Iterator>(seq.begin(),
                                                           seq.end());
}

/**
 * Create a progress bar for Range [start, end).
 */
template <typename T,
          class Bar = ProgressBarRange<typename ArangeIterator<T>::Iterator>>
Bar ProgressBar(const T& start, const T& end, const T& step = T(1)) {
    ArangeIterator<T> seq(start, end, step);
    return Bar(seq.begin(), seq.end());
}

} // namespace cl

#endif // CODELIBRARY_BASE_PROGRESS_BAR_H_

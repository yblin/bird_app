//
// Copyright 2011-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_SEGMENT_2D_H_
#define CODELIBRARY_GEOMETRY_SEGMENT_2D_H_

#include <algorithm>

#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {

/**
 * 2D line segment.
 *
 * Note: In this implementation, the segments defined as Segment(a, b)
 * and Segment(b, a) are considered equivalent. That is, the order of
 * the endpoints does not affect the identity of the segment.
 */
template <typename T>
class Segment2D {
public:
    using Point = Point2D<T>;
    using value_type = T;

    Segment2D() = default;

    Segment2D(const Point& p1, const Point& p2)
        : lower_point_(std::min(p1, p2)),
          upper_point_(std::max(p1, p2)),
          bounding_box_(std::min(p1.x, p2.x), std::max(p1.x, p2.x),
                        std::min(p1.y, p2.y), std::max(p1.y, p2.y)) {}

    /**
     * Return true if the segment is empty (invalid).
     */
    bool empty() const {
        return bounding_box_.empty();
    }

    /**
     * The left bottom endpoint of line segment.
     */
    const Point& lower_point() const {
        return lower_point_;
    }

    /**
     * The right top endpoint of line segment.
     */
    const Point& upper_point() const {
        return upper_point_;
    }

    /**
     * The bounding box of line segment.
     */
    const Box2D<T>& bounding_box() const {
        return bounding_box_;
    }

    /**
     * Return the direction of line segment.
     */
    Vector2D<T> direction() const {
        return upper_point_ - lower_point_;
    }

    /**
     * Return the length of this line segment.
     */
    T length() const {
        return direction().norm();
    }

    bool operator ==(const Segment2D& rhs) const {
        return lower_point_ == rhs.lower_point_ &&
               upper_point_ == rhs.upper_point_;
    }

    bool operator !=(const Segment2D& rhs) const {
        return !(*this == rhs);
    }

    bool operator <(const Segment2D& rhs) const {
        return lower_point_ < rhs.lower_point_ ||
               (lower_point_ == rhs.lower_point_ &&
                upper_point_ < rhs.upper_point_);
    }

    friend std::ostream& operator <<(std::ostream& os, const Segment2D& seg) {
        Message msg(seg.lower_point(), seg.upper_point());
        return os << msg;
    }

protected:
    Point lower_point_;     // The left bottom endpoint of line segment.
    Point upper_point_;     // The right top endpoint of line segment.
    Box2D<T> bounding_box_; // The bounding box of line segment.
};

using ISegment2D = Segment2D<int>;
using FSegment2D = Segment2D<float>;
using RSegment2D = Segment2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_SEGMENT_2D_H_

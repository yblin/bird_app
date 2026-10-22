//
// Copyright 2022-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POLYLINE_H_
#define CODELIBRARY_GEOMETRY_POLYLINE_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * ND Polyline. It is implemented like Array<Point>.
 */
template <typename Point>
class Polyline : public Array<Point> {
    using T = typename Point::value_type;

public:
    using value_type = T;

    /**
     * Unlike Segment2D and Segment3D, Polyline::Segment has a defined
     * direction.
     * Specifically, Segment(p1, p2) is distinct from Segment(p2, p1),
     * indicating that the segment direction from p1 to p2 matters.
     */
    struct Segment {
        Segment() = default;

        Segment(Point p1, Point p2)
            : source(p1), target(p2) {}

        Point source, target;
    };

    Polyline() = default;

    explicit Polyline(const Array<Point>& points)
        : Array<Point>(points) {}

    Polyline(const Point& s, const Point& t) {
        this->push_back(s);
        this->push_back(t);
    }

    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Polyline(Iter first, Iter last)
        : Array<Point>(first, last) {}

    /**
     * Return length of the polyline.
     */
    double Length() const {
        double l = 0.0;
        for (int i = 0; i + 1 < this->size(); ++i) {
            l += Distance(this->operator[](i), this->operator[](i + 1));
        }
        return l;
    }

    /**
     * Reverse the polyline.
     */
    void Reverse() {
        std::reverse(this->begin(), this->end());
    }

    /**
     * Split polyline into two polylines at i-th point.
     */
    void Split(int index, Polyline* polyline1, Polyline* polyline2) const {
        CHECK(index >= 0 && index < this->size());
        CHECK(this != polyline1 && this != polyline2);

        polyline1->assign(this->begin(), this->begin() + index + 1);
        polyline2->assign(this->begin() + index, this->end());
    }

    /**
     * Remove all duplicate vertices in the polyline.
     */
    void Trim() {
        this->erase(std::unique(this->begin(), this->end()), this->end());
    }

    /**
     * Return the i-th segment of the polyline.
     */
    Segment segment(int i) const {
        CHECK(i >= 0 && i + 1 < this->size());

        return Segment(this->operator[](i), this->operator[](i + 1));
    }

    Polyline& operator=(const Array<Point>& rhs) {
        Array<Point>::operator=(rhs);
        return *this;
    }

    friend std::ostream& operator <<(std::ostream& os,
                                     const Polyline& polyline) {
        Message msg(polyline.begin(), polyline.end());
        return os << msg;
    }
};

using IPolyline2D = Polyline<IPoint2D>;
using FPolyline2D = Polyline<FPoint2D>;
using RPolyline2D = Polyline<RPoint2D>;

using IPolyline3D = Polyline<IPoint3D>;
using FPolyline3D = Polyline<FPoint3D>;
using RPolyline3D = Polyline<RPoint3D>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POLYLINE_H_

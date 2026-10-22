//
// Copyright 2014-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Robust intersection test between 2D kernel geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_INTERSECT_2D_H_
#define CODELIBRARY_GEOMETRY_INTERSECT_2D_H_

#include <algorithm>
#include <cfloat>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_compare_2d.h"
#include "codelibrary/geometry/polygon_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/predicate_2d.h"
#include "codelibrary/geometry/range_ray_2d.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/triangle_2d.h"

namespace cl {
namespace geometry {

////////////////////////////////////////////////////////////////////////////////
// All Intersect() functions are used to test weather two objects intersect
// (including touch and contain).
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication and construction.
 */
template <typename T>
bool Intersect(const Box2D<T>& box1, const Box2D<T>& box2,
               Box2D<T>* res = nullptr) {
    if (box1.x_max() < box2.x_min() || box1.x_min() > box2.x_max() ||
        box1.y_max() < box2.y_min() || box1.y_min() > box2.y_max())
        return false;

    if (res) {
        *res = Box2D<T>(std::max(box1.x_min(), box2.x_min()),
                        std::min(box1.x_max(), box2.x_max()),
                        std::max(box1.y_min(), box2.y_min()),
                        std::min(box1.y_max(), box2.y_max()));
    }
    return true;
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Box2D<T>& box) {
    return (p.x >= box.x_min() && p.x <= box.x_max() &&
            p.y >= box.y_min() && p.y <= box.y_max());
}
template <typename T>
bool Intersect(const Box2D<T>& box, const Point2D<T>& p) {
    return Intersect(p, box);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Segment2D<T>& seg) {
    return p >= seg.lower_point() && p <= seg.upper_point() &&
           Orientation(seg.lower_point(), seg.upper_point(), p) == 0;
}
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Point2D<T>& p) {
    return Intersect(p, seg);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Line2D<T>& line) {
    return Orientation(line.point1(), line.point2(), p) == 0;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Point2D<T>& p) {
    return Intersect(p, line);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Triangle2D<T>& triangle) {
    if (triangle.empty()) return false;

    if (!Intersect(p, triangle.bounding_box())) return false;

    const Array<Point2D<T>>& vertices = triangle.vertices();
    int o1 = Orientation(vertices[0], vertices[1], p);
    if (o1 == 0) {
        if (p >= std::min(vertices[0], vertices[1]) &&
            p <= std::max(vertices[0], vertices[1])) return true;
    }

    int o2 = Orientation(vertices[1], vertices[2], p);
    if (o2 == 0) {
        return (p >= std::min(vertices[1], vertices[2]) &&
                p <= std::max(vertices[1], vertices[2]));
    }

    // Now, p is not on the edge of triangle.
    if (o1 != o2) return false;

    int o3 = Orientation(vertices[2], vertices[0], p);
    if (o3 == 0) {
        return (p >= std::min(vertices[2], vertices[0]) &&
                p <= std::max(vertices[2], vertices[0]));
    }
    return o1 == o3;
}
template <typename T>
bool Intersect(const Triangle2D<T>& triangle, const Point2D<T>& p) {
    return Intersect(p, triangle);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Polygon2D<T>& polygon) {
    if (!Intersect(p, polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (int i = 0; i < polygon.size(); ++i) {
        const Segment2D<T> seg = polygon.edge(i);
        if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) continue;

        int o = Orientation(seg.lower_point(), seg.upper_point(), p);
        if (o == 0) return true;
        if (o < 0) ++count;
    }
    return count % 2 != 0;
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Point2D<T>& p) {
    return Intersect(p, polygon);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const MultiPolygon2D<T>& multi_polygon) {
    if (!Intersect(p, multi_polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (const auto& b : multi_polygon.boundaries()) {
        const Polygon2D<T>& polygon = b.polygon;
        if (!Intersect(p, polygon.bounding_box())) continue;

        for (int j = 0; j < polygon.size(); ++j) {
            Segment2D<T> seg = polygon.edge(j);
            if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) {
                continue;
            }

            int o = Orientation(seg.lower_point(), seg.upper_point(), p);
            if (o == 0) return true;
            if (o < 0)  ++count;
        }
    }

    return (count % 2 != 0);
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& multi_polygon, const Point2D<T>& p) {
    return Intersect(p, multi_polygon);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Polyline<Point2D<T>>& polyline) {
    if (polyline.size() == 1) return p == polyline.front();

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        if (Intersect(p, Segment2D<T>(polyline[i], polyline[i + 1])))
            return true;
    }
    return false;
}
template <typename T>
bool Intersect(const Polyline<Point2D<T>>& polyline, const Point2D<T>& p) {
    return Intersect(p, polyline);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Line2D<T>& line) {
    int o1 = Orientation(line.point1(), line.point2(), seg.lower_point());
    if (o1 == 0) return true;

    int o2 = Orientation(line.point1(), line.point2(), seg.upper_point());
    if (o2 == 0) return true;

    return o1 != o2;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Segment2D<T>& seg) {
    return Intersect(seg, line);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Line2D<T>& line,
               Array<Point2D<T>>* intersections) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(intersections);

    intersections->clear();

    int o1 = Orientation(line.point1(), line.point2(), seg.lower_point());
    if (o1 == 0) {
        intersections->push_back(seg.lower_point());
    }

    int o2 = Orientation(line.point1(), line.point2(), seg.upper_point());
    if (o2 == 0) {
        intersections->push_back(seg.upper_point());
    }

    if (o1 == 0 || o2 == 0) return true;
    if (o1 == o2) return false;

    double x1 = line.point1().x, y1 = line.point1().y;
    double x2 = line.point2().x, y2 = line.point2().y;
    double x3 = seg.lower_point().x, y3 = seg.lower_point().y;
    double x4 = seg.upper_point().x, y4 = seg.upper_point().y;
    double t = ((x1 - x2) * (y1 - y3) - (x1 - x3) * (y1 - y2)) /
               ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
    if (!std::isfinite(t) || std::abs(t) < DBL_EPSILON) {
        t = 0.0;
    } else {
        t = Clamp(t, 0.0, 1.0);
    }

    if (t == 0.0) {
        intersections->push_back(seg.lower_point());
    } else if (t == 1.0) {
        intersections->push_back(seg.upper_point());
    } else {
        intersections->emplace_back(x3 + t * (x4 - x3), y3 + t * (y4 - y3));
    }
    return true;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Segment2D<T>& seg,
               Array<Point2D<T>>* intersections) {
    return Intersect(seg, line, intersections);
}

/**
 * Exact predication, inexact construction.
 *
 * Unlike the previous one, this function returns only one intersection point.
 * If the line segment lies entirely on the line, the lower endpoint of the
 * segment (by coordinate order) is returned as the result.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Line2D<T>& line,
               Point2D<T>* intersection) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(intersection);

    Array<Point2D<T>> inters;
    if (Intersect(seg, line, &inters)) {
        CHECK(!inters.empty());
        *intersection = inters.front();
        return true;
    }

    return false;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Segment2D<T>& seg,
               Point2D<T>* intersection) {
    return Intersect(seg, line, intersection);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Segment2D<T>& s1, const Segment2D<T>& s2) {
    if (!Intersect(s1.bounding_box(), s2.bounding_box())) return false;

    // Cross test.
    int o1 = Orientation(s1.lower_point(), s1.upper_point(), s2.lower_point());
    int o2 = Orientation(s1.lower_point(), s1.upper_point(), s2.upper_point());
    if (o1 == o2 && o1 != 0) {
        // Two endpoints of s2 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    int o3 = Orientation(s2.lower_point(), s2.upper_point(), s1.lower_point());
    int o4 = Orientation(s2.lower_point(), s2.upper_point(), s1.upper_point());

    // Two endpoints of s1 lay on the same side (left or right) of s2, thus
    // two segments never be intersected.
    return !(o3 == o4 && o3 != 0);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& s1, const Segment2D<T>& s2,
               Array<Point2D<T>>* intersections) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(intersections);

    if (!Intersect(s1.bounding_box(), s2.bounding_box())) return false;

    // Cross test.
    int o1 = Orientation(s1.lower_point(), s1.upper_point(), s2.lower_point());
    int o2 = Orientation(s1.lower_point(), s1.upper_point(), s2.upper_point());
    if (o1 == o2 && o1 != 0) {
        // Two endpoints of s2 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    int o3 = Orientation(s2.lower_point(), s2.upper_point(), s1.lower_point());
    int o4 = Orientation(s2.lower_point(), s2.upper_point(), s1.upper_point());

    // Two endpoints of s1 lay on the same side (left or right) of s2, thus
    // two segments never be intersected.
    if (o3 == o4 && o3 != 0) {
        return false;
    }

    if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0) {
        Point2D<T> p1 = std::max(s1.lower_point(), s2.lower_point());
        Point2D<T> p2 = std::min(s1.upper_point(), s2.upper_point());
        intersections->push_back(p1);
        if (p1 != p2) {
            intersections->push_back(p2);
        }
        return true;
    }

    // There must be an intersection between two segments.
    Line2D<T> line(s1.lower_point(), s1.upper_point());
    return Intersect(line, s2, intersections);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const Box2D<T>& box) {
    if (box.empty()) return false;

    Point2D<T> p1(box.x_min(), box.y_min());
    Point2D<T> p2(box.x_max(), box.y_min());
    Point2D<T> p3(box.x_max(), box.y_max());
    Point2D<T> p4(box.x_min(), box.y_max());
    return Intersect(line, Segment2D<T>(p1, p2)) ||
           Intersect(line, Segment2D<T>(p2, p3)) ||
           Intersect(line, Segment2D<T>(p3, p4)) ||
           Intersect(line, Segment2D<T>(p4, p1));
}
template <typename T>
bool Intersect(const Box2D<T>& box, const Line2D<T>& line) {
    return Intersect(line, box);
}

/**
 * Exact predication, inexact construction.
 *
 * It optionally returns the unordered intersection points. The number of
 * intersection points is always even, so users can easily get the intersection
 * line segments by sorting these points.
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const Polygon2D<T>& polygon,
               Array<Point2D<T>>* res = nullptr) {
    using Point = Point2D<T>;

    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!Intersect(line, polygon.bounding_box())) return false;

    // Make sure the polygon is in anti-clockwise.
    int n = polygon.size();
    // Generate 1 to n.
    Array<int> seq(n);
    std::iota(seq.begin(), seq.end(), 0);
    if (polygon.IsClockwise()) std::reverse(seq.begin(), seq.end());

    Point p1 = std::min(line.point1(), line.point2());
    Point p2 = std::max(line.point1(), line.point2());

    // Store the intersection points.
    int cur_o = 0, next_o = 0, prev_o = 0;
    for (int i = 0; i < n; ++i) {
        const Point& cur_p  = polygon.vertices()[seq[i]];
        const Point& next_p = i + 1 < n ? polygon.vertex(seq[i + 1])
                                        : polygon.vertex(seq[0]);
        const Point& prev_p = i == 0 ? polygon.vertex(seq.back())
                                     : polygon.vertex(seq[i - 1]);

        // Cross test.
        if (i == 0) {
            prev_o = Orientation(p1, p2, prev_p);
            cur_o  = Orientation(p1, p2, cur_p);
        } else {
            prev_o = cur_o;
            cur_o = next_o;
        }
        next_o = Orientation(p1, p2, next_p);

        if (cur_o == next_o && cur_o != 0) continue;
        if (cur_o == 0) {
            if (prev_o == next_o) {
                if (prev_o != 0) {
                    if (res) {
                        res->push_back(cur_p);
                        res->push_back(cur_p);
                    } else {
                        return true;
                    }
                }
                continue;
            }

            // prev_o != next_o
            if (prev_o == 0) {
                if (cur_p < prev_p && next_o < 0) {
                    if (res)
                        res->push_back(cur_p);
                    else
                        return true;
                }
                if (prev_p < cur_p && next_o > 0) {
                    if (res)
                        res->push_back(cur_p);
                    else
                        return true;
                }
            } else if (next_o == 0) {
                if (next_p < cur_p && prev_o < 0) {
                    if (res)
                        res->push_back(cur_p);
                    else
                        return true;
                }
                if (cur_p < next_p && prev_o > 0) {
                    if (res)
                        res->push_back(cur_p);
                    else
                        return true;
                }
            } else {
                if (res)
                    res->push_back(cur_p);
                else
                    return true;
            }
        } else if (next_o != 0) {
            // cur_o != next_o && next_o != 0 && cur_o != 0
            if (res) {
                Point2D<T> p;

                double x1 = p1.x, y1 = p1.y;
                double x2 = p2.x, y2 = p2.y;
                double x3 = cur_p.x, y3 = cur_p.y;
                double x4 = next_p.x, y4 = next_p.y;
                double t = ((x1 - x2) * (y1 - y3) - (x1 - x3) * (y1 - y2)) /
                           ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
                if (!std::isfinite(t) || std::abs(t) < DBL_EPSILON) {
                    t = 0.0;
                } else {
                    t = Clamp(t, 0.0, 1.0);
                }

                if (t == 0.0) {
                    p = cur_p;
                } else if (t == 1.0) {
                    p = next_p;
                } else {
                    p.x = x3 + t * (x4 - x3);
                    p.y = y3 + t * (y4 - y3);
                }

                res->push_back(p);
            } else {
                return true;
            }
        }
    }

    return (res) ? (!res->empty()) : false;
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Line2D<T>& line,
               Array<Point2D<T>>* res = nullptr) {
    return Intersect(line, polygon, res);
}

/**
 * Exact predication, inexact construction.
 *
 * See the previous one.
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const MultiPolygon2D<T>& polygon,
               Array<Point2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!Intersect(line, polygon.bounding_box())) return false;

    for (const auto& b : polygon.boundaries()) {
        if (res) {
            Array<Point2D<T>> tmp;
            if (Intersect(line, b.polygon, &tmp)) res->insert(tmp);
        } else {
            if (Intersect(line, b.polygon)) return true;
        }
    }

    return (res) ? (!res->empty()) : false;
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon, const Line2D<T>& line,
               Array<Point2D<T>>* res = nullptr) {
    return Intersect(line, polygon, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const MultiPolygon2D<T>& polygon,
               Array<Segment2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!res) {
        if (!Intersect(seg.bounding_box(), polygon.bounding_box()))
            return false;

        if (Intersect(seg.lower_point(), polygon) ||
            Intersect(seg.upper_point(), polygon)) return true;

        for (int i = 0; i < polygon.n_boundaries(); ++i) {
            const Polygon2D<T>& poly = polygon.boundaries()[i].polygon;
            for (int j = 0; j < poly.size(); ++j) {
                if (Intersect(seg, poly.edge(j))) return true;
            }
        }

        return false;
    } else {
        Line2D<T> line(seg.lower_point(), seg.upper_point());

        Array<Point2D<T>> ps;
        if (!Intersect(line, polygon, &ps)) return false;
        CHECK(ps.size() % 2 == 0);

        PointDotCompare2D<T> compare(seg);
        std::sort(ps.begin(), ps.end(), compare);
        for (int i = 0; i < ps.size(); i += 2) {
            if (compare(seg.upper_point(), ps[i]) ||
                compare(ps[i + 1], seg.lower_point())) continue;

            res->emplace_back(std::max(ps[i], seg.lower_point(), compare),
                              std::min(ps[i + 1], seg.upper_point(), compare));
        }

        return !res->empty();
    }
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon, const Segment2D<T>& seg,
               Array<Segment2D<T>>* res = nullptr) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication, inexact construction.
 *
 * It return the ordered intersection points. The number of intersection points
 * is always even, so users can easily get the intersection line segments.
 */
template <typename T>
bool Intersect(const typename Polyline<Point2D<T>>::Segment& seg,
               const MultiPolygon2D<T>& polygon,
               Array<Point2D<T>>* res) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(res);

    res->clear();

    Line2D<T> line(seg.source, seg.target);
    Array<Point2D<T>> ps;
    if (!Intersect(line, polygon, &ps)) return false;
    CHECK(ps.size() % 2 == 0);

    PointDotCompare2D<T> compare(seg.source, seg.target);
    std::sort(ps.begin(), ps.end(), compare);
    for (int i = 0; i < ps.size(); i += 2) {
        if (compare(seg.target, ps[i]) || compare(ps[i + 1], seg.source))
            continue;

        res->push_back(std::max(ps[i], seg.source, compare));
        res->push_back(std::min(ps[i + 1], seg.target, compare));
    }

    return !res->empty();
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon,
               const typename Polyline<Point2D<T>>::Segment& seg,
               Array<Point2D<T>>* res) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const typename Polyline<Point2D<T>>::Segment& seg,
               const MultiPolygon2D<T>& polygon,
               Array<typename Polyline<Point2D<T>>::Segment>* res) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(res);
    res->clear();

    Array<Point2D<T>> intersection;
    Intersect(seg, polygon, &intersection);
    CHECK(intersection.size() % 2 == 0);

    for (int i = 0; i < intersection.size(); i += 2) {
        res->emplace_back(intersection[i], intersection[i + 1]);
    }

    return !res->empty();
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon,
               const typename Polyline<Point2D<T>>::Segment& seg,
               Array<typename Polyline<Point2D<T>>::Segment>* res) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Polyline<Point2D<T>>& polyline) {
    if (polyline.empty()) return false;
    if (polyline.size() == 1) {
        return Intersect(seg, polyline.front());
    }

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        Segment2D<T> s(polyline[i], polyline[i + 1]);
        if (Intersect(seg, s)) return true;
    }
    return false;
}
/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Polyline<Point2D<T>>& polyline, const Segment2D<T>& seg) {
    return Intersect(seg, polyline);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Polyline<Point2D<T>>& polyline,
               Array<Point2D<T>>* intersections) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(intersections);

    intersections->clear();

    if (polyline.empty()) return false;
    if (polyline.size() == 1) {
        if (Intersect(seg, polyline.front())) {
            intersections->push_back(polyline.front());
            return true;
        }
        return false;
    }

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        Segment2D<T> s(polyline[i], polyline[i + 1]);
        Array<Point2D<T>> res;
        if (Intersect(s, seg, &res)) {
            intersections->insert(res);
        }
    }
    return !intersections->empty();
}
/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Polyline<Point2D<T>>& polyline, const Segment2D<T>& seg,
               Array<Point2D<T>>* intersections) {
    return Intersect(seg, polyline, intersections);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Polygon2D<T>& polygon,
               Array<Segment2D<T>>* res = nullptr) {
    MultiPolygon2D<T> poly(polygon);
    return Intersect(seg, poly, res);
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Segment2D<T>& seg,
               Array<Segment2D<T>>* res = nullptr) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Polygon2D<T>& polygon1, const Polygon2D<T>& polygon2) {
    if (!Intersect(polygon1.bounding_box(), polygon2.bounding_box()))
        return false;

    for (int i = 0; i < polygon1.size(); ++i) {
        if (Intersect(polygon1.vertices()[i], polygon2)) return true;
        if (Intersect(polygon1.edge(i), polygon2)) return true;
    }

    for (int i = 0; i < polygon2.size(); ++i) {
        if (Intersect(polygon2.vertices()[i], polygon1)) return true;
    }

    return false;
}

/**
 * Exact predication and inexact construction.
 *
 * It returns the intersecting polyline::segment pairs.
 */
template <typename T>
bool Intersect(const Polyline<Point2D<T>>& polyline1,
               const Polyline<Point2D<T>>& polyline2,
               Array<std::pair<int, int>>* res = nullptr) {
    if (res) res->clear();

    if (polyline1.size() <= 1 || polyline2.size() <= 1) return false;

    Array<Segment2D<T>> segs;
    segs.reserve(polyline1.size() - 1 + polyline2.size() - 1);
    for (int i = 0; i + 1 < polyline1.size(); ++i) {
        segs.emplace_back(polyline1[i], polyline1[i + 1]);
    }
    for (int i = 0; i + 1 < polyline2.size(); ++i) {
        segs.emplace_back(polyline2[i], polyline2[i + 1]);
    }

    int n = segs.size();

    // Get the sequence of segment by their lower endpoints' x coordinate.
    Array<T> values(n);
    for (int i = 0; i < n; ++i) {
        values[i] = segs[i].lower_point().x;
    }
    Array<int> seq;
    IndexSort(values.begin(), values.end(), &seq);

    // Get the intersection.
    Point2D<T> p;
    for (int i = 0; i < n; ++i) {
        T t = segs[seq[i]].upper_point().x;
        Segment2D<T>& s1 = segs[seq[i]];

        for (int j = i + 1; j < n && segs[seq[j]].lower_point().x <= t; ++j) {
            if ((seq[i] + 1 >= polyline1.size() &&
                 seq[j] + 1 >= polyline1.size()) ||
                (seq[i] + 1 < polyline1.size() &&
                 seq[j] + 1 < polyline1.size())) continue;
            Segment2D<T>& s2 = segs[seq[j]];

            if (Intersect(s1, s2)) {
                if (seq[i] + 1 >= polyline1.size()) {
                    if (res)
                        res->emplace_back(seq[j],
                                          seq[i] - polyline1.size() + 1);
                    else
                        return true;
                } else {
                    if (res)
                        res->emplace_back(seq[i],
                                          seq[j] - polyline1.size() + 1);
                    else
                        return true;
                }
            }
        }
    }

    return res ? (!res->empty()) : false;
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Polyline<Point2D<T>>& polyline,
               const MultiPolygon2D<T>& polygon) {
    if (polyline.empty()) return false;
    if (polyline.size() == 1) return Intersect(polyline.front(), polygon);

    Box2D<T> box(polyline.begin(), polyline.end());
    if (!Intersect(box, polygon.bounding_box())) return false;

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        Segment2D<T> s(polyline[i], polyline[i + 1]);
        if (Intersect(s, polygon)) return true;
    }
    return false;
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon,
               const Polyline<Point2D<T>>& polyline) {
    return Intersect(polyline, polygon);
}

////////////////////////////////////////////////////////////////////////////////
// All Cross() functions are used to test weather two objects are cross
// (excluding touch).
//
// Exact construction requires high-precision computation, which can greatly
// slow down the speed. In this library, we completely avoid such operations.
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Point2D<T>& point, const Box2D<T>& box) {
    return (point.x > box.x_min() && point.x < box.x_max() &&
            point.y > box.y_min() && point.y < box.y_max());
}
template <typename T>
bool Cross(const Box2D<T>& box, const Point2D<T>& point) {
    return Cross(point, box);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Segment2D<T>& seg) {
    if (line.empty()) return false;

    // Cross test.
    int o1 = Orientation(line.point1(), line.point2(), seg.lower_point());
    if (o1 == 0) return false;
    int o2 = Orientation(line.point1(), line.point2(), seg.upper_point());
    if (o2 == 0) return false;
    return o1 != o2;
}
template <typename T>
bool Cross(const Segment2D<T>& seg, const Line2D<T>& line) {
    return Cross(line, seg);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Segment2D<T>& seg, Point2D<T>* res) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(res);

    if (!Cross(line, seg)) return false;

    double x1 = line.point1().x, y1 = line.point1().y;
    double x2 = line.point2().x, y2 = line.point2().y;
    double x3 = seg.lower_point().x, y3 = seg.lower_point().y;
    double x4 = seg.upper_point().x, y4 = seg.upper_point().y;
    double t = ((x1 - x2) * (y1 - y3) - (x1 - x3) * (y1 - y2)) /
               ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
    if (!std::isfinite(t) || std::abs(t) < DBL_EPSILON) {
        t = 0.0;
    } else {
        t = Clamp(t, 0.0, 1.0);
    }

    if (t == 0.0) {
        *res = seg.lower_point();
    } else if (t == 1.0) {
        *res = seg.upper_point();
    } else {
        res->x = x3 + t * (x4 - x3);
        res->y = y3 + t * (y4 - y3);
    }
    return true;
}
template <typename T>
bool Cross(const Segment2D<T>& seg, const Line2D<T>& line, Point2D<T>* res) {
    return Cross(line, seg, res);
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Box2D<T>& box,
           Segment2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RangeRay2D<T> ray(line);
    ray.Intersect(box);

    if (ray.lower() > ray.upper()) return false;

    if (res) {
        *res = Segment2D<T>(ray.lower_point(), ray.upper_point());
    }
    return true;
}
template <typename T>
bool Cross(const Box2D<T>& box, const Line2D<T>& line,
           Segment2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    return Cross(line, box, res);
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Segment2D<T>& seg, const Box2D<T>& box,
           Segment2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RangeRay2D<T> ray(seg);
    ray.Intersect(box);

    if (ray.lower() > ray.upper()) return false;

    if (res) {
        *res = Segment2D<T>(ray.lower_point(), ray.upper_point());
    }
    return true;
}
template <typename T>
bool Cross(const Box2D<T>& box, const Segment2D<T>& seg,
           Segment2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    return Cross(seg, box, res);
}

/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether two lines cross is unnecessary in most cases.
 * Because it is very rare that two lines are strictly parallel. If there is a
 * real need, it is better to convert lines into long line segments.
 */
template <typename T>
bool Cross(const Line2D<T>& line1, const Line2D<T>& line2,
           Point2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    double x1 = line1.point1().x, y1 = line1.point1().y;
    double x2 = line1.point2().x, y2 = line1.point2().y;
    double x3 = line2.point1().x, y3 = line2.point1().y;
    double x4 = line2.point2().x, y4 = line2.point2().y;
    double t = ((x1 - x2) * (y1 - y3) - (x1 - x3) * (y1 - y2)) /
               ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
    if (!std::isfinite(t)) return false;

    T x = static_cast<T>(x3 + t * (x4 - x3));
    T y = static_cast<T>(y3 + t * (y4 - y3));
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }

    if (res) {
        res->x = x;
        res->y = y;
    }
    return true;
}

/**
 * Inexact predication and construction.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Circle2D<T>& circle,
           Segment2D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    if (line.empty()) return false;

    RangeRay2D<T> ray(line);
    if (!ray.Intersect(circle)) return false;

    if (res) {
        *res = Segment2D<T>(ray.lower_point(), ray.upper_point());
    }

    return true;
}
template <typename T>
bool Cross(const Circle2D<T>& circle, const Line2D<T>& line,
           Segment2D<T>* res = nullptr) {
    return Cross(line, circle, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Polyline<Point2D<T>>& polyline) {
    if (polyline.empty()) return false;

    if (polyline.size() == 1) {
        if (Intersect(line, polyline.front())) {
            return true;
        }
        return false;
    }

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        if (Intersect(line, Segment2D<T>(polyline[i], polyline[i + 1]))) {
            return true;
        }
    }

    return false;
}
template <typename T>
bool Cross(const Polyline<Point2D<T>>& polyline, const Line2D<T>& line) {
    return Cross(line, polyline);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Segment2D<T>& s1, const Segment2D<T>& s2) {
    if (!Intersect(s1.bounding_box(), s2.bounding_box())) return false;

    // Cross test.
    int o1 = Orientation(s1.lower_point(), s1.upper_point(), s2.lower_point());
    int o2 = Orientation(s1.lower_point(), s1.upper_point(), s2.upper_point());
    if (o1 == o2 || o1 == 0 || o2 == 0) {
        // Two endpoints of s2 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    int o3 = Orientation(s2.lower_point(), s2.upper_point(), s1.lower_point());
    int o4 = Orientation(s2.lower_point(), s2.upper_point(), s1.upper_point());
    if (o3 == o4 || o3 == 0 || o4 == 0) {
        // Two endpoints of s1 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    return true;
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Segment2D<T>& s1, const Segment2D<T>& s2, Point2D<T>* res) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(res);

    if (!Cross(s1, s2)) return false;

    // There must be an intersection between two segments.
    Line2D<T> line(s1.lower_point(), s1.upper_point());
    return Cross(line, s2, res);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Segment2D<T>& seg, const Polyline<Point2D<T>>& polyline) {
    if (polyline.size() <= 1) return false;

    for (int i = 0; i + 1 < polyline.size(); ++i) {
        Segment2D<T> s(polyline[i], polyline[i + 1]);
        if (Cross(seg, s)) return true;
    }
    return false;
}
/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Polyline<Point2D<T>>& polyline, const Segment2D<T>& seg) {
    return Cross(seg, polyline);
}

/**
 * Inexact predication and construction.
 *
 * Distance comparison is robust for predication. It is no need to use exact
 * predication.
 */
template <typename T>
bool Cross(const Circle2D<T>& circle1, const Circle2D<T>& circle2,
           Segment2D<T>* res = nullptr) {
    double dx = static_cast<double>(circle2.center().x) - circle1.center().x;
    double dy = static_cast<double>(circle2.center().y) - circle1.center().y;

    double dis = std::sqrt(dx * dx + dy * dy);
    if (dis == 0.0) return false;

    double r1 = circle1.radius();
    double r2 = circle2.radius();

    // No intersection.
    if (dis > r1 + r2) return false;

    // One circle is contained in the other.
    if (dis < std::fabs(r1 - r2)) return false;
    if (!res) return true;

    if (res) {
        CHECK(std::is_floating_point<T>::value);

        double a = (r1 * r1 - r2 * r2 + dis * dis) / (2.0 * dis);
        double x2 = dx * a / dis + circle1.center().x;
        double y2 = dy * a / dis + circle1.center().y;

        double dis2 = std::sqrt((r1 * r1) - (a * a));
        double rx = -dy * (dis2 / dis);
        double ry =  dx * (dis2 / dis);
        *res = Segment2D<T>(Point2D<T>(static_cast<T>(x2 + rx),
                                       static_cast<T>(y2 + ry)),
                            Point2D<T>(static_cast<T>(x2 - rx),
                                       static_cast<T>(y2 - ry)));
    }

    return true;
}

/**
 * Check all intersection between line segments.
 *
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Array<Segment2D<T>>& segs, Array<Point2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    int n = segs.size();

    // Get the sequence of segment by their lower endpoints' x coordinate.
    Array<T> values(n);
    for (int i = 0; i < n; ++i) {
        values[i] = segs[i].lower_point().x;
    }
    Array<int> seq;
    IndexSort(values.begin(), values.end(), &seq);

    // Get the intersection.
    Point2D<T> p;
    for (int i = 0; i < n; ++i) {
        T t = segs[seq[i]].upper_point().x;
        for (int j = i + 1; j < n && segs[seq[j]].lower_point().x <= t; ++j) {
            if (Cross(segs[seq[i]], segs[seq[j]], &p)) {
                if (!res)
                    return true;
                else
                    res->push_back(p);
            }
        }
    }

    return res ? (!res->empty()) : false;
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Point2D<T>& p, const Triangle2D<T>& triangle) {
    if (triangle.empty()) return false;

    if (!Cross(p, triangle.bounding_box())) return false;

    const Array<Point2D<T>>& vertices = triangle.vertices();
    int o1 = Orientation(vertices[0], vertices[1], p);
    if (o1 == 0) return false;

    int o2 = Orientation(vertices[1], vertices[2], p);
    if (o2 == 0) return false;

    // Now, p is not on the edge of triangle.
    if (o1 != o2) return false;

    int o3 = Orientation(vertices[2], vertices[0], p);
    return o1 == o3;
}
template <typename T>
bool Cross(const Triangle2D<T>& triangle, const Point2D<T>& p) {
    return Cross(p, triangle);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Point2D<T>& p, const Polygon2D<T>& polygon) {
    if (!Cross(p, polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (int j = 0; j < polygon.size(); ++j) {
        Segment2D<T> seg = polygon.edge(j);
        if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) {
            continue;
        }

        int o = Orientation(seg.lower_point(), seg.upper_point(), p);
        if (o == 0) return false;
        if (o < 0)  ++count;
    }

    return (count % 2 != 0);
}
template <typename T>
bool Cross(const Polygon2D<T>& polygon, const Point2D<T>& p) {
    return Cross(p, polygon);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Point2D<T>& p, const MultiPolygon2D<T>& multi_polygon) {
    if (!Cross(p, multi_polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (const auto& b : multi_polygon.boundaries()) {
        const Polygon2D<T>& polygon = b.polygon;
        if (!Cross(p, polygon.bounding_box())) continue;

        for (int j = 0; j < polygon.size(); ++j) {
            Segment2D<T> seg = polygon.edge(j);
            if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) {
                continue;
            }

            int o = Orientation(seg.lower_point(), seg.upper_point(), p);
            if (o == 0) return false;
            if (o < 0)  ++count;
        }
    }

    return (count % 2 != 0);
}
template <typename T>
bool Cross(const MultiPolygon2D<T>& multi_polygon, const Point2D<T>& p) {
    return Cross(p, multi_polygon);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_INTERSECT_2D_H_

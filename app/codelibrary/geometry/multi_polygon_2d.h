//
// Copyright 2012-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MULTI_POLYGON_2D_H_
#define CODELIBRARY_GEOMETRY_MULTI_POLYGON_2D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/polygon_2d.h"

namespace cl {

/**
 * Multi-polygon is a set of polygon with boundary flag (inner or outer).
 *
 * It must uphold the following conditions:
 *   1. Every boundaries must be a simple polygon (allow coincide edges).
 *   2. All boundaries must be disjoint from others (except on vertices).
 */
template <typename T>
class MultiPolygon2D {
    using Polygon = Polygon2D<T>;

public:
    using value_type = T;

    // The boundary for multi-polygon
    struct Boundary {
        Boundary() = default;

        Boundary(const Polygon& poly, bool outer)
            : polygon(poly), is_outer(outer) {}

        Polygon polygon;      // The polygon of the boundary.
        bool is_outer = true; // True if this boundary is outer boundary.
    };
    using Iterator = typename Array<Boundary>::const_iterator;

    MultiPolygon2D() = default;

    /**
     * Construct multi-polygon with given polygon as its outer boundary.
     */
    explicit MultiPolygon2D(const Polygon& polygon) {
        if (!polygon.empty()) Insert(polygon, true);
    }

    /**
     * Construct multi-polygon with given box as its outer boundary.
     */
    explicit MultiPolygon2D(const Box2D<T>& box) {
        if (!box.empty()) Insert(Polygon2D<T>(box), true);
    }

    /**
     * Check if this polygon is a polygon-with-holes.
     *
     * A "polygon with holes" is defined as a polygon structure that contains:
     * - Exactly one valid outer ring (exterior boundary), and
     * - Several inner rings (interior boundary, i.e., a hole).
     */
    bool IsPolygonWithHoles() const {
        if (this->empty()) return true;

        if (!boundaries_[0].is_outer) return false;

        for (int i = 1; i < boundaries_.size(); ++i) {
            if (boundaries_[i].is_outer) return false;
        }
        return true;
    }

    /**
     * Insert a new boundary.
     */
    void Insert(const Polygon& polygon, bool is_outer) {
        if (polygon.empty()) return;

        boundaries_.emplace_back(polygon, is_outer);
        bounding_box_.Union(polygon.bounding_box());
    }

    /**
     * Insert new boundaries from another multi-polygon.
     */
    void Insert(const MultiPolygon2D& polygon) {
        for (const auto& b : polygon.boundaries()) {
            this->Insert(b.polygon, b.is_outer);
        }
    }

    /**
     * Return the area of the polygon.
     */
    double Area() const {
        double area = 0.0;
        for (const auto& b : boundaries_) {
            if (b.is_outer) {
                area += b.polygon.Area();
            } else {
                area -= b.polygon.Area();
            }
        }

        return area;
    }

    /**
     * Scale the polygon.
     */
    void Scale(const T& v) {
        CHECK(v != 0);

        if (empty()) return;

        for (auto& b : boundaries_) {
            b.polygon.Scale(v);
        }
        bounding_box_ = Box2D<T>(std::min(bounding_box_.x_min() * v,
                                          bounding_box_.x_max() * v),
                                 std::max(bounding_box_.x_min() * v,
                                          bounding_box_.x_max() * v),
                                 std::min(bounding_box_.y_min() * v,
                                          bounding_box_.y_max() * v),
                                 std::max(bounding_box_.y_min() * v,
                                          bounding_box_.y_max() * v));
    }

    /**
     * Scale the polygon.
     */
    void Scale(const T& scale_x, const T& scale_y) {
        CHECK(scale_x != 0);
        CHECK(scale_y != 0);

        if (empty()) return;

        for (auto& b : boundaries_) {
            b.polygon.Scale(scale_x, scale_y);
        }
        bounding_box_ = Box2D<T>(std::min(bounding_box_.x_min() * scale_x,
                                          bounding_box_.x_max() * scale_x),
                                 std::max(bounding_box_.x_min() * scale_x,
                                          bounding_box_.x_max() * scale_x),
                                 std::min(bounding_box_.y_min() * scale_y,
                                          bounding_box_.y_max() * scale_y),
                                 std::max(bounding_box_.y_min() * scale_y,
                                          bounding_box_.y_max() * scale_y));
    }

    /**
     * Translate this polygon.
     */
    void Translate(const Vector2D<T>& v) {
        CHECK(!empty());

        for (auto& b : boundaries_) {
            b.polygon.Translate(v);
        }
        bounding_box_ = Box2D<T>(bounding_box_.x_min() + v.x,
                                 bounding_box_.x_max() + v.x,
                                 bounding_box_.y_min() + v.y,
                                 bounding_box_.y_max() + v.y);
    }

    /**
     * Clear the polygon.
     */
    void clear() {
        bounding_box_ = Box2D<T>();
        boundaries_.clear();
    }

    bool empty() const {
        return boundaries_.empty();
    }

    const Array<Boundary>& boundaries() const {
        return boundaries_;
    }

    /**
     * Return the bounding box of multi-polygon.
     */
    const Box2D<T>& bounding_box() const {
        return bounding_box_;
    }

    /**
     * Erase all coincident edges and empty boundaries.
     */
    void Trim() {
        bounding_box_ = Box2D<T>();
        int n_boundaries = 0;
        for (auto& b : boundaries_) {
            b.polygon.Trim();
            if (!b.polygon.empty()) {
                boundaries_[n_boundaries].polygon = b.polygon;
                boundaries_[n_boundaries].is_outer = b.is_outer;
                bounding_box_.Union(b.polygon.bounding_box());
                ++n_boundaries;
            }
        }
        boundaries_.resize(n_boundaries);
    }

    /**
     * Return the number of boundaries.
     */
    int n_boundaries() const {
        return boundaries_.size();
    }

    Iterator begin() const {
        return boundaries_.begin();
    }

    Iterator end() const {
        return boundaries_.end();
    }

protected:
    Box2D<T> bounding_box_;      // The bounding box of multi-polygon.
    Array<Boundary> boundaries_; // The boundaries of multi-polygon.
};

using IMultiPolygon2D = MultiPolygon2D<int>;
using FMultiPolygon2D = MultiPolygon2D<float>;
using RMultiPolygon2D = MultiPolygon2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MULTI_POLYGON_2D_H_

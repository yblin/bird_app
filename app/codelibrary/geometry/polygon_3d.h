//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POLYGON_3D_H_
#define CODELIBRARY_GEOMETRY_POLYGON_3D_H_

#include <algorithm>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/segment_3d.h"

namespace cl {

/**
 * Simple planar polygon. A 3D version of Polygon2D. Note that all vertices are
 * on the same plane.
 */
template <typename T>
class Polygon3D {
    static_assert(std::is_floating_point<T>::value);

public:
    using Point = Point3D<T>;
    using value_type = T;
    using Iterator = typename Array<Point>::const_iterator;

    Polygon3D() = default;

    /**
     * Construct a 3D polygon based on a 3D point set and a plane.
     * The user needs to ensure that the input points is on the given plane.
     */
    Polygon3D(const Array<Point>& vertices, const Plane3D<T>& plane)
        : Polygon3D(vertices.begin(), vertices.end(), plane) {}

    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Polygon3D(Iter first, Iter last, const Plane3D<T>& plane)
        : vertices_(first, last), plane_(plane) {
        Initialize();
    }

    /**
     * Erase all coincident edges.
     */
    void Trim() {
        if (size_ <= 2) {
            this->clear();
            return;
        }

        Array<int> prev(size_), next(size_);
        prev[0] = size_ - 1;
        next[size_ - 1] = 0;
        for (int i = 1; i < size_; ++i) {
            prev[i] = i - 1;
        }
        for (int i = 0; i + 1 < size_; ++i) {
            next[i] = i + 1;
        }

        Array<bool> removed(size_, false);
        for (int v = 0; v < size_; ++v) {
            if (removed[v]) continue;
            while (vertices_[prev[v]] == vertices_[next[v]]) {
                removed[v] = true;
                removed[prev[v]] = true;
                next[prev[prev[v]]] = next[v];
                prev[next[v]] = prev[prev[v]];
                v = next[v];
                if (removed[v]) break;
            }
        }

        size_ = 0;
        for (int i = 0; i < vertices_.size(); ++i) {
            if (!removed[i]) {
                vertices_[size_++] = vertices_[i];
            }
        }
        vertices_.resize(size_);

        if (size_ < 3) {
            this->clear();
        } else {
            Initialize();
        }
    }

    /**
     * Reverse the polygon vertices.
     */
    void Reverse() {
        std::reverse(vertices_.begin(), vertices_.end());
    }

    /**
     * Return true if the polygon does not have any vertices.
     */
    bool empty() const {
        return size_ == 0;
    }

    /**
     * Clear the polygon.
     */
    void clear() {
        bounding_box_ = Box3D<T>();
        vertices_.clear();
        size_ = 0;
        plane_ = Plane3D<T>();
    }

    /**
     * Return the size of vertices.
     */
    int size() const {
        return size_;
    }

    /**
     * Get the i-th edge of polygon.
     */
    Segment3D<T> edge(int i) const {
        CHECK(i >= 0 && i < size_);

        return (i + 1 == size_) ? Segment3D<T>(vertices_[i], vertices_[0])
                                : Segment3D<T>(vertices_[i], vertices_[i + 1]);
    }

    /**
     * Get the i-th vertex of polygon.
     */
    const Point& vertex(int i) const {
        CHECK(i >= 0 && i < size_);

        return vertices_[i];
    }

    /**
     * Get the next vertex of the i-th vertex.
     */
    const Point& next_vertex(int i) const {
        CHECK(i >= 0 && i < size_);

        return i + 1 == size_ ? vertices_[0] : vertices_[i + 1];
    }

    /**
     * Get the previous vertex of the i-th vertex.
     */
    const Point& prev_vertex(int i) const {
        CHECK(i >= 0 && i < size_);

        return i - 1 < 0 ? vertices_[size_ - 1] : vertices_[i - 1];
    }

    /**
     * Get the previous index of the i-th vertex.
     */
    int prev_index(int i) const {
        CHECK(i >= 0 && i < size_);

        return i - 1 < 0 ? size_ - 1 : i - 1;
    }

    /**
     * Get the next index of the i-th vertex.
     */
    int next_index(int i) const {
        CHECK(i >= 0 && i < size_);

        return i + 1 == size_ ? 0 : i + 1;
    }

    /**
     * Return the vertices of polygon.
     */
    const Array<Point>& vertices() const {
        return vertices_;
    }

    /**
     * Return the iterator to the first vertex.
     */
    Iterator begin() const {
        return vertices_.begin();
    }

    /**
     * Return the Iterator to the last vertex + 1.
     */
    Iterator end() const {
        return vertices_.end();
    }

    const Point& operator[] (int i) const {
        return vertices_[i];
    }

    /**
     * Return the plane where the vertices on.
     */
    const Plane3D<T>& plane() const {
        return plane_;
    }

    /**
     * Return the bounding box of this polygon.
     */
    const Box3D<T>& bounding_box() const {
        return bounding_box_;
    }

protected:
    /**
     * Initialize the polygon.
     */
    void Initialize() {
        if (vertices_.empty()) return;

        // Erase the duplicate points.
        int n = static_cast<int>(std::unique(vertices_.begin(),
                                             vertices_.end()) -
                                 vertices_.begin());
        vertices_.resize(n);
        if (vertices_.back() == vertices_.front()) {
            vertices_.pop_back();
        }
        size_ = vertices_.size();

        if (size_ < 3) {
            this->clear();
            return;
        }

        bounding_box_ = Box3D<T>(vertices_.begin(), vertices_.end());
    }

    // Number of vertices.
    int size_ = 0;

    // Vertices of polygon.
    Array<Point> vertices_;

    // All vertices are on the same plane.
    Plane3D<T> plane_;

    // Bounding box of polygon.
    Box3D<T> bounding_box_;
};

using FPolygon3D = Polygon3D<float>;
using RPolygon3D = Polygon3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POLYGON_3D_H_

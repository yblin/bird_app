//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MULTI_POLYGON_3D_H_
#define CODELIBRARY_GEOMETRY_MULTI_POLYGON_3D_H_

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/polygon_3d.h"
#include "codelibrary/geometry/util/pad_3d.h"

namespace cl {

/**
 * 3D version of MultiPolygon.
 */
template <typename T>
class MultiPolygon3D {
    using Polygon = Polygon3D<T>;

public:
    using value_type = T;

    MultiPolygon3D() = default;

    explicit MultiPolygon3D(const Plane3D<T>& plane)
        : pad_(plane) {}

    /**
     * Construct multi-polygon with given polygon as its outer boundary.
     */
    explicit MultiPolygon3D(const Polygon& polygon)
        : pad_(polygon.plane_) {
        Insert(polygon.vertices(), true);
    }

    /**
     * Insert a new boundary.
     */
    void Insert(const Array<Point3D<T>>& boundary, bool is_outer) {
        if (boundary.empty()) {
            LOG(WARNING) << "The input boundary is empty.";
            return;
        }

        Array<Point2D<T>> boundary_2d;
        for (const auto& p : boundary) {
            Point2D<T> q = pad_.Project(p);
            boundary_2d.push_back(q);
        }
        bounding_box_.Union(Box3D<T>(boundary.begin(), boundary.end()));
        vertices_.push_back(boundary);
        projected_polygon_.Insert(Polygon2D<T>(boundary_2d), is_outer);
    }

    /**
     * Clear the polygon.
     */
    void clear() {
        bounding_box_ = Box3D<T>();
        projected_polygon_.clear();
        vertices_.clear();
    }

    bool empty() const {
        return projected_polygon_.empty();
    }

    const MultiPolygon2D<T>& projected_polygon() const {
        return projected_polygon_;
    }

    const Array<Array<Point3D<T>>>& vertices() const {
        return vertices_;
    }

    const Plane3D<T>& plane() const {
        return pad_.plane();
    }

    const geometry::Pad3D<T>& pad() const {
        return pad_;
    }

    /**
     * Return the bounding box of multi-polygon.
     */
    const Box3D<T>& bounding_box() const {
        return bounding_box_;
    }

protected:
    // Pad for convert 3D polygon to 2D polygon.
    geometry::Pad3D<T> pad_;

    // The corresponding 2D polygon.
    MultiPolygon2D<T> projected_polygon_;

    // Raw vertices.
    Array<Array<Point3D<T>>> vertices_;

    // The bounding box of multi-polygon.
    Box3D<T> bounding_box_;
};

using FMultiPolygon3D = MultiPolygon3D<float>;
using RMultiPolygon3D = MultiPolygon3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MULTI_POLYGON_3D_H_

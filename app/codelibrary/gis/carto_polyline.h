//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_CARTO_POLYLINE_H_
#define CODELIBRARY_GIS_CARTO_POLYLINE_H_

#include <string>

#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace gis {

/**
 * Structure representing a cartographic 2D or 3D polyline for map-based
 * applications.
 *
 * A CartoPolyline is a core geometric element for 2D map systems (e.g.,
 * road/lane networks), characterized by a polyline with width variations at
 * each vertex. It is the fundamental "raw" geometric unit before being
 * segmented into uniform-width CartoPolylineKernel.
 *
 * For 3D scenarios, the Z-coordinate (if present) represents elevation/height
 * only—all width-related logic remains anchored to the 2D XY plane of the map.
 *
 * Key features:
 * - Primary: 2D planar geometry (map-specific, core use case)
 * - Secondary: 3D extension (Z-coordinate = elevation/height, no impact on
 *              width logic)
 * - Each vertex of the centerline has an independent width value (strictly 2D
 *   attribute)
 * - Retains semantic attributes (level/category) for map layer/classification
 */
template <typename Point>
struct CartoPolyline {
    // Vertical level (e.g., map layer/z-order for overpass/underpass in
    // road/lane systems).
    int level = 0;

    // Categorical type of the cartographic polyline (e.g., "road",
    // "left_turn_lane", "driveway").
    std::string category;

    // 2D or 3D polyline forming the geometric core of CartoPolyline.
    Polyline<Point> polyline;

    // Width values corresponding to each vertex in the polyline.
    Array<double> widths;

    CartoPolyline() = default;

    bool empty() const {
        return polyline.empty();
    }

    void clear() {
        level = 0;
        category = "";
        polyline.clear();
        widths.clear();
    }

    int size() const {
        return polyline.size();
    }
};

/**
 * Structure representing a foundational 2D or 3D cartographic polyline kernel.
 *
 * CartoPolylineKernel is a minimal, self-contained data structure serving as
 * the BASIC block for all algorithms involving 2D width-bearing polylines
 * (e.g., road/lane networks, cartographic linear features).
 *
 * For 3D scenarios, the Z-coordinate (if present) represents elevation/height
 * only—no change to the uniform width logic, which remains tied to the 2D XY
 * plane.
 *
 * It is defined by a single polyline and a uniform width value applied to the
 * entire polyline.
 */
template <typename Point>
struct CartoPolylineKernel {
    // Vertical level (e.g., map layer/z-order for overpass/underpass in
    // road/lane systems).
    int level = 0;

    // Categorical type of the cartographic polyline (e.g., "road",
    // "left_turn_lane", "driveway").
    std::string category;

    // 2D or 3D polyline forming the geometric core of CartoPolylineKernel.
    Polyline<Point> polyline;

    // Uniform width of CartoPolylineKernel.
    double width = 0.0;

    CartoPolylineKernel() = default;

    bool empty() const {
        return polyline.empty();
    }

    void clear() {
        level = 0;
        category = "";
        width = 0.0;
        polyline.clear();
    }
};

using CartoPolyline2D = CartoPolyline<RPoint2D>;
using CartoPolylineKernel2D = CartoPolylineKernel<RPoint2D>;
using CartoPolyline3D = CartoPolyline<RPoint3D>;
using CartoPolylineKernel3D = CartoPolylineKernel<RPoint3D>;

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_CARTO_POLYLINE_H_

//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_H_

#include <string>

#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace gis {

/**
 * Structure representing a road branch, defined based on a centerline polyline.
 *
 * A road branch is a segment of a road network, characterized by its
 * centerline, width variations along the centerline, and a categorical title.
 */
struct RoadBranch {
    // Category of the road branch, indicating its type.
    std::string category;

    // 2D polyline representing the centerline of the road branch.
    // This is the central line derived from the road's polygon shape, serving
    // as the primary geometric representation of the branch.
    RPolyline2D polyline;

    // Width values corresponding to each vertex in the polyline.
    Array<double> widths;

    RoadBranch() = default;

    bool empty() const {
        return polyline.empty();
    }
};

/**
 * Structure representing a segment of a road branch.
 *
 * A road branch can be decomposed into multiple segments, where each segment is
 * a sub-section of the branch with a uniform width across its entire length.
 */
struct RoadBranchSegment {
    // Category of this road segment, indicating its type.
    std::string category;

    // 2D polyline representing the centerline of this road segment.
    // This is a sub-section of the parent road branch's centerline, forming the
    // geometric basis of the segment.
    RPolyline2D polyline;

    // Uniform width of the entire road segment.
    // Unlike the parent road branch (where each vertex has its own width), this
    // single value represents the width of the road across all vertices of this
    // segment.
    double width;

    RoadBranchSegment() = default;

    explicit RoadBranchSegment(const RPolyline2D& p, double w = 0.0,
                               const std::string& title = "")
        : category(title), polyline(p), width(w) {}
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_H_

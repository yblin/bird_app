//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_H_

#include <string>

#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace gis {

/**
 * A 2D linear feature representing lane markings.
 */
struct LaneMarking {
    // Vertical level, matching the level of LandRegion to associate the marking
    // with its target region (critical for overlapping regions).
    int level = 0;
    // Type of road marking.
    std::string category;
    // Linear geometry representing the lane marking's shape.
    RPolyline2D polyline;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_MARKING_H_

//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_LANE_UTIL_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_LANE_UTIL_H_

#include <string>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace gis {
namespace lane_util {

/**
 * Check if the given lane marking title is a yellow line or a white line.
 *
 * In the United States, Canada, Mexico, Honduras, Puerto Rico, Virgin
 * Islands and Norway, yellow lines separate traffic going in opposite
 * directions and white separates lanes of traffic traveling in the same
 * direction; but that is not the case in many European countries.
 */
inline bool IsYellowLine(const std::string& title) {
    return title.find("yellow") != std::string::npos;
}
inline bool IsWhiteLine(const std::string& title) {
    return title.find("white") != std::string::npos;
}

} // namespace lane_util
} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_LANE_UTIL_H_

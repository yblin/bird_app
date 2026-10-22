//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LANE_SYSTEM_LANE_UTIL_H_
#define CODELIBRARY_GIS_LANE_SYSTEM_LANE_UTIL_H_

#include <string>

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

/**
 * Check if the given lane marking title is a solid dash line.
 */
inline bool IsSolidDashLine(const std::string& title) {
    return title == "solid_dash_white" || title == "solid_dash_yellow";
}

/**
 * Check if the given lane marking title is left turn.
 */
inline bool IsLeftTurn(const std::string& title) {
    return title == "left_turn";
}

/**
 * Check if the given lane marking title is right turn.
 */
inline bool IsRightTurn(const std::string& title) {
    return title == "right_turn";
}

} // namespace lane_util
} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LANE_SYSTEM_LANE_UTIL_H_

//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_UTIL_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_UTIL_H_

#include <string>

#include "codelibrary/gis/carto_polyline_util.h"

namespace cl {
namespace gis {
namespace road_util {

/**
 * Check if the given road category title is a road segment.
 */
inline bool IsRoadSegment(const std::string& title) {
    return title == "road" || title == "road_segment";
}

/**
 * Check if the given road category title is a road intersection.
 */
inline bool IsRoadIntersection(const std::string& title) {
    return title == "intersection_signal_controlled" ||
           title == "intersection_stop_controlled" ||
           title == "intersection_uncontrolled" ||
           title == "intersection_virtual" ||
           title == "virtual_intersection" ||
           title == "intersection_roundabout" ||
           title == "road-intersection" ||
           title == "road_intersection";
}

/**
 * Check if the given road category is a road overlap.
 */
inline bool IsRoadOverlap(const std::string& title) {
    return title == "road_overlap";
}

/**
 * Check if the given road category title is a road median.
 *
 * A traffic median is the reserved area that separates opposing lanes of
 * traffic on divided roadways.
 */
inline bool IsMedian(const std::string& title) {
    return title.find("median") != std::string::npos;
}

/**
 * Check if the given road category title is a gore.
 *
 * A gore is a triangular plot of land, not to be driven on, where a road forks
 * at the intersection with a second road, or merges on and off from a larger
 * one.
 */
inline bool IsGore(const std::string& title) {
    return title == "gore_area";
}

/**
 * Check if the given road category title is a crosswalk.
 *
 * Crosswalk is a place designated for pedestrians to cross a road.
 */
inline bool IsCrosswalk(const std::string& title) {
    return title.find("crosswalk") != std::string::npos ||
           title.find("cross_walk") != std::string::npos;
}

/**
 * Check if the given road category title is parking related (e.g.,
 * inside_parking, no_parking_area).
 */
inline bool IsParkingRelated(const std::string& title) {
    return title.find("parking") != std::string::npos;
}

/**
 * A driveway is a private road for local access to one or a small group of
 * structures owned and maintained by an individual or group.
 */
inline bool IsDriveway(const std::string& title) {
    return title.find("driveway") != std::string::npos;
}

} // namespace road_util
} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_UTIL_H_

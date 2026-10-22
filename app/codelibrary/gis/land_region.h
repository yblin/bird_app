//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LAND_REGION_H_
#define CODELIBRARY_GIS_LAND_REGION_H_

#include <map>
#include <set>
#include <string>

#include "codelibrary/geometry/multi_polygon_2d.h"

namespace cl {
namespace gis {

/**
 * A generic 2D closed region representing land cover features, designed to
 * encapsulate all enclosed shapes in geographic data (e.g., roads, lanes,
 * buildings, water bodies).
 *
 * It explicitly supports vertical levels to distinguish overlapping regions in
 * overhead views (e.g., overpasses), making it particularly suitable for
 * processing ortho-satellite imagery or aerial photographs where vertical
 * hierarchy is critical for disambiguating overlapping features.
 */
struct LandRegion {
    int id;                  // Unique ID.
    int level = 0;           // Vertical level.
    std::string category;    // Region type.
    RMultiPolygon2D polygon; // Polygon of the region.
};

/**
 * A single 2D closed region formed by merging fully overlapping LandRegions,
 * where the resulting polygon exactly covers the overlapping area, and
 * aggregates all levels and categories from the original overlapping regions.
 */
struct LandRegionOverlay {
    // Unique ID.
    int id = -1;
    // Aggregated categories into vertical levels.
    std::map<int, std::set<std::string>> level_categories;
    // Polygon representing the region.
    RMultiPolygon2D polygon;

    /**
     * Get category info for debug.
     */
    std::string GetCategoryInfo() const {
        std::string info;
        for (auto i : level_categories) {
            for (auto category : i.second) {
                info += category + '#';
            }
        }
        if (!info.empty()) return info.substr(0, info.size() - 1);
        return "";
    }

    /**
     * Get level info for debug.
     */
    std::string GetLevelInfo() const {
        std::string info;
        for (auto i : level_categories) {
            info += ToString(i.first) + '#';
        }
        if (!info.empty()) return info.substr(0, info.size() - 1);
        return "";
    }
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LAND_REGION_H_

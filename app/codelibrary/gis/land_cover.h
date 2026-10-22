//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_LAND_COVER_H_
#define CODELIBRARY_GIS_LAND_COVER_H_

#include <map>
#include <string>

#include "codelibrary/geometry/boolean/polygon_boolean_2d.h"
#include "codelibrary/geometry/boolean/polygon_union_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/util/snap_2d.h"

namespace cl {
namespace gis {

/**
 * In GIS context, a land cover represents the physical surface cover of an
 * area, consisting of multiple geographic polygons where each polygon is
 * associated with a specific category (e.g., forest, water, road, building).
 */
class LandCover {
public:
    using Iterator = std::map<std::string, Array<RMultiPolygon2D>>::iterator;
    using ConstIterator = std::map<std::string,
                                   Array<RMultiPolygon2D>>::const_iterator;

    LandCover() = default;

    LandCover(const std::map<std::string, Array<RMultiPolygon2D>> geometries)
        : geometries_(geometries) {}

    /**
     * Add a geometry into land cover.
     */
    void AddGeometry(const std::string& category,
                     const RMultiPolygon2D& geometry) {
        geometries_[category].push_back(geometry);
    }

    /**
     * Add geometries into land cover.
     */
    void AddGeometries(const std::string& category,
                       const Array<RMultiPolygon2D>& geometries) {
        geometries_[category].insert(geometries);
    }

    /**
     * Merge the adjacent polygons within the same category.
     */
    void MergeGeometries(double threshold) {
        CHECK(threshold > 0.0);

        geometry::PolygonUnion2D<double> polygon_union;
        Array<RMultiPolygon2D> merged;
        for (auto& p : geometries_) {
            polygon_union(p.second, threshold, &merged);
            p.second = merged;
        }
    }

    /**
     * Get the geometries for the speicific categories.
     */
    void GetGeometries(const Array<std::string>& categories,
                       Array<RMultiPolygon2D>* geometries) const {
        CHECK(geometries);

        geometries->clear();
        if (categories.empty()) return;
        for (const auto& p : geometries_) {
            if (categories.find(p.first) != categories.end()) {
                geometries->insert(p.second);
            }
        }
    }

    /**
     * Get the merged polygon for the given categories.
     */
    void GetMergedGeometries(const Array<std::string>& categories,
                             double threshold,
                             Array<RMultiPolygon2D>* merged) {
        CHECK(threshold > 0.0);
        CHECK(merged);

        merged->clear();
        if (categories.empty()) return;

        Array<RMultiPolygon2D> polygons;
        GetGeometries(categories, &polygons);

        geometry::PolygonUnion2D<double> polygon_union;
        polygon_union(polygons, threshold, merged);
    }

    /**
     * Convert land cover geometries into polygon-category pairs.
     */
    void ToPolygonCategoryPairs(Array<RMultiPolygon2D>* polygons,
                                Array<std::string>* categories) const {
        CHECK(polygons);
        CHECK(categories);

        polygons->clear();
        categories->clear();

        for (const auto& r : geometries_) {
            for (const auto& g : r.second) {
                polygons->push_back(g);
                categories->push_back(r.first);
            }
        }
    }

    /**
     * Get a union multi-polygon for the specific land cover category.
     * Assume that all polygons for the sepecific category are disjoint.
     */
    void GetUnionPolygon(const std::string& category,
                         RMultiPolygon2D* polygon) const {
        CHECK(polygon);

        polygon->clear();
        auto it = geometries_.find(category);
        if (it == geometries_.end()) {
            return;
        }

        for (auto& p : it->second) {
            polygon->Insert(p);
        }
    }

    /**
     * Get all multi-polygons for a specific land cover category.
     */
    const Array<RMultiPolygon2D>&
    get_geometries(const std::string& category) const {
        auto it = geometries_.find(category);
        if (it != geometries_.end()) {
            return it->second;
        }

        static const Array<RMultiPolygon2D> empty;
        return empty;
    }

    /**
     * Clip all polygons to a specified polygon.
     *
     * This function modifies the land cover polygons in-place: it retains only
     * the intersection between each polygon and the input aoi polygon.
     */
    void ClipAll(const RMultiPolygon2D& aoi, double threshold) {
        CHECK(threshold >= 0.0);

        Array<std::string> empty_categories;
        for (auto& p : geometries_) {
            Array<RMultiPolygon2D> polygons;
            for (auto& polygon : p.second) {
                geometry::PolygonBoolean2D<double> boolean(polygon, aoi,
                                                           threshold);
                RMultiPolygon2D inter;
                boolean.Intersection(&inter);
                if (!inter.empty()) {
                    polygons.push_back(inter);
                }
            }
            if (polygons.empty()) {
                empty_categories.push_back(p.first);
            }
            p.second = polygons;
        }

        for (auto c : empty_categories) {
            geometries_.erase(c);
        }
    }

    void clear() {
        geometries_.clear();
    }

    bool empty() const {
        return geometries_.empty();
    }

    const std::map<std::string, Array<RMultiPolygon2D>>& geometries() const {
        return geometries_;
    }

    Iterator begin()            { return geometries_.begin(); }
    Iterator end()              { return geometries_.end();   }
    ConstIterator begin() const { return geometries_.begin(); }
    ConstIterator end()   const { return geometries_.end();   }

protected:
    // Map land cover categories to their associated multi-polygon geometries.
    std::map<std::string, Array<RMultiPolygon2D>> geometries_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_LAND_COVER_H_

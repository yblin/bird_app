//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARAY_GEOMETRY_UTIL_WKT_UTIL_H_
#define CODELIBRARAY_GEOMETRY_UTIL_WKT_UTIL_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/log.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"

namespace cl {
namespace geometry {
namespace wkt_util {

/**
 * Parse WKT to a set of string according to the bracket pairs.
 */
inline bool ParseWKTSets(const std::string& wkt, Array<std::string>* sets) {
    CHECK(sets);

    sets->clear();
    if (wkt.empty()) {
        return true;
    }

    int nesting_level = 0;
    size_t start = std::string::npos;

    for (size_t i = 0; i < wkt.length(); ++i) {
        if (wkt[i] == '(') {
            if (nesting_level == 1) {
                start = i;
            }
            ++nesting_level;
        } else if (wkt[i] == ')') {
            --nesting_level;
            if (nesting_level < 0) {
                LOG(ERROR) << "Invalid WKT format: bracket mismatch.";
                return false;
            }

            if (nesting_level == 1 && start != std::string::npos) {
                sets->push_back(wkt.substr(start, i - start + 1));
                start = std::string::npos;
            }
        }
    }
    if (nesting_level != 0) {
        LOG(ERROR) << "Invalid WKT format: bracket mismatch.";
        return false;
    }

    return true;
}

/**
 * Parse a WKT set (enclosed in bracket) to a set of 3D points.
 *
 * For internal use.
 */
template <typename T>
bool ParseWKTSet(const std::string& wkt_set, Array<Point3D<T>>* points) {
    CHECK(points);

    points->clear();

    if (wkt_set.empty()) {
        return true;
    }

    if (wkt_set.front() != '(' && wkt_set.back() != ')') {
        LOG(ERROR) << "Invalid input WKT, no bracket found in '" << wkt_set
                   << "'.";
        return false;
    }

    std::string str = wkt_set.substr(1, wkt_set.size() - 2);

    double x, y, z;
    Array<std::string> res;
    StringSplit(str, ',', &res);
    for (const auto& s : res) {
        std::istringstream iss(s);
        if (!(iss >> x >> y >> z)) {
            LOG(ERROR) << "Invalid WKT format found in: " << wkt_set;
            return false;
        }
        points->emplace_back(x, y, z);
    }

    return true;
}

/**
 * Read a simple 3D polygon from WKT.
 */
template <typename T>
bool PolygonFromWKT(const std::string& wkt, Array<Array<Point3D<T>>>* polygon) {
    CHECK(wkt.substr(0, 7) == "POLYGON");
    CHECK(polygon);

    Array<std::string> set;
    polygon->clear();
    if (!ParseWKTSets(wkt, &set)) return false;

    Array<Point3D<T>> points;
    for (const auto& s : set) {
        if (!ParseWKTSet(s, &points)) return false;
        polygon->push_back(points);
    }

    return true;
}

} // namespace wkt_util
} // namespace geometry
} // namespace cl

#endif // CODELIBRARAY_GEOMETRY_UTIL_WKT_UTIL_H_

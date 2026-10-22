//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_UTIL_WKT_UTIL_H_
#define CODELIBRARY_GIS_UTIL_WKT_UTIL_H_

#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace gis {
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
 * Parse a WKT set (enclosed in bracket) to a set of 2D points.
 *
 * For internal use.
 */
template <typename T>
bool ParseWKTSet(const std::string& wkt_set, Array<Point2D<T>>* points) {
    CHECK(points);

    points->clear();

    if (wkt_set.empty()) return true;

    if (wkt_set.front() != '(' && wkt_set.back() != ')') {
        LOG(ERROR) << "Invalid WKT, no bracket found in '" << wkt_set << "'.";
        return false;
    }

    std::string str = wkt_set.substr(1, wkt_set.size() - 2);
    TextStream stream(str);
    stream.set_delimiter(',');

    T x, y;
    while (stream >> x >> y) {
        points->emplace_back(x, y);
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

    if (wkt_set.empty()) return true;

    if (wkt_set.front() != '(' && wkt_set.back() != ')') {
        LOG(ERROR) << "Invalid WKT, no bracket found in '" << wkt_set << "'.";
        return false;
    }

    std::string str = wkt_set.substr(1, wkt_set.size() - 2);
    TextStream stream(str);
    stream.set_delimiter(',');

    T x, y, z;
    while (stream >> x >> y >> z) {
        points->emplace_back(x, y, z);
    }

    return true;
}

/**
 * Read a simple 2D polygon from WKT.
 */
template <typename T>
bool PolygonFromWKT(const std::string& wkt, MultiPolygon2D<T>* polygon) {
    CHECK(polygon);

    std::string head = wkt.substr(0, 7);
    std::transform(head.begin(), head.end(), head.begin(), [](unsigned char c){
        return std::tolower(c);
    });
    CHECK(head == "polygon");

    Array<std::string> set;
    polygon->clear();
    if (!ParseWKTSets(wkt, &set)) return false;

    Array<Point2D<T>> points;
    for (const auto& s : set) {
        if (!ParseWKTSet(s, &points)) return false;
        Polygon2D<T> poly(points);
        if (!poly.empty()) {
            if (polygon->empty()) {
                polygon->Insert(poly, true);
            } else {
                polygon->Insert(poly, false);
            }
        }
    }

    return true;
}

} // namespace wkt_util
} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_UTIL_WKT_UTIL_H_

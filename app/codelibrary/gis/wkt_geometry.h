//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_WKT_GEOMETRY_H_
#define CODELIBRARY_GIS_WKT_GEOMETRY_H_

#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/string/string_trim.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace gis {

/**
 * Well-known text (WKT) is a text markup language for representing vector
 * geometry objects.
 *
 * This class loads CL-supported geometric objects from WKT strings, with the
 * coordinate numeric type defined by template parameter T (e.g., float/double).
 *
 * Currently supported WKT types:
 * 1. 2D Polygon
 * 2. 2D MultiPolygon
 * 3. 2D LineString / MultiLineString (stored as open polylines)
 * 4. 2D Point / MultiPoint
 * 5. GeometryCollection.
 */
template <typename T>
class WKTGeometry {
public:
    WKTGeometry() = default;

    WKTGeometry(const std::string& wkt) {
        bool success = Load(wkt);
        CHECK(success);
    }

    /**
     * Load geometry from WKT.
     */
    bool Load(const std::string& wkt) {
        if (wkt.empty()) return true;

        std::string type, content;
        if (!GetTypeAndContent(wkt, &type, &content)) return false;

        int dimension = GetDimension(&type);
        if (dimension == -1) return false;

        // Empty geometry.
        if (dimension == 0) return true;
        if (dimension != 2) {
            LOG(WARNING) << "Only support 2D WKT currently.";
            return false;
        }

        if (type == "geometrycollection") {
            Array<std::string> sub_wkts;
            ParseWKTSets(content, &sub_wkts);
            bool failed = false;
            for (auto& wkt : sub_wkts) {
                if (!Load(wkt)) failed = true;
            }
            return !failed;
        } else if (type == "polygon") {
            MultiPolygon2D<T> polygon;
            if (!LoadPolygon2D(content, &polygon)) return false;
            polygons_.push_back(polygon);
            return true;
        } else if (type == "multipolygon") {
            MultiPolygon2D<T> polygon;
            if (!LoadMultiPolygon2D(content, &polygon)) return false;
            polygons_.push_back(polygon);
            return true;
        } else if (type == "linestring") {
            Array<Point2D<T>> vertices;
            if (!LoadVertices(content, &vertices)) return false;
            if (!vertices.empty()) polylines_.push_back(vertices);
            return true;
        } else if (type == "multilinestring") {
            Array<std::string> set;
            if (!ParseWKTSets(content, &set)) return false;
            Array<Point2D<T>> vertices;
            for (const auto& s : set) {
                if (!LoadVertices(s, &vertices)) return false;
                if (!vertices.empty()) polylines_.push_back(vertices);
            }
            return true;
        } else if (type == "point") {
            Array<Point2D<T>> vertices;
            if (!LoadVertices(content, &vertices)) return false;
            for (const auto& p : vertices) points_.push_back(p);
            return true;
        } else if (type == "multipoint") {
            return LoadPoints2D(content);
        } else {
            LOG(ERROR) << "Unsupport WKT type: " << type;
            return false;
        }
    }

    /**
     * Extract a single 2D multi-polygon from WKT geometry.
     */
    bool ExtractSinglePolygon2D(MultiPolygon2D<T>* polygon) const {
        CHECK(polygon);

        polygon->clear();
        for (const auto& poly : polygons_) {
            polygon->Insert(poly);
        }
        return !polygon->empty();
    }

    const Array<MultiPolygon2D<T>>& polygons() const {
        return polygons_;
    }

    /**
     * Open polylines parsed from LineString/MultiLineString geometry. Each entry
     * is one polyline's ordered vertices; not closed.
     */
    const Array<Array<Point2D<T>>>& polylines() const {
        return polylines_;
    }

    /**
     * Points parsed from Point/MultiPoint geometry, flattened across features.
     */
    const Array<Point2D<T>>& points() const {
        return points_;
    }

private:

    /**
     * Parse WKT to a set of string according to the bracket pairs.
     */
    static bool ParseWKTSets(const std::string& wkt, Array<std::string>* sets) {
        CHECK(sets);

        sets->clear();
        if (wkt.empty()) return true;

        int nesting_level = 0;
        std::string term;

        for (size_t i = 0; i < wkt.length(); ++i) {
            if (wkt[i] == '(') {
                ++nesting_level;

                if (nesting_level == 1) continue;
            } else if (wkt[i] == ')') {
                --nesting_level;
                if (nesting_level < 0) {
                    LOG(ERROR) << "Invalid WKT format: bracket mismatch.";
                    return false;
                }
                if (nesting_level == 0) {
                    if (!term.empty()) {
                        sets->push_back(term);
                        term.clear();
                    }
                    continue;
                }
            } else if (wkt[i] == ',') {
                if (nesting_level == 1) {
                    sets->push_back(term);
                    term.clear();
                    continue;
                }
            }

            term.push_back(wkt[i]);
        }
        if (nesting_level != 0) {
            LOG(ERROR) << "Invalid WKT format: bracket mismatch.";
            return false;
        }

        if (!term.empty()) {
            LOG(ERROR) << "Invalid WKT at " << term;
            return false;
        }
        return true;
    }

    /**
     * Load 2D vertices from wkt content.
     */
    static bool LoadVertices(const std::string& wkt,
                             Array<Point2D<T>>* vertices) {
        CHECK(vertices);

        vertices->clear();

        if (wkt.empty()) return true;

        if (wkt.front() != '(' && wkt.back() != ')') {
            LOG(ERROR) << "Invalid WKT, no bracket found in '" << wkt << "'.";
            return false;
        }

        std::string str = wkt.substr(1, wkt.size() - 2);
        TextStream stream(str);
        stream.set_delimiter(',');

        T x, y;
        while (stream >> x >> y) {
            vertices->emplace_back(x, y);
        }

        return true;
    }

    /**
     * Load 2D polygon from WKT content.
     */
    static bool LoadPolygon2D(const std::string& wkt,
                              MultiPolygon2D<T>* polygon) {
        CHECK(polygon);
        polygon->clear();

        Array<std::string> set;
        if (!ParseWKTSets(wkt, &set)) return false;

        Array<Point2D<T>> vertices;
        for (const auto& s : set) {
            if (!LoadVertices(s, &vertices)) return false;
            Polygon2D<T> poly(vertices);
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

    /**
     * Load 2D multi-polygon from WKT content.
     */
    static bool LoadMultiPolygon2D(const std::string& wkt,
                                   MultiPolygon2D<T>* polygon) {
        CHECK(polygon);
        polygon->clear();

        Array<std::string> set;
        if (!ParseWKTSets(wkt, &set)) return false;

        MultiPolygon2D<T> poly;
        for (const auto& s : set) {
            if (!LoadPolygon2D(s, &poly)) return false;
            polygon->Insert(poly);
        }

        return true;
    }

    /**
     * Load MultiPoint content, accepting both WKT forms:
     * "((x y), (x y))" (bracketed members) and "(x y, x y)" (flat coordinates).
     */
    bool LoadPoints2D(const std::string& content) {
        Array<std::string> set;
        if (!ParseWKTSets(content, &set)) return false;

        Array<Point2D<T>> vertices;
        for (const auto& s : set) {
            if (s.find('(') != std::string::npos) {
                if (!LoadVertices(s, &vertices)) return false;
                for (const auto& p : vertices) points_.push_back(p);
            } else {
                TextStream stream(s);
                stream.set_delimiter(',');
                T x, y;
                if (stream >> x >> y) points_.emplace_back(x, y);
            }
        }
        return true;
    }

    /**
     * Extact type and content strings from WKT.
     */
    static bool GetTypeAndContent(const std::string& wkt, std::string* type,
                                  std::string* content) {
        CHECK(type);
        CHECK(content);

        if (wkt.empty()) {
            LOG(ERROR) << "Empty WKT string.";
            return false;
        }

        size_t first_bracket = wkt.find('(');
        if (first_bracket == std::string::npos) {
            if (wkt.find(')') != std::string::npos) {
                LOG(ERROR) << "Invalid WKT: no opening bracket found.";
                return false;
            }
            *type = wkt;
            content->clear();
        } else {
            *type = wkt.substr(0, first_bracket);
            *content = StringTrim(wkt.substr(first_bracket));
        }

        std::transform(type->begin(), type->end(), type->begin(),
                       [](unsigned char c) {
            return std::tolower(c);
        });

        *type = StringTrim(*type);
        return true;
    }

    /**
     * Get dimension info from wkt's head and trim the type.
     *
     * Return -1 if the dimension is invalid.
     */
    int GetDimension(std::string* type) {
        Array<std::string> terms;
        StringSplit(*type, &terms);

        if (terms.empty()) {
            LOG(ERROR) << "WKT type is empty.";
            return -1;
        }

        if (terms.size() == 1) return 2;
        if (terms.size() == 2) {
            *type = terms[0];
            if (terms[1] == "zm") return 4;
            if (terms[1] == "z" || terms[1] == "m") return 3;
            if (terms[1] == "empty") return 0;
        }

        LOG(ERROR) << "Invalid WKT type: " << *type;
        return -1;
    }

    Array<MultiPolygon2D<T>> polygons_;
    Array<Array<Point2D<T>>> polylines_;
    Array<Point2D<T>> points_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_WKT_GEOMETRY_H_

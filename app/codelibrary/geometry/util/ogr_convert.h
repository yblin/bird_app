//
// Copyright 2022-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_OGR_CONVERT_H_
#define CODELIBRARY_GEOMETRY_UTIL_OGR_CONVERT_H_

#if __has_include("ogr_feature.h")
#include "ogr_feature.h"

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/polyline.h"

namespace cl {
namespace geometry {
namespace util {

/**
 * Get OGR linear ring.
 */
template <typename T>
void GetOGRLinearRing(OGRLinearRing* ring, Polygon2D<T>* polygon) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(polygon);

    polygon->clear();
    if (!ring) return;

    Array<Point2D<T>> vertices;
    for (int i = 0; i < ring->getNumPoints(); ++i) {
        vertices.emplace_back(ring->getX(i), ring->getY(i));
    }
    *polygon = Polygon2D<T>(vertices);
}

/**
 * Get OGR polygon.
 */
template <typename T>
void GetOGRPolygon(OGRPolygon* ogr_polygon, MultiPolygon2D<T>* polygon) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(polygon);
    polygon->clear();

    if (!ogr_polygon) return;
    OGRLinearRing* ring = ogr_polygon->getExteriorRing();
    if (!ring) return;

    Polygon2D<T> poly;
    GetOGRLinearRing(ring, &poly);
    if (poly.empty()) return;

    if (poly.IsClockwise()) {
        poly.Reverse();
    }
    polygon->Insert(poly, true);
    int n_inter_rings = ogr_polygon->getNumInteriorRings();
    for (int i = 0; i < n_inter_rings; ++i) {
        ring = ogr_polygon->getInteriorRing(i);
        GetOGRLinearRing(ring, &poly);
        if (!poly.empty()) {
            if (!poly.IsClockwise()) {
                poly.Reverse();
            }
            polygon->Insert(poly, false);
        }
    }
}
template <typename T>
void GetOGRPolygon(OGRGeometry* geometry, MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    polygon->clear();

    if (!geometry) return;
    auto type = wkbFlatten(geometry->getGeometryType());

    CHECK(type == wkbPolygon || type == wkbMultiPolygon) <<
          "Type of geometry is: " << geometry->getGeometryName() <<
          ", not 2D polygon.";
    auto poly = reinterpret_cast<OGRPolygon*>(geometry);
    CHECK(poly);

    if (type == wkbPolygon) {
        OGRPolygon* ogr_poly = reinterpret_cast<OGRPolygon*>(geometry);
        CHECK(ogr_poly) << "Failed to cast geometry to OGRPolygon!";
        GetOGRPolygon(ogr_poly, polygon);
    } else {
        OGRMultiPolygon* ogr_multi_poly =
            reinterpret_cast<OGRMultiPolygon*>(geometry);
        CHECK(ogr_multi_poly) << "Failed to cast geometry to OGRMultiPolygon!";

        const int poly_count = ogr_multi_poly->getNumGeometries();
        for (int i = 0; i < poly_count; ++i) {
            OGRGeometry* sub_geom = ogr_multi_poly->getGeometryRef(i);
            if (!sub_geom ||
                wkbFlatten(sub_geom->getGeometryType()) != wkbPolygon) {
                continue;
            }
            OGRPolygon* sub_poly = reinterpret_cast<OGRPolygon*>(sub_geom);
            if (sub_poly) {
                MultiPolygon2D<T> sub_polygon;
                GetOGRPolygon(sub_poly, &sub_polygon);
                polygon->Insert(sub_polygon);
            }
        }
    }
}
template <typename T>
void GetOGRPolygon(OGRFeature* ogr, MultiPolygon2D<T>* polygon) {
    CHECK(polygon);
    polygon->clear();

    if (!ogr) return;
    GetOGRPolygon(ogr->GetGeometryRef(), polygon);
}

/**
 * Get OGR multi-polygon.
 */
template <typename T>
void GetOGRMultiPolygon(OGRGeometry* geometry,
                        Array<MultiPolygon2D<T>>* polygons) {
    CHECK(polygons);
    polygons->clear();

    if (!geometry) return;

    MultiPolygon2D<T> polygon;
    auto geom_type = wkbFlatten(geometry->getGeometryType());
    if (geom_type == wkbPolygon) {
        GetOGRPolygon(geometry, &polygon);
        polygons->push_back(polygon);
    } else if (geom_type == wkbGeometryCollection) {
        auto collection = reinterpret_cast<OGRGeometryCollection*>(geometry);
        CHECK(collection);

        int n_polygons = collection->getNumGeometries();
        for (int i = 0; i != n_polygons; ++i) {
            GetOGRPolygon(collection->getGeometryRef(i), &polygon);
            polygons->push_back(polygon);
        }
    } else if (geom_type == wkbMultiPolygon) {
        auto multi_polygon = reinterpret_cast<OGRMultiPolygon*>(geometry);
        CHECK(multi_polygon);

        int n_polygons = multi_polygon->getNumGeometries();
        for (int i = 0; i != n_polygons; ++i) {
            OGRPolygon* ogr_polygon =
                reinterpret_cast<OGRPolygon*>(multi_polygon->getGeometryRef(i));
            if (!ogr_polygon) continue;

            GetOGRPolygon(ogr_polygon, &polygon);
            polygons->push_back(polygon);
        }
    } else {
        CHECK(false) << "Type of geometry is: "
                     << wkbFlatten(geometry->getGeometryType())
                     << ", not a valid polygon (or polygon collection) type.";
    }
}
template <typename T>
void GetOGRMultiPolygon(OGRFeature* ogr, Array<MultiPolygon2D<T>>* polygons) {
    CHECK(polygons);
    polygons->clear();

    if (!ogr) return;
    GetOGRMultiPolygon(ogr->GetGeometryRef(), polygons);
}

/**
 * Get OGR 2D polyline.
 */
template <typename T>
void GetOGRPolyline(OGRLineString* line, Polyline<Point2D<T>>* polyline) {
    static_assert(std::is_floating_point<T>::value);

    CHECK(polyline);

    polyline->clear();
    if (!line) return;

    for (int i = 0; i < line->getNumPoints(); ++i) {
        polyline->emplace_back(line->getX(i), line->getY(i));
    }
}

template <typename T>
void GetOGRPolyline(OGRGeometry* geometry, Polyline<Point2D<T>>* polyline) {
    CHECK(polyline);
    polyline->clear();

    if (!geometry) return;
    CHECK(wkbFlatten(geometry->getGeometryType()) == wkbLineString) <<
          "Type of geometry is: " << geometry->getGeometryName() <<
          ", not wkbLineString.";
    auto poly = reinterpret_cast<OGRLineString*>(geometry);
    CHECK(poly);
    GetOGRPolyline(poly, polyline);
}
template <typename T>
void GetOGRPolyline(OGRFeature* ogr, Polyline<Point2D<T>>* polyline) {
    CHECK(polyline);
    polyline->clear();

    if (!ogr) return;
    GetOGRPolyline(ogr->GetGeometryRef(), polyline);
}

////////////////////////////////////////////////////////////////////////////////
// The following code covert codelibrary's geometric objects to OGR geometries.
//
// Use these codes with caution as they allocate raw pointers but never handle
// their lifetime.
////////////////////////////////////////////////////////////////////////////////

/**
 * Convert a polygon to OGR polygon.
 */
template <typename T>
OGRPolygon* ToOGRPolygon(const Polygon2D<T>& polygon) {
    OGRPolygon* ogr_polygon = new OGRPolygon();
    OGRLinearRing* ring = new OGRLinearRing();
    for (int i = 0; i < polygon.size(); ++i) {
      ring->addPoint(polygon[i].x, polygon[i].y);
    }
    ring->addPoint(polygon[0].x, polygon[0].y);
    ogr_polygon->addRing(ring);
    return ogr_polygon;
}

/**
 * Convert a multi-polygon to OGR polygon.
 */
template <typename T>
OGRPolygon* ToOGRPolygon(const MultiPolygon2D<T>& polygon) {
    OGRPolygon* ogr_polygon = new OGRPolygon();
    for (const auto& b : polygon.boundaries()) {
        OGRLinearRing* ring = new OGRLinearRing();
        const Polygon2D<T>& poly = b.polygon;
        for (int i = 0; i < poly.size(); ++i) {
          ring->addPoint(poly[i].x, poly[i].y);
        }
        ring->addPoint(poly[0].x, poly[0].y);
        ogr_polygon->addRing(ring);
    }
    return ogr_polygon;
}

/**
 * Convert polylines to OGRLineString.
 */
template <typename T>
OGRLineString* ToOGRLineString(const Polyline<Point2D<T>>& polyline) {
    OGRLineString* ogr_polyline = new OGRLineString();
    for (auto p : polyline) {
        ogr_polyline->addPoint(p.x, p.y);
    }
    return ogr_polyline;
}

} // namespace util
} // namespace geometry
} // namespace cl

#endif // if __has_inlcude("ogr_feature.h")

#endif // CODELIBRARY_GEOMETRY_UTIL_OGR_CONVERT_H_

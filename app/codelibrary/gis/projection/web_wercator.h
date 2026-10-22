//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_PROJECTION_WEB_MERCATOR_H_
#define CODELIBRARY_GIS_PROJECTION_WEB_MERCATOR_H_

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/numbers.h"
#include "codelibrary/gis/ellipsoid_model.h"

namespace cl {
namespace gis {

/**
 * Web Mercator, Google Web Mercator, Spherical Mercator, WGS 84 Web Mercator or
 * WGS 84/Pseudo-Mercator is a variant of the Mercator map projection and is the
 * de facto standard for Web mapping applications.
 *
 * Its official EPSG identifier is EPSG:3857, although others have been used
 * historically.
 */
class WebMercator {
public:
    WebMercator() {
        wgs84_ = EllipsoidModel::WGS84();
    }

    /**
     * Convert WebMercator coordinates to geographic coordinates (latitude,
     * longitude in degrees).
     */
    RPoint2D ToGeographic(double east, double north) const {
        double d = -north / wgs84_.long_radius();
        double phi = numbers::pi * 0.5 - 2.0 * std::atan(std::pow(numbers::e,
                                                                  d));
        double lambda = east / wgs84_.long_radius();

        return {RadianToDegree(lambda), RadianToDegree(phi)};
    }

    /**
     * Convert geographic coordinates (longitude, latitude in degrees) to Web
     * mercator coordinates.
     */
    RPoint2D FromGeographic(double lon, double lat) const {
        lat = Clamp(lat, -85.051129, 85.051129);

        double lambda = DegreeToRadian(lon);
        double phi = DegreeToRadian(lat);

        double east = wgs84_.long_radius() * lambda;
        double north = wgs84_.long_radius() *
                       std::log(std::tan(numbers::pi * 0.25 + phi * 0.5));

        return {east, north};
    }

private:
    EllipsoidModel wgs84_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_PROJECTION_WEB_MERCATOR_H_

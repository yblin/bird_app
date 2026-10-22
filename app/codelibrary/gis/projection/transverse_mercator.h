//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_PROJECTION_TRANSVERSE_MERCATOR_H_
#define CODELIBRARY_GIS_PROJECTION_TRANSVERSE_MERCATOR_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/gis/ellipsoid_model.h"

namespace cl {
namespace gis {

/**
 * The transverse Mercator map projection (TM, TMP) is an adaptation of the
 * standard Mercator projection. The transverse version is widely used in
 * national and international mapping systems around the world, including the
 * Universal Transverse Mercator. When paired with a suitable geodetic datum,
 * the transverse Mercator delivers high accuracy in zones less than a few
 * degrees in east-west extent.
 */
class TransverseMercator {
public:
    /**
     * Parameters:
     *  lon   - The longitude in degrees. It should be in the range of -180 to
     *          180 degrees.
     *  lat   - The latitude in degrees. It should be in the range of -90 to 90
     *          degrees.
     *  scale - The scale factor that applies to the projection.
     *          This is used to adjust the size of the projected coordinates.
     *  east  - offset in east.
     *  north - offset in north.
     */
    TransverseMercator(double lon, double lat, double scale = 1.0,
                       double east = 0.0, double north = 0.0) {
        wgs84_ = EllipsoidModel::WGS84();

        double f = wgs84_.flattening();
        double e = wgs84_.eccentricity();
        double phi0 = DegreeToRadian(lat);
        lambda0_ = DegreeToRadian(lon);

        double n = f / (2.0 - f);
        double n2 = n * n;
        double n3 = n2 * n;
        double n4 = n3 * n;
        b_ = (wgs84_.long_radius() / (1.0 + n)) * (1.0 + n2 / 4.0 + n4 / 64.0);
        h1_ = n / 2.0 - (2.0 / 3.0) * n2 + (5.0 / 16.0) * n3 +
              (41.0 / 180.0) * n4;
        h2_ = (13.0 / 48.0) * n2 - (3.0 / 5.0) * n3 + (557.0 / 1440.0) * n4;
        h3_ = (61.0 / 240.0) * n3 - (103.0 / 140.0) * n4;
        h4_ = (49561.0 / 161280.0) * n4;

        if (phi0 == 0.0) {
            m0_ = 0.0;
        } else if (phi0 == numbers::pi * 0.5) {
            m0_ = b_ * numbers::pi * 0.5;
        } else if (phi0 == -numbers::pi * 0.5) {
            m0_ = b_ * -numbers::pi * 0.5;
        } else {
            double q0 = std::asinh(std::tan(phi0)) -
                        (e * std::atanh(e * std::sin(phi0)));
            double xi00 = std::atan(std::sinh(q0));
            double xi01 = h1_ * std::sin(2.0 * xi00);
            double xi02 = h2_ * std::sin(4.0 * xi00);
            double xi03 = h3_ * std::sin(6.0 * xi00);
            double xi04 = h4_ * std::sin(8.0 * xi00);
            double xi0 = xi00 + xi01 + xi02 + xi03 + xi04;
            m0_ = b_ * xi0;
        }

        h1i_ = n / 2.0 - (2.0 / 3.0) * n2 + (37.0 / 96.0) * n3 +
               (1.0 / 360.0) * n4;
        h2i_ = (1.0 / 48.0) * n2 - (1.0 / 15.0) * n3 + (437.0 / 1440.0) * n4;
        h3i_ = (17.0 / 480.0) * n3 - (37.0 / 840.0) * n4;
        h4i_ = (4397.0 / 161280.0) * n4;

        scale_ = scale;
        east_ = east;
        north_ = north;
    }

    /**
     * Convert UTM coordinates to geographic coordinates (latitude, longitude
     * in degrees).
     */
    RPoint2D UTMToGeographic(double east, double north) const {
        double etai = (east - east_) / (b_ * scale_);
        double xii = ((north - north_) + scale_ * m0_) / (b_ * scale_);

        double xi1i = h1i_ * std::sin(2.0 * xii) * std::cosh(2.0 * etai);
        double xi2i = h2i_ * std::sin(4.0 * xii) * std::cosh(4.0 * etai);
        double xi3i = h3i_ * std::sin(6.0 * xii) * std::cosh(6.0 * etai);
        double xi4i = h4i_ * std::sin(8.0 * xii) * std::cosh(8.0 * etai);

        double xi0i = xii - (xi1i + xi2i + xi3i + xi4i);

        double eta1i = h1i_ * std::cos(2.0 * xii) * std::sinh(2.0 * etai);
        double eta2i = h2i_ * std::cos(4.0 * xii) * std::sinh(4.0 * etai);
        double eta3i = h3i_ * std::cos(6.0 * xii) * std::sinh(6.0 * etai);
        double eta4i = h4i_ * std::cos(8.0 * xii) * std::sinh(8.0 * etai);

        double eta0i = etai - (eta1i + eta2i + eta3i + eta4i);

        double betai = std::asin(std::sin(xi0i) / std::cosh(eta0i));
        double qi = std::asinh(std::tan(betai));
        double e = wgs84_.eccentricity();
        double qii = qi + (e * std::atanh(e * std::tanh(qi)));

        for (int i = 0; i < 15; ++i) {
            qii = qi + (e * std::atanh(e * std::tanh(qii)));
        }

        double phi = std::atan(std::sinh(qii));
        double lambda = lambda0_ + std::asin(std::tanh(eta0i) /
                                             std::cos(betai));

        return {RadianToDegree(lambda), RadianToDegree(phi)};
    }

    /**
     * Convert UTM coordinates to cartographic coordinates (latitude, longitude
     * in degrees, and altitude in metres).
     */
    RPoint3D UTMToGeographic(double east, double north, double h) const {
        RPoint2D q = UTMToGeographic(east, north);
        return {q.x, q.y, h};
    }

    /**
     * Convert geographic coordinates (longitude, latitude in degrees) to UTM
     * coordinates.
     */
    RPoint2D GeographicToUTM(double lon, double lat) const {
        double phi = DegreeToRadian(lat);
        double lambda = DegreeToRadian(lon);

        double e = wgs84_.eccentricity();
        double q = std::asinh(std::tan(phi)) -
                   e * std::atanh(e * std::sin(phi));
        double beta = std::atan(std::sinh(q));
        double eta0 = std::atanh(std::cos(beta) * std::sin(lambda - lambda0_));
        double xi0 = std::asin(std::sin(beta) * std::cosh(eta0));

        double xi1 = h1_ * std::sin(2.0 * xi0) * std::cosh(2.0 * eta0);
        double xi2 = h2_ * std::sin(4.0 * xi0) * std::cosh(4.0 * eta0);
        double xi3 = h3_ * std::sin(6.0 * xi0) * std::cosh(6.0 * eta0);
        double xi4 = h4_ * std::sin(8.0 * xi0) * std::cosh(8.0 * eta0);

        double xi = xi0 + xi1 + xi2 + xi3 + xi4;

        double eta1 = h1_ * std::cos(2.0 * xi0) * std::sinh(2.0 * eta0);
        double eta2 = h2_ * std::cos(4.0 * xi0) * std::sinh(4.0 * eta0);
        double eta3 = h3_ * std::cos(6.0 * xi0) * std::sinh(6.0 * eta0);
        double eta4 = h4_ * std::cos(8.0 * xi0) * std::sinh(8.0 * eta0);

        double eta = eta0 + eta1 + eta2 + eta3 + eta4;

        double east = east_ + scale_ * b_ * eta;
        double north = north_ + scale_ * (b_ * xi - m0_);

        return {east, north};
    }

    /**
     * Convert geographic coordinates (longitude, latitude in degrees, and
     * altitude in metres) to UTM coordinates.
     */
    RPoint3D GeographicToUTM(double lon, double lat, double h) const {
        RPoint2D q = GeographicToUTM(lon, lat);
        return {q.x, q.y, h};
    }

private:
    double lambda0_;
    double b_, h1_, h2_, h3_, h4_, m0_;
    double h1i_, h2i_, h3i_, h4i_;
    double scale_;
    double east_;
    double north_;
    EllipsoidModel wgs84_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_PROJECTION_TRANSVERSE_MERCATOR_H_

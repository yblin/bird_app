//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_PROJECTION_ELLIPSOID_MODEL_H_
#define CODELIBRARY_GIS_PROJECTION_ELLIPSOID_MODEL_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {
namespace gis {

/**
 * Provide an ellipsoid definition of a celestrial body and helper methods for
 * computing positions/transforms on that ellipsoid.
 */
class EllipsoidModel {
public:
    EllipsoidModel() = default;

    EllipsoidModel(double l, double s)
        : long_radius_(l), short_radius_(s) {
        CHECK(long_radius_ >= short_radius_);

        double flattening = (long_radius_ - short_radius_) / long_radius_;
        eccentricity_squared_ = 2.0 * flattening - flattening * flattening;
    }

    bool empty() const {
        return long_radius_ == 0.0 || short_radius_ == 0.0;
    }

    /**
     * Convert from Geographic coordinates (longitude, latitude in degrees,
     * and altitude in metres) to ECEF coordinates in metres.
     */
    RPoint3D GeographicToECEF(double longitude, double latitude,
                              double altitude) const {
        latitude = DegreeToRadian(latitude);
        longitude = DegreeToRadian(longitude);

        // for details on maths see https://en.wikipedia.org/wiki/ECEF
        double sin_latitude = std::sin(latitude);
        double cos_latitude = std::cos(latitude);
        double n = long_radius_ / std::sqrt(1.0 - eccentricity_squared_ *
                                            sin_latitude * sin_latitude);
        return {(n + altitude) * cos_latitude * std::cos(longitude),
                (n + altitude) * cos_latitude * std::sin(longitude),
                (n * (1.0 - eccentricity_squared_) + altitude) * sin_latitude};
    }

    /**
     * Convert from ECEF coordinates in metres to geographic coordinates
     * (longitude, latitude in degrees, and altitude in metres).
     */
    RPoint3D ECEFToGeographic(const RPoint3D& ecef) const {
        double latitude, longitude, height;
        const double pi_2 = 0.5 * numbers::pi;

        // Handle polar and center-of-earth cases directly.
        double epsilon = std::numeric_limits<double>::epsilon();
        if (std::fabs(ecef.x) > epsilon) {
            longitude = std::atan2(ecef.y, ecef.x);
        } else {
            if (ecef.y > epsilon) {
                longitude = pi_2;
            } else if (ecef.y < -epsilon) {
                longitude = -pi_2;
            } else {
                // At pole or at center of the spheroid.
                longitude = 0.0;
                // NOTE (modified by Claude): original code set latitude to
                // +/-180 degrees, which is outside the valid geographic
                // latitude range of [-90, 90]. Corrected to +/-90 (poles)
                // and 0 (earth center, which is a degenerate case).
                if (ecef.z > 0.0) {
                    // North pole.
                    latitude = 90.0;
                    height = ecef.z - short_radius_;
                } else if (ecef.z < 0.0) {
                    // South pole.
                    latitude = -90.0;
                    height = -ecef.z - short_radius_;
                } else {
                    // Center of earth.
                    latitude = 0.0;
                    height = -short_radius_;
                }
                return {longitude, latitude, height};
            }
        }

        double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
        double theta = std::atan2(ecef.z * long_radius_, (p * short_radius_));
        double e_dash_squared = (long_radius_ * long_radius_ -
                                 short_radius_ * short_radius_) /
                                (short_radius_ * short_radius_);

        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        latitude = std::atan((ecef.z + e_dash_squared * short_radius_ *
                              sin_theta * sin_theta * sin_theta) /
                             (p - eccentricity_squared_ * long_radius_ *
                              cos_theta * cos_theta * cos_theta));

        double sin_latitude = std::sin(latitude);
        double n = long_radius_ / std::sqrt(1.0 - eccentricity_squared_ *
                                            sin_latitude * sin_latitude);

        height = p / std::cos(latitude) - n;
        return {RadianToDegree(longitude), RadianToDegree(latitude), height};
    }

    /**
     * Compute the geographic center of given 2D geographic points (all in
     * degrees).
     */
    RPoint2D GeographicCenter(const Array<RPoint2D>& points) const {
        Array<RPoint3D> ecefs;
        for (auto p : points) {
            ecefs.push_back(GeographicToECEF(p.x, p.y, 0.0));
        }
        RPoint3D center = Center(ecefs);
        center = ECEFToGeographic(center);
        return RPoint2D(center.x, center.y);
    }

    /**
     * Compute the geographic center of given 3D geographic points (all in
     * degrees).
     */
    RPoint3D GeographicCenter(const Array<RPoint3D>& points) const {
        Array<RPoint3D> ecefs;
        for (auto p : points) {
            ecefs.push_back(GeographicToECEF(p.x, p.y, p.z));
        }
        return ECEFToGeographic(Center(ecefs));
    }

    static EllipsoidModel WGS84() {
        return EllipsoidModel(6378137.0, 6356752.3142451793);
    }

    double long_radius() const {
        return long_radius_;
    }

    double short_radius() const {
        return short_radius_;
    }

    double flattening() const {
        return (long_radius_ - short_radius_) / long_radius_;
    }

    double eccentricity() const {
        return std::sqrt(eccentricity_squared_);
    }

    double eccentricity_squared() const {
        return eccentricity_squared_;
    }

private:
    double long_radius_ = 0.0;
    double short_radius_ = 0.0;
    double eccentricity_squared_ = 0.0;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_PROJECTION_ELLIPSOID_MODEL_H_

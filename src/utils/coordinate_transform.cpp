#include "utils/coordinate_transform.hpp"
#include <cmath>

namespace sensor_fusion {

std::array<double, 3> CoordinateTransform::lla2ecef(double lat, double lon, double alt)
{
    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double sinLon = std::sin(lon);
    double cosLon = std::cos(lon);

    // Radius of curvature in prime vertical
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

    std::array<double, 3> ecef;
    ecef[0] = (N + alt) * cosLat * cosLon;
    ecef[1] = (N + alt) * cosLat * sinLon;
    ecef[2] = (N * (1.0 - WGS84_E2) + alt) * sinLat;

    return ecef;
}

std::array<double, 3> CoordinateTransform::ecef2lla(const std::array<double, 3>& ecef)
{
    double x = ecef[0];
    double y = ecef[1];
    double z = ecef[2];

    double lon = std::atan2(y, x);

    // Iterative algorithm for latitude
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1.0 - WGS84_E2));

    for (int i = 0; i < 5; ++i) {
        double sinLat = std::sin(lat);
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
        lat = std::atan2(z + WGS84_E2 * N * sinLat, p);
    }

    double sinLat = std::sin(lat);
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
    double alt = p / std::cos(lat) - N;

    return {lat, lon, alt};
}

void CoordinateTransform::getECEFtoNEDRotation(double lat, double lon, double R_ne[3][3])
{
    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double sinLon = std::sin(lon);
    double cosLon = std::cos(lon);

    R_ne[0][0] = -sinLat * cosLon;
    R_ne[0][1] = -sinLat * sinLon;
    R_ne[0][2] = cosLat;

    R_ne[1][0] = -sinLon;
    R_ne[1][1] = cosLon;
    R_ne[1][2] = 0.0;

    R_ne[2][0] = -cosLat * cosLon;
    R_ne[2][1] = -cosLat * sinLon;
    R_ne[2][2] = -sinLat;
}

std::array<double, 3> CoordinateTransform::ecefVelToNED(const std::array<double, 3>& v_ecef,
                                                       double lat, double lon)
{
    double R_ne[3][3];
    getECEFtoNEDRotation(lat, lon, R_ne);

    std::array<double, 3> v_ned;
    for (int i = 0; i < 3; ++i) {
        v_ned[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            v_ned[i] += R_ne[i][j] * v_ecef[j];
        }
    }

    return v_ned;
}

std::array<double, 3> CoordinateTransform::ned2ecef(const std::array<double, 3>& ned,
                                                   double ref_lat, double ref_lon, double ref_alt)
{
    // Get reference position in ECEF
    auto ref_ecef = lla2ecef(ref_lat, ref_lon, ref_alt);

    // Get rotation matrix from NED to ECEF (transpose of ECEF to NED)
    double R_ne[3][3];
    getECEFtoNEDRotation(ref_lat, ref_lon, R_ne);

    // Transpose to get R_en
    double R_en[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_en[i][j] = R_ne[j][i];
        }
    }

    // Transform NED to ECEF offset
    std::array<double, 3> ecef_offset;
    for (int i = 0; i < 3; ++i) {
        ecef_offset[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            ecef_offset[i] += R_en[i][j] * ned[j];
        }
    }

    // Add to reference
    return {
        ref_ecef[0] + ecef_offset[0],
        ref_ecef[1] + ecef_offset[1],
        ref_ecef[2] + ecef_offset[2]
    };
}

} // namespace sensor_fusion

#ifndef SENSOR_FUSION_COORDINATE_TRANSFORM_HPP
#define SENSOR_FUSION_COORDINATE_TRANSFORM_HPP

#include <array>

namespace sensor_fusion {

/**
 * @brief Coordinate transformation utilities
 */
class CoordinateTransform {
public:
    /**
     * @brief Convert geodetic coordinates (LLA) to ECEF
     * @param lat Latitude [rad]
     * @param lon Longitude [rad]
     * @param alt Altitude above ellipsoid [m]
     * @return ECEF coordinates [x, y, z] in meters
     */
    static std::array<double, 3> lla2ecef(double lat, double lon, double alt);

    /**
     * @brief Convert ECEF to geodetic coordinates (LLA)
     * @param ecef ECEF coordinates [x, y, z] in meters
     * @return [lat, lon, alt] where lat/lon are in radians, alt in meters
     */
    static std::array<double, 3> ecef2lla(const std::array<double, 3>& ecef);

    /**
     * @brief Get rotation matrix from ECEF to NED at given location
     * @param lat Latitude [rad]
     * @param lon Longitude [rad]
     * @param R_ne Output rotation matrix
     */
    static void getECEFtoNEDRotation(double lat, double lon, double R_ne[3][3]);

    /**
     * @brief Convert ECEF velocity to NED velocity
     * @param v_ecef Velocity in ECEF frame
     * @param lat Latitude [rad]
     * @param lon Longitude [rad]
     * @return Velocity in NED frame
     */
    static std::array<double, 3> ecefVelToNED(const std::array<double, 3>& v_ecef,
                                              double lat, double lon);

    /**
     * @brief Convert NED position to ECEF (relative to reference point)
     * @param ned NED coordinates [N, E, D] in meters
     * @param ref_lat Reference latitude [rad]
     * @param ref_lon Reference longitude [rad]
     * @param ref_alt Reference altitude [m]
     * @return ECEF coordinates
     */
    static std::array<double, 3> ned2ecef(const std::array<double, 3>& ned,
                                         double ref_lat, double ref_lon, double ref_alt);

    // WGS84 ellipsoid parameters
    static constexpr double WGS84_A = 6378137.0;           // Semi-major axis [m]
    static constexpr double WGS84_F = 1.0 / 298.257223563; // Flattening
    static constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_COORDINATE_TRANSFORM_HPP

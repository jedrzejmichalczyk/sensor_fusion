#ifndef SENSOR_FUSION_STATE_HPP
#define SENSOR_FUSION_STATE_HPP

#include <array>
#include <cmath>

namespace sensor_fusion {

/**
 * @brief Navigation state vector
 *
 * State consists of:
 * - Position (3D) in NED frame [m]
 * - Velocity (3D) in NED frame [m/s]
 * - Quaternion (4D) representing attitude
 * - Accelerometer bias (3D) [m/s^2]
 * - Gyroscope bias (3D) [rad/s]
 *
 * Total: 16 states
 */
struct NavigationState {
    std::array<double, 3> position_ned;     // [N, E, D] in meters
    std::array<double, 3> velocity_ned;     // [vN, vE, vD] in m/s
    std::array<double, 4> quaternion;       // [qw, qx, qy, qz] (w is scalar part)
    std::array<double, 3> accel_bias;       // [bx, by, bz] in m/s^2
    std::array<double, 3> gyro_bias;        // [bx, by, bz] in rad/s

    /**
     * @brief Default constructor initializes to stationary at origin
     */
    NavigationState();

    /**
     * @brief Get rotation matrix from body to NED frame
     */
    void getRotationMatrix(double R[3][3]) const;

    /**
     * @brief Get Euler angles (roll, pitch, yaw) from quaternion
     * @return [roll, pitch, yaw] in radians
     */
    std::array<double, 3> getEulerAngles() const;

    /**
     * @brief Set quaternion from Euler angles
     * @param roll Roll angle [rad]
     * @param pitch Pitch angle [rad]
     * @param yaw Yaw angle [rad]
     */
    void setFromEulerAngles(double roll, double pitch, double yaw);

    /**
     * @brief Normalize quaternion to unit length
     */
    void normalizeQuaternion();
};

/**
 * @brief Error state representation for EKF
 *
 * Error state consists of:
 * - Position error (3D)
 * - Velocity error (3D)
 * - Attitude error (3D) - using rotation vector representation
 * - Accelerometer bias error (3D)
 * - Gyroscope bias error (3D)
 *
 * Total: 15 states (note: attitude error uses 3D rotation vector, not quaternion)
 */
constexpr int ERROR_STATE_SIZE = 15;

// State indices for error state vector
constexpr int IDX_POS = 0;
constexpr int IDX_VEL = 3;
constexpr int IDX_ATT = 6;
constexpr int IDX_ACC_BIAS = 9;
constexpr int IDX_GYR_BIAS = 12;

} // namespace sensor_fusion

#endif // SENSOR_FUSION_STATE_HPP

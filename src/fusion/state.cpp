#include "fusion/state.hpp"
#include <cmath>
#include <algorithm>

namespace sensor_fusion {

NavigationState::NavigationState()
{
    // Initialize position to origin
    position_ned = {0.0, 0.0, 0.0};

    // Initialize velocity to zero
    velocity_ned = {0.0, 0.0, 0.0};

    // Initialize quaternion to identity (no rotation)
    quaternion = {1.0, 0.0, 0.0, 0.0};

    // Initialize biases to zero
    accel_bias = {0.0, 0.0, 0.0};
    gyro_bias = {0.0, 0.0, 0.0};
}

void NavigationState::getRotationMatrix(double R[3][3]) const
{
    // Convert quaternion to rotation matrix
    double qw = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    // Rotation matrix from body to navigation frame
    R[0][0] = 1.0 - 2.0*(qy*qy + qz*qz);
    R[0][1] = 2.0*(qx*qy - qw*qz);
    R[0][2] = 2.0*(qx*qz + qw*qy);

    R[1][0] = 2.0*(qx*qy + qw*qz);
    R[1][1] = 1.0 - 2.0*(qx*qx + qz*qz);
    R[1][2] = 2.0*(qy*qz - qw*qx);

    R[2][0] = 2.0*(qx*qz - qw*qy);
    R[2][1] = 2.0*(qy*qz + qw*qx);
    R[2][2] = 1.0 - 2.0*(qx*qx + qy*qy);
}

std::array<double, 3> NavigationState::getEulerAngles() const
{
    double qw = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    std::array<double, 3> euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        euler[1] = std::copysign(M_PI / 2.0, sinp); // Use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

void NavigationState::setFromEulerAngles(double roll, double pitch, double yaw)
{
    // Convert Euler angles to quaternion
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    quaternion[0] = cr * cp * cy + sr * sp * sy;  // qw
    quaternion[1] = sr * cp * cy - cr * sp * sy;  // qx
    quaternion[2] = cr * sp * cy + sr * cp * sy;  // qy
    quaternion[3] = cr * cp * sy - sr * sp * cy;  // qz

    normalizeQuaternion();
}

void NavigationState::normalizeQuaternion()
{
    double norm = std::sqrt(quaternion[0]*quaternion[0] +
                           quaternion[1]*quaternion[1] +
                           quaternion[2]*quaternion[2] +
                           quaternion[3]*quaternion[3]);

    if (norm > 1e-9) {
        for (int i = 0; i < 4; ++i) {
            quaternion[i] /= norm;
        }
    } else {
        // Reset to identity if degenerate
        quaternion = {1.0, 0.0, 0.0, 0.0};
    }
}

} // namespace sensor_fusion

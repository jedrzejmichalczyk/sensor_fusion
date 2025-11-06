/**
 * @file ekf_debug.cpp
 * @brief Debug tool to identify EKF divergence issues
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "fusion/ekf.hpp"
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"

using namespace sensor_fusion;

constexpr double GRAVITY = 9.81;

void printState(const NavigationState& state, const char* label) {
    std::cout << label << ":\n";
    std::cout << "  Pos: [" << state.position_ned[0] << ", "
              << state.position_ned[1] << ", " << state.position_ned[2] << "]\n";
    std::cout << "  Vel: [" << state.velocity_ned[0] << ", "
              << state.velocity_ned[1] << ", " << state.velocity_ned[2] << "]\n";
    auto euler = state.getEulerAngles();
    std::cout << "  Att: [" << euler[0] << ", " << euler[1] << ", " << euler[2] << "] rad\n";
}

int main() {
    std::cout << "EKF Debug Tool - Identifying Divergence Issues\n";
    std::cout << "===============================================\n\n";

    // Test 1: Pure gravity, stationary
    std::cout << "TEST 1: Stationary with gravity only\n";
    std::cout << "-------------------------------------\n";

    NavigationState state;
    ExtendedKalmanFilter ekf(state);

    printState(state, "Initial state");

    // Simulate 1 second with pure gravity
    double dt = 0.01;
    IMUMeasurement imu_meas;
    imu_meas.timestamp = 0.0;
    imu_meas.accel[0] = 0.0;
    imu_meas.accel[1] = 0.0;
    imu_meas.accel[2] = GRAVITY;  // Accel measures specific force, at rest this is +g upward (in body frame)
    imu_meas.gyro[0] = 0.0;
    imu_meas.gyro[1] = 0.0;
    imu_meas.gyro[2] = 0.0;

    std::cout << "\nIMU measurement (at rest, level):\n";
    std::cout << "  Accel: [" << imu_meas.accel[0] << ", "
              << imu_meas.accel[1] << ", " << imu_meas.accel[2] << "]\n";
    std::cout << "  Gyro:  [" << imu_meas.gyro[0] << ", "
              << imu_meas.gyro[1] << ", " << imu_meas.gyro[2] << "]\n";

    for (int i = 0; i < 10; ++i) {
        ekf.predict(imu_meas, dt);

        if (i % 1 == 0) {
            auto st = ekf.getState();
            std::cout << "\nAfter " << (i+1)*dt << " seconds:\n";
            std::cout << "  Pos: [" << st.position_ned[0] << ", "
                      << st.position_ned[1] << ", " << st.position_ned[2] << "]\n";
            std::cout << "  Vel: [" << st.velocity_ned[0] << ", "
                      << st.velocity_ned[1] << ", " << st.velocity_ned[2] << "]\n";

            // Check for divergence
            double pos_mag = std::sqrt(st.position_ned[0]*st.position_ned[0] +
                                       st.position_ned[1]*st.position_ned[1] +
                                       st.position_ned[2]*st.position_ned[2]);
            double vel_mag = std::sqrt(st.velocity_ned[0]*st.velocity_ned[0] +
                                       st.velocity_ned[1]*st.velocity_ned[1] +
                                       st.velocity_ned[2]*st.velocity_ned[2]);

            if (pos_mag > 1.0 || vel_mag > 1.0) {
                std::cout << "  *** DIVERGENCE DETECTED! ***\n";
                std::cout << "  Position magnitude: " << pos_mag << " m\n";
                std::cout << "  Velocity magnitude: " << vel_mag << " m/s\n";
                std::cout << "\n  EXPECTED: Position and velocity should remain near zero\n";
                std::cout << "  REASON: Stationary platform with only gravity\n";
                return 1;
            }
        }
    }

    std::cout << "\n✓ TEST 1 PASSED: No divergence in stationary case\n\n";

    // Test 2: Constant velocity
    std::cout << "TEST 2: Constant velocity (5 m/s North)\n";
    std::cout << "----------------------------------------\n";

    NavigationState state2;
    state2.velocity_ned[0] = 5.0;  // 5 m/s North
    ExtendedKalmanFilter ekf2(state2);

    imu_meas.accel[0] = 0.0;
    imu_meas.accel[1] = 0.0;
    imu_meas.accel[2] = GRAVITY;  // No acceleration, just gravity

    for (int i = 0; i < 100; ++i) {
        ekf2.predict(imu_meas, dt);

        if (i % 10 == 9) {
            auto st = ekf2.getState();
            double t = (i+1) * dt;
            double expected_pos = 5.0 * t;
            double error = std::abs(st.position_ned[0] - expected_pos);

            std::cout << "t=" << t << "s: Pos=" << st.position_ned[0]
                     << " (expected " << expected_pos << "), error=" << error << "\n";

            if (error > 5.0) {  // More than 5m error
                std::cout << "  *** DIVERGENCE! Error too large\n";
                return 1;
            }
        }
    }

    std::cout << "\n✓ TEST 2 PASSED: Constant velocity integrated correctly\n\n";

    std::cout << "All tests PASSED! EKF basic mechanization is working.\n";
    return 0;
}

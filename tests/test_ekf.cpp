/**
 * @file test_ekf.cpp
 * @brief Test Extended Kalman Filter
 */

#include <iostream>
#include <cmath>
#include "fusion/ekf.hpp"
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"

using namespace sensor_fusion;

constexpr double GRAVITY = 9.81;  // m/s^2

int main()
{
    std::cout << "Testing Extended Kalman Filter...\n";

    // Test 1: Initialization
    std::cout << "\nTest 1: EKF initialization\n";
    NavigationState initial_state;
    ExtendedKalmanFilter ekf(initial_state);

    auto state = ekf.getState();
    std::cout << "  Initial position: [" << state.position_ned[0] << ", "
             << state.position_ned[1] << ", " << state.position_ned[2] << "]\n";

    // Test 2: Prediction with zero input
    std::cout << "\nTest 2: Prediction with stationary input\n";
    IMUMeasurement imu_meas;
    imu_meas.timestamp = 0.0;
    imu_meas.accel[0] = 0.0;
    imu_meas.accel[1] = 0.0;
    imu_meas.accel[2] = GRAVITY;  // Only gravity
    imu_meas.gyro = {0.0, 0.0, 0.0};

    for (int i = 0; i < 100; ++i) {
        ekf.predict(imu_meas, 0.01);
    }

    state = ekf.getState();
    std::cout << "  Position after 1s: [" << state.position_ned[0] << ", "
             << state.position_ned[1] << ", " << state.position_ned[2] << "]\n";

    bool stationary_ok = (std::abs(state.position_ned[0]) < 0.1 &&
                         std::abs(state.position_ned[1]) < 0.1 &&
                         std::abs(state.position_ned[2]) < 0.1);

    // Test 3: GPS update
    std::cout << "\nTest 3: GPS measurement update\n";

    // Add some position error
    state.position_ned[0] = 10.0;

    GPSMeasurement gps_meas;
    gps_meas.timestamp = 1.0;
    gps_meas.position_ned = {0.0, 0.0, 0.0};  // True position is origin
    gps_meas.velocity_ned = {0.0, 0.0, 0.0};
    gps_meas.valid = true;

    ekf.updateGPS(gps_meas);
    state = ekf.getState();

    std::cout << "  Position after GPS update: [" << state.position_ned[0] << ", "
             << state.position_ned[1] << ", " << state.position_ned[2] << "]\n";

    bool gps_update_ok = std::abs(state.position_ned[0]) < 5.0;  // Should reduce error

    // Test 4: Covariance
    std::cout << "\nTest 4: Covariance matrix\n";
    auto cov_diag = ekf.getCovarianceDiagonal();
    std::cout << "  Position std: [" << std::sqrt(cov_diag[0]) << ", "
             << std::sqrt(cov_diag[1]) << ", " << std::sqrt(cov_diag[2]) << "]\n";

    bool all_tests_ok = stationary_ok && gps_update_ok;

    std::cout << "\nAll tests " << (all_tests_ok ? "PASSED" : "FAILED") << "!\n";

    return all_tests_ok ? 0 : 1;
}

/**
 * @file ekf_debug2.cpp
 * @brief Debug EKF with GPS updates
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "fusion/ekf.hpp"
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"

using namespace sensor_fusion;

constexpr double GRAVITY = 9.81;

int main() {
    std::cout << "EKF Debug - GPS Update Test\n";
    std::cout << "============================\n\n";

    // Test with GPS updates
    std::cout << "TEST 3: With GPS updates (stationary)\n";
    std::cout << "--------------------------------------\n";

    NavigationState state;
    ExtendedKalmanFilter ekf(state);

    // Print initial covariance
    auto cov = ekf.getCovarianceDiagonal();
    std::cout << "Initial covariance diagonal:\n";
    std::cout << "  Pos: [" << std::sqrt(cov[0]) << ", " << std::sqrt(cov[1]) << ", " << std::sqrt(cov[2]) << "] m\n";
    std::cout << "  Vel: [" << std::sqrt(cov[3]) << ", " << std::sqrt(cov[4]) << ", " << std::sqrt(cov[5]) << "] m/s\n";
    std::cout << "  Att: [" << std::sqrt(cov[6]) << ", " << std::sqrt(cov[7]) << ", " << std::sqrt(cov[8]) << "] rad\n\n";

    GPS gps(GPSErrorModel(), 1.0, 999);  // Fixed seed for reproducibility

    double dt = 0.01;
    IMUMeasurement imu_meas;
    imu_meas.timestamp = 0.0;
    imu_meas.accel[0] = 0.0;
    imu_meas.accel[1] = 0.0;
    imu_meas.accel[2] = GRAVITY;
    imu_meas.gyro[0] = 0.0;
    imu_meas.gyro[1] = 0.0;
    imu_meas.gyro[2] = 0.0;

    std::array<double, 3> true_pos = {0.0, 0.0, 0.0};
    std::array<double, 3> true_vel = {0.0, 0.0, 0.0};

    for (int i = 0; i < 1000; ++i) {
        double t = i * dt;

        // Prediction
        ekf.predict(imu_meas, dt);

        // GPS update every 1 second
        if (i % 100 == 0 && i > 0) {
            auto gps_meas = gps.measure(true_pos, true_vel, t);

            std::cout << "t=" << t << "s GPS measurement:\n";
            std::cout << "  Meas pos: [" << gps_meas.position_ned[0] << ", "
                     << gps_meas.position_ned[1] << ", " << gps_meas.position_ned[2] << "]\n";

            auto state_before = ekf.getState();
            std::cout << "  State before update: [" << state_before.position_ned[0] << ", "
                     << state_before.position_ned[1] << ", " << state_before.position_ned[2] << "]\n";

            ekf.updateGPS(gps_meas);

            auto state_after = ekf.getState();
            std::cout << "  State after update:  [" << state_after.position_ned[0] << ", "
                     << state_after.position_ned[1] << ", " << state_after.position_ned[2] << "]\n";

            // Check covariance
            cov = ekf.getCovarianceDiagonal();
            std::cout << "  Covariance (pos): [" << std::sqrt(cov[0]) << ", "
                     << std::sqrt(cov[1]) << ", " << std::sqrt(cov[2]) << "] m\n";

            // Check for divergence
            double pos_mag = std::sqrt(state_after.position_ned[0]*state_after.position_ned[0] +
                                       state_after.position_ned[1]*state_after.position_ned[1] +
                                       state_after.position_ned[2]*state_after.position_ned[2]);

            if (pos_mag > 100.0) {
                std::cout << "\n*** DIVERGENCE DETECTED! Position magnitude: " << pos_mag << " m ***\n";
                std::cout << "This suggests the GPS measurement update is causing instability!\n";
                return 1;
            }

            if (std::sqrt(cov[0]) > 1000.0) {
                std::cout << "\n*** COVARIANCE EXPLOSION! Std dev: " << std::sqrt(cov[0]) << " m ***\n";
                return 1;
            }

            std::cout << "\n";
        }

        // Progress check every second
        if (i % 100 == 99) {
            auto st = ekf.getState();
            std::cout << "t=" << (i+1)*dt << "s (no GPS): Pos=[" << st.position_ned[0] << ", "
                     << st.position_ned[1] << ", " << st.position_ned[2] << "]\n\n";
        }
    }

    std::cout << "✓ TEST 3 PASSED: GPS updates work correctly!\n";
    return 0;
}

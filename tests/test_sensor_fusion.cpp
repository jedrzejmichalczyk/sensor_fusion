/**
 * @file test_sensor_fusion.cpp
 * @brief Integration test for complete sensor fusion system
 */

#include <iostream>
#include <cmath>
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "sensors/barometer.hpp"
#include "fusion/ekf.hpp"

using namespace sensor_fusion;

int main()
{
    std::cout << "Testing complete sensor fusion system...\n";

    const double dt = 0.01;
    const double sim_time = 10.0;

    // Create sensors
    IMU imu(IMUErrorModel(), 1.0/dt);
    GPS gps(GPSErrorModel(), 1.0);
    Barometer baro(BarometerErrorModel(), 10.0);

    // Initialize EKF
    NavigationState initial_state;
    ExtendedKalmanFilter ekf(initial_state);

    std::cout << "\nRunning " << sim_time << "s simulation...\n";

    double next_gps_time = 0.0;
    double next_baro_time = 0.0;

    for (double t = 0.0; t <= sim_time; t += dt) {
        // True state: constant velocity
        std::array<double, 3> true_pos = {5.0 * t, 0.0, 0.0};
        std::array<double, 3> true_vel = {5.0, 0.0, 0.0};
        std::array<double, 3> true_accel = {0.0, 0.0, GRAVITY};
        std::array<double, 3> true_gyro = {0.0, 0.0, 0.0};

        // IMU measurement and prediction
        auto imu_meas = imu.measure(true_accel, true_gyro, t);
        ekf.predict(imu_meas, dt);

        // GPS update
        if (t >= next_gps_time) {
            auto gps_meas = gps.measure(true_pos, true_vel, t);
            ekf.updateGPS(gps_meas);
            next_gps_time += 1.0;
        }

        // Barometer update
        if (t >= next_baro_time) {
            auto baro_meas = baro.measure(-true_pos[2], t);
            ekf.updateBarometer(baro_meas);
            next_baro_time += 0.1;
        }
    }

    // Check final state
    auto final_state = ekf.getState();
    double expected_pos = 5.0 * sim_time;
    double position_error = std::abs(final_state.position_ned[0] - expected_pos);

    std::cout << "  Expected position: " << expected_pos << " m\n";
    std::cout << "  Estimated position: " << final_state.position_ned[0] << " m\n";
    std::cout << "  Error: " << position_error << " m\n";

    bool test_ok = position_error < 5.0;  // Within 5 meters

    std::cout << "\nIntegration test " << (test_ok ? "PASSED" : "FAILED") << "!\n";

    return test_ok ? 0 : 1;
}

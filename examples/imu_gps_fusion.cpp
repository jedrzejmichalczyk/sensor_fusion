/**
 * @file imu_gps_fusion.cpp
 * @brief Simplified IMU/GPS fusion example
 *
 * This example demonstrates basic INS/GPS integration with:
 * - High-rate IMU propagation
 * - Low-rate GPS updates
 * - Comparison of pure INS vs. fused solution
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "fusion/ekf.hpp"
#include "utils/math_utils.hpp"

using namespace sensor_fusion;

// Constants
constexpr double GRAVITY = 9.81;  // m/s^2

// Simple trajectory generator
struct Trajectory {
    double t;
    std::array<double, 3> position;
    std::array<double, 3> velocity;
    std::array<double, 3> acceleration;
};

Trajectory generateTrajectory(double t) {
    Trajectory traj;
    traj.t = t;

    // Constant velocity in North direction
    double velocity = 10.0;  // 10 m/s
    traj.position = {velocity * t, 0.0, 0.0};
    traj.velocity = {velocity, 0.0, 0.0};
    traj.acceleration[0] = 0.0;
    traj.acceleration[1] = 0.0;
    traj.acceleration[2] = GRAVITY;  // Only gravity in body frame

    return traj;
}

int main()
{
    std::cout << "IMU/GPS Sensor Fusion Example\n";
    std::cout << "==============================\n\n";

    const double dt = 0.01;        // 100 Hz IMU
    const double gps_dt = 1.0;     // 1 Hz GPS
    const double sim_time = 30.0;

    // Create sensors
    IMUErrorModel imu_error;
    IMU imu(imu_error, 1.0/dt);

    GPSErrorModel gps_error;
    GPS gps(gps_error, 1.0/gps_dt);

    // Initialize EKF
    NavigationState initial_state;
    ExtendedKalmanFilter ekf(initial_state);

    std::cout << "Running simulation with GPS outage from t=10s to t=20s\n\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time[s]  True_N[m]  EKF_N[m]  Error[m]  GPS_avail\n";
    std::cout << "----------------------------------------------------\n";

    double next_gps_time = 0.0;

    for (double t = 0.0; t <= sim_time; t += dt) {
        // Generate true trajectory
        auto traj = generateTrajectory(t);

        // IMU measurement
        std::array<double, 3> gyro = {0.0, 0.0, 0.0};  // No rotation
        auto imu_meas = imu.measure(traj.acceleration, gyro, t);

        // EKF prediction
        ekf.predict(imu_meas, dt);

        // GPS update (with outage simulation)
        bool gps_available = (t < 10.0 || t > 20.0);
        gps.setAvailability(gps_available);

        if (t >= next_gps_time && gps_available) {
            auto gps_meas = gps.measure(traj.position, traj.velocity, t);
            ekf.updateGPS(gps_meas);
            next_gps_time += gps_dt;
        }

        // Print state every second
        if (std::fmod(t, 1.0) < dt) {
            auto state = ekf.getState();
            double error = std::abs(state.position_ned[0] - traj.position[0]);

            std::cout << std::setw(6) << t << "   "
                     << std::setw(9) << traj.position[0] << "  "
                     << std::setw(8) << state.position_ned[0] << "  "
                     << std::setw(8) << error << "  "
                     << (gps_available ? "  YES" : "  NO") << "\n";
        }
    }

    std::cout << "\nSimulation complete!\n";
    std::cout << "\nNote: Error grows during GPS outage (10-20s) and is bounded when GPS is available.\n";

    return 0;
}

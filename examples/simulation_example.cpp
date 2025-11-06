/**
 * @file simulation_example.cpp
 * @brief Complete sensor fusion simulation example
 *
 * This example demonstrates:
 * - IMU sensor simulation with errors
 * - GPS measurements
 * - Barometric altitude
 * - Magnetometer readings
 * - Extended Kalman Filter fusion
 */

#include <iostream>
#include <iomanip>
#include <cmath>

#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "sensors/barometer.hpp"
#include "sensors/magnetometer.hpp"
#include "fusion/ekf.hpp"
#include "utils/math_utils.hpp"

using namespace sensor_fusion;

// Constants
constexpr double GRAVITY = 9.81;  // m/s^2

int main()
{
    std::cout << "Sensor Fusion Navigation System - Simulation Example\n";
    std::cout << "=====================================================\n\n";

    // Simulation parameters
    const double dt = 0.01;           // 100 Hz IMU rate
    const double sim_time = 60.0;     // 60 seconds simulation
    const double gps_rate = 1.0;      // 1 Hz GPS
    const double baro_rate = 10.0;    // 10 Hz barometer
    const double mag_rate = 10.0;     // 10 Hz magnetometer

    // Create sensors
    IMUErrorModel imu_error;
    IMU imu(imu_error, 1.0/dt, 42);

    GPSErrorModel gps_error;
    GPS gps(gps_error, gps_rate, 43);

    BarometerErrorModel baro_error;
    Barometer baro(baro_error, baro_rate, 44);

    MagnetometerErrorModel mag_error;
    Magnetometer mag(mag_error, mag_rate, 45);

    // Initialize EKF with initial state
    NavigationState initial_state;
    initial_state.position_ned = {0.0, 0.0, 0.0};
    initial_state.velocity_ned = {0.0, 0.0, 0.0};
    initial_state.setFromEulerAngles(0.0, 0.0, 0.0);  // Level attitude, north heading

    ExtendedKalmanFilter ekf(initial_state);

    // Set noise parameters
    ekf.setGPSNoise(2.5, 0.1);
    ekf.setBarometerNoise(1.0);
    ekf.setMagnetometerNoise(5.0);

    // Magnetic field at latitude 45°N
    auto mag_field_ned = Magnetometer::getLocalMagneticField(45.0);

    std::cout << "Simulation Configuration:\n";
    std::cout << "  Duration: " << sim_time << " s\n";
    std::cout << "  IMU rate: " << 1.0/dt << " Hz\n";
    std::cout << "  GPS rate: " << gps_rate << " Hz\n";
    std::cout << "  Barometer rate: " << baro_rate << " Hz\n";
    std::cout << "  Magnetometer rate: " << mag_rate << " Hz\n\n";

    // Simulation loop
    std::cout << "Starting simulation...\n\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time[s]  Pos_N[m]  Pos_E[m]  Pos_D[m]  Vel_N[m/s]  Roll[deg]  Pitch[deg]  Yaw[deg]\n";
    std::cout << "------------------------------------------------------------------------------------\n";

    int gps_counter = 0;
    int baro_counter = 0;
    int mag_counter = 0;

    for (double t = 0.0; t <= sim_time; t += dt) {
        // True trajectory: circular motion
        double omega = 2.0 * M_PI / 30.0;  // 30 second period
        double radius = 50.0;               // 50 meter radius

        // True position (circular path)
        std::array<double, 3> true_pos = {
            radius * std::sin(omega * t),
            radius * (1.0 - std::cos(omega * t)),
            0.0  // Constant altitude
        };

        // True velocity
        std::array<double, 3> true_vel = {
            radius * omega * std::cos(omega * t),
            radius * omega * std::sin(omega * t),
            0.0
        };

        // True acceleration in NED
        std::array<double, 3> true_accel_ned = {
            -radius * omega * omega * std::sin(omega * t),
            -radius * omega * omega * (1.0 - std::cos(omega * t)),
            0.0
        };

        // For simplicity, assume level flight (could compute proper attitude from acceleration)
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = omega * t;

        // Transform acceleration to body frame (simplified - assume small angles)
        std::array<double, 3> true_accel_body = {
            true_accel_ned[0],
            true_accel_ned[1],
            true_accel_ned[2] + GRAVITY  // Add gravity
        };

        // True angular velocity in body frame
        std::array<double, 3> true_gyro = {0.0, 0.0, omega};

        // IMU measurement (always available at high rate)
        auto imu_meas = imu.measure(true_accel_body, true_gyro, t);

        // EKF prediction step
        ekf.predict(imu_meas, dt);

        // GPS update (low rate)
        if (++gps_counter >= (int)(1.0 / (gps_rate * dt))) {
            gps_counter = 0;
            auto gps_meas = gps.measure(true_pos, true_vel, t);
            ekf.updateGPS(gps_meas);
        }

        // Barometer update
        if (++baro_counter >= (int)(1.0 / (baro_rate * dt))) {
            baro_counter = 0;
            auto baro_meas = baro.measure(-true_pos[2], t);  // Altitude = -Down
            ekf.updateBarometer(baro_meas);
        }

        // Magnetometer update
        if (++mag_counter >= (int)(1.0 / (mag_rate * dt))) {
            mag_counter = 0;

            // Transform magnetic field to body frame
            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);
            std::array<double, 3> mag_body = {
                cos_yaw * mag_field_ned[0] + sin_yaw * mag_field_ned[1],
                -sin_yaw * mag_field_ned[0] + cos_yaw * mag_field_ned[1],
                mag_field_ned[2]
            };

            auto mag_meas = mag.measure(mag_body, t);
            ekf.updateMagnetometer(mag_meas, mag_field_ned);
        }

        // Print state every 5 seconds
        if (std::fmod(t, 5.0) < dt) {
            auto state = ekf.getState();
            auto euler = state.getEulerAngles();

            std::cout << std::setw(6) << t << "  "
                     << std::setw(8) << state.position_ned[0] << "  "
                     << std::setw(8) << state.position_ned[1] << "  "
                     << std::setw(8) << state.position_ned[2] << "  "
                     << std::setw(10) << state.velocity_ned[0] << "  "
                     << std::setw(9) << MathUtils::rad2deg(euler[0]) << "  "
                     << std::setw(10) << MathUtils::rad2deg(euler[1]) << "  "
                     << std::setw(8) << MathUtils::rad2deg(euler[2]) << "\n";
        }
    }

    std::cout << "\nSimulation complete!\n";
    std::cout << "\nFinal State:\n";
    auto final_state = ekf.getState();
    auto final_euler = final_state.getEulerAngles();

    std::cout << "  Position NED: ["
             << final_state.position_ned[0] << ", "
             << final_state.position_ned[1] << ", "
             << final_state.position_ned[2] << "] m\n";
    std::cout << "  Velocity NED: ["
             << final_state.velocity_ned[0] << ", "
             << final_state.velocity_ned[1] << ", "
             << final_state.velocity_ned[2] << "] m/s\n";
    std::cout << "  Euler angles: ["
             << MathUtils::rad2deg(final_euler[0]) << ", "
             << MathUtils::rad2deg(final_euler[1]) << ", "
             << MathUtils::rad2deg(final_euler[2]) << "] deg\n";

    auto cov_diag = ekf.getCovarianceDiagonal();
    std::cout << "\nPosition uncertainty (std): ["
             << std::sqrt(cov_diag[0]) << ", "
             << std::sqrt(cov_diag[1]) << ", "
             << std::sqrt(cov_diag[2]) << "] m\n";

    return 0;
}

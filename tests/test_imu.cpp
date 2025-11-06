/**
 * @file test_imu.cpp
 * @brief Test IMU sensor model
 */

#include <iostream>
#include <cmath>
#include "sensors/imu.hpp"

using namespace sensor_fusion;

int main()
{
    std::cout << "Testing IMU sensor model...\n";

    IMUErrorModel error_model;
    IMU imu(error_model, 100.0);  // 100 Hz

    // Test 1: Zero input should produce only bias and noise
    std::cout << "\nTest 1: Zero input (stationary)\n";
    std::array<double, 3> zero = {0.0, 0.0, 0.0};

    for (int i = 0; i < 5; ++i) {
        auto meas = imu.measure(zero, zero, i * 0.01);
        std::cout << "  Accel: [" << meas.accel[0] << ", "
                 << meas.accel[1] << ", " << meas.accel[2] << "]\n";
    }

    // Test 2: Constant input
    std::cout << "\nTest 2: Constant acceleration input\n";
    std::array<double, 3> accel = {1.0, 0.0, 0.0};
    std::array<double, 3> gyro = {0.0, 0.0, 0.1};

    for (int i = 0; i < 5; ++i) {
        auto meas = imu.measure(accel, gyro, i * 0.01);
        std::cout << "  Gyro: [" << meas.gyro[0] << ", "
                 << meas.gyro[1] << ", " << meas.gyro[2] << "]\n";
    }

    // Test 3: Check bias evolution
    std::cout << "\nTest 3: Bias evolution over time\n";
    auto bias_initial = imu.getGyroBias();
    std::cout << "  Initial gyro bias: [" << bias_initial[0] << ", "
             << bias_initial[1] << ", " << bias_initial[2] << "]\n";

    for (int i = 0; i < 1000; ++i) {
        imu.measure(zero, zero, i * 0.01);
    }

    auto bias_after = imu.getGyroBias();
    std::cout << "  After 10s gyro bias: [" << bias_after[0] << ", "
             << bias_after[1] << ", " << bias_after[2] << "]\n";

    // Test 4: Calibration
    std::cout << "\nTest 4: Calibration\n";
    imu.calibrate();
    auto bias_calibrated = imu.getGyroBias();
    std::cout << "  Calibrated gyro bias: [" << bias_calibrated[0] << ", "
             << bias_calibrated[1] << ", " << bias_calibrated[2] << "]\n";

    bool calibration_ok = (std::abs(bias_calibrated[0]) < 1e-9 &&
                          std::abs(bias_calibrated[1]) < 1e-9 &&
                          std::abs(bias_calibrated[2]) < 1e-9);

    std::cout << "\nAll tests " << (calibration_ok ? "PASSED" : "FAILED") << "!\n";

    return calibration_ok ? 0 : 1;
}

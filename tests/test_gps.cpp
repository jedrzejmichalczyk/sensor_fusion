/**
 * @file test_gps.cpp
 * @brief Test GPS sensor model
 */

#include <iostream>
#include <cmath>
#include "sensors/gps.hpp"

using namespace sensor_fusion;

int main()
{
    std::cout << "Testing GPS sensor model...\n";

    GPSErrorModel error_model;
    GPS gps(error_model, 1.0);  // 1 Hz

    // Test 1: Fixed position measurement
    std::cout << "\nTest 1: Fixed position measurement\n";
    std::array<double, 3> true_pos = {100.0, 200.0, -50.0};
    std::array<double, 3> true_vel = {5.0, 0.0, 0.0};

    double sum_pos_error = 0.0;
    int n_samples = 100;

    for (int i = 0; i < n_samples; ++i) {
        auto meas = gps.measure(true_pos, true_vel, i);
        double error = std::sqrt(
            std::pow(meas.position_ned[0] - true_pos[0], 2) +
            std::pow(meas.position_ned[1] - true_pos[1], 2) +
            std::pow(meas.position_ned[2] - true_pos[2], 2)
        );
        sum_pos_error += error;
    }

    double mean_error = sum_pos_error / n_samples;
    std::cout << "  Mean position error: " << mean_error << " m\n";
    std::cout << "  Expected: ~" << std::sqrt(2*2.5*2.5 + 5.0*5.0) << " m (RMS)\n";

    // Test 2: GPS availability
    std::cout << "\nTest 2: GPS availability control\n";
    gps.setAvailability(false);
    auto meas_unavail = gps.measure(true_pos, true_vel, 0.0);
    std::cout << "  GPS available: " << (meas_unavail.valid ? "YES" : "NO") << "\n";

    gps.setAvailability(true);
    auto meas_avail = gps.measure(true_pos, true_vel, 0.0);
    std::cout << "  GPS available: " << (meas_avail.valid ? "YES" : "NO") << "\n";

    bool test_ok = !meas_unavail.valid && meas_avail.valid;

    std::cout << "\nAll tests " << (test_ok ? "PASSED" : "FAILED") << "!\n";

    return test_ok ? 0 : 1;
}

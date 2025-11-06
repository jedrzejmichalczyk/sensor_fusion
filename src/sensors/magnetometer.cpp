#include "sensors/magnetometer.hpp"
#include <cmath>
#include <algorithm>

namespace sensor_fusion {

Magnetometer::Magnetometer(const MagnetometerErrorModel& error_model, double update_rate, unsigned int seed)
    : error_model_(error_model),
      update_rate_(update_rate),
      rng_(seed),
      noise_dist_(0.0, error_model.noise_density * std::sqrt(update_rate))
{
    // Initialize hard iron bias
    std::normal_distribution<double> bias_dist(0.0, error_model.hard_iron_bias);
    for (int i = 0; i < 3; ++i) {
        hard_iron_bias_[i] = bias_dist(rng_);
    }
}

MagnetometerMeasurement Magnetometer::measure(const std::array<double, 3>& true_mag_field_body,
                                             double timestamp)
{
    MagnetometerMeasurement meas;
    meas.timestamp = timestamp;

    // Apply error model
    for (int i = 0; i < 3; ++i) {
        double scale_error = 1.0 + error_model_.soft_iron_scale_error * (2.0 * (double)rand() / RAND_MAX - 1.0);
        meas.magnetic_field[i] = scale_error * true_mag_field_body[i] + hard_iron_bias_[i] + noise_dist_(rng_);
    }

    return meas;
}

void Magnetometer::calibrate()
{
    std::fill(hard_iron_bias_.begin(), hard_iron_bias_.end(), 0.0);
}

std::array<double, 3> Magnetometer::getLocalMagneticField(double latitude, double longitude)
{
    // Simplified Earth magnetic field model
    // Uses dipole approximation
    const double field_magnitude = 50.0;  // uT, typical mid-latitude value

    // Convert latitude to radians
    double lat_rad = latitude * M_PI / 180.0;

    // Magnetic inclination (dip angle) varies with latitude
    // I ≈ arctan(2 * tan(latitude)) for dipole model
    double inclination = std::atan(2.0 * std::tan(lat_rad));

    // Magnetic declination (variation) - simplified, set to 0
    double declination = 0.0;

    // Field components in NED frame
    std::array<double, 3> mag_field_ned;
    mag_field_ned[0] = field_magnitude * std::cos(inclination) * std::cos(declination);  // North
    mag_field_ned[1] = field_magnitude * std::cos(inclination) * std::sin(declination);  // East
    mag_field_ned[2] = field_magnitude * std::sin(inclination);                          // Down

    return mag_field_ned;
}

} // namespace sensor_fusion

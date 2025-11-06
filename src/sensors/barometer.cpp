#include "sensors/barometer.hpp"
#include <cmath>

namespace sensor_fusion {

Barometer::Barometer(const BarometerErrorModel& error_model, double update_rate, unsigned int seed)
    : error_model_(error_model),
      update_rate_(update_rate),
      reference_pressure_(101325.0),  // Standard sea level pressure
      rng_(seed),
      altitude_noise_dist_(0.0, error_model.altitude_noise_std),
      pressure_noise_dist_(0.0, error_model.pressure_noise_std)
{
}

BarometerMeasurement Barometer::measure(double true_altitude, double timestamp, double temperature)
{
    BarometerMeasurement meas;
    meas.timestamp = timestamp;
    meas.temperature = temperature;

    // Convert true altitude to pressure using ISA model
    double true_pressure = altitudeToPressure(true_altitude, reference_pressure_);

    // Add noise to pressure
    meas.pressure = true_pressure + pressure_noise_dist_(rng_);

    // Convert noisy pressure back to altitude
    meas.altitude = pressureToAltitude(meas.pressure, reference_pressure_) + altitude_noise_dist_(rng_);

    return meas;
}

double Barometer::pressureToAltitude(double pressure, double reference_pressure)
{
    // International Standard Atmosphere (ISA) model
    // h = 44330 * (1 - (P/P0)^0.1903)
    const double exponent = 0.190284;  // More precise value
    return 44330.0 * (1.0 - std::pow(pressure / reference_pressure, exponent));
}

double Barometer::altitudeToPressure(double altitude, double reference_pressure)
{
    // Inverse of ISA model: P = P0 * (1 - h/44330)^(1/0.1903)
    const double exponent = 5.25588;  // 1/0.190284
    return reference_pressure * std::pow(1.0 - altitude / 44330.0, exponent);
}

} // namespace sensor_fusion

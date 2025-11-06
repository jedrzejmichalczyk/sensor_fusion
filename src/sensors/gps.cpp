#include "sensors/gps.hpp"

namespace sensor_fusion {

GPS::GPS(const GPSErrorModel& error_model, double update_rate, unsigned int seed)
    : error_model_(error_model),
      update_rate_(update_rate),
      available_(true),
      num_satellites_(12),
      rng_(seed),
      horizontal_noise_dist_(0.0, error_model.horizontal_position_std),
      vertical_noise_dist_(0.0, error_model.vertical_position_std),
      velocity_noise_dist_(0.0, error_model.velocity_std)
{
}

GPSMeasurement GPS::measure(const std::array<double, 3>& true_position_ned,
                           const std::array<double, 3>& true_velocity_ned,
                           double timestamp)
{
    GPSMeasurement meas;
    meas.timestamp = timestamp;
    meas.valid = available_;
    meas.num_satellites = available_ ? num_satellites_ : 0;

    if (available_) {
        // Add noise to position (different std for horizontal and vertical)
        meas.position_ned[0] = true_position_ned[0] + horizontal_noise_dist_(rng_); // North
        meas.position_ned[1] = true_position_ned[1] + horizontal_noise_dist_(rng_); // East
        meas.position_ned[2] = true_position_ned[2] + vertical_noise_dist_(rng_);   // Down

        // Add noise to velocity (same std for all axes)
        for (int i = 0; i < 3; ++i) {
            meas.velocity_ned[i] = true_velocity_ned[i] + velocity_noise_dist_(rng_);
        }
    } else {
        // No GPS fix, return zeros
        meas.position_ned = {0.0, 0.0, 0.0};
        meas.velocity_ned = {0.0, 0.0, 0.0};
    }

    return meas;
}

} // namespace sensor_fusion

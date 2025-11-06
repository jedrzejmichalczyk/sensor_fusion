#include "sensors/imu.hpp"
#include <algorithm>

namespace sensor_fusion {

IMU::IMU(const IMUErrorModel& error_model, double sample_rate, unsigned int seed)
    : error_model_(error_model),
      sample_rate_(sample_rate),
      dt_(1.0 / sample_rate),
      rng_(seed),
      accel_noise_dist_(0.0, error_model.accel_noise_density * std::sqrt(sample_rate)),
      gyro_noise_dist_(0.0, error_model.gyro_noise_density * std::sqrt(sample_rate)),
      accel_bias_noise_dist_(0.0, error_model.accel_bias_stability * std::sqrt(2.0 / error_model.accel_bias_correlation_time)),
      gyro_bias_noise_dist_(0.0, error_model.gyro_bias_stability * std::sqrt(2.0 / error_model.gyro_bias_correlation_time))
{
    // Initialize biases to small random values
    for (int i = 0; i < 3; ++i) {
        accel_bias_[i] = accel_bias_noise_dist_(rng_);
        gyro_bias_[i] = gyro_bias_noise_dist_(rng_);
    }
}

IMUMeasurement IMU::measure(const std::array<double, 3>& true_accel,
                           const std::array<double, 3>& true_gyro,
                           double timestamp)
{
    // Update bias states (Gauss-Markov process)
    updateBias();

    IMUMeasurement meas;
    meas.timestamp = timestamp;

    // Apply error model to accelerometer
    for (int i = 0; i < 3; ++i) {
        double scale_error = 1.0 + error_model_.accel_scale_factor_error * (2.0 * (double)rand() / RAND_MAX - 1.0);
        meas.accel[i] = scale_error * true_accel[i] + accel_bias_[i] + accel_noise_dist_(rng_);
    }

    // Apply error model to gyroscope
    for (int i = 0; i < 3; ++i) {
        double scale_error = 1.0 + error_model_.gyro_scale_factor_error * (2.0 * (double)rand() / RAND_MAX - 1.0);
        meas.gyro[i] = scale_error * true_gyro[i] + gyro_bias_[i] + gyro_noise_dist_(rng_);
    }

    return meas;
}

void IMU::updateBias()
{
    // First-order Gauss-Markov process: b(k+1) = exp(-dt/tau) * b(k) + w
    double accel_beta = std::exp(-dt_ / error_model_.accel_bias_correlation_time);
    double gyro_beta = std::exp(-dt_ / error_model_.gyro_bias_correlation_time);

    for (int i = 0; i < 3; ++i) {
        accel_bias_[i] = accel_beta * accel_bias_[i] + accel_bias_noise_dist_(rng_) * std::sqrt(dt_);
        gyro_bias_[i] = gyro_beta * gyro_bias_[i] + gyro_bias_noise_dist_(rng_) * std::sqrt(dt_);
    }
}

void IMU::calibrate()
{
    std::fill(accel_bias_.begin(), accel_bias_.end(), 0.0);
    std::fill(gyro_bias_.begin(), gyro_bias_.end(), 0.0);
}

} // namespace sensor_fusion

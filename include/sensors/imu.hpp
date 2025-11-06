#ifndef SENSOR_FUSION_IMU_HPP
#define SENSOR_FUSION_IMU_HPP

#include <array>
#include <random>
#include <cmath>

namespace sensor_fusion {

/**
 * @brief IMU error model parameters
 *
 * These parameters define the error characteristics of a MEMS IMU
 * following the models described in literature (Aggarwal et al., 2013)
 */
struct IMUErrorModel {
    // Accelerometer parameters
    double accel_noise_density;      // m/s^2/sqrt(Hz)
    double accel_bias_stability;     // m/s^2
    double accel_bias_correlation_time; // seconds
    double accel_scale_factor_error; // dimensionless (fraction)

    // Gyroscope parameters
    double gyro_noise_density;       // rad/s/sqrt(Hz)
    double gyro_bias_stability;      // rad/s
    double gyro_bias_correlation_time; // seconds
    double gyro_scale_factor_error;  // dimensionless (fraction)

    // Default constructor with typical consumer-grade MEMS values
    IMUErrorModel() :
        accel_noise_density(150e-6 * 9.81),  // 150 ug/sqrt(Hz)
        accel_bias_stability(40e-6 * 9.81),   // 40 ug
        accel_bias_correlation_time(3600.0),  // 1 hour
        accel_scale_factor_error(0.005),      // 0.5%
        gyro_noise_density(0.01 * M_PI / 180.0), // 0.01 deg/s/sqrt(Hz)
        gyro_bias_stability(10.0 * M_PI / 180.0 / 3600.0), // 10 deg/h
        gyro_bias_correlation_time(3600.0),   // 1 hour
        gyro_scale_factor_error(0.005)        // 0.5%
    {}
};

/**
 * @brief IMU measurement data
 */
struct IMUMeasurement {
    double timestamp;              // seconds
    std::array<double, 3> accel;   // specific force [m/s^2] in body frame
    std::array<double, 3> gyro;    // angular velocity [rad/s] in body frame
};

/**
 * @brief Inertial Measurement Unit simulator
 *
 * Simulates a 6-DOF IMU with realistic error models including:
 * - White noise
 * - Bias instability (first-order Gauss-Markov process)
 * - Scale factor errors
 */
class IMU {
public:
    /**
     * @brief Construct IMU with specified error model
     * @param error_model Error parameters
     * @param sample_rate IMU output rate in Hz
     * @param seed Random seed for noise generation
     */
    IMU(const IMUErrorModel& error_model, double sample_rate, unsigned int seed = 0);

    /**
     * @brief Generate IMU measurement from true specific force and angular velocity
     * @param true_accel True specific force in body frame [m/s^2]
     * @param true_gyro True angular velocity in body frame [rad/s]
     * @param timestamp Current time [s]
     * @return Corrupted IMU measurement
     */
    IMUMeasurement measure(const std::array<double, 3>& true_accel,
                          const std::array<double, 3>& true_gyro,
                          double timestamp);

    /**
     * @brief Get current accelerometer bias
     */
    std::array<double, 3> getAccelBias() const { return accel_bias_; }

    /**
     * @brief Get current gyroscope bias
     */
    std::array<double, 3> getGyroBias() const { return gyro_bias_; }

    /**
     * @brief Reset biases to zero (simulates calibration)
     */
    void calibrate();

private:
    IMUErrorModel error_model_;
    double sample_rate_;
    double dt_;  // Time step

    // Current bias states (evolve according to Gauss-Markov process)
    std::array<double, 3> accel_bias_;
    std::array<double, 3> gyro_bias_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> accel_noise_dist_;
    std::normal_distribution<double> gyro_noise_dist_;
    std::normal_distribution<double> accel_bias_noise_dist_;
    std::normal_distribution<double> gyro_bias_noise_dist_;

    /**
     * @brief Update bias state using first-order Gauss-Markov process
     */
    void updateBias();
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_IMU_HPP

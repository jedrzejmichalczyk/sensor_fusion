#ifndef SENSOR_FUSION_MAGNETOMETER_HPP
#define SENSOR_FUSION_MAGNETOMETER_HPP

#include <array>
#include <random>
#include <cmath>

namespace sensor_fusion {

/**
 * @brief Magnetometer error model parameters
 */
struct MagnetometerErrorModel {
    double noise_density;           // uT/sqrt(Hz)
    double hard_iron_bias;          // uT (constant offset)
    double soft_iron_scale_error;   // dimensionless (scale factor error)

    // Default values for typical 3-axis magnetometer
    MagnetometerErrorModel() :
        noise_density(0.3),          // 0.3 uT/sqrt(Hz)
        hard_iron_bias(5.0),         // 5 uT
        soft_iron_scale_error(0.01)  // 1%
    {}
};

/**
 * @brief Magnetometer measurement data
 */
struct MagnetometerMeasurement {
    double timestamp;                    // seconds
    std::array<double, 3> magnetic_field; // [Bx, By, Bz] in body frame [uT]
};

/**
 * @brief 3-axis magnetometer simulator
 *
 * Simulates a magnetometer measuring Earth's magnetic field with errors:
 * - White Gaussian noise
 * - Hard iron bias (constant offset from ferromagnetic materials)
 * - Soft iron errors (scale factor variations)
 * - Non-orthogonality (not yet implemented)
 *
 * The Earth's magnetic field model is simplified (no variation with position).
 */
class Magnetometer {
public:
    /**
     * @brief Construct magnetometer with specified error model
     * @param error_model Error parameters
     * @param update_rate Magnetometer output rate in Hz
     * @param seed Random seed for noise generation
     */
    Magnetometer(const MagnetometerErrorModel& error_model, double update_rate, unsigned int seed = 0);

    /**
     * @brief Generate magnetometer measurement from true magnetic field
     * @param true_mag_field_body True magnetic field in body frame [uT]
     * @param timestamp Current time [s]
     * @return Magnetometer measurement with errors
     */
    MagnetometerMeasurement measure(const std::array<double, 3>& true_mag_field_body,
                                   double timestamp);

    /**
     * @brief Calibrate magnetometer (estimate and remove hard iron bias)
     */
    void calibrate();

    /**
     * @brief Get current hard iron bias estimate
     */
    std::array<double, 3> getHardIronBias() const { return hard_iron_bias_; }

    /**
     * @brief Set local magnetic field vector in NED frame
     * @param latitude Latitude in degrees
     * @param longitude Longitude in degrees (not used in simplified model)
     *
     * Simplified model: magnitude ~50 uT, inclination varies with latitude
     */
    static std::array<double, 3> getLocalMagneticField(double latitude, double longitude = 0.0);

private:
    MagnetometerErrorModel error_model_;
    double update_rate_;

    // Hard iron bias (simulates constant offset)
    std::array<double, 3> hard_iron_bias_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_MAGNETOMETER_HPP

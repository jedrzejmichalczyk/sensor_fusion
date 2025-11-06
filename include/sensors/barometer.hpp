#ifndef SENSOR_FUSION_BAROMETER_HPP
#define SENSOR_FUSION_BAROMETER_HPP

#include <random>

namespace sensor_fusion {

/**
 * @brief Barometric altimeter error model parameters
 */
struct BarometerErrorModel {
    double altitude_noise_std;   // meters (1-sigma)
    double pressure_noise_std;   // Pascals (1-sigma)

    // Default values for typical barometric altimeter
    BarometerErrorModel() :
        altitude_noise_std(1.0),
        pressure_noise_std(10.0)
    {}
};

/**
 * @brief Barometer measurement data
 */
struct BarometerMeasurement {
    double timestamp;    // seconds
    double altitude;     // meters above reference (typically sea level)
    double pressure;     // Pascals
    double temperature;  // Celsius (optional, for compensation)
};

/**
 * @brief Barometric altimeter simulator
 *
 * Simulates barometric pressure sensor with conversion to altitude.
 * Uses the International Standard Atmosphere (ISA) model:
 * h = 44330 * (1 - (P/P0)^0.1903)
 *
 * Error sources include:
 * - White Gaussian noise in pressure measurement
 * - Slow-varying bias due to weather changes (not yet implemented)
 */
class Barometer {
public:
    /**
     * @brief Construct barometer with specified error model
     * @param error_model Error parameters
     * @param update_rate Barometer output rate in Hz
     * @param seed Random seed for noise generation
     */
    Barometer(const BarometerErrorModel& error_model, double update_rate, unsigned int seed = 0);

    /**
     * @brief Generate barometer measurement from true altitude
     * @param true_altitude True altitude above reference [m]
     * @param timestamp Current time [s]
     * @param temperature Ambient temperature [°C] (default 15°C at sea level)
     * @return Barometer measurement with errors
     */
    BarometerMeasurement measure(double true_altitude, double timestamp, double temperature = 15.0);

    /**
     * @brief Set reference pressure (for relative altitude measurements)
     * @param pressure_pa Reference pressure in Pascals
     */
    void setReferencePressure(double pressure_pa) { reference_pressure_ = pressure_pa; }

    /**
     * @brief Convert pressure to altitude using ISA model
     * @param pressure Measured pressure [Pa]
     * @return Altitude [m]
     */
    static double pressureToAltitude(double pressure, double reference_pressure = 101325.0);

    /**
     * @brief Convert altitude to pressure using ISA model
     * @param altitude Altitude [m]
     * @return Pressure [Pa]
     */
    static double altitudeToPressure(double altitude, double reference_pressure = 101325.0);

private:
    BarometerErrorModel error_model_;
    double update_rate_;
    double reference_pressure_;  // Pa, typically 101325 (sea level)

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> altitude_noise_dist_;
    std::normal_distribution<double> pressure_noise_dist_;
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_BAROMETER_HPP

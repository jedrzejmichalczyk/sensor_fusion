#ifndef SENSOR_FUSION_GPS_HPP
#define SENSOR_FUSION_GPS_HPP

#include <array>
#include <random>

namespace sensor_fusion {

/**
 * @brief GPS receiver error model parameters
 */
struct GPSErrorModel {
    double horizontal_position_std;  // meters (1-sigma)
    double vertical_position_std;    // meters (1-sigma)
    double velocity_std;             // m/s (1-sigma)

    // Default values for typical consumer GPS
    GPSErrorModel() :
        horizontal_position_std(2.5),
        vertical_position_std(5.0),
        velocity_std(0.1)
    {}
};

/**
 * @brief GPS measurement data
 */
struct GPSMeasurement {
    double timestamp;                   // seconds
    std::array<double, 3> position_ned; // [N, E, D] in meters
    std::array<double, 3> velocity_ned; // [vN, vE, vD] in m/s
    bool valid;                         // GPS fix availability
    int num_satellites;                 // Number of satellites used
};

/**
 * @brief GPS receiver simulator
 *
 * Simulates GPS measurements with realistic errors:
 * - White Gaussian noise in position and velocity
 * - Intermittent loss of fix
 * - Position-dependent errors (dilution of precision effects could be added)
 */
class GPS {
public:
    /**
     * @brief Construct GPS with specified error model
     * @param error_model Error parameters
     * @param update_rate GPS output rate in Hz
     * @param seed Random seed for noise generation
     */
    GPS(const GPSErrorModel& error_model, double update_rate, unsigned int seed = 0);

    /**
     * @brief Generate GPS measurement from true position and velocity
     * @param true_position_ned True position in NED frame [m]
     * @param true_velocity_ned True velocity in NED frame [m/s]
     * @param timestamp Current time [s]
     * @return GPS measurement with errors
     */
    GPSMeasurement measure(const std::array<double, 3>& true_position_ned,
                          const std::array<double, 3>& true_velocity_ned,
                          double timestamp);

    /**
     * @brief Set GPS availability (simulate outages)
     * @param available GPS fix availability
     */
    void setAvailability(bool available) { available_ = available; }

    /**
     * @brief Get current GPS availability
     */
    bool isAvailable() const { return available_; }

private:
    GPSErrorModel error_model_;
    double update_rate_;
    bool available_;
    int num_satellites_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> horizontal_noise_dist_;
    std::normal_distribution<double> vertical_noise_dist_;
    std::normal_distribution<double> velocity_noise_dist_;
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_GPS_HPP

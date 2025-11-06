#ifndef SENSOR_FUSION_DATA_LOGGER_HPP
#define SENSOR_FUSION_DATA_LOGGER_HPP

#include <string>
#include <fstream>
#include <vector>
#include "fusion/state.hpp"
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"

namespace sensor_fusion {

/**
 * @brief Data logger for sensor fusion results
 *
 * Logs state estimates, measurements, and covariances to CSV file
 * for post-processing and visualization
 */
class DataLogger {
public:
    /**
     * @brief Construct data logger with output filename
     * @param filename CSV output file path
     */
    explicit DataLogger(const std::string& filename);

    /**
     * @brief Destructor closes file
     */
    ~DataLogger();

    /**
     * @brief Log timestamped state estimate
     * @param timestamp Time [s]
     * @param state Navigation state
     * @param cov_diag Covariance diagonal (optional)
     */
    void logState(double timestamp,
                  const NavigationState& state,
                  const std::array<double, ERROR_STATE_SIZE>* cov_diag = nullptr);

    /**
     * @brief Log IMU measurement
     * @param meas IMU measurement
     */
    void logIMU(const IMUMeasurement& meas);

    /**
     * @brief Log GPS measurement
     * @param meas GPS measurement
     */
    void logGPS(const GPSMeasurement& meas);

    /**
     * @brief Check if logger is open and ready
     */
    bool isOpen() const { return file_.is_open(); }

private:
    std::ofstream file_;
    bool header_written_;

    void writeHeader();
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_DATA_LOGGER_HPP

#include "utils/data_logger.hpp"
#include "utils/math_utils.hpp"
#include <iomanip>

namespace sensor_fusion {

DataLogger::DataLogger(const std::string& filename)
    : header_written_(false)
{
    file_.open(filename);
    if (file_.is_open()) {
        writeHeader();
    }
}

DataLogger::~DataLogger()
{
    if (file_.is_open()) {
        file_.close();
    }
}

void DataLogger::writeHeader()
{
    if (!file_.is_open() || header_written_) return;

    file_ << "timestamp,";
    file_ << "pos_n,pos_e,pos_d,";
    file_ << "vel_n,vel_e,vel_d,";
    file_ << "roll,pitch,yaw,";
    file_ << "acc_bias_x,acc_bias_y,acc_bias_z,";
    file_ << "gyro_bias_x,gyro_bias_y,gyro_bias_z,";
    file_ << "std_pos_n,std_pos_e,std_pos_d,";
    file_ << "std_vel_n,std_vel_e,std_vel_d,";
    file_ << "std_att_x,std_att_y,std_att_z\n";

    header_written_ = true;
}

void DataLogger::logState(double timestamp,
                          const NavigationState& state,
                          const std::array<double, ERROR_STATE_SIZE>* cov_diag)
{
    if (!file_.is_open()) return;

    auto euler = state.getEulerAngles();

    file_ << std::fixed << std::setprecision(6);
    file_ << timestamp << ",";

    // Position
    file_ << state.position_ned[0] << ","
         << state.position_ned[1] << ","
         << state.position_ned[2] << ",";

    // Velocity
    file_ << state.velocity_ned[0] << ","
         << state.velocity_ned[1] << ","
         << state.velocity_ned[2] << ",";

    // Attitude (Euler angles in degrees)
    file_ << MathUtils::rad2deg(euler[0]) << ","
         << MathUtils::rad2deg(euler[1]) << ","
         << MathUtils::rad2deg(euler[2]) << ",";

    // Biases
    file_ << state.accel_bias[0] << ","
         << state.accel_bias[1] << ","
         << state.accel_bias[2] << ",";
    file_ << state.gyro_bias[0] << ","
         << state.gyro_bias[1] << ","
         << state.gyro_bias[2];

    // Covariance diagonal (standard deviations)
    if (cov_diag) {
        for (int i = 0; i < 9; ++i) {  // Only pos, vel, att
            file_ << "," << std::sqrt((*cov_diag)[i]);
        }
    } else {
        for (int i = 0; i < 9; ++i) {
            file_ << ",0.0";
        }
    }

    file_ << "\n";
}

void DataLogger::logIMU(const IMUMeasurement& meas)
{
    // Could log to separate IMU CSV file if needed
    // For now, just a placeholder
}

void DataLogger::logGPS(const GPSMeasurement& meas)
{
    // Could log to separate GPS CSV file if needed
    // For now, just a placeholder
}

} // namespace sensor_fusion

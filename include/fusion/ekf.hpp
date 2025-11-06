#ifndef SENSOR_FUSION_EKF_HPP
#define SENSOR_FUSION_EKF_HPP

#include "fusion/state.hpp"
#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "sensors/barometer.hpp"
#include "sensors/magnetometer.hpp"
#include <array>

namespace sensor_fusion {

/**
 * @brief Extended Kalman Filter for INS/GPS/Baro/Mag fusion
 *
 * Implements error-state EKF formulation following the ESKF approach.
 * This is more numerically stable than direct state estimation.
 *
 * State vector (15D error state):
 * - Position error (3)
 * - Velocity error (3)
 * - Attitude error (3) - rotation vector
 * - Accelerometer bias error (3)
 * - Gyroscope bias error (3)
 *
 * The filter alternates between:
 * 1. Prediction using IMU measurements (high rate)
 * 2. Update using GPS, barometer, or magnetometer (lower rate)
 */
class ExtendedKalmanFilter {
public:
    /**
     * @brief Construct EKF with initial state and covariance
     * @param initial_state Initial navigation state estimate
     */
    ExtendedKalmanFilter(const NavigationState& initial_state);

    /**
     * @brief Predict state forward using IMU measurements
     * @param imu_meas IMU measurement
     * @param dt Time step [s]
     */
    void predict(const IMUMeasurement& imu_meas, double dt);

    /**
     * @brief Update state using GPS position measurement
     * @param gps_meas GPS measurement
     */
    void updateGPS(const GPSMeasurement& gps_meas);

    /**
     * @brief Update state using barometric altitude measurement
     * @param baro_meas Barometer measurement
     */
    void updateBarometer(const BarometerMeasurement& baro_meas);

    /**
     * @brief Update state using magnetometer measurement
     * @param mag_meas Magnetometer measurement
     * @param mag_field_ned Local magnetic field in NED frame [uT]
     */
    void updateMagnetometer(const MagnetometerMeasurement& mag_meas,
                           const std::array<double, 3>& mag_field_ned);

    /**
     * @brief Get current state estimate
     */
    NavigationState getState() const { return state_; }

    /**
     * @brief Get error covariance matrix diagonal
     * @return 15 diagonal elements of P matrix
     */
    std::array<double, ERROR_STATE_SIZE> getCovarianceDiagonal() const;

    /**
     * @brief Set process noise covariance
     */
    void setProcessNoise(const std::array<double, ERROR_STATE_SIZE>& Q_diag);

    /**
     * @brief Set GPS measurement noise covariance
     */
    void setGPSNoise(double pos_std, double vel_std);

    /**
     * @brief Set barometer measurement noise covariance
     */
    void setBarometerNoise(double alt_std);

    /**
     * @brief Set magnetometer measurement noise covariance
     */
    void setMagnetometerNoise(double mag_std);

private:
    NavigationState state_;  // Current state estimate

    // Error covariance matrix (15x15, stored as 1D array)
    std::array<double, ERROR_STATE_SIZE * ERROR_STATE_SIZE> P_;

    // Process noise covariance (diagonal elements)
    std::array<double, ERROR_STATE_SIZE> Q_diag_;

    // Measurement noise standard deviations
    double gps_pos_std_;
    double gps_vel_std_;
    double baro_alt_std_;
    double mag_std_;

    /**
     * @brief Compute state transition matrix F
     * @param dt Time step [s]
     * @param f_body Specific force in body frame [m/s^2]
     * @param F Output state transition matrix (15x15)
     */
    void computeStateTransitionMatrix(double dt,
                                     const std::array<double, 3>& f_body,
                                     double F[ERROR_STATE_SIZE][ERROR_STATE_SIZE]);

    /**
     * @brief Predict covariance forward: P = F*P*F' + Q
     */
    void predictCovariance(double F[ERROR_STATE_SIZE][ERROR_STATE_SIZE], double dt);

    /**
     * @brief Scalar measurement update (Joseph form, numerically stable)
     * @param measurement_value Measured value
     * @param predicted_value Predicted value h(x)
     * @param state_index Index in state vector being measured
     * @param R Measurement variance (scalar)
     */
    void scalarUpdate(double measurement_value, double predicted_value,
                     int state_index, double R);

    /**
     * @brief Generic measurement update (DEPRECATED - use scalarUpdate)
     * @param H Measurement matrix
     * @param R Measurement noise covariance
     * @param innovation Innovation vector (z - h(x))
     * @param meas_size Measurement dimension
     */
    void measurementUpdate(const double* H, const double* R,
                          const double* innovation, int meas_size);

    /**
     * @brief Apply error state correction to nominal state
     * @param error_state Error state vector (15D)
     */
    void applyErrorStateCorrection(const std::array<double, ERROR_STATE_SIZE>& error_state);

    /**
     * @brief Skew-symmetric matrix from vector (for cross product)
     */
    static void skewSymmetric(const std::array<double, 3>& v, double S[3][3]);

    /**
     * @brief Matrix multiplication: C = A * B
     */
    static void matrixMultiply(const double* A, const double* B, double* C,
                             int m, int n, int p);

    /**
     * @brief Matrix transpose
     */
    static void matrixTranspose(const double* A, double* At, int m, int n);
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_EKF_HPP

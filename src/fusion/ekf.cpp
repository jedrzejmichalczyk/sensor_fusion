#include "fusion/ekf.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>

namespace sensor_fusion {

// Earth parameters
constexpr double EARTH_ROTATION_RATE = 7.2921159e-5;  // rad/s
constexpr double GRAVITY = 9.81;  // m/s^2

ExtendedKalmanFilter::ExtendedKalmanFilter(const NavigationState& initial_state)
    : state_(initial_state),
      gps_pos_std_(2.5),
      gps_vel_std_(0.1),
      baro_alt_std_(1.0),
      mag_std_(5.0)
{
    // Initialize error covariance matrix to small values
    std::fill(P_.begin(), P_.end(), 0.0);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        P_[i * ERROR_STATE_SIZE + i] = 1.0;  // 1.0 variance on diagonal
    }

    // Initialize process noise (default values)
    Q_diag_ = {
        0.01, 0.01, 0.01,      // Position process noise
        0.1, 0.1, 0.1,         // Velocity process noise
        0.01, 0.01, 0.01,      // Attitude process noise
        1e-6, 1e-6, 1e-6,      // Accel bias process noise
        1e-7, 1e-7, 1e-7       // Gyro bias process noise
    };
}

void ExtendedKalmanFilter::predict(const IMUMeasurement& imu_meas, double dt)
{
    // Compensate for estimated biases
    std::array<double, 3> f_body;
    std::array<double, 3> omega_body;

    for (int i = 0; i < 3; ++i) {
        f_body[i] = imu_meas.accel[i] - state_.accel_bias[i];
        omega_body[i] = imu_meas.gyro[i] - state_.gyro_bias[i];
    }

    // Get current rotation matrix
    double R_bn[3][3];
    state_.getRotationMatrix(R_bn);

    // Transform specific force to navigation frame
    std::array<double, 3> f_nav = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            f_nav[i] += R_bn[i][j] * f_body[j];
        }
    }

    // Subtract gravity (add it as positive down in NED frame)
    f_nav[2] -= GRAVITY;

    // Update velocity (v = v + a*dt)
    for (int i = 0; i < 3; ++i) {
        state_.velocity_ned[i] += f_nav[i] * dt;
    }

    // Update position (p = p + v*dt)
    for (int i = 0; i < 3; ++i) {
        state_.position_ned[i] += state_.velocity_ned[i] * dt;
    }

    // Update quaternion using angular velocity
    // q_dot = 0.5 * q * omega
    double qw = state_.quaternion[0];
    double qx = state_.quaternion[1];
    double qy = state_.quaternion[2];
    double qz = state_.quaternion[3];

    double wx = omega_body[0];
    double wy = omega_body[1];
    double wz = omega_body[2];

    // Quaternion derivative
    double qw_dot = 0.5 * (-qx*wx - qy*wy - qz*wz);
    double qx_dot = 0.5 * ( qw*wx + qy*wz - qz*wy);
    double qy_dot = 0.5 * ( qw*wy - qx*wz + qz*wx);
    double qz_dot = 0.5 * ( qw*wz + qx*wy - qy*wx);

    // Integrate quaternion
    state_.quaternion[0] += qw_dot * dt;
    state_.quaternion[1] += qx_dot * dt;
    state_.quaternion[2] += qy_dot * dt;
    state_.quaternion[3] += qz_dot * dt;

    // Normalize quaternion
    state_.normalizeQuaternion();

    // Predict covariance
    double F[ERROR_STATE_SIZE][ERROR_STATE_SIZE];
    computeStateTransitionMatrix(dt, f_body, F);
    predictCovariance(F, dt);
}

void ExtendedKalmanFilter::computeStateTransitionMatrix(double dt,
                                                       const std::array<double, 3>& f_body,
                                                       double F[ERROR_STATE_SIZE][ERROR_STATE_SIZE])
{
    // Initialize to identity
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            F[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Get rotation matrix
    double R[3][3];
    state_.getRotationMatrix(R);

    // Position rows: dp/dt = v
    for (int i = 0; i < 3; ++i) {
        F[i][i + 3] = dt;  // Position depends on velocity
    }

    // Velocity rows: dv/dt = R*f
    // Skew-symmetric matrix of f_body for cross product
    double f_skew[3][3];
    std::array<double, 3> f_body_arr = {f_body[0], f_body[1], f_body[2]};
    skewSymmetric(f_body_arr, f_skew);

    // dv/dtheta = -R * [f]_x
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += R[i][k] * f_skew[k][j];
            }
            F[3 + i][6 + j] = -dt * sum;
        }
    }

    // dv/dba = R
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F[3 + i][9 + j] = dt * R[i][j];
        }
    }

    // Attitude rows: dtheta/dbg = -I
    for (int i = 0; i < 3; ++i) {
        F[6 + i][12 + i] = -dt;
    }

    // Bias rows: evolve slowly (could add Gauss-Markov here)
    // For now, assume constant: F already has identity
}

void ExtendedKalmanFilter::predictCovariance(double F[ERROR_STATE_SIZE][ERROR_STATE_SIZE], double dt)
{
    // Compute P = F*P*F' + Q
    // This is simplified - full implementation would use matrix operations

    // Create Q matrix (diagonal)
    double Q[ERROR_STATE_SIZE][ERROR_STATE_SIZE] = {0};
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        Q[i][i] = Q_diag_[i] * dt;  // Scale by time step
    }

    // Copy current P to temporary
    double P_old[ERROR_STATE_SIZE][ERROR_STATE_SIZE];
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_old[i][j] = P_[i * ERROR_STATE_SIZE + j];
        }
    }

    // Compute F*P
    double FP[ERROR_STATE_SIZE][ERROR_STATE_SIZE] = {0};
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                FP[i][j] += F[i][k] * P_old[k][j];
            }
        }
    }

    // Compute F*P*F'
    double FPFT[ERROR_STATE_SIZE][ERROR_STATE_SIZE] = {0};
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                FPFT[i][j] += FP[i][k] * F[j][k];  // F[j][k] is F'[k][j]
            }
        }
    }

    // Add Q and store result
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_[i * ERROR_STATE_SIZE + j] = FPFT[i][j] + Q[i][j];
        }
    }
}

void ExtendedKalmanFilter::updateGPS(const GPSMeasurement& gps_meas)
{
    if (!gps_meas.valid) return;

    // Use sequential scalar updates for numerical stability
    // Process each measurement component independently

    // Update with position measurements
    for (int i = 0; i < 3; ++i) {
        scalarUpdate(gps_meas.position_ned[i], state_.position_ned[i],
                     i, gps_pos_std_ * gps_pos_std_);
    }

    // Update with velocity measurements
    for (int i = 0; i < 3; ++i) {
        scalarUpdate(gps_meas.velocity_ned[i], state_.velocity_ned[i],
                     3 + i, gps_vel_std_ * gps_vel_std_);
    }
}

void ExtendedKalmanFilter::updateBarometer(const BarometerMeasurement& baro_meas)
{
    // Measurement model: z = -p_D (altitude is negative down position)
    const int meas_size = 1;

    // Measurement matrix H (1x15)
    double H[ERROR_STATE_SIZE] = {0};
    H[2] = -1.0;  // Altitude = -Down

    // Measurement noise covariance R (1x1)
    double R = baro_alt_std_ * baro_alt_std_;

    // Innovation
    double innovation = baro_meas.altitude - (-state_.position_ned[2]);

    // Perform measurement update
    measurementUpdate(H, &R, &innovation, meas_size);
}

void ExtendedKalmanFilter::updateMagnetometer(const MagnetometerMeasurement& mag_meas,
                                             const std::array<double, 3>& mag_field_ned)
{
    // Measurement model: z = R_nb * mag_field_ned
    // where R_nb is body-to-nav rotation transpose (nav-to-body)

    const int meas_size = 3;

    // Get rotation matrix
    double R_bn[3][3];
    state_.getRotationMatrix(R_bn);

    // Predicted measurement: R_nb * mag_field_ned
    std::array<double, 3> h = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            h[i] += R_bn[j][i] * mag_field_ned[j];  // Transpose: R_nb = R_bn'
        }
    }

    // Innovation
    double innovation[meas_size];
    for (int i = 0; i < 3; ++i) {
        innovation[i] = mag_meas.magnetic_field[i] - h[i];
    }

    // Measurement Jacobian (simplified - linearized around current state)
    // H = d(R_nb * m)/d(error_state)
    // For error state, this involves the attitude error part
    double H[meas_size * ERROR_STATE_SIZE] = {0};

    // Skew-symmetric matrix of predicted measurement
    double h_skew[3][3];
    skewSymmetric(h, h_skew);

    // H[:, 6:9] = [h]_x (measurement depends on attitude error)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H[i * ERROR_STATE_SIZE + (6 + j)] = h_skew[i][j];
        }
    }

    // Measurement noise covariance R (3x3)
    double R[meas_size * meas_size] = {0};
    for (int i = 0; i < 3; ++i) {
        R[i * meas_size + i] = mag_std_ * mag_std_;
    }

    // Perform measurement update
    measurementUpdate(H, R, innovation, meas_size);
}

void ExtendedKalmanFilter::measurementUpdate(const double* H, const double* R,
                                            const double* innovation, int meas_size)
{
    // Compute Kalman gain: K = P*H' * (H*P*H' + R)^(-1)

    // Compute H*P (meas_size x ERROR_STATE_SIZE)
    std::vector<double> HP(meas_size * ERROR_STATE_SIZE);
    for (int i = 0; i < meas_size; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            HP[i * ERROR_STATE_SIZE + j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                HP[i * ERROR_STATE_SIZE + j] += H[i * ERROR_STATE_SIZE + k] * P_[k * ERROR_STATE_SIZE + j];
            }
        }
    }

    // Compute H*P*H' (meas_size x meas_size)
    std::vector<double> HPH(meas_size * meas_size);
    for (int i = 0; i < meas_size; ++i) {
        for (int j = 0; j < meas_size; ++j) {
            HPH[i * meas_size + j] = R[i * meas_size + j];  // Initialize with R
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                HPH[i * meas_size + j] += HP[i * ERROR_STATE_SIZE + k] * H[j * ERROR_STATE_SIZE + k];
            }
        }
    }

    // Invert HPH (simple for small matrices)
    // For 1x1: inv = 1/value
    // For larger: use numerical methods (simplified here)
    std::vector<double> HPH_inv(meas_size * meas_size);
    if (meas_size == 1) {
        HPH_inv[0] = 1.0 / HPH[0];
    } else {
        // Simplified: use diagonal approximation for demo
        for (int i = 0; i < meas_size * meas_size; ++i) {
            HPH_inv[i] = 0.0;
        }
        for (int i = 0; i < meas_size; ++i) {
            HPH_inv[i * meas_size + i] = 1.0 / HPH[i * meas_size + i];
        }
    }

    // Compute P*H' (ERROR_STATE_SIZE x meas_size)
    std::vector<double> PHt(ERROR_STATE_SIZE * meas_size);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < meas_size; ++j) {
            PHt[i * meas_size + j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                PHt[i * meas_size + j] += P_[i * ERROR_STATE_SIZE + k] * H[j * ERROR_STATE_SIZE + k];
            }
        }
    }

    // Compute Kalman gain: K = P*H' * HPH_inv
    std::vector<double> K(ERROR_STATE_SIZE * meas_size);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < meas_size; ++j) {
            K[i * meas_size + j] = 0.0;
            for (int k = 0; k < meas_size; ++k) {
                K[i * meas_size + j] += PHt[i * meas_size + k] * HPH_inv[k * meas_size + j];
            }
        }
    }

    // Compute error state correction: dx = K * innovation
    std::array<double, ERROR_STATE_SIZE> error_state = {0};
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < meas_size; ++j) {
            error_state[i] += K[i * meas_size + j] * innovation[j];
        }
    }

    // Apply correction to state
    applyErrorStateCorrection(error_state);

    // Update covariance: P = (I - K*H) * P
    std::vector<double> KH(ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            KH[i * ERROR_STATE_SIZE + j] = 0.0;
            for (int k = 0; k < meas_size; ++k) {
                KH[i * ERROR_STATE_SIZE + j] += K[i * meas_size + k] * H[k * ERROR_STATE_SIZE + j];
            }
        }
    }

    std::vector<double> P_new(ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_new[i * ERROR_STATE_SIZE + j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                double IKH_k = (i == k ? 1.0 : 0.0) - KH[i * ERROR_STATE_SIZE + k];
                P_new[i * ERROR_STATE_SIZE + j] += IKH_k * P_[k * ERROR_STATE_SIZE + j];
            }
        }
    }

    // Copy result back
    std::copy(P_new.begin(), P_new.end(), P_.begin());
}

void ExtendedKalmanFilter::scalarUpdate(double measurement_value,
                                       double predicted_value,
                                       int state_index,
                                       double R)
{
    // Innovation: y = z - h(x)
    double innovation = measurement_value - predicted_value;

    // Measurement Jacobian H (1 x 15 vector)
    // H has 1 at state_index, 0 elsewhere
    std::array<double, ERROR_STATE_SIZE> H = {0};
    H[state_index] = 1.0;

    // Innovation covariance: S = H*P*H' + R  (scalar)
    double S = R;  // Start with R
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        S += H[i] * P_[i * ERROR_STATE_SIZE + state_index];
    }

    // Check for numerical issues
    if (S <= 0.0 || std::isnan(S) || std::isinf(S)) {
        return;  // Skip this update
    }

    // Kalman gain: K = P*H' / S  (15 x 1 vector)
    std::array<double, ERROR_STATE_SIZE> K;
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        K[i] = P_[i * ERROR_STATE_SIZE + state_index] / S;
    }

    // State update: x = x + K*innovation
    std::array<double, ERROR_STATE_SIZE> error_state = {0};
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        error_state[i] = K[i] * innovation;
    }
    applyErrorStateCorrection(error_state);

    // Covariance update using Joseph form for numerical stability:
    // P = (I - K*H) * P * (I - K*H)' + K*R*K'

    // Compute (I - K*H)
    std::vector<double> IKH(ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            IKH[i * ERROR_STATE_SIZE + j] = (i == j ? 1.0 : 0.0) - K[i] * H[j];
        }
    }

    // Temp = (I-KH) * P
    std::vector<double> Temp(ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            Temp[i * ERROR_STATE_SIZE + j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                Temp[i * ERROR_STATE_SIZE + j] += IKH[i * ERROR_STATE_SIZE + k] * P_[k * ERROR_STATE_SIZE + j];
            }
        }
    }

    // P_new = Temp * (I-KH)' + K*R*K'
    std::vector<double> P_new(ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_new[i * ERROR_STATE_SIZE + j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                P_new[i * ERROR_STATE_SIZE + j] += Temp[i * ERROR_STATE_SIZE + k] * IKH[j * ERROR_STATE_SIZE + k];
            }
            // Add K*R*K'
            P_new[i * ERROR_STATE_SIZE + j] += K[i] * R * K[j];
        }
    }

    // Copy back to P_ and ensure symmetry
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_[i * ERROR_STATE_SIZE + j] = P_new[i * ERROR_STATE_SIZE + j];
        }
    }

    // Enforce symmetry (average with transpose)
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = i + 1; j < ERROR_STATE_SIZE; ++j) {
            double avg = 0.5 * (P_[i * ERROR_STATE_SIZE + j] + P_[j * ERROR_STATE_SIZE + i]);
            P_[i * ERROR_STATE_SIZE + j] = avg;
            P_[j * ERROR_STATE_SIZE + i] = avg;
        }
    }
}

void ExtendedKalmanFilter::applyErrorStateCorrection(const std::array<double, ERROR_STATE_SIZE>& error_state)
{
    // Position correction
    for (int i = 0; i < 3; ++i) {
        state_.position_ned[i] += error_state[IDX_POS + i];
    }

    // Velocity correction
    for (int i = 0; i < 3; ++i) {
        state_.velocity_ned[i] += error_state[IDX_VEL + i];
    }

    // Attitude correction (using small angle approximation)
    double dtheta[3] = {error_state[IDX_ATT], error_state[IDX_ATT + 1], error_state[IDX_ATT + 2]};
    double angle = std::sqrt(dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2]);

    if (angle > 1e-9) {
        // Rotation quaternion from error
        double dq[4];
        dq[0] = std::cos(angle / 2.0);
        dq[1] = dtheta[0] / angle * std::sin(angle / 2.0);
        dq[2] = dtheta[1] / angle * std::sin(angle / 2.0);
        dq[3] = dtheta[2] / angle * std::sin(angle / 2.0);

        // Apply correction: q = q * dq
        double q_new[4];
        q_new[0] = state_.quaternion[0]*dq[0] - state_.quaternion[1]*dq[1] - state_.quaternion[2]*dq[2] - state_.quaternion[3]*dq[3];
        q_new[1] = state_.quaternion[0]*dq[1] + state_.quaternion[1]*dq[0] + state_.quaternion[2]*dq[3] - state_.quaternion[3]*dq[2];
        q_new[2] = state_.quaternion[0]*dq[2] - state_.quaternion[1]*dq[3] + state_.quaternion[2]*dq[0] + state_.quaternion[3]*dq[1];
        q_new[3] = state_.quaternion[0]*dq[3] + state_.quaternion[1]*dq[2] - state_.quaternion[2]*dq[1] + state_.quaternion[3]*dq[0];

        std::copy(q_new, q_new + 4, state_.quaternion.begin());
        state_.normalizeQuaternion();
    }

    // Bias corrections
    for (int i = 0; i < 3; ++i) {
        state_.accel_bias[i] += error_state[IDX_ACC_BIAS + i];
        state_.gyro_bias[i] += error_state[IDX_GYR_BIAS + i];
    }
}

std::array<double, ERROR_STATE_SIZE> ExtendedKalmanFilter::getCovarianceDiagonal() const
{
    std::array<double, ERROR_STATE_SIZE> diag;
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        diag[i] = P_[i * ERROR_STATE_SIZE + i];
    }
    return diag;
}

void ExtendedKalmanFilter::setProcessNoise(const std::array<double, ERROR_STATE_SIZE>& Q_diag)
{
    Q_diag_ = Q_diag;
}

void ExtendedKalmanFilter::setGPSNoise(double pos_std, double vel_std)
{
    gps_pos_std_ = pos_std;
    gps_vel_std_ = vel_std;
}

void ExtendedKalmanFilter::setBarometerNoise(double alt_std)
{
    baro_alt_std_ = alt_std;
}

void ExtendedKalmanFilter::setMagnetometerNoise(double mag_std)
{
    mag_std_ = mag_std;
}

void ExtendedKalmanFilter::skewSymmetric(const std::array<double, 3>& v, double S[3][3])
{
    S[0][0] = 0.0;    S[0][1] = -v[2];  S[0][2] = v[1];
    S[1][0] = v[2];   S[1][1] = 0.0;    S[1][2] = -v[0];
    S[2][0] = -v[1];  S[2][1] = v[0];   S[2][2] = 0.0;
}

} // namespace sensor_fusion

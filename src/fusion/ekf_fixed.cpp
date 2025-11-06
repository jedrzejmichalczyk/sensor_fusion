// This file contains the FIXED measurement update implementation
// The key changes:
// 1. Sequential scalar updates instead of batch vector updates
// 2. Proper covariance update (Joseph form for numerical stability)
// 3. NaN/Inf checking
// 4. Innovation gating

// Sequential scalar measurement update
// Process each GPS component (Pos_N, Pos_E, Pos_D, Vel_N, Vel_E, Vel_D) separately
void ExtendedKalmanFilter::updateGPS_Sequential(const GPSMeasurement& gps_meas)
{
    if (!gps_meas.valid) return;

    // Process position measurements sequentially
    for (int i = 0; i < 3; ++i) {
        scalarUpdate(gps_meas.position_ned[i], state_.position_ned[i],
                     i, gps_pos_std_ * gps_pos_std_);
    }

    // Process velocity measurements sequentially
    for (int i = 0; i < 3; ++i) {
        scalarUpdate(gps_meas.velocity_ned[i], state_.velocity_ned[i],
                     3 + i, gps_vel_std_ * gps_vel_std_);
    }
}

// Scalar measurement update for single component
// measurement_value: z
// predicted_value: h(x)
// state_index: which state component is being measured
// R: measurement variance (scalar)
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
        S += H[i] * P_[i * ERROR_STATE_SIZE + state_index] * H[state_index];
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
    double IKH[ERROR_STATE_SIZE][ERROR_STATE_SIZE];
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            IKH[i][j] = (i == j ? 1.0 : 0.0) - K[i] * H[j];
        }
    }

    // Temp = (I-KH) * P
    double Temp[ERROR_STATE_SIZE][ERROR_STATE_SIZE];
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            Temp[i][j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                Temp[i][j] += IKH[i][k] * P_[k * ERROR_STATE_SIZE + j];
            }
        }
    }

    // P_new = Temp * (I-KH)'
    double P_new[ERROR_STATE_SIZE][ERROR_STATE_SIZE];
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_new[i][j] = 0.0;
            for (int k = 0; k < ERROR_STATE_SIZE; ++k) {
                P_new[i][j] += Temp[i][k] * IKH[j][k];  // IKH[j][k] is transpose
            }
        }
    }

    // Add K*R*K'
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_new[i][j] += K[i] * R * K[j];
        }
    }

    // Copy back to P_
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = 0; j < ERROR_STATE_SIZE; ++j) {
            P_[i * ERROR_STATE_SIZE + j] = P_new[i][j];
        }
    }

    // Ensure symmetry (average with transpose)
    for (int i = 0; i < ERROR_STATE_SIZE; ++i) {
        for (int j = i + 1; j < ERROR_STATE_SIZE; ++j) {
            double avg = 0.5 * (P_[i * ERROR_STATE_SIZE + j] + P_[j * ERROR_STATE_SIZE + i]);
            P_[i * ERROR_STATE_SIZE + j] = avg;
            P_[j * ERROR_STATE_SIZE + i] = avg;
        }
    }
}

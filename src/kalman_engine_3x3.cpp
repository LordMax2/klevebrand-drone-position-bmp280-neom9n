#include "kalman_engine_3x3.h"

void KalmanEngine3x3::reset()
{
    _stateVector[0] = 0.0f;
    _stateVector[1] = 0.0f;
    _stateVector[2] = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            _covarianceMatrix[i][j] = i == j ? 1.0f : 0.0f;
        }
    }
}

float KalmanEngine3x3::predictKinematics(const float input, const float delta_time_seconds)
{
    // 1. Remove background estimated sensor bias from the raw measurement
    const float pure_input = input - _stateVector[2];

    // 2. Physics Integration: Update State Vector
    _stateVector[0] = _stateVector[0] + _stateVector[1] * delta_time_seconds + 0.5f * pure_input * delta_time_seconds * delta_time_seconds;
    _stateVector[1] = _stateVector[1] + pure_input * delta_time_seconds;

    // 3. Unrolled Covariance Update: P = F * P * F^T + Q
    float p00 = _covarianceMatrix[0][0], p01 = _covarianceMatrix[0][1], p02 = _covarianceMatrix[0][2];
    float p10 = _covarianceMatrix[1][0], p11 = _covarianceMatrix[1][1], p12 = _covarianceMatrix[1][2];
    float p20 = _covarianceMatrix[2][0], p21 = _covarianceMatrix[2][1], p22 = _covarianceMatrix[2][2];

    _covarianceMatrix[0][0] = p00 + delta_time_seconds*(p10 + p01 + delta_time_seconds*p11) - delta_time_seconds*delta_time_seconds*(p20 + p02 + delta_time_seconds*p21) + 0.25f*delta_time_seconds*delta_time_seconds*delta_time_seconds*delta_time_seconds*p22 + Q1;
    _covarianceMatrix[0][1] = p01 + delta_time_seconds*p11 - 0.5f*delta_time_seconds*delta_time_seconds*p21 - delta_time_seconds*p02 - 0.5f*delta_time_seconds*delta_time_seconds*p12 + 0.5f*delta_time_seconds*delta_time_seconds*delta_time_seconds*p22;
    _covarianceMatrix[0][2] = p02 + delta_time_seconds*p12 - 0.5f*delta_time_seconds*delta_time_seconds*p22;

    _covarianceMatrix[1][0] = p10 + delta_time_seconds*p11 - delta_time_seconds*p20 - 0.5f*delta_time_seconds*delta_time_seconds*p21 - 0.5f*delta_time_seconds*delta_time_seconds*p12 + 0.5f*delta_time_seconds*delta_time_seconds*delta_time_seconds*p22;
    _covarianceMatrix[1][1] = p11 - delta_time_seconds*p21 - delta_time_seconds*p12 + delta_time_seconds*delta_time_seconds*p22 + Q2;
    _covarianceMatrix[1][2] = p12 - delta_time_seconds*p22;

    _covarianceMatrix[2][0] = p20 + delta_time_seconds*p21 - 0.5f*delta_time_seconds*delta_time_seconds*p22;
    _covarianceMatrix[2][1] = p21 - delta_time_seconds*p22;
    _covarianceMatrix[2][2] = p22 + Q3;

    return _stateVector[0];
}

float KalmanEngine3x3::updateZeroState(const float measured_value, const float R_sensor)
{
    // 1. Calculate Innovation Error (Discrepancy between reality and prediction)
    const float error = measured_value - _stateVector[0];

    // 2. Innovation Covariance: S = H * P * H^T + R
    const float S = _covarianceMatrix[0][0] + R_sensor;

    // Avoid division by zero if tuning parameters are malformed
    if (S == 0.0f) {
        return error;
    }

    // 3. Compute 3-element Kalman Gain Vector: K = P * H^T / S
    float K[3];
    K[0] = _covarianceMatrix[0][0] / S; // Gain for State 0 (Position)
    K[1] = _covarianceMatrix[1][0] / S; // Gain for State 1 (Velocity)
    K[2] = _covarianceMatrix[2][0] / S; // Gain for State 2 (Bias)

    // 4. Inject Correction into State Vector
    _stateVector[0] += K[0] * error;
    _stateVector[1] += K[1] * error;
    _stateVector[2] += K[2] * error;

    // 5. Shrink the Covariance Matrix Uncertainty: P = (I - K * H) * P
    float p00 = _covarianceMatrix[0][0], p01 = _covarianceMatrix[0][1], p02 = _covarianceMatrix[0][2];

    _covarianceMatrix[0][0] -= K[0] * p00;  _covarianceMatrix[0][1] -= K[0] * p01;  _covarianceMatrix[0][2] -= K[0] * p02;
    _covarianceMatrix[1][0] -= K[1] * p00;  _covarianceMatrix[1][1] -= K[1] * p01;  _covarianceMatrix[1][2] -= K[1] * p02;
    _covarianceMatrix[2][0] -= K[2] * p00;  _covarianceMatrix[2][1] -= K[2] * p01;  _covarianceMatrix[2][2] -= K[2] * p02;

    // Return the error back to caller (highly valuable for debugging/glitch detection)
    return error;
}

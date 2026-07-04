#pragma once

class KalmanEngine3x3
{
    float _stateVector[3]{};

    float _covarianceMatrix[3][3]{};

    float Q1 = 0.01f;
    float Q2 = 0.01f;
    float Q3 = 0.0001f;

public:
    KalmanEngine3x3() = default;
    ~KalmanEngine3x3() = default;

    float predictKinematics(float input, float delta_time_seconds);
    float updateZeroState(float measured_value, float R_sensor);

    void reset();

    float getVelocity() const
    {
        return _stateVector[1];
    }

    float getPosition() const
    {
        return _stateVector[0];
    }
};

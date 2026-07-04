#pragma once

class KalmanEngine
{

public:
    KalmanEngine() = default;
    ~KalmanEngine() = default;

    float predict(float uncertain_value, float delta_time_seconds);
    float update(float certain_value, float delta_time_seconds);

    float getUncertainValue();
    float getCertainValue();
    float getBias();
};

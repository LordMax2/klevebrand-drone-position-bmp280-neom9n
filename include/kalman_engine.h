#ifndef KLEVEBRAND_DRONE_POSITION_BMP280_NEOM9N_KALMAN_ENGINE_H
#define KLEVEBRAND_DRONE_POSITION_BMP280_NEOM9N_KALMAN_ENGINE_H

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

#endif //KLEVEBRAND_DRONE_POSITION_BMP280_NEOM9N_KALMAN_ENGINE_H
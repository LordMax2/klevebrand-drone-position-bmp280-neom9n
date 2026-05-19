#ifndef KLEVEBRAND_MAXFLY_DRONE_QUADCOPTER_POSITION_H
#define KLEVEBRAND_MAXFLY_DRONE_QUADCOPTER_POSITION_H

#include <Adafruit_BMP280.h>

#include "base_drone_gyro.h"
#include "base_drone_position.h"
#include "kalman_engine_3x3.h"

class QuadcopterPosition : public BaseDronePosition {
    Adafruit_BMP280 _bmp_device;

    float _temperature = 0.0f;
    float _pressure = 0.0f;
    float _bmp280_last_altitude = 0.0f;

    unsigned long _last_run_timestamp_microseconds = 0;
    float _last_imu_update_seconds = 0;

    unsigned long _run_interval_microseconds;
    static constexpr float sea_level_pressure_pa = 101325.0f;

    KalmanEngine3x3 _kalman_altitude{};

    BaseDroneGyro* _gyro;

    static float pressureToAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

public:
    explicit QuadcopterPosition(BaseDroneGyro* gyro, const int run_interval_hz = 25) {
        _run_interval_microseconds = 1000000UL / run_interval_hz;
        _gyro = gyro;
    }

    void setup();

    float getTemperature() const { return _temperature; }

    float getPressure() const { return _pressure; }

    float getAltitude() override;

    float getVelocityZ() override;

    void run(bool has_gyro_update);
};

#endif

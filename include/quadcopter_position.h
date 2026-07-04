#pragma once

#include <Adafruit_BMP280.h>
#include "concept_drone_gyro.h"
#include "concept_drone_position.h"
#include "kalman_engine_3x3.h"

template <DroneGyroConcept SomeDroneGyroType>
class QuadcopterPosition {
    Adafruit_BMP280 _bmp_device;

    float _temperature = 0.0f;
    float _pressure = 0.0f;
    float _bmp280_last_altitude = 0.0f;

    unsigned long _last_run_timestamp_microseconds = 0;
    float _last_imu_update_seconds = 0;

    unsigned long _run_interval_microseconds;
    static constexpr float sea_level_pressure_pa = 101325.0f;

    KalmanEngine3x3 _kalman_altitude{};

    SomeDroneGyroType* _gyro;

    static float pressureToAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

public:
    explicit QuadcopterPosition<SomeDroneGyroType>(SomeDroneGyroType* gyro, const int run_interval_hz = 25) {
        _run_interval_microseconds = 1000000UL / run_interval_hz;
        _gyro = gyro;
    }

    void setup();

    float getTemperature() const { return _temperature; }

    float getPressure() const { return _pressure; }

    float getAltitude();

    float getRawAltitude() const;

    float getVelocityZ();

    float getVelocityX() const {return 0.0f};

    float getVelocityY() const {return 0.0f};

    float getLongitude() const {return 0.0f};

    float getLatitude() const {return 0.0f};

    void run(bool has_gyro_update);

    static_assert(DronePositionConcept<QuadcopterPosition<SomeDroneGyroType>>, "QuadcopterPosition must implement DronePositionConcept");
};

#include "quadcopter_position.ipp"
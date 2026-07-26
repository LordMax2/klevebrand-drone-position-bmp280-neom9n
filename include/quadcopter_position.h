#pragma once

#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "concept_drone_gyro.h"
#include "kalman_engine_3x3.h"

static constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
static constexpr float GPS_DEGREES_SCALE = 1e-7f;
static constexpr unsigned long GPS_BAUD_RATE = 38400UL;
static constexpr uint8_t GPS_NAVIGATION_FREQUENCY_HZ = 10;

template <DroneGyroConcept SomeDroneGyroType>
class QuadcopterPosition
{
    Adafruit_BMP280 _bmp_device;
    SFE_UBLOX_GNSS_SERIAL _gps;
    HardwareSerial& _gps_serial;

    float _temperature = 0.0f;
    float _pressure = 0.0f;
    float _bmp280_last_altitude = 0.0f;
    float _latitude = 0.0f;
    float _longitude = 0.0f;

    unsigned long _last_run_timestamp_microseconds = 0;
    float _last_imu_update_seconds = 0;

    unsigned long _run_interval_microseconds;

    bool _gps_ready = false;

    KalmanEngine3x3 _kalman_altitude{};

    SomeDroneGyroType* _gyro;

    static float pressureToAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

    bool setupNeoM9n();
    bool setupBmp280();

public:
    QuadcopterPosition(SomeDroneGyroType* gyro, HardwareSerial& gps_serial, const int run_interval_hz = 25)
        : _gps_serial(gps_serial)
    {
        _run_interval_microseconds = 1000000UL / run_interval_hz;
        _gyro = gyro;
    }

    void setup();

    float getTemperature() const { return _temperature; }

    float getPressure() const { return _pressure; }

    float getAltitude();

    float getRawAltitude() const;

    float getVelocityZ();

    float getVelocityX() const { return 0.0f; }

    float getVelocityY() const { return 0.0f; }

    float getLongitude() const { return _longitude; }

    float getLatitude() const { return _latitude; }

    void run(bool has_gyro_update);
};

#include "quadcopter_position.ipp"

#pragma once

#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "concept_drone_gyro.h"
#include "kalman_engine_3x3.h"
#include "position_local_meters.h"

static constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
static constexpr float GPS_DEGREES_SCALE = 1e-7f;
static constexpr unsigned long GPS_BAUD_RATE = 38400UL;
static constexpr uint8_t GPS_NAVIGATION_FREQUENCY_HZ = 10;
static constexpr float METERS_PER_DEGREE_LATITUDE = 111320.0f;
static constexpr float GPS_POSITION_VARIANCE = 4.0f;
static constexpr float GPS_VELOCITY_VARIANCE = 0.01f;
static constexpr float GPS_VELOCITY_SCALE = 1e-3f;
static constexpr float BMP_ALTITUDE_VARIANCE = 0.14f;

template <DroneGyroConcept SomeDroneGyroType>
class DroneBmp280Neom9nPosition
{
    Adafruit_BMP280 _bmp_device;
    SFE_UBLOX_GNSS_SERIAL _gps;
    HardwareSerial& _gps_serial;

    float _temperature = 0.0f;
    float _pressure = 0.0f;
    float _bmp280_last_altitude = 0.0f;
    float _latitude = 0.0f;
    float _longitude = 0.0f;
    float _origin_latitude = 0.0f;
    float _origin_longitude = 0.0f;
    float _origin_pressure_pa = SEA_LEVEL_PRESSURE_PA;

    unsigned long _last_run_timestamp_microseconds = 0;
    unsigned long _last_imu_update_microseconds = 0;

    unsigned long _run_interval_microseconds;

    bool _gps_ready = false;
    bool _bmp280_ready = false;
    bool _ready = false;

    KalmanEngine3x3 _kalman_altitude{0.01f, 0.01f, 0.0001f};
    KalmanEngine3x3 _kalman_east{0.01f, 0.01f, 0.0001f};
    KalmanEngine3x3 _kalman_north{0.01f, 0.01f, 0.0001f};

    SomeDroneGyroType* _gyro;

    static float pressureToAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

    PositionLocalMeters latitudeLongitudeToLocalMeters(float latitude, float longitude) const;
    float localMetersToLatitude(float north_meters) const;
    float localMetersToLongitude(float east_meters) const;

    bool setupNeoM9n();
    bool setupBmp280();

public:
    DroneBmp280Neom9nPosition(SomeDroneGyroType* gyro, HardwareSerial& gps_serial, const int run_interval_hz = 25)
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

    float getVelocityX() const;

    float getVelocityY() const;

    float getLongitude() const;

    float getLatitude() const;

    float getRawLongitude() const { return _longitude; }

    float getRawLatitude() const { return _latitude; }

    bool isReady() const { return _ready; }

    void run(bool has_gyro_update);
};

#include "drone_bmp280_neom9n_position.ipp"

#include <math.h>

template <class SomeDroneGyroType>
void QuadcopterPosition<SomeDroneGyroType>::setup()
{
    if (!_bmp_device.begin())
    {
        Serial.println("FAILED TO SETUP BMP280..");

        return;
    }

    _bmp_device.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X4,
        Adafruit_BMP280::FILTER_X4,
        Adafruit_BMP280::STANDBY_MS_1
    );

    _kalman_altitude.reset();
}

template <class SomeDroneGyroType>
float QuadcopterPosition<SomeDroneGyroType>::pressureToAltitudeMeters(const float pressure_pa, const float sea_level_pressure_pa)
{
    return 44330.0f * (1.0f - pow(pressure_pa / sea_level_pressure_pa, 0.1903f));
}

template <class SomeDroneGyroType>
float QuadcopterPosition<SomeDroneGyroType>::getAltitude()
{
    return _kalman_altitude.getPosition();
}

template <class SomeDroneGyroType>
float QuadcopterPosition<SomeDroneGyroType>::getRawAltitude() const {
    return _bmp280_last_altitude;
}

template <class SomeDroneGyroType>
float QuadcopterPosition<SomeDroneGyroType>::getVelocityZ()
{
    return _kalman_altitude.getVelocity();
}

template <class SomeDroneGyroType>
void QuadcopterPosition<SomeDroneGyroType>::run(const bool has_gyro_update)
{
    const unsigned long now = micros();

    if (has_gyro_update)
    {
        const float imu_acceleration_z = _gyro->accelerationZ();

        if (_last_imu_update_seconds != 0.0f)
        {
            const float delta_time_seconds = now / 1000000.0f - _last_imu_update_seconds;
            _kalman_altitude.predictKinematics(imu_acceleration_z, delta_time_seconds);
        }

        _last_imu_update_seconds = now / 1000000.0f;
    }

    if (_last_run_timestamp_microseconds != 0 && now - _last_run_timestamp_microseconds < _run_interval_microseconds)
    {
        return;
    }

    _last_run_timestamp_microseconds = now;

    _pressure = _bmp_device.readPressure();
    _temperature = _bmp_device.readTemperature();

    if (_pressure <= 0.0f)
    {
        return;
    }

    _bmp280_last_altitude = pressureToAltitudeMeters(_pressure, sea_level_pressure_pa);

    _kalman_altitude.updateZeroState(_bmp280_last_altitude, 0.14);
}

#include <math.h>

template <class SomeDroneGyroType>
void QuadcopterPosition<SomeDroneGyroType>::setup()
{
    if (!_bmp_device.begin())
    {
        Serial.println("FAILED TO SETUP BMP280..");

        //return;
    }

    _bmp_device.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X4,
        Adafruit_BMP280::FILTER_X4,
        Adafruit_BMP280::STANDBY_MS_1
    );

    _kalman_altitude.reset();

    _gps_serial.begin(gps_baud_rate);
    delay(100);

    while (_gps_serial.available() > 0)
    {
        _gps_serial.read();
    }

    Serial.print(F("Connecting to NEO-M9N... "));

    if (!_gps.begin(_gps_serial))
    {
        Serial.println(F("FAILED"));

        return;
    }

    Serial.println(F("OK"));

    _gps.setUART1Output(COM_TYPE_UBX);
    _gps.setNavigationFrequency(gps_navigation_frequency_hz);
    _gps.setAutoPVT(true);

    _gps_ready = true;

    Serial.print(F("Navigation frequency set to "));
    Serial.print(gps_navigation_frequency_hz);
    Serial.println(F(" Hz"));
    Serial.println(F("Waiting for GPS lock..."));

    while (true)
    {
        if (_gps.getPVT())
        {
            const uint8_t fix_type = _gps.getFixType();
            const uint8_t satellite_count = _gps.getSIV();

            _latitude = _gps.getLatitude() * gps_degrees_scale;
            _longitude = _gps.getLongitude() * gps_degrees_scale;

            Serial.print(F("fix="));
            Serial.print(fix_type);
            Serial.print(F(" sats="));
            Serial.print(satellite_count);
            Serial.print(F(" lat="));
            Serial.print(_latitude, 7);
            Serial.print(F(" lon="));
            Serial.println(_longitude, 7);

            if (fix_type >= 3)
            {
                Serial.println(F("GPS lock acquired"));

                break;
            }
        }

        delay(500);
    }
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

    if (_gps_ready && _gps.getPVT())
    {
        _latitude = _gps.getLatitude() * gps_degrees_scale;
        _longitude = _gps.getLongitude() * gps_degrees_scale;
    }

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

#include "drone_bmp280_neom9n_position.h"
#include "bno08x_drone_gyro.h"

static Bno08xDroneGyro gyro(10);
static DroneBmp280Neom9nPosition drone_bmp280_neom9n_position(&gyro, Serial2);
static unsigned long gyro_last_run_milliseconds = 0;

void setup()
{
    Serial.begin(115200);

    Serial.println(F("START"));

    gyro.setup();
    gyro.setModeEulerAndAcceleration();

    drone_bmp280_neom9n_position.setup();
}

void loop()
{
    if (millis() - gyro_last_run_milliseconds > 4)
    {
        gyro_last_run_milliseconds = millis();

        if (gyro.reload())
        {
            drone_bmp280_neom9n_position.run(true);
        }
        else
        {
            drone_bmp280_neom9n_position.run(false);
        }
    }
    else
    {
        drone_bmp280_neom9n_position.run(false);
    }

    if (!drone_bmp280_neom9n_position.isReady())
    {
        return;
    }

    const float altitude = drone_bmp280_neom9n_position.getAltitude();
    const float raw_altitude = drone_bmp280_neom9n_position.getRawAltitude();
    const float velocity_x = drone_bmp280_neom9n_position.getVelocityX();
    const float velocity_y = drone_bmp280_neom9n_position.getVelocityY();
    const float velocity_z = drone_bmp280_neom9n_position.getVelocityZ();
    const float latitude = drone_bmp280_neom9n_position.getLatitude();
    const float longitude = drone_bmp280_neom9n_position.getLongitude();

    Serial.print(altitude);
    Serial.print(F("\t"));
    Serial.print(raw_altitude);
    Serial.print(F("\t"));
    Serial.print(velocity_x);
    Serial.print(F("\t"));
    Serial.print(velocity_y);
    Serial.print(F("\t"));
    Serial.print(velocity_z);
    Serial.print(F("\t"));
    Serial.print(latitude, 7);
    Serial.print(F("\t"));
    Serial.print(longitude, 7);
    Serial.println();

    delay(2);
}

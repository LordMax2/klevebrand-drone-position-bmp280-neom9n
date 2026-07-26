#include "quadcopter_position.h"
#include "bno08x_drone_gyro.h"

static Bno08xDroneGyro gyro(10);
static QuadcopterPosition quadcopter_position(&gyro, Serial2);
static unsigned long gyro_last_run_milliseconds = 0;

void setup()
{
    Serial.begin(115200);

    Serial.println(F("START"));

    gyro.setup();
    gyro.setModeEulerAndAcceleration();

    quadcopter_position.setup();
}

void loop()
{
    if (millis() - gyro_last_run_milliseconds > 4)
    {
        gyro_last_run_milliseconds = millis();

        if (gyro.reload())
        {
            quadcopter_position.run(true);
        }
        else
        {
            quadcopter_position.run(false);
        }
    }
    else
    {
        quadcopter_position.run(false);
    }

    if (!quadcopter_position.isReady())
    {
        return;
    }

    const float altitude = quadcopter_position.getAltitude();
    const float raw_altitude = quadcopter_position.getRawAltitude();
    const float velocity_x = quadcopter_position.getVelocityX();
    const float velocity_y = quadcopter_position.getVelocityY();
    const float velocity_z = quadcopter_position.getVelocityZ();
    const float latitude = quadcopter_position.getLatitude();
    const float longitude = quadcopter_position.getLongitude();

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

#include "quadcopter_position.h"
#include "bno08x_drone_gyro.h"

static Bno08xDroneGyro gyro(10);
static QuadcopterPosition<Bno08xDroneGyro> quadcopter_position(&gyro, Serial2);
static unsigned long gyro_last_run_milliseconds = 0;

void setup()
{
    Serial.begin(115200);

    Serial.println("START");

    quadcopter_position.setup();
    gyro.setup();
}

void loop()
{
    if (millis() - gyro_last_run_milliseconds > 4)
    {
        gyro.reload();
        gyro_last_run_milliseconds = millis();

        quadcopter_position.run(true);
    }
    else
    {
        quadcopter_position.run(false);
    }

    const float altitude = quadcopter_position.getAltitude();
    const float raw_altitude = quadcopter_position.getRawAltitude();
    const float velocity_z = quadcopter_position.getVelocityZ();
    const float latitude = quadcopter_position.getLatitude();
    const float longitude = quadcopter_position.getLongitude();

    Serial.print(altitude);
    Serial.print("\t");
    Serial.print(raw_altitude);
    Serial.print("\t");
    Serial.print(velocity_z);
    Serial.print("\t");
    Serial.print(latitude, 7);
    Serial.print("\t");
    Serial.print(longitude, 7);
    Serial.println();

    delay(2);
}

#include "quadcopter_position.h"
#include "bno08x_drone_gyro.h"

static QuadcopterPosition quadcopter_position;
static Bno08xDroneGyro gyro(10);
static unsigned long gyro_last_run_milliseconds = 0;


void setup() {
    Serial.begin(115200);

    Serial.println("START");

    quadcopter_position.setup();
    gyro.setup();

    gyro.printYawPitchRollAndAcceleration();
}

void loop() {
    quadcopter_position.run();

    if (millis() - gyro_last_run_milliseconds > 4000)
    {
        gyro.reload();
        gyro_last_run_milliseconds = millis();
    }

    Serial.print("Altitude: ");
    Serial.print(quadcopter_position.getAltitude());
    Serial.print("\t");
    Serial.print("VelZ :");
    Serial.print("\t");
    Serial.println(quadcopter_position.getVelocityZ());
}
#include "quadcopter_position.h"

QuadcopterPosition quadcopter_position;

void setup() {
    Serial.begin(115200);

    Serial.println("START");

    quadcopter_position.setup();
}

void loop() {
    quadcopter_position.run();

    Serial.print("Altitude: ");
    Serial.print(quadcopter_position.getAltitude());
    Serial.print("\t");
    Serial.print("VelZ :");
    Serial.print("\t");
    Serial.println(quadcopter_position.getVelocityZ());
}
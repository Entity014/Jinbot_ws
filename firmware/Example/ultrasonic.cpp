#include <Arduino.h>
#include <NewPing.h>

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar(4, 5, MAX_DISTANCE);

void setup()
{
    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop()
{
    Serial.print("=");
    Serial.print(sonar.ping_cm());
    Serial.print("cm ");
    Serial.println();
}

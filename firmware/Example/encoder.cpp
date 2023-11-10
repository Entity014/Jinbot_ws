#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder(600);
float rpm;

void setup()
{

    Serial.begin(115200);
    // Enable the weak pull down resistors

    // ESP32Encoder::useInternalWeakPullResistors=DOWN;
    //  Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // use pin 19 and 18 for the first encoder
    encoder.attachSingleEdge(14, 13);
}

void loop()
{
    rpm = encoder.getRPM();
    // Loop and read the count
    Serial.println("Encoder count = " + String(rpm));
    delay(100);
}

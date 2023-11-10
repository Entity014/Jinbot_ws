#include <Arduino.h>

int fsrPin = 25;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println(analogRead(fsrPin));
}
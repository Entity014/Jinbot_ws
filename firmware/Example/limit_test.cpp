#include <Arduino.h>
#include <config_flag.h>

void setup()
{
    Serial.begin(115200);
    pinMode(LIMIT1, INPUT_PULLDOWN);
    pinMode(LIMIT2, INPUT_PULLDOWN);
    pinMode(LIMIT3, INPUT_PULLDOWN);
}

void loop()
{
    Serial.print(!digitalRead(LIMIT1));
    Serial.print(" ");
    Serial.print(!digitalRead(LIMIT2));
    Serial.print(" ");
    Serial.print(!digitalRead(LIMIT3));
    Serial.println("");
}
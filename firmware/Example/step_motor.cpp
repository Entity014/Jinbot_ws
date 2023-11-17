#include <Arduino.h>
#include <AccelStepper.h>
#include "config_flag.h"

AccelStepper step1(AccelStepper::DRIVER, STEP1_PWM, STEP1_DIR);

float speed = 1000;
void setup()
{
    Serial.begin(115200);
    // Sets the two pins as Outputs
    step1.setMaxSpeed(speed);
    step1.setAcceleration(speed * 2);
    step1.moveTo(8000);
}
void loop()
{
    Serial.println(step1.speed());
    step1.runSpeedToPosition();
}
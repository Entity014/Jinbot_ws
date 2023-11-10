#include <Arduino.h>
#include <AccelStepper.h>

const int dirPin = 3;
const int stepPin = 4;
int i = 0;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup()
{
    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(100);
    stepper.moveTo(400);
}

void loop()
{
    if (stepper.distanceToGo() == 0)
    {
        i++;
        stepper.moveTo(400 * i);
    }

    stepper.run();
}

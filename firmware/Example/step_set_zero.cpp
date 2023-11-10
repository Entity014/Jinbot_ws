#include <Arduino.h>
#include <AccelStepper.h>

const int dirPin = 3;
const int stepPin = 4;
const int limit_s0 = 0;
int pre_limit = -1;
int state_limit = 0;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

int lim_switch()
{
    return not(digitalRead(limit_s0));
}

void setup()
{
    pinMode(limit_s0, INPUT_PULLUP);
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(1000);
    stepper.move(1e5);
}

void loop()
{
    if (pre_limit != lim_switch())
        pre_limit = lim_switch();

    if (lim_switch() && state_limit < 1)
    {
        stepper.setSpeed(0);
        stepper.setCurrentPosition(0);
        state_limit++;
    }

    if (state_limit == 1)
    {
        delay(1000);
        stepper.setSpeed(1000);
        stepper.move(2500);
        state_limit++;
    }
    else if (state_limit == 2 && stepper.distanceToGo() == 0)
    {
        stepper.runToNewPosition(0);
        state_limit++;
    }
    else if (state_limit == 3 && stepper.distanceToGo() == 0)
    {
        stepper.setSpeed(0);
        state_limit++;
    }

    stepper.run();
}

#include <Arduino.h>
#include <task_flag.h>
#include <config_flag.h>
#define TAG "main"

void setup()
{
    task_microros_init();
    task_arduino_init();
}

void loop()
{
}

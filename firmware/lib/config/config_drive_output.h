#ifndef CONFIG_DRIVE_OUTPUT_H
#define CONFIG_DRIVE_OUTPUT_H

#define LED1_PIN 32
#define LED2_PIN 33
#define RLED1_PIN 21
#define GLED1_PIN 22
#define BLED1_PIN 23
#define RLED2_PIN 25
#define GLED2_PIN 26
#define BLED2_PIN 27

#define USE_PRIK_KEE_NOO_DRIVER // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE)

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
#define PWM_BITS 10         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000 // PWM Frequency

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV false
#define MOTOR3_INV true
#define MOTOR4_INV false

#ifdef USE_PRIK_KEE_NOO_DRIVER
#define MOTOR1_PWM 2
#define MOTOR1_IN_A 0
#define MOTOR1_IN_B 18

#define MOTOR2_PWM 5
#define MOTOR2_IN_A 19
#define MOTOR2_IN_B 4

#define MOTOR3_PWM 14
#define MOTOR3_IN_A 12
#define MOTOR3_IN_B 13

#define MOTOR4_PWM 17
#define MOTOR4_IN_A 15
#define MOTOR4_IN_B 16

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
#endif

#endif
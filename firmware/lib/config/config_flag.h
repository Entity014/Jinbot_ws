#ifndef CONFIG_FLAG_H
#define CONFIG_FLAG_H

#define LED1_PIN 16
#define LED2_PIN 17

#define USE_PARALLEL_3DOF // parallel gripper 3 degree of freedom

#define STEP1_SPEED 100
#define STEP2_SPEED 100
#define STEP3_SPEED 100
#define LENGTH_A
#define LENGTH_B
#define LENGTH_C
#define LENGTH_D
#define LENGTH_E

// servo pin
#define SERVO1 0
#define SERVO2 2

// step motor pin
#define STEP1_PWM 4
#define STEP1_DIR 1

#define STEP2_PWM 5
#define STEP2_DIR 4

#define STEP3_PWM 12
#define STEP3_DIR 13

// ultrasonic pin
#define TRIGER 14
#define ECHO 15

// limit switch pin
#define LIMIT1 34
#define LIMIT2 35
#define LIMIT3 36

#endif
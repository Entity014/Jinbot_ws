#ifndef CONFIG_FLAG_H
#define CONFIG_FLAG_H

#define LED1_PIN 32
#define LED2_PIN 33

#define USE_PARALLEL_3DOF // parallel gripper 3 degree of freedom

#define STEP1_SPEED 40000
#define STEP2_SPEED 40000
#define STEP3_SPEED 43400
#define LENGTH_A 0.293
#define LENGTH_B 0.352
#define LENGTH_C 0.352
#define LENGTH_D 0.293
#define LENGTH_E 0.080
#define LENGTH_F 0.127

// servo pin
#define SERVO1 2
#define SERVO2 16

// step motor pin
#define STEP1_PWM 5
#define STEP1_DIR 4

#define STEP2_PWM 13
#define STEP2_DIR 12

#define STEP3_PWM 15
#define STEP3_DIR 14

// limit switch pin
#define LIMIT1 18
#define LIMIT2 21
#define LIMIT3 35

// type definitions
#define TASK_MICROROS_PRIO 1
#define TASK_MICROROS_CORE 1
#define TASK_MICROROS_BYTES (int)pow(2, 14) // 12 -> 4096
#define TASK_ARDUINO_PRIO 0
#define TASK_ARDUINO_CORE 0
#define TASK_ARDUINO_BYTES (int)pow(2, 12) // 11 -> 2048

#endif
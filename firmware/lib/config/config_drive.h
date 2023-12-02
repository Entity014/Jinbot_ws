#ifndef CONFIG_DRIVE_H
#define CONFIG_DRIVE_H

#define LED1_PIN 27
#define LED2_PIN 32

#define BASE MECANUM // Mecanum drive robot

#define USE_PRIK_KEE_NOO_DRIVER // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE)

#define USE_GY87_IMU

#define K_P 0.0
#define K_I 0.0
#define K_D 0.0

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
#define MOTOR_MAX_RPM 600               // motor's max RPM
#define MAX_RPM_RATIO 1.0               // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define WHEEL_DIAMETER 0.048            // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.785        // distance between left and right wheels
#define PWM_BITS 10                     // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000             // PWM Frequency

#define COUNTS_PER_REV1 600 // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 600 // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 600 // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 0   // wheel4 encoder's no of ticks per rev

// ENCODER PINS
#define MOTOR1_ENCODER_A 18
#define MOTOR1_ENCODER_B 19

#define MOTOR2_ENCODER_A 23
#define MOTOR2_ENCODER_B 25

#define MOTOR3_ENCODER_A 36
#define MOTOR3_ENCODER_B 39

#define MOTOR4_ENCODER_A -1
#define MOTOR4_ENCODER_B -1

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

#ifdef USE_PRIK_KEE_NOO_DRIVER
#define MOTOR1_PWM 2
#define MOTOR1_IN_A 0
#define MOTOR1_IN_B 1

#define MOTOR2_PWM 5
#define MOTOR2_IN_A 3
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
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1 / 60

class Kinematics
{
public:
    enum base
    {
        MECANUM
    };

    base base_platform_;

    struct rpm
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct pwm
    {
        int motor1;
        int motor2;
        int motor3;
        int motor4;
    };
    Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
               float motor_operating_voltage, float motor_power_max_voltage,
               float wheel_diameter, float wheels_y_distance);
    velocities getVelocities(float rpm1, float rpm2, float rpm3);
    rpm getRPM(float linear_x, float linear_y, float angular_z);
    float getMaxRPM();

private:
    rpm calculateRPM(float linear_x, float linear_y, float angular_z);
    int getTotalWheels(base robot_base);

    float max_rpm_;
    float wheels_y_distance_;
    float pwm_res_;
    float wheel_circumference_;
    int total_wheels_;
};

#endif
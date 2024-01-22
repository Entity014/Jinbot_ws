#include "Arduino.h"
#include "kinematics.h"

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance, float min_pwm,
                       float max_pwm) : base_platform_(robot_base),
                                        wheels_y_distance_(wheels_y_distance),
                                        wheel_circumference_(PI * wheel_diameter),
                                        total_wheels_(getTotalWheels(robot_base)),
                                        min_pwm_(min_pwm),
                                        max_pwm_(max_pwm)
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

    // convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    // convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    // calculate the scale value how much each target velocity
    // must be scaled down in such cases where the total required RPM
    // is more than the motor's max RPM
    // this is to ensure that the required motion is achieved just with slower speed
    if (xy_sum >= max_rpm_ && angular_z == 0)
    {
        float vel_scaler = max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }

    else if (xtan_sum >= max_rpm_ && linear_y == 0)
    {
        float vel_scaler = max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;

    // calculate for the target encoder RPM and direction
    // left encoder
    rpm.motor1 = x_rpm + tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    // right encoder
    rpm.motor2 = x_rpm - tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    // rear encoder
    rpm.motor3 = y_rpm;
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    return rpm;
}

Kinematics::pwm Kinematics::calculatePWM(float pwm1, float pwm2, float pwm3)
{
    Kinematics::pwm pwm;
    pwm.motor1 = pwm2 - pwm3;
    pwm.motor1 = constrain(pwm.motor1, min_pwm_, max_pwm_);

    pwm.motor2 = pwm1 + pwm3;
    pwm.motor2 = constrain(pwm.motor2, min_pwm_, max_pwm_);

    pwm.motor3 = pwm2 + pwm3;
    pwm.motor3 = constrain(pwm.motor3, min_pwm_, max_pwm_);

    pwm.motor4 = pwm1 - pwm3;
    pwm.motor4 = constrain(pwm.motor4, min_pwm_, max_pwm_);

    return pwm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    return calculateRPM(linear_x, linear_y, angular_z);
}

Kinematics::pwm Kinematics::getPWM(float pwm1, float pwm2, float pwm3)
{
    return calculatePWM(pwm1, pwm2, pwm3);
}

Kinematics::velocities Kinematics::getVelocities(float rpm1, float rpm2, float rpm3)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / 2) / 60.0;   // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    // convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = (float)rpm3 / 60.0; // RPM
    if (base_platform_ == MECANUM)
        vel.linear_y = average_rps_y * wheel_circumference_; // m/s
    else
        vel.linear_y = 0;

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(rpm1 - rpm2) / 2) / 60.0;
    vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch (robot_base)
    {
    case MECANUM:
        return 4;
    default:
        return 2;
    }
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}
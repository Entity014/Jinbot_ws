#include <Arduino.h>
#include "gripper_kinematics.h"

Parallel_3dof::Parallel_3dof(float length_a, float length_b, float length_c, float length_d, float length_e, float length_f) : length_a_(length_a),
                                                                                                                               length_b_(length_b),
                                                                                                                               length_c_(length_c),
                                                                                                                               length_d_(length_d),
                                                                                                                               length_e_(length_e),
                                                                                                                               length_f_(length_f)
{
}

float Parallel_3dof::inverseFormular(float point_a, float point_b, float point_c, bool invert = false)
{
    float u = !invert ? (point_b + sqrt(pow(point_a, 2) + pow(point_b, 2) - pow(point_c, 2))) / (point_c + point_a) : (point_b - sqrt(pow(point_a, 2) + pow(point_b, 2) - pow(point_c, 2))) / (point_c + point_a);
    return (2 * atan(u)) * RAD_TO_DEG;
}

float Parallel_3dof::toRangeDegree(float degree)
{
    return degree > 180 ? -(360 - degree) : degree;
}

Parallel_3dof::angular Parallel_3dof::inverseKinematics(float position_x, float position_y)
{
    // Angular 1
    float point1_a = 2 * position_x * length_a_;
    float point1_b = 2 * position_y * length_a_;
    float point1_c = pow(position_x, 2) + pow(position_y, 2) + pow(length_a_, 2) - pow(length_b_, 2);

    // Angular 2
    float point2_a = 2 * length_d_ * (position_x - length_e_);
    float point2_b = 2 * position_y * length_d_;
    float point2_c = pow(position_x - length_e_, 2) + pow(position_y, 2) + pow(length_d_, 2) - pow(length_c_, 2);

    Parallel_3dof::angular angular;
    angular.angular_a = toRangeDegree(180 - inverseFormular(point1_a, point1_b, point1_c));
    angular.angular_b = inverseFormular(point2_a, point2_b, point2_c, true);
    // angular.angular_c = constrain(72 + (0.95 * asin((position_y - (length_a_ * sin(angular.angular_a * DEG_TO_RAD))) / length_b_) * RAD_TO_DEG), 0, 180);
    return angular;
}

Parallel_3dof::angular Parallel_3dof::getAngular(float position_x, float position_y)
{
    position_y += 0.012;
    position_x += 0.04;
    return inverseKinematics(position_x, position_y);
}

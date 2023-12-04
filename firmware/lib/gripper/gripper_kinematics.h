#ifndef GRIPPER_KINEMATICS_H
#define GRIPPER_KINEMATICS_H

class Parallel_3dof
{
public:
    struct angular
    {
        float angular_a;
        float angular_b;
    };

    Parallel_3dof(float length_a, float length_b, float length_c, float length_d, float length_e, float length_f);
    angular getAngular(float position_x, float position_y);

private:
    angular inverseKinematics(float position_x, float position_y);
    float inverseFormular(float point_a, float point_b, float point_c, bool invert);
    float length_a_;
    float length_b_;
    float length_c_;
    float length_d_;
    float length_e_;
    float length_f_;
};

#endif
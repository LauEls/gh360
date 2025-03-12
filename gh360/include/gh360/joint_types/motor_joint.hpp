#ifndef MOTOR_JOINT_HPP_
#define MOTOR_JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <math.h>

#include "gh360/joint_types/joint.hpp"
#include "gh360/motor_dictionaries/mx_106_dict.hpp"
#include "gh360/motor_dictionaries/mx_64_dict.hpp"

class MotorJoint: public Joint
{
    public:
        MotorJoint();
        virtual ~MotorJoint();

        void set_joint_angle(double new_joint_angle);
        void set_joint_velocity(double new_joint_velocity);

        double get_joint_angle();
        double get_joint_velocity();

};

#endif // MOTOR_JOINT_HPP_
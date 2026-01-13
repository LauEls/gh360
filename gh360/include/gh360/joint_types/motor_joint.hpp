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

        /**
         * @brief This function is currently not used, since the joint angle is retrieved from the motor
         */
        void set_joint_angle(double new_joint_angle);

        /**
         * @brief This function is currently not used, since the joint velocity is retrieved from the motor
         */
        void set_joint_velocity(double new_joint_velocity);

        /**
         * @brief Returns the angle of the joint which is the same as the present position of the motor
         * @return The angle of the joint in radians
         */
        double get_joint_angle();

        /**
         * @brief Returns the velocity of the joint which is the same as the present velocity of the motor
         * @return The velocity of the joint in rad/s
         */
        double get_joint_velocity();

};

#endif // MOTOR_JOINT_HPP_
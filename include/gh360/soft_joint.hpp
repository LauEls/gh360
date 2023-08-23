#ifndef SOFT_JOINT_HPP_
#define SOFT_JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>

#include "joint.hpp"
#include "mx_106_dict.hpp"
#include "mx_64_dict.hpp"

class SoftJoint: public Joint
{
    public:
        SoftJoint();
        virtual ~SoftJoint();

        void set_joint_name(std::string name);
        void set_right_motor_id(int motor_id);
        void set_left_motor_id(int motor_id);
        void set_right_action_id(int action_id);
        void set_left_action_id(int action_id);
        void set_right_movement_direction(int movement_direction);
        void set_left_movement_direction(int movement_direction);
        void set_right_motor_model(gh360::MotorDictionary* motor_model);
        void set_left_motor_model(gh360::MotorDictionary* motor_model);
        void set_right_motor_present_position(int position);
        void set_left_motor_present_position(int position);

        std::string get_joint_name();
        std::string get_joint_type();
        int get_right_motor_id();
        int get_left_motor_id();
        int get_right_action_id();
        int get_left_action_id();
        int get_right_movement_direction();
        int get_left_movement_direction();
        gh360::MotorDictionary* get_right_motor_model();
        gh360::MotorDictionary* get_left_motor_model();
        double get_right_motor_present_position();
        double get_left_motor_present_position();

    private:
        std::string joint_type;
        int right_motor_id;
        int left_motor_id;
        int right_action_id;
        int left_action_id;
        int right_movement_direction;
        int left_movement_direction;
        gh360::MotorDictionary* right_motor_model;
        gh360::MotorDictionary* left_motor_model;
        double right_motor_present_position;
        double left_motor_present_position;


};

#endif // SOFT_JOINT_HPP_
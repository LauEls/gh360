#ifndef MOTOR_JOINT_HPP_
#define MOTOR_JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <math.h>

#include "joint.hpp"
#include "mx_106_dict.hpp"
#include "mx_64_dict.hpp"

class MotorJoint: public Joint
{
    public:
        MotorJoint();
        virtual ~MotorJoint();

        void set_joint_name(std::string name);
        void set_motor_id(int motor_id);
        void set_action_id(int action_id);
        void set_movement_direction(int movement_direction);
        void set_motor_model(gh360::MotorDictionary* motor_model);
        void set_motor_present_position(int position);
        void set_motor_goal_position(double goal_pos);
        void set_motor_goal_velocity(double goal_vel);
        void set_motor_goal_current(double goal_current);
        void set_torque_enabled(bool torque);
        void set_motor_status(int data, uint8_t address);
        void set_offset(double offset);

        std::string get_joint_name();
        std::string get_joint_type();
        double get_joint_angle();
        void set_joint_angle(double new_joint_angle);

        int get_motor_id();
        int get_action_id();
        int get_movement_direction();
        gh360::MotorDictionary* get_motor_model();
        double get_motor_present_position();
        double get_motor_present_velocity();
        double get_motor_present_current();
        double get_motor_present_temperature();
        double get_motor_goal_position();
        int get_motor_goal_position_int();
        double get_motor_goal(uint8_t address);
        int get_motor_goal_int(uint8_t address);
        bool get_torque_enabled();

    private:
        std::string joint_type;
        int motor_id;
        int action_id;
        int movement_direction;
        gh360::MotorDictionary* motor_model;
        
        // Motor Position in rad
        double motor_present_position;
        double motor_goal_position;
        // Motor Velocity in rpm
        double motor_present_velocity;
        double motor_goal_velocity;
        // Motor Current in mA
        double motor_present_current;
        double motor_goal_current;
        // Motor Temperature in degrees
        double motor_present_temperature;
        double offset;

        bool motor_init_pos = true;
        bool motor_torque_enabled = false;


};

#endif // MOTOR_JOINT_HPP_
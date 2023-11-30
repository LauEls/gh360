#ifndef SOFT_JOINT_HPP_
#define SOFT_JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <math.h>

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
        void set_right_offset(double offset);
        void set_left_offset(double offset);
        void set_right_motor_model(gh360::MotorDictionary* motor_model);
        void set_left_motor_model(gh360::MotorDictionary* motor_model);
        void set_right_motor_present_position(int position);
        void set_left_motor_present_position(int position);
        void set_right_motor_goal_position(double goal_pos);
        void set_left_motor_goal_position(double goal_pos);
        void set_right_motor_status(int data, uint8_t address);
        void set_left_motor_status(int data, uint8_t address);
        void set_right_reference_current(double data);
        void set_left_reference_current(double data);
        void set_right_reference_position(double data);
        void set_left_reference_position(double data);
        void set_reference_joint_angle(double data);
        void set_initialize(bool init);

        std::string get_joint_name();
        std::string get_joint_type();
        double get_joint_angle();
        void set_joint_angle(double new_angle);

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
        double get_right_motor_present_velocity();
        double get_left_motor_present_velocity();
        double get_right_motor_present_current();
        double get_left_motor_present_current();
        double get_right_motor_present_temperature();
        double get_left_motor_present_temperature();
        double get_right_motor_goal_position();
        double get_left_motor_goal_position();
        int get_right_motor_goal_position_int();
        int get_left_motor_goal_position_int();
        double get_right_motor_goal(uint8_t address);
        double get_left_motor_goal(uint8_t address);
        int get_right_motor_goal_int(uint8_t address);
        int get_left_motor_goal_int(uint8_t address);
        double get_right_reference_current();
        double get_left_reference_current();
        double get_right_reference_position();
        double get_left_reference_position();
        double get_reference_joint_angle();
        bool get_initialize();
        double get_right_offset();
        double get_left_offset();

        bool right_motor_goal_reached();
        bool left_motor_goal_reached();

    private:
        std::string joint_type;
        int right_motor_id;
        int left_motor_id;
        int right_action_id;
        int left_action_id;
        int right_movement_direction;
        int left_movement_direction;
        double right_offset;
        double left_offset;
        gh360::MotorDictionary* right_motor_model;
        gh360::MotorDictionary* left_motor_model;
        // Motor Position in rad
        double right_motor_present_position;
        double left_motor_present_position;
        double right_motor_goal_position;
        double left_motor_goal_position;
        // Motor Velocity in rpm
        double right_motor_present_velocity;
        double left_motor_present_velocity;
        double right_motor_goal_velocity;
        double left_motor_goal_velocity;
        // Motor Current in mA
        double right_motor_present_current;
        double left_motor_present_current;
        double right_motor_goal_current;
        double left_motor_goal_current;
        // Motor Temperature in degrees
        double right_motor_present_temperature;
        double left_motor_present_temperature;

        double right_reference_current;
        double left_reference_current;
        double right_reference_position;
        double left_reference_position;
        double reference_joint_angle;
        
        bool right_motor_init_pos = true;
        bool left_motor_init_pos = true;
        bool initialize = true;

        


};

#endif // SOFT_JOINT_HPP_
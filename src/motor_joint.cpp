#include "motor_joint.hpp"

MotorJoint::MotorJoint()
{

}

MotorJoint::~MotorJoint()
{

}

void MotorJoint::set_joint_name(std::string name)
{
    this->joint_name = name;
}

void MotorJoint::set_motor_id(int motor_id)
{
    this->motor_id = motor_id;
}

void MotorJoint::set_action_id(int action_id)
{
    this->action_id = action_id;
}

void MotorJoint::set_movement_direction(int movement_direction)
{
    this->movement_direction = movement_direction;
}

void MotorJoint::set_motor_model(gh360::MotorDictionary* motor_model)
{
    this->motor_model = motor_model;
}

void MotorJoint::set_motor_present_position(int position)
{
    double motor_pos = position * (2*M_PI/4096);
    this->motor_present_position = motor_pos;
}

void MotorJoint::set_motor_goal_position(double goal_pos)
{
    this->motor_goal_pos = goal_pos;
}

std::string MotorJoint::get_joint_name()
{
    return this->joint_name;
}

std::string MotorJoint::get_joint_type()
{
    return this->joint_type;
}

int MotorJoint::get_motor_id()
{
    return this->motor_id;
}

int MotorJoint::get_action_id()
{
    return this->action_id;
}

int MotorJoint::get_movement_direction()
{
    return this->movement_direction;
}

gh360::MotorDictionary* MotorJoint::get_motor_model()
{
    return this->motor_model;
}

double MotorJoint::get_motor_present_position()
{
    return this->motor_present_position;
}

double MotorJoint::get_motor_goal_position()
{
    return this->motor_goal_pos;
}

int MotorJoint::get_motor_goal_position_int()
{
    int goal_pos_int = this->motor_goal_pos / (2*M_PI/4096);
    return goal_pos_int;
}
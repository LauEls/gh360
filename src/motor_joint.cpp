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
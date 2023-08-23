#include "soft_joint.hpp"


SoftJoint::SoftJoint() 
{
    this->joint_type = "soft_joint";
}

SoftJoint::~SoftJoint() 
{

}

void SoftJoint::set_joint_name(std::string name) 
{
    this->joint_name = name;
}

void SoftJoint::set_right_motor_id(int motor_id)
{
    this->right_motor_id = motor_id;
}

void SoftJoint::set_left_motor_id(int motor_id)
{
    this->left_motor_id = motor_id;
}

void SoftJoint::set_right_action_id(int action_id)
{
    this->right_action_id = action_id;
}

void SoftJoint::set_left_action_id(int action_id)
{
    this->left_action_id = action_id;
}

void SoftJoint::set_right_movement_direction(int movement_direction)
{
    this->right_movement_direction = movement_direction;
}

void SoftJoint::set_left_movement_direction(int movement_direction)
{
    this->left_movement_direction = movement_direction;
}

void SoftJoint::set_right_motor_model(gh360::MotorDictionary* motor_model)
{
    this->right_motor_model = motor_model;
}

void SoftJoint::set_left_motor_model(gh360::MotorDictionary* motor_model)
{
    this->left_motor_model = motor_model;
}

void SoftJoint::set_right_motor_present_position(int position)
{
    double motor_pos = position * 0.088;
}

void SoftJoint::set_left_motor_present_position(int position)
{

}

std::string SoftJoint::get_joint_name()
{
    return this->joint_name;
}

std::string SoftJoint::get_joint_type()
{
    return this->joint_type;
}

int SoftJoint::get_right_motor_id()
{
    return this->right_motor_id;
}

int SoftJoint::get_left_motor_id()
{
    return this->left_motor_id;
}

int SoftJoint::get_right_action_id()
{
    return this->right_action_id;
}

int SoftJoint::get_left_action_id()
{
    return this->left_action_id;
}

int SoftJoint::get_right_movement_direction()
{
    return this->right_movement_direction;
}

int SoftJoint::get_left_movement_direction()
{
    return this->left_movement_direction;
}

gh360::MotorDictionary* SoftJoint::get_right_motor_model()
{
    return this->right_motor_model;
}

gh360::MotorDictionary* SoftJoint::get_left_motor_model()
{
    return this->left_motor_model;
}
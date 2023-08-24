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
    double motor_pos = position * (2*M_PI/4096);
    this->right_motor_present_position = motor_pos;
}

void SoftJoint::set_left_motor_present_position(int position)
{
    double motor_pos = position * (2*M_PI/4096);
    this->left_motor_present_position = motor_pos;
}

void SoftJoint::set_right_motor_goal_position(double goal_pos)
{
    this->right_motor_goal_position = goal_pos;
}

void SoftJoint::set_left_motor_goal_position(double goal_pos)
{
    this->left_motor_goal_position = goal_pos;
}

void SoftJoint::set_right_motor_status(int data, uint8_t address)
{
    if (address == this->right_motor_model->Present_Position.address) 
    {
        this->right_motor_present_position = data * (2*M_PI/4096);
    }
    else if (address == this->right_motor_model->Present_Velocity.address) 
    {
        this->right_motor_present_velocity = data * 0.229;
    }
    else if (address == this->right_motor_model->Present_Current.address) 
    {
        this->right_motor_present_current = data * 3.36;
    }
    else if (address == this->right_motor_model->Present_Temperature.address)
    {
        this->right_motor_present_temperature = data;
    }
    
}

void SoftJoint::set_left_motor_status(int data, uint8_t address)
{
    if (address == this->left_motor_model->Present_Position.address) 
    {
        this->left_motor_present_position = data * (2*M_PI/4096);
    }
    else if (address == this->left_motor_model->Present_Velocity.address) 
    {
        this->left_motor_present_velocity = data * 0.229;
    }
    else if (address == this->left_motor_model->Present_Current.address) 
    {
        this->left_motor_present_current = data * 3.36;
    }
    else if (address == this->left_motor_model->Present_Temperature.address)
    {
        this->left_motor_present_temperature = data;
    }
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

double SoftJoint::get_right_motor_present_position()
{
    return this->right_motor_present_position;
}

double SoftJoint::get_left_motor_present_position()
{
    return this->left_motor_present_position;
}

double SoftJoint::get_right_motor_goal_position()
{
    return this->right_motor_goal_position;
}

double SoftJoint::get_left_motor_goal_position()
{
    return this->left_motor_goal_position;
}

int SoftJoint::get_right_motor_goal_position_int()
{
    int goal_pos_int = this->right_motor_goal_position / (2*M_PI/4096);
    return goal_pos_int;
}

int SoftJoint::get_left_motor_goal_position_int()
{
    int goal_pos_int = this->left_motor_goal_position / (2*M_PI/4096);
    return goal_pos_int;
}

double SoftJoint::get_right_motor_goal(uint8_t address)
{
    if (address == this->right_motor_model->Goal_Position.address) 
    {
        return this->right_motor_goal_position;
    }
    else if (address == this->right_motor_model->Goal_Velocity.address) 
    {
        return this->right_motor_goal_velocity;
    }
    else if (address == this->right_motor_model->Goal_Current.address) 
    {
        return this->right_motor_goal_current;
    }

    return 0.0;
}

double SoftJoint::get_left_motor_goal(uint8_t address)
{
    if (address == this->left_motor_model->Goal_Position.address) 
    {
        return this->left_motor_goal_position;
    }
    else if (address == this->left_motor_model->Goal_Velocity.address) 
    {
        return this->left_motor_goal_velocity;
    }
    else if (address == this->left_motor_model->Goal_Current.address) 
    {
        return this->left_motor_goal_current;
    }

    return 0.0;
}

int SoftJoint::get_right_motor_goal_int(uint8_t address)
{
    if (address == this->right_motor_model->Goal_Position.address) 
    {
        int goal_pos_int = this->right_motor_goal_position / (2*M_PI/4096);
        return goal_pos_int;
    }
    else if (address == this->right_motor_model->Goal_Velocity.address) 
    {
        int goal_vel_int = this->right_motor_goal_velocity / 0.229;
        return goal_vel_int;
    }
    else if (address == this->right_motor_model->Goal_Current.address) 
    {
        int goal_current_int = this->right_motor_goal_current / 3.36;
        return goal_current_int;
    }
    
    return 0;
}

int SoftJoint::get_left_motor_goal_int(uint8_t address)
{
    if (address == this->left_motor_model->Goal_Position.address) 
    {
        int goal_pos_int = this->left_motor_goal_position / (2*M_PI/4096);
        return goal_pos_int;
    }
    else if (address == this->left_motor_model->Goal_Velocity.address) 
    {
        int goal_vel_int = this->left_motor_goal_velocity / 0.229;
        return goal_vel_int;
    }
    else if (address == this->left_motor_model->Goal_Current.address) 
    {
        int goal_current_int = this->left_motor_goal_current / 3.36;
        return goal_current_int;
    }
    
    return 0;
}
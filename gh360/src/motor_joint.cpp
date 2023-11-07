#include "motor_joint.hpp"

MotorJoint::MotorJoint()
{
    this->joint_type = "motor_joint";
    this->motor_init_pos = true;
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

    if (this->motor_init_pos == true)
    {
        this->motor_goal_position = this->motor_present_position;
        this->motor_init_pos = false;
    }
}

void MotorJoint::set_motor_goal_position(double goal_pos)
{
    this->motor_goal_position = goal_pos;
}

void MotorJoint::set_motor_status(int data, uint8_t address)
{
    if (address == this->motor_model->Present_Position.address) 
    {
        this->motor_present_position = data * (2*M_PI/4096);
        if (this->motor_init_pos == true)
        {
            this->motor_goal_position = this->motor_present_position;
            this->motor_init_pos = false;
        }
    }
    else if (address == this->motor_model->Present_Velocity.address) 
    {
        this->motor_present_velocity = data * 0.229;
    }
    else if (address == this->motor_model->Present_Current.address) 
    {
        // this->motor_present_current = data * 3.36;
        if (data > 0x7fff)
        {
            this->motor_present_current = (data - 65536) * 3.36;
        }
        else
        {
            this->motor_present_current = data * 3.36;
        }
    }
    else if (address == this->motor_model->Present_Temperature.address)
    {
        this->motor_present_temperature = data;
    }
    
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

double MotorJoint::get_motor_present_velocity()
{
    return this->motor_present_velocity;
}

double MotorJoint::get_motor_present_current()
{
    return this->motor_present_current;
}

double MotorJoint::get_motor_present_temperature()
{
    return this->motor_present_temperature;
}

double MotorJoint::get_motor_goal_position()
{
    return this->motor_goal_position;
}

int MotorJoint::get_motor_goal_position_int()
{
    int goal_pos_int = this->motor_goal_position / (2*M_PI/4096);
    return goal_pos_int;
}

double MotorJoint::get_motor_goal(uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        return this->motor_goal_position;
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        return this->motor_goal_velocity;
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        return this->motor_goal_current;
    }

    return 0.0;
}

int MotorJoint::get_motor_goal_int(uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        int goal_pos_int = this->motor_goal_position / (2*M_PI/4096);
        return goal_pos_int;
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        int goal_vel_int = this->motor_goal_velocity / 0.229;
        return goal_vel_int;
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        int goal_current_int = this->motor_goal_current / 3.36;
        return goal_current_int;
    }
    
    return 0;
}
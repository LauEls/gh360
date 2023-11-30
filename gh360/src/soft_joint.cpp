#include "soft_joint.hpp"


SoftJoint::SoftJoint() 
{
    this->joint_type = "soft_joint";
    this->right_motor_init_pos = true;
    this->left_motor_init_pos = true;
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

void SoftJoint::set_right_offset(double offset)
{
    this->right_offset = offset;
}

void SoftJoint::set_left_offset(double offset)
{
    this->left_offset = offset;
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
    // double motor_pos = position * (2*M_PI/4096);
    double motor_pos = positionIntToDouble(position);
    this->right_motor_present_position = motor_pos;

    if (this->right_motor_init_pos == true)
    {
        this->right_motor_goal_position = this->right_motor_present_position;
        this->right_motor_init_pos = false;
    }
}

void SoftJoint::set_left_motor_present_position(int position)
{
    // double motor_pos = position * (2*M_PI/4096);
    double motor_pos = positionIntToDouble(position);
    this->left_motor_present_position = motor_pos;

    if (this->left_motor_init_pos == true)
    {
        this->left_motor_goal_position = this->left_motor_present_position;
        this->left_motor_init_pos = false;
    }
}

void SoftJoint::set_right_motor_goal_position(double goal_pos)
{
    // this->right_motor_goal_position = (goal_pos - this->right_offset) * this->right_movement_direction;
    this->right_motor_goal_position = this->calc_set_motor_goal_pos(goal_pos, this->right_offset, this->right_movement_direction);
}

void SoftJoint::set_left_motor_goal_position(double goal_pos)
{
    // this->left_motor_goal_position = (goal_pos - this->left_offset) * this->left_movement_direction;
    this->left_motor_goal_position = this->calc_set_motor_goal_pos(goal_pos, this->left_offset, this->left_movement_direction);
}

void SoftJoint::set_right_motor_status(int data, uint8_t address)
{
    if (address == this->right_motor_model->Present_Position.address) 
    {
        // this->right_motor_present_position = data * (2*M_PI/4096);
        // this->right_motor_present_position = positionIntToDouble(data);
        // if (this->right_motor_init_pos == true)
        // {
        //     this->right_motor_goal_position = this->right_motor_present_position;
        //     this->right_motor_init_pos = false;
        // }
        this->set_right_motor_present_position(data);
    }
    else if (address == this->right_motor_model->Present_Velocity.address) 
    {
        // this->right_motor_present_velocity = data * 0.229 * 0.10472;
        this->right_motor_present_velocity = velocityIntToDouble(data);
    }
    else if (address == this->right_motor_model->Present_Current.address) 
    {
        // if (data > 0x7fff)
        // {
        //     this->right_motor_present_current = (data - 65536) * 3.36;
        // }
        // else
        // {
        //     this->right_motor_present_current = data * 3.36;
        // }
        this->right_motor_present_current = currentIntToDouble(data);
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
        // this->left_motor_present_position = data * (2*M_PI/4096);
        // this->left_motor_present_position = positionIntToDouble(data);
        // if (this->left_motor_init_pos == true)
        // {
        //     this->left_motor_goal_position = this->left_motor_present_position;
        //     this->left_motor_init_pos = false;
        // }
        this->set_left_motor_present_position(data);
    }
    else if (address == this->left_motor_model->Present_Velocity.address) 
    {
        // this->left_motor_present_velocity = data * 0.229;
        this->left_motor_present_velocity = velocityIntToDouble(data);
    }
    else if (address == this->left_motor_model->Present_Current.address) 
    {
        // if (data > 0x7fff)
        // {
        //     this->left_motor_present_current = (data - 65536) * 3.36;
        // }
        // else
        // {
        //     this->left_motor_present_current = data * 3.36;
        // }
        this->left_motor_present_current = currentIntToDouble(data);
    }
    else if (address == this->left_motor_model->Present_Temperature.address)
    {
        this->left_motor_present_temperature = data;
    }
}

void SoftJoint::set_initialize(bool init)
{
    this->initialize = init;
}

double SoftJoint::get_joint_angle()
{
    return this->joint_angle;
}

void SoftJoint::set_joint_angle(double new_angle)
{
    this->joint_angle = new_angle;
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
    // return this->right_motor_present_position / this->right_movement_direction + this->right_offset;
    return this->calc_get_motor_pos(this->right_motor_present_position, this->right_offset, this->right_movement_direction);
}

double SoftJoint::get_left_motor_present_position()
{
    // return this->left_motor_present_position / this->left_movement_direction + this->left_offset;
    return this->calc_get_motor_pos(this->left_motor_present_position, this->left_offset, this->left_movement_direction);
}

double SoftJoint::get_right_motor_present_velocity()
{
    // return this->right_motor_present_velocity;
    return this->calc_get_motor_vel(this->right_motor_present_velocity, this->right_movement_direction);
}

double SoftJoint::get_left_motor_present_velocity()
{
    // return this->left_motor_present_velocity;
    return this->calc_get_motor_vel(this->left_motor_present_velocity, this->left_movement_direction);
}

double SoftJoint::get_right_motor_present_current()
{
    // return this->right_motor_present_current;
    return this->calc_get_motor_current(this->right_motor_present_current, this->right_movement_direction);
}

double SoftJoint::get_left_motor_present_current()
{
    // return this->left_motor_present_current;
    return this->calc_get_motor_current(this->left_motor_present_current, this->left_movement_direction);
}

double SoftJoint::get_right_motor_present_temperature()
{
    return this->right_motor_present_temperature;
}

double SoftJoint::get_left_motor_present_temperature()
{
    return this->left_motor_present_temperature;
}

double SoftJoint::get_right_motor_goal_position()
{
    // return this->right_motor_goal_position;
    return this->calc_get_motor_pos(this->right_motor_goal_position, this->right_offset, this->right_movement_direction);
}

double SoftJoint::get_left_motor_goal_position()
{
    // return this->left_motor_goal_position;
    return this->calc_get_motor_pos(this->left_motor_goal_position, this->left_offset, this->left_movement_direction);
}

int SoftJoint::get_right_motor_goal_position_int()
{
    // int goal_pos_int = this->right_motor_goal_position / (2*M_PI/4096);
    int goal_pos_int = positionDoubleToInt(this->right_motor_goal_position);
    return goal_pos_int;
}

int SoftJoint::get_left_motor_goal_position_int()
{
    // int goal_pos_int = this->left_motor_goal_position / (2*M_PI/4096);
    int goal_pos_int = positionDoubleToInt(this->left_motor_goal_position);
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
        // int goal_pos_int = this->right_motor_goal_position / (2*M_PI/4096);
        int goal_pos_int = positionDoubleToInt(this->right_motor_goal_position);
        return goal_pos_int;
    }
    else if (address == this->right_motor_model->Goal_Velocity.address) 
    {
        // int goal_vel_int = this->right_motor_goal_velocity / 0.229;
        int goal_vel_int = velocityDoubleToInt(this->right_motor_goal_velocity);
        return goal_vel_int;
    }
    else if (address == this->right_motor_model->Goal_Current.address) 
    {
        // int goal_current_int = this->right_motor_goal_current / 3.36;
        int goal_current_int = currentDoubleToInt(this->right_motor_goal_current);
        return goal_current_int;
    }
    
    return 0;
}

int SoftJoint::get_left_motor_goal_int(uint8_t address)
{
    if (address == this->left_motor_model->Goal_Position.address) 
    {
        // int goal_pos_int = this->left_motor_goal_position / (2*M_PI/4096);
        int goal_pos_int = positionDoubleToInt(this->left_motor_goal_position);
        return goal_pos_int;
    }
    else if (address == this->left_motor_model->Goal_Velocity.address) 
    {
        // int goal_vel_int = this->left_motor_goal_velocity / 0.229;
        int goal_vel_int = velocityDoubleToInt(this->left_motor_goal_velocity);
        return goal_vel_int;
    }
    else if (address == this->left_motor_model->Goal_Current.address) 
    {
        // int goal_current_int = this->left_motor_goal_current / 3.36;
        int goal_current_int = currentDoubleToInt(this->left_motor_goal_current);
        return goal_current_int;
    }
    
    return 0;
}

void SoftJoint::set_right_reference_current(double data)
{
    this->right_reference_current = data;
}

void SoftJoint::set_left_reference_current(double data)
{
    this->left_reference_current = data;
}

void SoftJoint::set_right_reference_position(double data)
{
    this->right_reference_position = data;
}

void SoftJoint::set_left_reference_position(double data)
{
    this->left_reference_position = data;
}

void SoftJoint::set_reference_joint_angle(double data)
{
    this->reference_joint_angle = data;
}

double SoftJoint::get_right_reference_current()
{
    return this->right_reference_current;
}

double SoftJoint::get_left_reference_current()
{
    return this->left_reference_current;
}

double SoftJoint::get_right_reference_position()
{
    return this->right_reference_position;
}

double SoftJoint::get_left_reference_position()
{
    return this->left_reference_position;
}

double SoftJoint::get_reference_joint_angle()
{
    return this->reference_joint_angle;
}

bool SoftJoint::get_initialize()
{
    return this->initialize;
}

double SoftJoint::get_right_offset()
{
    return this->right_offset;
}

double SoftJoint::get_left_offset()
{
    return this->left_offset;
}

bool SoftJoint::right_motor_goal_reached()
{
    if ((abs(this->get_right_motor_present_position() - this->get_right_motor_goal_position()) > 0.1) || (this->get_right_motor_present_velocity() > 0.0)) 
    {
        return false;
    }

    return true;
}

bool SoftJoint::left_motor_goal_reached()
{
    if ((abs(this->get_left_motor_present_position() - this->get_left_motor_goal_position()) > 0.1) || (this->get_left_motor_present_velocity() > 0.0))
    {
        return false;
    }

    return true;
}
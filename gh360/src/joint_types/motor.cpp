#include "gh360/joint_types/motor.hpp"

Motor::Motor()
{
    this->first_pos_value = true;
}

Motor::~Motor()
{
}

void Motor::set_motor_id(int motor_id)
{
    this->motor_id = motor_id;
}

void Motor::set_movement_direction(int movement_direction)
{
    this->movement_direction = movement_direction;
}

void Motor::set_offset(double offset)
{
    this->offset = offset;
}

void Motor::set_motor_model(MotorDictionary *motor_model)
{
    this->motor_model = motor_model;
    this->max_current = this->motor_model->CURRENT_LIMIT;
    this->min_current = -this->motor_model->CURRENT_LIMIT;
}

void Motor::set_torque_enabled(bool torque)
{
    this->torque_enabled = torque;
}

void Motor::set_safety_check(bool safety_check)
{
    this->safety_check = safety_check;
}

void Motor::set_motor_state(int data, uint8_t address)
{
    if (address == this->motor_model->Present_Position.address) 
    {
        this->set_present_position(data);
    }
    else if (address == this->motor_model->Present_Velocity.address) 
    {
        this->set_present_velocity(data);
    }
    else if (address == this->motor_model->Present_Current.address) 
    {
        this->set_present_current(data);
    }
    else if (address == this->motor_model->Present_Temperature.address)
    {
        this->set_present_temperature(data);
    }
    else if (address == this->motor_model->Moving.address)
    {
        this->set_moving(data);
    }
}

void Motor::set_motor_goal_adjusted(double data)
{
    if (this->operating_mode == this->motor_model->OperatingMode::POSITION || this->operating_mode == this->motor_model->OperatingMode::EXTENDED_POSITION) 
    {
        this->set_goal_position_adjusted(data);
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::VELOCITY) 
    {
        this->set_goal_velocity_adjusted(data);
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::CURRENT) 
    {
        this->set_goal_current_adjusted(data);
    }
}

void Motor::set_motor_goal_adjusted(double data, uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        this->set_goal_position_adjusted(data);
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        this->set_goal_velocity_adjusted(data);
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        this->set_goal_current_adjusted(data);
    }
}

void Motor::set_motor_goal_adjusted(gh360_interfaces::msg::SetPosition motor_goal_msg)
{
    this->set_goal_position_adjusted(motor_goal_msg.position);
}

void Motor::set_motor_goal_adjusted(gh360_interfaces::msg::SetVelocity motor_goal_msg)
{
    this->set_goal_velocity_adjusted(motor_goal_msg.velocity);
}

void Motor::set_motor_goal_adjusted(gh360_interfaces::msg::SetCurrent motor_goal_msg)
{
    this->set_goal_current_adjusted(motor_goal_msg.current);
}

void Motor::set_present_position(int position)
{
    double motor_pos = this->positionIntToDouble(position);
    this->present_position = motor_pos;
    // this->joint_angle = motor_pos;

    if (this->first_pos_value == true)
    {
        this->goal_position = this->present_position;
        this->first_pos_value = false;
    }
}

void Motor::set_present_position_adjusted(double position)
{
    this->present_position = (position + this->offset) * this->movement_direction;
}

void Motor::set_present_velocity(int velocity)
{
    this->present_velocity = this->velocityIntToDouble(velocity);
    if (this->present_velocity != 0.0)
    {
        this->moving = true;
    }
    else
    {
        this->moving = false;
    }
}

void Motor::set_present_velocity_adjusted(double velocity)
{
    this->present_velocity = velocity * this->movement_direction;
    if (this->present_velocity != 0.0)
    {
        this->moving = true;
    }
    else
    {
        this->moving = false;
    }
}

void Motor::set_present_current(int current)
{
    this->present_current = this->currentIntToDouble(current);
}

void Motor::set_present_current_adjusted(double current)
{
    this->present_current = current * this->movement_direction;
}

void Motor::set_present_temperature(int temperature)
{
    this->present_temperature = temperature;
}

void Motor::set_moving(bool moving)
{
    this->moving = moving;
}

void Motor::set_goal_position(double goal_pos)
{
    this->goal_position = goal_pos;
}

void Motor::set_goal_position_adjusted(double goal_pos)
{
    this->goal_position = (goal_pos + this->offset) * this->movement_direction;
}

void Motor::set_reference_position(double ref_pos)
{
    this->reference_position = ref_pos;
}

void Motor::set_reference_position_adjusted(double ref_pos)
{
    this->reference_position = (ref_pos + this->offset) * this->movement_direction;
}

void Motor::set_goal_velocity(double goal_vel)
{
    this->goal_velocity = goal_vel;
}

void Motor::set_goal_velocity_adjusted(double goal_vel)
{
    this->goal_velocity = goal_vel * this->movement_direction;
}

void Motor::set_goal_current(double goal_current)
{
    this->goal_current = goal_current;
}

void Motor::set_goal_current_adjusted(double goal_current)
{
    this->goal_current = goal_current * this->movement_direction;
}

void Motor::set_reference_current(double ref_current)
{
    this->reference_current = ref_current;
}

void Motor::set_reference_current_adjusted(double ref_current)
{
    this->reference_current = ref_current * this->movement_direction;
}

void Motor::set_operating_mode(int operating_mode)
{
    this->operating_mode = operating_mode;
}

void Motor::set_max_current(double max_current)
{
    this->max_current = max_current;
}

void Motor::set_min_current(double min_current)
{
    this->min_current = min_current;
}

void Motor::set_max_velocity(double max_velocity)
{
    this->max_velocity = max_velocity;
}

void Motor::set_min_velocity(double min_velocity)
{
    this->min_velocity = min_velocity;
}

int Motor::get_motor_id()
{
    return this->motor_id;
}

int Motor::get_movement_direction()
{
    return this->movement_direction;
}

double Motor::get_offset()
{
    return this->offset;
}

MotorDictionary * Motor::get_motor_model()
{
    return this->motor_model;
}

bool Motor::get_torque_enabled()
{
    return this->torque_enabled;
}

bool Motor::get_safety_check()
{
    return this->safety_check;
}

double Motor::get_motor_state(uint8_t address)
{
    if (address == this->motor_model->Present_Position.address) 
    {
        return this->get_present_position();
    }
    else if (address == this->motor_model->Present_Velocity.address) 
    {
        return this->get_present_velocity();
    }
    else if (address == this->motor_model->Present_Current.address) 
    {
        return this->get_present_current();
    }
    else if (address == this->motor_model->Present_Temperature.address)
    {
        return this->get_present_temperature();
    }
    else if (address == this->motor_model->Moving.address)
    {
        return this->get_moving();
    }

    return 0.0;
}

double Motor::get_motor_state_adjusted(uint8_t address)
{
    if (address == this->motor_model->Present_Position.address) 
    {
        return this->get_present_position_adjusted();
    }
    else if (address == this->motor_model->Present_Velocity.address) 
    {
        return this->get_present_velocity_adjusted();
    }
    else if (address == this->motor_model->Present_Current.address) 
    {
        return this->get_present_current_adjusted();
    }
    else if (address == this->motor_model->Present_Temperature.address)
    {
        return this->get_present_temperature();
    }
    else if (address == this->motor_model->Moving.address)
    {
        return this->get_moving();
    }

    return 0.0;
}

double Motor::get_motor_goal()
{
    if (this->operating_mode == this->motor_model->OperatingMode::POSITION || this->operating_mode == this->motor_model->OperatingMode::EXTENDED_POSITION) 
    {
        return this->get_goal_position();
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::VELOCITY) 
    {
        return this->get_goal_velocity();
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::CURRENT) 
    {
        return this->get_goal_current();
    }

    return 0.0;
}

double Motor::get_motor_goal(uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        return this->get_goal_position();
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        return this->get_goal_velocity();
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        return this->get_goal_current();
    }

    return 0.0;
}

int Motor::get_motor_goal_int()
{
    if (this->operating_mode == this->motor_model->OperatingMode::POSITION || this->operating_mode == this->motor_model->OperatingMode::EXTENDED_POSITION) 
    {
        return this->positionDoubleToInt(this->get_goal_position());
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::VELOCITY) 
    {
        return this->velocityDoubleToInt(this->get_goal_velocity());
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::CURRENT) 
    {
        return this->currentDoubleToInt(this->get_goal_current());
    }

    return 0;
}

int Motor::get_motor_goal_int(uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        return this->positionDoubleToInt(this->get_goal_position());
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        return this->velocityDoubleToInt(this->get_goal_velocity());
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        return this->currentDoubleToInt(this->get_goal_current());
    }

    return 0;
}

double Motor::get_motor_goal_adjusted()
{
    if (this->operating_mode == this->motor_model->OperatingMode::POSITION || this->operating_mode == this->motor_model->OperatingMode::EXTENDED_POSITION) 
    {
        return this->get_goal_position_adjusted();
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::VELOCITY) 
    {
        return this->get_goal_velocity_adjusted();
    }
    else if (this->operating_mode == this->motor_model->OperatingMode::CURRENT) 
    {
        return this->get_goal_current_adjusted();
    }

    return 0.0;
}

double Motor::get_motor_goal_adjusted(uint8_t address)
{
    if (address == this->motor_model->Goal_Position.address) 
    {
        return this->get_goal_position_adjusted();
    }
    else if (address == this->motor_model->Goal_Velocity.address) 
    {
        return this->get_goal_velocity_adjusted();
    }
    else if (address == this->motor_model->Goal_Current.address) 
    {
        return this->get_goal_current_adjusted();
    }

    return 0.0;
}



double Motor::get_present_position()
{
    return this->present_position;
}

double Motor::get_present_position_adjusted()
{
    return this->present_position / this->movement_direction - this->offset;
}

double Motor::get_present_velocity()
{
    return this->present_velocity;
}

double Motor::get_present_velocity_adjusted()
{
    return this->present_velocity / this->movement_direction;
}

double Motor::get_present_current()
{
    return this->present_current;
}

double Motor::get_present_current_adjusted()
{
    return this->present_current / this->movement_direction;
}

double Motor::get_present_temperature()
{
    return this->present_temperature;
}

bool Motor::get_moving()
{
    return this->moving;
}

double Motor::get_goal_position()
{
    return this->goal_position;
}

double Motor::get_goal_position_adjusted()
{
    return this->goal_position / this->movement_direction - this->offset;
}

double Motor::get_reference_position()
{
    return this->reference_position;
}

double Motor::get_reference_position_adjusted()
{
    return this->reference_position / this->movement_direction - this->offset;
}

double Motor::get_goal_velocity()
{
    return this->goal_velocity;
}

double Motor::get_goal_velocity_adjusted()
{
    return this->goal_velocity / this->movement_direction;
}

double Motor::get_goal_current()
{
    return this->goal_current;
}

double Motor::get_goal_current_adjusted()
{
    return this->goal_current / this->movement_direction;
}

double Motor::get_reference_current()
{
    return this->reference_current;
}

double Motor::get_reference_current_adjusted()
{
    return this->reference_current / this->movement_direction;
}

bool Motor::goal_position_reached()
{
    if ((abs(this->get_present_position_adjusted() - this->get_goal_position_adjusted()) > 0.1) || (this->get_present_velocity_adjusted() != 0.0)) 
    {
        return false;
    }

    return true;
}

int Motor::get_operating_mode()
{
    return this->operating_mode;
}

double Motor::get_max_current()
{
    return this->max_current;
}

double Motor::get_min_current()
{
    return this->min_current;
}

double Motor::get_max_velocity()
{
    return this->max_velocity;
}

double Motor::get_min_velocity()
{
    return this->min_velocity;
}

double Motor::positionIntToDouble(int data)
{
    double value;

    value = data * (2*M_PI/4096);

    return value;
}

double Motor::velocityIntToDouble(int data)
{
    double value;

    value = data * 0.229 * 0.10472;

    return value;
}

double Motor::currentIntToDouble(int data)
{
    double value;

    if (data > 0x7fff)
    {
        value = (data - 65536) * 3.36;
    }
    else
    {
        value = data * 3.36;
    }

    return value;
}

int Motor::positionDoubleToInt(double value)
{
    int data;

    data = value / (2*M_PI/4096);

    return data;
}

int Motor::velocityDoubleToInt(double value)
{
    int data;

    data = value / 0.229 / 0.10472;

    return data;
}

int Motor::currentDoubleToInt(double value)
{
    int data;

    if (value < 0)
    {
        data = value / 3.36 + 65536;
    }
    else{
        data = value / 3.36;
    }

    return data;
}
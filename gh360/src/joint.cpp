#include "joint.hpp"


double Joint::positionIntToDouble(int data)
{
    double value;

    value = data * (2*M_PI/4096);

    return value;
}

double Joint::velocityIntToDouble(int data)
{
    //Transform motor velocity from int to rad/s
    double value;

    value = data * 0.229 * 0.10472;

    return value;
}

double Joint::currentIntToDouble(int data)
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

int Joint::positionDoubleToInt(double value)
{
    int data;

    data = value / (2*M_PI/4096);

    return data;
}

int Joint::velocityDoubleToInt(double value)
{
    //Transform motor velocity from rad/s to int
    int data;

    data = value / 0.229 / 0.10472;

    return data;
}

int Joint::currentDoubleToInt(double value)
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

double Joint::calc_set_motor_goal_pos(double goal_pos_adjusted, double offset, int movement_direction)
{
    double goal_pos_raw = (goal_pos_adjusted + offset) * movement_direction;
    return goal_pos_raw;
}

double Joint::calc_set_motor_goal_vel(double goal_vel, int movement_direction)
{
    double goal_vel_raw = goal_vel * movement_direction;
    return goal_vel_raw;
}

double Joint::calc_set_motor_goal_current(double goal_current, int movement_direction)
{
    double goal_current_raw = goal_current * movement_direction;
    return goal_current_raw;
}

double Joint::calc_get_motor_pos(double present_pos_raw, double offset, int movement_direction)
{
    double present_pos_adjusted = present_pos_raw / movement_direction - offset;
    return present_pos_adjusted;
}

double Joint::calc_get_motor_vel(double present_vel_raw, int movement_direction)
{
    double present_vel_adjusted = present_vel_raw / movement_direction;
    return present_vel_adjusted;
}

double Joint::calc_get_motor_current(double present_current_raw, int movement_direction)
{
    double present_current_adjusted = present_current_raw / movement_direction;
    return present_current_adjusted;
}

double Joint::get_min_joint_angle()
{
    return this->min_joint_angle;
}
        
double Joint::get_max_joint_angle()
{
    return this->max_joint_angle;
}

double Joint::get_motor_init_pos()
{
    return this->motor_init_pos;
}

int Joint::get_operating_mode()
{
    return this->operating_mode;
}

void Joint::set_min_joint_angle(double min_joint_angle)
{
    this->min_joint_angle = min_joint_angle;
}

void Joint::set_max_joint_angle(double max_joint_angle)
{
    this->max_joint_angle = max_joint_angle;
}

void Joint::set_motor_init_pos(double init_pos)
{
    this->motor_init_pos = init_pos;
}

void Joint::set_operating_mode(int operating_mode)
{
    this->operating_mode = operating_mode;
}
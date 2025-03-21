#include "gh360/joint_types/joint.hpp"


void Joint::set_joint_name(std::string name) 
{
    this->joint_name = name;
}

void Joint::set_joint_angle(double new_joint_angle)
{
    this->joint_angle = new_joint_angle;
}

std::string Joint::get_joint_name()
{
    return this->joint_name;
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

    for (int i=0; i<this->motor_cnt; i++) 
    {
        this->motors[i]->set_operating_mode(operating_mode);

        if (operating_mode == this->position_mode_id)
        {
            this->motors[i]->set_goal_position(this->motors[i]->get_present_position());
        }
        else if (operating_mode == this->velocity_mode_id)
        {
            this->motors[i]->set_goal_velocity(0.0);
        }
        else if (operating_mode == this->current_mode_id)
        {
            this->motors[i]->set_goal_current(0.0);
        }   
    }
}

Motor * Joint::get_motor(int motor_index)
{
    return this->motors[motor_index];
}

int Joint::get_motor_cnt()
{
    return this->motor_cnt;
}

int Joint::get_position_mode_id()
{
    return this->position_mode_id;
}

int Joint::get_current_mode_id()
{
    return this->current_mode_id;
}

int Joint::get_velocity_mode_id()
{
    return this->velocity_mode_id;
}

void Joint::set_joint_goal_angle(double goal_pos)
{
    this->joint_goal_angle = goal_pos;
}

void Joint::set_joint_goal_velocity(double goal_vel)
{
    this->joint_goal_velocity = goal_vel;
}

double Joint::get_joint_goal_angle()
{
    return this->joint_goal_angle;
}

double Joint::get_joint_goal_velocity()
{
    return this->joint_goal_velocity;
}
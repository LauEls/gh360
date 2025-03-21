#include "gh360/joint_types/motor_joint.hpp"

MotorJoint::MotorJoint()
{
    // this->motor_init_pos = true;

    this->motors.push_back(new Motor());
    this->motor_cnt = 1;
}

MotorJoint::~MotorJoint()
{

}

void MotorJoint::set_joint_angle(double new_joint_angle)
{
    
}

void MotorJoint::set_joint_velocity(double new_joint_velocity)
{

}

double MotorJoint::get_joint_angle()
{
    return this->motors[0]->get_present_position_adjusted();
}

double MotorJoint::get_joint_velocity()
{
    return this->motors[0]->get_present_velocity_adjusted();
}


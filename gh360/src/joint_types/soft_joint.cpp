#include "gh360/joint_types/soft_joint.hpp"


SoftJoint::SoftJoint() 
{
    // this->right_motor_init_pos = true;
    // this->left_motor_init_pos = true;
    this->joint_velocities = std::vector<double>(10,0.0);

    // this->motor_init_pos = true;

    this->motors.push_back(new Motor());
    this->motors.push_back(new Motor());
    this->motor_cnt = 2;

    this->position_mode_id = MotorDictionary::OperatingMode::EXTENDED_POSITION;
}

SoftJoint::~SoftJoint() 
{

}

void SoftJoint::set_joint_angle(double new_angle)
{
    this->joint_angle = new_angle;
}

void SoftJoint::set_joint_velocity(double new_velocity)
{
    this->joint_velocities.push_back(new_velocity);
    this->joint_velocities.erase(this->joint_velocities.begin());
    double vel = 0.0;
    for (uint i = 0; i < this->joint_velocities.size(); i++)
    {
        vel += this->joint_velocities[i];
    }
    this->joint_velocity = vel / this->joint_velocities.size();
}

void SoftJoint::set_reference_joint_angle(double data)
{
    this->reference_joint_angle = data;
}

void SoftJoint::set_radius_active_pulley(double radius)
{
    this->radius_active_pulley = radius;
}

void SoftJoint::set_radius_passive_pulley(double radius)
{
    this->radius_passive_pulley = radius;
}

double SoftJoint::get_joint_angle()
{
    return this->joint_angle;
}

double SoftJoint::get_joint_velocity()
{
    return this->joint_velocity;
}

double SoftJoint::get_reference_joint_angle()
{
    return this->reference_joint_angle;
}

double SoftJoint::get_radius_active_pulley()
{
    return this->radius_active_pulley;
}

double SoftJoint::get_radius_passive_pulley()
{
    return this->radius_passive_pulley;
}
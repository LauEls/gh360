#include "gh360/joint_types/encoder.hpp"

Encoder::Encoder(std::string joint_name, std::string port_name, int port_id, double offset, int inverter)
{
    this->joint_name = joint_name;
    this->port_name = port_name;
    this->port_id = port_id;
    this->offset = offset;
    this->inverter = inverter;

}

Encoder::~Encoder()
{

}

void Encoder::set_joint_angle(double angle)
{
    this->joint_angle = angle * this->inverter - this->offset;
}

double Encoder::calc_joint_velocity(std::chrono::time_point<std::chrono::system_clock> time)
{
    std::chrono::duration<double> elapsed_seconds;

    if (this->prev_time == std::chrono::time_point<std::chrono::system_clock>())
    {
        this->prev_time = time;
        this->joint_velocity = 0.0;
        this->prev_joint_angle = this->joint_angle;
    }
    else
    {
        elapsed_seconds = time - this->prev_time;
        this->prev_time = time;
    }

    float new_vel = (this->joint_angle - this->prev_joint_angle) / elapsed_seconds.count();
    this->joint_velocity += this->alpha*(new_vel-this->joint_velocity);
    this->prev_joint_angle = this->joint_angle;

    return this->joint_velocity;
}

std::string Encoder::get_joint_name()
{
    return this->joint_name;
}

std::string Encoder::get_port_name()
{
    return this->port_name;
}

int Encoder::get_port_id()
{
    return this->port_id;
}

double Encoder::get_offset()
{
    return this->offset;
}

int Encoder::get_inverter()
{
    return this->inverter;
}

double Encoder::get_joint_angle()
{
    return this->joint_angle;
}

double Encoder::get_prev_joint_angle()
{
    return this->prev_joint_angle;
}

double Encoder::get_joint_velocity()
{
    return this->joint_velocity;
}
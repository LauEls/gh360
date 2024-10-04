#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <math.h>

class Joint
{
    public:
        virtual ~Joint(){}

        std::string joint_name;
        double joint_angle;
        double joint_velocity;
        double min_joint_angle;
        double max_joint_angle;
        double motor_init_pos;
        int operating_mode;


        virtual std::string get_joint_name() = 0;
        virtual std::string get_joint_type() = 0;
        virtual double get_joint_angle() = 0;
        
        double get_min_joint_angle();
        double get_max_joint_angle();
        double get_motor_init_pos();
        int get_operating_mode();

        void set_min_joint_angle(double min_joint_angle);
        void set_max_joint_angle(double max_joint_angle);
        void set_motor_init_pos(double init_pos);
        void set_operating_mode(int operating_mode);

        double positionIntToDouble(int data);
        double velocityIntToDouble(int data);
        double currentIntToDouble(int data);

        int positionDoubleToInt(double value);
        int velocityDoubleToInt(double value);
        int currentDoubleToInt(double value);

        double calc_set_motor_goal_pos(double goal_pos_adjusted, double offset, int movement_direction);
        double calc_set_motor_goal_vel(double goal_vel, int movement_direction);
        double calc_set_motor_goal_current(double goal_current, int movement_direction);
        double calc_get_motor_pos(double present_pos_raw, double offset, int movement_direction);
        double calc_get_motor_vel(double present_vel_raw, int movement_direction);
        double calc_get_motor_current(double present_current_raw, int movement_direction);

    protected:
        Joint(){}

};

#endif // JOINT_HPP_
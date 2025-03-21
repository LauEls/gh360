#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <math.h>
#include <vector>

#include "gh360/joint_types/motor.hpp"

class Joint
{
    public:
        virtual ~Joint(){}

        /**
         * @brief Set the angle of the joint
         * @param new_joint_angle The new angle of the joint in radians
         */
        virtual void set_joint_angle(double new_joint_angle) = 0;

        /**
         * @brief Set the velocity of the joint
         * @param new_joint_velocity The new velocity of the joint in rad/s
         */
        virtual void set_joint_velocity(double new_joint_velocity) = 0;

        /**
         * @brief Get the angle of the joint
         * @return The angle of the joint in radians
         */
        virtual double get_joint_angle() = 0;

        /**
         * @brief Get the velocity of the joint
         * @return The velocity of the joint in rad/s
         */
        virtual double get_joint_velocity() = 0;

        /**
         * @brief Get the name of the joint
         * @return The name of the joint
         */
        std::string get_joint_name();

        /**
         * @brief Get the minimum angle of the joint
         * @return The minimum angle of the joint in radians
         */
        double get_min_joint_angle();

        /**
         * @brief Get the maximum angle of the joint
         * @return The maximum angle of the joint in radians
         */
        double get_max_joint_angle();

        /**
         * @brief Get the initial position of the joint motors
         * @return The initial position of the motors in radians
         */
        double get_motor_init_pos();

        /**
         * @brief Get the current operating mode of the joint
         * @return The operating mode of the joint as an integer (from the motor dictionary)
         */
        int get_operating_mode();
        
        /**
         * @brief Set the name of the joint
         * @param name The name of the joint
         */
        void set_joint_name(std::string name);

        /**
         * @brief Set the minimum angle of the joint
         * @param min_joint_angle The minimum angle of the joint in radians
         */
        void set_min_joint_angle(double min_joint_angle);

        /**
         * @brief Set the maximum angle of the joint
         * @param max_joint_angle The maximum angle of the joint in radians
         */
        void set_max_joint_angle(double max_joint_angle);

        /**
         * @brief Set the initial position of the joint motors
         * @param init_pos The initial position of the motors in radians
         */
        void set_motor_init_pos(double init_pos);

        /**
         * @brief Set the operating mode of the joint
         * @param operating_mode The operating mode of the joint as an integer (from the motor dictionary)
         */
        void set_operating_mode(int operating_mode);

        /**
         * @brief Returns the motor object at the given index
         * @param motor_index The index of the motor object in the motors vector
         * @return The motor object
         */
        Motor * get_motor(int motor_index);

        /**
         * @brief Returns the number of motors in the joint
         * @return The number of motors
         */
        int get_motor_cnt();

        /**
         * @return Returns the id of the position operating mode
         */
        int get_position_mode_id();

        /**
         * @return Returns the id of the velocity operating mode
         */
        int get_velocity_mode_id();

        /**
         * @return Returns the id of the current operating mode
         */
        int get_current_mode_id();

        /**
         * @brief Set the goal position of the joint
         * @param goal_pos The goal position in radians
         */
        void set_joint_goal_angle(double goal_pos);

        /**
         * @brief Set the goal velocity of the joint
         * @param goal_vel The goal velocity in rad/s
         */
        void set_joint_goal_velocity(double goal_vel);

        /**
         * @brief Get the goal position of the joint
         * @return The goal position in radians
         */
        double get_joint_goal_angle();

        /**
         * @brief Get the goal velocity of the joint
         * @return The goal velocity in rad/s
         */
        double get_joint_goal_velocity();

    protected:
        Joint(){}

        std::string joint_name;
        double joint_angle;
        double joint_velocity;
        double joint_goal_angle;
        double joint_goal_velocity;
        double min_joint_angle;
        double max_joint_angle;
        double motor_init_pos;

        int operating_mode;
        int position_mode_id = MotorDictionary::OperatingMode::POSITION;
        int velocity_mode_id = MotorDictionary::OperatingMode::VELOCITY;
        int current_mode_id = MotorDictionary::OperatingMode::CURRENT;


        int motor_cnt;
        std::vector<Motor*> motors;

    private:
        

};

#endif // JOINT_HPP_
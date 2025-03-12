#ifndef SOFT_JOINT_HPP_
#define SOFT_JOINT_HPP_

#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <math.h>

#include "gh360/joint_types/joint.hpp"
#include "gh360/motor_dictionaries/mx_106_dict.hpp"
#include "gh360/motor_dictionaries/mx_64_dict.hpp"

class SoftJoint: public Joint
{
    public:
        SoftJoint();
        virtual ~SoftJoint();

        enum MotorIndex
        {
            RIGHT = 0,
            LEFT = 1
        };
        
        /**
         * @brief Set the joint angle
         * @param new_angle The new joint angle in radians
         */
        void set_joint_angle(double new_angle);

        /**
         * @brief Add a new joint velocities to the joint_velocities vector and calculate the average joint velocity.
         * @param new_velocity The new joint velocity in rad/s
         */
        void set_joint_velocity(double new_velocity);

        /**
         * @brief Set the reference joint angle. This is used during the initialization of the motors to compare with the present joint angle.
         * @param data The reference joint angle in radians
         */
        void set_reference_joint_angle(double data);

        /**
         * @brief Set the radius of the active pulleys
         * @param radius The radius of the active pulleys in meters
         */
        void set_radius_active_pulley(double radius);

        /**
         * @brief Set the radius of the passive pulleys
         * @param radius The radius of the passive pulleys in meters
         */
        void set_radius_passive_pulley(double radius);

        /**
         * @brief Get the present joint angle
         * @return The joint angle in radians
         */
        double get_joint_angle();

        /**
         * @brief Get the present joint velocity
         * @return The joint velocity in rad/s
         */
        double get_joint_velocity();
        
        /**
         * @brief Get the reference joint angle
         * @return The reference joint angle in radians
         */
        double get_reference_joint_angle();

        /**
         * @brief Get the radius of the active pulleys
         * @return The radius of the active pulleys in meters
         */
        double get_radius_active_pulley();

        /**
         * @brief Get the radius of the passive pulleys
         * @return The radius of the passive pulleys in meters
         */
        double get_radius_passive_pulley();

    private:
        double reference_joint_angle;
        std::vector<double> joint_velocities;
        double radius_active_pulley;
        double radius_passive_pulley;


};

#endif // SOFT_JOINT_HPP_
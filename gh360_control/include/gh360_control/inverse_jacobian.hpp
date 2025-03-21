#ifndef INVERSE_JACOBIAN_HPP_
#define INVERSE_JACOBIAN_HPP_

#include <iostream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

// #include "sensor_msgs/msg/joint_state.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "gh360_interfaces/msg/joint_encoder_state.hpp"
// #include "gh360_interfaces/msg/space_mouse.hpp"

// using namespace std::chrono_literals;


class InverseJacobian
{
    public:
        InverseJacobian(std::string robot_description, std::string base_link_name="base_link", std::string eef_link_name="eef");
        virtual ~InverseJacobian();

        /**
         * @brief Calculate the joint velocities required to achieve the desired end-effector velocity
         * @param desired_eef_velocity The desired end-effector velocity in m/s and rad/s
         * @param current_joint_pos The current joint positions in radians
         * @return The joint velocities required to achieve the desired end-effector velocity
         */
        std::vector<double> calculate_goal_joint_velocities(std::vector<double> desired_eef_velocity, std::vector<double> current_joint_pos);

    private:
        KDL::Chain chain;
        KDL::ChainIkSolverVel_pinv* ik_solver;
        int num_joints;
};

#endif // INVERSE_JACOBIAN_HPP_
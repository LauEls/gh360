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

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "gh360_interfaces/msg/joint_encoder_state.hpp"
#include "gh360_interfaces/msg/space_mouse.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    class InverseJacobian : public rclcpp::Node
    {
        public:
            InverseJacobian();
            virtual ~InverseJacobian();

        private:
            void timer_callback();
            void spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg);
            void joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_publisher_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_position_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SpaceMouse>::SharedPtr spacemouse_subscriber_;

            geometry_msgs::msg::Twist desired_velocity;
            sensor_msgs::msg::JointState current_joint_pos;
            sensor_msgs::msg::JointState joint_vel_msg;
            KDL::Chain chain;
            KDL::ChainFkSolverPos_recursive* fk_solver;
            KDL::ChainIkSolverVel_pinv* ik_solver;
            std::string tcp_link_name;
            int num_joints;
    };
}

#endif // INVERSE_JACOBIAN_HPP_
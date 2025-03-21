#ifndef JOINT_VELOCITY_HPP_
#define JOINT_VELOCITY_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include "gh360/motor_dictionaries/motor_dict.hpp"
#include "gh360/joint_types/soft_joint.hpp"
#include "gh360/util/config_parser.hpp"
#include "gh360_interfaces/msg/set_motor_velocities.hpp"

using namespace std::chrono_literals;

class JointVelocity : public rclcpp::Node
{
    public:
        JointVelocity();
        virtual ~JointVelocity();

    private:
        // void timer_callback();
        void cmd_joint_vel_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        // rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr motor_velocity_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_joint_vel_subscriber_;

        // std::vector<double> desired_velocity;
        // std::vector<double> current_joint_pos;
        // sensor_msgs::msg::JointState joint_goal_vel_msg;

        std::vector<Joint*> joints;
        double max_motor_vel;
        //motor_goal_vel_msg
        //joint_states_subscriber
        gh360_interfaces::msg::SetMotorVelocities motor_goal_vel_msg;
        std::string control_mode; //(open loop or closed loop)

};

#endif // JOINT_VELOCITY_HPP_
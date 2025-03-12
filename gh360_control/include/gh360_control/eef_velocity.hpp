#ifndef EEF_VELOCITY_HPP_
#define EEF_VELOCITY_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "inverse_jacobian.hpp"

using namespace std::chrono_literals;

class EEFVelocity : public rclcpp::Node
{
    public:
        EEFVelocity();
        virtual ~EEFVelocity();

    private:
        void cmd_eef_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_velocity_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_position_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_eef_vel_subscriber_;

        std::vector<double> desired_velocity;
        std::vector<double> current_joint_pos;
        sensor_msgs::msg::JointState joint_goal_vel_msg;
        InverseJacobian* inverse_jacobian;
        bool joint_states_recieved = false;


};

#endif // EEF_VELOCITY_HPP_